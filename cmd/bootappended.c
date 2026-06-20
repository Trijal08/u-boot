// SPDX-License-Identifier: GPL-2.0+
/*
 * Boot an image appended directly after the U-Boot binary in memory.
 *
 * The previous-stage bootloader loads u-boot.bin to a fixed address
 * (CONFIG_APPENDED_PAYLOAD_BASE) before U-Boot relocates itself to the
 * top of DRAM. Anything concatenated to u-boot.bin therefore stays put
 * at the original load location, right after the image, at:
 *
 *     CONFIG_APPENDED_PAYLOAD_BASE + sizeof(u-boot.bin)
 *
 * The flashed u-boot.bin is u-boot-dtb.bin == u-boot-nodtb.bin + dtb:
 *
 *   - the nodtb image spans [__image_copy_start, _end); its size is the
 *     binary size and includes the .rela.dyn relocation section for PIE
 *     builds, so the end marker is _end, NOT __image_copy_end.
 *   - with CONFIG_OF_SEPARATE the device tree is concatenated (no
 *     padding) right after that, so its totalsize is added too. The
 *     FDT magic check makes this a no-op when no DTB is appended (e.g.
 *     CONFIG_OF_EMBED), where the DTB is already inside the nodtb image.
 *
 * Both sizes are relocation-invariant, so the math holds regardless of
 * where U-Boot ends up running from.
 *
 * Build a combined image with e.g.:
 *
 *     cat u-boot.bin payload.img > u-boot-combined.bin
 *
 * The payload may be an EFI application, an arm64 Linux 'Image', a FIT
 * or a legacy uImage, optionally wrapped in any compression U-Boot
 * supports (gzip/bzip2/lzma/lzo/lz4/zstd). Both the outer compression
 * and the image format are auto-detected.
 */

#include <command.h>
#include <env.h>
#include <image.h>
#include <lmb.h>
#include <mapmem.h>
#include <vsprintf.h>
#include <asm/global_data.h>
#include <asm-generic/sections.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

/* arm64 Linux 'Image' magic, found at byte offset 56 (little-endian) */
#define LINUX_ARM64_IMAGE_MAGIC	0x644d5241

static inline u32 be32_at(const u8 *p)
{
	return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

/*
 * Sanity-check an FDT header so a stray 0xd00dfeed inside a kernel or
 * compressed blob is not mistaken for the appended device tree.
 */
static bool fdt_looks_valid(const u8 *q)
{
	u32 tsize = be32_at(q + 4);
	u32 ver = be32_at(q + 20);
	u32 lver = be32_at(q + 24);

	if (be32_at(q) != FDT_MAGIC)
		return false;
	if (tsize < 0x100 || tsize > (64 << 20))
		return false;
	if (ver < 16 || ver > 17 || lver < 16 || lver > 17)
		return false;

	return true;
}

/*
 * Scan [start, start + len) on 4-byte boundaries for a valid FDT header.
 * The appended device tree is concatenated (unpadded) after the kernel
 * file, whose true file size cannot be derived from the arm64 Image
 * header (its image_size is the effective in-memory size, not the file
 * length), so we locate the tree by its signature instead.
 */
static ulong scan_for_fdt(ulong start, ulong len)
{
	void *p = map_sysmem(start, len);
	const u8 *base = p;
	ulong off, found = 0;

	for (off = 0; off + 0x28 <= len; off += 4) {
		if (fdt_looks_valid(base + off)) {
			found = start + off;
			break;
		}
	}
	unmap_sysmem(p);

	return found;
}

static ulong hex8(const u8 *s)
{
	ulong v = 0;
	int i;

	for (i = 0; i < 8; i++) {
		u8 c = s[i];

		v <<= 4;
		if (c >= '0' && c <= '9')
			v |= c - '0';
		else if (c >= 'a' && c <= 'f')
			v |= c - 'a' + 10;
		else if (c >= 'A' && c <= 'F')
			v |= c - 'A' + 10;
	}

	return v;
}

/*
 * Walk a 'newc' cpio archive (uncompressed initramfs) to its TRAILER!!!
 * entry and return the total byte size, so a raw appended ramdisk can be
 * sized automatically. Returns 0 if it is not a newc cpio.
 */
static ulong cpio_newc_size(ulong addr, ulong max)
{
	void *p = map_sysmem(addr, max);
	const u8 *b = p;
	ulong off = 0, total = 0;

	while (off + 110 <= max) {
		ulong fsize, nsize, namelen;

		if (memcmp(b + off, "070701", 6) && memcmp(b + off, "070702", 6))
			break;
		fsize = hex8(b + off + 54);
		nsize = hex8(b + off + 94);
		namelen = nsize;
		if (off + 110 + namelen > max)
			break;
		if (nsize == 11 && !memcmp(b + off + 110, "TRAILER!!!", 10)) {
			total = ALIGN(off + 110 + nsize, 4);
			break;
		}
		off += ALIGN(110 + nsize, 4);
		off += ALIGN(fsize, 4);
	}
	unmap_sysmem(p);

	return total;
}

/*
 * Pick a free, aligned region of tracked DRAM. The appended image is
 * loaded into memory the bootloader mapped but that U-Boot does not list
 * as usable RAM, so a kernel placed there cannot be reserved by booti.
 * Ask LMB for a free spot, then release it so booti can re-reserve it.
 */
static ulong lmb_pick(ulong size, ulong align)
{
	phys_addr_t addr = 0;

	if (lmb_alloc_mem(LMB_MEM_ALLOC_ANY, align, &addr, size, LMB_NONE))
		return 0;
	lmb_free(addr, size, LMB_NONE);

	return (ulong)addr;
}

static int do_bootappended(struct cmd_tbl *cmdtp, int flag, int argc,
			   char *const argv[])
{
	ulong base, nodtb_size, dtb_addr, dtb_size, payload, boot_addr;
	u8 hdr[64];
	void *p;
	int comp;
	char cmd[160];
	const char *fdt;
	u32 be0, le_img;

	base = (argc > 1) ? hextoul(argv[1], NULL) : CONFIG_APPENDED_PAYLOAD_BASE;

	/* Size of the relocatable image (u-boot-nodtb.bin): ends at _end,
	 * which for PIE builds is past .rela.dyn, not at __image_copy_end.
	 */
	nodtb_size = (ulong)_end - (ulong)__image_copy_start;
	dtb_addr = base + nodtb_size;

	/* A device tree concatenated by CONFIG_OF_SEPARATE sits here. Read
	 * its totalsize directly from the appended copy (intact at the load
	 * address); skip it when no FDT is present.
	 */
	p = map_sysmem(dtb_addr, 8);
	be0 = ((u32)((u8 *)p)[0] << 24) | (((u8 *)p)[1] << 16) |
	      (((u8 *)p)[2] << 8) | ((u8 *)p)[3];
	dtb_size = (be0 == FDT_MAGIC) ?
		(((u32)((u8 *)p)[4] << 24) | (((u8 *)p)[5] << 16) |
		 (((u8 *)p)[6] << 8) | ((u8 *)p)[7]) : 0;
	unmap_sysmem(p);

	payload = dtb_addr + dtb_size;

	printf("U-Boot load base : 0x%lx\n", base);
	printf("nodtb image size : 0x%lx (%lu bytes)\n", nodtb_size, nodtb_size);
	printf("appended dtb size: 0x%lx (%lu bytes)\n", dtb_size, dtb_size);
	printf("Appended payload : 0x%lx\n", payload);

	/* Auto-detect outer compression from the first bytes of the payload */
	p = map_sysmem(payload, sizeof(hdr));
	comp = image_decomp_type(p, sizeof(hdr));
	unmap_sysmem(p);

	if (comp > IH_COMP_NONE) {
		/*
		 * Decompress into a free, aligned region of tracked DRAM so
		 * booti can later reserve the kernel there. An explicit
		 * appended_comp_addr overrides the automatic LMB placement.
		 */
		ulong unc = env_get_ulong("appended_comp_size", 16, 128 << 20);
		ulong dst = env_get_ulong("appended_comp_addr", 16, 0);
		ulong clen = gd->ram_top - payload;
		void *dst_p, *src_p;
		ulong end;
		int ret;

		if (!dst)
			dst = lmb_pick(unc, SZ_2M);
		if (!dst) {
			printf("No free DRAM region (%lu MiB) for decompression;\n"
			       "  set 'appended_comp_size'/'appended_comp_addr'\n",
			       unc >> 20);
			return CMD_RET_FAILURE;
		}

		printf("Compression      : %s, decompressing to 0x%lx\n",
		       genimg_get_comp_short_name(comp), dst);

		dst_p = map_sysmem(dst, unc);
		src_p = map_sysmem(payload, clen);
		ret = image_decomp(comp, dst, payload, IH_TYPE_KERNEL,
				   dst_p, src_p, clen, unc, &end);
		unmap_sysmem(src_p);
		unmap_sysmem(dst_p);
		if (ret) {
			printf("Decompression failed (err %d)\n", ret);
			return CMD_RET_FAILURE;
		}
		printf("Decompressed     : %lu bytes\n", end - dst);
		boot_addr = dst;
	} else {
		boot_addr = payload;
	}

	/* Sniff the (decompressed) header to pick a boot method */
	p = map_sysmem(boot_addr, sizeof(hdr));
	memcpy(hdr, p, sizeof(hdr));
	unmap_sysmem(p);

	be0 = ((u32)hdr[0] << 24) | (hdr[1] << 16) | (hdr[2] << 8) | hdr[3];
	le_img = hdr[56] | (hdr[57] << 8) | (hdr[58] << 16) | ((u32)hdr[59] << 24);

	fdt = env_get("appended_fdt");
	if (!fdt)
		fdt = "${fdtcontroladdr}";

	if (be0 == FDT_MAGIC || be0 == IH_MAGIC) {
		printf("Detected         : %s -> bootm\n",
		       be0 == FDT_MAGIC ? "FIT image" : "legacy uImage");
		snprintf(cmd, sizeof(cmd), "bootm 0x%lx", boot_addr);
	} else if (le_img == LINUX_ARM64_IMAGE_MAGIC) {
		/*
		 * Checked before the EFI 'MZ' test: an arm64 Image with the
		 * EFI stub is also a valid PE binary, but we want it booted as
		 * Linux so the appended dtb/ramdisk handling below applies.
		 */
		ulong scan_max = env_get_ulong("appended_scan_max", 16, 64 << 20);
		ulong scan_end = payload + scan_max;
		ulong dtb_blob = 0, rd = 0;
		char fdtbuf[24], rdbuf[40];
		const char *fdtarg = fdt;
		const char *rdarg = "-";

		printf("Detected         : arm64 Linux Image -> booti\n");

		if (scan_end > gd->ram_top)
			scan_end = gd->ram_top;

		/*
		 * An uncompressed Image still sits in the appended region, which
		 * is outside tracked DRAM, so booti could not reserve it. Move it
		 * into a free LMB region. (Compressed kernels already landed in
		 * one via lmb_pick() above.) The dtb/ramdisk stay where they are;
		 * U-Boot relocates those itself.
		 */
		if (boot_addr == payload) {
			ulong isz = hdr[16] | ((ulong)hdr[17] << 8) |
				    ((ulong)hdr[18] << 16) | ((ulong)hdr[19] << 24);
			ulong d;

			if (!isz)
				isz = 32 << 20;
			d = lmb_pick(isz, SZ_2M);
			if (d) {
				void *dp = map_sysmem(d, isz);
				void *sp = map_sysmem(payload, isz);

				memmove(dp, sp, isz);
				unmap_sysmem(sp);
				unmap_sysmem(dp);
				boot_addr = d;
				printf("Relocated kernel : 0x%lx (0x%lx bytes)\n",
				       d, isz);
			}
		}

		/*
		 * Locate the device tree: an explicit appended_fdt wins; else
		 * find one appended after the kernel by its signature. Anything
		 * appended after that tree is treated as a ramdisk.
		 */
		if (env_get("appended_fdt")) {
			fdtarg = fdt;
		} else {
			dtb_blob = scan_for_fdt(payload, scan_end - payload);
			if (dtb_blob) {
				snprintf(fdtbuf, sizeof(fdtbuf), "0x%lx", dtb_blob);
				fdtarg = fdtbuf;
				printf("Appended dtb     : 0x%lx\n", dtb_blob);
			}
		}

		if (env_get("appended_ramdisk")) {
			ulong rsz = env_get_ulong("appended_ramdisk_size", 16, 0);

			if (rsz)
				snprintf(rdbuf, sizeof(rdbuf), "%s:0x%lx",
					 env_get("appended_ramdisk"), rsz);
			else
				snprintf(rdbuf, sizeof(rdbuf), "%s",
					 env_get("appended_ramdisk"));
			rdarg = rdbuf;
		} else if (dtb_blob) {
			u8 rh[16];
			u8 dh[8];

			p = map_sysmem(dtb_blob, sizeof(dh));
			memcpy(dh, p, sizeof(dh));
			unmap_sysmem(p);
			rd = dtb_blob + be32_at(dh + 4);

			p = map_sysmem(rd, sizeof(rh));
			memcpy(rh, p, sizeof(rh));
			unmap_sysmem(p);

			if (be32_at(rh) == IH_MAGIC || be32_at(rh) == FDT_MAGIC) {
				/* uImage / FIT ramdisk: size is self-describing */
				snprintf(rdbuf, sizeof(rdbuf), "0x%lx", rd);
				rdarg = rdbuf;
				printf("Appended ramdisk : 0x%lx (%s)\n", rd,
				       be32_at(rh) == IH_MAGIC ? "uImage" : "FIT");
			} else {
				ulong rsz = env_get_ulong("appended_ramdisk_size",
							  16, 0);

				if (!rsz)
					rsz = cpio_newc_size(rd, scan_end - rd);
				if (rsz) {
					snprintf(rdbuf, sizeof(rdbuf),
						 "0x%lx:0x%lx", rd, rsz);
					rdarg = rdbuf;
					printf("Appended ramdisk : 0x%lx size 0x%lx\n",
					       rd, rsz);
				} else if (image_decomp_type(rh, sizeof(rh)) > 0) {
					printf("Appended ramdisk : 0x%lx (compressed, size unknown)\n"
					       "  set 'appended_ramdisk_size'; booting without it\n",
					       rd);
				}
			}
		}

		snprintf(cmd, sizeof(cmd), "booti 0x%lx %s %s",
			 boot_addr, rdarg, fdtarg);
	} else if (hdr[0] == 'M' && hdr[1] == 'Z') {
		printf("Detected         : EFI application -> bootefi\n");
		snprintf(cmd, sizeof(cmd), "bootefi 0x%lx %s", boot_addr, fdt);
	} else {
		printf("Detected         : unknown, trying Linux Image then EFI\n");
		snprintf(cmd, sizeof(cmd),
			 "booti 0x%lx - %s; bootefi 0x%lx %s",
			 boot_addr, fdt, boot_addr, fdt);
	}

	printf("Running          : %s\n", cmd);

	return run_command(cmd, flag) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_LONGHELP(bootappended,
	"[base]\n"
	"    - boot an image appended to the U-Boot binary in memory\n"
	"\tThe payload is located at 'base' + the U-Boot image size, where\n"
	"\t'base' defaults to CONFIG_APPENDED_PAYLOAD_BASE (the address the\n"
	"\tprevious bootloader loaded u-boot.bin to, before relocation).\n"
	"\tEFI / arm64 Image / FIT / legacy uImage formats and gzip, bzip2,\n"
	"\tlzma, lzo, lz4 and zstd compression are auto-detected.\n"
	"\tFor an arm64 Image, a device tree appended after the kernel and a\n"
	"\tramdisk appended after that tree are located automatically. uImage,\n"
	"\tFIT and uncompressed cpio ramdisks are auto-sized; for a compressed\n"
	"\traw ramdisk set 'appended_ramdisk_size'.\n"
	"\tEnv overrides: appended_fdt, appended_ramdisk[, appended_ramdisk_size],\n"
	"\tappended_scan_max (dtb/ramdisk search window, default 64M),\n"
	"\tappended_comp_addr and appended_comp_size (decompression scratch).\n"
	);

U_BOOT_CMD(
	bootappended, 2, 1, do_bootappended,
	"boot an image appended to the U-Boot binary", bootappended_help_text
);
