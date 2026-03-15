// SPDX-License-Identifier: GPL-2.0+
/*
 * Zuma pinctrl driver.
 *
 * Bank data is ported from Linux mainline's zuma pinctrl tables and adapted
 * to U-Boot's simpler Exynos pinctrl model.
 */

#include <dm.h>
#include <dm/pinctrl.h>
#include "pinctrl-exynos.h"

#define ZUMA_PIN_BANK(pins, reg, id)		\
	{					\
		.type		= &zuma_bank_type,	\
		.offset		= reg,		\
		.nr_pins	= pins,		\
		.name		= id,		\
	}

/* CON, DAT, PUD, DRV */
static const struct samsung_pin_bank_type zuma_bank_type = {
	.fld_width = { 4, 1, 4, 4, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, },
};

static const struct pinctrl_ops zuma_pinctrl_ops = {
	.set_state = exynos_pinctrl_set_state,
};

static const struct samsung_pin_bank_data zuma_pin_alive[] = {
	ZUMA_PIN_BANK(4, 0x000, "gpa0"),
	ZUMA_PIN_BANK(6, 0x020, "gpa1"),
	ZUMA_PIN_BANK(4, 0x040, "gpa2"),
	ZUMA_PIN_BANK(4, 0x060, "gpa3"),
	ZUMA_PIN_BANK(2, 0x080, "gpa4"),
	ZUMA_PIN_BANK(6, 0x0a0, "gpa6"),
	ZUMA_PIN_BANK(8, 0x0c0, "gpa7"),
	ZUMA_PIN_BANK(4, 0x0e0, "gpa8"),
	ZUMA_PIN_BANK(7, 0x100, "gpa9"),
	ZUMA_PIN_BANK(5, 0x120, "gpa10"),
};

static const struct samsung_pin_bank_data zuma_pin_custom[] = {
	ZUMA_PIN_BANK(1, 0x000, "gpn0"),
	ZUMA_PIN_BANK(1, 0x020, "gpn1"),
	ZUMA_PIN_BANK(1, 0x040, "gpn2"),
	ZUMA_PIN_BANK(1, 0x060, "gpn3"),
	ZUMA_PIN_BANK(1, 0x080, "gpn4"),
	ZUMA_PIN_BANK(1, 0x0a0, "gpn5"),
	ZUMA_PIN_BANK(1, 0x0c0, "gpn6"),
	ZUMA_PIN_BANK(1, 0x0e0, "gpn7"),
	ZUMA_PIN_BANK(1, 0x100, "gpn8"),
	ZUMA_PIN_BANK(1, 0x120, "gpn9"),
};

static const struct samsung_pin_bank_data zuma_pin_far[] = {
	ZUMA_PIN_BANK(8, 0x000, "gpa5"),
};

static const struct samsung_pin_bank_data zuma_pin_gsacore0[] = {
	ZUMA_PIN_BANK(2, 0x000, "gps0"),
};

static const struct samsung_pin_bank_data zuma_pin_gsacore1[] = {
	ZUMA_PIN_BANK(4, 0x000, "gps1"),
};

static const struct samsung_pin_bank_data zuma_pin_gsacore2[] = {
	ZUMA_PIN_BANK(4, 0x000, "gps2"),
};

static const struct samsung_pin_bank_data zuma_pin_gsacore3[] = {
	ZUMA_PIN_BANK(3, 0x000, "gps3"),
};

static const struct samsung_pin_bank_data zuma_pin_gsactrl[] = {
	ZUMA_PIN_BANK(4, 0x000, "gps4"),
};

static const struct samsung_pin_bank_data zuma_pin_hsi1[] = {
	ZUMA_PIN_BANK(4, 0x000, "gph0"),
	ZUMA_PIN_BANK(8, 0x020, "gph1"),
	ZUMA_PIN_BANK(4, 0x040, "gph2"),
};

static const struct samsung_pin_bank_data zuma_pin_hsi2[] = {
	ZUMA_PIN_BANK(6, 0x000, "gph3"),
	ZUMA_PIN_BANK(7, 0x020, "gph4"),
};

static const struct samsung_pin_bank_data zuma_pin_hsi2ufs[] = {
	ZUMA_PIN_BANK(2, 0x000, "gph5"),
};

static const struct samsung_pin_bank_data zuma_pin_peric0[] = {
	ZUMA_PIN_BANK(5, 0x000, "gpp0"),
	ZUMA_PIN_BANK(4, 0x020, "gpp1"),
	ZUMA_PIN_BANK(4, 0x040, "gpp2"),
	ZUMA_PIN_BANK(2, 0x060, "gpp3"),
	ZUMA_PIN_BANK(4, 0x080, "gpp4"),
	ZUMA_PIN_BANK(2, 0x0a0, "gpp5"),
	ZUMA_PIN_BANK(4, 0x0c0, "gpp6"),
	ZUMA_PIN_BANK(2, 0x0e0, "gpp7"),
	ZUMA_PIN_BANK(4, 0x100, "gpp8"),
	ZUMA_PIN_BANK(2, 0x120, "gpp9"),
	ZUMA_PIN_BANK(4, 0x140, "gpp10"),
	ZUMA_PIN_BANK(2, 0x160, "gpp11"),
	ZUMA_PIN_BANK(4, 0x180, "gpp12"),
	ZUMA_PIN_BANK(2, 0x1a0, "gpp13"),
	ZUMA_PIN_BANK(2, 0x1c0, "gpp14"),
	ZUMA_PIN_BANK(2, 0x1e0, "gpp15"),
	ZUMA_PIN_BANK(2, 0x200, "gpp17"),
	ZUMA_PIN_BANK(4, 0x220, "gpp16"),
};

static const struct samsung_pin_bank_data zuma_pin_peric1[] = {
	ZUMA_PIN_BANK(8, 0x000, "gpp19"),
	ZUMA_PIN_BANK(4, 0x020, "gpp20"),
	ZUMA_PIN_BANK(8, 0x040, "gpp21"),
	ZUMA_PIN_BANK(4, 0x060, "gpp24"),
	ZUMA_PIN_BANK(4, 0x080, "gpp22"),
	ZUMA_PIN_BANK(4, 0x0a0, "gpp23"),
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_alive[] = {
	{ .pin_banks = zuma_pin_alive, .nr_banks = ARRAY_SIZE(zuma_pin_alive), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_custom[] = {
	{ .pin_banks = zuma_pin_custom, .nr_banks = ARRAY_SIZE(zuma_pin_custom), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_far[] = {
	{ .pin_banks = zuma_pin_far, .nr_banks = ARRAY_SIZE(zuma_pin_far), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_gsacore0[] = {
	{ .pin_banks = zuma_pin_gsacore0, .nr_banks = ARRAY_SIZE(zuma_pin_gsacore0), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_gsacore1[] = {
	{ .pin_banks = zuma_pin_gsacore1, .nr_banks = ARRAY_SIZE(zuma_pin_gsacore1), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_gsacore2[] = {
	{ .pin_banks = zuma_pin_gsacore2, .nr_banks = ARRAY_SIZE(zuma_pin_gsacore2), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_gsacore3[] = {
	{ .pin_banks = zuma_pin_gsacore3, .nr_banks = ARRAY_SIZE(zuma_pin_gsacore3), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_gsactrl[] = {
	{ .pin_banks = zuma_pin_gsactrl, .nr_banks = ARRAY_SIZE(zuma_pin_gsactrl), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_hsi1[] = {
	{ .pin_banks = zuma_pin_hsi1, .nr_banks = ARRAY_SIZE(zuma_pin_hsi1), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_hsi2[] = {
	{ .pin_banks = zuma_pin_hsi2, .nr_banks = ARRAY_SIZE(zuma_pin_hsi2), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_hsi2ufs[] = {
	{ .pin_banks = zuma_pin_hsi2ufs, .nr_banks = ARRAY_SIZE(zuma_pin_hsi2ufs), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_peric0[] = {
	{ .pin_banks = zuma_pin_peric0, .nr_banks = ARRAY_SIZE(zuma_pin_peric0), },
	{ }
};

static const struct samsung_pin_ctrl zuma_pin_ctrl_peric1[] = {
	{ .pin_banks = zuma_pin_peric1, .nr_banks = ARRAY_SIZE(zuma_pin_peric1), },
	{ }
};

static const struct samsung_pin_ctrl *zuma_get_pin_ctrl(fdt_addr_t base)
{
	switch (base) {
	case 0x10840000:
		return zuma_pin_ctrl_peric0;
	case 0x10c40000:
		return zuma_pin_ctrl_peric1;
	case 0x12040000:
		return zuma_pin_ctrl_hsi1;
	case 0x13040000:
		return zuma_pin_ctrl_hsi2;
	case 0x13060000:
		return zuma_pin_ctrl_hsi2ufs;
	case 0x15060000:
		return zuma_pin_ctrl_custom;
	case 0x154d0000:
		return zuma_pin_ctrl_alive;
	case 0x154e0000:
		return zuma_pin_ctrl_far;
	case 0x16140000:
		return zuma_pin_ctrl_gsactrl;
	case 0x16280000:
		return zuma_pin_ctrl_gsacore0;
	case 0x16290000:
		return zuma_pin_ctrl_gsacore1;
	case 0x162a0000:
		return zuma_pin_ctrl_gsacore2;
	case 0x162b0000:
		return zuma_pin_ctrl_gsacore3;
	default:
		return NULL;
	}
}

static int zuma_pinctrl_probe(struct udevice *dev)
{
	struct exynos_pinctrl_priv *priv = dev_get_priv(dev);
	const struct samsung_pin_ctrl *pin_ctrl;
	fdt_addr_t base;

	if (!priv)
		return -EINVAL;

	base = dev_read_addr(dev);
	if (base == FDT_ADDR_T_NONE)
		return -EINVAL;

	pin_ctrl = zuma_get_pin_ctrl(base);
	if (!pin_ctrl)
		return -ENODEV;

	priv->base = base;
	priv->pin_ctrl = pin_ctrl;

	return 0;
}

static const struct udevice_id zuma_pinctrl_ids[] = {
	{ .compatible = "google,zuma-pinctrl" },
	{ }
};

U_BOOT_DRIVER(pinctrl_zuma) = {
	.name		= "pinctrl_zuma",
	.id		= UCLASS_PINCTRL,
	.of_match	= zuma_pinctrl_ids,
	.priv_auto	= sizeof(struct exynos_pinctrl_priv),
	.ops		= &zuma_pinctrl_ops,
	.probe		= zuma_pinctrl_probe,
	.bind		= exynos_pinctrl_bind,
};
