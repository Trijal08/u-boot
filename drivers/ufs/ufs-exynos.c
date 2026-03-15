// SPDX-License-Identifier: GPL-2.0-only

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <generic-phy.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>

#include "ufs.h"
#include "../phy/phy-samsung-ufs.h"

#define COMP_CLK_PERIOD			0x44
#define HCI_TXPRDT_ENTRY_SIZE		0x00
#define HCI_RXPRDT_ENTRY_SIZE		0x04
#define HCI_1US_TO_CNT_VAL		0x0C
#define CNT_VAL_1US_MASK		0x3FF
#define HCI_UTRL_NEXUS_TYPE		0x40
#define HCI_UTMRL_NEXUS_TYPE		0x44
#define HCI_SW_RST			0x50
#define UFS_LINK_SW_RST			BIT(0)
#define UFS_UNIPRO_SW_RST		BIT(1)
#define UFS_SW_RST_MASK			(UFS_LINK_SW_RST | UFS_UNIPRO_SW_RST)
#define HCI_DATA_REORDER		0x60
#define HCI_AXIDMA_RWDATA_BURST_LEN	0x6C
#define WLU_EN				BIT(31)
#define WLU_BURST_LEN(x)		(((x) << 27) | ((x) & 0xf))
#define HCI_GPIO_OUT			0x70
#define HCI_ERR_EN_DL_LAYER		0x7C
#define HCI_ERR_EN_N_LAYER		0x80
#define HCI_ERR_EN_T_LAYER		0x84
#define HCI_ERR_EN_DME_LAYER		0x88
#define HCI_V2P1_CTRL			0x8C
#define IA_TICK_SEL			BIT(16)
#define HCI_CLKSTOP_CTRL		0xB0
#define REFCLKOUT_STOP			BIT(4)
#define MPHY_APBCLK_STOP		BIT(3)
#define REFCLK_STOP			BIT(2)
#define UNIPRO_MCLK_STOP		BIT(1)
#define UNIPRO_PCLK_STOP		BIT(0)
#define CLK_STOP_MASK			(REFCLKOUT_STOP | REFCLK_STOP | \
					 UNIPRO_MCLK_STOP | MPHY_APBCLK_STOP | \
					 UNIPRO_PCLK_STOP)
#define HCI_MISC			0xB4
#define REFCLK_CTRL_EN			BIT(7)
#define UNIPRO_PCLK_CTRL_EN		BIT(6)
#define UNIPRO_MCLK_CTRL_EN		BIT(5)
#define HCI_CORECLK_CTRL_EN		BIT(4)
#define CLK_CTRL_EN_MASK		(REFCLK_CTRL_EN | \
					 UNIPRO_PCLK_CTRL_EN | \
					 UNIPRO_MCLK_CTRL_EN)
#define HCI_IOP_ACG_DISABLE		0x100
#define HCI_IOP_ACG_DISABLE_EN		BIT(0)
#define DFES_ERR_EN			BIT(31)
#define DFES_DEF_L2_ERRS	(UIC_DATA_LINK_LAYER_ERROR_RX_BUF_OF | \
				 UIC_DATA_LINK_LAYER_ERROR_PA_INIT)
#define DFES_DEF_L3_ERRS	(UIC_NETWORK_UNSUPPORTED_HEADER_TYPE | \
				 UIC_NETWORK_BAD_DEVICEID_ENC | \
				 UIC_NETWORK_LHDR_TRAP_PACKET_DROPPING)
#define DFES_DEF_L4_ERRS	(UIC_TRANSPORT_UNSUPPORTED_HEADER_TYPE | \
				 UIC_TRANSPORT_UNKNOWN_CPORTID | \
				 UIC_TRANSPORT_NO_CONNECTION_RX | \
				 UIC_TRANSPORT_BAD_TC)
#define UFS_SHAREABILITY_OFFSET		0x710
#define UFS_GS101_WR_SHARABLE		BIT(1)
#define UFS_GS101_RD_SHARABLE		BIT(0)
#define UFS_GS101_SHARABLE		(UFS_GS101_WR_SHARABLE | UFS_GS101_RD_SHARABLE)
#define DATA_UNIT_SIZE			4096
#define IATOVAL_NSEC			20000
#define CNTR_DIV_VAL			40
#define PA_DBG_CLK_PERIOD		0x9514
#define PA_GS101_DBG_OPTION_SUITE1	0x956a
#define PA_GS101_DBG_OPTION_SUITE2	0x956d

#define VND_TX_CLK_PRD			0xAA
#define VND_TX_CLK_PRD_EN		0xA9
#define VND_TX_LINERESET_PVALUE0	0xAD
#define VND_TX_LINERESET_PVALUE1	0xAC
#define VND_TX_LINERESET_PVALUE2	0xAB
#define VND_RX_CLK_PRD			0x12
#define VND_RX_CLK_PRD_EN		0x11
#define VND_RX_LINERESET_VALUE0		0x1D
#define VND_RX_LINERESET_VALUE1		0x1C
#define VND_RX_LINERESET_VALUE2		0x1B
#define TX_LINE_RESET_TIME		3200
#define RX_LINE_RESET_TIME		1000

#define E2EFC_OFF			(0 << 0)
#define CSD_N_OFF			(1 << 1)
#define CSV_N_OFF			(1 << 2)
#define CPORT_DEF_FLAGS			(CSV_N_OFF | CSD_N_OFF | E2EFC_OFF)
#define CPORT_IDLE			0
#define CPORT_CONNECTED			1

struct exynos_ufs;

struct exynos_ufs_uic_attr {
	u32 tx_trailingclks;
	u32 pa_dbg_clk_period_off;
	u32 pa_dbg_opt_suite1_val;
	u32 pa_dbg_opt_suite1_off;
	u32 pa_dbg_opt_suite2_val;
	u32 pa_dbg_opt_suite2_off;
};

struct exynos_ufs_drv_data {
	u32 quirks;
	u32 iocc_mask;
	const struct exynos_ufs_uic_attr *uic_attr;
	int (*drv_init)(struct exynos_ufs *ufs);
	int (*pre_link)(struct exynos_ufs *ufs);
	int (*post_link)(struct exynos_ufs *ufs);
};

struct exynos_ufs {
	struct ufs_hba *hba;
	void __iomem *reg_hci;
	void __iomem *reg_unipro;
	void __iomem *reg_ufsp;
	struct clk_bulk clks;
	struct clk core_clk;
	struct clk unipro_clk;
	struct phy phy;
	struct regmap *sysreg;
	unsigned long mclk_rate;
	unsigned long pclk_rate;
	u32 iocc_offset;
	const struct exynos_ufs_drv_data *drv_data;
};

static int zuma_ufs_drv_init(struct exynos_ufs *ufs);
static int zuma_ufs_pre_link(struct exynos_ufs *ufs);
static int zuma_ufs_post_link(struct exynos_ufs *ufs);

static const struct exynos_ufs_uic_attr gs101_uic_attr = {
	.tx_trailingclks	= 0xff,
	.pa_dbg_opt_suite1_val	= 0x90913C1C,
	.pa_dbg_opt_suite1_off	= PA_GS101_DBG_OPTION_SUITE1,
	.pa_dbg_opt_suite2_val	= 0xE01C115F,
	.pa_dbg_opt_suite2_off	= PA_GS101_DBG_OPTION_SUITE2,
};

static const struct exynos_ufs_drv_data zuma_ufs_drv_data = {
	.quirks		= UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
			  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.iocc_mask	= UFS_GS101_SHARABLE,
	.uic_attr	= &gs101_uic_attr,
	.drv_init	= zuma_ufs_drv_init,
	.pre_link	= zuma_ufs_pre_link,
	.post_link	= zuma_ufs_post_link,
};

static inline void hci_writel(struct exynos_ufs *ufs, u32 val, u32 reg)
{
	writel(val, ufs->reg_hci + reg);
}

static inline u32 hci_readl(struct exynos_ufs *ufs, u32 reg)
{
	return readl(ufs->reg_hci + reg);
}

static inline void unipro_writel(struct exynos_ufs *ufs, u32 val, u32 reg)
{
	writel(val, ufs->reg_unipro + reg);
}

static void exynos_ufs_auto_ctrl_hcc(struct exynos_ufs *ufs, bool en)
{
	u32 misc = hci_readl(ufs, HCI_MISC);

	if (en)
		hci_writel(ufs, misc | HCI_CORECLK_CTRL_EN, HCI_MISC);
	else
		hci_writel(ufs, misc & ~HCI_CORECLK_CTRL_EN, HCI_MISC);
}

static void exynos_ufs_disable_auto_ctrl_hcc_save(struct exynos_ufs *ufs, u32 *val)
{
	*val = hci_readl(ufs, HCI_MISC);
	exynos_ufs_auto_ctrl_hcc(ufs, false);
}

static void exynos_ufs_auto_ctrl_hcc_restore(struct exynos_ufs *ufs, u32 *val)
{
	hci_writel(ufs, *val, HCI_MISC);
}

static void exynos_ufs_ctrl_clkstop(struct exynos_ufs *ufs, bool en)
{
	u32 ctrl = hci_readl(ufs, HCI_CLKSTOP_CTRL);
	u32 misc = hci_readl(ufs, HCI_MISC);

	if (en) {
		hci_writel(ufs, misc | CLK_CTRL_EN_MASK, HCI_MISC);
		hci_writel(ufs, ctrl | CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
	} else {
		hci_writel(ufs, ctrl & ~CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
		hci_writel(ufs, misc & ~CLK_CTRL_EN_MASK, HCI_MISC);
	}
}

static inline u32 get_mclk_period_unipro_18(struct exynos_ufs *ufs)
{
	return 16 * 1000 * 1000000UL / ufs->mclk_rate;
}

static long exynos_ufs_calc_time_cntr(struct exynos_ufs *ufs, long period_ns)
{
	unsigned long clk_period = DIV_ROUND_UP(1000000000UL, ufs->pclk_rate);

	return DIV_ROUND_CLOSEST(period_ns, clk_period);
}

static int exynos_ufs_shareability(struct exynos_ufs *ufs)
{
	if (IS_ERR_OR_NULL(ufs->sysreg))
		return 0;

	return regmap_update_bits(ufs->sysreg, ufs->iocc_offset,
				  ufs->drv_data->iocc_mask,
				  ufs->drv_data->iocc_mask);
}

static void exynos_ufs_config_intr(struct exynos_ufs *ufs, u32 errs, u32 reg)
{
	hci_writel(ufs, DFES_ERR_EN | errs, reg);
}

static void exynos_ufs_establish_connt(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_IDLE);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), 0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), true);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERCPORTID), 0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CPORTFLAGS), CPORT_DEF_FLAGS);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_TRAFFICCLASS), 0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);
}

static void exynos_ufs_config_unipro(struct exynos_ufs *ufs)
{
	const struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_hba *hba = ufs->hba;

	if (!attr)
		return;

	if (attr->pa_dbg_clk_period_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_clk_period_off),
			       DIV_ROUND_UP(1000000000UL, ufs->mclk_rate));

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXTRAILINGCLOCKS),
		       attr->tx_trailingclks);

	if (attr->pa_dbg_opt_suite1_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
			       attr->pa_dbg_opt_suite1_val);

	if (attr->pa_dbg_opt_suite2_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite2_off),
			       attr->pa_dbg_opt_suite2_val);
}

static int exynos_ufs_init(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);
	int ret;

	ufs->hba = hba;
	ufs->drv_data = (const struct exynos_ufs_drv_data *)dev_get_driver_data(hba->dev);
	ufs->reg_hci = dev_read_addr_name_ptr(hba->dev, "vs_hci");
	ufs->reg_unipro = dev_read_addr_name_ptr(hba->dev, "unipro");
	ufs->reg_ufsp = dev_read_addr_name_ptr(hba->dev, "ufsp");
	if (!ufs->reg_hci || !ufs->reg_unipro || !ufs->reg_ufsp)
		return -EINVAL;

	ret = clk_get_bulk(hba->dev, &ufs->clks);
	if (ret)
		return ret;

	ret = clk_enable_bulk(&ufs->clks);
	if (ret)
		return ret;

	ret = clk_get_by_name(hba->dev, "core_clk", &ufs->core_clk);
	if (ret)
		return ret;

	ret = clk_get_by_name(hba->dev, "sclk_unipro_main", &ufs->unipro_clk);
	if (ret)
		return ret;

	ufs->pclk_rate = clk_get_rate(&ufs->core_clk);
	ufs->mclk_rate = clk_get_rate(&ufs->unipro_clk);
	if (!ufs->pclk_rate || !ufs->mclk_rate)
		return -EINVAL;
	dev_info(hba->dev, "core_clk=%lu unipro_clk=%lu\n",
		 ufs->pclk_rate, ufs->mclk_rate);

	ret = generic_phy_get_by_name(hba->dev, "ufs-phy", &ufs->phy);
	if (ret)
		return ret;

	ufs->sysreg = syscon_regmap_lookup_by_phandle(hba->dev, "samsung,sysreg");
	if (IS_ERR(ufs->sysreg))
		ufs->sysreg = NULL;
	ufs->iocc_offset = dev_read_u32_default(hba->dev, "google,iocc-offset",
						UFS_SHAREABILITY_OFFSET);

	hba->quirks = ufs->drv_data->quirks;
	ret = ufs->drv_data->drv_init ? ufs->drv_data->drv_init(ufs) :
	      exynos_ufs_shareability(ufs);
	if (ret)
		return ret;

	return 0;
}

static int zuma_ufs_drv_init(struct exynos_ufs *ufs)
{
	hci_writel(ufs, hci_readl(ufs, HCI_IOP_ACG_DISABLE) &
		   ~HCI_IOP_ACG_DISABLE_EN, HCI_IOP_ACG_DISABLE);

	return exynos_ufs_shareability(ufs);
}

static int zuma_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	u32 tx_line_reset_period, rx_line_reset_period;
	int i;

	rx_line_reset_period = (RX_LINE_RESET_TIME * ufs->mclk_rate) / 1000000UL;
	tx_line_reset_period = (TX_LINE_RESET_TIME * ufs->mclk_rate) / 1000000UL;

	unipro_writel(ufs, get_mclk_period_unipro_18(ufs), COMP_CLK_PERIOD);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x44), 0x00);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x202), 0x22);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for (i = 0; i < 2; i++) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(1000000000UL, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       rx_line_reset_period & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x79);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for (i = 0; i < 2; i++) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(1000000000UL, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i), 0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       tx_line_reset_period & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x7f, i), 0);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x3000), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x3001), 0x1);

	return 0;
}

static int zuma_ufs_post_link(struct exynos_ufs *ufs)
{
	int ret;

	ret = samsung_ufs_phy_calibrate_stage(&ufs->phy, CFG_POST_INIT);
	if (ret)
		return ret;

	hci_writel(ufs, WLU_EN | WLU_BURST_LEN(3), HCI_AXIDMA_RWDATA_BURST_LEN);
	ufshcd_dme_set(ufs->hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0x3e8);

	return 0;
}

static int exynos_ufs_hce_enable_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);
	u32 val, saved_misc;
	unsigned long timeout;

	if (status == POST_CHANGE) {
		exynos_ufs_auto_ctrl_hcc(ufs, true);
		return 0;
	}

	exynos_ufs_disable_auto_ctrl_hcc_save(ufs, &saved_misc);
	hci_writel(ufs, 0, HCI_GPIO_OUT);
	udelay(5);
	hci_writel(ufs, 1, HCI_GPIO_OUT);

	hci_writel(ufs, UFS_SW_RST_MASK, HCI_SW_RST);
	timeout = timer_get_us() + 1000;
	do {
		val = hci_readl(ufs, HCI_SW_RST);
		if (!(val & UFS_SW_RST_MASK)) {
			exynos_ufs_auto_ctrl_hcc_restore(ufs, &saved_misc);
			return 0;
		}
	} while (timer_get_us() < timeout);

	dev_err(hba->dev, "ufs host reset timeout\n");
	exynos_ufs_auto_ctrl_hcc_restore(ufs, &saved_misc);
	return -ETIMEDOUT;
}

static int exynos_ufs_link_startup_notify(struct ufs_hba *hba,
					  enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);
	u32 val;
	int ret;

	if (status == PRE_CHANGE) {
		dev_info(hba->dev, "ufs pre-link start\n");
		exynos_ufs_config_intr(ufs, DFES_DEF_L2_ERRS, HCI_ERR_EN_DL_LAYER);
		exynos_ufs_config_intr(ufs, DFES_DEF_L3_ERRS, HCI_ERR_EN_N_LAYER);
		exynos_ufs_config_intr(ufs, DFES_DEF_L4_ERRS, HCI_ERR_EN_T_LAYER);
		exynos_ufs_config_intr(ufs, 0, HCI_ERR_EN_DME_LAYER);
		exynos_ufs_ctrl_clkstop(ufs, false);
		exynos_ufs_config_unipro(ufs);

		if (ufs->drv_data->pre_link) {
			ret = ufs->drv_data->pre_link(ufs);
			if (ret)
				return ret;
		}

		generic_phy_power_off(&ufs->phy);
		generic_phy_exit(&ufs->phy);

		ret = generic_phy_init(&ufs->phy);
		if (ret)
			return ret;

		ret = generic_phy_power_on(&ufs->phy);
		if (!ret)
			dev_info(hba->dev, "ufs phy ready for link startup\n");

		return ret;
	}

	exynos_ufs_establish_connt(ufs);
	val = exynos_ufs_calc_time_cntr(ufs, IATOVAL_NSEC / CNTR_DIV_VAL);
	hci_writel(ufs, hci_readl(ufs, HCI_V2P1_CTRL) | IA_TICK_SEL, HCI_V2P1_CTRL);
	hci_writel(ufs, val & CNT_VAL_1US_MASK, HCI_1US_TO_CNT_VAL);
	hci_writel(ufs, 0xa, HCI_DATA_REORDER);
	hci_writel(ufs, 12, HCI_TXPRDT_ENTRY_SIZE);
	hci_writel(ufs, 12, HCI_RXPRDT_ENTRY_SIZE);
	hci_writel(ufs, 0xffffffff, HCI_UTRL_NEXUS_TYPE);
	hci_writel(ufs, 0xff, HCI_UTMRL_NEXUS_TYPE);
	ret = ufs->drv_data->post_link ? ufs->drv_data->post_link(ufs) : 0;
	if (ret)
		return ret;
	dev_info(hba->dev, "ufs post-link complete\n");

	return 0;
}

static int exynos_ufs_device_reset(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);

	hci_writel(ufs, 0, HCI_GPIO_OUT);
	udelay(10);
	hci_writel(ufs, 1, HCI_GPIO_OUT);
	udelay(10);

	return 0;
}

static struct ufs_hba_ops exynos_ufs_hba_ops = {
	.init			= exynos_ufs_init,
	.hce_enable_notify	= exynos_ufs_hce_enable_notify,
	.link_startup_notify	= exynos_ufs_link_startup_notify,
	.device_reset		= exynos_ufs_device_reset,
};

static int exynos_ufs_probe(struct udevice *dev)
{
	return ufshcd_probe(dev, &exynos_ufs_hba_ops);
}

static const struct udevice_id exynos_ufs_ids[] = {
	{ .compatible = "google,zuma-ufs", .data = (ulong)&zuma_ufs_drv_data },
	{ .compatible = "google,gs101-ufs", .data = (ulong)&zuma_ufs_drv_data },
	{ }
};

U_BOOT_DRIVER(exynos_ufshcd) = {
	.name		= "exynos-ufshcd",
	.id		= UCLASS_UFS,
	.of_match	= exynos_ufs_ids,
	.probe		= exynos_ufs_probe,
	.priv_auto	= sizeof(struct exynos_ufs),
};
