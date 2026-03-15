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
#include <linux/err.h>
#include <linux/iopoll.h>

#include "phy-samsung-ufs.h"

#define PHY_GS101_LANE_OFFSET		0x200
#define PHY_PLL_LOCK_STATUS		0x1e
#define PHY_PLL_LOCK_BIT		BIT(5)
#define TRSV_REG31D			0x31d
#define TRSV_REG338			0x338
#define TRSV_REG339			0x339
#define LN_RX_CAL_DONE_ZUMA		BIT(3)
#define LN_RX_CAL_DONE_GS101		BIT(3)
#define LN_RX_CDR_DONE_GS101		BIT(3)

#define PHY_PMA_TRSV_ADDR(reg, lane)	(PHY_APB_ADDR((reg) + \
					 ((lane) * PHY_GS101_LANE_OFFSET)))
#define PHY_TRSV_REG_CFG_GS101(o, v, d) \
	PHY_TRSV_REG_CFG_OFFSET(o, v, d, PHY_GS101_LANE_OFFSET)

static const struct samsung_ufs_phy_cfg tensor_zuma_pre_init_cfg[] = {
	PHY_COMN_REG_CFG(0x50, 0x08, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x05, 0x19, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0b, 0x44, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0c, 0xc4, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0d, 0xc3, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0f, 0x88, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x16, 0x1a, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x19, 0x04, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x54, 0x88, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x67, 0x4c, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x68, 0x4c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x201, 0x44, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x202, 0x44, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x203, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x204, 0x18, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x205, 0xc0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x207, 0x1c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2ec, 0x8c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x27c, 0xd0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x288, 0xfa, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x289, 0x60, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x234, 0x30, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x239, 0x05, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x23d, 0x05, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x24d, 0x1a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x24e, 0x12, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x24f, 0x5e, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x259, 0x2a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x260, 0x54, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x266, 0x54, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x273, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x274, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2ab, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2ac, 0x02, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x50, 0x0c, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x50, 0x00, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg tensor_gs101_pre_pwr_hs_config[] = {
	PHY_TRSV_REG_CFG_GS101(0x369, 0x11, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x246, 0x03, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg tensor_gs101_post_pwr_hs_config[] = {
	PHY_COMN_REG_CFG(0x8, 0x60, PWR_MODE_PWM_ANY),
	PHY_TRSV_REG_CFG_GS101(0x222, 0x08, PWR_MODE_PWM_ANY),
	PHY_TRSV_REG_CFG_GS101(0x246, 0x01, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg * const tensor_zuma_ufs_phy_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT]		= tensor_zuma_pre_init_cfg,
	[CFG_PRE_PWR_HS]	= tensor_gs101_pre_pwr_hs_config,
	[CFG_POST_PWR_HS]	= tensor_gs101_post_pwr_hs_config,
};

static int zuma_phy_wait_for_calibration(struct udevice *dev,
					 void __iomem *reg_pma, u8 lane)
{
	const unsigned int timeout_us = 200000;
	const unsigned int sleep_us = 40;
	u32 val_c74, val_ce0;
	u32 off_c74, off_ce0;
	unsigned int elapsed = 0;

	off_c74 = PHY_PMA_TRSV_ADDR(TRSV_REG31D, lane);
	off_ce0 = PHY_PMA_TRSV_ADDR(TRSV_REG338, lane);

	do {
		val_c74 = readl(reg_pma + off_c74);
		val_ce0 = readl(reg_pma + off_ce0);
		if ((val_c74 & LN_RX_CAL_DONE_ZUMA) ||
		    (val_ce0 & LN_RX_CAL_DONE_GS101))
			return 0;
		udelay(sleep_us);
		elapsed += sleep_us;
	} while (elapsed < timeout_us);

	dev_err(dev, "ufs phy cal timeout lane %u c74=%#x ce0=%#x\n",
		lane, val_c74, val_ce0);
	return -ETIMEDOUT;
}

static int gs101_phy_wait_for_cdr_lock(struct udevice *dev,
				       void __iomem *reg_pma, u8 lane)
{
	u32 val;
	int i;

	for (i = 0; i < 100; i++) {
		udelay(40);
		val = readl(reg_pma + PHY_PMA_TRSV_ADDR(TRSV_REG339, lane));
		if (val & LN_RX_CDR_DONE_GS101)
			return 0;
	}

	dev_err(dev, "ufs phy cdr timeout lane %u\n", lane);
	return -ETIMEDOUT;
}

static const struct samsung_ufs_phy_drvdata tensor_zuma_ufs_phy = {
	.cfgs			= tensor_zuma_ufs_phy_cfgs,
	.isol			= {
		.offset = 0x3ec8,
		.mask = BIT(0),
		.en = BIT(0),
	},
	.lane_cnt		= 2,
	.wait_for_cal		= zuma_phy_wait_for_calibration,
	.wait_for_cdr		= NULL,
};

static void samsung_ufs_phy_apply_cfg(struct samsung_ufs_phy *phy,
				      const struct samsung_ufs_phy_cfg *cfgs)
{
	const struct samsung_ufs_phy_cfg *cfg;
	int lane;

	if (!cfgs)
		return;

	for (cfg = cfgs; cfg->id; cfg++) {
		for (lane = 0; lane < phy->drvdata->lane_cnt; lane++) {
			if (cfg->id == PHY_COMN_BLK)
				writel(cfg->val, phy->reg_pma + cfg->off_0);
			else if (lane == 0)
				writel(cfg->val, phy->reg_pma + cfg->off_0);
			else
				writel(cfg->val, phy->reg_pma + cfg->off_1);
		}
	}
}

static void samsung_ufs_phy_set_isol(struct samsung_ufs_phy *phy, bool isolate)
{
	regmap_update_bits(phy->reg_pmu, phy->drvdata->isol.offset,
			   phy->drvdata->isol.mask,
			   isolate ? 0 : phy->drvdata->isol.en);
}

int samsung_ufs_phy_calibrate_stage(struct phy *generic_phy,
				    enum samsung_ufs_phy_cfg_tag stage)
{
	struct samsung_ufs_phy *phy = dev_get_priv(generic_phy->dev);
	const struct samsung_ufs_phy_cfg *cfgs;
	int lane;
	static const char * const stage_name[] = {
		[CFG_PRE_INIT] = "pre-init",
		[CFG_POST_INIT] = "post-init",
		[CFG_PRE_PWR_HS] = "pre-pwr-hs",
		[CFG_POST_PWR_HS] = "post-pwr-hs",
	};

	if (stage >= CFG_TAG_MAX)
		return -EINVAL;

	cfgs = phy->drvdata->cfgs[stage];
	if (!cfgs)
		return 0;

	dev_info(generic_phy->dev, "ufs phy stage %s\n", stage_name[stage]);
	phy->state = stage;
	samsung_ufs_phy_apply_cfg(phy, cfgs);

	if (stage == CFG_PRE_INIT) {
		for (lane = 0; lane < phy->drvdata->lane_cnt; lane++) {
			int ret;

			ret = phy->drvdata->wait_for_cal(generic_phy->dev,
							 phy->reg_pma, lane);
			if (ret)
				return ret;
		}
	} else if (stage == CFG_POST_PWR_HS && phy->drvdata->wait_for_cdr) {
		for (lane = 0; lane < phy->drvdata->lane_cnt; lane++) {
			int ret;

			ret = phy->drvdata->wait_for_cdr(generic_phy->dev,
							 phy->reg_pma, lane);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int samsung_ufs_phy_power_on(struct phy *generic_phy)
{
	struct samsung_ufs_phy *phy = dev_get_priv(generic_phy->dev);
	int ret;

	ret = clk_enable_bulk(&phy->clks);
	if (ret)
		return ret;

	samsung_ufs_phy_set_isol(phy, false);
	dev_info(generic_phy->dev, "ufs phy power_on de-isolated\n");

	ret = samsung_ufs_phy_calibrate_stage(generic_phy, CFG_PRE_INIT);
	if (ret)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_bulk(&phy->clks);
	return ret;
}

static int samsung_ufs_phy_power_off(struct phy *generic_phy)
{
	struct samsung_ufs_phy *phy = dev_get_priv(generic_phy->dev);

	samsung_ufs_phy_set_isol(phy, true);
	clk_disable_bulk(&phy->clks);

	return 0;
}

static const struct phy_ops samsung_ufs_phy_ops = {
	.power_on	= samsung_ufs_phy_power_on,
	.power_off	= samsung_ufs_phy_power_off,
};

static int samsung_ufs_phy_probe(struct udevice *dev)
{
	struct samsung_ufs_phy *phy = dev_get_priv(dev);

	phy->drvdata = (const struct samsung_ufs_phy_drvdata *)dev_get_driver_data(dev);
	phy->reg_pma = dev_read_addr_name_ptr(dev, "phy-pma");
	if (!phy->reg_pma)
		return -EINVAL;

	phy->reg_pmu = syscon_regmap_lookup_by_phandle(dev, "samsung,pmu-syscon");
	if (IS_ERR(phy->reg_pmu))
		return PTR_ERR(phy->reg_pmu);

	phy->state = CFG_PRE_INIT;

	return clk_get_bulk(dev, &phy->clks);
}

static const struct udevice_id samsung_ufs_phy_ids[] = {
	{ .compatible = "google,zuma-ufs-phy", .data = (ulong)&tensor_zuma_ufs_phy },
	{ .compatible = "google,gs101-ufs-phy", .data = (ulong)&tensor_zuma_ufs_phy },
	{ }
};

U_BOOT_DRIVER(samsung_ufs_phy) = {
	.name		= "samsung-ufs-phy",
	.id		= UCLASS_PHY,
	.of_match	= samsung_ufs_phy_ids,
	.ops		= &samsung_ufs_phy_ops,
	.probe		= samsung_ufs_phy_probe,
	.priv_auto	= sizeof(struct samsung_ufs_phy),
};
