/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _PHY_SAMSUNG_UFS_H_
#define _PHY_SAMSUNG_UFS_H_

#include <clk.h>
#include <generic-phy.h>
#include <regmap.h>

#define PHY_COMN_BLK	1
#define PHY_TRSV_BLK	2
#define END_UFS_PHY_CFG { 0 }
#define PHY_APB_ADDR(off)	((off) << 2)

#define PHY_COMN_REG_CFG(o, v, d) {	\
	.off_0 = PHY_APB_ADDR((o)),	\
	.off_1 = 0,			\
	.val = (v),			\
	.desc = (d),			\
	.id = PHY_COMN_BLK,		\
}

#define PHY_TRSV_REG_CFG_OFFSET(o, v, d, c) {	\
	.off_0 = PHY_APB_ADDR((o)),		\
	.off_1 = PHY_APB_ADDR((o) + (c)),	\
	.val = (v),				\
	.desc = (d),				\
	.id = PHY_TRSV_BLK,			\
}

enum {
	PWR_DESC_ANY	= 0,
	PWR_DESC_PWM	= 1,
	PWR_DESC_HS	= 2,
	PWR_DESC_SER_A	= 1,
	PWR_DESC_SER_B	= 2,
	MD_MASK		= 0x3,
	SR_MASK		= 0x3,
	GR_MASK		= 0x7,
};

#define PWR_MODE(g, s, m)	((((g) & GR_MASK) << 4) | \
				 (((s) & SR_MASK) << 2) | ((m) & MD_MASK))
#define PWR_MODE_PWM_ANY	PWR_MODE(PWR_DESC_ANY, PWR_DESC_ANY, PWR_DESC_PWM)
#define PWR_MODE_ANY		PWR_MODE(PWR_DESC_ANY, PWR_DESC_ANY, PWR_DESC_ANY)

enum samsung_ufs_phy_cfg_tag {
	CFG_PRE_INIT,
	CFG_POST_INIT,
	CFG_PRE_PWR_HS,
	CFG_POST_PWR_HS,
	CFG_TAG_MAX,
};

struct samsung_ufs_phy_cfg {
	u32 off_0;
	u32 off_1;
	u32 val;
	u8 desc;
	u8 id;
};

struct samsung_ufs_phy_pmu_isol {
	u32 offset;
	u32 mask;
	u32 en;
};

struct samsung_ufs_phy_drvdata {
	const struct samsung_ufs_phy_cfg * const *cfgs;
	struct samsung_ufs_phy_pmu_isol isol;
	u8 lane_cnt;
	int (*wait_for_cal)(struct udevice *dev, void __iomem *reg_pma, u8 lane);
	int (*wait_for_cdr)(struct udevice *dev, void __iomem *reg_pma, u8 lane);
};

struct samsung_ufs_phy {
	void __iomem *reg_pma;
	struct regmap *reg_pmu;
	struct clk_bulk clks;
	const struct samsung_ufs_phy_drvdata *drvdata;
	enum samsung_ufs_phy_cfg_tag state;
};

int samsung_ufs_phy_calibrate_stage(struct phy *generic_phy,
				    enum samsung_ufs_phy_cfg_tag stage);

#endif
