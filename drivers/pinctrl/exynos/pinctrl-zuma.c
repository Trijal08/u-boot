// SPDX-License-Identifier: GPL-2.0+
/*
 * Zuma pinctrl driver.
 *
 * Minimal U-Boot support for the GPIO ALIVE and GPIO CUSTOM ALIVE
 * controllers used by Pixel 9 button input.
 */

#include <dm.h>
#include <dm/pinctrl.h>
#include "pinctrl-exynos.h"

static const struct samsung_pin_bank_type bank_type_zuma_alive = {
	.fld_width = { 4, 1, 4, 4, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, },
};

static const struct pinctrl_ops zuma_pinctrl_ops = {
	.set_state	= exynos_pinctrl_set_state,
};

static const struct samsung_pin_bank_data zuma_pin_banks_alive[] = {
	{ .type = &bank_type_zuma_alive, .offset = 0x000, .nr_pins = 8, .name = "gpa0" },
	{ .type = &bank_type_zuma_alive, .offset = 0x020, .nr_pins = 8, .name = "gpa1" },
	{ .type = &bank_type_zuma_alive, .offset = 0x040, .nr_pins = 8, .name = "gpa2" },
	{ .type = &bank_type_zuma_alive, .offset = 0x060, .nr_pins = 8, .name = "gpa3" },
	{ .type = &bank_type_zuma_alive, .offset = 0x080, .nr_pins = 2, .name = "gpa4" },
	{ .type = &bank_type_zuma_alive, .offset = 0x0a0, .nr_pins = 6, .name = "gpa6" },
	{ .type = &bank_type_zuma_alive, .offset = 0x0c0, .nr_pins = 8, .name = "gpa7" },
	{ .type = &bank_type_zuma_alive, .offset = 0x0e0, .nr_pins = 4, .name = "gpa8" },
	{ .type = &bank_type_zuma_alive, .offset = 0x100, .nr_pins = 7, .name = "gpa9" },
	{ .type = &bank_type_zuma_alive, .offset = 0x120, .nr_pins = 5, .name = "gpa10" },
};

static const struct samsung_pin_bank_data zuma_pin_banks_custom_alive[] = {
	{ .type = &bank_type_zuma_alive, .offset = 0x000, .nr_pins = 1, .name = "gpn0" },
	{ .type = &bank_type_zuma_alive, .offset = 0x020, .nr_pins = 1, .name = "gpn1" },
	{ .type = &bank_type_zuma_alive, .offset = 0x040, .nr_pins = 1, .name = "gpn2" },
	{ .type = &bank_type_zuma_alive, .offset = 0x060, .nr_pins = 1, .name = "gpn3" },
	{ .type = &bank_type_zuma_alive, .offset = 0x080, .nr_pins = 1, .name = "gpn4" },
	{ .type = &bank_type_zuma_alive, .offset = 0x0a0, .nr_pins = 1, .name = "gpn5" },
	{ .type = &bank_type_zuma_alive, .offset = 0x0c0, .nr_pins = 1, .name = "gpn6" },
	{ .type = &bank_type_zuma_alive, .offset = 0x0e0, .nr_pins = 1, .name = "gpn7" },
	{ .type = &bank_type_zuma_alive, .offset = 0x100, .nr_pins = 1, .name = "gpn8" },
	{ .type = &bank_type_zuma_alive, .offset = 0x120, .nr_pins = 1, .name = "gpn9" },
};

static const struct samsung_pin_ctrl zuma_pin_ctrl[] = {
	{
		.pin_banks	= zuma_pin_banks_alive,
		.nr_banks	= ARRAY_SIZE(zuma_pin_banks_alive),
	}, {
		.pin_banks	= zuma_pin_banks_custom_alive,
		.nr_banks	= ARRAY_SIZE(zuma_pin_banks_custom_alive),
	},
	{ }
};

static const struct udevice_id zuma_pinctrl_ids[] = {
	{ .compatible = "google,zuma-pinctrl", .data = (ulong)zuma_pin_ctrl },
	{ }
};

U_BOOT_DRIVER(pinctrl_zuma) = {
	.name		= "pinctrl_zuma",
	.id		= UCLASS_PINCTRL,
	.of_match	= zuma_pinctrl_ids,
	.priv_auto	= sizeof(struct exynos_pinctrl_priv),
	.ops		= &zuma_pinctrl_ops,
	.probe		= exynos_pinctrl_probe,
	.bind		= exynos_pinctrl_bind,
};
