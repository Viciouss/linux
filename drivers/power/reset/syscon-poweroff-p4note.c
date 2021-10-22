// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Generic Syscon Poweroff Driver
 *
 * Copyright (c) 2015, National Instruments Corp.
 * Author: Moritz Fischer <moritz.fischer@ettus.com>
 */

#include <linux/kallsyms.h>
#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>

static struct regmap *map;
static u32 offset;
static u32 value;
static u32 mask;
static u32 lpm_offset;
static struct extcon_dev *edev;

static void syscon_poweroff(void)
{
	if (extcon_get_state(edev, EXTCON_CHG_USB_SDP) || extcon_get_state(edev, EXTCON_CHG_USB_DCP)) {
		regmap_update_bits(map, lpm_offset, 0xffffffff, 0);
		regmap_update_bits(map, 0x0400, 0xffffffff, 0x1);
	} else {
		/* Issue the poweroff */
		regmap_update_bits(map, offset, mask, value);

		mdelay(1000);

		pr_emerg("Unable to poweroff system\n");
	}
}

static int syscon_poweroff_probe(struct platform_device *pdev)
{
	char symname[KSYM_NAME_LEN];
	int mask_err, value_err;

	map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "regmap");
	if (IS_ERR(map)) {
		dev_err(&pdev->dev, "unable to get syscon");
		return PTR_ERR(map);
	}

	if (of_property_read_u32(pdev->dev.of_node, "offset", &offset)) {
		dev_err(&pdev->dev, "unable to read 'offset'");
		return -EINVAL;
	}

	value_err = of_property_read_u32(pdev->dev.of_node, "value", &value);
	mask_err = of_property_read_u32(pdev->dev.of_node, "mask", &mask);
	if (value_err && mask_err) {
		dev_err(&pdev->dev, "unable to read 'value' and 'mask'");
		return -EINVAL;
	}

	if (value_err) {
		/* support old binding */
		value = mask;
		mask = 0xFFFFFFFF;
	} else if (mask_err) {
		/* support value without mask*/
		mask = 0xFFFFFFFF;
	}

	if (pm_power_off) {
		lookup_symbol_name((ulong)pm_power_off, symname);
		dev_err(&pdev->dev,
		"pm_power_off already claimed %p %s",
		pm_power_off, symname);
		return -EBUSY;
	}

	edev = extcon_get_edev_by_phandle(&pdev->dev, 0);
	if (IS_ERR(edev)) {
		if (PTR_ERR(edev) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "missing extcon connection\n");
		return PTR_ERR(edev);
	}

	if (of_property_read_u32(pdev->dev.of_node, "lpm_offset", &lpm_offset) && edev) {
		dev_err(&pdev->dev, "lpm offset not set");
		return -EINVAL;
	}

	pm_power_off = syscon_poweroff;

	return 0;
}

static int syscon_poweroff_remove(struct platform_device *pdev)
{
	if (pm_power_off == syscon_poweroff)
		pm_power_off = NULL;

	return 0;
}

static const struct of_device_id syscon_poweroff_of_match[] = {
	{ .compatible = "syscon-poweroff,p4note" },
	{}
};

static struct platform_driver syscon_poweroff_driver = {
	.probe = syscon_poweroff_probe,
	.remove = syscon_poweroff_remove,
	.driver = {
		.name = "syscon-poweroff_p4note",
		.of_match_table = syscon_poweroff_of_match,
	},
};

builtin_platform_driver(syscon_poweroff_driver);