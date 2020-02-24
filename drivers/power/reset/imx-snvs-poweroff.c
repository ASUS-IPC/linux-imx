/* Power off driver for i.mx6
 * Copyright (c) 2014, FREESCALE CORPORATION.  All rights reserved.
 *
 * based on msm-poweroff.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#define SNVS_LPCR_REG	0x38    /* LP Control Register */
#define SNVS_LPCR_ON	BIT(20) | BIT(21)
#define SNVS_TOP_DP	BIT(6) | BIT(5)

#define ON_TIME_500MS	0x0 << 20
#define ON_TIME_50MS	0x1 << 20
#define ON_TIME_100MS	0x2 << 20
#define ON_TIME_0MS	0x3 << 20
#define TOP_DP_EN	0x3 << 5

static void __iomem *snvs_base;

struct poweroff_drv_data {
	struct regmap *snvs;
};

struct poweroff_drv_data *pdata = NULL;

static void do_imx_poweroff(void)
{
//	u32 value = readl(snvs_base);

	/* set TOP and DP_EN bit */
//	writel(value | 0x60, snvs_base);

	regmap_update_bits(pdata->snvs, SNVS_LPCR_REG, SNVS_LPCR_ON, ON_TIME_50MS);
	regmap_update_bits(pdata->snvs, SNVS_LPCR_REG, SNVS_TOP_DP, TOP_DP_EN);
}

static int imx_poweroff_probe(struct platform_device *pdev)
{
/*
	snvs_base = of_iomap(pdev->dev.of_node, 0);
	if (!snvs_base) {
		dev_err(&pdev->dev, "failed to get memory\n");
		return -ENODEV;
	}

	pm_power_off = do_imx_poweroff;
*/
	struct device_node *np;

	/* Get SNVS register Page */
	np = pdev->dev.of_node;
	if (!np)
		return -ENODEV;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->snvs = syscon_regmap_lookup_by_phandle(np, "regmap");
	if (IS_ERR(pdata->snvs)) {
		dev_err(&pdev->dev, "Can't get snvs syscon\n");
		return PTR_ERR(pdata->snvs);
	}

	/* set ON_TIME to 50 sec */
	regmap_update_bits(pdata->snvs, SNVS_LPCR_REG, SNVS_LPCR_ON, ON_TIME_50MS);

	pm_power_off = do_imx_poweroff;

	return 0;
}

static const struct of_device_id of_imx_poweroff_match[] = {
	{ .compatible = "fsl,sec-v4.0-poweroff", },
	{},
};
MODULE_DEVICE_TABLE(of, of_imx_poweroff_match);

static struct platform_driver imx_poweroff_driver = {
	.probe = imx_poweroff_probe,
	.driver = {
		.name = "imx-snvs-poweroff",
		.of_match_table = of_match_ptr(of_imx_poweroff_match),
	},
};

static int __init imx_poweroff_init(void)
{
	return platform_driver_register(&imx_poweroff_driver);
}
device_initcall(imx_poweroff_init);
