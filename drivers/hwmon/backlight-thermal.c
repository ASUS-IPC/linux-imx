/*
 * backlight-thermal.c - Hwmon driver for fans connected to Backlight lines.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *
 * Author: Kamil Debski <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/backlight.h>

#define MAX_BACKLIGHT 1000
#define BL_DEBUG 0
struct backlight_thermal_ctx {
	struct mutex lock;
	unsigned int backlight_thermal_value;
	unsigned int backlight_thermal_state;
	unsigned int backlight_thermal_max_state;
	unsigned int *backlight_thermal_cooling_levels;
	struct thermal_cooling_device *cdev;
};
static int retrytimes = 0;
extern void update_thermal_max_brightness(int thermal_max_brightness);
extern int get_panelid(void);
extern int panel_simple_enstatus(void);

static void backlight_thermal_update_max_value(struct backlight_thermal_ctx *ctx, int state, unsigned long backlight)
{
	if(BL_DEBUG) printk("%s state-%d thermal max brightness set to %lu", __func__, state, backlight);
	mutex_lock(&ctx->lock);
	ctx->backlight_thermal_cooling_levels[state] = backlight;
	update_thermal_max_brightness(ctx->backlight_thermal_cooling_levels[ctx->backlight_thermal_state]);
	mutex_unlock(&ctx->lock);
}

static ssize_t set_lvds_max_brightness0(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct backlight_thermal_ctx *ctx = dev_get_drvdata(dev);
	unsigned long backlight;

	if (kstrtoul(buf, 10, &backlight) || backlight > MAX_BACKLIGHT || (backlight < ctx->backlight_thermal_cooling_levels[1]))
		return -EINVAL;

	backlight_thermal_update_max_value(ctx,0,backlight);
	return count;
}

static ssize_t show_lvds_max_brightness0(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct backlight_thermal_ctx *ctx = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", ctx->backlight_thermal_cooling_levels[0]);
}

static ssize_t set_lvds_max_brightness1(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct backlight_thermal_ctx *ctx = dev_get_drvdata(dev);
	unsigned long backlight;

	if (kstrtoul(buf, 10, &backlight) || backlight > MAX_BACKLIGHT || (backlight < ctx->backlight_thermal_cooling_levels[2]) || (backlight > ctx->backlight_thermal_cooling_levels[0]))
		return -EINVAL;
	backlight_thermal_update_max_value(ctx,1,backlight);
	return count;
}

static ssize_t show_lvds_max_brightness1(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct backlight_thermal_ctx *ctx = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", ctx->backlight_thermal_cooling_levels[1]);
}

static ssize_t set_lvds_max_brightness2(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct backlight_thermal_ctx *ctx = dev_get_drvdata(dev);
	unsigned long backlight;

	if (kstrtoul(buf, 10, &backlight) || backlight > MAX_BACKLIGHT || (backlight > ctx->backlight_thermal_cooling_levels[1]))
		return -EINVAL;

	backlight_thermal_update_max_value(ctx,2,backlight);
	return count;
}

static ssize_t show_lvds_max_brightness2(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct backlight_thermal_ctx *ctx = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", ctx->backlight_thermal_cooling_levels[2]);
}

static SENSOR_DEVICE_ATTR(lvds_max_brightness0, S_IRUGO | S_IWUSR, show_lvds_max_brightness0, set_lvds_max_brightness0, 0);
static SENSOR_DEVICE_ATTR(lvds_max_brightness1, S_IRUGO | S_IWUSR, show_lvds_max_brightness1, set_lvds_max_brightness1, 0);
static SENSOR_DEVICE_ATTR(lvds_max_brightness2, S_IRUGO | S_IWUSR, show_lvds_max_brightness2, set_lvds_max_brightness2, 0);

static struct attribute *backlight_thermal_attrs[] = {
	&sensor_dev_attr_lvds_max_brightness0.dev_attr.attr,
	&sensor_dev_attr_lvds_max_brightness1.dev_attr.attr,
	&sensor_dev_attr_lvds_max_brightness2.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(backlight_thermal);

/* thermal cooling device callbacks */
static int backlight_thermal_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct backlight_thermal_ctx *ctx = cdev->devdata;
	if(BL_DEBUG) printk("%s state: %d\n", __func__, ctx->backlight_thermal_max_state);
	if (!ctx)
		return -EINVAL;

	*state = ctx->backlight_thermal_max_state;

	return 0;
}

static int backlight_thermal_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct backlight_thermal_ctx *ctx = cdev->devdata;
	if(BL_DEBUG) printk("%s state: %d\n", __func__, ctx->backlight_thermal_state);
	if (!ctx)
		return -EINVAL;

	*state = ctx->backlight_thermal_state;

	return 0;
}

static int
backlight_thermal_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct backlight_thermal_ctx *ctx = cdev->devdata;
	int ret = 0;

	if(BL_DEBUG) printk("%s current backlight_thermal_state: %d set to %d\n", __func__, ctx->backlight_thermal_state, state);
	if (!ctx || (state > ctx->backlight_thermal_max_state))
		return -EINVAL;

	mutex_lock(&ctx->lock);
	ctx->backlight_thermal_state = state;
	update_thermal_max_brightness(ctx->backlight_thermal_cooling_levels[ctx->backlight_thermal_state]);
	mutex_unlock(&ctx->lock);
	return ret;
}

static const struct thermal_cooling_device_ops backlight_thermal_cooling_ops = {
	.get_max_state = backlight_thermal_get_max_state,
	.get_cur_state = backlight_thermal_get_cur_state,
	.set_cur_state = backlight_thermal_set_cur_state,
};

static int backlight_thermal_of_get_cooling_data(struct device *dev,
				       struct backlight_thermal_ctx *ctx)
{
	struct device_node *np = dev->of_node;
	int num, i, ret;
	int panelid = 2;
	const char *koe_cooling_levels="koe-cooling-levels";
	const char *amp_cooling_levels="amp-cooling-levels";

	panelid = get_panelid();

	if(panelid == 1) {
		if (!of_find_property(np, koe_cooling_levels, NULL))
			return 0;

		ret = of_property_count_u32_elems(np, koe_cooling_levels);
		if (ret <= 0) {
			dev_err(dev, "Wrong data!\n");
			return ret ? : -EINVAL;
		}

		num = ret;
		ctx->backlight_thermal_cooling_levels = devm_kcalloc(dev, num, sizeof(u32),
							GFP_KERNEL);
		if (!ctx->backlight_thermal_cooling_levels)
			return -ENOMEM;

		ret = of_property_read_u32_array(np, koe_cooling_levels,
						ctx->backlight_thermal_cooling_levels, num);
		if (ret) {
			dev_err(dev, "Property 'cooling-levels' cannot be read!\n");
			return ret;
		}

	} else if(panelid == 2) {
		if (!of_find_property(np, amp_cooling_levels, NULL))
		return 0;

		ret = of_property_count_u32_elems(np, amp_cooling_levels);
		if (ret <= 0) {
			dev_err(dev, "Wrong data!\n");
			return ret ? : -EINVAL;
		}

		num = ret;
		ctx->backlight_thermal_cooling_levels = devm_kcalloc(dev, num, sizeof(u32),
							GFP_KERNEL);
		if (!ctx->backlight_thermal_cooling_levels)
			return -ENOMEM;

		ret = of_property_read_u32_array(np, amp_cooling_levels,
						ctx->backlight_thermal_cooling_levels, num);
		if (ret) {
			dev_err(dev, "Property 'cooling-levels' cannot be read!\n");
			return ret;
		}

	} else {
		dev_err(dev, "Cannot find panel!\n");
		return -EINVAL;
	}

	for (i = 0; i < num; i++) {
		if (ctx->backlight_thermal_cooling_levels[i] > MAX_BACKLIGHT) {
			dev_err(dev, "thermal backlight state[%d]:%d > %d\n", i,
				ctx->backlight_thermal_cooling_levels[i], MAX_BACKLIGHT);
			return -EINVAL;
		}
	}

	ctx->backlight_thermal_max_state = num - 1;

	return 0;
}

static int backlight_thermal_probe(struct platform_device *pdev)
{
	struct thermal_cooling_device *cdev;
	struct device *dev = &pdev->dev;
	struct backlight_thermal_ctx *ctx;
	struct device *hwmon;
	int ret;

	printk("backlight_thermal_probe +\n");

	if(!panel_simple_enstatus() && retrytimes < 3) {
		retrytimes++;
		return -EPROBE_DEFER;
	}

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);

	platform_set_drvdata(pdev, ctx);

	ctx->backlight_thermal_value = MAX_BACKLIGHT;

	hwmon = devm_hwmon_device_register_with_groups(&pdev->dev, "backlightthermal",
						       ctx, backlight_thermal_groups);
	if (IS_ERR(hwmon)) {
		dev_err(&pdev->dev, "Failed to register hwmon device\n");
		ret = PTR_ERR(hwmon);
		goto err_backlight_disable;
	}

	ret = backlight_thermal_of_get_cooling_data(&pdev->dev, ctx);
	if (ret)
		goto err_backlight_disable;

	ctx->backlight_thermal_state = 0;
	if (IS_ENABLED(CONFIG_THERMAL)) {
		cdev = devm_thermal_of_cooling_device_register(dev,dev->of_node,
							  "backlight-thermal", ctx,
							  &backlight_thermal_cooling_ops);
		if (IS_ERR(cdev)) {
			dev_err(&pdev->dev,
				"Failed to register backlight-thermal as cooling device");
			ret = PTR_ERR(cdev);
			goto err_backlight_disable;
		}
		ctx->cdev = cdev;
		//thermal_cdev_update(cdev);
	}

	return 0;

err_backlight_disable:
	printk("backlight_thermal_probe -\n");
	return ret;
}

static int backlight_thermal_remove(struct platform_device *pdev)
{
	struct backlight_thermal_ctx *ctx = platform_get_drvdata(pdev);

	thermal_cooling_device_unregister(ctx->cdev);

	return 0;
}

static const struct of_device_id of_backlight_thermal_match[] = {
	{ .compatible = "backlight-thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, of_backlight_thermal_match);

static struct platform_driver backlight_thermal_driver = {
	.probe		= backlight_thermal_probe,
	.remove		= backlight_thermal_remove,
	.driver	= {
		.name		= "backlight-thermal",
		.of_match_table	= of_backlight_thermal_match,
	},
};

module_platform_driver(backlight_thermal_driver);

MODULE_AUTHOR("Gary_Gen@asus.com");
MODULE_ALIAS("platform:backlight-thermal");
MODULE_DESCRIPTION("Backlight Thermal driver");
MODULE_LICENSE("GPL");
