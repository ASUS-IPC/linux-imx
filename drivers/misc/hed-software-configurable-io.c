/*
 * hed-software-configurable-io.c
 *
 * Copyright 2016,2019,2022 HED, Inc.
 *
 * Author: Matthew Starr <mstarr@hedonline.com>
 *
 * based on nokia-modem.c
 *
 * Copyright (C) 2014 Sebastian Reichel <sre@kernel.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#define HED_SOFTWARE_CONFIGURABLE_IO_MODE_MAX_GPIOS 3

enum hed_software_configurable_io_mode {
	HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD,
	HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB,
	HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG,
};

struct hed_software_configurable_io_gpio {
	struct gpio_desc *gpio;
	const char *name;
};

/*
 * resume_mode - value to restore after resuming from suspend
 */
struct hed_software_configurable_io_data {
	enum hed_software_configurable_io_mode mode;
	enum hed_software_configurable_io_mode default_mode;
	enum hed_software_configurable_io_mode resume_mode;
	struct gpio_desc *input_gpio;
	struct hed_software_configurable_io_gpio *sel_gpios;
	int num_gpios;
};

static void set_mode_gpios(struct device *dev)
{
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);

	if (data->mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD) {
		gpiod_direction_output(data->sel_gpios[0].gpio, 0);
		gpiod_direction_output(data->sel_gpios[1].gpio, 1);
		gpiod_direction_output(data->sel_gpios[2].gpio, 1);
		dev_dbg(dev, "Switched gpios to VTD\n");
	} else if (data->mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB) {
		gpiod_direction_output(data->sel_gpios[0].gpio, 1);
		gpiod_direction_output(data->sel_gpios[1].gpio, 0);
		gpiod_direction_output(data->sel_gpios[2].gpio, 1);
		dev_dbg(dev, "Switched gpios to STB\n");
	} else if (data->mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG) {
		gpiod_direction_output(data->sel_gpios[0].gpio, 0);
		gpiod_direction_output(data->sel_gpios[1].gpio, 0);
		gpiod_direction_output(data->sel_gpios[2].gpio, 0);
		dev_dbg(dev, "Switched gpios to STG\n");
	}
}

static ssize_t hed_software_configurable_io_show_mode(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);
	int ret;

	if (data->mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD)
		ret = sprintf(buf, "VTD\n");
	else if (data->mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB)
		ret = sprintf(buf, "STB\n");
	else if (data->mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG)
		ret = sprintf(buf, "STG\n");

	return ret;
}

static ssize_t hed_software_configurable_io_set_mode(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);

	/*
	 * sysfs_streq() doesn't need the \n's, but we add them so the strings
	 * will be shared with show_state(), above.
	 */
	if (sysfs_streq(buf, "VTD\n") || sysfs_streq(buf, "vtd\n"))
		data->mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD;
	else if (sysfs_streq(buf, "STB\n") || sysfs_streq(buf, "stb\n"))
		data->mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB;
	else if (sysfs_streq(buf, "STG\n") || sysfs_streq(buf, "stg\n"))
		data->mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG;
	else {
		dev_err(dev, "Configuring invalid mode\n");
		return count;
	}

	set_mode_gpios(dev);

	return count;
}

/*
 * create attributes referenced by this device
 */

/*
 * create dev_attr_mode structure (device_attribute)
 *    dev_attr_mode
 *       attr.name = mode
 *       attr.mode = 644
 *       show  = hed_software_configurable_io_show_mode
 *       store = hed_software_configurable_io_set_mode
 */
static DEVICE_ATTR(mode, 0644, hed_software_configurable_io_show_mode,
					hed_software_configurable_io_set_mode);

static struct attribute *attributes[] = {
	&dev_attr_mode.attr,
	NULL,
};

static const struct attribute_group attr_group = {
	.attrs	= attributes,
};

static int hed_software_configurable_io_probe(struct platform_device *pdev)
{
	struct hed_software_configurable_io_data *drvdata;
	int gpio_count, gpio_name_count, ret, err, i;
	const char *default_mode;

	drvdata = devm_kzalloc(&pdev->dev,
			       sizeof(struct hed_software_configurable_io_data),
			       GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	/* Setup default mode value */
	ret = of_property_read_string(pdev->dev.of_node,
					"default-mode", &default_mode);
	if (ret)
		return ret;

	if (strncmp("VTD", default_mode, 3) ||
			strncmp("vtd", default_mode, 3)) {
		drvdata->default_mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD;
	} else if (strncmp("STB", default_mode, 3) ||
			strncmp("stb", default_mode, 3)) {
		drvdata->default_mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB;
	} else if (strncmp("STG", default_mode, 3) ||
			strncmp("stg", default_mode, 3)) {
		drvdata->default_mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG;
	} else {
		dev_err(&pdev->dev, "Invalid default mode\n");
		return -EINVAL;
	}

	drvdata->mode = drvdata->default_mode;

	/* Setup input gpio */
	drvdata->input_gpio =
			devm_gpiod_get(&pdev->dev, "input", GPIOD_IN);
	if (IS_ERR(drvdata->input_gpio)) {
		err = PTR_ERR(drvdata->input_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"Error getting input gpio: %d\n", err);
		return err;
	}

	err = gpiod_export(drvdata->input_gpio, 0);
	if (err)
		return err;

	err = gpiod_export_link(&pdev->dev, "MPU_IN_ADC", drvdata->input_gpio);
	if (err)
		return err;

	/* Setup selection gpios */
	gpio_count = of_gpio_named_count(pdev->dev.of_node, "selection-gpios");
	if (gpio_count < 0) {
		dev_err(&pdev->dev, "Missing selection-gpios: %d\n",
								gpio_count);
		return gpio_count;
	}

	gpio_name_count = of_property_count_strings(pdev->dev.of_node,
							"selection-gpio-names");

	if (gpio_count != gpio_name_count) {
		dev_err(&pdev->dev,
		       "selection-gpios count != selection-gpio-names count\n");
		return -EINVAL;
	}

	drvdata->sel_gpios = devm_kzalloc(&pdev->dev, gpio_count *
			sizeof(struct hed_software_configurable_io_gpio), GFP_KERNEL);
	if (!drvdata->sel_gpios)
		return -ENOMEM;

	drvdata->num_gpios = gpio_count;

	if (gpio_count > HED_SOFTWARE_CONFIGURABLE_IO_MODE_MAX_GPIOS) {
		dev_err(&pdev->dev, "Too many gpios defined\n");
		return -EINVAL;
	}

	for (i = 0; i < drvdata->num_gpios; i++) {
		drvdata->sel_gpios[i].gpio =
			devm_gpiod_get_index(&pdev->dev, "selection",
						i, GPIOD_ASIS);
		if (IS_ERR(drvdata->sel_gpios[i].gpio)) {
			dev_err(&pdev->dev,
				"Could not get selection-gpios %d\n", i);
			return PTR_ERR(drvdata->sel_gpios[i].gpio);
		}

		err = of_property_read_string_index(pdev->dev.of_node,
					"selection-gpio-names", i,
					&(drvdata->sel_gpios[i].name));

		err = gpiod_export(drvdata->sel_gpios[i].gpio, 0);
		if (err)
			return err;

		err = gpiod_export_link(&pdev->dev, drvdata->sel_gpios[i].name,
						drvdata->sel_gpios[i].gpio);
		if (err)
			return err;
	}

	/* Create sysfs mode file */
	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret != 0)
		return ret;

	platform_set_drvdata(pdev, drvdata);

	/* Set initial mode */
	set_mode_gpios(&pdev->dev);

	dev_info(&pdev->dev, "Initialized HED Input Controller");

	return 0;
}

static int hed_software_configurable_io_remove(struct platform_device *pdev)
{
	struct hed_software_configurable_io_data *data = platform_get_drvdata(pdev);
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &attr_group);

	sysfs_remove_link(&pdev->dev.kobj, "MPU_IN_ADC");
	gpiod_unexport(data->input_gpio);

	for (i = 0; i < data->num_gpios; i++) {
		sysfs_remove_link(&pdev->dev.kobj, data->sel_gpios[i].name);
		gpiod_unexport(data->sel_gpios[i].gpio);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hed_software_configurable_io_suspend(struct device *dev)
{
	/*
	 * set pin 12 on the CL-713 to VTD (HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD)
	 *  - place pin in high impedance to minimize current draw
	 */
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);

	dev_dbg(dev, "%s : %d", __func__, data->mode);
	data->resume_mode = data->mode;
	data->mode        = HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD;
	set_mode_gpios(dev);
	return 0;
}

static int hed_software_configurable_io_resume(struct device *dev)
{
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);

	dev_dbg(dev, "%s : %d", __func__, data->mode);
	data->mode = data->resume_mode;
	set_mode_gpios(dev);
	return 0;
}
#else
#define hed_software_configurable_io_suspend NULL
#define hed_software_configurable_io_resume  NULL
#endif

#if defined(CONFIG_OF)
static const struct of_device_id hed_software_configurable_io_of_match[] = {
	{ .compatible = "hed,software-configurable-io", },
	{},
};
MODULE_DEVICE_TABLE(of, hed_software_configurable_io_of_match);
#endif

struct dev_pm_ops hed_software_configurable_io_driver_power_management = {
	.suspend    = hed_software_configurable_io_suspend,
	.resume     = hed_software_configurable_io_resume,
};

static struct platform_driver hed_software_configurable_io_driver = {
	.probe		= hed_software_configurable_io_probe,
	.remove		= hed_software_configurable_io_remove,
	.driver		= {
		.name		    = "hed-software-configurable-io",
		.of_match_table	= hed_software_configurable_io_of_match,
		.pm             = &hed_software_configurable_io_driver_power_management,
	}, /* end struct device_driver */
};   /* end struct platform_driver */

module_platform_driver(hed_software_configurable_io_driver);

MODULE_AUTHOR("Matthew Starr <mstarr@hedonline.com>");
MODULE_DESCRIPTION("HED input mode controller");
MODULE_LICENSE("GPL");
