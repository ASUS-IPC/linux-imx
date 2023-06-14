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
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/math64.h>
#include <stdbool.h>

#define HED_SOFTWARE_CONFIGURABLE_IO_MODE_MAX_GPIOS 3
#define HED_SOFTWARE_CONFIGURABLE_IO_SCALE_MODIFIER 1000000000LL

enum hed_software_configurable_io_mode {
	HED_SOFTWARE_CONFIGURABLE_IO_MODE_INVALID = -1,
	HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD,
	HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB,
	HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG,
};

struct hed_software_configurable_io_gpio {
	struct gpio_desc *gpio;
	const char *name;
};

struct hed_software_configurable_io_mode_config {
	uint32_t mapping;
	enum hed_software_configurable_io_mode name;
};

/*
 * resume_mode - value to restore after resuming from suspend
 */
struct hed_software_configurable_io_data {
	enum hed_software_configurable_io_mode input_mode;
	enum hed_software_configurable_io_mode default_input_mode;
	enum hed_software_configurable_io_mode resume_mode;
	struct gpio_desc *input_gpio;
	int sel_gpio_count;
	struct hed_software_configurable_io_gpio *sel_gpios;
	int mode_config_count;
	struct hed_software_configurable_io_mode_config *mode_config;
	struct iio_channel *adc_channels;
	s64 voltage_scale;
	u8 adc_resolution;
	bool non_selectable_input;
};

static void hed_software_configurable_io_channel_release(void *data);

static enum hed_software_configurable_io_mode get_mode_from_string(
						const char* mode_str) {
	enum hed_software_configurable_io_mode mode =
			HED_SOFTWARE_CONFIGURABLE_IO_MODE_INVALID;

	if (strncmp("VTD", mode_str, 3) == 0 ||
			strncmp("vtd", mode_str, 3) == 0) {
		mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD;
	} else if (strncmp("STB", mode_str, 3) == 0 ||
			strncmp("stb", mode_str, 3) == 0) {
		mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB;
	} else if (strncmp("STG", mode_str, 3) == 0 ||
			strncmp("stg", mode_str, 3) == 0) {
		mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG;
	} else {
		mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_INVALID;
	}

	return mode;
}

static char* get_string_from_mode(
			enum hed_software_configurable_io_mode mode) {
	if (mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD)
		return "VTD";
	else if (mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB)
		return "STB";
	else if (mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG)
		return "STG";
	else
		return "INVALID";
}

static int get_mapping_from_mode(enum hed_software_configurable_io_mode mode,
				struct device *dev) {
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);
	int i = 0;

	for (i=0; i < data->mode_config_count; i++) {
		dev_dbg(dev, "Checking mode %d, mapping %d\n",
					data->mode_config[i].name,
					data->mode_config[i].mapping);
		if (mode == data->mode_config[i].name) {
			dev_dbg(dev, "Switching to mode %d, mapping %d\n",
					data->mode_config[i].name,
					data->mode_config[i].mapping);
			return data->mode_config[i].mapping;
		}
	}

	/* No match found */
	return -EINVAL;
}

static int set_mode_gpios(struct device *dev)
{
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);
	int mapping = 0, ret = 0, i = 0;

	if (data->non_selectable_input == false) {
		mapping = get_mapping_from_mode(data->input_mode, dev);
		if (mapping < 0) {
			dev_err(dev, "%s: No matching mapping from mode\n", __func__);
			return mapping;
		}

		ret = gpiod_direction_output(data->sel_gpios[i].gpio,
							(mapping & 0x4) >> 2);
		if (ret == 0) {
			i = 1;
			ret = gpiod_direction_output(data->sel_gpios[i].gpio,
							(mapping & 0x2) >> 1);
		}
		if (ret == 0) {
			i = 2;
			ret = gpiod_direction_output(data->sel_gpios[i].gpio,
							mapping & 0x1);
		}

		if (ret != 0) {
			dev_err(dev, "Failed to set GPIO %s state\n",
				data->sel_gpios[i].name);
			return ret;
		}
		dev_dbg(dev, "Switched gpios to %s\n",
					get_string_from_mode(data->input_mode));
		return ret;
	}
	return 0;
}

static ssize_t hed_software_configurable_io_show_mode(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);
	int ret;

	if (data->input_mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD)
		ret = sprintf(buf, "VTD\n");
	else if (data->input_mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB)
		ret = sprintf(buf, "STB\n");
	else if (data->input_mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG)
		ret = sprintf(buf, "STG\n");

	return ret;
}

static ssize_t hed_software_configurable_io_set_mode(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (data->non_selectable_input == false){
		/*
		* sysfs_streq() doesn't need the \n's, but we add them so the strings
		* will be shared with show_state(), above.
		*/
		if (sysfs_streq(buf, "VTD\n") == 1 ||
					sysfs_streq(buf, "vtd\n") == 1)
			data->input_mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD;
		else if (sysfs_streq(buf, "STB\n") == 1 ||
					sysfs_streq(buf, "stb\n") == 1)
			data->input_mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB;
		else if (sysfs_streq(buf, "STG\n") == 1 ||
					sysfs_streq(buf, "stg\n") == 1)
			data->input_mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG;
		else {
			dev_err(dev, "Configuring invalid input_mode\n");
			return count;
		}

		ret = set_mode_gpios(dev);
		if (ret < 0)
			return ret;
	}

	return count;
}

static ssize_t hed_software_configurable_io_show_input_value(
						struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);
	int ret = 0;
	int val_r = 0, val_p = 0;

	ret = iio_read_channel_raw(&data->adc_channels[0], &val_r);
	if (ret < 0) {
		dev_err(dev, "error getting iiochan raw value: %d\n",
			ret);
		ret = sprintf(buf, "error read raw: %d\n", ret);
		return ret;
	}

	if (data->input_mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD) {
		/* Calculate mV processed value */
		val_p = div_s64((val_r * data->voltage_scale),
				HED_SOFTWARE_CONFIGURABLE_IO_SCALE_MODIFIER);
		dev_dbg(dev, "%s: raw: %d, processed: %d mv\n",
			__func__, val_r, val_p);
	} else if (data->input_mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STB ||
		data->input_mode == HED_SOFTWARE_CONFIGURABLE_IO_MODE_STG) {
		if (data->input_gpio) {
			/* Use GPIO value if available */
			ret = gpiod_get_value(data->input_gpio);
			if (ret < 0) {
				dev_err(dev, "error reading GPIO value\n");
				return val_p;
			} else {
				val_p = ret;
			}
		} else {
			/* Use ADC if GPIO value not available */
			/* Check for < 50% of ADC range for a 0 value */
			if (val_r < (1 << data->adc_resolution) / 2)
				val_p = 0;
			/* Check for >= 50% of ADC range for a 1 value */
			else if (val_r >= (1 << data->adc_resolution) / 2)
				val_p = 1;
		}
		dev_dbg(dev, "%s: raw: %d, processed: %d\n",
			__func__, val_r, val_p);
	}
	ret = sprintf(buf, "%d\n", val_p);

	return ret;
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

/*
 * create dev_attr_input_value structure (device_attribute)
 *    dev_attr_input_value
 *       attr.name = input_value
 *       attr.mode = 644
 *       show  = hed_software_configurable_io_show_input_value
 *       store = N/A
 */
static DEVICE_ATTR(input_value, 0644,
			hed_software_configurable_io_show_input_value, NULL);

static struct attribute *attributes[] = {
	&dev_attr_mode.attr,
	&dev_attr_input_value.attr,
	NULL,
};

static const struct attribute_group attr_group = {
	.attrs	= attributes,
};

static void hed_software_configurable_io_channel_release(void *data)
{
	struct iio_channel *adc_channels = data;

	iio_channel_release_all(adc_channels);
}

/*
 * setup mapping of SEL gpio pins
 */
static int setup_mapping(struct platform_device *pdev, struct hed_software_configurable_io_data *drvdata){
	struct property *prop;
	int gpio_name_count, mode_name_count, err, i;
	char *temp_str = NULL;

	/* Setup Input GPIO if used */
	prop = of_find_property(pdev->dev.of_node, "input-gpio", NULL);
	if (prop) {
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

		err = gpiod_export_link(&pdev->dev,
					"MPU_IN_ADC",
					drvdata->input_gpio);
		if (err)
			return err;
	} else {
		drvdata->input_gpio = NULL;
	}

	/* Setup selection gpios */
	drvdata->sel_gpio_count =
		of_gpio_named_count(pdev->dev.of_node, "selection-gpios");
	if (drvdata->sel_gpio_count < 0) {
		dev_err(&pdev->dev, "Missing selection-gpios: %d\n",
						drvdata->sel_gpio_count);
		return drvdata->sel_gpio_count;
	}

	gpio_name_count = of_property_count_strings(pdev->dev.of_node,
							"selection-gpio-names");
	if (gpio_name_count < 0) {
		dev_err(&pdev->dev,
			"Could not get selection-gpio-names %d\n", i);
		return err;
	}

	if (drvdata->sel_gpio_count != gpio_name_count) {
		dev_err(&pdev->dev,
			"selection-gpios count != selection-gpio-names count\n");
		return -EINVAL;
	}

	drvdata->sel_gpios = devm_kzalloc(&pdev->dev,
			drvdata->sel_gpio_count *
			sizeof(struct hed_software_configurable_io_gpio),
			GFP_KERNEL);
	if (!drvdata->sel_gpios)
		return -ENOMEM;

	if (drvdata->sel_gpio_count >
			HED_SOFTWARE_CONFIGURABLE_IO_MODE_MAX_GPIOS) {
		dev_err(&pdev->dev, "Too many gpios defined\n");
		return -EINVAL;
	}

	for (i = 0; i < drvdata->sel_gpio_count; i++) {
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
		if (err) {
			dev_err(&pdev->dev,
				"Could not get selection-gpio-names %d\n", i);
			return err;
		}

		err = gpiod_export(drvdata->sel_gpios[i].gpio, 0);
		if (err)
			return err;

		err = gpiod_export_link(&pdev->dev, drvdata->sel_gpios[i].name,
						drvdata->sel_gpios[i].gpio);
		if (err)
			return err;
	}

	/* Process selection mappings */
	mode_name_count = of_property_count_strings(pdev->dev.of_node,
						"selection-mapping-names");
	if (mode_name_count < 0) {
		dev_err(&pdev->dev, "Missing selection-mapping-names: %d\n",
						mode_name_count);
		return mode_name_count;
	}

	drvdata->mode_config_count = of_property_count_u32_elems(pdev->dev.of_node,
						"selection-mappings");
	if (drvdata->mode_config_count < 0) {
		dev_err(&pdev->dev, "Missing selection-mapping-names: %d\n",
						drvdata->mode_config_count);
		return drvdata->mode_config_count;
	}

	if (drvdata->mode_config_count != mode_name_count) {
		dev_err(&pdev->dev,
			"selection-mappings count != selection-mapping-names count\n");
		return -EINVAL;
	}

	drvdata->mode_config = devm_kzalloc(&pdev->dev,
		drvdata->mode_config_count * 
		sizeof(struct hed_software_configurable_io_mode_config),
		GFP_KERNEL);
	if (!drvdata->mode_config)
		return -ENOMEM;

	for (i = 0; i < drvdata->mode_config_count; i++) {
		err = of_property_read_string_index(pdev->dev.of_node,
					"selection-mapping-names", i,
					(const char **)&temp_str);
		if (err) {
			dev_err(&pdev->dev,
				"Could not get selection-mapping-names %d\n",
				i);
			return err;
		}
		drvdata->mode_config[i].name = get_mode_from_string(temp_str);
		if (drvdata->mode_config[i].name ==
				HED_SOFTWARE_CONFIGURABLE_IO_MODE_INVALID) {
			dev_err(&pdev->dev,
				"Unsupported selection-mapping-names: %s\n",
				temp_str);
			return -EINVAL;
		}

		err = of_property_read_u32_index(pdev->dev.of_node,
					"selection-mappings", i,
					&(drvdata->mode_config[i].mapping));
		if (err) {
			dev_err(&pdev->dev,
				"Could not get selection-mapping %d\n", i);
			return err;
		}
	}

	return 0;
}

static int hed_software_configurable_io_probe(struct platform_device *pdev)
{
	struct hed_software_configurable_io_data *drvdata;
	int ret, err;
	const char *default_input_mode;
	char *temp_str = NULL;
	int max_voltage = -1;
	s64 processed_scale = -1;
	struct iio_channel *adc_channels = NULL;
	struct iio_chan_spec *adc_channel_spec;
	enum hed_software_configurable_io_mode mode_name;

	drvdata = devm_kzalloc(&pdev->dev,
			       sizeof(struct hed_software_configurable_io_data),
			       GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	/* Setup reading of ADC */
	adc_channels = iio_channel_get_all(&pdev->dev);
	if (IS_ERR(adc_channels)) {
		ret = PTR_ERR(adc_channels);
		if (ret == -ENODEV)
			return -EPROBE_DEFER;
		dev_err(&pdev->dev, "Failed to get all iio channels: %d", ret);
		return ret;
	}

	err = devm_add_action(&pdev->dev,
			hed_software_configurable_io_channel_release,
			adc_channels);
	if (err) {
		iio_channel_release_all(adc_channels);
		dev_err(&pdev->dev,
			"Failed to register iio channel release action");
		return err;
	}

	drvdata->adc_channels = adc_channels;

	/* Setup default input mode value */
	ret = of_property_read_string(pdev->dev.of_node,
					"default-input-mode",
					&default_input_mode);
	if (ret)
		return ret;

	drvdata->default_input_mode = get_mode_from_string(default_input_mode);
	if (drvdata->default_input_mode ==
				HED_SOFTWARE_CONFIGURABLE_IO_MODE_INVALID) {
		dev_err(&pdev->dev, "Invalid default input mode\n");
		return -EINVAL;
	}

	drvdata->input_mode = drvdata->default_input_mode;

	/* Figure out the voltage scale to convert raw ADC to mV */
	ret = of_property_read_u32(pdev->dev.of_node,
					"max-voltage",
					&max_voltage);
	if (ret) {
		dev_err(&pdev->dev, "Missing selection-gpios: %d\n", ret);
		return ret;
	}

	/* voltage_scale = (max_voltage / total ADC steps)
	 * Example: 5657mV / 4095
	 * Multiply max_voltage by HED_SOFTWARE_CONFIGURABLE_IO_SCALE_MODIFIER
	 * to eliminate floating point math in kernel with dividing by the same
	 * when calculating the current mV value later */
	adc_channel_spec = (struct iio_chan_spec *)adc_channels[0].channel;
	drvdata->adc_resolution = adc_channel_spec->scan_type.realbits;
	processed_scale =
		max_voltage * HED_SOFTWARE_CONFIGURABLE_IO_SCALE_MODIFIER;
	drvdata->voltage_scale =
		div_s64(processed_scale, (1 << drvdata->adc_resolution) - 1);

	/* Determine if input mode is selectable */
	drvdata->non_selectable_input =
		of_property_read_bool(pdev->dev.of_node, "non-selectable-input");
	if (drvdata->non_selectable_input != true) {
		drvdata->non_selectable_input = false;
	}

    /* Set initial maping */
	if (drvdata->non_selectable_input == false) {
		ret = setup_mapping(pdev, drvdata);
		if (ret != 0) {
			dev_err(&pdev->dev, "Failed to set mapping\n");
			return ret;
		}
	}

	/* Create sysfs mode file */
	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to create sysfs group\n");
		return ret;
	}

	platform_set_drvdata(pdev, drvdata);

	/* Set initial input mode */
	ret = set_mode_gpios(&pdev->dev);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to set mode selection gpios\n");
		return ret;
	}

	dev_info(&pdev->dev, "Initialized HED Input Controller");

	return 0;
}

static int hed_software_configurable_io_remove(struct platform_device *pdev)
{
	struct hed_software_configurable_io_data *data =
						platform_get_drvdata(pdev);
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &attr_group);

	sysfs_remove_link(&pdev->dev.kobj, "MPU_IN_ADC");
	gpiod_unexport(data->input_gpio);

	for (i = 0; i < data->sel_gpio_count; i++) {
		sysfs_remove_link(&pdev->dev.kobj, data->sel_gpios[i].name);
		gpiod_unexport(data->sel_gpios[i].gpio);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hed_software_configurable_io_suspend(struct device *dev)
{
	/*
	 * set pin to VTD (HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD)
	 *  - place pin in high impedance to minimize current draw
	 */
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);
	int ret = 0;

	dev_dbg(dev, "%s : %d", __func__, data->input_mode);
	data->resume_mode = data->input_mode;
	data->input_mode = HED_SOFTWARE_CONFIGURABLE_IO_MODE_VTD;
	set_mode_gpios(dev);
	if (ret != 0) {
		dev_err(dev, "Failed to set mode selection gpios at suspend\n");
		return ret;
	}
	return 0;
}

static int hed_software_configurable_io_resume(struct device *dev)
{
	struct hed_software_configurable_io_data *data = dev_get_drvdata(dev);
	int ret = 0;

	dev_dbg(dev, "%s : %d", __func__, data->input_mode);
	data->input_mode = data->resume_mode;
	set_mode_gpios(dev);
	if (ret != 0) {
		dev_err(dev, "Failed to set mode selection gpios at resume\n");
		return ret;
	}
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
MODULE_DESCRIPTION("HED software configurable I/O driver");
MODULE_LICENSE("GPL");
