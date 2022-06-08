// SPDX-License-Identifier: GPL-2.0-only
/* cm32183 optical sensors driver
 *
 * Copyright (C) 2022 Vishay Capella Microsystems Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/init.h>
#include <linux/cm32183.h>

struct cm32183_chip {
	struct i2c_client *client;
	struct device *dev;
	uint16_t ls_cmd;
	struct mutex als_get_adc_mutex;
	int als_enable;
};

static int cm32183_get_lux(struct cm32183_chip *cm32183, uint16_t reg);

/**
 * cm32183_reg_init() - Initialize CM32183 registers
 * @cm32183:	pointer of struct cm32183.
 *
 * Initialize CM32183 ambient light sensor register to default values.
 *
 * Return: 0 for success; otherwise for error code.
 */
static int cm32183_reg_init(struct cm32183_chip *cm32183)
{
	struct i2c_client *client = cm32183->client;
	s32 ret;

	cm32183->ls_cmd = CM32183_CONF_IT_200MS | CM32183_CONF_CH_EN;

	pr_info("%s shut down CM32183\n", __func__);
	// Shut down CM32183
	cm32183->ls_cmd |= CM32183_CONF_SD;
	ret = i2c_smbus_write_word_data(client, CM32183_CONF, cm32183->ls_cmd);
	if (ret < 0)
		return ret;

	// Enable CM32183
	cm32183->ls_cmd &= CM32183_CONF_SD_MASK;
	ret = i2c_smbus_write_word_data(client, CM32183_CONF, cm32183->ls_cmd);
	pr_info("%s enable CM32183\n", __func__);
	if (ret < 0)
		return ret;

	msleep(300);
	// Get initial ALS light data
	ret = cm32183_get_lux(cm32183, CM32183_ALS_DATA);
	if (ret < 0) {
		pr_err("%s: get lux for GREEN fail\n", __func__);
		return -EIO;
	}
	pr_info("CM32183_ALS_DATA %x\n", ret);

	// Get initial WHITE light data
	ret = cm32183_get_lux(cm32183, CM32183_W_DATA);
	if (ret < 0) {
		pr_err("%s: get lux for Blue fail\n", __func__);
		return -EIO;
	}
	pr_info("CM32183_W_DATA %x\n", ret);
	cm32183->als_enable = 1;

	return 0;
}

/**
 * cm32183_get_lux() - report current lux value
 * @cm32183:	pointer of struct cm32183.
 *
 * Convert sensor raw data to lux.  It depends on integration
 * time and calibscale variable.
 *
 * Return: Positive value is lux, otherwise is error code.
 */
static int cm32183_get_lux(struct cm32183_chip *cm32183, uint16_t reg)
{
	struct i2c_client *client = cm32183->client;
	int ret;

	mutex_lock(&cm32183->als_get_adc_mutex);
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		mutex_unlock(&cm32183->als_get_adc_mutex);
		return -EINVAL;
	}

	mutex_unlock(&cm32183->als_get_adc_mutex);
	return ret;
}

static int cm32183_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct cm32183_chip *cm32183 = iio_priv(indio_dev);
	int value;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->channel2) {
		case IIO_MOD_LIGHT_GREEN:
			value = cm32183_get_lux(cm32183, CM32183_ALS_DATA);
			if (value < 0)
				return value;
			*val = value;
			return IIO_VAL_INT;
		case IIO_MOD_LIGHT_IR:
			value = cm32183_get_lux(cm32183, CM32183_W_DATA);
			if (value < 0)
				return value;
			*val = value;
			return IIO_VAL_INT;
		return -EINVAL;
		}
	}

	return -EINVAL;
}

static ssize_t mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cm32183_chip *cm32183 = iio_priv(dev_to_iio_dev(dev));
	struct i2c_client *client = cm32183->client;
	s32 ret;

	int state;

	if ((kstrtoint(buf, 10, &state) < 0 || (state > 1)))
		return -EINVAL;
	pr_info("%s state is %d\n", __func__, state);

	if (state == 1)
		cm32183->ls_cmd &= CM32183_CONF_SD_MASK;
	else
		cm32183->ls_cmd |= CM32183_CONF_SD;

	ret = i2c_smbus_write_word_data(client, CM32183_CONF, cm32183->ls_cmd);
	if (ret < 0)
		return 0;

	if (state == 1)
		msleep(200); /*wait for 200 ms for the first report*/

	cm32183->als_enable = state;
	return count;

}

static ssize_t mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm32183_chip *cm32183 = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", cm32183->als_enable);
}

static ssize_t conf_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm32183_chip *cm32183 = iio_priv(dev_to_iio_dev(dev));
	struct i2c_client *client = cm32183->client;
	s32 ret;

	ret = i2c_smbus_read_word_data(client, CM32183_CONF);
	if (ret < 0) {
		pr_err("%s: CM32183_I2C_Read_Word for CM32183_CONF fail\n", __func__);
		return -EIO;
	}
	pr_info("CM32183 CM32183_CONF %x\n", ret);

	return sprintf(buf, "%x\n", ret);
}

static ssize_t conf_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cm32183_chip *cm32183 = iio_priv(dev_to_iio_dev(dev));
	struct i2c_client *client = cm32183->client;
	int value = 0;
	s32 ret;

	if (kstrtoint(buf, 16, &value) < 0)
		return -EINVAL;

	pr_info("%s CM32183_CONF value is %x\n", __func__, value);

	ret = i2c_smbus_write_word_data(client, CM32183_CONF, value);
	if (ret < 0)
		return 0;

	return count;
}

static const struct iio_chan_spec cm32183_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_PROCESSED),
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_GREEN,

	},
	{
		.type = IIO_LIGHT,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_PROCESSED),
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
	},
};

static IIO_DEVICE_ATTR(mode, 0644, mode_show, mode_store, 0);
static IIO_DEVICE_ATTR(conf, 0644, conf_show, conf_store, 0);

static struct attribute *cm32183_attributes[] = {
	&iio_dev_attr_mode.dev_attr.attr,
	&iio_dev_attr_conf.dev_attr.attr,
	NULL,
};

static const struct attribute_group cm32183_attribute_group = {
	.attrs = cm32183_attributes
};


static const struct iio_info cm32183_info = {
	.read_raw		= &cm32183_read_raw,
	.attrs			= &cm32183_attribute_group,
};

static int cm32183_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cm32183_chip *cm32183;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*cm32183));
	if (!indio_dev)
		return -ENOMEM;

	cm32183 = iio_priv(indio_dev);
	cm32183->client = client;
	cm32183->dev = dev;

	indio_dev->channels = cm32183_channels;
	indio_dev->num_channels = ARRAY_SIZE(cm32183_channels);
	indio_dev->info = &cm32183_info;
	indio_dev->name = CM32183_I2C_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = cm32183_reg_init(cm32183);
	if (ret) {
		dev_err(dev, "%s: register init failed\n", __func__);
		return ret;
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "%s: regist device failed\n", __func__);
		return ret;
	}

	mutex_init(&cm32183->als_get_adc_mutex);
	return 0;
}

static const struct i2c_device_id cm32183_i2c_id[] = {
	{CM32183_I2C_NAME, 0},
	{}
};

static const struct of_device_id cm32183_of_match[] = {
	{ .compatible = "capella,cm32183",},
	{ },
};

MODULE_DEVICE_TABLE(of, cm32183_of_match);

static struct i2c_driver cm32183_driver = {
	.id_table = cm32183_i2c_id,
	.driver = {
		.name	= CM32183_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cm32183_of_match),
	},
	.probe_new	= cm32183_probe,
};

static int __init cm32183_init(void)
{
	return i2c_add_driver(&cm32183_driver);
}

static void __exit cm32183_exit(void)
{
	i2c_del_driver(&cm32183_driver);
}

module_init(cm32183_init);
module_exit(cm32183_exit);

MODULE_AUTHOR("Joy Huang <joy_huang@asus.com>");
MODULE_DESCRIPTION("CM32183 ambient light sensor driver");
MODULE_LICENSE("GPL");
