// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADS7128 - Texas Instruments Analog-to-Digital Converter
 *
 * Copyright (c) 2016, Intel Corporation.
 *
 * IIO driver for ADS7128 ADC 8-channel 12-bit I2C slave address:
 *	* 0x14 - ADDR connected R1 100Kohm and R2 DNP
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/property.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/adc/ads7128.h>

#define ADS7128_DRV_NAME "ads7128"
#define ADS7128_CHANNELS 8

/* ADS7128 opcodes */
#define ADS7128_OPCODE_READ		0x10
#define ADS7128_OPCODE_WRITE	0x08

/* AVDD (VREF) operating range */
#define ADS7128_VREF_MV_MIN	2350
#define ADS7128_VREF_MV_MAX	5500

enum ads7128_channels {
	ADS7128_AIN0 = 0,
	ADS7128_AIN1,
	ADS7128_AIN2,
	ADS7128_AIN3,
	ADS7128_AIN4,
	ADS7128_AIN5,
	ADS7128_AIN6,
	ADS7128_AIN7,
};

/* ADS7128 data */
struct ads7128_data {
	struct i2c_client *client;
	struct mutex lock;
	struct regulator *vref;
	unsigned int lsb;
};

s32 ads7128_read_reg(const struct ads7128_data *data, u8 reg)
{
	int ret;

	ret = i2c_smbus_write_byte_data(data->client, ADS7128_OPCODE_READ, reg);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte(data->client);
	return ret;
}

s32 ads7128_write_reg(const struct ads7128_data *data, u8 reg, u8 value)
{
	int ret;
	u16 svalue = reg | (value << 8);

	ret = i2c_smbus_write_word_data(data->client, ADS7128_OPCODE_WRITE, svalue);
	return ret;
}

static int ads7128_read_value(struct ads7128_data *data, int *val)
{
	int ret;
	s32 temp;
	u8 rxbuf[2];
	int value;
	int channel_num;

	ret = i2c_master_recv(data->client, rxbuf, 2);
	if (ret < 0)
		return ret;

	dev_dbg(&data->client->dev, "%s ret %d, rxbuf[0]: 0x%x, rxbuf[1]: 0x%x\n",
		__FUNCTION__, ret, rxbuf[0], rxbuf[1]);

	temp = (rxbuf[1] | rxbuf[0] << 8);
	dev_dbg(&data->client->dev, "%s temp: 0x%x\n", __FUNCTION__, temp);

	channel_num = temp & 0xf;
	value = le16_to_cpu((temp >> 4) & 0xfff);

	dev_dbg(&data->client->dev, "ADC channel_num: %d, value: 0x%x %d\n",
		channel_num, value, value);

	*val = value;

	dev_dbg(&data->client->dev, "%s val: %d\n", __FUNCTION__, *val);
	dev_dbg(&data->client->dev, "%s voltage : %d mV\n",
		__FUNCTION__, DIV_ROUND_CLOSEST(value * data->lsb, 1000));
	return 0;
}

static int ads7128_read_raw(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    int *val, int *val2, long mask)
{
	struct ads7128_data *data = iio_priv(indio_dev);
	int ret;

	dev_dbg(&data->client->dev, "%s select channel num: %d", __FUNCTION__, chan->channel);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->lock);
		/* Select ADC channel and read the data */
		ret = ads7128_write_reg(data, MANUAL_CH_SEL_ADDRESS, chan->channel);
		if (ret < 0) {
			dev_err(&data->client->dev, "Failed to select channel %d\n", chan->channel);
			mutex_unlock(&data->lock);
			return ret;
		} else {
			ret = ads7128_read_value(data, val);
			if (ret < 0) {
				mutex_unlock(&data->lock);
				return ret;
			}
		}
		mutex_unlock(&data->lock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(data->vref);
		if (ret < 0) {
			dev_err(&data->client->dev, "failed to get voltage\n");
			return ret;
		}

		*val = ret / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ads7128_iio_info = {
	.read_raw = ads7128_read_raw,
};

#define ADS7128_CHANNEL(_index, _id, _res) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = _index,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.datasheet_name = _id,					\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = _res,				\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
}

static const struct iio_chan_spec ads7128_iio_channels[] = {
	ADS7128_CHANNEL(0, "adc1", 12),
	ADS7128_CHANNEL(1, "adc2", 12),
	ADS7128_CHANNEL(2, "adc3", 12),
	ADS7128_CHANNEL(3, "adc4", 12),
	ADS7128_CHANNEL(4, "adc5", 12),
	ADS7128_CHANNEL(5, "CURSNS1", 12),
	ADS7128_CHANNEL(6, "CURSNS2", 12),
	ADS7128_CHANNEL(7, "battery_P_ADC", 12),
};

static int ads7128_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev = NULL;
	struct ads7128_data *data = NULL;
	int ret;
	int vref_uv = 0;
	int vref_mv = 0;

	dev_info(dev, "%s +++\n", __FUNCTION__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C |
				I2C_FUNC_SMBUS_WRITE_BYTE))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	mutex_init(&data->lock);

	/* Get AVDD (Vref) */
	data->vref = devm_regulator_get(dev, "AVDD");
	if (IS_ERR(data->vref)) {
		dev_err(dev, "Failed to get AVDD (Vref)\n");
		return PTR_ERR(data->vref);
	} else {
		ret = regulator_enable(data->vref);
		if (ret) {
			dev_err(dev, "Failed to enable AVDD supply: %d\n", ret);
			return ret;
		}

		vref_uv = regulator_get_voltage(data->vref);
		vref_mv = DIV_ROUND_CLOSEST(vref_uv, 1000);
	}

	if (vref_mv < ADS7128_VREF_MV_MIN ||
			vref_mv > ADS7128_VREF_MV_MAX) {
		dev_err(dev, "AVDD (Vref) is not in range\n");
		return -EINVAL;
	}

	/* Calculate LSB */
	vref_mv = clamp_val(vref_mv, ADS7128_VREF_MV_MIN, ADS7128_VREF_MV_MAX);
	dev_info(dev, "%s vref_mv = %d\n", __FUNCTION__, vref_mv);
	data->lsb = DIV_ROUND_CLOSEST(vref_mv * 1000, 4096);
	dev_info(dev, "%s LSB = %d\n", __FUNCTION__, data->lsb);

	mutex_lock(&data->lock);

	/* Software reset */
	ret = ads7128_write_reg(data, GENERAL_CFG_ADDRESS, RST_START);
	if (ret < 0) {
		dev_err(dev, "Failed to reset\n");
		return ret;
	}

	/* Get system status */
	ret = ads7128_read_reg(data, SYSTEM_STATUS_ADDRESS);
	if (ret < 0)	{
		dev_err(dev, "Failed to read system status\n");
		return ret;
	}

	dev_info(dev, "ads7128 status 0x%x\n", ret);

	/* Set data configuration */
	ret = ads7128_write_reg(data, DATA_CFG_ADDRESS, APPEND_STATUS_ID);
	if (ret < 0) {
		dev_err(dev, "Failed to set data configuration\n");
		return ret;
	}

	/* Manual mode, conversions are initiated by host */
	ret = ads7128_write_reg(data, OPMODE_CFG_ADDRESS, OPMODE_CFG_DEFAULT);
	if (ret < 0) {
		dev_err(dev, "Failed to set opmode\n");
		return ret;
	}

	/* Configure all channels as analog input */
	ret = ads7128_write_reg(data, PIN_CFG_ADDRESS, PIN_CFG_DEFAULT);
	if (ret < 0) {
		dev_err(dev, "Failed to set pin configuration\n");
		return ret;
	}

	mutex_unlock(&data->lock);

	indio_dev->name = ADS7128_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ads7128_iio_info;
	indio_dev->channels = ads7128_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads7128_iio_channels);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register IIO device\n");
		return ret;
	}

	dev_info(dev, "%s ---\n", __FUNCTION__);
	return 0;
}

static int ads7128_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct of_device_id __maybe_unused ads7128_of_match[] = {
	{
		.compatible = "ti,ads7128",
	}
};
MODULE_DEVICE_TABLE(of, ads7128_of_match);

static struct i2c_driver ads7128_driver = {
	.driver = {
		.name = ADS7128_DRV_NAME,
		.of_match_table = of_match_ptr(ads7128_of_match),
	},
	.probe = ads7128_probe,
	.remove = ads7128_remove,
};

module_i2c_driver(ads7128_driver);

MODULE_AUTHOR("Pomelo Hsieh <pomelo_hsieh@asus.com>");
MODULE_DESCRIPTION("Texas Instruments ADS7128 ADC driver");
MODULE_LICENSE("GPL v2");
