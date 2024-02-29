/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Luca Hsu <luca_hsu@ilitek.com>
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 *
 */

#include "ilitek_ts.h"
#include "ilitek_common.h"

static int ilitek_touch_driver_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	if (!client) {
		tp_err("i2c client is NULL\n");
		return -ENODEV;
	}

	tp_msg("ILITEK client->addr: 0x%x, client->irq: %d\n",
		client->addr, client->irq);

	return ilitek_main_probe(client, &client->dev);
}

static int ilitek_touch_driver_remove(struct i2c_client *client)
{
	return ilitek_main_remove(client);
}

#ifdef CONFIG_OF
static struct of_device_id ilitek_touch_match_table[] = {
	{.compatible = "tchip,ilitek",},
	{},
};
MODULE_DEVICE_TABLE(of, ilitek_touch_match_table);
#endif

static const struct i2c_device_id ilitek_touch_device_id[] = {
	{ILITEK_TS_NAME, 0},
	{},			/* should not omitted */
};
MODULE_DEVICE_TABLE(i2c, ilitek_touch_device_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id ilitekts_acpi_id[] = {
	{"ILTK0001", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, ilitekts_acpi_id);
#endif


#ifndef ILITEK_REGISTER_SUSPEND_RESUME
static int __maybe_unused ilitek_ts_suspend(struct device *dev)
{
	ilitek_suspend();
	return 0;
}

static int __maybe_unused ilitek_ts_resume(struct device *dev)
{
	ilitek_resume();
	return 0;
}

static SIMPLE_DEV_PM_OPS(ilitek_pm_ops, ilitek_ts_suspend, ilitek_ts_resume);
#endif

static struct i2c_driver ilitek_touch_device_driver = {
	.driver = {
		.name = ILITEK_TS_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ilitek_touch_match_table),
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(ilitekts_acpi_id),
#endif

#ifndef ILITEK_REGISTER_SUSPEND_RESUME
		.pm = &ilitek_pm_ops,
#endif
	},

	.probe = ilitek_touch_driver_probe,
	.remove = ilitek_touch_driver_remove,
	.id_table = ilitek_touch_device_id,

};
module_i2c_driver(ilitek_touch_device_driver);

MODULE_AUTHOR("ILITEK");
MODULE_DESCRIPTION("ILITEK I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
