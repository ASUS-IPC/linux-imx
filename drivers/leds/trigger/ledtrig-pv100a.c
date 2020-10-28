/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>

DEFINE_LED_TRIGGER(ledtrig_wifi_led_r);
DEFINE_LED_TRIGGER(ledtrig_wifi_led_g);
DEFINE_LED_TRIGGER(ledtrig_lte_led_r);
DEFINE_LED_TRIGGER(ledtrig_lte_led_g);
DEFINE_LED_TRIGGER(ledtrig_gps_led_r);
DEFINE_LED_TRIGGER(ledtrig_gps_led_g);

void ledtrig_wifi_led_r_ctrl(bool on)
{
	enum led_brightness brt = on ? LED_FULL : LED_OFF;

	led_trigger_event(ledtrig_wifi_led_r, brt);
}
EXPORT_SYMBOL_GPL(ledtrig_wifi_led_r_ctrl);

void ledtrig_wifi_led_g_ctrl(bool on)
{
	enum led_brightness brt = on ? LED_FULL : LED_OFF;

	led_trigger_event(ledtrig_wifi_led_g, brt);
}
EXPORT_SYMBOL_GPL(ledtrig_wifi_led_g_ctrl);

void ledtrig_lte_led_r_ctrl(bool on)
{
	enum led_brightness brt = on ? LED_FULL : LED_OFF;

	led_trigger_event(ledtrig_lte_led_r, brt);
}
EXPORT_SYMBOL_GPL(ledtrig_lte_led_r_ctrl);

void ledtrig_lte_led_g_ctrl(bool on)
{
	enum led_brightness brt = on ? LED_FULL : LED_OFF;

	led_trigger_event(ledtrig_lte_led_g, brt);
}
EXPORT_SYMBOL_GPL(ledtrig_lte_led_g_ctrl);

void ledtrig_gps_led_r_ctrl(bool on)
{
	enum led_brightness brt = on ? LED_FULL : LED_OFF;

	led_trigger_event(ledtrig_gps_led_r, brt);
}
EXPORT_SYMBOL_GPL(ledtrig_gps_led_r_ctrl);

void ledtrig_gps_led_g_ctrl(bool on)
{
	enum led_brightness brt = on ? LED_FULL : LED_OFF;

	led_trigger_event(ledtrig_gps_led_g, brt);
}
EXPORT_SYMBOL_GPL(ledtrig_gps_led_g_ctrl);

static int __init ledtrig_pv100a_init(void)
{
	led_trigger_register_simple("wifi_led_r", &ledtrig_wifi_led_r);
	led_trigger_register_simple("wifi_led_g", &ledtrig_wifi_led_g);
	led_trigger_register_simple("lte_led_r", &ledtrig_lte_led_r);
	led_trigger_register_simple("lte_led_g", &ledtrig_lte_led_g);
	led_trigger_register_simple("gps_led_r", &ledtrig_gps_led_r);
	led_trigger_register_simple("gps_led_g", &ledtrig_gps_led_g);

	return 0;
}
module_init(ledtrig_pv100a_init);

static void __exit ledtrig_pv100a_exit(void)
{
	led_trigger_unregister_simple(ledtrig_wifi_led_r);
	led_trigger_unregister_simple(ledtrig_wifi_led_g);
	led_trigger_unregister_simple(ledtrig_lte_led_r);
	led_trigger_unregister_simple(ledtrig_lte_led_g);
	led_trigger_unregister_simple(ledtrig_gps_led_r);
	led_trigger_unregister_simple(ledtrig_gps_led_g);
}
module_exit(ledtrig_pv100a_exit);

MODULE_DESCRIPTION("LED Trigger for PV100A leds");
MODULE_AUTHOR("Tzuhsuan Chen");
MODULE_LICENSE("GPL");
