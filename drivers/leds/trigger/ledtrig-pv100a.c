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

DEFINE_LED_TRIGGER(ledtrig_wifi_r);
DEFINE_LED_TRIGGER(ledtrig_wifi_g);
DEFINE_LED_TRIGGER(ledtrig_lte_r);
DEFINE_LED_TRIGGER(ledtrig_lte_g);
DEFINE_LED_TRIGGER(ledtrig_gps_r);
DEFINE_LED_TRIGGER(ledtrig_gps_g);

void ledtrig_led_func_ctrl(int func, bool light, bool on)
{
	enum led_brightness brt = on ? LED_FULL : LED_OFF;

	switch (func) {
		case LED_FUNC_WIFI:
			if (light == LED_LIGHT_RED)
				led_trigger_event(ledtrig_wifi_r, brt);
			else if (light == LED_LIGHT_GREEN)
				led_trigger_event(ledtrig_wifi_g, brt);
			break;
		case LED_FUNC_GPS:
			if (light == LED_LIGHT_RED)
				led_trigger_event(ledtrig_gps_r, brt);
			else if (light == LED_LIGHT_GREEN)
				led_trigger_event(ledtrig_gps_g, brt);
			break;
		case LED_FUNC_LTE:
			if (light == LED_LIGHT_RED)
				led_trigger_event(ledtrig_lte_r, brt);
			else if (light == LED_LIGHT_GREEN)
				led_trigger_event(ledtrig_lte_g, brt);
			break;
	}
}
EXPORT_SYMBOL_GPL(ledtrig_led_func_ctrl);

void ledtrig_led_func_blink(int func, bool light, unsigned long delay_on, unsigned long delay_off)
{
	switch (func) {
		case LED_FUNC_WIFI:
			if (light == LED_LIGHT_RED)
				led_trigger_blink(ledtrig_wifi_r, &delay_on, &delay_off);
			else if (light == LED_LIGHT_GREEN)
				led_trigger_blink(ledtrig_wifi_g, &delay_on, &delay_off);
			break;
		case LED_FUNC_GPS:
			if (light == LED_LIGHT_RED)
				led_trigger_blink(ledtrig_gps_r, &delay_on, &delay_off);
			else if (light == LED_LIGHT_GREEN)
				led_trigger_blink(ledtrig_gps_g, &delay_on, &delay_off);
			break;
		case LED_FUNC_LTE:
			if (light == LED_LIGHT_RED)
				led_trigger_blink(ledtrig_lte_r, &delay_on, &delay_off);
			else if (light == LED_LIGHT_GREEN)
				led_trigger_blink(ledtrig_lte_g, &delay_on, &delay_off);
			break;
	}
}
EXPORT_SYMBOL_GPL(ledtrig_led_func_blink);

void ledtrig_led_func_blink_oneshot(int func, bool light, unsigned long delay_on, unsigned long delay_off, int invert)
{
	switch (func) {
		case LED_FUNC_WIFI:
			if (light == LED_LIGHT_RED)
				led_trigger_blink_oneshot(ledtrig_wifi_r, &delay_on, &delay_off, invert);
			else if (light == LED_LIGHT_GREEN)
				led_trigger_blink_oneshot(ledtrig_wifi_g, &delay_on, &delay_off, invert);
			break;
		case LED_FUNC_GPS:
			if (light == LED_LIGHT_RED)
				led_trigger_blink_oneshot(ledtrig_gps_r, &delay_on, &delay_off, invert);
			else if (light == LED_LIGHT_GREEN)
				led_trigger_blink_oneshot(ledtrig_gps_g, &delay_on, &delay_off, invert);
			break;
		case LED_FUNC_LTE:
			if (light == LED_LIGHT_RED)
				led_trigger_blink_oneshot(ledtrig_lte_r, &delay_on, &delay_off, invert);
			else if (light == LED_LIGHT_GREEN)
				led_trigger_blink_oneshot(ledtrig_lte_g, &delay_on, &delay_off, invert);
			break;
	}
}
EXPORT_SYMBOL_GPL(ledtrig_led_func_blink_oneshot);

static int __init ledtrig_pv100a_init(void)
{
	led_trigger_register_simple("wifi-r", &ledtrig_wifi_r);
	led_trigger_register_simple("wifi-g", &ledtrig_wifi_g);
	led_trigger_register_simple("lte-r", &ledtrig_lte_r);
	led_trigger_register_simple("lte-g", &ledtrig_lte_g);
	led_trigger_register_simple("gps-r", &ledtrig_gps_r);
	led_trigger_register_simple("gps-g", &ledtrig_gps_g);

	return 0;
}
module_init(ledtrig_pv100a_init);

static void __exit ledtrig_pv100a_exit(void)
{
	led_trigger_unregister_simple(ledtrig_wifi_r);
	led_trigger_unregister_simple(ledtrig_wifi_g);
	led_trigger_unregister_simple(ledtrig_lte_r);
	led_trigger_unregister_simple(ledtrig_lte_g);
	led_trigger_unregister_simple(ledtrig_gps_r);
	led_trigger_unregister_simple(ledtrig_gps_g);
}
module_exit(ledtrig_pv100a_exit);

MODULE_DESCRIPTION("LED Trigger for PV100A leds");
MODULE_AUTHOR("Tzuhsuan Chen");
MODULE_LICENSE("GPL");
