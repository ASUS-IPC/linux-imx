/*
 * Copyright 2016, Tobias Doerffel <tobias.doerffel@xxxxxxxxxxxxxx>
 *
 * This program is free software; you uart redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/leds.h>
#include <linux/serial_leds.h>
#include <linux/delay.h>
static unsigned long led_delay = 50;
module_param(led_delay, ulong, 0644);
MODULE_PARM_DESC(led_delay,
		"blink delay time for activity leds (msecs, default: 50).");
DEFINE_LED_TRIGGER(ledtrig_gps_led_r);


/* Trigger a LED event in response to a UART device event */
void uart_led_event(struct uart_port *port, enum uart_led_event event)
{
	switch (event) {
	case UART_LED_EVENT_TX:
		if (led_delay) {
			led_trigger_blink_oneshot(port->tx_led_trig,
						  &led_delay, &led_delay, 1);
			led_trigger_blink_oneshot(port->rxtx_led_trig,
						  &led_delay, &led_delay, 1);
		}
		break;
	case UART_LED_EVENT_RX:
		if (led_delay) {
		    
			led_trigger_blink_oneshot(port->rx_led_trig,
						  &led_delay, &led_delay, 1);
			led_trigger_blink_oneshot(port->rxtx_led_trig,
						  &led_delay, &led_delay, 1);
		}
		break;
	}
}
EXPORT_SYMBOL_GPL(uart_led_event);

void uart_led_register(struct uart_driver *drv, struct uart_port *port)
{
	snprintf(port->tx_led_trig_name, sizeof(port->tx_led_trig_name),
		 "%s%d-tx", drv->dev_name, drv->tty_driver->name_base + port->line);
	snprintf(port->rx_led_trig_name, sizeof(port->rx_led_trig_name),
		 "%s%d-rx", drv->dev_name, drv->tty_driver->name_base + port->line);
	snprintf(port->rxtx_led_trig_name, sizeof(port->rxtx_led_trig_name),
		 "%s%d-rxtx", drv->dev_name, drv->tty_driver->name_base + port->line);

	led_trigger_register_simple(port->tx_led_trig_name, &port->tx_led_trig);
	led_trigger_register_simple(port->rx_led_trig_name, &port->rx_led_trig);
	led_trigger_register_simple(port->rxtx_led_trig_name, &port->rxtx_led_trig);
}
EXPORT_SYMBOL_GPL(uart_led_register);

void uart_led_unregister(struct uart_port *port)
{
	led_trigger_unregister_simple(port->tx_led_trig);
	led_trigger_unregister_simple(port->rx_led_trig);
	led_trigger_unregister_simple(port->rxtx_led_trig);
}
EXPORT_SYMBOL_GPL(uart_led_unregister);
