/*
 * Copyright 2016, Tobias Doerffel <tobias.doerffel@xxxxxxxxxxxxxx>
 *
 * This program is free software; you uart redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SERIAL_LEDS_H
#define _SERIAL_LEDS_H

#include <linux/serial_core.h>
#include <linux/leds.h>

enum uart_led_event {
	UART_LED_EVENT_TX,
	UART_LED_EVENT_RX,
};

#ifdef CONFIG_SERIAL_LEDS

void uart_led_event(struct uart_port *port, enum uart_led_event event);
void uart_led_register(struct uart_driver *drv, struct uart_port *port);
void uart_led_unregister(struct uart_port *port);

#else

static inline void uart_led_event(struct uart_port *port, enum uart_led_event event)
{
}
static inline void uart_led_register(struct uart_driver *drv, struct uart_port *port)
{
}
static inline void uart_led_unregister(struct uart_port *port)
{
}

#endif

#endif /* !_SERIAL_LEDS_H */
