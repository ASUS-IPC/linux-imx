/*
 * Copyright (C) 2010 ROCKCHIP, Inc.
 * Author: roger_chen <cz@rock-chips.com>
 *
 * This program is the virtual flash device
 * used to store bd_addr or MAC
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include "eth_mac_stmmac.h"

#if 1
#define DBG(x...)   printk("eth_mac_stmmac:" x)
#else
#define DBG(x...)
#endif

#define VERSION "0.1"

int eth_mac_eeprom_stmmac(u8 *eth_mac_stmmac)
{
	int i;
	memset(eth_mac_stmmac, 0, 6);
	printk("Read the Ethernet MAC address from EEPROM (STMMAC):");
	at24_read_eeprom(eth_mac_stmmac, 6, 6);
	for(i=0; i<5; i++)
		printk("%2.2x:", eth_mac_stmmac[i]);
	printk("%2.2x\n", eth_mac_stmmac[i]);

	return 0;
}
