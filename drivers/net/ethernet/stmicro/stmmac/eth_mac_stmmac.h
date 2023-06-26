#ifndef _ETH_MAC_H_
#define _ETH_MAC_H_
/*
 *  eth_mac/eth_mac.h
 *
 *  Copyright (C) 2001 Russell King.
 *
 *  This file is placed under the LGPL.
 *
 */
extern void at24_read_eeprom(char *buf, unsigned int off, size_t count);
int eth_mac_eeprom_stmmac(u8 *eth_mac);
#endif /* _ETH_MAC_H_ */
