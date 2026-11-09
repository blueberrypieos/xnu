/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * The Ethernet PHY behind a MAC's MDIO controller (RPiPHY.cpp, for
 * RPiGENET and RPiGEM): the MAC supplies clause 22 reads (-1: failed) and
 * writes of PHY registers. Private to this kext.
 */

#ifndef _RPI_PHY_H
#define _RPI_PHY_H

#include <IOKit/IOService.h>

struct RPiMDIO {
	void *context;
	int phy;
	int (*read)(void *context, int phy, int reg);
	bool (*write)(void *context, int phy, int reg, uint16_t value);
};
/* phy-mode (or defaultMode): whether the PHY adds the RX and TX RGMII delays. */
const char *RPiPHYMode(IOService *provider, const char *defaultMode, bool *rxDelay, bool *txDelay);
/* The PHY at address first, else the first that answers (sets mdio->phy). */
bool RPiPHYFind(RPiMDIO *mdio, int first, const char *who);
/* RGMII delays (Broadcom BCM54xx shadow registers), then autonegotiation. */
void RPiPHYConfigure(const RPiMDIO *mdio, bool rxDelay, bool txDelay);
/* Whether the link is up, and at what speed and duplex. */
bool RPiPHYLink(const RPiMDIO *mdio, uint32_t *mbps, bool *full);

#endif /* _RPI_PHY_H */
