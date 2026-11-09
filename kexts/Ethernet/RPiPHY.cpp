/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * The Ethernet PHY the Raspberry Pi 4's GENET and the Pi 5's RP1 GEM both
 * have (a Broadcom BCM54213PE on RGMII), reached through each MAC's own
 * MDIO controller: finding it, its RGMII clock delays (Linux's
 * bcm54xx_config_clock_delay), autonegotiation up to 1000BASE-T, and the
 * link it settled on (see RPiPHY.h).
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>

#include "RPiPHY.h"

/* IEEE 802.3 clause 22, plus the Broadcom shadow registers. */
enum {
	MII_BMCR = 0, MII_BMSR = 1, MII_PHYSID1 = 2, MII_PHYSID2 = 3, MII_ADVERTISE = 4,
	MII_LPA = 5, MII_CTRL1000 = 9, MII_STAT1000 = 10,
	MII_BCM54XX_AUX_CTL = 0x18, MII_BCM54XX_SHD = 0x1C,

	BMCR_ANENABLE = 0x1000, BMCR_ANRESTART = 0x0200,
	BMSR_LSTATUS = 0x0004, BMSR_ANEGCOMPLETE = 0x0020,
	ADVERTISE_ALL = 0x01E1,         /* 10/100, half and full, CSMA */
	ADVERTISE_1000 = 0x0300,        /* 1000BASE-T half and full */
	LPA_1000FULL = 0x0800, LPA_1000HALF = 0x0400,
	ADV_1000FULL = 0x0200, ADV_1000HALF = 0x0100,
	LPA_100FULL = 0x0100, LPA_100HALF = 0x0080, LPA_10FULL = 0x0040,

	AUXCTL_SHDWSEL_MISC = 0x07,
	AUXCTL_SHDWSEL_MASK = 0x0007,
	AUXCTL_SHDWSEL_READ_SHIFT = 12,
	AUXCTL_MISC_WREN = 0x8000,
	AUXCTL_MISC_RGMII_SKEW_EN = 0x0100,
	SHD_WRITE = 0x8000,
	SHD_CLK_CTL = 0x3,
	SHD_CLK_CTL_GTXCLK_EN = 1U << 9,
};

static int
rd(const RPiMDIO *m, int reg)
{
	return m->read(m->context, m->phy, reg);
}

static void
wr(const RPiMDIO *m, int reg, uint16_t val)
{
	m->write(m->context, m->phy, reg, val);
}

const char *
RPiPHYMode(IOService *provider, const char *defaultMode, bool *rxDelay, bool *txDelay)
{
	OSData *mode = OSDynamicCast(OSData, provider->getProperty("phy-mode"));
	const char *name = mode ? (const char *)mode->getBytesNoCopy() : defaultMode;
	if (mode != NULL && strnlen(name, mode->getLength()) == mode->getLength()) {
		name = defaultMode;     /* not a string */
	}
	*rxDelay = !strcmp(name, "rgmii-rxid") || !strcmp(name, "rgmii-id");
	*txDelay = !strcmp(name, "rgmii-txid") || !strcmp(name, "rgmii-id");
	return name;
}

bool
RPiPHYFind(RPiMDIO *m, int first, const char *who)
{
	for (int i = -1; i < 32; i++) {
		int addr = i < 0 ? first : i;
		if (i == first) {
			continue;
		}
		m->phy = addr;
		int id1 = rd(m, MII_PHYSID1), id2 = rd(m, MII_PHYSID2);
		if (id1 > 0 && id1 != 0xFFFF && id2 >= 0 && id2 != 0xFFFF) {
			IOLog("%s: PHY %04x:%04x at MDIO address %d\n", who, id1, id2, addr);
			return true;
		}
	}
	return false;
}

void
RPiPHYConfigure(const RPiMDIO *m, bool rxDelay, bool txDelay)
{
	wr(m, MII_BCM54XX_AUX_CTL, AUXCTL_SHDWSEL_MASK | AUXCTL_SHDWSEL_MISC << AUXCTL_SHDWSEL_READ_SHIFT);
	int misc = rd(m, MII_BCM54XX_AUX_CTL);
	if (misc >= 0) {
		misc |= AUXCTL_MISC_WREN;
		if (rxDelay) {
			misc |= AUXCTL_MISC_RGMII_SKEW_EN;
		} else {
			misc &= ~AUXCTL_MISC_RGMII_SKEW_EN;
		}
		wr(m, MII_BCM54XX_AUX_CTL, (uint16_t)(AUXCTL_SHDWSEL_MISC | misc));
	}
	wr(m, MII_BCM54XX_SHD, (uint16_t)(SHD_CLK_CTL << 10));
	int clk = rd(m, MII_BCM54XX_SHD);
	if (clk >= 0) {
		clk &= 0x3FF;
		if (txDelay) {
			clk |= SHD_CLK_CTL_GTXCLK_EN;
		} else {
			clk &= ~SHD_CLK_CTL_GTXCLK_EN;
		}
		wr(m, MII_BCM54XX_SHD, (uint16_t)(SHD_WRITE | SHD_CLK_CTL << 10 | clk));
	}

	int adv = rd(m, MII_ADVERTISE);
	wr(m, MII_ADVERTISE, (uint16_t)((adv >= 0 ? adv & ~0x0FE0 : 0) | ADVERTISE_ALL));
	int ctrl1000 = rd(m, MII_CTRL1000);
	wr(m, MII_CTRL1000, (uint16_t)((ctrl1000 >= 0 ? ctrl1000 & ~0x0300 : 0) | ADVERTISE_1000));
	int bmcr = rd(m, MII_BMCR);
	wr(m, MII_BMCR, (uint16_t)((bmcr >= 0 ? bmcr : 0) | BMCR_ANENABLE | BMCR_ANRESTART));
}

bool
RPiPHYLink(const RPiMDIO *m, uint32_t *mbps, bool *full)
{
	rd(m, MII_BMSR);                /* the link bit latches low */
	int bmsr = rd(m, MII_BMSR);
	if (bmsr < 0 || !(bmsr & BMSR_LSTATUS) || !(bmsr & BMSR_ANEGCOMPLETE)) {
		return false;
	}
	int lpa = rd(m, MII_LPA), adv = rd(m, MII_ADVERTISE);
	int stat1000 = rd(m, MII_STAT1000), ctrl1000 = rd(m, MII_CTRL1000);
	int common = (lpa >= 0 && adv >= 0) ? (lpa & adv) : 0;
	bool have1000 = stat1000 >= 0 && ctrl1000 >= 0;
	*mbps = 10;
	*full = false;
	if (have1000 && (stat1000 & LPA_1000FULL) && (ctrl1000 & ADV_1000FULL)) {
		*mbps = 1000;
		*full = true;
	} else if (have1000 && (stat1000 & LPA_1000HALF) && (ctrl1000 & ADV_1000HALF)) {
		*mbps = 1000;
	} else if (common & (LPA_100FULL | LPA_100HALF)) {
		*mbps = 100;
		*full = (common & LPA_100FULL) != 0;
	} else {
		*full = (common & LPA_10FULL) != 0;
	}
	return true;
}
