/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * RPiGENET: the Raspberry Pi 4's Ethernet MAC (Broadcom GENET v5,
 * brcm,bcm2711-genet-v5) and its BCM54213PE PHY as an IOEthernetController.
 *
 * The bring-up follows Linux's bcmgenet driver: the UniMAC is reset and
 * set up, then only the default DMA ring (16) is used, in both directions,
 * with all 256 descriptors. Descriptors live in the controller; packet
 * buffers are fixed 2 KB slots in RAM below 4 GB, copied to and from mbufs.
 * GENET DMA is not cache coherent on the BCM2711, so every buffer is
 * cleaned before the controller reads it and cleaned and invalidated
 * around the controller's writes. Frames carry the 64-byte status blocks
 * (TSB/RSB) the hardware is set up for, and received frames start 2 bytes
 * after the RSB (RBUF_ALIGN_2B), as with Linux.
 *
 * The PHY (MDIO, through the UniMAC's MDIO controller) is polled once a
 * second for the link; its RGMII clock delays are set from the device
 * tree's phy-mode, as Linux's Broadcom PHY driver does (RPiPHY.cpp, shared
 * with RPiGEM).
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOTimerEventSource.h>
#include <libkern/OSByteOrder.h>

#include <pie/rpi.h>

#include "RPiPHY.h"

/*
 * The family is built with __PRIVATE_SPI__ (as its kext target is), which
 * changes its classes' vtables: drivers linked with it must see the same.
 */
#define __PRIVATE_SPI__ 1

/* IONetworkingFamily's headers (and kpi_mbuf.h) predate xnu's own warnings. */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-html"
#include <sys/kpi_mbuf.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IONetworkMedium.h>
#pragma clang diagnostic pop

/* Register blocks (GENET v4/v5 layout). */
enum {
	SYS_REV_CTRL        = 0x0000,
	SYS_PORT_CTRL       = 0x0004,
	SYS_RBUF_FLUSH_CTRL = 0x0008,
	EXT_RGMII_OOB_CTRL  = 0x008C,
	INTRL2_0            = 0x0200,
	RBUF_CTRL           = 0x0300,
	RBUF_TBUF_SIZE_CTRL = 0x03B4,
	TBUF_CTRL           = 0x0600,
	UMAC                = 0x0800,
	RDMA                = 0x2000,
	TDMA                = 0x4000,
	HFB_CTRL            = 0xFC00,
};

/* INTRL2 registers. */
enum { INTRL2_STAT = 0x00, INTRL2_CLEAR = 0x08, INTRL2_MASK_SET = 0x10, INTRL2_MASK_CLEAR = 0x14 };

/* UniMAC registers (from UMAC). */
enum {
	UMAC_CMD           = 0x008,
	UMAC_MAC0          = 0x00C,
	UMAC_MAC1          = 0x010,
	UMAC_MAX_FRAME_LEN = 0x014,
	UMAC_TX_FLUSH      = 0x334,
	UMAC_MIB_CTRL      = 0x580,
	UMAC_MDIO_CMD      = 0x614,
	UMAC_MDF_CTRL      = 0x650,
	UMAC_MDF_ADDR      = 0x654,
};

enum {
	CMD_TX_EN           = 1U << 0,
	CMD_RX_EN           = 1U << 1,
	CMD_SPEED_SHIFT     = 2,
	CMD_SPEED_MASK      = 3U << 2,
	CMD_PROMISC         = 1U << 4,
	CMD_CRC_FWD         = 1U << 6,
	CMD_RX_PAUSE_IGNORE = 1U << 8,
	CMD_HD_EN           = 1U << 10,
	CMD_SW_RESET        = 1U << 13,
	CMD_TX_PAUSE_IGNORE = 1U << 28,

	UMAC_SPEED_10   = 0,
	UMAC_SPEED_100  = 1,
	UMAC_SPEED_1000 = 2,

	MIB_RESET_RX   = 1U << 0,
	MIB_RESET_RUNT = 1U << 1,
	MIB_RESET_TX   = 1U << 2,

	MDIO_START_BUSY = 1U << 29,
	MDIO_READ_FAIL  = 1U << 28,
	MDIO_RD         = 2U << 26,
	MDIO_WR         = 1U << 26,
	MDIO_PMD_SHIFT  = 21,
	MDIO_REG_SHIFT  = 16,

	MAX_MDF_FILTER = 17,

	PORT_MODE_EXT_GPHY = 3,
	RGMII_LINK      = 1U << 4,
	OOB_DISABLE     = 1U << 5,
	RGMII_MODE_EN   = 1U << 6,
	ID_MODE_DIS     = 1U << 16,

	RBUF_64B_EN   = 1U << 0,
	RBUF_ALIGN_2B = 1U << 1,
	TBUF_64B_EN   = 1U << 0,

	UMAC_IRQ_RXDMA_MBDONE = 1U << 13,
	UMAC_IRQ_TXDMA_MBDONE = 1U << 16,
};

/* DMA: 256 descriptors of 3 words, 17 rings of 0x40 bytes, then the common registers. */
#define TOTAL_DESC      256
#define DESC_SIZE       12
#define RING            16                      /* the default ring */
#define RING_REGS(dma)  ((dma) + TOTAL_DESC * DESC_SIZE + RING * 0x40)
#define DMA_COMMON(dma) ((dma) + TOTAL_DESC * DESC_SIZE + 17 * 0x40)

enum {
	/* RDMA ring */
	RDMA_WRITE_PTR = 0x00, RDMA_PROD_INDEX = 0x08, RDMA_CONS_INDEX = 0x0C,
	RDMA_XON_XOFF_THRESH = 0x28, RDMA_READ_PTR = 0x2C,
	/* TDMA ring */
	TDMA_READ_PTR = 0x00, TDMA_CONS_INDEX = 0x08, TDMA_PROD_INDEX = 0x0C,
	TDMA_FLOW_PERIOD = 0x28, TDMA_WRITE_PTR = 0x2C,
	/* both */
	DMA_RING_BUF_SIZE = 0x10, DMA_START_ADDR = 0x14, DMA_END_ADDR = 0x1C,
	DMA_MBUF_DONE_THRESH = 0x24,
	/* common */
	DMA_RING_CFG = 0x00, DMA_CTRL = 0x04, DMA_STATUS = 0x08, DMA_SCB_BURST_SIZE = 0x0C,

	DESC_LENGTH_STATUS = 0x0, DESC_ADDRESS_LO = 0x4, DESC_ADDRESS_HI = 0x8,
};

enum {
	DMA_EN               = 1U << 0,
	DMA_RING_BUF_EN_SHIFT = 1,
	DMA_DISABLED         = 1U << 0,
	DMA_RING_SIZE_SHIFT  = 16,
	DMA_MAX_BURST_LENGTH = 0x08,
	DMA_FC_THRESH_HI     = TOTAL_DESC >> 4,
	DMA_FC_THRESH_LO     = 5,
	DMA_XOFF_THRESHOLD_SHIFT = 16,

	DMA_BUFLENGTH_SHIFT = 16,
	DMA_BUFLENGTH_MASK  = 0x0FFF,
	DMA_OWN   = 0x8000,
	DMA_EOP   = 0x4000,
	DMA_SOP   = 0x2000,
	DMA_TX_APPEND_CRC = 0x0040,
	DMA_TX_QTAG_SHIFT = 7,
	DMA_TX_QTAG_MASK  = 0x3F,
	DMA_RX_LG  = 0x0010,
	DMA_RX_NO  = 0x0008,
	DMA_RX_RXER = 0x0004,
	DMA_RX_CRC_ERROR = 0x0002,
	DMA_RX_OV  = 0x0001,
};

#define STATUS_BLOCK    64      /* TSB, RSB */
#define RX_PAD          2       /* RBUF_ALIGN_2B */
#define BUF_SIZE        2048
#define MAX_FRAME       1536    /* ENET_MAX_MTU_SIZE */
#define MIN_FRAME       60

class RPiGENET : public IOEthernetController
{
	OSDeclareDefaultStructors(RPiGENET);

	IOMemoryMap *regsMap;
	volatile uint32_t *regs;
	IOWorkLoop *workLoop;
	IOInterruptEventSource *intSource;
	IOTimerEventSource *linkTimer;
	IOEthernetInterface *netif;
	IONetworkStats *netStats;              /* the interface's counters */
	IOGatedOutputQueue *txQueue;
	IONetworkMedium *mediumAuto;
	IONetworkMedium *mediums[6];
	IOBufferMemoryDescriptor *rxMem;
	IOBufferMemoryDescriptor *txMem;
	uint8_t *rxBufs;
	uint8_t *txBufs;
	uint64_t rxPhys;
	uint64_t txPhys;
	uint16_t rxCons;        /* consumer index, as the hardware counts */
	uint16_t txProd;
	uint16_t txCons;
	IOEthernetAddress mac;
	RPiMDIO mdio;
	bool rxDelay, txDelay;  /* in the PHY, from phy-mode */
	bool enabled;
	bool linkUp;
	bool promiscuous;
	uint32_t mcCount;
	IOEthernetAddress mcList[MAX_MDF_FILTER - 2];

	uint32_t rd(uint32_t off) { return regs[off / 4]; }
	void wr(uint32_t off, uint32_t v) { regs[off / 4] = v; }

	static int mdioRead(void *context, int phy, int reg);
	static bool mdioWrite(void *context, int phy, int reg, uint16_t val);
	void resetUMAC(void);
	void initUMAC(void);
	void disableDMA(void);
	void initRings(void);
	void enableDMA(void);
	void setRxMode(void);
	void checkLink(void);
	void timerFired(IOTimerEventSource *src);
	void interruptOccurred(IOInterruptEventSource *src, int count);
	void receivePackets(void);
	bool reclaimTx(void);
	bool readMACAddress(IOService *provider);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	virtual bool createWorkLoop(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;
	virtual IOOutputQueue *createOutputQueue(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn enable(IONetworkInterface *interface) APPLE_KEXT_OVERRIDE;
	virtual IOReturn disable(IONetworkInterface *interface) APPLE_KEXT_OVERRIDE;
	virtual UInt32 outputPacket(mbuf_t m, void *param) APPLE_KEXT_OVERRIDE;
	virtual IOReturn getHardwareAddress(IOEthernetAddress *addr) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setPromiscuousMode(bool active) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setMulticastMode(bool active) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setMulticastList(IOEthernetAddress *addrs, UInt32 count) APPLE_KEXT_OVERRIDE;
};

#define super IOEthernetController
OSDefineMetaClassAndStructors(RPiGENET, IOEthernetController);

#pragma mark MDIO and PHY

int
RPiGENET::mdioRead(void *context, int phy, int reg)
{
	RPiGENET *self = (RPiGENET *)context;
	self->wr(UMAC + UMAC_MDIO_CMD, MDIO_RD | (uint32_t)phy << MDIO_PMD_SHIFT | (uint32_t)reg << MDIO_REG_SHIFT);
	self->wr(UMAC + UMAC_MDIO_CMD, self->rd(UMAC + UMAC_MDIO_CMD) | MDIO_START_BUSY);
	for (int i = 0; i < 1000; i++) {
		uint32_t cmd = self->rd(UMAC + UMAC_MDIO_CMD);
		if (!(cmd & MDIO_START_BUSY)) {
			return (cmd & MDIO_READ_FAIL) ? -1 : (int)(cmd & 0xFFFF);
		}
		IODelay(10);
	}
	return -1;
}

bool
RPiGENET::mdioWrite(void *context, int phy, int reg, uint16_t val)
{
	RPiGENET *self = (RPiGENET *)context;
	self->wr(UMAC + UMAC_MDIO_CMD, MDIO_WR | (uint32_t)phy << MDIO_PMD_SHIFT |
	    (uint32_t)reg << MDIO_REG_SHIFT | val);
	self->wr(UMAC + UMAC_MDIO_CMD, self->rd(UMAC + UMAC_MDIO_CMD) | MDIO_START_BUSY);
	for (int i = 0; i < 1000; i++) {
		if (!(self->rd(UMAC + UMAC_MDIO_CMD) & MDIO_START_BUSY)) {
			return true;
		}
		IODelay(10);
	}
	return false;
}

/* Poll the PHY; on a change, set up the MAC for the link and report it. */
void
RPiGENET::checkLink(void)
{
	uint32_t mbps = 10;
	bool full = false;
	bool up = RPiPHYLink(&mdio, &mbps, &full);
	if (up == linkUp) {
		return;
	}
	linkUp = up;
	if (!up) {
		IOLog("RPiGENET: link down\n");
		setLinkStatus(kIONetworkLinkValid);
		return;
	}
	uint32_t speed = mbps == 1000 ? UMAC_SPEED_1000 : mbps == 100 ? UMAC_SPEED_100 : UMAC_SPEED_10;

	/* RGMII block, then the UniMAC (out of reset on the first link). */
	uint32_t oob = rd(EXT_RGMII_OOB_CTRL);
	wr(EXT_RGMII_OOB_CTRL, (oob & ~OOB_DISABLE) | RGMII_LINK);
	uint32_t cmd = rd(UMAC + UMAC_CMD);
	cmd &= ~(CMD_SPEED_MASK | CMD_HD_EN | CMD_RX_PAUSE_IGNORE | CMD_TX_PAUSE_IGNORE);
	cmd |= speed << CMD_SPEED_SHIFT | CMD_RX_PAUSE_IGNORE | CMD_TX_PAUSE_IGNORE;
	if (!full) {
		cmd |= CMD_HD_EN;
	}
	if (cmd & CMD_SW_RESET) {
		cmd &= ~CMD_SW_RESET;
		wr(UMAC + UMAC_CMD, cmd);
		IODelay(2);
	}
	wr(UMAC + UMAC_CMD, cmd | CMD_TX_EN | CMD_RX_EN);

	uint32_t index = (mbps == 1000 ? 4 : mbps == 100 ? 2 : 0) + (full ? 1 : 0);
	IOLog("RPiGENET: link up, %u Mb/s, %s duplex\n", mbps, full ? "full" : "half");
	setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, mediums[index],
	    (UInt64)mbps * 1000000ULL);
}

void
RPiGENET::timerFired(IOTimerEventSource *src)
{
	if (enabled) {
		checkLink();
		/* Interrupts can be missed around ring setup; polling keeps up. */
		receivePackets();
		if (reclaimTx()) {
			txQueue->service(IOBasicOutputQueue::kServiceAsync);
		}
		src->setTimeoutMS(1000);
	}
}

#pragma mark MAC and DMA

void
RPiGENET::resetUMAC(void)
{
	wr(SYS_RBUF_FLUSH_CTRL, 0);
	IODelay(10);
	wr(UMAC + UMAC_CMD, CMD_SW_RESET);
	IODelay(2);
}

void
RPiGENET::initUMAC(void)
{
	wr(UMAC + UMAC_MIB_CTRL, MIB_RESET_RX | MIB_RESET_TX | MIB_RESET_RUNT);
	wr(UMAC + UMAC_MIB_CTRL, 0);
	wr(UMAC + UMAC_MAX_FRAME_LEN, MAX_FRAME);
	wr(TBUF_CTRL, rd(TBUF_CTRL) | TBUF_64B_EN);
	wr(RBUF_CTRL, rd(RBUF_CTRL) | RBUF_ALIGN_2B | RBUF_64B_EN);
	wr(RBUF_TBUF_SIZE_CTRL, 1);
	/* Every interrupt masked and cleared. */
	wr(INTRL2_0 + INTRL2_MASK_SET, 0xFFFFFFFF);
	wr(INTRL2_0 + INTRL2_CLEAR, 0xFFFFFFFF);
	wr(INTRL2_0 + 0x40 + INTRL2_MASK_SET, 0xFFFFFFFF);
	wr(INTRL2_0 + 0x40 + INTRL2_CLEAR, 0xFFFFFFFF);
	/* The station address. */
	wr(UMAC + UMAC_MAC0, (uint32_t)mac.bytes[0] << 24 | (uint32_t)mac.bytes[1] << 16 |
	    (uint32_t)mac.bytes[2] << 8 | mac.bytes[3]);
	wr(UMAC + UMAC_MAC1, (uint32_t)mac.bytes[4] << 8 | mac.bytes[5]);
	/* No hardware filter blocks. */
	wr(HFB_CTRL, 0);
	/* External RGMII PHY. */
	wr(SYS_PORT_CTRL, PORT_MODE_EXT_GPHY);
	uint32_t oob = rd(EXT_RGMII_OOB_CTRL) & ~ID_MODE_DIS;
	if (!rxDelay && !txDelay) {
		oob |= ID_MODE_DIS;     /* phy-mode "rgmii": no delays at all */
	}
	wr(EXT_RGMII_OOB_CTRL, oob | RGMII_MODE_EN);
}

void
RPiGENET::disableDMA(void)
{
	uint32_t bits = 1U << (RING + DMA_RING_BUF_EN_SHIFT) | DMA_EN;
	wr(DMA_COMMON(TDMA) + DMA_CTRL, rd(DMA_COMMON(TDMA) + DMA_CTRL) & ~bits);
	wr(DMA_COMMON(RDMA) + DMA_CTRL, rd(DMA_COMMON(RDMA) + DMA_CTRL) & ~bits);
	wr(UMAC + UMAC_TX_FLUSH, 1);
	IODelay(10);
	wr(UMAC + UMAC_TX_FLUSH, 0);
	uint32_t flush = rd(SYS_RBUF_FLUSH_CTRL);
	wr(SYS_RBUF_FLUSH_CTRL, flush | 1);
	IODelay(10);
	wr(SYS_RBUF_FLUSH_CTRL, flush);
	IODelay(10);
	for (int i = 0; i < 1000; i++) {
		if ((rd(DMA_COMMON(TDMA) + DMA_STATUS) & DMA_DISABLED) &&
		    (rd(DMA_COMMON(RDMA) + DMA_STATUS) & DMA_DISABLED)) {
			break;
		}
		IODelay(10);
	}
}

/* Ring 16 over all 256 descriptors, both ways. */
void
RPiGENET::initRings(void)
{
	uint32_t rx = RING_REGS(RDMA), tx = RING_REGS(TDMA);

	/* Receive buffers: the controller owns all of them, clean in RAM. */
	rxMem->performOperation(kIOMemoryIncoherentIOFlush, 0, TOTAL_DESC * BUF_SIZE);
	for (uint32_t i = 0; i < TOTAL_DESC; i++) {
		uint64_t pa = rxPhys + (uint64_t)i * BUF_SIZE;
		wr(RDMA + i * DESC_SIZE + DESC_ADDRESS_LO, (uint32_t)pa);
		wr(RDMA + i * DESC_SIZE + DESC_ADDRESS_HI, (uint32_t)(pa >> 32));
	}
	wr(rx + RDMA_PROD_INDEX, 0);
	wr(rx + RDMA_CONS_INDEX, 0);
	wr(rx + DMA_RING_BUF_SIZE, TOTAL_DESC << DMA_RING_SIZE_SHIFT | BUF_SIZE);
	wr(rx + RDMA_XON_XOFF_THRESH, DMA_FC_THRESH_LO << DMA_XOFF_THRESHOLD_SHIFT | DMA_FC_THRESH_HI);
	wr(rx + DMA_START_ADDR, 0);
	wr(rx + RDMA_READ_PTR, 0);
	wr(rx + RDMA_WRITE_PTR, 0);
	wr(rx + DMA_END_ADDR, TOTAL_DESC * (DESC_SIZE / 4) - 1);
	wr(rx + DMA_MBUF_DONE_THRESH, 1);
	rxCons = 0;

	wr(tx + TDMA_PROD_INDEX, 0);
	wr(tx + TDMA_CONS_INDEX, 0);
	wr(tx + DMA_MBUF_DONE_THRESH, 1);
	wr(tx + TDMA_FLOW_PERIOD, 0);
	wr(tx + DMA_RING_BUF_SIZE, TOTAL_DESC << DMA_RING_SIZE_SHIFT | BUF_SIZE);
	wr(tx + DMA_START_ADDR, 0);
	wr(tx + TDMA_READ_PTR, 0);
	wr(tx + TDMA_WRITE_PTR, 0);
	wr(tx + DMA_END_ADDR, TOTAL_DESC * (DESC_SIZE / 4) - 1);
	txProd = txCons = 0;

	wr(DMA_COMMON(RDMA) + DMA_SCB_BURST_SIZE, DMA_MAX_BURST_LENGTH);
	wr(DMA_COMMON(TDMA) + DMA_SCB_BURST_SIZE, DMA_MAX_BURST_LENGTH);
	wr(DMA_COMMON(RDMA) + DMA_RING_CFG, rd(DMA_COMMON(RDMA) + DMA_RING_CFG) | 1U << RING);
	wr(DMA_COMMON(TDMA) + DMA_RING_CFG, rd(DMA_COMMON(TDMA) + DMA_RING_CFG) | 1U << RING);
}

void
RPiGENET::enableDMA(void)
{
	uint32_t bits = 1U << (RING + DMA_RING_BUF_EN_SHIFT) | DMA_EN;
	wr(DMA_COMMON(RDMA) + DMA_CTRL, rd(DMA_COMMON(RDMA) + DMA_CTRL) | bits);
	wr(DMA_COMMON(TDMA) + DMA_CTRL, rd(DMA_COMMON(TDMA) + DMA_CTRL) | bits);
}

/*
 * Promiscuous mode, or the MDF address filters: broadcast, our address,
 * then the multicast list (Linux's bcmgenet_set_rx_mode).
 */
void
RPiGENET::setRxMode(void)
{
	uint32_t cmd = rd(UMAC + UMAC_CMD);
	uint32_t n = 2 + mcCount;
	if (promiscuous || n > MAX_MDF_FILTER) {
		wr(UMAC + UMAC_CMD, cmd | CMD_PROMISC);
		wr(UMAC + UMAC_MDF_CTRL, 0);
		return;
	}
	wr(UMAC + UMAC_CMD, cmd & ~CMD_PROMISC);

	static const IOEthernetAddress broadcast = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } };
	for (uint32_t i = 0; i < n; i++) {
		const uint8_t *a = i == 0 ? broadcast.bytes : i == 1 ? mac.bytes : mcList[i - 2].bytes;
		wr(UMAC + UMAC_MDF_ADDR + i * 8, (uint32_t)a[0] << 8 | a[1]);
		wr(UMAC + UMAC_MDF_ADDR + i * 8 + 4, (uint32_t)a[2] << 24 | (uint32_t)a[3] << 16 |
		    (uint32_t)a[4] << 8 | a[5]);
	}
	/* Filters are enabled from the top bit down. */
	uint32_t mask = ((1U << n) - 1) << (MAX_MDF_FILTER - n);
	wr(UMAC + UMAC_MDF_CTRL, mask);
}

#pragma mark IOService

/*
 * The address the firmware put in the device tree (from the board's OTP),
 * else what is in the UniMAC already (the firmware's or a boot loader's).
 */
bool
RPiGENET::readMACAddress(IOService *provider)
{
	OSData *d = OSDynamicCast(OSData, provider->getProperty("local-mac-address"));
	if (d != NULL && d->getLength() == 6) {
		bcopy(d->getBytesNoCopy(), mac.bytes, 6);
		for (int i = 0; i < 6; i++) {
			if (mac.bytes[i] != 0) {
				return true;
			}
		}
	}
	uint32_t mac0 = rd(UMAC + UMAC_MAC0), mac1 = rd(UMAC + UMAC_MAC1);
	if (mac0 != 0 || (mac1 & 0xFFFF) != 0) {
		mac.bytes[0] = (uint8_t)(mac0 >> 24);
		mac.bytes[1] = (uint8_t)(mac0 >> 16);
		mac.bytes[2] = (uint8_t)(mac0 >> 8);
		mac.bytes[3] = (uint8_t)mac0;
		mac.bytes[4] = (uint8_t)(mac1 >> 8);
		mac.bytes[5] = (uint8_t)mac1;
		return true;
	}
	return false;
}

bool
RPiGENET::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		IOLog("RPiGENET: cannot map the registers\n");
		return false;
	}
	regs = (volatile uint32_t *)regsMap->getVirtualAddress();

	uint32_t rev = rd(SYS_REV_CTRL);
	uint32_t major = (rev >> 24) & 0xF;
	if (major == 6) {
		major = 5;              /* GENET v5 reports 6 */
	}
	if (major != 5) {
		IOLog("RPiGENET: unexpected GENET revision 0x%08x\n", rev);
		return false;
	}

	/* phy-mode: where the RGMII delays are (rgmii-rxid on the Pi 4). */
	const char *phyMode = RPiPHYMode(provider, "rgmii-rxid", &rxDelay, &txDelay);
	mdio.context = this;
	mdio.read = &RPiGENET::mdioRead;
	mdio.write = &RPiGENET::mdioWrite;

	if (!readMACAddress(provider)) {
		/* A locally administered address of our own. */
		static const uint8_t fallback[6] = { 0x02, 0x78, 0x6e, 0x70, 0x69, 0x34 };
		bcopy(fallback, mac.bytes, sizeof(fallback));
		IOLog("RPiGENET: no MAC address from the firmware; using a local one\n");
	}

	/* Packet buffers below 4 GB (the controller's DMA window). */
	rxMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, TOTAL_DESC * BUF_SIZE, 0xFFFFF000ULL);
	txMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, TOTAL_DESC * BUF_SIZE, 0xFFFFF000ULL);
	if (rxMem == NULL || txMem == NULL) {
		return false;
	}
	rxBufs = (uint8_t *)rxMem->getBytesNoCopy();
	txBufs = (uint8_t *)txMem->getBytesNoCopy();
	rxPhys = rxMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	txPhys = txMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	bzero(txBufs, TOTAL_DESC * BUF_SIZE);
	txMem->performOperation(kIOMemoryIncoherentIOStore, 0, TOTAL_DESC * BUF_SIZE);

	if (!RPiPHYFind(&mdio, 1, "RPiGENET")) {
		IOLog("RPiGENET: no PHY answers on MDIO\n");
		return false;
	}

	intSource = IOInterruptEventSource::interruptEventSource(this,
	    OSMemberFunctionCast(IOInterruptEventAction, this, &RPiGENET::interruptOccurred),
	    provider, 0);
	linkTimer = IOTimerEventSource::timerEventSource(this,
	    OSMemberFunctionCast(IOTimerEventSource::Action, this, &RPiGENET::timerFired));
	if (intSource == NULL || linkTimer == NULL ||
	    workLoop->addEventSource(intSource) != kIOReturnSuccess ||
	    workLoop->addEventSource(linkTimer) != kIOReturnSuccess) {
		IOLog("RPiGENET: no interrupt or timer\n");
		return false;
	}

	/* Media: autoselect, and what the link can come up as. */
	static const struct { IOMediumType type; UInt64 speed; } media[6] = {
		{ kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex, 10 },
		{ kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex, 10 },
		{ kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex, 100 },
		{ kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex, 100 },
		{ kIOMediumEthernet1000BaseT | kIOMediumOptionHalfDuplex, 1000 },
		{ kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex, 1000 },
	};
	OSDictionary *dict = OSDictionary::withCapacity(7);
	mediumAuto = IONetworkMedium::medium(kIOMediumEthernetAuto, 0);
	if (dict == NULL || mediumAuto == NULL || !IONetworkMedium::addMedium(dict, mediumAuto)) {
		OSSafeReleaseNULL(dict);
		return false;
	}
	for (int i = 0; i < 6; i++) {
		mediums[i] = IONetworkMedium::medium(media[i].type, media[i].speed * 1000000ULL);
		if (mediums[i] == NULL || !IONetworkMedium::addMedium(dict, mediums[i])) {
			dict->release();
			return false;
		}
	}
	bool published = publishMediumDictionary(dict);
	dict->release();
	if (!published) {
		return false;
	}
	setSelectedMedium(mediumAuto);

	if (!attachInterface((IONetworkInterface **)&netif, false)) {
		return false;
	}
	IOLog("RPiGENET: GENET v5, %02x:%02x:%02x:%02x:%02x:%02x, %s\n", mac.bytes[0],
	    mac.bytes[1], mac.bytes[2], mac.bytes[3], mac.bytes[4], mac.bytes[5], phyMode);
	/* Output packets and errors are counted here (IONetworkInterface counts the rest). */
	IONetworkData *stats = netif->getParameter(kIONetworkStatsKey);
	netStats = stats != NULL ? (IONetworkStats *)stats->getBuffer() : NULL;
	netif->registerService();
	return true;
}

void
RPiGENET::free(void)
{
	if (workLoop != NULL) {
		if (intSource != NULL) {
			workLoop->removeEventSource(intSource);
		}
		if (linkTimer != NULL) {
			linkTimer->cancelTimeout();
			workLoop->removeEventSource(linkTimer);
		}
	}
	OSSafeReleaseNULL(intSource);
	OSSafeReleaseNULL(linkTimer);
	OSSafeReleaseNULL(netif);
	OSSafeReleaseNULL(txQueue);
	OSSafeReleaseNULL(mediumAuto);
	for (int i = 0; i < 6; i++) {
		OSSafeReleaseNULL(mediums[i]);
	}
	OSSafeReleaseNULL(rxMem);
	OSSafeReleaseNULL(txMem);
	OSSafeReleaseNULL(workLoop);
	OSSafeReleaseNULL(regsMap);
	super::free();
}

#pragma mark IONetworkController

bool
RPiGENET::createWorkLoop(void)
{
	workLoop = IOWorkLoop::workLoop();
	return workLoop != NULL;
}

IOWorkLoop *
RPiGENET::getWorkLoop(void) const
{
	return workLoop;
}

IOOutputQueue *
RPiGENET::createOutputQueue(void)
{
	txQueue = IOGatedOutputQueue::withTarget(this, getWorkLoop());
	if (txQueue != NULL) {
		txQueue->retain();
	}
	return txQueue;
}

IOReturn
RPiGENET::enable(IONetworkInterface *interface)
{
	(void)interface;
	if (enabled) {
		return kIOReturnSuccess;
	}
	resetUMAC();
	initUMAC();
	disableDMA();
	initRings();
	enableDMA();
	setRxMode();
	RPiPHYConfigure(&mdio, rxDelay, txDelay);
	linkUp = false;

	wr(INTRL2_0 + INTRL2_CLEAR, 0xFFFFFFFF);
	wr(INTRL2_0 + INTRL2_MASK_CLEAR, UMAC_IRQ_RXDMA_MBDONE | UMAC_IRQ_TXDMA_MBDONE);
	intSource->enable();
	txQueue->setCapacity(TOTAL_DESC);
	txQueue->start();
	enabled = true;
	setLinkStatus(kIONetworkLinkValid);
	linkTimer->setTimeoutMS(100);
	return kIOReturnSuccess;
}

IOReturn
RPiGENET::disable(IONetworkInterface *interface)
{
	(void)interface;
	if (!enabled) {
		return kIOReturnSuccess;
	}
	enabled = false;
	linkTimer->cancelTimeout();
	txQueue->stop();
	txQueue->flush();
	wr(INTRL2_0 + INTRL2_MASK_SET, 0xFFFFFFFF);
	intSource->disable();
	wr(UMAC + UMAC_CMD, rd(UMAC + UMAC_CMD) & ~(CMD_TX_EN | CMD_RX_EN));
	IODelay(10);
	disableDMA();
	linkUp = false;
	setLinkStatus(kIONetworkLinkValid);
	return kIOReturnSuccess;
}

/* Called on the work loop (IOGatedOutputQueue). */
UInt32
RPiGENET::outputPacket(mbuf_t m, void *param)
{
	(void)param;
	size_t len = mbuf_pkthdr_len(m);
	if (len > MAX_FRAME - 4 || !linkUp) {
		freePacket(m);
		if (netStats != NULL) {
			netStats->outputErrors++;
		}
		return kIOReturnOutputDropped;
	}
	if ((uint16_t)(txProd - txCons) >= TOTAL_DESC - 1) {
		reclaimTx();
		if ((uint16_t)(txProd - txCons) >= TOTAL_DESC - 1) {
			return kIOReturnOutputStall;
		}
	}

	uint32_t slot = txProd % TOTAL_DESC;
	uint8_t *buf = txBufs + (size_t)slot * BUF_SIZE;
	size_t frame = len < MIN_FRAME ? MIN_FRAME : len;
	bzero(buf, STATUS_BLOCK);       /* TSB: no offloads */
	mbuf_copydata(m, 0, len, buf + STATUS_BLOCK);
	if (frame > len) {
		bzero(buf + STATUS_BLOCK + len, frame - len);
	}
	freePacket(m);
	txMem->performOperation(kIOMemoryIncoherentIOStore, (IOByteCount)slot * BUF_SIZE,
	    STATUS_BLOCK + frame);

	uint64_t pa = txPhys + (uint64_t)slot * BUF_SIZE;
	uint32_t desc = TDMA + slot * DESC_SIZE;
	wr(desc + DESC_ADDRESS_LO, (uint32_t)pa);
	wr(desc + DESC_ADDRESS_HI, (uint32_t)(pa >> 32));
	wr(desc + DESC_LENGTH_STATUS, (uint32_t)(STATUS_BLOCK + frame) << DMA_BUFLENGTH_SHIFT |
	    DMA_TX_QTAG_MASK << DMA_TX_QTAG_SHIFT | DMA_TX_APPEND_CRC | DMA_SOP | DMA_EOP);
	txProd++;
	wr(RING_REGS(TDMA) + TDMA_PROD_INDEX, txProd);
	if (netStats != NULL) {
		netStats->outputPackets++;
	}
	return kIOReturnOutputSuccess;
}

/* Count the descriptors the controller has sent. */
bool
RPiGENET::reclaimTx(void)
{
	uint16_t cons = (uint16_t)(rd(RING_REGS(TDMA) + TDMA_CONS_INDEX) & 0xFFFF);
	bool any = cons != txCons;
	txCons = cons;
	return any;
}

void
RPiGENET::receivePackets(void)
{
	uint32_t rx = RING_REGS(RDMA);
	uint16_t prod = (uint16_t)(rd(rx + RDMA_PROD_INDEX) & 0xFFFF);
	bool crcFwd = (rd(UMAC + UMAC_CMD) & CMD_CRC_FWD) != 0;
	bool any = false;

	while (rxCons != prod) {
		uint32_t slot = rxCons % TOTAL_DESC;
		uint8_t *buf = rxBufs + (size_t)slot * BUF_SIZE;
		rxMem->performOperation(kIOMemoryIncoherentIOFlush, (IOByteCount)slot * BUF_SIZE, BUF_SIZE);

		/* The RSB's first word: the length and status of the frame. */
		uint32_t lenStatus = OSReadLittleInt32(buf, 0);
		uint32_t flags = lenStatus & 0xFFFF;
		uint32_t len = (lenStatus >> DMA_BUFLENGTH_SHIFT) & DMA_BUFLENGTH_MASK;
		bool good = (flags & (DMA_SOP | DMA_EOP)) == (DMA_SOP | DMA_EOP) &&
		    !(flags & (DMA_RX_CRC_ERROR | DMA_RX_OV | DMA_RX_NO | DMA_RX_LG | DMA_RX_RXER)) &&
		    len > STATUS_BLOCK + RX_PAD && len <= BUF_SIZE;
		if (good && enabled) {
			uint32_t frame = len - STATUS_BLOCK - RX_PAD - (crcFwd ? 4 : 0);
			mbuf_t m = allocatePacket(frame);
			if (m != NULL) {
				mbuf_copyback(m, 0, frame, buf + STATUS_BLOCK + RX_PAD, MBUF_DONTWAIT);
				netif->inputPacket(m, frame, IONetworkInterface::kInputOptionQueuePacket);
				any = true;
			}
		}
		rxCons++;
		wr(rx + RDMA_CONS_INDEX, rxCons);
	}
	if (any) {
		netif->flushInputQueue();
	}
}

void
RPiGENET::interruptOccurred(IOInterruptEventSource *src, int count)
{
	(void)src;
	(void)count;
	uint32_t status = rd(INTRL2_0 + INTRL2_STAT);
	wr(INTRL2_0 + INTRL2_CLEAR, status);
	if (!enabled) {
		return;
	}
	if (status & UMAC_IRQ_RXDMA_MBDONE) {
		receivePackets();
	}
	if (reclaimTx()) {
		txQueue->service(IOBasicOutputQueue::kServiceAsync);
	}
}

IOReturn
RPiGENET::getHardwareAddress(IOEthernetAddress *addr)
{
	*addr = mac;
	return kIOReturnSuccess;
}

IOReturn
RPiGENET::setPromiscuousMode(bool active)
{
	promiscuous = active;
	if (enabled) {
		setRxMode();
	}
	return kIOReturnSuccess;
}

IOReturn
RPiGENET::setMulticastMode(bool active)
{
	if (!active) {
		mcCount = 0;
		if (enabled) {
			setRxMode();
		}
	}
	return kIOReturnSuccess;
}

/* More addresses than filters: promiscuous (setRxMode). */
IOReturn
RPiGENET::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
	mcCount = count;
	for (UInt32 i = 0; i < count && i < MAX_MDF_FILTER - 2; i++) {
		mcList[i] = addrs[i];
	}
	if (enabled) {
		setRxMode();
	}
	return kIOReturnSuccess;
}
