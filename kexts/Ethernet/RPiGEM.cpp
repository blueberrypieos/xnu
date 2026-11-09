/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * RPiGEM: the Raspberry Pi 5's Ethernet MAC, a Cadence GEM in RP1
 * (raspberrypi,rp1-gem), and its BCM54213PE PHY as an IOEthernetController,
 * after Linux's macb driver.
 *
 * Queue 0 only, both ways, with 256 64-bit descriptors each (address low,
 * control, address high, unused) in memory mapped non-cacheable; the other
 * queues point at a tie-off descriptor. Packet buffers are fixed 2 KB
 * slots, copied to and from mbufs and cleaned or invalidated around the
 * controller's accesses (RP1's DMA is not cache coherent). DMA addresses
 * are RP1's view of RAM ("pieboot,dma-ranges": 64 GB up). The MAC switches
 * its RGMII transmit clock with the link speed itself (Linux's
 * MACB_CAPS_CLK_HW_CHG for RP1). The PHY is polled once a second for the
 * link (RPiPHY.cpp); the interrupt is RP1's IRQ 6, through RP1.
 *
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOTimerEventSource.h>
#include <libkern/OSByteOrder.h>

#include <pie/rpi.h>

#include "RPiPHY.h"

/*
 * piegpio's interface for other drivers (kexts/GPIO/include/PieGPIO.h):
 * a GPIO controller publishes its device tree phandle as IOGPIOPHandle and
 * serves platform function PieGPIOSetOutput (line, level, consumer).
 */
#define kGPIOPHandleKey "IOGPIOPHandle"
#define kGPIOSetOutput  "PieGPIOSetOutput"
#define kGPIOWaitNs     (5ULL * 1000 * 1000 * 1000)

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

/* Registers (Linux's macb.h). */
enum {
	NCR = 0x0000, NCFGR = 0x0004, NSR = 0x0008, USRIO = 0x000c, DMACFG = 0x0010,
	TSR = 0x0014, RBQP = 0x0018, TBQP = 0x001c, RSR = 0x0020, ISR = 0x0024, IER = 0x0028,
	IDR = 0x002c, MAN = 0x0034, HRB = 0x0080, HRT = 0x0084, SA1B = 0x0088, SA1T = 0x008c,
	GEM_MID = 0x00fc,               /* module ID and revision */
	DCFG1 = 0x0280, DCFG6 = 0x0294,
	ISR_Q = 0x0400, TBQP_Q = 0x0440, RBQP_Q = 0x0480, TBQPH = 0x04c8, RBQPH = 0x04d4,
	IDR_Q = 0x0620,
};

enum {
	NCR_RE = 1U << 2, NCR_TE = 1U << 3, NCR_MPE = 1U << 4, NCR_CLRSTAT = 1U << 5,
	NCR_TSTART = 1U << 9,

	NCFGR_SPD = 1U << 0, NCFGR_FD = 1U << 1, NCFGR_CAF = 1U << 4, NCFGR_MTI = 1U << 6,
	NCFGR_BIG = 1U << 8, NCFGR_GBE = 1U << 10, NCFGR_DRFCS = 1U << 17,
	NCFGR_CLK_SHIFT = 18, NCFGR_CLK_MASK = 7U << 18, CLK_DIV96 = 5,  /* MDC: pclk / 96 */
	NCFGR_DBW_SHIFT = 21, NCFGR_DBW_MASK = 3U << 21,

	USRIO_RGMII = 1U << 0,

	DMACFG_FBLDO_MASK = 0x1f, DMACFG_FBLDO_INCR16 = 16,
	DMACFG_ENDIA_DESC = 1U << 6, DMACFG_ENDIA_PKT = 1U << 7, DMACFG_RXBMS = 3U << 8,
	DMACFG_TXPBMS = 1U << 10, DMACFG_TXCOEN = 1U << 11,
	DMACFG_RXBS_SHIFT = 16, DMACFG_RXBS_MASK = 0xffU << 16, DMACFG_ADDR64 = 1U << 30,

	NSR_IDLE = 1U << 2,

	MAN_SOF_C22 = 1U << 30, MAN_READ = 2U << 28, MAN_WRITE = 1U << 28,
	MAN_PHYA_SHIFT = 23, MAN_REGA_SHIFT = 18, MAN_CODE = 2U << 16,

	INT_RCOMP = 1U << 1, INT_TUND = 1U << 4, INT_TCOMP = 1U << 7, INT_ROVR = 1U << 10,
	INT_HRESP = 1U << 11,

	DCFG1_IRQCOR = 1U << 23, DCFG1_DBWDEF_SHIFT = 25, DCFG6_DAW64 = 1U << 23,
};

/* Descriptors (64-bit addressing: 4 words). */
enum {
	RX_USED = 1U << 0, RX_WRAP = 1U << 1,
	RX_SOF = 1U << 14, RX_EOF = 1U << 15, RX_LEN_MASK = 0x1fff,
	TX_LAST = 1U << 15, TX_WRAP = 1U << 30, TX_USED = 1U << 31, TX_LEN_MASK = 0x3fff,
};
struct GEMDesc {
	uint32_t addr;
	uint32_t ctrl;
	uint32_t addrHi;
	uint32_t unused;
};

#define RING_SIZE       256
#define BUF_SIZE        2048
#define MAX_FRAME       1536
#define MIN_FRAME       60
#define MAX_QUEUES      16

class RPiGEM : public IOEthernetController
{
	OSDeclareDefaultStructors(RPiGEM);

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
	IOBufferMemoryDescriptor *ringMem;      /* descriptors: RX, TX, RX and TX tie-offs */
	IOMemoryMap *ringMap;
	volatile struct GEMDesc *rxRing;
	volatile struct GEMDesc *txRing;
	uint64_t ringBus;
	IOBufferMemoryDescriptor *rxMem;
	IOBufferMemoryDescriptor *txMem;
	uint8_t *rxBufs;
	uint8_t *txBufs;
	uint64_t rxBus, txBus;
	uint32_t rxNext;
	uint32_t txProd, txCons;
	uint32_t queueMask;
	IOEthernetAddress mac;
	RPiMDIO mdio;
	bool rxDelay, txDelay;
	bool enabled;
	bool isrClearOnWrite;                   /* DCFG1.IRQCOR 0: ISR bits clear when written, not read */
	bool linkUp;
	bool promiscuous;
	bool multicast;

	uint32_t rd(uint32_t off) { return regs[off / 4]; }
	void wr(uint32_t off, uint32_t v) { regs[off / 4] = v; }

	static int mdioRead(void *context, int phy, int reg);
	static bool mdioWrite(void *context, int phy, int reg, uint16_t val);
	bool mdioIdle(void);
	bool readMACAddress(IOService *provider);
	uint32_t takeStatus(uint32_t reg);
	void resetHW(void);
	void initHW(void);
	void resetPHY(IOService *provider);
	void initRings(void);
	void setRxMode(void);
	void checkLink(void);
	void timerFired(IOTimerEventSource *src);
	void interruptOccurred(IOInterruptEventSource *src, int count);
	void receivePackets(void);
	bool reclaimTx(void);

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
OSDefineMetaClassAndStructors(RPiGEM, IOEthernetController);

#pragma mark MDIO and PHY

bool
RPiGEM::mdioIdle(void)
{
	for (int i = 0; i < 1000; i++) {
		if (rd(NSR) & NSR_IDLE) {
			return true;
		}
		IODelay(10);
	}
	return false;
}

int
RPiGEM::mdioRead(void *context, int phy, int reg)
{
	RPiGEM *self = (RPiGEM *)context;
	if (!self->mdioIdle()) {
		return -1;
	}
	self->wr(MAN, MAN_SOF_C22 | MAN_READ | (uint32_t)phy << MAN_PHYA_SHIFT | (uint32_t)reg << MAN_REGA_SHIFT |
	    MAN_CODE);
	if (!self->mdioIdle()) {
		return -1;
	}
	return (int)(self->rd(MAN) & 0xffff);
}

bool
RPiGEM::mdioWrite(void *context, int phy, int reg, uint16_t val)
{
	RPiGEM *self = (RPiGEM *)context;
	if (!self->mdioIdle()) {
		return false;
	}
	self->wr(MAN, MAN_SOF_C22 | MAN_WRITE | (uint32_t)phy << MAN_PHYA_SHIFT | (uint32_t)reg << MAN_REGA_SHIFT |
	    MAN_CODE | val);
	return self->mdioIdle();
}

/* Poll the PHY; on a change, set up the MAC for the link and report it. */
void
RPiGEM::checkLink(void)
{
	uint32_t mbps = 10;
	bool full = false;
	bool up = RPiPHYLink(&mdio, &mbps, &full);
	if (up == linkUp) {
		return;
	}
	linkUp = up;
	if (!up) {
		IOLog("RPiGEM: link down\n");
		setLinkStatus(kIONetworkLinkValid);
		return;
	}
	uint32_t cfg = rd(NCFGR) & ~(NCFGR_SPD | NCFGR_FD | NCFGR_GBE);
	cfg |= (mbps == 1000 ? NCFGR_GBE : 0) | (mbps == 100 ? NCFGR_SPD : 0) | (full ? NCFGR_FD : 0);
	wr(NCFGR, cfg);
	wr(NCR, rd(NCR) | NCR_RE | NCR_TE);

	uint32_t index = (mbps == 1000 ? 4 : mbps == 100 ? 2 : 0) + (full ? 1 : 0);
	IOLog("RPiGEM: link up, %u Mb/s, %s duplex\n", mbps, full ? "full" : "half");
	setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, mediums[index], (UInt64)mbps * 1000000ULL);
}

void
RPiGEM::timerFired(IOTimerEventSource *src)
{
	if (enabled) {
		checkLink();
		receivePackets();
		if (reclaimTx()) {
			txQueue->service(IOBasicOutputQueue::kServiceAsync);
		}
		src->setTimeoutMS(1000);
	}
}

#pragma mark MAC and DMA

/*
 * An interrupt status register's bits, cleared: by the read itself, or
 * (DCFG1.IRQCOR 0, as RP1's GEM may be built) by writing them back, as
 * Linux's macb does with MACB_CAPS_ISR_CLEAR_ON_WRITE. Not cleared, a
 * level interrupt stays up and the work loop runs the handler forever.
 */
uint32_t
RPiGEM::takeStatus(uint32_t reg)
{
	uint32_t status = rd(reg);
	if (isrClearOnWrite && status != 0) {
		wr(reg, status);
	}
	return status;
}

/* Receiver and transmitter off, statistics and status cleared, interrupts off. */
void
RPiGEM::resetHW(void)
{
	wr(NCR, (rd(NCR) & ~(NCR_RE | NCR_TE)) | NCR_CLRSTAT);
	wr(TSR, ~0U);
	wr(RSR, ~0U);
	wr(IDR, ~0U);
	takeStatus(ISR);
	for (unsigned q = 1; q < MAX_QUEUES; q++) {
		if (queueMask & (1U << q)) {
			wr(IDR_Q + 4 * (q - 1), ~0U);
			takeStatus(ISR_Q + 4 * (q - 1));
		}
	}
}

/* Address, network configuration, RGMII, DMA (Linux's macb_init_hw). */
void
RPiGEM::initHW(void)
{
	wr(SA1B, (uint32_t)mac.bytes[0] | (uint32_t)mac.bytes[1] << 8 | (uint32_t)mac.bytes[2] << 16 |
	    (uint32_t)mac.bytes[3] << 24);
	wr(SA1T, (uint32_t)mac.bytes[4] | (uint32_t)mac.bytes[5] << 8);
	for (uint32_t off = SA1B + 8; off <= SA1B + 24; off += 4) {
		wr(off, 0);             /* SA2..SA4 unused */
	}

	uint32_t dbw = (rd(DCFG1) >> DCFG1_DBWDEF_SHIFT) & 7;
	uint32_t cfg = (uint32_t)CLK_DIV96 << NCFGR_CLK_SHIFT | NCFGR_DRFCS | NCFGR_BIG |
	    (dbw == 4 ? 2U : dbw == 2 ? 1U : 0U) << NCFGR_DBW_SHIFT;
	wr(NCFGR, cfg);
	wr(USRIO, USRIO_RGMII);

	uint32_t dma = rd(DMACFG) & ~(DMACFG_RXBS_MASK | DMACFG_FBLDO_MASK | DMACFG_ENDIA_DESC |
	    DMACFG_ENDIA_PKT | DMACFG_TXCOEN);
	dma |= (BUF_SIZE / 64U) << DMACFG_RXBS_SHIFT | DMACFG_FBLDO_INCR16 | DMACFG_TXPBMS | DMACFG_RXBMS |
	    DMACFG_ADDR64;
	wr(DMACFG, dma);
	wr(NCR, rd(NCR) | NCR_MPE);
}

/*
 * Every receive slot to the controller, every transmit slot ours; the
 * queues other than 0 parked on the tie-offs (used and wrapping).
 */
void
RPiGEM::initRings(void)
{
	rxMem->performOperation(kIOMemoryIncoherentIOFlush, 0, RING_SIZE * BUF_SIZE);
	for (uint32_t i = 0; i < RING_SIZE; i++) {
		uint64_t bus = rxBus + (uint64_t)i * BUF_SIZE;
		rxRing[i].addrHi = (uint32_t)(bus >> 32);
		rxRing[i].ctrl = 0;
		rxRing[i].addr = ((uint32_t)bus & ~3U) | (i == RING_SIZE - 1 ? RX_WRAP : 0);
		txRing[i].addr = 0;
		txRing[i].addrHi = 0;
		txRing[i].ctrl = TX_USED | (i == RING_SIZE - 1 ? TX_WRAP : 0);
	}
	volatile struct GEMDesc *rxTie = txRing + RING_SIZE, *txTie = rxTie + 1;
	rxTie->addr = RX_USED | RX_WRAP;
	rxTie->addrHi = 0;
	rxTie->ctrl = 0;
	txTie->addr = 0;
	txTie->addrHi = 0;
	txTie->ctrl = TX_USED | TX_WRAP;
	__builtin_arm_dsb(0xf);

	uint64_t rxRingBus = ringBus, txRingBus = ringBus + RING_SIZE * sizeof(struct GEMDesc);
	uint64_t tieBus = txRingBus + RING_SIZE * sizeof(struct GEMDesc);
	wr(RBQPH, (uint32_t)(rxRingBus >> 32));
	wr(TBQPH, (uint32_t)(txRingBus >> 32));
	wr(RBQP, (uint32_t)rxRingBus);
	wr(TBQP, (uint32_t)txRingBus);
	for (unsigned q = 1; q < MAX_QUEUES; q++) {
		if (queueMask & (1U << q)) {
			wr(RBQP_Q + 4 * (q - 1), (uint32_t)tieBus);
			wr(TBQP_Q + 4 * (q - 1), (uint32_t)(tieBus + sizeof(struct GEMDesc)));
		}
	}
	rxNext = 0;
	txProd = txCons = 0;
}

/* Promiscuous (copy all frames), or every multicast frame (hash all ones). */
void
RPiGEM::setRxMode(void)
{
	uint32_t cfg = rd(NCFGR) & ~(NCFGR_CAF | NCFGR_MTI);
	if (promiscuous) {
		cfg |= NCFGR_CAF;
	}
	if (multicast) {
		wr(HRB, ~0U);
		wr(HRT, ~0U);
		cfg |= NCFGR_MTI;
	} else {
		wr(HRB, 0);
		wr(HRT, 0);
	}
	wr(NCFGR, cfg);
}

#pragma mark IOService

/*
 * The address the firmware put in the device tree (from the board's OTP),
 * else what is in the MAC already (the firmware's or a boot loader's).
 */
bool
RPiGEM::readMACAddress(IOService *provider)
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
	uint32_t bottom = rd(SA1B), top = rd(SA1T);
	if (bottom != 0 || (top & 0xffff) != 0) {
		for (int i = 0; i < 4; i++) {
			mac.bytes[i] = (uint8_t)(bottom >> (8 * i));
		}
		mac.bytes[4] = (uint8_t)top;
		mac.bytes[5] = (uint8_t)(top >> 8);
		return true;
	}
	return false;
}

/*
 * The PHY's reset line, as Linux's macb pulses it before probing MDIO:
 * phy-reset-gpios (<GPIO controller's phandle, line, flags>, flag 1 for
 * active low; on the Pi 5, RP1 GPIO 32, ETH_RST_N) held for
 * phy-reset-duration ms. The firmware leaves the PHY in reset when it does
 * not boot from the network. The line is driven through its GPIO
 * controller's platform functions (piegpio, which /etc/rc loads before
 * this kext): the controller whose IOGPIOPHandle is that phandle, waited
 * for a while; without one the PHY is probed as it is.
 */
void
RPiGEM::resetPHY(IOService *provider)
{
	OSData *gpios = OSDynamicCast(OSData, provider->getProperty("phy-reset-gpios"));
	if (gpios == NULL || gpios->getLength() < 3 * sizeof(uint32_t)) {
		return;
	}
	const uint8_t *cells = (const uint8_t *)gpios->getBytesNoCopy();
	uint32_t phandle = OSReadBigInt32(cells, 0);
	unsigned gpio = OSReadBigInt32(cells, 4);
	bool activeLow = OSReadBigInt32(cells, 8) & 1;
	uint32_t ms = 5;
	OSData *duration = OSDynamicCast(OSData, provider->getProperty("phy-reset-duration"));
	if (duration != NULL && duration->getLength() >= sizeof(uint32_t)) {
		ms = OSReadBigInt32(duration->getBytesNoCopy(), 0);
	}

	const OSSymbol *key = OSSymbol::withCStringNoCopy(kGPIOPHandleKey);
	OSNumber *handle = OSNumber::withNumber(phandle, 32);
	OSDictionary *match = key != NULL && handle != NULL ? IOService::propertyMatching(key, handle) : NULL;
	IOService *controller = match != NULL ? IOService::waitForMatchingService(match, kGPIOWaitNs) : NULL;
	OSSafeReleaseNULL(match);
	OSSafeReleaseNULL(handle);
	OSSafeReleaseNULL(key);
	const OSSymbol *setOutput = OSSymbol::withCStringNoCopy(kGPIOSetOutput);
	IOReturn ret = kIOReturnNoDevice;
	if (controller != NULL && setOutput != NULL) {
		ret = controller->callPlatformFunction(setOutput, false, (void *)(uintptr_t)gpio,
		    (void *)(uintptr_t)!activeLow, (void *)"RPiGEM", NULL);
	}
	if (ret == kIOReturnSuccess) {
		IOSleep(ms ? ms : 1);
		controller->callPlatformFunction(setOutput, false, (void *)(uintptr_t)gpio,
		    (void *)(uintptr_t)activeLow, (void *)"RPiGEM", NULL);
		IOSleep(30);            /* the BCM54213PE answers MDIO a few ms after reset */
		IOLog("RPiGEM: PHY reset (GPIO %u of %s) pulsed for %u ms\n", gpio, controller->getName(), ms);
	} else {
		IOLog("RPiGEM: cannot drive the PHY reset (GPIO %u of phandle %u: %s)\n", gpio, phandle,
		    controller != NULL ? "refused" : "no GPIO controller; is piegpio loaded?");
	}
	OSSafeReleaseNULL(setOutput);
	OSSafeReleaseNULL(controller);
}

bool
RPiGEM::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		IOLog("RPiGEM: cannot map the registers\n");
		return false;
	}
	regs = (volatile uint32_t *)regsMap->getVirtualAddress();
	uint32_t mid = rd(GEM_MID);
	if (mid == 0xffffffff || mid == 0) {
		IOLog("RPiGEM: no GEM at 0x%llx (module ID 0x%08x)\n", regsMap->getPhysicalAddress(), mid);
		return false;
	}
	if (!(rd(DCFG6) & DCFG6_DAW64)) {
		IOLog("RPiGEM: no 64-bit DMA addressing (DCFG6 0x%08x)\n", rd(DCFG6));
		return false;
	}
	queueMask = (rd(DCFG6) & 0xff) | 1;
	isrClearOnWrite = !(rd(DCFG1) & DCFG1_IRQCOR);

	const char *phyMode = RPiPHYMode(provider, "rgmii-id", &rxDelay, &txDelay);
	mdio.context = this;
	mdio.read = &RPiGEM::mdioRead;
	mdio.write = &RPiGEM::mdioWrite;
	if (!readMACAddress(provider)) {
		static const uint8_t fallback[6] = { 0x02, 0x78, 0x6e, 0x70, 0x69, 0x35 };
		bcopy(fallback, mac.bytes, sizeof(fallback));
		IOLog("RPiGEM: no MAC address from the firmware; using a local one\n");
	}

	/* Descriptors, non-cacheable; packet buffers, cached and maintained by hand. */
	size_t ringBytes = (2 * RING_SIZE + 2) * sizeof(struct GEMDesc);
	ringMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, ringBytes, 0xFFFFF000ULL);
	rxMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, RING_SIZE * BUF_SIZE, 0xFFFFF000ULL);
	txMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, RING_SIZE * BUF_SIZE, 0xFFFFF000ULL);
	if (ringMem == NULL || rxMem == NULL || txMem == NULL) {
		return false;
	}
	ringMem->performOperation(kIOMemoryIncoherentIOFlush, 0, ringBytes);
	ringMap = ringMem->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
	if (ringMap == NULL ||
	    !RPiDMAAddress(provider, ringMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &ringBus) ||
	    !RPiDMAAddress(provider, rxMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &rxBus) ||
	    !RPiDMAAddress(provider, txMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &txBus)) {
		IOLog("RPiGEM: DMA memory the controller cannot reach\n");
		return false;
	}
	rxRing = (volatile struct GEMDesc *)ringMap->getVirtualAddress();
	txRing = rxRing + RING_SIZE;
	rxBufs = (uint8_t *)rxMem->getBytesNoCopy();
	txBufs = (uint8_t *)txMem->getBytesNoCopy();

	resetHW();
	initHW();
	resetPHY(provider);
	if (!RPiPHYFind(&mdio, 1, "RPiGEM")) {
		IOLog("RPiGEM: no PHY answers on MDIO\n");
		return false;
	}

	intSource = IOInterruptEventSource::interruptEventSource(this,
	    OSMemberFunctionCast(IOInterruptEventAction, this, &RPiGEM::interruptOccurred), provider, 0);
	linkTimer = IOTimerEventSource::timerEventSource(this,
	    OSMemberFunctionCast(IOTimerEventSource::Action, this, &RPiGEM::timerFired));
	if (intSource == NULL || linkTimer == NULL ||
	    workLoop->addEventSource(intSource) != kIOReturnSuccess ||
	    workLoop->addEventSource(linkTimer) != kIOReturnSuccess) {
		IOLog("RPiGEM: no interrupt or timer\n");
		return false;
	}

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
	IOLog("RPiGEM: GEM, %02x:%02x:%02x:%02x:%02x:%02x, %s, DMA at 0x%llx, interrupt status cleared on %s\n",
	    mac.bytes[0], mac.bytes[1], mac.bytes[2], mac.bytes[3], mac.bytes[4], mac.bytes[5], phyMode, rxBus,
	    isrClearOnWrite ? "write" : "read");
	/* Output packets and errors are counted here (IONetworkInterface counts the rest). */
	IONetworkData *stats = netif->getParameter(kIONetworkStatsKey);
	netStats = stats != NULL ? (IONetworkStats *)stats->getBuffer() : NULL;
	netif->registerService();
	return true;
}

void
RPiGEM::free(void)
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
	OSSafeReleaseNULL(ringMap);
	OSSafeReleaseNULL(ringMem);
	OSSafeReleaseNULL(rxMem);
	OSSafeReleaseNULL(txMem);
	OSSafeReleaseNULL(workLoop);
	OSSafeReleaseNULL(regsMap);
	super::free();
}

#pragma mark IONetworkController

bool
RPiGEM::createWorkLoop(void)
{
	workLoop = IOWorkLoop::workLoop();
	return workLoop != NULL;
}

IOWorkLoop *
RPiGEM::getWorkLoop(void) const
{
	return workLoop;
}

IOOutputQueue *
RPiGEM::createOutputQueue(void)
{
	txQueue = IOGatedOutputQueue::withTarget(this, getWorkLoop());
	if (txQueue != NULL) {
		txQueue->retain();
	}
	return txQueue;
}

IOReturn
RPiGEM::enable(IONetworkInterface *interface)
{
	(void)interface;
	if (enabled) {
		return kIOReturnSuccess;
	}
	resetHW();
	initHW();
	initRings();
	setRxMode();
	RPiPHYConfigure(&mdio, rxDelay, txDelay);
	linkUp = false;

	takeStatus(ISR);
	wr(IER, INT_RCOMP | INT_TCOMP | INT_TUND | INT_ROVR | INT_HRESP);
	intSource->enable();
	txQueue->setCapacity(RING_SIZE);
	txQueue->start();
	enabled = true;
	setLinkStatus(kIONetworkLinkValid);
	linkTimer->setTimeoutMS(100);
	return kIOReturnSuccess;
}

IOReturn
RPiGEM::disable(IONetworkInterface *interface)
{
	(void)interface;
	if (!enabled) {
		return kIOReturnSuccess;
	}
	enabled = false;
	linkTimer->cancelTimeout();
	txQueue->stop();
	txQueue->flush();
	wr(IDR, ~0U);
	intSource->disable();
	resetHW();
	linkUp = false;
	setLinkStatus(kIONetworkLinkValid);
	return kIOReturnSuccess;
}

/* Called on the work loop (IOGatedOutputQueue). */
UInt32
RPiGEM::outputPacket(mbuf_t m, void *param)
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
	if (txProd - txCons >= RING_SIZE - 1) {
		reclaimTx();
		if (txProd - txCons >= RING_SIZE - 1) {
			return kIOReturnOutputStall;
		}
	}

	uint32_t slot = txProd % RING_SIZE;
	uint8_t *buf = txBufs + (size_t)slot * BUF_SIZE;
	size_t frame = len < MIN_FRAME ? MIN_FRAME : len;
	mbuf_copydata(m, 0, len, buf);
	if (frame > len) {
		bzero(buf + len, frame - len);
	}
	freePacket(m);
	txMem->performOperation(kIOMemoryIncoherentIOStore, (IOByteCount)slot * BUF_SIZE, frame);

	uint64_t bus = txBus + (uint64_t)slot * BUF_SIZE;
	txRing[slot].addr = (uint32_t)bus;
	txRing[slot].addrHi = (uint32_t)(bus >> 32);
	__builtin_arm_dmb(0xf);
	/* Clearing the used bit gives the slot to the controller. */
	txRing[slot].ctrl = ((uint32_t)frame & TX_LEN_MASK) | TX_LAST | (slot == RING_SIZE - 1 ? TX_WRAP : 0);
	__builtin_arm_dsb(0xf);
	txProd++;
	wr(NCR, rd(NCR) | NCR_TSTART);
	if (netStats != NULL) {
		netStats->outputPackets++;
	}
	return kIOReturnOutputSuccess;
}

/* The slots the controller has sent: their used bits are set again. */
bool
RPiGEM::reclaimTx(void)
{
	bool any = false;
	while (txCons != txProd && (txRing[txCons % RING_SIZE].ctrl & TX_USED)) {
		txCons++;
		any = true;
	}
	return any;
}

/* Frames the controller has written: their used bits are set. */
void
RPiGEM::receivePackets(void)
{
	bool any = false;
	for (uint32_t n = 0; n < RING_SIZE; n++) {
		volatile struct GEMDesc *d = &rxRing[rxNext];
		uint32_t addr = d->addr;
		if (!(addr & RX_USED)) {
			break;
		}
		__builtin_arm_dmb(0xf);
		uint32_t status = d->ctrl;
		uint32_t len = status & RX_LEN_MASK;
		rxMem->performOperation(kIOMemoryIncoherentIOFlush, (IOByteCount)rxNext * BUF_SIZE, BUF_SIZE);
		if (enabled && (status & (RX_SOF | RX_EOF)) == (RX_SOF | RX_EOF) && len > 0 && len <= BUF_SIZE) {
			mbuf_t m = allocatePacket(len);
			if (m != NULL) {
				mbuf_copyback(m, 0, len, rxBufs + (size_t)rxNext * BUF_SIZE, MBUF_DONTWAIT);
				netif->inputPacket(m, len, IONetworkInterface::kInputOptionQueuePacket);
				any = true;
			}
		}
		d->ctrl = 0;
		__builtin_arm_dmb(0xf);
		d->addr = addr & ~RX_USED;      /* back to the controller */
		rxNext = (rxNext + 1) % RING_SIZE;
	}
	__builtin_arm_dsb(0xf);
	if (any) {
		netif->flushInputQueue();
	}
}

void
RPiGEM::interruptOccurred(IOInterruptEventSource *src, int count)
{
	(void)src;
	(void)count;
	uint32_t status = takeStatus(ISR);
	if (!enabled) {
		return;
	}
	if (status & INT_HRESP) {
		IOLog("RPiGEM: DMA bus error (ISR 0x%08x)\n", status);
	}
	if (status & (INT_RCOMP | INT_ROVR)) {
		receivePackets();
	}
	if (reclaimTx()) {
		txQueue->service(IOBasicOutputQueue::kServiceAsync);
	}
}

IOReturn
RPiGEM::getHardwareAddress(IOEthernetAddress *addr)
{
	*addr = mac;
	return kIOReturnSuccess;
}

IOReturn
RPiGEM::setPromiscuousMode(bool active)
{
	promiscuous = active;
	if (enabled) {
		setRxMode();
	}
	return kIOReturnSuccess;
}

IOReturn
RPiGEM::setMulticastMode(bool active)
{
	multicast = active;
	if (enabled) {
		setRxMode();
	}
	return kIOReturnSuccess;
}

/* Every multicast frame is taken (the hash filter all ones) while any is wanted. */
IOReturn
RPiGEM::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
	(void)addrs;
	multicast = count > 0;
	if (enabled) {
		setRxMode();
	}
	return kIOReturnSuccess;
}
