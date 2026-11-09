/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * RPiPL011Serial: the Raspberry Pi 4's and 5's PL011 UARTs (arm,pl011,
 * arm,pl011-axi: the Pi 4's uart0 and uart2-5, the Pi 5's debug UART and
 * RP1's uart0-5) as serial ports, under IOSerialFamily: each is an
 * IOSerialDriverSync with an IORS232SerialStreamSync nub, which
 * IOSerialBSDClient makes the ttys /dev/cu.serialN and /dev/tty.serialN (N
 * the device tree's serialN alias: on a Pi 5 serial0 is RP1's uart0 on
 * GPIO 14 and 15, dtparam=uart0=on; on a Pi 4 serial3 is uart3 on GPIO 4
 * and 5, dtoverlay=uart3). termios is the family's, on the events below.
 *
 * The PL011's registers (ARM's PrimeCell UART TRM): DR 0x00 (data, bits
 * 11:8 its overrun, break, parity and framing errors), FR 0x18 (CTS bit 0,
 * BUSY 3, RXFE 4, TXFF 5, RXFF 6, TXFE 7), IBRD 0x24 and FBRD 0x28 (the
 * divisor UARTCLK / (16 * baud), 16.6 fixed point), LCR_H 0x2c (BRK 0,
 * PEN 1, EPS 2, STP2 3, FEN 4, WLEN 6:5, SPS 7: written after IBRD and
 * FBRD, with the UART off), CR 0x30 (UARTEN 0, LBE 7, TXE 8, RXE 9, DTR
 * 10, RTS 11, RTSEN 14, CTSEN 15), IFLS 0x34, IMSC 0x38, MIS 0x40, ICR
 * 0x44.
 *
 * UARTCLK: a "clock-frequency" on the node, else its first clock's (a
 * fixed-clock's: the Pi 5's debug UART, 9.216 MHz), else on a Pi 4 the
 * firmware's UART clock (GET_CLOCK_RATE, clock 2), else on RP1 the
 * read-only RP1 clk_uart source/divider registers and its fixed XOSC
 * parent. Unknown RP1 clock sources are declined; no clock is changed.
 *
 * Interrupts (the node's, shared on the Pi 4) take received bytes and
 * capture RX into an interrupt-protected ring; the workloop moves it to
 * the tty and refills TX. A 10 ms poll services deferred work as well
 * while the port is active, should one be missed. Received bytes with a
 * parity, framing or break error are dropped. Hardware flow control is the
 * PL011's own (RTSEN, CTSEN), where the tree says uart-has-rtscts and
 * pinctrl-0 gives the UART its CTS and RTS pins (a ctsrts group, as the
 * trees and overlays name them). IOSerialBSDClient leaves XON/XOFF
 * (IXON, IXOFF, IXANY) to the driver, which does none: the flow control it
 * cannot do is not taken, and tcgetattr shows it off.
 *
 * The UART the kernel uses (pexpert's choice: /defaults serial-device, or
 * the serial-device or serial-device-name boot-arg; while the serial
 * boot-arg, or debug's 0x8 or 0x2, sends it the console or kprintf)
 * is declined: on the images' Pi 4 uart0, on a Pi 5 the debug UART.
 *
 * For tests: an administrator may set the property Loopback (a Boolean,
 * IORegistryEntrySetCFProperties) to route TX to RX inside the PL011
 * (CR.LBE), which needs no wiring. Registry properties: UARTClock (Hz),
 * BaudRate, Loopback; RX error/IRQ/poll counters at close (cumulative
 * 32-bit, wrapping). Internal loopback also supports requested RTS/CTS.
 */

#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/serial/IORS232SerialStreamSync.h>
#include <IOKit/serial/IOSerialDriverSync.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <libkern/OSByteOrder.h>
#include <pexpert/pexpert.h>

#include <pie/rpi.h>

#include "PieGPIOKernel.h"

enum {
	DR = 0x00, DR_FE = 1U << 8, DR_PE = 1U << 9, DR_BE = 1U << 10, DR_OE = 1U << 11,
	FR = 0x18, FR_CTS = 1U << 0, FR_BUSY = 1U << 3, FR_RXFE = 1U << 4, FR_TXFF = 1U << 5, FR_TXFE = 1U << 7,
	IBRD = 0x24, FBRD = 0x28,
	LCR_H = 0x2c, LCR_BRK = 1U << 0, LCR_PEN = 1U << 1, LCR_EPS = 1U << 2, LCR_STP2 = 1U << 3, LCR_FEN = 1U << 4,
	LCR_WLEN_SHIFT = 5, LCR_SPS = 1U << 7,
	CR = 0x30, CR_UARTEN = 1U << 0, CR_LBE = 1U << 7, CR_TXE = 1U << 8, CR_RXE = 1U << 9, CR_DTR = 1U << 10,
	CR_RTS = 1U << 11, CR_RTSEN = 1U << 14, CR_CTSEN = 1U << 15,
	IFLS = 0x34, IFLS_RX_EIGHTH_TX_HALF = (0U << 3) | (2U << 0),
	IMSC = 0x38, IM_RX = 1U << 4, IM_TX = 1U << 5, IM_RT = 1U << 6, IM_OE = 1U << 10,
	MIS = 0x40, ICR = 0x44, ICR_ALL = 0x7ff,
	PL011_REGS = 0x48,
	FW_GET_CLOCK_RATE = 0x00030002, FW_CLOCK_UART = 2,
	DB_PRT_BIT = 0x2, DB_KPRT_BIT = 0x8,
};

#define kQueueSize      4096
#define kPollMs         10
#define kMinBaud        50

/* The bits a client may set (the family's: not ACQUIRED or ACTIVE, nor LOOP). */
#define EXTERNAL_MASK   (PD_S_MASK | (PD_RS232_S_MASK & ~PD_RS232_S_LOOP))

struct Ring {
	uint8_t *buf;
	uint32_t head, tail, count;     /* head: next in; tail: next out */

	uint32_t room(void) const { return kQueueSize - count; }
	void clear(void) { head = tail = count = 0; }
	uint32_t put(const uint8_t *p, uint32_t n)
	{
		uint32_t done = 0;
		while (done < n && count < kQueueSize) {
			buf[head] = p[done++];
			head = (head + 1) % kQueueSize;
			count++;
		}
		return done;
	}
	uint32_t get(uint8_t *p, uint32_t n)
	{
		uint32_t done = 0;
		while (done < n && count > 0) {
			p[done++] = buf[tail];
			tail = (tail + 1) % kQueueSize;
			count--;
		}
		return done;
	}
};

class RPiPL011Serial : public IOSerialDriverSync
{
	OSDeclareDefaultStructors(RPiPL011Serial);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE { return fWorkLoop; }
	virtual IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;

	virtual IOReturn acquirePort(bool sleep, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn releasePort(void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setState(UInt32 state, UInt32 mask, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual UInt32 getState(void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn watchState(UInt32 *state, UInt32 mask, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual UInt32 nextEvent(void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn executeEvent(UInt32 event, UInt32 data, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn requestEvent(UInt32 event, UInt32 *data, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn enqueueEvent(UInt32 event, UInt32 data, bool sleep, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn dequeueEvent(UInt32 *event, UInt32 *data, bool sleep, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn enqueueData(UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep, void *refCon)
	APPLE_KEXT_OVERRIDE;
	virtual IOReturn dequeueData(UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min, void *refCon)
	APPLE_KEXT_OVERRIDE;

private:
	IOMemoryMap *fMap;
	volatile uint32_t *fRegs;
	IOWorkLoop *fWorkLoop;
	IOCommandGate *fGate;
	IOFilterInterruptEventSource *fInterrupt;
	IOTimerEventSource *fPoll;
	IORS232SerialStreamSync *fNub;
	char fConsumer[16];
	bool fPinctrl, fRTSCTS, fLoopback, fStopping;
	bool fGateAdded, fPollAdded, fInterruptAdded, fHardwareTouched;

	uint32_t fClockHz;
	UInt32 fState, fWatchMask;
	uint32_t fImsc;
	Ring fTx, fRx, fIRQRx;
	IOSimpleLock *fRxLock;
	bool fIRQAccepting;
	uint32_t fTxLow, fTxHigh, fRxLow, fRxHigh;
	uint32_t fBaud, fDataBits, fParity, fStopHalfBits, fFlow;
	uint32_t fXon, fXoff, fLatency, fDelay, fMinLatency;
	uint32_t fOverruns, fErrors, fIRQEvents, fIRQBytes, fPollBytes;

	uint32_t rd(unsigned off) { return fRegs[off / 4]; }
	void wr(unsigned off, uint32_t v) { fRegs[off / 4] = v; }

	bool kernelConsole(IOService *provider);
	uint32_t uartClock(IOService *provider);
	int aliasNumber(IOService *provider);
	bool divisor(uint32_t baud, uint32_t *ibrd, uint32_t *fbrd);
	void program(void);
	void activate(bool on);
	void changeState(UInt32 state, UInt32 mask);
	void queueStates(void);
	void service(void);
	void captureRx(void);
	void drainRx(void);
	void flushRx(void);
	void fillTx(void);
	IOReturn execute(UInt32 event, UInt32 data);
	IOReturn request(UInt32 event, UInt32 *data);
	static bool filter(OSObject *owner, IOFilterInterruptEventSource *source);
	static void interrupt(OSObject *owner, IOInterruptEventSource *source, int count);
	static void poll(OSObject *owner, IOTimerEventSource *timer);
};

OSDefineMetaClassAndStructors(RPiPL011Serial, IOSerialDriverSync);
#define super IOSerialDriverSync

#pragma mark start and stop

/* Whether one of pinctrl-0's groups is the UART's CTS and RTS (rp1_uart0_ctsrts_16_17, uart3-ctsrts-gpio6). */
static bool
ctsrtsPins(IORegistryEntry *device)
{
	OSData *list = OSDynamicCast(OSData, device->getProperty("pinctrl-0"));
	unsigned n = list != NULL ? list->getLength() / 4 : 0;
	bool found = false;
	for (unsigned i = 0; i < n && !found; i++) {
		IORegistryEntry *config = PieGPIOCopyNodeForPHandle(OSReadBigInt32(list->getBytesNoCopy(), 4 * i));
		const char *name = config != NULL ? config->getName(gIODTPlane) : NULL;
		for (size_t j = 0; name != NULL && !found && name[j] != '\0'; j++) {
			found = strncmp(name + j, "ctsrts", 6) == 0;
		}
		OSSafeReleaseNULL(config);
	}
	return found;
}

static bool
phandleOf(IORegistryEntry *e, uint32_t *phandle)
{
	return PieGPIOReadPHandle(e, phandle);
}

/*
 * The UART pexpert writes the console or kprintf to (the images' boot-args
 * send it both), chosen as get_serial_device_phandle does: /defaults
 * serial-device, the serial-device boot-arg over it, serial-device-name's
 * node (the first of that name) over both.
 */
bool
RPiPL011Serial::kernelConsole(IOService *provider)
{
	uint32_t want = 0, self;
	bool have = false;
	IORegistryEntry *defaults = IORegistryEntry::fromPath("/defaults", gIODTPlane);
	OSData *dev = defaults != NULL ? OSDynamicCast(OSData, defaults->getProperty("serial-device")) : NULL;
	if (dev != NULL && dev->getLength() == sizeof(want)) {
		want = *(const uint32_t *)dev->getBytesNoCopy();
		have = true;
	}
	OSSafeReleaseNULL(defaults);
	if (PE_parse_boot_argn("serial-device", &want, sizeof(want))) {
		have = true;
	}
	char name[32];
	if (PE_parse_boot_arg_str("serial-device-name", name, sizeof(name))) {
		IORegistryIterator *it = IORegistryIterator::iterateOver(gIODTPlane, kIORegistryIterateRecursively);
		IORegistryEntry *e;
		while (it != NULL && (e = it->getNextObject()) != NULL) {
			OSData *n = OSDynamicCast(OSData, e->getProperty("name"));
			if (n != NULL && strncmp((const char *)n->getBytesNoCopy(), name, n->getLength()) == 0 &&
			    phandleOf(e, &want)) {
				have = true;
				break;
			}
		}
		OSSafeReleaseNULL(it);
	}
	bool same = have && phandleOf(provider, &self) && self == want;
	uint32_t serial = 0, debug = 0;
	PE_parse_boot_argn("serial", &serial, sizeof(serial));
	PE_parse_boot_argn("debug", &debug, sizeof(debug));
	return same && (serial != 0 || (debug & (DB_PRT_BIT | DB_KPRT_BIT)) != 0);
}

static bool
cell(IORegistryEntry *e, const char *key, unsigned i, uint32_t *v)
{
	OSData *d = OSDynamicCast(OSData, e->getProperty(key));
	if (d == NULL || d->getLength() < 4 * (i + 1)) {
		return false;
	}
	*v = OSReadBigInt32(d->getBytesNoCopy(), 4 * i);
	return true;
}

static bool
compatible(IORegistryEntry *e, const char *compat)
{
	OSData *d = OSDynamicCast(OSData, e->getProperty("compatible"));
	if (d == NULL) {
		return false;
	}
	const char *p = (const char *)d->getBytesNoCopy(), *end = p + d->getLength();
	for (; p < end; p += strnlen(p, (size_t)(end - p)) + 1) {
		if (strncmp(p, compat, (size_t)(end - p)) == 0) {
			return true;
		}
	}
	return false;
}

/* Read-only support for the firmware's XOSC-fed RP1 clk_uart.
 * CTRL auxiliary source is bits 9:5 (XOSC = 2), ENABLE bit 11;
 * DIV_INT is an 8-bit divisor, SEL must indicate the active mux input.
 * Register facts: Raspberry Pi RP1 clock documentation/reference driver.
 * Unknown PLL/external sources are declined rather than guessed.
 */
static uint32_t
rp1UartRate(uint32_t oscillator, uint32_t control, uint32_t divider, uint32_t selected)
{
	if (oscillator == 0 || !(control & (1U << 11)) ||
	    ((control >> 5) & 31) != 2 || selected != 1 || divider == 0 || divider > 255) {
		return 0;
	}
	return oscillator / divider;
}

uint32_t
RPiPL011Serial::uartClock(IOService *provider)
{
	uint32_t hz = 0, clock = 0;
	if (cell(provider, "clock-frequency", 0, &hz) && hz != 0) {
		return hz;
	}
	IORegistryEntry *node = cell(provider, "clocks", 0, &clock) ? PieGPIOCopyNodeForPHandle(clock) : NULL;
	if (node != NULL && compatible(node, "fixed-clock") && cell(node, "clock-frequency", 0, &hz)) {
		node->release();
		return hz;
	}
	bool rp1 = node != NULL && compatible(node, "raspberrypi,rp1-clocks");
	if (rp1) {
		IOService *clockNub = OSDynamicCast(IOService, node);
		IOMemoryMap *map = clockNub != NULL ? clockNub->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache) : NULL;
		hz = 0;
		uint32_t clockId = 0;
		if (map != NULL && map->getLength() >= 0x64 &&
		    cell(provider, "clocks", 1, &clockId) && clockId == 15) {
			volatile uint32_t *regs = (volatile uint32_t *)map->getVirtualAddress();
			uint32_t ctrl = regs[0x54 / 4], div = regs[0x58 / 4], sel = regs[0x60 / 4];
			setProperty("UARTClockControl", ctrl, 32);
			setProperty("UARTClockDivider", div, 32);
			setProperty("UARTClockSelect", sel, 32);
			/* RP1 clk_uart: auxiliary XOSC source, integer divider, enabled. */
			uint32_t osc = 0, oscHz = 0;
			IORegistryEntry *oscNode = cell(node, "clocks", 0, &osc) ? PieGPIOCopyNodeForPHandle(osc) : NULL;
			if (oscNode != NULL && compatible(oscNode, "fixed-clock") &&
			    cell(oscNode, "clock-frequency", 0, &oscHz)) {
				hz = rp1UartRate(oscHz, ctrl, div, sel);
			}
			OSSafeReleaseNULL(oscNode);
			IOLog("%s: RP1 UART clock ctrl=0x%x div=%u sel=0x%x: %u Hz\n", getName(), ctrl, div, sel, hz);
		}
		OSSafeReleaseNULL(map);
		OSSafeReleaseNULL(node);
		return hz;
	}
	OSSafeReleaseNULL(node);
	uint32_t data[2] = { FW_CLOCK_UART, 0 };
	if (RPiFirmwareProperty(FW_GET_CLOCK_RATE, data, sizeof(data)) == kIOReturnSuccess && data[1] != 0) {
		return data[1];
	}
	return 0;
}

/* The device tree's serialN alias for the node, by the alias's last path component; -1 if none. */
int
RPiPL011Serial::aliasNumber(IOService *provider)
{
	OSData *index = OSDynamicCast(OSData, provider->getProperty("pieboot,serial-index"));
	if (index != NULL && index->getLength() == sizeof(uint32_t)) {
		uint32_t number = 0;
		bcopy(index->getBytesNoCopy(), &number, sizeof(number));
		if (number < 1000) { return (int)number; }
	}

	char self[64];
	const char *location = provider->getLocation(gIODTPlane);
	snprintf(self, sizeof(self), "%s%s%s", provider->getName(gIODTPlane), location != NULL ? "@" : "",
	    location != NULL ? location : "");
	IORegistryEntry *aliases = IORegistryEntry::fromPath("/aliases", gIODTPlane);
	if (aliases == NULL) {
		return -1;
	}
	int number = -1;
	OSDictionary *props = aliases->dictionaryWithProperties();
	OSCollectionIterator *keys = props != NULL ? OSCollectionIterator::withCollection(props) : NULL;
	OSSymbol *key;
	while (keys != NULL && number < 0 && (key = OSDynamicCast(OSSymbol, keys->getNextObject())) != NULL) {
		const char *k = key->getCStringNoCopy();
		char *end = NULL;
		if (strncmp(k, "serial", 6) != 0 || k[6] < '0' || k[6] > '9') {
			continue;
		}
		unsigned long n = strtoul(k + 6, &end, 10);
		OSData *path = OSDynamicCast(OSData, props->getObject(key));
		if (end == NULL || *end != '\0' || path == NULL || path->getLength() == 0) {
			continue;
		}
		const char *p = (const char *)path->getBytesNoCopy();
		size_t len = strnlen(p, path->getLength());
		const char *last = p;
		for (size_t i = 0; i < len; i++) {
			if (p[i] == '/') {
				last = p + i + 1;
			}
		}
		if ((size_t)(p + len - last) == strlen(self) && strncmp(last, self, strlen(self)) == 0) {
			number = (int)n;
		}
	}
	OSSafeReleaseNULL(keys);
	OSSafeReleaseNULL(props);
	aliases->release();
	return number;
}

bool
RPiPL011Serial::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	if (kernelConsole(provider)) {
		IOLog("%s: %s is the kernel's console (the serial or debug boot-arg); not a tty\n", getName(),
		    provider->getName());
		return false;
	}
	fMap = provider->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache);
	if (fMap == NULL || fMap->getLength() < PL011_REGS) {
		IOLog("%s: cannot map its registers\n", getName());
		return false;
	}
	fRegs = (volatile uint32_t *)fMap->getVirtualAddress();
	fClockHz = uartClock(provider);
	if (fClockHz < 16 * kMinBaud) {
		IOLog("%s: %s: no UART clock rate; not a tty\n", getName(), provider->getName());
		return false;
	}
	int n = aliasNumber(provider);
	char suffix[24];
	if (n >= 0) {
		snprintf(suffix, sizeof(suffix), "%d", n);
	} else {
		const char *loc = provider->getLocation(gIODTPlane);
		snprintf(suffix, sizeof(suffix), "-%s", loc != NULL ? loc : "0");
	}
	snprintf(fConsumer, sizeof(fConsumer), "serial%s", suffix);

	fTx.buf = (uint8_t *)IOMallocData(kQueueSize);
	fRx.buf = (uint8_t *)IOMallocData(kQueueSize);
	fIRQRx.buf = (uint8_t *)IOMallocData(kQueueSize);
	fRxLock = IOSimpleLockAlloc();
	fWorkLoop = IOWorkLoop::workLoop();
	fGate = fWorkLoop != NULL ? IOCommandGate::commandGate(this) : NULL;
	fPoll = fWorkLoop != NULL ? IOTimerEventSource::timerEventSource(this, poll) : NULL;
	if (fTx.buf == NULL || fRx.buf == NULL || fIRQRx.buf == NULL || fRxLock == NULL || fGate == NULL || fPoll == NULL) {
		return false;
	}
	if (fWorkLoop->addEventSource(fGate) != kIOReturnSuccess) { return false; }
	fGateAdded = true;
	if (fWorkLoop->addEventSource(fPoll) != kIOReturnSuccess) { return false; }
	fPollAdded = true;
	if (provider->getProperty("pinctrl-0") != NULL) {
		OSData *pins = OSDynamicCast(OSData, provider->getProperty("pinctrl-0"));
		if (pins == NULL || pins->getLength() == 0 || pins->getLength() % 4 != 0) { return false; }
		fPinctrl = true;  /* also release partial groups if a later one fails */
		IOReturn ret = PieGPIOApplyPinctrl(provider, fConsumer, kPieGPIOWaitNs);
		if (ret != kIOReturnSuccess) {
			IOLog("%s: %s: cannot reserve/configure pins (0x%x); not a tty\n", getName(), fConsumer, ret);
			return false;
		}
		fRTSCTS = provider->getProperty("uart-has-rtscts") != NULL && ctsrtsPins(provider);
	}
	/* Quiet until a client activates it, after any header pins are reserved. */
	fHardwareTouched = true;
	wr(IMSC, 0);
	wr(CR, 0);
	wr(ICR, ICR_ALL);
	fInterrupt = IOFilterInterruptEventSource::filterInterruptEventSource(this, interrupt, filter, provider, 0);
	if (fInterrupt == NULL || fWorkLoop->addEventSource(fInterrupt) != kIOReturnSuccess) {
		IOLog("%s: %s: no interrupt; the 10 ms poll only\n", getName(), fConsumer);
		OSSafeReleaseNULL(fInterrupt);
	} else {
		fInterruptAdded = true;
		fInterrupt->enable();
	}
	fBaud = 9600;
	fDataBits = 8;
	fParity = PD_RS232_PARITY_NONE;
	fStopHalfBits = 2;
	fXon = 0x11;
	fXoff = 0x13;
	fTxLow = kQueueSize / 4;
	fTxHigh = kQueueSize * 3 / 4;
	fRxLow = kQueueSize / 4;
	fRxHigh = kQueueSize * 3 / 4;
	setProperty("UARTClock", fClockHz, 32);
	setProperty("BaudRate", fBaud, 32);
	setProperty("Loopback", false);

	fNub = OSTypeAlloc(IORS232SerialStreamSync);
	if (fNub == NULL || !fNub->init(NULL, NULL)) {
		OSSafeReleaseNULL(fNub);
		return false;
	}
	fNub->setProperty(kIOTTYBaseNameKey, "serial");
	fNub->setProperty(kIOTTYSuffixKey, suffix);
	if (!fNub->attach(this)) {
		OSSafeReleaseNULL(fNub);
		return false;
	}
	fNub->registerService();
	registerService();
	IOLog("%s: %s [%s], UART clock %u Hz%s\n", getName(), fConsumer, provider->getName(), fClockHz,
	    fRTSCTS ? ", RTS/CTS" : "");
	return true;
}

void
RPiPL011Serial::stop(IOService *provider)
{
	if (fGate != NULL) {
		fGate->runActionBlock(^IOReturn {
			fStopping = true;
			activate(false);
			fHardwareTouched = false;
			fState = 0;
			fGate->commandWakeup(&fState, false);
			fGate->commandWakeup(&fTx, false);
			fGate->commandWakeup(&fRx, false);
			return kIOReturnSuccess;
		});
	}
	if (fNub != NULL) {
		fNub->terminate(kIOServiceSynchronous);
		fNub->release();
		fNub = NULL;
	}
	if (fInterrupt != NULL) {
		fInterrupt->disable();
		if (fInterruptAdded) { fWorkLoop->removeEventSource(fInterrupt); fInterruptAdded = false; }
	}
	if (fPoll != NULL) {
		fPoll->cancelTimeout();
		if (fPollAdded) { fWorkLoop->removeEventSource(fPoll); fPollAdded = false; }
	}
	if (fGate != NULL) {
		if (fGateAdded) { fWorkLoop->removeEventSource(fGate); fGateAdded = false; }
	}
	if (fPinctrl) {
		PieGPIOReleaseAll(fConsumer);
		fPinctrl = false;
	}
	super::stop(provider);
}

void
RPiPL011Serial::free(void)
{
	/* Failed start is freed without stop: detach sources before releasing buffers/maps. */
	if (fInterrupt != NULL) { fInterrupt->disable(); }
	if (fInterruptAdded) { fWorkLoop->removeEventSource(fInterrupt); fInterruptAdded = false; }
	if (fPoll != NULL) { fPoll->cancelTimeout(); }
	if (fPollAdded) { fWorkLoop->removeEventSource(fPoll); fPollAdded = false; }
	if (fHardwareTouched) {
		wr(IMSC, 0); wr(CR, 0); wr(ICR, ICR_ALL);
		fHardwareTouched = false;
	}
	if (fPinctrl) { PieGPIOReleaseAll(fConsumer); fPinctrl = false; }
	if (fGateAdded) { fWorkLoop->removeEventSource(fGate); fGateAdded = false; }
	OSSafeReleaseNULL(fNub);
	OSSafeReleaseNULL(fInterrupt);
	if (fRxLock != NULL) { IOSimpleLockFree(fRxLock); fRxLock = NULL; }
	if (fIRQRx.buf != NULL) { IOFreeData(fIRQRx.buf, kQueueSize); fIRQRx.buf = NULL; }
	OSSafeReleaseNULL(fPoll);
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fWorkLoop);
	OSSafeReleaseNULL(fMap);
	if (fTx.buf != NULL) {
		IOFreeData(fTx.buf, kQueueSize);
	}
	if (fRx.buf != NULL) {
		IOFreeData(fRx.buf, kQueueSize);
	}
	super::free();
}

/* An administrator's Loopback (a Boolean): TX to RX inside the PL011, for tests. */
IOReturn
RPiPL011Serial::setProperties(OSObject *properties)
{
	OSDictionary *dict = OSDynamicCast(OSDictionary, properties);
	OSBoolean *loop = dict != NULL ? OSDynamicCast(OSBoolean, dict->getObject("Loopback")) : NULL;
	if (loop == NULL) {
		return kIOReturnUnsupported;
	}
	if (IOUserClient::clientHasPrivilege(current_task(), kIOClientPrivilegeAdministrator) != kIOReturnSuccess) {
		return kIOReturnNotPrivileged;
	}
	bool on = loop->isTrue();
	return fGate->runActionBlock(^IOReturn {
		fLoopback = on;
		if (!on && !fRTSCTS) { fFlow = 0; }
		if (fState & PD_S_ACTIVE) {
			wr(CR, on ? rd(CR) | CR_LBE : rd(CR) & ~CR_LBE);
			program();
		}
		setProperty("Loopback", on);
		return kIOReturnSuccess;
	});
}

#pragma mark the hardware, on the work loop

/* UARTCLK / (16 * baud) as 16.6 fixed point: IBRD 1-65535. */
bool
RPiPL011Serial::divisor(uint32_t baud, uint32_t *ibrd, uint32_t *fbrd)
{
	if (baud < kMinBaud) {
		return false;
	}
	uint64_t div64 = ((uint64_t)fClockHz * 4 + baud / 2) / baud;
	*ibrd = (uint32_t)(div64 >> 6);
	*fbrd = (uint32_t)(div64 & 63);
	return *ibrd >= 1 && *ibrd <= 0xffff;
}

/* The line's settings into the PL011: off, idle, divisor, LCR_H, on as it was. */
void
RPiPL011Serial::program(void)
{
	IOInterruptState irqState = IOSimpleLockLockDisableInterrupt(fRxLock);
	bool accepting = fIRQAccepting;
	fIRQAccepting = false;
	wr(IMSC, 0);
	IOSimpleLockUnlockEnableInterrupt(fRxLock, irqState);
	uint32_t cr = rd(CR), ibrd = 1, fbrd = 0;
	divisor(fBaud, &ibrd, &fbrd);
	wr(CR, cr & ~CR_UARTEN);
	for (unsigned i = 0; i < 10000 && (rd(FR) & FR_BUSY); i++) {
		IODelay(1);
	}
	uint32_t lcr = (rd(LCR_H) & LCR_BRK) | LCR_FEN | (fDataBits - 5) << LCR_WLEN_SHIFT;
	if (fStopHalfBits == 4) {
		lcr |= LCR_STP2;
	}
	switch (fParity) {
	case PD_RS232_PARITY_ODD: lcr |= LCR_PEN; break;
	case PD_RS232_PARITY_EVEN: lcr |= LCR_PEN | LCR_EPS; break;
	case PD_RS232_PARITY_MARK: lcr |= LCR_PEN | LCR_SPS; break;
	case PD_RS232_PARITY_SPACE: lcr |= LCR_PEN | LCR_EPS | LCR_SPS; break;
	default: break;
	}
	wr(IBRD, ibrd);
	wr(FBRD, fbrd);
	wr(LCR_H, lcr);
	cr &= ~(CR_RTSEN | CR_CTSEN);
	if ((fRTSCTS || fLoopback) && (fFlow & PD_RS232_A_RFR)) {
		cr |= CR_RTSEN;
	}
	if ((fRTSCTS || fLoopback) && (fFlow & PD_RS232_A_CTS)) {
		cr |= CR_CTSEN;
	}
	wr(CR, cr);
	irqState = IOSimpleLockLockDisableInterrupt(fRxLock);
	fIRQAccepting = accepting;
	if (accepting) { wr(IMSC, fInterruptAdded ? fImsc : 0); }
	IOSimpleLockUnlockEnableInterrupt(fRxLock, irqState);
	setProperty("BaudRate", fBaud, 32);
}

void
RPiPL011Serial::activate(bool on)
{
	if (on == ((fState & PD_S_ACTIVE) != 0)) {
		return;
	}
	if (on) {
		IOInterruptState irqState = IOSimpleLockLockDisableInterrupt(fRxLock);
		fIRQAccepting = false;
		wr(IMSC, 0);
		fIRQRx.clear();
		IOSimpleLockUnlockEnableInterrupt(fRxLock, irqState);
		fTx.clear();
		fRx.clear();
		wr(IMSC, 0);
		wr(ICR, ICR_ALL);
		wr(IFLS, IFLS_RX_EIGHTH_TX_HALF);
		wr(CR, CR_TXE | CR_RXE | CR_DTR | CR_RTS | (fLoopback ? CR_LBE : 0));
		program();
		wr(CR, rd(CR) | CR_UARTEN);
		/* Whatever was in the receive FIFO before is not this session's. */
		irqState = IOSimpleLockLockDisableInterrupt(fRxLock);
		for (unsigned i = 0; i < 256 && !(rd(FR) & FR_RXFE); i++) {
			(void)rd(DR);
		}
		fImsc = IM_RX | IM_RT | IM_OE;
		fIRQAccepting = true;
		wr(IMSC, fInterruptAdded ? fImsc : 0);
		IOSimpleLockUnlockEnableInterrupt(fRxLock, irqState);
		changeState(PD_S_ACTIVE | PD_RS232_S_DTR | PD_RS232_S_RTS, PD_S_ACTIVE | PD_RS232_S_DTR | PD_RS232_S_RTS);
		queueStates();
		fPoll->setTimeoutMS(kPollMs);
	} else {
		fPoll->cancelTimeout();
		IOInterruptState irqState = IOSimpleLockLockDisableInterrupt(fRxLock);
		fIRQAccepting = false;
		fIRQRx.clear();
		fImsc = 0;
		wr(IMSC, 0);
		wr(CR, 0);
		wr(ICR, ICR_ALL);
		IOSimpleLockUnlockEnableInterrupt(fRxLock, irqState);
		fTx.clear();
		fRx.clear();
		changeState(0, PD_S_ACTIVE | PD_S_TX_BUSY | PD_S_RX_BUSY | PD_RS232_S_DTR | PD_RS232_S_RTS);
		queueStates();
		fGate->commandWakeup(&fTx, false);
		fGate->commandWakeup(&fRx, false);
	}
}

/* fState's bits in mask become state's; DTR and RTS go to the PL011; watchers of a bit that changed wake. */
void
RPiPL011Serial::changeState(UInt32 state, UInt32 mask)
{
	UInt32 next = (fState & ~mask) | (state & mask);
	UInt32 delta = next ^ fState;
	fState = next;
	if ((delta & (PD_RS232_S_DTR | PD_RS232_S_RTS)) && (fState & PD_S_ACTIVE)) {
		uint32_t cr = rd(CR) & ~(CR_DTR | CR_RTS);
		cr |= (fState & PD_RS232_S_DTR) ? CR_DTR : 0;
		cr |= (fState & PD_RS232_S_RTS) ? CR_RTS : 0;
		wr(CR, cr);
	}
	if (delta & fWatchMask) {
		fWatchMask = 0;
		fGate->commandWakeup(&fState, false);
	}
}

/* The queues' state bits and the modem inputs (a three-wire UART: carrier and DSR always on). */
void
RPiPL011Serial::queueStates(void)
{
	UInt32 s = PD_RS232_S_CAR | PD_RS232_S_DSR;
	if (!fRTSCTS || !(fState & PD_S_ACTIVE) || (rd(FR) & FR_CTS)) {
		s |= PD_RS232_S_CTS;
	}
	s |= fTx.count == 0 ? PD_S_TXQ_EMPTY : 0;
	s |= fTx.count <= fTxLow ? PD_S_TXQ_LOW_WATER : 0;
	s |= fTx.count >= fTxHigh ? PD_S_TXQ_HIGH_WATER : 0;
	s |= fTx.count == kQueueSize ? PD_S_TXQ_FULL : 0;
	s |= fRx.count == 0 ? PD_S_RXQ_EMPTY : 0;
	s |= fRx.count <= fRxLow ? PD_S_RXQ_LOW_WATER : 0;
	s |= fRx.count >= fRxHigh ? PD_S_RXQ_HIGH_WATER : 0;
	s |= fRx.count == kQueueSize ? PD_S_RXQ_FULL : 0;
	if ((fState & PD_S_ACTIVE) && (fTx.count > 0 || (rd(FR) & FR_BUSY))) {
		s |= PD_S_TX_BUSY;
	}
	changeState(s, PD_S_TXQ_MASK | PD_S_RXQ_MASK | PD_S_TX_BUSY | PD_RS232_S_CAR | PD_RS232_S_DSR | PD_RS232_S_CTS);
}

/* Both the primary ISR and timeout recovery hold fRxLock while reading DR. */
void
RPiPL011Serial::captureRx(void)
{
	for (unsigned i = 0; i < 256 && fIRQRx.room() > 0 && !(rd(FR) & FR_RXFE); i++) {
		uint32_t data = rd(DR);
		if (data & DR_OE) { fOverruns++; }
		if (data & (DR_FE | DR_PE | DR_BE)) { fErrors++; continue; }
		uint8_t byte = (uint8_t)data;
		fIRQRx.put(&byte, 1);
	}
}

/* Received bytes into the ring while there is room (else left in the FIFO, the receive interrupts off). */
void
RPiPL011Serial::drainRx(void)
{
	uint32_t got = 0;
	if (fInterruptAdded) {
		IOInterruptState irqState = IOSimpleLockLockDisableInterrupt(fRxLock);
		/* Recover short FIFOs even if a timeout interrupt was lost/coalesced. */
		if (fIRQAccepting) {
			uint32_t before = fIRQRx.count;
			captureRx();
			fPollBytes += fIRQRx.count - before;
		}
		while (fIRQRx.count > 0 && fRx.room() > 0) {
			uint8_t byte;
			fIRQRx.get(&byte, 1);
			fRx.put(&byte, 1);
			got++;
		}
		if (fIRQRx.room() > 0) { fImsc |= IM_RX | IM_RT | IM_OE; }
		else { fImsc &= ~(IM_RX | IM_RT | IM_OE); }
		IOSimpleLockUnlockEnableInterrupt(fRxLock, irqState);
		if (got > 0) { fGate->commandWakeup(&fRx, false); }
		return;
	}
	while (!(rd(FR) & FR_RXFE)) {
		if (fRx.room() == 0) {
			fImsc &= ~(IM_RX | IM_RT | IM_OE);
			break;
		}
		uint32_t d = rd(DR);
		if (d & DR_OE) {
			fOverruns++;
		}
		if (d & (DR_FE | DR_PE | DR_BE)) {
			fErrors++;
			continue;
		}
		uint8_t b = (uint8_t)d;
		fRx.put(&b, 1);
		got++;
	}
	if (got > 0) {
		fGate->commandWakeup(&fRx, false);
	}
}

/* Discard both software stages and pending hardware bytes as one RX transaction. */
void
RPiPL011Serial::flushRx(void)
{
	IOInterruptState irqState = IOSimpleLockLockDisableInterrupt(fRxLock);
	fRx.clear();
	fIRQRx.clear();
	if (fState & PD_S_ACTIVE) {
		wr(IMSC, 0);
		for (unsigned i = 0; i < 256 && !(rd(FR) & FR_RXFE); i++) { (void)rd(DR); }
		wr(ICR, ICR_ALL);
		fImsc |= IM_RX | IM_RT | IM_OE;
		wr(IMSC, fInterruptAdded ? fImsc : 0);
	}
	IOSimpleLockUnlockEnableInterrupt(fRxLock, irqState);
	queueStates();
}

/* The transmit FIFO filled from the ring; the transmit interrupt on while the ring has more. */
void
RPiPL011Serial::fillTx(void)
{
	uint32_t sent = 0;
	while (fTx.count > 0 && !(rd(FR) & FR_TXFF)) {
		uint8_t b;
		fTx.get(&b, 1);
		wr(DR, b);
		sent++;
	}
	fImsc = fTx.count > 0 ? fImsc | IM_TX : fImsc & ~IM_TX;
	if (sent > 0) {
		fGate->commandWakeup(&fTx, false);
	}
}

void
RPiPL011Serial::service(void)
{
	if (!(fState & PD_S_ACTIVE) || fStopping) {
		return;
	}
	/* With an IRQ, only its capture path acknowledges RX/timeout events. */
	if (!fInterruptAdded) { wr(ICR, ICR_ALL); }
	drainRx();
	if (fTx.count > 0 && (fState & PD_S_TX_ENABLE)) {
		fillTx();
	} else {
		fImsc &= ~IM_TX;
	}
	queueStates();
	wr(IMSC, fInterruptAdded ? fImsc : 0);
}

/*
 * Primary interrupt: bounded RX capture avoids a FIFO overrun while the
 * workloop is scheduled. TX is masked until the workloop refills it.
 * signalInterrupt()+false is IOKit's SoftDMA path: keep the shared GIC
 * vector enabled after clearing/masking this device's interrupt condition.
 */
bool
RPiPL011Serial::filter(OSObject *owner, IOFilterInterruptEventSource *source)
{
	RPiPL011Serial *self = (RPiPL011Serial *)owner;
	IOInterruptState irqState = IOSimpleLockLockDisableInterrupt(self->fRxLock);
	if (!self->fIRQAccepting || self->rd(MIS) == 0) {
		IOSimpleLockUnlockEnableInterrupt(self->fRxLock, irqState);
		return false;
	}
	self->fIRQEvents++;
	uint32_t before = self->fIRQRx.count;
	self->captureRx();
	self->fIRQBytes += self->fIRQRx.count - before;
	uint32_t mask = self->rd(IMSC) & ~IM_TX;
	if (self->fIRQRx.room() == 0 || !(self->rd(FR) & FR_RXFE)) {
		mask &= ~(IM_RX | IM_RT | IM_OE);
	}
	self->wr(IMSC, mask);
	self->wr(ICR, ICR_ALL);
	IOSimpleLockUnlockEnableInterrupt(self->fRxLock, irqState);
	source->signalInterrupt();
	return false;
}

void
RPiPL011Serial::interrupt(OSObject *owner, IOInterruptEventSource *, int)
{
	((RPiPL011Serial *)owner)->service();
}

void
RPiPL011Serial::poll(OSObject *owner, IOTimerEventSource *timer)
{
	RPiPL011Serial *self = (RPiPL011Serial *)owner;
	self->service();
	if ((self->fState & PD_S_ACTIVE) && !self->fStopping) {
		timer->setTimeoutMS(kPollMs);
	}
}

#pragma mark IOSerialDriverSync

IOReturn
RPiPL011Serial::acquirePort(bool sleep, void *)
{
	return fGate->runActionBlock(^IOReturn {
		while (fState & PD_S_ACQUIRED) {
			if (!sleep) {
				return kIOReturnExclusiveAccess;
			}
			fWatchMask |= PD_S_ACQUIRED;
			if (fGate->commandSleep(&fState, THREAD_INTERRUPTIBLE) == THREAD_INTERRUPTED) {
				return kIOReturnIPCError;
			}
			if (fStopping) {
				return kIOReturnOffline;
			}
		}
		if (fStopping) {
			return kIOReturnOffline;
		}
		fState = PD_S_ACQUIRED;
		queueStates();
		return kIOReturnSuccess;
	});
}

IOReturn
RPiPL011Serial::releasePort(void *)
{
	return fGate->runActionBlock(^IOReturn {
		if (!(fState & PD_S_ACQUIRED)) {
			return kIOReturnNotOpen;
		}
		activate(false);
		/* Quiesced RX counters: useful for diagnosing overruns without ISR allocations. */
		setProperty("RXOverruns", fOverruns, 32);
		setProperty("RXErrors", fErrors, 32);
		setProperty("RXIRQEvents", fIRQEvents, 32);
		setProperty("RXIRQBytes", fIRQBytes, 32);
		setProperty("RXPolledBytes", fPollBytes, 32);
		fState = 0;
		fWatchMask = 0;
		fGate->commandWakeup(&fState, false);
		return kIOReturnSuccess;
	});
}

UInt32
RPiPL011Serial::getState(void *)
{
	UInt32 s = 0, *out = &s;
	fGate->runActionBlock(^IOReturn {
		queueStates();
		*out = fState;
		return kIOReturnSuccess;
	});
	return s;
}

IOReturn
RPiPL011Serial::setState(UInt32 state, UInt32 mask, void *)
{
	if (mask & (PD_S_ACQUIRED | PD_S_ACTIVE | ~EXTERNAL_MASK)) {
		return kIOReturnBadArgument;
	}
	return fGate->runActionBlock(^IOReturn {
		if (!(fState & PD_S_ACQUIRED)) {
			return kIOReturnNotOpen;
		}
		changeState(state, mask);
		/* Transmitting allowed again: what waits in the ring goes. */
		if ((fState & PD_S_ACTIVE) && (fState & PD_S_TX_ENABLE) && fTx.count > 0) {
			fillTx();
			wr(IMSC, fInterruptAdded ? fImsc : 0);
		}
		return kIOReturnSuccess;
	});
}

/*
 * Until a bit of mask equals state's: then the state. Without ACTIVE in
 * mask, the port's deactivation ends the wait too (kIOReturnIOError).
 */
IOReturn
RPiPL011Serial::watchState(UInt32 *state, UInt32 mask, void *)
{
	if (state == NULL) {
		return kIOReturnBadArgument;
	}
	return fGate->runActionBlock(^IOReturn {
		if (!(fState & PD_S_ACQUIRED)) {
			return kIOReturnNotOpen;
		}
		UInt32 want = *state, m = mask;
		bool implicit = false;
		if (!(m & (PD_S_ACQUIRED | PD_S_ACTIVE))) {
			want &= ~PD_S_ACTIVE;
			m |= PD_S_ACTIVE;
			implicit = true;
		}
		for (;;) {
			queueStates();
			UInt32 found = ~(want ^ fState) & m;
			if (found) {
				*state = fState;
				return implicit && (found & PD_S_ACTIVE) ? kIOReturnIOError : kIOReturnSuccess;
			}
			if (fStopping) {
				return kIOReturnOffline;
			}
			fWatchMask |= m;
			if (fGate->commandSleep(&fState, THREAD_INTERRUPTIBLE) == THREAD_INTERRUPTED) {
				return kIOReturnIPCError;
			}
		}
	});
}

UInt32
RPiPL011Serial::nextEvent(void *)
{
	return PD_E_EOQ;
}

IOReturn
RPiPL011Serial::executeEvent(UInt32 event, UInt32 data, void *)
{
	return fGate->runActionBlock(^IOReturn {
		return execute(event, data);
	});
}

IOReturn
RPiPL011Serial::execute(UInt32 event, UInt32 data)
{
	if (!(fState & PD_S_ACQUIRED)) {
		return kIOReturnNotOpen;
	}
	uint32_t ibrd, fbrd;
	switch (event) {
	case PD_E_ACTIVE:
		activate(data != 0);
		return kIOReturnSuccess;
	case PD_E_DATA_RATE:
		if (data >> 1 == 0) {
			return kIOReturnSuccess;        /* B0: the family drops DTR */
		}
		if (!divisor(data >> 1, &ibrd, &fbrd)) {
			return kIOReturnBadArgument;
		}
		fBaud = data >> 1;
		break;
	case PD_E_RX_DATA_RATE:
		return data == 0 || data >> 1 == fBaud ? kIOReturnSuccess : kIOReturnBadArgument;
	case PD_E_DATA_SIZE:
		if (data >> 1 < 5 || data >> 1 > 8) {
			return kIOReturnBadArgument;
		}
		fDataBits = data >> 1;
		break;
	case PD_E_RX_DATA_SIZE:
		return data == 0 || data >> 1 == fDataBits ? kIOReturnSuccess : kIOReturnBadArgument;
	case PD_E_DATA_INTEGRITY:
		if (data < PD_RS232_PARITY_NONE || data > PD_RS232_PARITY_SPACE) {
			return kIOReturnBadArgument;
		}
		fParity = data;
		break;
	case PD_E_RX_DATA_INTEGRITY:
		return data == PD_RS232_PARITY_DEFAULT || data == fParity ? kIOReturnSuccess : kIOReturnBadArgument;
	case PD_RS232_E_STOP_BITS:
		if (data != 2 && data != 4) {
			return kIOReturnBadArgument;
		}
		fStopHalfBits = data;
		break;
	case PD_RS232_E_RX_STOP_BITS:
		return data == 0 || data == fStopHalfBits ? kIOReturnSuccess : kIOReturnBadArgument;
	case PD_E_FLOW_CONTROL:
		/* RTS/CTS needs header pins, or the PL011 internal modem loopback. */
		fFlow = data & ((fRTSCTS || fLoopback) ? (PD_RS232_A_RFR | PD_RS232_A_CTS) : 0);
		break;
	case PD_RS232_E_LINE_BREAK:
		if (fState & PD_S_ACTIVE) {
			wr(LCR_H, data ? rd(LCR_H) | LCR_BRK : rd(LCR_H) & ~LCR_BRK);
			changeState(data ? PD_RS232_S_BRK : 0, PD_RS232_S_BRK);
		}
		return kIOReturnSuccess;
	case PD_E_TXQ_FLUSH:
		fTx.clear();
		queueStates();
		fGate->commandWakeup(&fTx, false);
		return kIOReturnSuccess;
	case PD_E_RXQ_FLUSH:
		flushRx();
		return kIOReturnSuccess;
	case PD_RS232_E_XON_BYTE:
		fXon = data;
		return kIOReturnSuccess;
	case PD_RS232_E_XOFF_BYTE:
		fXoff = data;
		return kIOReturnSuccess;
	case PD_E_DATA_LATENCY:
		fLatency = data;
		return kIOReturnSuccess;
	case PD_E_DELAY:
		fDelay = data;
		return kIOReturnSuccess;
	case PD_RS232_E_MIN_LATENCY:
		fMinLatency = data;
		return kIOReturnSuccess;
	case PD_E_TXQ_LOW_WATER:
	case PD_E_TXQ_HIGH_WATER:
	case PD_E_RXQ_LOW_WATER:
	case PD_E_RXQ_HIGH_WATER:
		if (data == 0 || data >= kQueueSize) {
			return kIOReturnBadArgument;
		}
		*(event == PD_E_TXQ_LOW_WATER ? &fTxLow : event == PD_E_TXQ_HIGH_WATER ? &fTxHigh :
		event == PD_E_RXQ_LOW_WATER ? &fRxLow : &fRxHigh) = data;
		queueStates();
		return kIOReturnSuccess;
	case PD_E_EXTERNAL_CLOCK_MODE:
		return data == 0 ? kIOReturnSuccess : kIOReturnBadArgument;
	default:
		return kIOReturnBadArgument;
	}
	/* A line setting: into the hardware if it runs. */
	if (fState & PD_S_ACTIVE) {
		program();
	}
	return kIOReturnSuccess;
}

IOReturn
RPiPL011Serial::requestEvent(UInt32 event, UInt32 *data, void *)
{
	if (data == NULL) {
		return kIOReturnBadArgument;
	}
	return fGate->runActionBlock(^IOReturn {
		return request(event, data);
	});
}

IOReturn
RPiPL011Serial::request(UInt32 event, UInt32 *data)
{
	switch (event) {
	case PD_E_ACTIVE: *data = (fState & PD_S_ACTIVE) != 0; break;
	case PD_E_DATA_RATE:
	case PD_E_RX_DATA_RATE: *data = fBaud << 1; break;
	case PD_E_DATA_SIZE:
	case PD_E_RX_DATA_SIZE: *data = fDataBits << 1; break;
	case PD_E_DATA_INTEGRITY:
	case PD_E_RX_DATA_INTEGRITY: *data = fParity; break;
	case PD_RS232_E_STOP_BITS:
	case PD_RS232_E_RX_STOP_BITS: *data = fStopHalfBits; break;
	case PD_E_FLOW_CONTROL: *data = fFlow; break;
	case PD_E_TXQ_SIZE:
	case PD_E_RXQ_SIZE: *data = kQueueSize; break;
	case PD_E_TXQ_LOW_WATER: *data = fTxLow; break;
	case PD_E_TXQ_HIGH_WATER: *data = fTxHigh; break;
	case PD_E_RXQ_LOW_WATER: *data = fRxLow; break;
	case PD_E_RXQ_HIGH_WATER: *data = fRxHigh; break;
	case PD_E_TXQ_AVAILABLE: *data = fTx.room(); break;
	case PD_E_RXQ_AVAILABLE: *data = fRx.count; break;
	case PD_RS232_E_XON_BYTE: *data = fXon; break;
	case PD_RS232_E_XOFF_BYTE: *data = fXoff; break;
	case PD_E_DATA_LATENCY: *data = fLatency; break;
	case PD_E_DELAY: *data = fDelay; break;
	case PD_RS232_E_MIN_LATENCY: *data = fMinLatency; break;
	case PD_RS232_E_LINE_BREAK: *data = (fState & PD_RS232_S_BRK) != 0; break;
	case PD_E_EXTERNAL_CLOCK_MODE: *data = 0; break;
	default: return kIOReturnBadArgument;
	}
	return kIOReturnSuccess;
}

IOReturn
RPiPL011Serial::enqueueEvent(UInt32 event, UInt32 data, bool, void *refCon)
{
	return executeEvent(event, data, refCon);
}

IOReturn
RPiPL011Serial::dequeueEvent(UInt32 *event, UInt32 *data, bool, void *)
{
	if (event == NULL || data == NULL) {
		return kIOReturnBadArgument;
	}
	*event = PD_E_EOQ;
	*data = 0;
	return kIOReturnSuccess;
}

/* Into the transmit ring, as much as fits; with sleep, all of it, waiting for room. */
IOReturn
RPiPL011Serial::enqueueData(UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep, void *)
{
	if (buffer == NULL || count == NULL) {
		return kIOReturnBadArgument;
	}
	*count = 0;
	return fGate->runActionBlock(^IOReturn {
		for (;;) {
			if (!(fState & PD_S_ACTIVE) || fStopping) {
				return kIOReturnNotOpen;
			}
			*count += fTx.put(buffer + *count, size - *count);
			if (fState & PD_S_TX_ENABLE) {
				fillTx();
				wr(IMSC, fInterruptAdded ? fImsc : 0);
			}
			queueStates();
			if (*count == size || !sleep) {
				return kIOReturnSuccess;
			}
			if (fGate->commandSleep(&fTx, THREAD_INTERRUPTIBLE) == THREAD_INTERRUPTED) {
				return kIOReturnIPCError;
			}
		}
	});
}

/* From the receive ring, up to size bytes, waiting for min of them. */
IOReturn
RPiPL011Serial::dequeueData(UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min, void *)
{
	if (buffer == NULL || count == NULL || min > size) {
		return kIOReturnBadArgument;
	}
	*count = 0;
	return fGate->runActionBlock(^IOReturn {
		for (;;) {
			if (!(fState & PD_S_ACTIVE) || fStopping) {
				return kIOReturnNotOpen;
			}
			*count += fRx.get(buffer + *count, size - *count);
			if (fRx.room() > 0 && !(fImsc & IM_RX)) {
				/* Room again: the bytes left in the FIFO come in. */
				fImsc |= IM_RX | IM_RT | IM_OE;
				drainRx();
				wr(IMSC, fInterruptAdded ? fImsc : 0);
				*count += fRx.get(buffer + *count, size - *count);
			}
			queueStates();
			if (*count >= min) {
				return kIOReturnSuccess;
			}
			if (fGate->commandSleep(&fRx, THREAD_INTERRUPTIBLE) == THREAD_INTERRUPTED) {
				return kIOReturnIPCError;
			}
		}
	});
}
