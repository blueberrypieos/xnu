/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <libkern/OSByteOrder.h>
#include <pexpert/pexpert.h>
#include <sys/param.h>

#include "RPiPlatform.h"
#include "RPiSDHCIRegs.h"
#include "../sdmmc/PIESDIO.h"

/*
 * The firmware's property tags for a clock's rate and the GPIO expander's
 * lines, whose line N is gpio 128 + N. A line is first configured with
 * SET_GPIO_CONFIG: {gpio, direction 1 for an output, polarity as
 * GET_GPIO_CONFIG gives it, no termination, state}.
 * The brcmstb GIO's banks of 32 lines,
 * 0x20 bytes apart: DATA, and IODIR with 1 for an input. The BCM2711's
 * function selects, 3 bits a line, and pulls, 2 bits a line: 0 none, 1 up,
 * 2 down. The BCM2712 D0 pin controller's function selects, 4 bits a pin,
 * and pulls, 2 bits a pin: 0 none, 1 down, 2 up.
 */
enum {
	FW_GET_CLOCK_RATE = 0x00030002, FW_CLOCK_EMMC = 1,
	FW_GET_GPIO_STATE = 0x00030041, FW_GET_GPIO_CONFIG = 0x00030043, FW_SET_GPIO_CONFIG = 0x00038043,
	FW_EXP_GPIO_BASE = 128,
	GIO_BANK = 0x20, GIO_DATA = 0x04, GIO_IODIR = 0x08,
	BCM2711_GPFSEL0 = 0x00, BCM2711_PUP_PDN0 = 0xe4,
	D0_PULL_NONE = 0, D0_PULL_DOWN = 1, D0_PULL_UP = 2,
	WL_OFF_MS = 20, WL_ON_MS = 150,
	SPIN_USEC = 1000,
};

/*
 * The Raspberry Pi's SDHCI controllers as an SDIO host, for the slot of
 * the board's WiFi chip: non-removable, with a brcm,bcm4329-fmac child.
 * The Pi 4's is the BCM2835 Arasan controller, whose writes must be 32
 * bits and two SD clocks apart at the slowest clock, and which has no
 * high-speed enable bit; its base clock is the firmware's EMMC clock. The
 * Pi 5's and the Pi 500's is the BCM2712's. Commands and data are polled,
 * data by PIO. The card interrupt is the only interrupt used. WL_ON, the
 * chip's power, is the vmmc-supply regulator's GIO line where the tree
 * gives one, else the firmware GPIO expander's line named WL_ON. The
 * boot-arg pie_sdio=0 leaves the slot to RPiSDHCI.
 */
class RPiSDHCISDIO : public PIESDIOHost
{
	OSDeclareDefaultStructors(RPiSDHCISDIO);

	IOMemoryMap *regsMap;
	volatile uint32_t *regs;
	IOWorkLoop *workLoop;
	IOFilterInterruptEventSource *intSource;
	uint32_t baseClock;
	bool arasan;
	bool clockAsked;
	enum { POWER_NONE, POWER_FIRMWARE, POWER_GIO } power;
	IOMemoryMap *powerMap;
	unsigned powerLine;
	bool powerActiveHigh;
	uint32_t powerOnMs;
	uint32_t powerPolarity;
	bool polarityKnown;
	unsigned powerSets;
	bool powerLogged;

	uint32_t rd(uint32_t off) { return regs[off / 4]; }
	void
	wr(uint32_t off, uint32_t v)
	{
		regs[off / 4] = v;
		if (arasan && sdioClock <= kSDIOClockInitHz) {
			IODelay(10);
		}
	}

	bool waitFor(uint32_t off, uint32_t mask, uint32_t want, uint32_t usec);
	uint32_t waitStatus(uint32_t mask, uint32_t usec);
	void failed(uint32_t status, uint32_t lines);
	void resetLines(uint32_t which);
	IOReturn command(uint32_t index, uint32_t arg, uint32_t flags, uint32_t xfer, uint32_t resp[4], bool dataLines);
	bool askClock(void);
	void findPower(IOService *provider);
	bool setWLOn(bool on);
	void applyPins(IORegistryEntry *node);
	void pins2711(IORegistryEntry *ctrl, IORegistryEntry *group);
	void pins2712d0(IORegistryEntry *ctrl, IORegistryEntry *group);
	bool interruptFilter(IOFilterInterruptEventSource *src);
	void interruptOccurred(IOInterruptEventSource *src, int count);

protected:
	virtual bool sdioPowerUp(void) APPLE_KEXT_OVERRIDE;
	virtual bool sdioSetClock(uint32_t hz) APPLE_KEXT_OVERRIDE;
	virtual void sdioSetBusWidth4(void) APPLE_KEXT_OVERRIDE;
	virtual bool sdioSetHighSpeed(bool on) APPLE_KEXT_OVERRIDE;
	virtual IOReturn sdioCommand(uint32_t index, uint32_t arg, SDIOResponse type, uint32_t resp[4]) APPLE_KEXT_OVERRIDE;
	virtual IOReturn sdioData(uint32_t arg, void *buf, uint32_t blksz, uint32_t nblk, bool write,
	    uint32_t *r5) APPLE_KEXT_OVERRIDE;
	virtual void sdioInterruptEnable(bool on) APPLE_KEXT_OVERRIDE;
	virtual void sdioRecover(void) APPLE_KEXT_OVERRIDE;

public:
	virtual IOService *probe(IOService *provider, SInt32 *score) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
};

#define super PIESDIOHost
OSDefineMetaClassAndStructors(RPiSDHCISDIO, PIESDIOHost);

#pragma mark Device tree

/* The node with this phandle, retained; the walk starts again if the plane changes under it. */
static IORegistryEntry *
nodeForPHandle(uint32_t phandle)
{
	IORegistryIterator *it = IORegistryIterator::iterateOver(IORegistryEntry::getRegistryRoot(), gIODTPlane,
	    kIORegistryIterateRecursively);
	IORegistryEntry *found = NULL;
	if (it == NULL) {
		return NULL;
	}
	do {
		OSSafeReleaseNULL(found);
		it->reset();
		for (IORegistryEntry *e; (e = it->getNextObject()) != NULL;) {
			OSData *apple = OSDynamicCast(OSData, e->getProperty("AAPL,phandle"));
			OSData *fdt = OSDynamicCast(OSData, e->getProperty("phandle"));
			if ((apple != NULL && apple->getLength() == 4 && OSReadLittleInt32(apple->getBytesNoCopy(), 0) == phandle) ||
			    (fdt != NULL && fdt->getLength() == 4 && OSReadBigInt32(fdt->getBytesNoCopy(), 0) == phandle)) {
				found = e;
				found->retain();
				break;
			}
		}
	} while (found == NULL && !it->isValid());
	it->release();
	return found;
}

/*
 * Whether a node's compatible, name, device_type or model is compat. Unlike
 * IODTMatchNubWithKeys(), it reads the properties of any entry: a node
 * that is not a nub compares only its name there.
 */
static bool
dtIs(IORegistryEntry *e, const char *compat)
{
	OSString *key = OSString::withCString(compat);
	bool is = key != NULL && IODTCompareNubName(e, key, NULL);

	OSSafeReleaseNULL(key);
	return is;
}

/* Cell i of a property, or false if it is shorter. */
static bool
propCell(IORegistryEntry *node, const char *prop, unsigned i, uint32_t *out)
{
	OSData *d = OSDynamicCast(OSData, node->getProperty(prop));
	if (d == NULL || d->getLength() < 4 * (i + 1)) {
		return false;
	}
	*out = OSReadBigInt32(d->getBytesNoCopy(), 4 * i);
	return true;
}

static IORegistryEntry *
phandleProp(IORegistryEntry *node, const char *prop, unsigned i)
{
	uint32_t ph;
	return propCell(node, prop, i, &ph) ? nodeForPHandle(ph) : NULL;
}

/*
 * The node's first register range, mapped. IODTResolveAddressing() keeps
 * the array it makes as the node's IODeviceMemory property and does not
 * give the caller a reference to it.
 */
static IOMemoryMap *
mapNode(IORegistryEntry *node)
{
	OSArray *mem = OSDynamicCast(OSArray, node->getProperty(gIODeviceMemoryKey));
	if (mem == NULL) {
		mem = IODTResolveAddressing(node, "reg", NULL);
	}
	IOMemoryDescriptor *range = mem != NULL ? OSDynamicCast(IOMemoryDescriptor, mem->getObject(0)) : NULL;
	return range != NULL ? range->map(kIOMapInhibitCache) : NULL;
}

/* Index of want in a list of strings, or -1. */
static int
stringIndex(OSData *list, const char *want)
{
	const char *p = list != NULL ? (const char *)list->getBytesNoCopy() : NULL;
	const char *end = p != NULL ? p + list->getLength() : NULL;
	for (int i = 0; p != NULL && p < end; i++) {
		size_t n = strnlen(p, (size_t)(end - p));
		if (strlen(want) == n && strncmp(p, want, n) == 0) {
			return i;
		}
		p += n + 1;
	}
	return -1;
}

static bool
hasFmacChild(IORegistryEntry *node)
{
	OSIterator *it = node->getChildIterator(gIODTPlane);
	bool found = false;
	for (IORegistryEntry *c; it != NULL && !found && (c = OSDynamicCast(IORegistryEntry, it->getNextObject())) != NULL;) {
		found = dtIs(c, "brcm,bcm4329-fmac");
	}
	OSSafeReleaseNULL(it);
	return found;
}

#pragma mark Pins

/* The BCM2711's pins of a brcm,pins group: function and pull as the group gives them. */
void
RPiSDHCISDIO::pins2711(IORegistryEntry *ctrl, IORegistryEntry *group)
{
	OSData *pins = OSDynamicCast(OSData, group->getProperty("brcm,pins"));
	OSData *fn = OSDynamicCast(OSData, group->getProperty("brcm,function"));
	OSData *pull = OSDynamicCast(OSData, group->getProperty("brcm,pull"));
	unsigned n = pins != NULL ? pins->getLength() / 4 : 0;
	IOMemoryMap *map = n != 0 && fn != NULL ? mapNode(ctrl) : NULL;
	if (map == NULL || n > 16 || (fn->getLength() != 4 && fn->getLength() != 4 * n) ||
	    (pull != NULL && pull->getLength() != 4 && pull->getLength() != 4 * n)) {
		IOLog("RPiSDHCISDIO: pins of %s: not understood; left as they are\n", group->getName());
		OSSafeReleaseNULL(map);
		return;
	}
	volatile uint32_t *r = (volatile uint32_t *)map->getVirtualAddress();
	char was[96] = "", now[96] = "";
	unsigned changed = 0;
	for (unsigned i = 0; i < n; i++) {
		uint32_t pin = OSReadBigInt32(pins->getBytesNoCopy(), 4 * i) & 63;
		uint32_t f = OSReadBigInt32(fn->getBytesNoCopy(), fn->getLength() == 4 ? 0 : 4 * i) & 7;
		uint32_t fsel = BCM2711_GPFSEL0 / 4 + pin / 10, fshift = 3 * (pin % 10);
		uint32_t pud = BCM2711_PUP_PDN0 / 4 + pin / 16, pshift = 2 * (pin % 16);
		uint32_t oldF = (r[fsel] >> fshift) & 7, oldP = (r[pud] >> pshift) & 3;
		snprintf(was + strlen(was), sizeof(was) - strlen(was), " %u/%u", oldF, oldP);
		if (oldF != f) {
			r[fsel] = (r[fsel] & ~(7U << fshift)) | (f << fshift);
			changed++;
		}
		if (pull != NULL) {
			uint32_t v = OSReadBigInt32(pull->getBytesNoCopy(), pull->getLength() == 4 ? 0 : 4 * i);
			uint32_t p = v == 2 ? 1 : v == 1 ? 2 : 0;
			if (oldP != p) {
				r[pud] = (r[pud] & ~(3U << pshift)) | (p << pshift);
				changed++;
			}
		}
		snprintf(now + strlen(now), sizeof(now) - strlen(now), " %u/%u", (r[fsel] >> fshift) & 7,
		    (r[pud] >> pshift) & 3);
	}
	IOLog("RPiSDHCISDIO: pins of %s, function/pull:%s; %s\n", group->getName(), was,
	    changed != 0 ? "set to" : "unchanged");
	if (changed != 0) {
		IOLog("RPiSDHCISDIO: pins of %s, function/pull:%s\n", group->getName(), now);
	}
	map->release();
}

/*
 * The BCM2712 D0's pins of a group, or of its subgroups: gpio28 to gpio35,
 * whose functions are gpio, 0, and for gpio30 to gpio35 sd2, 1. Pin N's
 * function select is nibble (N - 8) % 8 of word (N - 8) / 8, and its pull
 * is slot (N + 57) % 15 of word (N + 57) / 15, as OpenBSD's bcmstbpinctrl
 * lists them.
 */
void
RPiSDHCISDIO::pins2712d0(IORegistryEntry *ctrl, IORegistryEntry *group)
{
	IOMemoryMap *map = mapNode(ctrl);
	if (map == NULL || map->getLength() < 0x20) {
		IOLog("RPiSDHCISDIO: pins of %s: no pin controller registers; left as they are\n", group->getName());
		OSSafeReleaseNULL(map);
		return;
	}
	volatile uint32_t *r = (volatile uint32_t *)map->getVirtualAddress();
	OSArray *leaves = OSArray::withCapacity(4);
	if (leaves == NULL) {
		map->release();
		return;
	}
	if (group->getProperty("pins") != NULL) {
		leaves->setObject(group);
	} else {
		OSIterator *it = group->getChildIterator(gIODTPlane);
		for (IORegistryEntry *c; it != NULL && (c = OSDynamicCast(IORegistryEntry, it->getNextObject())) != NULL;) {
			leaves->setObject(c);
		}
		OSSafeReleaseNULL(it);
	}
	for (unsigned l = 0; l < leaves->getCount(); l++) {
		IORegistryEntry *leaf = OSDynamicCast(IORegistryEntry, leaves->getObject(l));
		OSData *pins = OSDynamicCast(OSData, leaf->getProperty("pins"));
		OSData *fn = OSDynamicCast(OSData, leaf->getProperty("function"));
		const char *f = fn != NULL ? (const char *)fn->getBytesNoCopy() : "";
		size_t flen = fn != NULL ? strnlen(f, fn->getLength()) : 0;
		int pull = leaf->getProperty("bias-pull-up") != NULL ? D0_PULL_UP :
		    leaf->getProperty("bias-pull-down") != NULL ? D0_PULL_DOWN :
		    leaf->getProperty("bias-disable") != NULL ? D0_PULL_NONE : -1;
		const char *p = pins != NULL ? (const char *)pins->getBytesNoCopy() : NULL;
		const char *end = p != NULL ? p + pins->getLength() : NULL;
		for (; p != NULL && p < end; p += strnlen(p, (size_t)(end - p)) + 1) {
			unsigned pin = 0;
			for (const char *d = strncmp(p, "gpio", 4) == 0 ? p + 4 : ""; *d >= '0' && *d <= '9' && pin < 100; d++) {
				pin = pin * 10 + (unsigned)(*d - '0');
			}
			int sel = flen == 4 && strncmp(f, "gpio", 4) == 0 ? 0 :
			    flen == 3 && strncmp(f, "sd2", 3) == 0 && pin >= 30 ? 1 : -1;
			if (pin < 28 || pin > 35 || sel < 0) {
				IOLog("RPiSDHCISDIO: %s %.*s: %.*s not known here; left as it is\n", leaf->getName(),
				    (int)strnlen(p, (size_t)(end - p)), p, (int)flen, f);
				continue;
			}
			unsigned m = pin - 8, q = pin + 57;
			unsigned mw = m / 8, ms = 4 * (m % 8), pw = q / 15, ps = 2 * (q % 15);
			uint32_t oldSel = (r[mw] >> ms) & 0xf, oldPull = (r[pw] >> ps) & 3;
			bool change = oldSel != (uint32_t)sel || (pull >= 0 && oldPull != (uint32_t)pull);
			if (oldSel != (uint32_t)sel) {
				r[mw] = (r[mw] & ~(0xfU << ms)) | ((uint32_t)sel << ms);
			}
			if (pull >= 0 && oldPull != (uint32_t)pull) {
				r[pw] = (r[pw] & ~(3U << ps)) | ((uint32_t)pull << ps);
			}
			IOLog("RPiSDHCISDIO: %s gpio%u: function %u, pull %u%s\n", leaf->getName(), pin, oldSel, oldPull,
			    change ? "; set to" : "; unchanged");
			if (change) {
				IOLog("RPiSDHCISDIO: %s gpio%u: function %u, pull %u\n", leaf->getName(), pin, (r[mw] >> ms) & 0xf,
				    (r[pw] >> ps) & 3);
			}
		}
	}
	leaves->release();
	map->release();
}

/* The groups of node's pinctrl-0, where the pin controller is one of the two above. */
void
RPiSDHCISDIO::applyPins(IORegistryEntry *node)
{
	for (unsigned i = 0;; i++) {
		uint32_t ph;
		if (!propCell(node, "pinctrl-0", i, &ph)) {
			break;
		}
		IORegistryEntry *group = nodeForPHandle(ph);
		IORegistryEntry *ctrl = group != NULL ? group->getParentEntry(gIODTPlane) : NULL;
		if (ctrl != NULL && dtIs(ctrl, "brcm,bcm2711-gpio")) {
			pins2711(ctrl, group);
		} else if (ctrl != NULL && dtIs(ctrl, "brcm,bcm2712d0-pinctrl")) {
			pins2712d0(ctrl, group);
		} else {
			IOLog("RPiSDHCISDIO: %s: pin group 0x%x left as the firmware set it\n", node->getName(), ph);
		}
		OSSafeReleaseNULL(group);
	}
}

#pragma mark WL_ON

/* WL_ON: the vmmc-supply regulator's GIO line, else the firmware expander's line named WL_ON. */
void
RPiSDHCISDIO::findPower(IOService *provider)
{
	uint32_t line = 0, flags = 0, us = 0;
	IORegistryEntry *reg = phandleProp(provider, "vmmc-supply", 0);
	if (reg != NULL && dtIs(reg, "regulator-fixed")) {
		const char *prop = reg->getProperty("gpio") != NULL ? "gpio" : "gpios";
		IORegistryEntry *gio = phandleProp(reg, prop, 0);
		IOMemoryMap *map = NULL;
		if (gio != NULL && dtIs(gio, "brcm,brcmstb-gpio") && propCell(reg, prop, 1, &line) &&
		    propCell(reg, prop, 2, &flags) && line < 64 && (map = mapNode(gio)) != NULL &&
		    map->getLength() >= GIO_BANK * (line / 32) + GIO_IODIR + 4) {
			power = POWER_GIO;
			powerMap = map;
			map = NULL;
			powerLine = line;
			powerActiveHigh = reg->getProperty("enable-active-high") != NULL;
			powerOnMs = propCell(reg, "startup-delay-us", 0, &us) ? (us + 999) / 1000 : WL_ON_MS;
			applyPins(reg);
			RPiReserveGPIOLine(gio, line, "WL_ON (kernel)");
			IOLog("RPiSDHCISDIO: %s: WL_ON is line %u of %s, %s, %u ms to start\n", provider->getName(), line,
			    gio->getName(), powerActiveHigh ? "active high" : "active low", powerOnMs);
		}
		OSSafeReleaseNULL(map);
		OSSafeReleaseNULL(gio);
	}
	OSSafeReleaseNULL(reg);
	if (power != POWER_NONE) {
		return;
	}
	OSArray *nodes = RPiDTFindCompatible("raspberrypi,firmware-gpio");
	IORegistryEntry *exp = nodes != NULL ? OSDynamicCast(IORegistryEntry, nodes->getObject(0)) : NULL;
	int idx = exp != NULL ? stringIndex(OSDynamicCast(OSData, exp->getProperty("gpio-line-names")), "WL_ON") : -1;
	if (idx >= 0) {
		power = POWER_FIRMWARE;
		powerLine = (unsigned)idx;
		powerActiveHigh = true;
		powerOnMs = WL_ON_MS;
		IOLog("RPiSDHCISDIO: %s: WL_ON is the firmware expander's line %u\n", provider->getName(), powerLine);
	} else {
		IOLog("RPiSDHCISDIO: %s: no WL_ON line found; the chip is taken to be powered\n", provider->getName());
	}
	OSSafeReleaseNULL(nodes);
}

/*
 * The chip's power on or off; the first call logs the line as it was.
 * The expander's line is set only once its polarity has been read. False
 * if the line was not set.
 */
bool
RPiSDHCISDIO::setWLOn(bool on)
{
	bool high = on == powerActiveHigh;
	if (power == POWER_GIO) {
		volatile uint32_t *bank = (volatile uint32_t *)(powerMap->getVirtualAddress() + GIO_BANK * (powerLine / 32));
		uint32_t bit = 1U << (powerLine % 32);
		if (!powerLogged) {
			IOLog("RPiSDHCISDIO: WL_ON was %s, %s\n", (bank[GIO_IODIR / 4] & bit) ? "an input" : "an output",
			    (bank[GIO_DATA / 4] & bit) ? "high" : "low");
			powerLogged = true;
		}
		RPiGPIOUpdate(&bank[GIO_DATA / 4], high ? 0 : bit, high ? bit : 0);
		RPiGPIOUpdate(&bank[GIO_IODIR / 4], bit, 0);
		return true;
	}
	if (power != POWER_FIRMWARE) {
		return true;
	}
	uint32_t gpio = FW_EXP_GPIO_BASE + powerLine;
	uint32_t d[6] = { gpio, 0, 0, 0, 0, 0 };
	IOReturn r = kIOReturnSuccess;
	const char *level = "unreadable";
	if (!powerLogged) {
		r = RPiFirmwareProperty(FW_GET_GPIO_STATE, d, 2 * sizeof(d[0]));
		level = r == kIOReturnSuccess && d[0] == 0 ? (d[1] ? "high" : "low") : "unreadable";
	}
	for (int tries = 0; !polarityKnown && tries < 3; tries++) {
		d[0] = gpio;
		d[1] = d[2] = d[3] = d[4] = 0;
		r = RPiFirmwareProperty(FW_GET_GPIO_CONFIG, d, 5 * sizeof(d[0]));
		if (r == kIOReturnSuccess && d[0] == 0) {
			powerPolarity = d[2];
			polarityKnown = true;
			if (!powerLogged) {
				IOLog("RPiSDHCISDIO: WL_ON was %s, %s, polarity %u\n", level, d[1] ? "an output" : "an input",
				    powerPolarity);
				powerLogged = true;
			}
		} else {
			IOSleep(10);
		}
	}
	if (!polarityKnown) {
		IOLog("RPiSDHCISDIO: WL_ON's configuration does not read: 0x%x, status 0x%x\n", r, d[0]);
		return false;
	}
	d[0] = gpio;
	d[1] = 1;
	d[2] = powerPolarity;
	d[3] = d[4] = 0;
	d[5] = high ? 1 : 0;
	r = RPiFirmwareProperty(FW_SET_GPIO_CONFIG, d, sizeof(d));
	if (r != kIOReturnSuccess || d[0] != 0) {
		IOLog("RPiSDHCISDIO: setting WL_ON %s failed: 0x%x, status 0x%x\n", on ? "on" : "off", r, d[0]);
		return false;
	}
	if (powerSets < 2) {
		d[0] = gpio;
		d[1] = 0;
		r = RPiFirmwareProperty(FW_GET_GPIO_STATE, d, 2 * sizeof(d[0]));
		IOLog("RPiSDHCISDIO: WL_ON is %s\n", r == kIOReturnSuccess && d[0] == 0 ? (d[1] ? "high" : "low") :
		    "unreadable");
	}
	powerSets++;
	return true;
}

#pragma mark Controller

bool
RPiSDHCISDIO::waitFor(uint32_t off, uint32_t mask, uint32_t want, uint32_t usec)
{
	for (uint32_t t = 0; t < usec; t += 10) {
		if ((rd(off) & mask) == want) {
			return true;
		}
		IODelay(10);
	}
	return (rd(off) & mask) == want;
}

/* The status once a bit of mask or an error is up, or 0 after usec: spinning at first, then sleeping. */
uint32_t
RPiSDHCISDIO::waitStatus(uint32_t mask, uint32_t usec)
{
	for (uint32_t t = 0;;) {
		uint32_t st = rd(SDHCI_INT_STATUS);
		if (st & (mask | INT_ERROR)) {
			return st;
		}
		if (t >= usec) {
			return 0;
		}
		if (t < SPIN_USEC) {
			IODelay(10);
			t += 10;
		} else {
			IOSleep(1);
			t += 1000;
		}
	}
}

void
RPiSDHCISDIO::resetLines(uint32_t which)
{
	wr(SDHCI_CLOCK_CONTROL, rd(SDHCI_CLOCK_CONTROL) | which);
	waitFor(SDHCI_CLOCK_CONTROL, which, 0, 100000);
}

/* The status bits cleared, all but the card interrupt, and the lines reset. */
void
RPiSDHCISDIO::failed(uint32_t status, uint32_t lines)
{
	wr(SDHCI_INT_STATUS, (status != 0 ? status : 0xFFFFFFFFU) & ~INT_CARD);
	resetLines(lines);
}

/* One command, polled; a CMD52 does not wait for the data lines. */
IOReturn
RPiSDHCISDIO::command(uint32_t index, uint32_t arg, uint32_t flags, uint32_t xfer, uint32_t resp[4], bool dataLines)
{
	uint32_t inhibit = PRESENT_CMD_INHIBIT | (dataLines ? PRESENT_DAT_INHIBIT : 0);
	if (!waitFor(SDHCI_PRESENT_STATE, inhibit, 0, 100000)) {
		resetLines(RESET_CMD | RESET_DATA);
		return kIOReturnBusy;
	}
	wr(SDHCI_INT_STATUS, ~INT_CARD);
	wr(SDHCI_ARGUMENT, arg);
	wr(SDHCI_CMD_XFER, ((index << 8 | flags) << 16) | xfer);
	uint32_t st = waitStatus(INT_CMD_DONE, 1000000);
	if (!(st & INT_CMD_DONE) || (st & INT_ERROR)) {
		failed(st, RESET_CMD | RESET_DATA);
		return st == 0 || (st & INT_CMD_TIMEOUT) ? kIOReturnTimeout : kIOReturnIOError;
	}
	wr(SDHCI_INT_STATUS, INT_CMD_DONE);
	if (resp != NULL) {
		for (int i = 0; i < 4; i++) {
			resp[i] = rd(SDHCI_RESPONSE + 4 * i);
		}
	}
	return kIOReturnSuccess;
}

/*
 * The Arasan's base clock is the firmware's EMMC clock; its capabilities do
 * not give it. Without it the power-up fails, and the next one asks again.
 */
bool
RPiSDHCISDIO::askClock(void)
{
	if (!arasan || clockAsked) {
		return true;
	}
	uint32_t d[2] = { FW_CLOCK_EMMC, 0 };
	if (RPiFirmwareProperty(FW_GET_CLOCK_RATE, d, sizeof(d)) != kIOReturnSuccess || d[0] != FW_CLOCK_EMMC ||
	    d[1] == 0) {
		IOLog("RPiSDHCISDIO: the firmware did not give the EMMC clock\n");
		return false;
	}
	IOLog("RPiSDHCISDIO: the firmware's EMMC clock is %u kHz; the capabilities said %u kHz\n", d[1] / 1000,
	    baseClock / 1000);
	baseClock = d[1];
	clockAsked = true;
	return true;
}

#pragma mark PIESDIOHost

/* WL_ON off, the controller reset and powered at 3.3 V, then WL_ON on and the chip's start-up time. */
bool
RPiSDHCISDIO::sdioPowerUp(void)
{
	if (!askClock() || !setWLOn(false)) {
		return false;
	}
	sdioClock = 0;
	wr(SDHCI_CLOCK_CONTROL, RESET_ALL);
	if (!waitFor(SDHCI_CLOCK_CONTROL, RESET_ALL, 0, 100000)) {
		IOLog("RPiSDHCISDIO: controller reset timed out\n");
		return false;
	}
	wr(SDHCI_HOST_CONTROL, POWER_ON_3V3);
	wr(SDHCI_CLOCK_CONTROL, (rd(SDHCI_CLOCK_CONTROL) & ~(0xFU << 16)) | TIMEOUT_MAX);
	wr(SDHCI_INT_ENABLE, 0xFFFF00FFU);
	wr(SDHCI_SIGNAL_ENABLE, 0);
	IOSleep(WL_OFF_MS);
	if (!setWLOn(true)) {
		return false;
	}
	IOSleep(powerOnMs);
	return true;
}

/* The 10-bit divided clock: base / (2 n), or the base clock itself for n = 0. */
bool
RPiSDHCISDIO::sdioSetClock(uint32_t hz)
{
	uint32_t n = 0;
	if (baseClock > hz) {
		n = (baseClock + 2 * hz - 1) / (2 * hz);
		if (n > 0x3FF) {
			n = 0x3FF;
		}
	}
	uint32_t cc = rd(SDHCI_CLOCK_CONTROL) & ~0xFFFFU;
	wr(SDHCI_CLOCK_CONTROL, cc);
	cc |= ((n & 0xFF) << 8) | (((n >> 8) & 3) << 6) | CLOCK_INTERNAL_EN;
	wr(SDHCI_CLOCK_CONTROL, cc);
	if (!waitFor(SDHCI_CLOCK_CONTROL, CLOCK_STABLE, CLOCK_STABLE, 20000)) {
		IOLog("RPiSDHCISDIO: clock not stable\n");
		return false;
	}
	wr(SDHCI_CLOCK_CONTROL, cc | CLOCK_CARD_EN);
	sdioClock = n != 0 ? baseClock / (2 * n) : baseClock;
	IODelay(1000);
	return true;
}

void
RPiSDHCISDIO::sdioSetBusWidth4(void)
{
	wr(SDHCI_HOST_CONTROL, rd(SDHCI_HOST_CONTROL) | HC_DATA_4BIT);
}

/* The Arasan has no high-speed bit and runs high speed without it. */
bool
RPiSDHCISDIO::sdioSetHighSpeed(bool on)
{
	if (!arasan) {
		uint32_t hc = rd(SDHCI_HOST_CONTROL);
		wr(SDHCI_HOST_CONTROL, on ? hc | HC_HIGH_SPEED : hc & ~HC_HIGH_SPEED);
	}
	return true;
}

IOReturn
RPiSDHCISDIO::sdioCommand(uint32_t index, uint32_t arg, SDIOResponse type, uint32_t resp[4])
{
	static const uint32_t flags[] = { R_NONE, R1, R3, R1 };
	return command(index, arg, flags[type], 0, resp, false);
}

/* CMD53 by PIO, a block at a time; a partial last word is moved whole. */
IOReturn
RPiSDHCISDIO::sdioData(uint32_t arg, void *buf, uint32_t blksz, uint32_t nblk, bool write, uint32_t *r5)
{
	uint32_t resp[4] = { 0, 0, 0, 0 };
	uint32_t xfer = XFER_BLOCK_COUNT_EN | (write ? 0 : XFER_READ) | (nblk > 1 ? XFER_MULTI : 0);
	wr(SDHCI_BLOCK, (nblk << 16) | blksz);
	IOReturn ret = command(53, arg, R1 | CMD_DATA, xfer, resp, true);
	*r5 = resp[0];
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint8_t *p = (uint8_t *)buf;
	uint32_t ready = write ? INT_WRITE_READY : INT_READ_READY;
	for (uint32_t b = 0; b < nblk; b++, p += blksz) {
		uint32_t st = waitStatus(ready, 1000000);
		if (!(st & ready) || (st & INT_ERROR)) {
			failed(st, RESET_CMD | RESET_DATA);
			IOLog("RPiSDHCISDIO: CMD53 %s, block %u of %u: status 0x%08x\n", write ? "write" : "read", b, nblk, st);
			return st == 0 ? kIOReturnTimeout : kIOReturnIOError;
		}
		wr(SDHCI_INT_STATUS, ready);
		for (uint32_t i = 0; i < blksz; i += 4) {
			uint32_t w = 0, n = MIN(4U, blksz - i);
			if (write) {
				bcopy(p + i, &w, n);
				wr(SDHCI_BUFFER, w);
			} else {
				w = rd(SDHCI_BUFFER);
				bcopy(&w, p + i, n);
			}
		}
	}
	uint32_t st = waitStatus(INT_XFER_DONE, write ? 2000000 : 1000000);
	if (!(st & INT_XFER_DONE) || (st & INT_ERROR)) {
		failed(st, RESET_CMD | RESET_DATA);
		IOLog("RPiSDHCISDIO: CMD53 %s of %u bytes did not complete: status 0x%08x\n", write ? "write" : "read",
		    blksz * nblk, st);
		return st == 0 ? kIOReturnTimeout : kIOReturnIOError;
	}
	wr(SDHCI_INT_STATUS, INT_XFER_DONE);
	return kIOReturnSuccess;
}

/* The card interrupt's status and signal enables; it is the only signal enabled. */
void
RPiSDHCISDIO::sdioInterruptEnable(bool on)
{
	uint32_t ie = rd(SDHCI_INT_ENABLE);
	wr(SDHCI_INT_ENABLE, on ? ie | INT_CARD : ie & ~INT_CARD);
	wr(SDHCI_SIGNAL_ENABLE, on ? INT_CARD : 0);
}

void
RPiSDHCISDIO::sdioRecover(void)
{
	failed(0, RESET_CMD | RESET_DATA);
}

/* At interrupt level: the card's interrupt, masked here, or another controller's on a shared line. */
bool
RPiSDHCISDIO::interruptFilter(IOFilterInterruptEventSource *src)
{
	(void)src;
	if (regs == NULL || !(regs[SDHCI_SIGNAL_ENABLE / 4] & INT_CARD) || !(regs[SDHCI_INT_STATUS / 4] & INT_CARD)) {
		return false;
	}
	regs[SDHCI_SIGNAL_ENABLE / 4] = 0;
	return true;
}

void
RPiSDHCISDIO::interruptOccurred(IOInterruptEventSource *src, int count)
{
	(void)src;
	(void)count;
	sdioInterrupt();
}

#pragma mark IOService

IOService *
RPiSDHCISDIO::probe(IOService *provider, SInt32 *score)
{
	uint32_t on = 1;
	if ((PE_parse_boot_argn("pie_sdio", &on, sizeof(on)) && on == 0) || provider->getProperty("non-removable") == NULL ||
	    !hasFmacChild(provider)) {
		return NULL;
	}
	return super::probe(provider, score);
}

bool
RPiSDHCISDIO::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		IOLog("RPiSDHCISDIO: %s: cannot map the registers\n", provider->getName());
		return false;
	}
	regs = (volatile uint32_t *)regsMap->getVirtualAddress();
	arasan = dtIs(provider, "brcm,bcm2835-sdhci");
	uint32_t caps = rd(SDHCI_CAPABILITIES), hz = 0;
	baseClock = ((caps >> 8) & 0xFF) * 1000000;
	IORegistryEntry *clk = phandleProp(provider, "clocks", 0);
	if (clk != NULL && dtIs(clk, "fixed-clock") && propCell(clk, "clock-frequency", 0, &hz) && hz != 0) {
		baseClock = hz;
	}
	OSSafeReleaseNULL(clk);
	if (baseClock == 0) {
		baseClock = 100000000;
	}
	IOLog("RPiSDHCISDIO: %s at 0x%llx, SDHCI version %u, base clock %u kHz, present state 0x%08x\n",
	    provider->getName(), regsMap->getPhysicalAddress(), (rd(SDHCI_VERSION) >> 16) & 0xFF, baseClock / 1000,
	    rd(SDHCI_PRESENT_STATE));
	applyPins(provider);
	findPower(provider);

	workLoop = IOWorkLoop::workLoop();
	if (workLoop != NULL) {
		intSource = IOFilterInterruptEventSource::filterInterruptEventSource(this,
		    OSMemberFunctionCast(IOInterruptEventAction, this, &RPiSDHCISDIO::interruptOccurred),
		    OSMemberFunctionCast(IOFilterInterruptEventSource::Filter, this, &RPiSDHCISDIO::interruptFilter),
		    provider, 0);
	}
	if (intSource == NULL || workLoop->addEventSource(intSource) != kIOReturnSuccess) {
		IOLog("RPiSDHCISDIO: %s: no interrupt; the card's interrupt needs sdio_poll=1\n", provider->getName());
		OSSafeReleaseNULL(intSource);
	} else {
		wr(SDHCI_SIGNAL_ENABLE, 0);
		intSource->enable();
	}
	return sdioStartBus();
}

/*
 * The card interrupt masked, then the event source removed, which
 * unregisters it: a disabled registration would keep a shared line, as
 * the Pi 4's with the SD card's controller, masked for everyone.
 */
void
RPiSDHCISDIO::stop(IOService *provider)
{
	sdioStopBus();
	if (regs != NULL) {
		wr(SDHCI_SIGNAL_ENABLE, 0);
		wr(SDHCI_INT_ENABLE, rd(SDHCI_INT_ENABLE) & ~INT_CARD);
	}
	if (intSource != NULL) {
		intSource->disable();
		if (workLoop != NULL) {
			workLoop->removeEventSource(intSource);
		}
		OSSafeReleaseNULL(intSource);
	}
	super::stop(provider);
}

void
RPiSDHCISDIO::free(void)
{
	if (workLoop != NULL && intSource != NULL) {
		workLoop->removeEventSource(intSource);
	}
	OSSafeReleaseNULL(intSource);
	OSSafeReleaseNULL(workLoop);
	OSSafeReleaseNULL(powerMap);
	OSSafeReleaseNULL(regsMap);
	super::free();
}
