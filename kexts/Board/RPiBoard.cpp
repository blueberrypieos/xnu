/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * RPiBoard: the board's LEDs and its power button, as the device tree
 * describes them (Linux's gpio-leds and gpio-keys).
 *
 * The LEDs: the gpio-leds node's children whose names say act or pwr,
 * each on a GPIO line (gpios = <phandle line flags>, flags bit 0 active
 * low): a line of one of piegpio's controllers, driven through its
 * platform functions (<PieGPIOKernel.h>), or of the firmware's GPIO
 * expander (raspberrypi,firmware-gpio, the Pi 4's PWR LED), driven
 * through the mailbox: SET_GPIO_STATE 0x00038041 and GET_GPIO_STATE
 * 0x00030041, each {gpio, state}, the expander's line N being gpio 128 + N.
 * Each follows a trigger, the tree's linux,default-trigger at first: none
 * (off, or on if default-state says so), default-on, disk (mmc0, mmc1,
 * disk-activity: lit for 50 ms whenever the reads and writes counted by
 * any IOBlockStorageDriver's Statistics change, which covers the SD card,
 * NVMe and USB storage alike) or heartbeat (two blinks a second). The
 * hw.led.act and hw.led.pwr sysctls: brightness (read back from the line;
 * root may set it, which sets the trigger to none), trigger (root) and
 * toggles. As a panic begins (IOPlatformPanicAction), the ACT LED lights
 * and stays lit, by a write to its GPIO block's registers alone (the Pi
 * 4's GPSET/GPCLR, the BCM2712 GIO_AON bank's DATA): nothing else is safe
 * then. At a halt or a restart the ACT LED goes off; the PWR LED stays as
 * it is (the firmware has it next).
 *
 * The power button: a gpio-keys child whose linux,code is KEY_POWER (116),
 * the Pi 5's and the Pi 500's (the GIO's line 20, active low). piegpio
 * calls back at its edges (kPieGPIORequestEdges); once the line has held
 * for the key's debounce-interval (at least 20 ms), a press does what
 * hw.powerbutton.action says: halt (the default: init, sent SIGUSR2,
 * shuts down as halt(8) does), log or none. hw.powerbutton: present,
 * action, presses, and test_press (root: 1 acts as a press does).
 *
 * The firmware's expander is not QEMU's (its revision says): there the
 * expander's LEDs are left alone.
 */

#include <IOKit/IOCommandGate.h>
#include <IOKit/IOKitKeysPrivate.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMessage.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOService.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <libkern/OSByteOrder.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <sys/signal.h>
#include <sys/sysctl.h>

#include <pie/rpi.h>

#include "PieGPIOKernel.h"

enum {
	FW_GET_FIRMWARE_REVISION = 0x00000001,
	FW_VIDEOCORE_SINCE = 1546300800,        /* 2019-01-01 UTC: older than any BCM2711 or BCM2712 firmware */
	FW_GET_GPIO_STATE = 0x00030041, FW_SET_GPIO_STATE = 0x00038041,
	FW_EXP_GPIO_BASE = 128,
	KEY_POWER = 116,
	TICK_MS = 50,
	BUTTON_MIN_DEBOUNCE_MS = 20,
	/* The BCM2711's GPIO (GPSET0/1, GPCLR0/1) and the BCM2712 GIO bank's DATA. */
	BCM2711_GPSET0 = 0x1c, BCM2711_GPCLR0 = 0x28, BRCMSTB_BANK = 0x20, BRCMSTB_DATA = 0x04,
};

enum { TRIG_NONE, TRIG_ON, TRIG_DISK, TRIG_HEARTBEAT, TRIG_COUNT };
static const char *const kTriggerNames[TRIG_COUNT] = { "none", "default-on", "disk", "heartbeat" };

enum { ACTION_HALT, ACTION_LOG, ACTION_NONE, ACTION_COUNT };
static const char *const kActionNames[ACTION_COUNT] = { "halt", "log", "none" };

/* The heartbeat's pattern, in 50 ms ticks of a second: on, off, on, off. */
static inline bool
heartbeatOn(unsigned phase)
{
	return phase < 2 || (phase >= 5 && phase < 7);
}

enum { LED_ACT, LED_PWR, LED_COUNT };
static const char *const kLEDNames[LED_COUNT] = { "act", "pwr" };

class RPiBoard : public IOService
{
	OSDeclareDefaultStructors(RPiBoard);

public:
	struct LED {
		bool present, firmware, activeLow;
		uint8_t cells[12];
		uint32_t line;
		IOService *controller;          /* retained; piegpio's */
		char consumer[kPieGPIONameSize];
		int trigger;
		bool on, set;
		uint64_t toggles;
		/* The panic's way to light it: its GPIO block's registers. */
		IOMemoryMap *panicMap;
		volatile uint32_t *panicSet, *panicClear;
		uint32_t panicBit;
		bool panicRMW;
	};

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn callPlatformFunction(const OSSymbol *functionName, bool waitForFunction, void *param1,
	    void *param2, void *param3, void *param4) APPLE_KEXT_OVERRIDE;

	int brightness(unsigned i, bool *on);
	int setBrightness(unsigned i, bool on);
	int setTrigger(unsigned i, int trigger);
	int setAction(int action);
	void press(const char *why);

	LED fLEDs[LED_COUNT];
	bool fButton;
	int fAction;
	uint64_t fPresses;
	IOCommandGate *fGate;

private:
	IOWorkLoop *fWorkLoop;
	IOTimerEventSource *fTimer, *fButtonTimer;
	IONotifier *fHaltNotifier;
	bool fQEMU, fSysctls, fHalting, fPressed;
	bool fGateAdded, fTimerAdded, fButtonTimerAdded, fClaimed;
	unsigned fPhase;
	uint64_t fDiskOps;
	uint8_t fButtonCells[12];
	unsigned fDebounceMs;

	void cleanup(void);
	void findLEDs(void);
	void findButton(void);
	bool setUpPanic(LED &l);
	void drive(LED &l, bool on);
	void initialState(LED &l, IORegistryEntry *node);
	void tick(void);
	uint64_t diskOps(void);
	void buttonSettled(void);
	void registerSysctls(void);
	void unregisterSysctls(void);
	static void timerFired(OSObject *owner, IOTimerEventSource *sender);
	static void buttonTimerFired(OSObject *owner, IOTimerEventSource *sender);
	static void buttonEdge(void *refcon, uint32_t line, bool rising);
	static IOReturn haltRestart(void *target, void *refCon, UInt32 messageType, IOService *provider,
	    void *messageArgument, vm_size_t argSize);
};

OSDefineMetaClassAndStructors(RPiBoard, IOService);
#define super IOService

/* The instance the sysctls reach; set while they are registered. */
static RPiBoard *sBoard;

#pragma mark device tree

static OSData *
dtData(IORegistryEntry *e, const char *key)
{
	return OSDynamicCast(OSData, e->getProperty(key));
}

static bool
dtOkay(IORegistryEntry *e)
{
	OSData *status = dtData(e, "status");
	if (status == NULL) {
		return true;
	}
	const char *s = (const char *)status->getBytesNoCopy();
	size_t len = strnlen(s, status->getLength());
	return (len == 4 && !strncmp(s, "okay", 4)) || (len == 2 && !strncmp(s, "ok", 2));
}

/* Whether a string (list) property has value among its strings. */
static bool
dtString(IORegistryEntry *e, const char *key, const char *value)
{
	OSData *d = dtData(e, key);
	if (d == NULL) {
		return false;
	}
	const char *p = (const char *)d->getBytesNoCopy(), *end = p + d->getLength();
	for (; p < end; p += strnlen(p, (size_t)(end - p)) + 1) {
		if (strncmp(p, value, (size_t)(end - p)) == 0) {
			return true;
		}
	}
	return false;
}

/* Whether s has sub in it (the kernel's strnstr is private). */
static bool
contains(const char *s, const char *sub)
{
	size_t n = strlen(sub);
	for (; *s != '\0'; s++) {
		if (strncmp(s, sub, n) == 0) {
			return true;
		}
	}
	return false;
}

static bool
dtU32(IORegistryEntry *e, const char *key, uint32_t *v)
{
	OSData *d = dtData(e, key);
	if (d == NULL || d->getLength() < 4) {
		return false;
	}
	*v = OSReadBigInt32(d->getBytesNoCopy(), 0);
	return true;
}

/* The children of every okay node with that compatible (retained), or NULL. */
static OSArray *
childrenOf(const char *compatible)
{
	OSArray *nodes = RPiDTFindCompatible(compatible);
	OSArray *children = OSArray::withCapacity(4);
	for (unsigned n = 0; nodes != NULL && children != NULL && n < nodes->getCount(); n++) {
		IORegistryEntry *node = OSDynamicCast(IORegistryEntry, nodes->getObject(n));
		if (node == NULL || !dtOkay(node)) {
			continue;
		}
		OSIterator *it = node->getChildIterator(gIODTPlane);
		IORegistryEntry *child;
		while (it != NULL && (child = OSDynamicCast(IORegistryEntry, it->getNextObject())) != NULL) {
			if (dtOkay(child)) {
				children->setObject(child);
			}
		}
		OSSafeReleaseNULL(it);
	}
	OSSafeReleaseNULL(nodes);
	return children;
}

#pragma mark start and stop

bool
RPiBoard::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	if (!OSCompareAndSwapPtr(NULL, this, &sBoard)) { return false; }
	fClaimed = true;
	uint32_t revision = 0;
	fQEMU = RPiFirmwareProperty(FW_GET_FIRMWARE_REVISION, &revision, sizeof(revision)) != kIOReturnSuccess ||
	    revision < FW_VIDEOCORE_SINCE;
	fWorkLoop = IOWorkLoop::workLoop();
	fGate = fWorkLoop != NULL ? IOCommandGate::commandGate(this) : NULL;
	fTimer = fWorkLoop != NULL ? IOTimerEventSource::timerEventSource(this, timerFired) : NULL;
	fButtonTimer = fWorkLoop != NULL ? IOTimerEventSource::timerEventSource(this, buttonTimerFired) : NULL;
	if (fGate == NULL || fTimer == NULL || fButtonTimer == NULL) { return false; }
	if (fWorkLoop->addEventSource(fGate) != kIOReturnSuccess) { return false; }
	fGateAdded = true;
	if (fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess) { return false; }
	fTimerAdded = true;
	if (fWorkLoop->addEventSource(fButtonTimer) != kIOReturnSuccess) { return false; }
	fButtonTimerAdded = true;
	fGate->runActionBlock(^IOReturn {
		findLEDs();
		findButton();
		fDiskOps = diskOps();
		return kIOReturnSuccess;
	});
	if (fLEDs[LED_ACT].panicMap != NULL) {
		/* At kPEPanicBegin, callPlatformFunction(IOPlatformPanicAction, ...): registerService installs it. */
		setProperty(kIOPlatformPanicActionKey, 1000ULL, 32);
	}
	fHaltNotifier = registerPrioritySleepWakeInterest(haltRestart, this, NULL);
	if (fHaltNotifier == NULL) { return false; }
	registerSysctls();
	registerService();
	fGate->runActionBlock(^IOReturn {
		fTimer->setTimeoutMS(TICK_MS);
		return kIOReturnSuccess;
	});
	return true;
}

void
RPiBoard::cleanup(void)
{
	unregisterSysctls();
	if (fHaltNotifier != NULL) { fHaltNotifier->remove(); fHaltNotifier = NULL; }
	if (fGateAdded) {
		fGate->runActionBlock(^IOReturn {
			fHalting = true;
			if (fTimer != NULL) { fTimer->cancelTimeout(); }
			if (fButtonTimer != NULL) { fButtonTimer->cancelTimeout(); }
			return kIOReturnSuccess;
		});
	}
	/* Never hold our gate while waiting for the GPIO callback workloop. */
	if (fButton) { PieGPIOReleaseAll("pwr_button"); fButton = false; }
	if (fGateAdded) {
		/* An in-flight edge may have rearmed this timer before release finished. */
		fGate->runActionBlock(^IOReturn {
			if (fButtonTimer != NULL) { fButtonTimer->cancelTimeout(); }
			return kIOReturnSuccess;
		});
	}
	for (unsigned i = 0; i < LED_COUNT; i++) {
		LED &l = fLEDs[i];
		/* Panic action remains registered until detach: invalidate before unclaim. */
		l.panicSet = l.panicClear = NULL;
		__builtin_arm_dsb(0xf);
		if (l.present && l.set && !l.firmware) { PieGPIOReleaseAll(l.consumer); }
		l.present = false;
		OSSafeReleaseNULL(l.controller);
		OSSafeReleaseNULL(l.panicMap);
	}
	if (fTimerAdded) { fWorkLoop->removeEventSource(fTimer); fTimerAdded = false; }
	if (fButtonTimerAdded) { fWorkLoop->removeEventSource(fButtonTimer); fButtonTimerAdded = false; }
	if (fGateAdded) { fWorkLoop->removeEventSource(fGate); fGateAdded = false; }
	if (fClaimed) { OSCompareAndSwapPtr(this, NULL, &sBoard); fClaimed = false; }
}

void
RPiBoard::stop(IOService *provider)
{
	cleanup();
	super::stop(provider);
}

void
RPiBoard::free(void)
{
	cleanup();
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fButtonTimer);
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fWorkLoop);
	super::free();
}

#pragma mark LEDs

/* On the work loop, from start. */
void
RPiBoard::findLEDs(void)
{
	OSArray *children = childrenOf("gpio-leds");
	for (unsigned c = 0; children != NULL && c < children->getCount(); c++) {
		IORegistryEntry *node = (IORegistryEntry *)children->getObject(c);
		const char *name = node->getName(gIODTPlane);
		OSData *gpios = dtData(node, "gpios");
		int which = name == NULL ? -1 : contains(name, "act") ? LED_ACT : contains(name, "pwr") ? LED_PWR : -1;
		if (which < 0 || fLEDs[which].present || gpios == NULL || gpios->getLength() < 12) {
			if (name != NULL && which < 0) {
				IOLog("RPiBoard: LED %s: neither act nor pwr, left alone\n", name);
			}
			continue;
		}
		LED &l = fLEDs[which];
		bcopy(gpios->getBytesNoCopy(), l.cells, sizeof(l.cells));
		l.line = OSReadBigInt32(l.cells, 4);
		l.activeLow = OSReadBigInt32(l.cells, 8) & 1;
		strlcpy(l.consumer, name, sizeof(l.consumer));
		IORegistryEntry *gpio = PieGPIOCopyNodeForPHandle(OSReadBigInt32(l.cells, 0));
		l.firmware = gpio != NULL && dtString(gpio, "compatible", "raspberrypi,firmware-gpio");
		if (l.firmware && fQEMU) {
			IOLog("RPiBoard: LED %s: on the firmware's GPIO expander, which QEMU's firmware is not; left alone\n",
			    name);
		} else if (l.firmware) {
			l.present = true;
		} else if ((l.controller = PieGPIOCopyController(OSReadBigInt32(l.cells, 0), kPieGPIOWaitNs)) == NULL) {
			IOLog("RPiBoard: LED %s: no GPIO controller for it (piegpio not loaded?)\n", name);
		} else {
			l.present = true;
		}
		if (l.present) {
			initialState(l, node);
			if (!l.set) { l.present = false; OSSafeReleaseNULL(l.controller); OSSafeReleaseNULL(gpio); continue; }
			/* Panic writes are permitted only after the LED pin was claimed. */
			if (which == LED_ACT && !l.firmware) { setUpPanic(l); }
			IOLog("RPiBoard: LED %s: %s line %u%s, %s\n", name, l.firmware ? "the firmware's expander" : "GPIO",
			    l.line, l.activeLow ? " (active low)" : "", kTriggerNames[l.trigger]);
		}
		OSSafeReleaseNULL(gpio);
	}
	OSSafeReleaseNULL(children);
}

/* The tree's trigger and default state. */
void
RPiBoard::initialState(LED &l, IORegistryEntry *node)
{
	static const struct { const char *name; int trigger; } kTriggers[] = {
		{ "none", TRIG_NONE }, { "default-on", TRIG_ON }, { "mmc0", TRIG_DISK }, { "mmc1", TRIG_DISK },
		{ "disk-activity", TRIG_DISK }, { "heartbeat", TRIG_HEARTBEAT },
	};
	l.trigger = TRIG_NONE;
	OSData *t = dtData(node, "linux,default-trigger");
	if (t != NULL) {
		const char *s = (const char *)t->getBytesNoCopy();
		bool known = false;
		for (unsigned i = 0; i < sizeof(kTriggers) / sizeof(kTriggers[0]); i++) {
			if (strncmp(s, kTriggers[i].name, t->getLength()) == 0) {
				l.trigger = kTriggers[i].trigger;
				known = true;
			}
		}
		if (!known) {
			IOLog("RPiBoard: LED %s: trigger %.*s unknown here, none\n", l.consumer, (int)strnlen(s, t->getLength()), s);
		}
	}
	bool on = l.trigger == TRIG_ON || (l.trigger == TRIG_NONE && dtString(node, "default-state", "on"));
	drive(l, on);
}

/*
 * The ACT LED's GPIO block mapped, for the panic: the Pi 4's BCM2711 GPIO
 * (set and clear registers) or a BCM2712 GIO bank (its DATA register).
 */
bool
RPiBoard::setUpPanic(LED &l)
{
	IORegistryEntry *gpio = PieGPIOCopyNodeForPHandle(OSReadBigInt32(l.cells, 0));
	IOService *nub = OSDynamicCast(IOService, gpio);
	bool bcm2711 = gpio != NULL && dtString(gpio, "compatible", "brcm,bcm2711-gpio");
	bool brcmstb = gpio != NULL && dtString(gpio, "compatible", "brcm,brcmstb-gpio");
	if (nub != NULL && (bcm2711 || brcmstb)) {
		l.panicMap = nub->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache);
	}
	unsigned bank = l.line / 32;
	size_t need = bcm2711 ? BCM2711_GPCLR0 + 8 : BRCMSTB_BANK * bank + BRCMSTB_DATA + 4;
	if (l.panicMap != NULL && l.panicMap->getLength() >= need) {
		volatile uint32_t *regs = (volatile uint32_t *)l.panicMap->getVirtualAddress();
		l.panicBit = 1U << (l.line % 32);
		if (bcm2711) {
			l.panicSet = regs + (BCM2711_GPSET0 + 4 * bank) / 4;
			l.panicClear = regs + (BCM2711_GPCLR0 + 4 * bank) / 4;
		} else {
			l.panicSet = l.panicClear = regs + (BRCMSTB_BANK * bank + BRCMSTB_DATA) / 4;
			l.panicRMW = true;
		}
	} else {
		OSSafeReleaseNULL(l.panicMap);
	}
	OSSafeReleaseNULL(gpio);
	return l.panicMap != NULL;
}

/* On the work loop: the LED lit or not (logical; active low applied here). */
void
RPiBoard::drive(LED &l, bool on)
{
	if (!l.present || (l.set && l.on == on)) {
		return;
	}
	bool pin = on != l.activeLow;
	IOReturn ret;
	if (l.firmware) {
		uint32_t data[2] = { FW_EXP_GPIO_BASE + l.line, pin };
		ret = RPiFirmwareProperty(FW_SET_GPIO_STATE, data, sizeof(data));
	} else {
		ret = PieGPIOCall(l.controller, kPieGPIOSetOutput, (void *)(uintptr_t)l.line, (void *)(uintptr_t)pin,
		    l.consumer);
	}
	if (ret != kIOReturnSuccess) {
		if (!l.set) {
			IOLog("RPiBoard: LED %s: setting it failed: 0x%x\n", l.consumer, ret);
		}
		return;
	}
	if (l.set) {
		l.toggles++;
	}
	l.on = on;
	l.set = true;
}

int
RPiBoard::brightness(unsigned i, bool *on)
{
	LED &l = fLEDs[i];
	IOReturn ret = fGate->runActionBlock(^IOReturn {
		uint32_t pin = 0;
		IOReturn r;
		if (l.firmware) {
			uint32_t data[2] = { FW_EXP_GPIO_BASE + l.line, 0 };
			r = RPiFirmwareProperty(FW_GET_GPIO_STATE, data, sizeof(data));
			/* data[0] comes back as the firmware's status: 0 if it read the line. */
			if (r == kIOReturnSuccess && data[0] != 0) {
				r = kIOReturnIOError;
			}
			pin = data[1];
		} else {
			r = PieGPIOCall(l.controller, kPieGPIOGetValue, (void *)(uintptr_t)l.line, &pin, NULL);
		}
		*on = (pin != 0) != l.activeLow;
		return r;
	});
	return ret == kIOReturnSuccess ? 0 : EIO;
}

int
RPiBoard::setBrightness(unsigned i, bool on)
{
	IOReturn ret = fGate->runActionBlock(^IOReturn {
		LED &l = fLEDs[i];
		l.trigger = TRIG_NONE;
		drive(l, on);
		return l.set && l.on == on ? kIOReturnSuccess : kIOReturnIOError;
	});
	return ret == kIOReturnSuccess ? 0 : EIO;
}

int
RPiBoard::setTrigger(unsigned i, int trigger)
{
	if (trigger < 0 || trigger >= TRIG_COUNT) {
		return EINVAL;
	}
	fGate->runActionBlock(^IOReturn {
		LED &l = fLEDs[i];
		l.trigger = trigger;
		if (trigger == TRIG_NONE || trigger == TRIG_ON) {
			drive(l, trigger == TRIG_ON);
		}
		return kIOReturnSuccess;
	});
	IOLog("RPiBoard: LED %s: trigger %s (hw.led.%s.trigger)\n", fLEDs[i].consumer, kTriggerNames[trigger],
	    kLEDNames[i]);
	return 0;
}

/* The reads and writes every IOBlockStorageDriver has counted (its Statistics). */
uint64_t
RPiBoard::diskOps(void)
{
	OSDictionary *match = IOService::serviceMatching("IOBlockStorageDriver");
	OSIterator *it = match != NULL ? IOService::getMatchingServices(match) : NULL;
	uint64_t ops = 0;
	IOService *s;
	while (it != NULL && (s = OSDynamicCast(IOService, it->getNextObject())) != NULL) {
		OSDictionary *stats = OSDynamicCast(OSDictionary, s->getProperty("Statistics"));
		OSNumber *r = stats != NULL ? OSDynamicCast(OSNumber, stats->getObject("Operations (Read)")) : NULL;
		OSNumber *w = stats != NULL ? OSDynamicCast(OSNumber, stats->getObject("Operations (Write)")) : NULL;
		ops += (r != NULL ? r->unsigned64BitValue() : 0) + (w != NULL ? w->unsigned64BitValue() : 0);
	}
	OSSafeReleaseNULL(it);
	OSSafeReleaseNULL(match);
	return ops;
}

/* Every 50 ms, on the work loop: the disk and heartbeat triggers. */
void
RPiBoard::tick(void)
{
	if (fHalting) {
		return;
	}
	bool anyDisk = false;
	for (unsigned i = 0; i < LED_COUNT; i++) {
		anyDisk |= fLEDs[i].present && fLEDs[i].trigger == TRIG_DISK;
	}
	bool busy = false;
	if (anyDisk) {
		uint64_t ops = diskOps();
		busy = ops != fDiskOps;
		fDiskOps = ops;
	}
	fPhase = (fPhase + 1) % (1000 / TICK_MS);
	for (unsigned i = 0; i < LED_COUNT; i++) {
		LED &l = fLEDs[i];
		if (!l.present) {
			continue;
		}
		if (l.trigger == TRIG_DISK) {
			drive(l, busy);
		} else if (l.trigger == TRIG_HEARTBEAT) {
			drive(l, heartbeatOn(fPhase));
		}
	}
	fTimer->setTimeoutMS(TICK_MS);
}

void
RPiBoard::timerFired(OSObject *owner, IOTimerEventSource *)
{
	((RPiBoard *)owner)->tick();
}

/*
 * As a panic begins (IOPlatformPanicAction, param2 the kPEPanic* type):
 * the ACT LED lit, by its registers alone, without locks.
 */
IOReturn
RPiBoard::callPlatformFunction(const OSSymbol *functionName, bool waitForFunction, void *param1, void *param2,
    void *param3, void *param4)
{
	if (functionName->isEqualTo(kIOPlatformPanicActionKey)) {
		LED &l = fLEDs[LED_ACT];
		/* Each pointer read once: stop clears them before it unmaps. */
		volatile uint32_t *set = *(volatile uint32_t *volatile *)&l.panicSet;
		volatile uint32_t *clear = *(volatile uint32_t *volatile *)&l.panicClear;
		if ((uintptr_t)param2 == kPEPanicBegin && set != NULL && clear != NULL) {
			bool pin = !l.activeLow;
			if (l.panicRMW) {
				*set = pin ? *set | l.panicBit : *set & ~l.panicBit;
			} else {
				*(pin ? set : clear) = l.panicBit;
			}
		}
		return kIOReturnSuccess;
	}
	return super::callPlatformFunction(functionName, waitForFunction, param1, param2, param3, param4);
}

/* A halt or a restart: the triggers stop, ACT off. */
IOReturn
RPiBoard::haltRestart(void *target, void *, UInt32 messageType, IOService *, void *, vm_size_t)
{
	RPiBoard *me = (RPiBoard *)target;
	if (messageType != kIOMessageSystemWillPowerOff && messageType != kIOMessageSystemWillRestart) {
		return kIOReturnUnsupported;
	}
	me->fGate->runActionBlock(^IOReturn {
		me->fHalting = true;
		me->fTimer->cancelTimeout();
		me->drive(me->fLEDs[LED_ACT], false);
		return kIOReturnSuccess;
	});
	return kIOReturnSuccess;
}

#pragma mark the power button

/* On the work loop, from start. */
void
RPiBoard::findButton(void)
{
	OSArray *children = childrenOf("gpio-keys");
	for (unsigned c = 0; children != NULL && c < children->getCount() && !fButton; c++) {
		IORegistryEntry *node = (IORegistryEntry *)children->getObject(c);
		OSData *gpios = dtData(node, "gpios");
		uint32_t code = 0, ms = 0;
		if (!dtU32(node, "linux,code", &code) || code != KEY_POWER || gpios == NULL || gpios->getLength() < 12) {
			continue;
		}
		bcopy(gpios->getBytesNoCopy(), fButtonCells, sizeof(fButtonCells));
		dtU32(node, "debounce-interval", &ms);
		fDebounceMs = ms < BUTTON_MIN_DEBOUNCE_MS ? BUTTON_MIN_DEBOUNCE_MS : ms;
		IOReturn ret = PieGPIORequestLineEdges(fButtonCells, true, true, "pwr_button", buttonEdge, this,
		    kPieGPIOWaitNs);
		if (ret != kIOReturnSuccess) {
			IOLog("RPiBoard: power button (line %u): its edges cannot be had: 0x%x\n",
			    OSReadBigInt32(fButtonCells, 4), ret);
			continue;
		}
		fButton = true;
		bool v = false;
		PieGPIOGetLine(fButtonCells, &v, 0);
		fPressed = v;
		IOLog("RPiBoard: power button: line %u, debounce %u ms, %s\n", OSReadBigInt32(fButtonCells, 4),
		    fDebounceMs, kActionNames[fAction]);
	}
	OSSafeReleaseNULL(children);
}

/*
 * On piegpio's work loop: the line moved; it is read once it has held
 * still. Not through this driver's gate, which may be waiting on
 * piegpio's (an LED's change); stop cancels the timer under the gate
 * after the release that ends these calls.
 */
void
RPiBoard::buttonEdge(void *refcon, uint32_t, bool)
{
	RPiBoard *me = (RPiBoard *)refcon;
	me->fButtonTimer->setTimeoutMS(me->fDebounceMs);
}

void
RPiBoard::buttonTimerFired(OSObject *owner, IOTimerEventSource *)
{
	((RPiBoard *)owner)->buttonSettled();
}

/* On the work loop: the line has held for the debounce interval. */
void
RPiBoard::buttonSettled(void)
{
	if (fHalting || !fButton) { return; }
	bool v = false;
	if (PieGPIOGetLine(fButtonCells, &v, 0) != kIOReturnSuccess) {
		return;
	}
	if (v && !fPressed) {
		press("pressed");
	}
	fPressed = v;
}

/* On the work loop. */
void
RPiBoard::press(const char *why)
{
	fPresses++;
	if (fAction == ACTION_NONE) {
		return;
	}
	if (fAction == ACTION_LOG || fHalting) {
		IOLog("RPiBoard: power button %s\n", why);
		return;
	}
	IOLog("RPiBoard: power button %s: halting\n", why);
	fHalting = true;
	/* init halts at SIGUSR2, as halt(8) asks it to: every process told to quit, sync, then the halt. */
	proc_signal(1, SIGUSR2);
}

int
RPiBoard::setAction(int action)
{
	if (action < 0 || action >= ACTION_COUNT) {
		return EINVAL;
	}
	fGate->runActionBlock(^IOReturn {
		fAction = action;
		return kIOReturnSuccess;
	});
	IOLog("RPiBoard: power button: %s (hw.powerbutton.action)\n", kActionNames[action]);
	return 0;
}

#pragma mark sysctls

enum { LED_BRIGHTNESS, LED_TRIGGER, LED_TOGGLES };

/* arg2: the LED's index * 4 + what. */
static int
sysctl_led(struct sysctl_oid *oidp, void *, int arg2, struct sysctl_req *req)
{
	RPiBoard *b = sBoard;
	unsigned i = (unsigned)arg2 / 4, what = (unsigned)arg2 % 4;
	if (b == NULL || i >= LED_COUNT || !b->fLEDs[i].present) {
		return ENOENT;
	}
	RPiBoard::LED &l = b->fLEDs[i];
	int err;
	if (what == LED_TRIGGER) {
		char name[16];
		strlcpy(name, kTriggerNames[l.trigger], sizeof(name));
		err = sysctl_handle_string(oidp, name, sizeof(name), req);
		if (err != 0 || req->newptr == USER_ADDR_NULL) {
			return err;
		}
		for (int t = 0; t < TRIG_COUNT; t++) {
			if (strcmp(name, kTriggerNames[t]) == 0) {
				return b->setTrigger(i, t);
			}
		}
		return EINVAL;
	}
	if (what == LED_TOGGLES) {
		int64_t v = (int64_t)l.toggles;
		err = sysctl_handle_quad(oidp, &v, 0, req);
		return err != 0 || req->newptr == USER_ADDR_NULL ? err : EPERM;
	}
	bool on = false;
	err = b->brightness(i, &on);
	if (err != 0) {
		return err;
	}
	int v = on;
	err = sysctl_handle_int(oidp, &v, 0, req);
	if (err != 0 || req->newptr == USER_ADDR_NULL) {
		return err;
	}
	if (v != 0 && v != 1) {
		return EINVAL;
	}
	return b->setBrightness(i, v != 0);
}

enum { PB_PRESENT, PB_ACTION, PB_PRESSES, PB_TEST };

static int
sysctl_button(struct sysctl_oid *oidp, void *, int what, struct sysctl_req *req)
{
	RPiBoard *b = sBoard;
	if (b == NULL) {
		return ENOENT;
	}
	int err;
	if (what == PB_ACTION) {
		char name[16];
		strlcpy(name, kActionNames[b->fAction], sizeof(name));
		err = sysctl_handle_string(oidp, name, sizeof(name), req);
		if (err != 0 || req->newptr == USER_ADDR_NULL) {
			return err;
		}
		for (int a = 0; a < ACTION_COUNT; a++) {
			if (strcmp(name, kActionNames[a]) == 0) {
				return b->setAction(a);
			}
		}
		return EINVAL;
	}
	int64_t v = what == PB_PRESENT ? b->fButton : what == PB_PRESSES ? (int64_t)b->fPresses : 0;
	err = sysctl_handle_quad(oidp, &v, 0, req);
	if (err != 0 || req->newptr == USER_ADDR_NULL) {
		return err;
	}
	if (what != PB_TEST) {
		return EPERM;
	}
	if (v != 1) {
		return EINVAL;
	}
	b->fGate->runActionBlock(^IOReturn {
		b->press("pressed (hw.powerbutton.test_press)");
		return kIOReturnSuccess;
	});
	return 0;
}

#define LED_OIDS(led, index) \
	SYSCTL_NODE(_hw_led, OID_AUTO, led, CTLFLAG_RW | CTLFLAG_LOCKED, NULL, "the " #led " LED"); \
	SYSCTL_PROC(_hw_led_##led, OID_AUTO, brightness, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, \
	    (index) * 4 + LED_BRIGHTNESS, sysctl_led, "I", "1: lit (read from its line); setting it sets the trigger to none"); \
	SYSCTL_PROC(_hw_led_##led, OID_AUTO, trigger, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, \
	    (index) * 4 + LED_TRIGGER, sysctl_led, "A", "none, default-on, disk or heartbeat"); \
	SYSCTL_PROC(_hw_led_##led, OID_AUTO, toggles, CTLTYPE_QUAD | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, \
	    (index) * 4 + LED_TOGGLES, sysctl_led, "Q", "times it went on or off since the driver loaded")

SYSCTL_NODE(_hw, OID_AUTO, led, CTLFLAG_RW | CTLFLAG_LOCKED, NULL, "the board's LEDs");
LED_OIDS(act, LED_ACT);
LED_OIDS(pwr, LED_PWR);

SYSCTL_NODE(_hw, OID_AUTO, powerbutton, CTLFLAG_RW | CTLFLAG_LOCKED, NULL, "the power button");
SYSCTL_PROC(_hw_powerbutton, OID_AUTO, present, CTLTYPE_QUAD | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, PB_PRESENT,
    sysctl_button, "Q", "1: the board has one (gpio-keys, KEY_POWER)");
SYSCTL_PROC(_hw_powerbutton, OID_AUTO, action, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, PB_ACTION,
    sysctl_button, "A", "halt, log or none");
SYSCTL_PROC(_hw_powerbutton, OID_AUTO, presses, CTLTYPE_QUAD | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, PB_PRESSES,
    sysctl_button, "Q", "presses since the driver loaded");
SYSCTL_PROC(_hw_powerbutton, OID_AUTO, test_press, CTLTYPE_QUAD | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, PB_TEST,
    sysctl_button, "Q", "1: act as a press does");

static struct sysctl_oid *const sCommonOIDs[] = {
	&sysctl__hw_led, &sysctl__hw_powerbutton, &sysctl__hw_powerbutton_present, &sysctl__hw_powerbutton_action,
	&sysctl__hw_powerbutton_presses, &sysctl__hw_powerbutton_test_press,
};
static struct sysctl_oid *const sLEDOIDs[LED_COUNT][4] = {
	{ &sysctl__hw_led_act, &sysctl__hw_led_act_brightness, &sysctl__hw_led_act_trigger, &sysctl__hw_led_act_toggles },
	{ &sysctl__hw_led_pwr, &sysctl__hw_led_pwr_brightness, &sysctl__hw_led_pwr_trigger, &sysctl__hw_led_pwr_toggles },
};

void
RPiBoard::registerSysctls(void)
{
	fSysctls = true;
	for (unsigned i = 0; i < sizeof(sCommonOIDs) / sizeof(sCommonOIDs[0]); i++) {
		sysctl_register_oid(sCommonOIDs[i]);
	}
	for (unsigned i = 0; i < LED_COUNT; i++) {
		for (unsigned j = 0; fLEDs[i].present && j < 4; j++) {
			sysctl_register_oid(sLEDOIDs[i][j]);
		}
	}
}

/* Leaves first, then the nodes; sysctl_unregister_oid waits for handlers that are running. */
void
RPiBoard::unregisterSysctls(void)
{
	if (!fSysctls) {
		return;
	}
	for (unsigned i = LED_COUNT; i-- > 0;) {
		for (unsigned j = 4; fLEDs[i].present && j-- > 0;) {
			sysctl_unregister_oid(sLEDOIDs[i][j]);
		}
	}
	for (unsigned i = sizeof(sCommonOIDs) / sizeof(sCommonOIDs[0]); i-- > 0;) {
		sysctl_unregister_oid(sCommonOIDs[i]);
	}
	fSysctls = false;
}
