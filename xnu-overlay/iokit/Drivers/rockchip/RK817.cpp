/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/pwr_mgt/IOPM.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <kern/clock.h>
#include <libkern/OSByteOrder.h>
#include <mach/mach_time.h>
#include <sys/errno.h>
#include <sys/sysctl.h>

#include "RKI2C.h"
#include "RKSoC.h"

/* RK817 registers. */
enum {
	GG_ADC_CONFIG0 = 0x50, GG_Q_PRES = 0x74, GG_CAL_OFFSET = 0x91, GG_VOL_ADC_B = 0xa9,
	PMIC_POWER_EN3 = 0xb4, PMIC_CHRG_OUT = 0xe4, PMIC_CHRG_IN = 0xe5, PMIC_CHRG_STS = 0xeb, PMIC_CHIP_NAME = 0xed,
	PMIC_SYS_STS = 0xf0,
	CHRG_IN_USB_ILIM = 0x0f, CHRG_IN_USB_ILIM_1500MA = 0x0b,
	CHIP_NAME_HIGH = 0x81, CHIP_NAME_LOW = 0x7,
	ADC_CONFIG0_GG_EN = 1U << 7, ADC_CONFIG0_BAT_VOL = 1U << 3, ADC_CONFIG0_BAT_CUR = 1U << 2,
	CHRG_STS_BAT_EXISTS = 1U << 7, CHRG_STS_USB_EXISTS = 1U << 1, SYS_STS_PLUG_IN = 1U << 6,
	POWER_EN_BOOST = 1,
	PMIC_POWER_EN1 = 0xb2, PMIC_LDO1_ON_VSEL = 0xcc, LDOS = 9, LDO_ALL = (1U << LDOS) - 1,
	LDO_VSEL = 0x7f, LDO_VSEL_MAX = 0x70, LDO_MIN_UV = 600000, LDO_STEP_UV = 25000,
};

/* The charger's state, CHRG_STS<6:4>. */
enum {
	CHARGE_OFF, CHARGE_DEAD, CHARGE_TRICKLE, CHARGE_CCCV, CHARGE_DONE, CHARGE_USB_OV, CHARGE_TEMP_ERROR,
	CHARGE_TIME_ERROR,
};

/*
 * The power switches this driver controls when their device tree node says
 * regulator-always-on. An enable register's upper four bits are write
 * enables for the lower four. BOOST feeds the host USB-C socket. The RK817
 * datasheet, sections 1.2 and 4.7, says it cannot be used with the charger,
 * and names no hardware rule that turns it off when USB power comes. So
 * from start to stop the charger's USB power is checked every 50 ms, and
 * BOOST is turned off when it comes, or once the charger's status has not
 * read for 50 ms. It is turned on while the charger has no USB power, as a
 * check reads, from the boot volume on or a minute after start, and never
 * once stop has begun. It is turned off as at stop when the system halts
 * or restarts. hw.battery.boost shows its state as it reads back.
 */
static int sBoost = -1;

static const struct RK817Switch {
	const char *node;
	uint8_t reg, bit;
	int *state;
} rk817Switches[] = {
	{ "BOOST", PMIC_POWER_EN3, POWER_EN_BOOST, &sBoost },
};

#define SWITCHES        (sizeof(rk817Switches) / sizeof(rk817Switches[0]))
enum { FOLLOW_MS = 50, FOLLOW_TRIES = 3, BLIND_MS = 50, UNFOLLOW_MS = 2000, LDO_SLOT_MS = 250 };
enum { REFUSED_BOOST = 1, REFUSED_UNREAD = 2 };
enum { SLOT_NONE, SLOT_OFF, SLOT_BOOST };
#define OCV_POINTS      32

struct RK817Report {
	bool external, installed, charging, full;
	int percent, millivolt, milliamp, remaining;
};

/*
 * The battery as IOPMPowerSource shows it, from what RK817 reports. Its
 * capacities are in mAh, out of the design capacity. The current capacity
 * and the voltage are left out while they are unknown. Once one of its
 * values has changed, updateStatus copies them all to the registry and
 * tells the clients, as IOPMPowerSource's does, but writes no log line for
 * it: on a DEVELOPMENT kernel that line comes with every gauge reading.
 */
class RK817Battery : public IOPMPowerSource
{
	OSDeclareDefaultStructors(RK817Battery);

	unsigned int capacity;
	void forget(const OSSymbol *key);
	static bool publish(void *target, const OSSymbol *key, OSObject *value);

public:
	virtual void updateStatus(void) APPLE_KEXT_OVERRIDE;
	void setDesign(unsigned int milliampHours);
	void report(const struct RK817Report *r);
};

OSDefineMetaClassAndStructors(RK817Battery, IOPMPowerSource);

void
RK817Battery::setDesign(unsigned int milliampHours)
{
	const OSSymbol *key = OSSymbol::withCString(kIOPMPSDesignCapacityKey);
	OSNumber *value = OSNumber::withNumber(milliampHours, 32);
	if (key != NULL && value != NULL) {
		setPSProperty(key, value);
	}
	OSSafeReleaseNULL(key);
	OSSafeReleaseNULL(value);
	capacity = milliampHours;
	setMaxCapacity(milliampHours);
}

void
RK817Battery::forget(const OSSymbol *key)
{
	if (properties->getObject(key) != NULL) {
		properties->removeObject(key);
		removeProperty(key);
		settingsChangedSinceUpdate = true;
	}
}

bool
RK817Battery::publish(void *target, const OSSymbol *key, OSObject *value)
{
	((RK817Battery *)target)->setProperty(key, value);
	return false;
}

void
RK817Battery::updateStatus(void)
{
	if (settingsChangedSinceUpdate) {
		settingsChangedSinceUpdate = false;
		properties->iterateObjects(this, &RK817Battery::publish);
		messageClients(kIOPMMessageBatteryStatusHasChanged);
	}
}

void
RK817Battery::report(const struct RK817Report *r)
{
	setExternalConnected(r->external);
	setExternalChargeCapable(r->external);
	setBatteryInstalled(r->installed);
	setIsCharging(r->charging);
	if (r->percent >= 0) {
		setCurrentCapacity((capacity * (unsigned int)r->percent + 50) / 100);
	} else {
		forget(currentCapacityKey);
	}
	if (r->millivolt > 0) {
		setVoltage((unsigned int)r->millivolt);
	} else {
		forget(voltageKey);
	}
	setAmperage(r->milliamp);
	setTimeRemaining(r->remaining);
	const OSSymbol *key = OSSymbol::withCString(kIOPMFullyChargedKey);
	if (key != NULL) {
		setPSProperty(key, r->full ? kOSBooleanTrue : kOSBooleanFalse);
		key->release();
	}
	updateStatus();
}

class RK817 : public RKRegulators
{
	OSDeclareDefaultStructors(RK817);

	RKI2C *bus;
	uint8_t address;
	IOLock *lock;
	bool stopping;
	const char *ending;
	unsigned threads;
	IONotifier *haltNotifier;
	uint32_t following, failing;
	uint64_t blindUntil;
	int lastUsb;
	uint64_t ldoSlotNext;

	RK817Battery *battery;
	uint32_t senseMicroOhm, resistanceMicroOhm, designMicroAh;
	uint32_t ocvPoints, ocvMicroVolt[OCV_POINTS], ocvPercent[OCV_POINTS];
	int smoothedCharge, smoothedCurrent;
	bool currentSeeded, gaugeLogged;
	uint8_t ldoVsel[LDOS];
	uint16_t ldoOn, ldoKnown;
	uint8_t ldoRefused[LDOS];

	IOReturn read(uint8_t reg, uint8_t *value);
	unsigned ldo(IORegistryEntry *node);
	bool switchesOff(void);
	bool switchesKnown(void);
	int ldoSlot(void);
	bool readLDOs(void);
	bool readLDO(unsigned n);
	bool firstRefusal(unsigned n, uint8_t why);
	void logLDOs(bool tried);
	IOReturn setSwitch(const struct RK817Switch *sw, bool on);
	int switchState(const struct RK817Switch *sw);
	int usbPower(uint64_t until);
	int checkPower(void);
	void settle(const struct RK817Switch *sw, int state, bool on, bool wrote, const char *why);
	int turnOff(const struct RK817Switch *sw, const char *why);
	int turnOn(const struct RK817Switch *sw);
	void unfollow(const char *why);
	bool startThread(void (*fn)(void *, wait_result_t));
	bool pause(uint32_t ms);
	void threadDone(void);
	void endThreads(const char *why);
	static void follow(void *arg, wait_result_t wr);
	static IOReturn haltRestart(void *target, void *refCon, UInt32 messageType, IOService *provider, void *argument,
	    vm_size_t size);

	void setInputLimit(void);
	bool batteryFound(IOService *provider);
	int ocvCharge(int millivolt);
	void batteryStart(void);
	void batteryPoll(bool gauge);
	static void batteryThread(void *arg, wait_result_t wr);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setVoltage(IORegistryEntry *node, uint32_t microvolt) APPLE_KEXT_OVERRIDE;
	virtual IOReturn getVoltage(IORegistryEntry *node, uint32_t *microvolt) APPLE_KEXT_OVERRIDE;
};

#define super RKRegulators
OSDefineMetaClassAndStructors(RK817, RKRegulators);

IOReturn
RK817::read(uint8_t reg, uint8_t *value)
{
	return bus->readRegisters(address, reg, value, 1);
}

IOReturn
RK817::setSwitch(const struct RK817Switch *sw, bool on)
{
	uint8_t v = (uint8_t)((1U << (sw->bit + 4)) | (on ? 1U << sw->bit : 0));
	return bus->writeRegisters(address, sw->reg, &v, 1);
}

/* The switch as its enable register reads: 1 on, 0 off, -1 unknown. */
int
RK817::switchState(const struct RK817Switch *sw)
{
	uint8_t v;
	return read(sw->reg, &v) == kIOReturnSuccess ? (v >> sw->bit) & 1 : -1;
}

/*
 * 1 with USB power at the charger's input, as SYS_STS or CHRG_STS shows
 * it; 0 when both read without it; -1 when neither shows it and one cannot
 * be read. If SYS_STS does not read at or after until, CHRG_STS is not read
 * either: the status has gone unread too long already.
 */
int
RK817::usbPower(uint64_t until)
{
	uint8_t sys, chrg;
	bool haveSys = read(PMIC_SYS_STS, &sys) == kIOReturnSuccess;
	if (haveSys && (sys & SYS_STS_PLUG_IN)) {
		return 1;
	}
	if (!haveSys && mach_absolute_time() >= until) {
		return -1;
	}
	bool haveChrg = read(PMIC_CHRG_STS, &chrg) == kIOReturnSuccess;
	if (haveChrg && (chrg & CHRG_STS_USB_EXISTS)) {
		return 1;
	}
	return haveSys && haveChrg ? 0 : -1;
}

/*
 * The charger's USB power as usbPower reads it, kept in lastUsb. blindUntil
 * is BLIND_MS after the start of the first of a run of checks that cannot
 * read it, and 0 while checks can.
 */
int
RK817::checkPower(void)
{
	uint64_t until = blindUntil;
	if (until == 0) {
		clock_interval_to_deadline(BLIND_MS, kMillisecondScale, &until);
	}
	int usb = usbPower(until);
	blindUntil = usb >= 0 ? 0 : until;
	lastUsb = usb;
	return usb;
}

/*
 * The switch's state, read back after a write or read before one that was
 * not needed, kept in sw->state. A change is logged with why after it, and
 * a write that did not take once, until the switch reads as wanted.
 */
void
RK817::settle(const struct RK817Switch *sw, int state, bool on, bool wrote, const char *why)
{
	uint32_t bit = 1U << (unsigned)(sw - rk817Switches);
	*sw->state = state;
	if (state == (on ? 1 : 0)) {
		if (wrote) {
			IOLog("RK817: %s turned %s%s\n", sw->node, on ? "on" : "off", why);
		}
		failing &= ~bit;
	} else if (wrote && !(failing & bit)) {
		IOLog("RK817: %s did not turn %s\n", sw->node, on ? "on" : "off");
		failing |= bit;
	}
}

/*
 * The switch turned off, unless it reads off already. A write that does
 * not read back off is tried again at once, three tries in all.
 */
int
RK817::turnOff(const struct RK817Switch *sw, const char *why)
{
	int state = switchState(sw);
	bool wrote = false;
	for (int tries = 0; state != 0 && tries < FOLLOW_TRIES; tries++) {
		wrote = true;
		setSwitch(sw, false);
		state = switchState(sw);
	}
	settle(sw, state, false, wrote, why);
	return state;
}

/*
 * The switch turned on, unless it reads on already or cannot be read, and
 * only while the charger's USB power, read again just before the write,
 * is still absent. The write is made under lock, and not once stop has
 * begun, so none follows stop. The switch's state is unknown from the
 * write until it reads back. That power is read once more just after:
 * if it has come in between, the switch is turned off at once.
 */
int
RK817::turnOn(const struct RK817Switch *sw)
{
	int state = switchState(sw);
	if (state != 0 || checkPower() != 0) {
		settle(sw, state, true, false, "");
		return state;
	}
	IOLockLock(lock);
	bool stopped = stopping;
	if (!stopped) {
		setSwitch(sw, true);
		*sw->state = -1;
	}
	IOLockUnlock(lock);
	if (stopped) {
		settle(sw, state, true, false, "");
		return state;
	}
	if (checkPower() == 1) {
		return turnOff(sw, ": the charger has USB power");
	}
	settle(sw, switchState(sw), true, true, "");
	return *sw->state;
}

/*
 * The switches that follow the charger's USB power turned off, as nothing
 * follows it any more. One that does not read back off is tried again
 * every 50 ms for UNFOLLOW_MS, and logged if it still does not.
 */
void
RK817::unfollow(const char *why)
{
	for (unsigned i = 0; i < SWITCHES; i++) {
		const struct RK817Switch *sw = &rk817Switches[i];
		if (!(following & (1U << i))) {
			continue;
		}
		uint64_t until;
		clock_interval_to_deadline(UNFOLLOW_MS, kMillisecondScale, &until);
		int state = turnOff(sw, why);
		while (state != 0 && mach_absolute_time() < until) {
			IOSleep(FOLLOW_MS);
			state = turnOff(sw, why);
		}
		if (state != 0) {
			IOLog("RK817: %s DID NOT TURN OFF in 2 s. Nothing follows the charger's USB power now, and %s may run "
			    "with it.\n", sw->node, sw->node);
		}
	}
	following = 0;
}

/* Runs fn on a new thread, counted until it calls threadDone. */
bool
RK817::startThread(void (*fn)(void *, wait_result_t))
{
	IOLockLock(lock);
	threads++;
	IOLockUnlock(lock);
	if (RKStartThread(this, fn)) {
		return true;
	}
	IOLockLock(lock);
	threads--;
	IOLockUnlock(lock);
	return false;
}

/* Waits ms, or less once stop has begun; false once it has. */
bool
RK817::pause(uint32_t ms)
{
	IOLockLock(lock);
	if (!stopping && ms != 0) {
		uint64_t deadline;
		clock_interval_to_deadline(ms, kMillisecondScale, &deadline);
		IOLockSleepDeadline(lock, &stopping, deadline, THREAD_UNINT);
	}
	bool go = !stopping;
	IOLockUnlock(lock);
	return go;
}

/* A thread's last call: stop may go on, and the thread's reference is released. */
void
RK817::threadDone(void)
{
	IOLockLock(lock);
	threads--;
	IOLockWakeup(lock, &threads, false);
	IOLockUnlock(lock);
	release();
}

/*
 * From start until stop, the charger's USB power is read every 50 ms for
 * the switches that follow it. While the charger has it, they are turned
 * off at once. So they are once its status has gone unread for BLIND_MS,
 * from the start of the first check that could not read it: a check stops
 * reading once a failed read finds that time passed, and they are turned
 * off in the same pass. One check that cannot read it leaves them as they
 * are. While a check reads no USB power, they are turned on, from the boot
 * volume on or a minute after start. Their states are read again every
 * second. They are turned off before this thread ends, so stop does not
 * leave them on unwatched while it waits for the battery's thread.
 */
void
RK817::follow(void *arg, wait_result_t wr)
{
	(void)wr;
	RK817 *me = (RK817 *)arg;
	bool booted = false, first = true;
	uint64_t giveUp, nextSecond = 0;
	clock_interval_to_deadline(60000, kMillisecondScale, &giveUp);
	while (me->pause(first ? 0 : FOLLOW_MS)) {
		first = false;
		bool second = mach_absolute_time() >= nextSecond, now = false;
		if (second) {
			clock_interval_to_deadline(1000, kMillisecondScale, &nextSecond);
		}
		if (second && !booted) {
			booted = now = RKWaitForBootVolume(0);
			if (!booted && mach_absolute_time() >= giveUp) {
				IOLog("RK817: no boot volume within a minute\n");
				booted = now = true;
			}
		}
		int usb = me->checkPower();
		bool blind = usb < 0 && mach_absolute_time() >= me->blindUntil;
		for (unsigned i = 0; i < SWITCHES; i++) {
			const struct RK817Switch *sw = &rk817Switches[i];
			if (!(me->following & (1U << i))) {
				continue;
			}
			if ((usb == 1 || blind) && (*sw->state != 0 || second)) {
				me->turnOff(sw, usb == 1 ? ": the charger has USB power" : ": the charger's status cannot be read");
			} else if (usb == 0 && booted && (*sw->state != 1 || second)) {
				me->turnOn(sw);
			} else if (second) {
				*sw->state = me->switchState(sw);
			}
			if (now && usb == 1 && *sw->state == 0) {
				IOLog("RK817: %s left off: the charger has USB power\n", sw->node);
			}
		}
	}
	me->unfollow(me->ending);
	me->threadDone();
}

/*
 * The RG353V's device tree values, for while pieboot drops property names of
 * 32 characters or more: rockchip,resistor-sense-micro-ohms, and the
 * battery's charge-full-design-microamp-hours and
 * factory-internal-resistance-micro-ohms.
 */
enum {
	RG353V_SENSE_MICRO_OHM = 10000, RG353V_DESIGN_MICRO_AH = 3151000, RG353V_RESISTANCE_MICRO_OHM = 117000,
};

static bool
onRG353V(void)
{
	IORegistryEntry *root = IORegistryEntry::fromPath("/", gIODTPlane);
	bool yes = root != NULL && IODTMatchNubWithKeys(root, "'anbernic,rg353v'");
	OSSafeReleaseNULL(root);
	return yes;
}

/*
 * On the RG353V, set the charger's USB input current limit to 1.5 A with
 * CHRG_IN's USB_ILIM_EN enabled and USB_ILIM_SEL set to 011. Keep the
 * voltage limit in CHRG_IN<7:4> and leave the other charger registers
 * unchanged.
 */
void
RK817::setInputLimit(void)
{
	uint8_t out[2], now;
	if (!onRG353V() || bus->readRegisters(address, PMIC_CHRG_OUT, out, sizeof(out)) != kIOReturnSuccess) {
		return;
	}
	uint8_t want = (uint8_t)((out[1] & ~CHRG_IN_USB_ILIM) | CHRG_IN_USB_ILIM_1500MA);
	if (out[1] == want) {
		IOLog("RK817: charger: CHRG_OUT 0x%02x, CHRG_IN 0x%02x, input limit 1.5 A\n", out[0], out[1]);
		return;
	}
	if (bus->writeRegisters(address, PMIC_CHRG_IN, &want, 1) != kIOReturnSuccess ||
	    read(PMIC_CHRG_IN, &now) != kIOReturnSuccess) {
		IOLog("RK817: charger: CHRG_IN 0x%02x could not be set to 0x%02x\n", out[1], want);
		return;
	}
	IOLog("RK817: charger: CHRG_OUT 0x%02x, CHRG_IN 0x%02x set to 0x%02x, reads 0x%02x: input limit 1.5 A\n", out[0],
	    out[1], want, now);
}

#pragma mark Battery

/*
 * The battery, read from the charger's state and the gas gauge, and never
 * written: no gauge register is set. The datasheet gives the gauge's ADC
 * ranges but not its registers' scales. The scales here were measured on
 * an RG353V, and root can set others: two reference voltages for the
 * VCALIB0 and VCALIB1 codes, the current step in nV across the sense
 * resistor, and the current's sign. The charge is an estimate: the
 * voltage less the current through the battery's internal resistance,
 * looked up in its OCV table.
 */
enum {
	BAT_PERCENT, BAT_CHARGING, BAT_EXTERNAL, BAT_INSTALLED, BAT_FULL, BAT_STATE, BAT_VOLTAGE, BAT_CURRENT,
	BAT_REMAINING, BAT_DESIGN, BAT_GAUGE, BAT_RAW_VOLTAGE, BAT_RAW_CURRENT, BAT_RAW_CHARGE, BAT_VCALIB0,
	BAT_VCALIB1, BAT_IOFFSET, BAT_CAL_OFFSET, BAT_VALUES,
};

static IOLock *sBatteryLock;
static int sBattery[BAT_VALUES];
static int sVref0 = 2250, sVref1 = 4050, sCurrentStep = 1736, sCurrentSign = 1;

static int
sysctl_boost SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int value = sBoost;
	return sysctl_io_number(req, value, sizeof(value), NULL, NULL);
}

static int
sysctl_battery SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1)
	if (sBatteryLock == NULL || arg2 < 0 || arg2 >= BAT_VALUES) {
		return ENOENT;
	}
	IOLockLock(sBatteryLock);
	int value = sBattery[arg2];
	IOLockUnlock(sBatteryLock);
	return sysctl_io_number(req, value, sizeof(value), NULL, NULL);
}

enum { SCALE_MILLIVOLT, SCALE_NANOVOLT, SCALE_SIGN };

static int
sysctl_battery_scale SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp)
	int *p = (int *)arg1;
	if (sBatteryLock == NULL || p == NULL) {
		return ENOENT;
	}
	IOLockLock(sBatteryLock);
	int value = *p;
	IOLockUnlock(sBatteryLock);
	int changed = 0;
	int err = sysctl_io_number(req, value, sizeof(value), &value, &changed);
	if (err != 0 || !changed) {
		return err;
	}
	bool ok = arg2 == SCALE_MILLIVOLT ? value >= 0 && value <= 6000 : arg2 == SCALE_NANOVOLT ?
	    value >= 1 && value <= 100000 : value == 1 || value == -1;
	if (!ok) {
		return EINVAL;
	}
	IOLockLock(sBatteryLock);
	*p = value;
	IOLockUnlock(sBatteryLock);
	return 0;
}

SYSCTL_NODE_MANUAL(_hw, OID_AUTO, battery, CTLFLAG_RW | CTLFLAG_LOCKED, NULL, "the battery, from the PMIC");
#define BAT_OID(name, which, descr) \
	SYSCTL_OID_MANUAL(_hw_battery, OID_AUTO, name, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, which, \
	    sysctl_battery, "I", descr)
#define BAT_SCALE(name, var, kind, descr) \
	SYSCTL_OID_MANUAL(_hw_battery, OID_AUTO, name, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, &var, kind, \
	    sysctl_battery_scale, "I", descr)
BAT_OID(percent, BAT_PERCENT, "charge, an estimate from the voltage and the battery's OCV table, 0-100; -1: unknown");
BAT_OID(charging, BAT_CHARGING, "1: charging");
BAT_OID(external, BAT_EXTERNAL, "1: the charger has USB power");
BAT_OID(installed, BAT_INSTALLED, "1: the charger sees a battery");
BAT_OID(full, BAT_FULL, "1: charging has ended, the battery is full");
BAT_OID(charger_state, BAT_STATE, "CHRG_STS<6:4>: 0 off, 1 dead battery, 2 trickle, 3 CC or CV, 4 done, 5 USB over voltage, 6 temperature error, 7 timer error");
BAT_OID(voltage_mv, BAT_VOLTAGE, "battery voltage, mV, by the scales below; -1: unknown");
BAT_OID(current_ma, BAT_CURRENT, "battery current, mA, positive into the battery, by the scales below");
BAT_OID(time_remaining, BAT_REMAINING, "minutes to empty, or to full while charging, from the charge estimate and the current; -1: unknown");
BAT_OID(design_mah, BAT_DESIGN, "the battery's design capacity, mAh, from the device tree");
BAT_OID(gauge, BAT_GAUGE, "1: the gas gauge measures voltage and current; 0: its ADCs are off, left so");
BAT_OID(raw_voltage, BAT_RAW_VOLTAGE, "BAT_VOL, the gauge's voltage code");
BAT_OID(raw_current, BAT_RAW_CURRENT, "BAT_CUR, the gauge's signed current code");
BAT_OID(raw_charge, BAT_RAW_CHARGE, "Q_PRES, the gauge's coulomb count");
BAT_OID(vcalib0, BAT_VCALIB0, "VCALIB0, the voltage code at the gauge's first reference");
BAT_OID(vcalib1, BAT_VCALIB1, "VCALIB1, the voltage code at the gauge's second reference");
BAT_OID(ioffset, BAT_IOFFSET, "IOFFSET, the gauge's current offset");
BAT_OID(cal_offset, BAT_CAL_OFFSET, "CAL_OFFSET, the board's current offset");
SYSCTL_OID_MANUAL(_hw_battery, OID_AUTO, boost, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, sysctl_boost, "I",
    "1: BOOST, the host USB-C socket's 5 V, is on, as POWER_EN3 reads back; 0: off; -1: unknown");
BAT_SCALE(scale_vref0_mv, sVref0, SCALE_MILLIVOLT, "mV that VCALIB0 stands for, measured; 0 with scale_vref1_mv: 4600 mV per 65536 codes");
BAT_SCALE(scale_vref1_mv, sVref1, SCALE_MILLIVOLT, "mV that VCALIB1 stands for");
BAT_SCALE(scale_current_nv, sCurrentStep, SCALE_NANOVOLT, "nV across the sense resistor per BAT_CUR step");
BAT_SCALE(scale_current_sign, sCurrentSign, SCALE_SIGN, "1: a positive BAT_CUR charges the battery; -1: it discharges it");

static struct sysctl_oid *const sBatteryOIDs[] = {
	&sysctl__hw_battery, &sysctl__hw_battery_percent, &sysctl__hw_battery_charging, &sysctl__hw_battery_external,
	&sysctl__hw_battery_installed, &sysctl__hw_battery_full, &sysctl__hw_battery_charger_state,
	&sysctl__hw_battery_voltage_mv, &sysctl__hw_battery_current_ma, &sysctl__hw_battery_time_remaining,
	&sysctl__hw_battery_design_mah, &sysctl__hw_battery_gauge, &sysctl__hw_battery_raw_voltage,
	&sysctl__hw_battery_raw_current, &sysctl__hw_battery_raw_charge, &sysctl__hw_battery_vcalib0,
	&sysctl__hw_battery_vcalib1, &sysctl__hw_battery_ioffset, &sysctl__hw_battery_cal_offset, &sysctl__hw_battery_boost,
	&sysctl__hw_battery_scale_vref0_mv, &sysctl__hw_battery_scale_vref1_mv,
	&sysctl__hw_battery_scale_current_nv, &sysctl__hw_battery_scale_current_sign,
};

static bool
dtNumber(IORegistryEntry *node, const char *name, uint32_t *value)
{
	OSData *d = node != NULL ? OSDynamicCast(OSData, node->getProperty(name)) : NULL;
	if (d == NULL || d->getLength() != 4) {
		return false;
	}
	*value = OSSwapBigToHostInt32(*(const uint32_t *)d->getBytesNoCopy());
	return true;
}

/* The charger's node and its monitored-battery: the sense resistor, the design capacity, the OCV table. */
bool
RK817::batteryFound(IOService *provider)
{
	IORegistryEntry *charger = RKChildNamed(provider, "charger");
	uint32_t ph = 0;
	IORegistryEntry *cell = charger != NULL && dtNumber(charger, "monitored-battery", &ph) ?
	    RKFindPHandle(IORegistryEntry::getRegistryRoot(), ph) : NULL;
	bool sense = cell != NULL && dtNumber(charger, "rockchip,resistor-sense-micro-ohms", &senseMicroOhm);
	bool design = cell != NULL && dtNumber(cell, "charge-full-design-microamp-hours", &designMicroAh);
	bool resistance = cell != NULL && dtNumber(cell, "factory-internal-resistance-micro-ohms", &resistanceMicroOhm);
	if (cell != NULL && (!sense || !design || !resistance) && onRG353V()) {
		IOLog("RK817: the battery's sense resistor, design capacity or internal resistance is not in the device "
		    "tree, whose loader drops property names of 32 characters or more; the RG353V's are used\n");
		senseMicroOhm = sense ? senseMicroOhm : RG353V_SENSE_MICRO_OHM;
		designMicroAh = design ? designMicroAh : RG353V_DESIGN_MICRO_AH;
		resistanceMicroOhm = resistance ? resistanceMicroOhm : RG353V_RESISTANCE_MICRO_OHM;
		sense = design = true;
	}
	bool found = cell != NULL && sense && senseMicroOhm != 0 && design;
	if (found) {
		OSData *table = OSDynamicCast(OSData, cell->getProperty("ocv-capacity-table-0"));
		const uint32_t *c = table != NULL ? (const uint32_t *)table->getBytesNoCopy() : NULL;
		for (uint32_t i = 0; c != NULL && i + 1 < table->getLength() / 4 && ocvPoints < OCV_POINTS; i += 2) {
			ocvMicroVolt[ocvPoints] = OSSwapBigToHostInt32(c[i]);
			ocvPercent[ocvPoints] = OSSwapBigToHostInt32(c[i + 1]);
			ocvPoints++;
		}
	}
	OSSafeReleaseNULL(cell);
	OSSafeReleaseNULL(charger);
	return found;
}

/* The charge at an open-circuit voltage, in hundredths of a percent, from the OCV table; -1 without one. */
int
RK817::ocvCharge(int millivolt)
{
	if (ocvPoints < 2) {
		return -1;
	}
	int64_t uv = (int64_t)millivolt * 1000;
	uint32_t top = 0, bottom = 0;
	for (uint32_t i = 1; i < ocvPoints; i++) {
		top = ocvMicroVolt[i] > ocvMicroVolt[top] ? i : top;
		bottom = ocvMicroVolt[i] < ocvMicroVolt[bottom] ? i : bottom;
	}
	if (uv >= ocvMicroVolt[top]) {
		return (int)ocvPercent[top] * 100;
	}
	if (uv <= ocvMicroVolt[bottom]) {
		return (int)ocvPercent[bottom] * 100;
	}
	for (uint32_t i = 0; i + 1 < ocvPoints; i++) {
		int64_t v0 = ocvMicroVolt[i], v1 = ocvMicroVolt[i + 1];
		int64_t p0 = (int64_t)ocvPercent[i] * 100, p1 = (int64_t)ocvPercent[i + 1] * 100;
		if (v0 != v1 && uv >= (v0 < v1 ? v0 : v1) && uv <= (v0 < v1 ? v1 : v0)) {
			return (int)(p0 + (uv - v0) * (p1 - p0) / (v1 - v0));
		}
	}
	return -1;
}

void
RK817::batteryPoll(bool gauge)
{
	uint8_t chrg, sys;
	if (read(PMIC_CHRG_STS, &chrg) != kIOReturnSuccess || read(PMIC_SYS_STS, &sys) != kIOReturnSuccess) {
		return;
	}
	int state = (chrg >> 4) & 7;
	bool external = (sys & SYS_STS_PLUG_IN) || (chrg & CHRG_STS_USB_EXISTS);
	bool charging = external && (state == CHARGE_DEAD || state == CHARGE_TRICKLE || state == CHARGE_CCCV);
	bool full = external && state == CHARGE_DONE;

	IOLockLock(sBatteryLock);
	int v[BAT_VALUES];
	bcopy(sBattery, v, sizeof(v));
	int vref0 = sVref0, vref1 = sVref1, step = sCurrentStep, sign = sCurrentSign;
	IOLockUnlock(sBatteryLock);

	v[BAT_CHARGING] = charging;
	v[BAT_EXTERNAL] = external;
	v[BAT_INSTALLED] = (chrg & CHRG_STS_BAT_EXISTS) != 0;
	v[BAT_FULL] = full;
	v[BAT_STATE] = state;

	uint8_t cfg, g[14], cal[8];
	if (gauge && read(GG_ADC_CONFIG0, &cfg) == kIOReturnSuccess &&
	    bus->readRegisters(address, GG_Q_PRES, g, sizeof(g)) == kIOReturnSuccess &&
	    bus->readRegisters(address, GG_CAL_OFFSET, cal, sizeof(cal)) == kIOReturnSuccess) {
		v[BAT_GAUGE] = (cfg & ADC_CONFIG0_GG_EN) || (cfg & (ADC_CONFIG0_BAT_VOL | ADC_CONFIG0_BAT_CUR)) ==
		    (ADC_CONFIG0_BAT_VOL | ADC_CONFIG0_BAT_CUR);
		v[BAT_RAW_CHARGE] = (int)OSReadBigInt32(g, 0);
		v[BAT_RAW_VOLTAGE] = OSReadBigInt16(g, 4);
		v[BAT_RAW_CURRENT] = (int16_t)OSReadBigInt16(g, 6);
		v[BAT_CAL_OFFSET] = OSReadBigInt16(cal, 0);
		v[BAT_VCALIB0] = OSReadBigInt16(cal, 2);
		v[BAT_VCALIB1] = OSReadBigInt16(cal, 4);
		v[BAT_IOFFSET] = (int16_t)OSReadBigInt16(cal, 6);

		int64_t code = v[BAT_RAW_VOLTAGE], c0 = v[BAT_VCALIB0], c1 = v[BAT_VCALIB1];
		int mv = vref0 > 0 && vref1 > vref0 && c1 > c0 ?
		    (int)(vref0 + (code - c0) * (vref1 - vref0) / (c1 - c0)) : (int)(code * 4600 / 65536);
		int ma = (int)((int64_t)v[BAT_RAW_CURRENT] * step * sign / (int64_t)senseMicroOhm);
		bool sane = v[BAT_GAUGE] && mv >= 2500 && mv <= 4800;
		v[BAT_VOLTAGE] = sane ? mv : -1;
		v[BAT_CURRENT] = v[BAT_GAUGE] ? ma : 0;

		int charge = full ? 10000 : sane ? ocvCharge(mv - (int)((int64_t)ma * resistanceMicroOhm / 1000000)) : -1;
		if (charge >= 0) {
			smoothedCharge = smoothedCharge < 0 ? charge : (smoothedCharge * 3 + charge) / 4;
		}
		if (v[BAT_GAUGE]) {
			smoothedCurrent = currentSeeded ? (smoothedCurrent * 3 + ma) / 4 : ma;
			currentSeeded = true;
		} else {
			smoothedCurrent = 0;
			currentSeeded = false;
		}
		v[BAT_PERCENT] = smoothedCharge < 0 ? -1 : (smoothedCharge + 50) / 100;

		int64_t mah = designMicroAh / 1000;
		v[BAT_REMAINING] = -1;
		if (v[BAT_PERCENT] >= 0 && !charging && smoothedCurrent < -20) {
			v[BAT_REMAINING] = (int)(mah * v[BAT_PERCENT] * 60 / 100 / -smoothedCurrent);
		} else if (v[BAT_PERCENT] >= 0 && charging && smoothedCurrent > 20) {
			v[BAT_REMAINING] = (int)(mah * (100 - v[BAT_PERCENT]) * 60 / 100 / smoothedCurrent);
		}
		if (v[BAT_REMAINING] > 5999) {
			v[BAT_REMAINING] = 5999;
		}

		if (!gaugeLogged) {
			uint8_t otp[8];
			bool haveOTP = bus->readRegisters(address, GG_VOL_ADC_B, otp, sizeof(otp)) == kIOReturnSuccess;
			IOLog("RK817: gauge: ADC_CONFIG0 0x%02x, BAT_VOL %d, BAT_CUR %d, Q_PRES %d, VCALIB0 %d, VCALIB1 %d, "
			    "IOFFSET %d, CAL_OFFSET %d, VOL_ADC_B 0x%08x, CUR_ADC_K 0x%08x\n", cfg, v[BAT_RAW_VOLTAGE],
			    v[BAT_RAW_CURRENT], v[BAT_RAW_CHARGE], v[BAT_VCALIB0], v[BAT_VCALIB1], v[BAT_IOFFSET],
			    v[BAT_CAL_OFFSET], haveOTP ? OSReadBigInt32(otp, 0) : 0, haveOTP ? OSReadBigInt32(otp, 4) : 0);
			gaugeLogged = true;
		}
	} else if (full) {
		smoothedCharge = 10000;
		v[BAT_PERCENT] = 100;
	}

	IOLockLock(sBatteryLock);
	bcopy(v, sBattery, sizeof(v));
	IOLockUnlock(sBatteryLock);

	struct RK817Report r = {
		external, v[BAT_INSTALLED] != 0, charging, full,
		v[BAT_PERCENT], v[BAT_VOLTAGE], v[BAT_CURRENT], v[BAT_REMAINING],
	};
	battery->report(&r);
}

void
RK817::batteryStart(void)
{
	sBatteryLock = IOLockAlloc();
	battery = OSTypeAlloc(RK817Battery);
	if (sBatteryLock == NULL || battery == NULL || !battery->init() || !battery->attach(this)) {
		IOLog("RK817: no battery object\n");
		OSSafeReleaseNULL(battery);
		return;
	}
	for (unsigned i = 0; i < BAT_VALUES; i++) {
		sBattery[i] = -1;
	}
	sBattery[BAT_DESIGN] = (int)(designMicroAh / 1000);
	smoothedCharge = -1;
	battery->setDesign(designMicroAh / 1000);
	batteryPoll(true);
	battery->registerService();
	for (unsigned i = 0; i < sizeof(sBatteryOIDs) / sizeof(sBatteryOIDs[0]); i++) {
		sysctl_register_oid(sBatteryOIDs[i]);
	}
	IOLog("RK817: battery %u mAh, %u OCV points, internal resistance %u uOhm, sense resistor %u uOhm; "
	    "%d%%, %d mV, %d mA, charger state %d\n", designMicroAh / 1000,
	    ocvPoints, resistanceMicroOhm, senseMicroOhm, sBattery[BAT_PERCENT], sBattery[BAT_VOLTAGE],
	    sBattery[BAT_CURRENT], sBattery[BAT_STATE]);
}

/*
 * After the boot volume, waited for in steps of a second: the charger's
 * state every 5 s, the gauge every 30 s, until stop.
 */
void
RK817::batteryThread(void *arg, wait_result_t wr)
{
	(void)wr;
	RK817 *me = (RK817 *)arg;
	bool found = false;
	for (int s = 0; s < 60 && !found && me->pause(0); s++) {
		found = RKWaitForBootVolume(1);
	}
	if (me->pause(0)) {
		me->batteryStart();
	}
	for (uint32_t n = 1; me->battery != NULL && me->pause(5000); n++) {
		me->batteryPoll(n % 6 == 0);
	}
	me->threadDone();
}

#pragma mark Regulators

/* The LDO's number, 1 to 9, of a node LDO_REG1 to LDO_REG9 of this PMIC's regulators; 0 for any other node. */
unsigned
RK817::ldo(IORegistryEntry *node)
{
	IORegistryEntry *group = node != NULL ? node->copyParentEntry(gIODTPlane) : NULL;
	IORegistryEntry *pmic = group != NULL ? group->copyParentEntry(gIODTPlane) : NULL;
	const char *name = node != NULL ? node->getName(gIODTPlane) : NULL;
	const char *groupName = group != NULL ? group->getName(gIODTPlane) : NULL;
	unsigned n = 0;
	if (pmic != NULL && pmic == getProvider() && groupName != NULL && strcmp(groupName, "regulators") == 0 &&
	    name != NULL && strncmp(name, "LDO_REG", 7) == 0 && name[7] >= '1' && name[7] <= '9' && name[8] == '\0') {
		n = (unsigned)(name[7] - '0');
	}
	OSSafeReleaseNULL(pmic);
	OSSafeReleaseNULL(group);
	return n;
}

/*
 * True while each switch that follows the charger's USB power reads off.
 * turnOn marks a switch unknown as it writes it, under lock, so the
 * switches stay off while a caller that found them off holds the lock.
 */
bool
RK817::switchesOff(void)
{
	for (unsigned i = 0; i < SWITCHES; i++) {
		if ((following & (1U << i)) && *rk817Switches[i].state != 0) {
			return false;
		}
	}
	return true;
}

/* True while no switch that follows the charger's USB power is unknown, as while turnOn writes it. */
bool
RK817::switchesKnown(void)
{
	for (unsigned i = 0; i < SWITCHES; i++) {
		if ((following & (1U << i)) && *rk817Switches[i].state < 0) {
			return false;
		}
	}
	return true;
}

/*
 * Under lock, when a call may make LDO transfers: SLOT_OFF at once while
 * the switches that follow the charger's USB power read off; SLOT_BOOST,
 * for two one-byte transfers at most, while BOOST reads on, the last check
 * read no USB power and no switch is unknown, once LDO_SLOT_MS have passed
 * since the last such slot began, waited for with the lock let go;
 * SLOT_NONE otherwise, and once stop has begun.
 */
int
RK817::ldoSlot(void)
{
	for (;;) {
		if (stopping) {
			return SLOT_NONE;
		}
		if (switchesOff()) {
			return SLOT_OFF;
		}
		if (lastUsb != 0 || !switchesKnown()) {
			return SLOT_NONE;
		}
		if (mach_absolute_time() >= ldoSlotNext) {
			clock_interval_to_deadline(LDO_SLOT_MS, kMillisecondScale, &ldoSlotNext);
			return SLOT_BOOST;
		}
		IOLockSleepDeadline(lock, &stopping, ldoSlotNext, THREAD_UNINT);
	}
}

/*
 * The LDOs' enables and active voltages read into ldoOn and ldoVsel, under
 * lock: POWER_EN1 to POWER_EN3, then LDO1_ON_VSEL to LDO9_ON_VSEL. Bit n-1
 * of ldoKnown says whether LDO n's hold.
 */
bool
RK817::readLDOs(void)
{
	uint8_t en[3], v[2 * LDOS - 1];
	bool ok = bus->readRegisters(address, PMIC_POWER_EN1, en, sizeof(en)) == kIOReturnSuccess &&
	    bus->readRegisters(address, PMIC_LDO1_ON_VSEL, v, sizeof(v)) == kIOReturnSuccess;
	ldoKnown = ok ? LDO_ALL : 0;
	if (ok) {
		ldoOn = (uint16_t)((en[0] & 0xf) | (en[1] & 0xf) << 4 | (en[2] & 1) << 8);
		for (unsigned i = 0; i < LDOS; i++) {
			ldoVsel[i] = v[2 * i];
		}
	}
	return ok;
}

/* LDO n's enable and active voltage read into the copy, under lock, in two one-byte transfers. */
bool
RK817::readLDO(unsigned n)
{
	uint8_t en, v;
	uint16_t bit = (uint16_t)(1U << (n - 1));
	if (read((uint8_t)(PMIC_POWER_EN1 + (n - 1) / 4), &en) != kIOReturnSuccess ||
	    read((uint8_t)(PMIC_LDO1_ON_VSEL + 2 * (n - 1)), &v) != kIOReturnSuccess) {
		ldoKnown &= (uint16_t)~bit;
		return false;
	}
	ldoOn = (uint16_t)((ldoOn & ~bit) | (((en >> ((n - 1) % 4)) & 1) << (n - 1)));
	ldoVsel[n - 1] = v;
	ldoKnown |= bit;
	return true;
}

/*
 * Whether a call on LDO n refused for why, or 0 if it was not refused,
 * starts a run of refusals for that reason, under lock; only that call
 * logs it.
 */
bool
RK817::firstRefusal(unsigned n, uint8_t why)
{
	bool first = why != 0 && ldoRefused[n - 1] != why;
	ldoRefused[n - 1] = why;
	return first;
}

/*
 * The LDOs' registers are read at start and kept, and getVoltage answers
 * from them. setVoltage writes and reads back. A call's transfers, and
 * reads that refill an LDO's registers once a transfer has failed, are
 * made under lock in a slot that ldoSlot gives: at once while the switches
 * that follow the charger's USB power read off, and while BOOST is on and
 * the charger has no USB power, in slots of two one-byte transfers at
 * least 250 ms apart. So within BOOST's bound they delay the follower by
 * at most one such slot. A call that gets no slot is refused with
 * kIOReturnBusy, and so is one whose refill fails. A write that waited for
 * its slot checks the copy again: one that another call cleared meanwhile
 * is read again in the slot while BOOST is off, and the call is refused
 * while it is on, for its caller to try again. A write whose read-back
 * confirms it marks the copy known. The first call of a run of refusals on
 * an LDO logs why. Every call is refused once stop has begun.
 */
IOReturn
RK817::getVoltage(IORegistryEntry *node, uint32_t *microvolt)
{
	unsigned n = ldo(node);
	if (n == 0) {
		return kIOReturnUnsupported;
	}
	uint16_t bit = (uint16_t)(1U << (n - 1));
	IOLockLock(lock);
	IOReturn ret = kIOReturnSuccess;
	uint8_t why = 0;
	if (!stopping && !(ldoKnown & bit)) {
		int slot = ldoSlot();
		if (slot == SLOT_NONE) {
			ret = kIOReturnBusy;
			why = REFUSED_BOOST;
		} else if (!(slot == SLOT_OFF ? readLDOs() : readLDO(n))) {
			ret = kIOReturnBusy;
			why = REFUSED_UNREAD;
		}
	}
	if (stopping) {
		ret = kIOReturnOffline;
		why = 0;
	}
	uint32_t code = ldoVsel[n - 1] & LDO_VSEL;
	bool on = (ldoOn >> (n - 1)) & 1;
	bool log = firstRefusal(n, why);
	IOLockUnlock(lock);
	if (log) {
		IOLog("RK817: LDO%u not read %s\n", n, why == REFUSED_UNREAD ? "as the LDO registers did not read" :
		    "while BOOST may be on");
	}
	if (ret == kIOReturnSuccess) {
		*microvolt = !on ? 0 : LDO_MIN_UV + LDO_STEP_UV * (code < LDO_VSEL_MAX ? code : LDO_VSEL_MAX);
	}
	return ret;
}

/*
 * An LDO's active voltage, in 25 mV steps from 0.6 V to 3.4 V within the
 * node's limits; its current limit is kept. An LDO that is off is left
 * off, with kIOReturnNotReady.
 */
IOReturn
RK817::setVoltage(IORegistryEntry *node, uint32_t microvolt)
{
	unsigned n = ldo(node);
	uint32_t low, high;
	if (n == 0) {
		return kIOReturnUnsupported;
	}
	if (!dtNumber(node, "regulator-min-microvolt", &low) || !dtNumber(node, "regulator-max-microvolt", &high) ||
	    microvolt < low || microvolt > high || microvolt < LDO_MIN_UV ||
	    microvolt > LDO_MIN_UV + LDO_STEP_UV * LDO_VSEL_MAX || (microvolt - LDO_MIN_UV) % LDO_STEP_UV != 0) {
		return kIOReturnBadArgument;
	}
	uint8_t reg = (uint8_t)(PMIC_LDO1_ON_VSEL + 2 * (n - 1));
	uint8_t code = (uint8_t)((microvolt - LDO_MIN_UV) / LDO_STEP_UV), v;
	uint16_t bit = (uint16_t)(1U << (n - 1));
	IOLockLock(lock);
	IOReturn ret = kIOReturnSuccess;
	uint8_t why = 0;
	if (!stopping && !(ldoKnown & bit)) {
		int slot = ldoSlot();
		if (slot == SLOT_NONE) {
			ret = kIOReturnBusy;
			why = REFUSED_BOOST;
		} else if (!(slot == SLOT_OFF ? readLDOs() : readLDO(n))) {
			ret = kIOReturnBusy;
			why = REFUSED_UNREAD;
		}
	}
	int slot = SLOT_NONE;
	if (ret == kIOReturnSuccess && !stopping && (ldoOn & bit) && (ldoVsel[n - 1] & LDO_VSEL) != code) {
		slot = ldoSlot();
		if (slot == SLOT_NONE) {
			ret = kIOReturnBusy;
			why = REFUSED_BOOST;
		} else if (!(ldoKnown & bit) && !(slot == SLOT_OFF && readLDOs())) {
			ret = kIOReturnBusy;
			why = slot == SLOT_OFF ? REFUSED_UNREAD : REFUSED_BOOST;
		}
	}
	if (ret == kIOReturnSuccess && !stopping && !(ldoOn & bit)) {
		ret = kIOReturnNotReady;
	} else if (ret == kIOReturnSuccess && !stopping && slot != SLOT_NONE && (ldoVsel[n - 1] & LDO_VSEL) != code) {
		v = (uint8_t)((ldoVsel[n - 1] & ~LDO_VSEL) | code);
		ret = bus->writeRegisters(address, reg, &v, 1);
		if (ret == kIOReturnSuccess) {
			ret = read(reg, &v);
		}
		if (ret == kIOReturnSuccess) {
			ldoVsel[n - 1] = v;
			ldoKnown |= bit;
			ret = (v & LDO_VSEL) == code ? kIOReturnSuccess : kIOReturnIOError;
		} else {
			ldoKnown &= (uint16_t)~bit;
		}
	}
	if (stopping) {
		ret = kIOReturnOffline;
		why = 0;
	}
	bool log = firstRefusal(n, why);
	IOLockUnlock(lock);
	if (log) {
		IOLog("RK817: LDO%u left as it is %s\n", n,
		    why == REFUSED_UNREAD ? "as the LDO registers did not read" : "while BOOST may be on");
	} else if (ret == kIOReturnNotReady) {
		IOLog("RK817: LDO%u is off, and is left off\n", n);
	}
	return ret;
}

/* The LDOs' enables and active voltages as start read them, or why it did not. */
void
RK817::logLDOs(bool tried)
{
	if (ldoKnown != LDO_ALL) {
		IOLog("RK817: the LDOs %s\n", tried ? "cannot be read" : "are not read while BOOST may be on");
		return;
	}
	char line[80];
	size_t at = 0;
	for (unsigned i = 0; i < LDOS && at < sizeof(line); i++) {
		uint32_t code = ldoVsel[i] & LDO_VSEL;
		if ((ldoOn >> i) & 1) {
			at += (size_t)snprintf(line + at, sizeof(line) - at, " %u",
			    (LDO_MIN_UV + LDO_STEP_UV * (code < LDO_VSEL_MAX ? code : LDO_VSEL_MAX)) / 1000);
		} else {
			at += (size_t)snprintf(line + at, sizeof(line) - at, " off");
		}
	}
	IOLog("RK817: LDO1 to LDO9 at%s mV\n", line);
}

#pragma mark IOService

bool
RK817::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	lock = IOLockAlloc();
	if (lock == NULL) {
		return false;
	}
	lastUsb = -1;
	bus = OSDynamicCast(RKI2C, provider->getProvider());
	OSData *reg = OSDynamicCast(OSData, provider->getProperty("pieboot,bus-reg"));
	uint64_t a = reg != NULL && reg->getLength() >= 16 ? *(const uint64_t *)reg->getBytesNoCopy() : ~0ULL;
	if (bus == NULL || a > 0x7f) {
		IOLog("RK817: %s: no I2C bus or address\n", provider->getName());
		bus = NULL;
		return false;
	}
	bus->retain();
	address = (uint8_t)a;

	uint8_t id[2], en3, sys, chrg;
	IOReturn ret = kIOReturnError;
	for (int tries = 0; ret != kIOReturnSuccess && tries < 3; tries++) {
		ret = bus->readRegisters(address, PMIC_CHIP_NAME, id, sizeof(id));
	}
	if (ret != kIOReturnSuccess || id[0] != CHIP_NAME_HIGH || (id[1] >> 4) != CHIP_NAME_LOW) {
		IOLog("RK817: %s at 0x%02x: no RK817 (0x%x, chip 0x%02x%02x)\n", provider->getName(), address, ret,
		    ret == kIOReturnSuccess ? id[0] : 0, ret == kIOReturnSuccess ? id[1] : 0);
		return false;
	}
	if (read(PMIC_POWER_EN3, &en3) != kIOReturnSuccess || read(PMIC_SYS_STS, &sys) != kIOReturnSuccess ||
	    read(PMIC_CHRG_STS, &chrg) != kIOReturnSuccess) {
		IOLog("RK817: %s at 0x%02x, version %u; cannot read the power state\n", provider->getName(), address,
		    id[1] & 0xf);
	} else {
		IOLog("RK817: %s at 0x%02x, version %u; POWER_EN3 0x%02x, SYS_STS 0x%02x, CHRG_STS 0x%02x\n",
		    provider->getName(), address, id[1] & 0xf, en3, sys, chrg);
	}
	setInputLimit();

	/*
	 * A switch that must follow the charger's USB power is turned off now if
	 * the charger has it, and followed whatever its state reads.
	 */
	IORegistryEntry *regulators = RKChildNamed(provider, "regulators");
	int usb = checkPower();
	for (unsigned i = 0; i < SWITCHES; i++) {
		const struct RK817Switch *sw = &rk817Switches[i];
		IORegistryEntry *node = RKChildNamed(regulators, sw->node);
		bool alwaysOn = node != NULL && node->getProperty("regulator-always-on") != NULL;
		OSSafeReleaseNULL(node);
		if (!alwaysOn) {
			IOLog("RK817: %s is not always on in the device tree; left as it is\n", sw->node);
			continue;
		}
		following |= 1U << i;
		*sw->state = switchState(sw);
		if (*sw->state < 0) {
			IOLog("RK817: %s: cannot read its state\n", sw->node);
		} else if (*sw->state == 1) {
			IOLog("RK817: %s was on\n", sw->node);
		}
		if (usb == 1) {
			turnOff(sw, ": the charger has USB power");
		}
	}
	OSSafeReleaseNULL(regulators);

	/*
	 * The LDOs' registers are read while the switches read off, or once the
	 * check above has read the charger's status without USB power: USB
	 * power that comes after that check meets fewer of start's transfers,
	 * these two included, than its bound for BOOST counts.
	 */
	IOLockLock(lock);
	bool tried = switchesOff() || usb == 0;
	if (tried) {
		readLDOs();
	}
	IOLockUnlock(lock);
	logLDOs(tried);

	if (following != 0 && !startThread(&RK817::follow)) {
		IOLog("RK817: no thread to follow the charger's USB power\n");
		unfollow(": nothing follows the charger's USB power");
	}
	if (following != 0) {
		haltNotifier = registerPrioritySleepWakeInterest(&RK817::haltRestart, this);
		if (haltNotifier == NULL) {
			IOLog("RK817: no notice of halt or restart; BOOST may stay on through them\n");
		}
	}
	if (!batteryFound(provider)) {
		IOLog("RK817: no charger or battery in the device tree\n");
	} else if (!startThread(&RK817::batteryThread)) {
		IOLog("RK817: no thread for the battery\n");
	}

	registerService();
	return true;
}

/*
 * The threads are ended and waited for. The follower turns its switches off
 * before it ends, with why in its log, as nothing follows the charger's USB
 * power after this.
 */
void
RK817::endThreads(const char *why)
{
	IOLockLock(lock);
	if (!stopping) {
		ending = why;
		stopping = true;
	}
	IOLockWakeup(lock, &stopping, false);
	while (threads != 0) {
		IOLockSleep(lock, &threads, THREAD_UNINT);
	}
	IOLockUnlock(lock);
}

/*
 * At halt and restart the threads end as at stop, so that BOOST is off
 * before the system goes: nothing turns it off in the firmware, or before
 * the next kernel's RK817 starts.
 */
IOReturn
RK817::haltRestart(void *target, void *refCon, UInt32 messageType, IOService *provider, void *argument,
    vm_size_t size)
{
	(void)refCon;
	(void)provider;
	(void)argument;
	(void)size;
	if (messageType == kIOMessageSystemWillPowerOff) {
		((RK817 *)target)->endThreads(": the system is halting");
	} else if (messageType == kIOMessageSystemWillRestart) {
		((RK817 *)target)->endThreads(": the system is restarting");
	}
	return kIOReturnSuccess;
}

void
RK817::stop(IOService *provider)
{
	if (haltNotifier != NULL) {
		haltNotifier->remove();
		haltNotifier = NULL;
	}
	endThreads(": its driver has stopped");
	super::stop(provider);
}

void
RK817::free(void)
{
	OSSafeReleaseNULL(battery);
	OSSafeReleaseNULL(bus);
	if (lock != NULL) {
		IOLockFree(lock);
		lock = NULL;
	}
	super::free();
}
