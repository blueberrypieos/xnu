/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * RPiThermal: the SoC's temperature and the fan that cools it, on the
 * Raspberry Pi 4 (BCM2711) and 5 (BCM2712).
 *
 * The temperature is the AVS monitor's (brcm,bcm2711-avs-monitor, on both
 * SoCs): AVS_RO_TEMP_STATUS at 0x200, a 10-bit reading valid when bit 16
 * or 10 is set, in millidegrees Celsius slope * raw + offset with the
 * thermal zone's "coefficients" (the device trees': BCM2711 -487, 410040;
 * BCM2712 -550, 450000). Where the monitor reads nothing valid the
 * firmware's (mailbox tag GET_TEMPERATURE) stands in, and so where the
 * firmware is not the VideoCore's: QEMU's raspi4b has no AVS monitor, and
 * nothing at its address, where a read aborts. QEMU's mailbox gives its
 * firmware revision as 346337 on every machine; the VideoCore's is its
 * build time, which is 2019 or later on the BCM2711 and BCM2712. The
 * registry property TemperatureSource says which is read: avs-monitor or
 * firmware.
 *
 * The fan follows the device tree's thermal zone as Linux's step governor
 * reads these trees: every polling-delay ms, an active trip goes on at its
 * temperature and off below its temperature less its hysteresis, and the
 * fan's state is the highest that a cooling map gives a trip that is on
 * (the Pi 5's: 50, 60, 67.5 and 75 C for states 1-4; config.txt's
 * fan_temp* dtparams change them in the tree). Critical trips are logged:
 * the firmware throttles the CPUs first. Fans, the first one whose node is
 * okay:
 * - pwm-fan on RP1's PWM (the Pi 5's fan connector, with dtparam
 *   cooling_fan): "pwms" names the PWM block, channel, period and
 *   polarity; "cooling-levels" the duty per state, out of 255. The block's
 *   clock (clk_pwm1, or clk_pwm0, in RP1's clock block) runs from the 50 MHz
 *   crystal undivided; a channel runs trailing-edge mark/space modulation
 *   (CTRL mode 1) over RANGE ticks, high for DUTY, inverted by CTRL bit 3,
 *   enabled by its bit in GLOBAL_CTRL, whose bit 31 makes new values take.
 *   The pin's function (GPIO45 pwm1) comes from the PWM node's pinctrl-0,
 *   set by piegpio; the fan's speed, where the tree names it, is a word of
 *   the PWM block's (rpm-regmap, rpm-offset).
 * - gpio-fan (the Pi 4's gpio-fan overlay): its "gpios", set per state to
 *   the bits of "gpio-fan,speed-map"'s value, through piegpio.
 * Unloading leaves the fan at its top state, so that it never stops on a
 * hot board.
 *
 * What it reads and does is under the hw.thermal sysctls: temperature,
 * temperature_firmware, trips, and with a fan fan_state, fan_max_state,
 * fan_duty (out of 255, as the hardware has it), fan_rpm (-1: unknown)
 * and fan_override (root: -1 automatic, else a state to hold).
 */

#include "RPiThermalPolicy.h"

#include <IOKit/IOCommandGate.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOWorkLoop.h>
#include <libkern/OSByteOrder.h>
#include <sys/errno.h>
#include <sys/sysctl.h>

#include <pie/rpi.h>

#include "PieGPIOKernel.h"

#define kMaxTrips       8
#define kMaxStates      16
#define kMaxFanGPIOs    4

enum {
	AVS_RO_TEMP_STATUS = 0x200, AVS_VALID = (1U << 16) | (1U << 10), AVS_DATA = 0x3ff,
	FW_GET_TEMPERATURE = 0x00030006,
	FW_GET_FIRMWARE_REVISION = 0x00000001,
	FW_VIDEOCORE_SINCE = 1546300800,        /* 2019-01-01 UTC: older than any BCM2711 or BCM2712 firmware */

	PWM_GLOBAL_CTRL = 0x000, PWM_SET_UPDATE = 1U << 31,
	PWM_CTRL_MODE_MS = 1U << 0, PWM_CTRL_INVERT = 1U << 3, PWM_CTRL_FIFO_POP_MASK = 1U << 8,
	CLK_CTRL_ENABLE = 1U << 11, CLK_CTRL_AUXSRC = 0x1fU << 5, CLK_AUXSRC_XOSC = 2,
	CLK_XOSC_NS = 20,       /* 50 MHz */
};

static inline unsigned PWM_CTRL(unsigned ch) { return 0x14 + 16 * ch; }
static inline unsigned PWM_RANGE(unsigned ch) { return 0x18 + 16 * ch; }
static inline unsigned PWM_DUTY(unsigned ch) { return 0x20 + 16 * ch; }

/* RP1's PWM blocks and their clocks' registers in its clock block, by the blocks' distance apart. */
static const struct {
	uint64_t pwmFromClocks;
	unsigned clockRegs;
} sPWMClocks[] = {
	{ 0x9c000 - 0x18000, 0x84 },    /* pwm1, clk_pwm1 */
	{ 0x98000 - 0x18000, 0x74 },    /* pwm0, clk_pwm0 */
};

class RPiThermal : public IOService
{
	OSDeclareDefaultStructors(RPiThermal);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	int temperature(int32_t *mC);
	int firmwareTemperature(int32_t *mC);
	int fanDuty(void);
	int fanRPM(void);
	int setOverride(int state);

	char fTripText[256];
	unsigned fMaxState;
	int fState, fOverride;

private:
	struct Trip {
		int32_t temperature, hysteresis;
		uint32_t phandle;
		int state;              /* the fan's, from the cooling maps; -1: none */
		bool critical, on;
	};
	struct FanGPIO {
		IOService *controller;
		uint32_t line;
		bool activeLow;
	};

	IOMemoryMap *fAVSMap;
	volatile uint32_t *fAVS;
	const char *fSource;            /* TemperatureSource's value */
	int32_t fSlope, fOffset;
	uint32_t fPollMs;
	IOWorkLoop *fWorkLoop;
	IOCommandGate *fGate;
	IOTimerEventSource *fTimer;
	Trip fTrips[kMaxTrips];
	unsigned fTripCount;
	bool fCriticalLogged, fReadFailed;

	char fFanName[32];
	uint32_t fFanPHandle;
	bool fHaveFan;
	uint32_t fLevels[kMaxStates];
	/* pwm-fan */
	IOMemoryMap *fPWMMap;
	volatile uint32_t *fPWM;
	unsigned fChannel;
	uint32_t fRange;
	int fRPMOffset;
	/* gpio-fan */
	FanGPIO fGPIOs[kMaxFanGPIOs];
	unsigned fGPIOCount;

	bool fSysctls;

	void readZone(IORegistryEntry *provider);
	bool setUpPWMFan(IORegistryEntry *fan);
	bool setUpGPIOFan(IORegistryEntry *fan);
	bool setFanState(unsigned state);
	void update(void);
	void registerSysctls(void);
	void unregisterSysctls(void);
	static void timerFired(OSObject *owner, IOTimerEventSource *timer);
};

OSDefineMetaClassAndStructors(RPiThermal, IOService);
#define super IOService

/* The instance the sysctls read; set while they are registered. */
static RPiThermal *sThermal;

#pragma mark device tree

static bool
cellAt(OSData *d, unsigned i, uint32_t *v)
{
	if (d == NULL || d->getLength() < 4 * (i + 1)) {
		return false;
	}
	*v = OSReadBigInt32(d->getBytesNoCopy(), 4 * i);
	return true;
}

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

static uint32_t
dtPHandle(IORegistryEntry *e)
{
	uint32_t v = 0;
	PieGPIOReadPHandle(e, &v);
	return v;
}

/* The first device tree node of that name (retained). */
static IORegistryEntry *
dtFindNamed(const char *name)
{
	IORegistryEntry *root = IORegistryEntry::fromPath("/", gIODTPlane);
	IORegistryIterator *it = root != NULL ?
	    IORegistryIterator::iterateOver(root, gIODTPlane, kIORegistryIterateRecursively) : NULL;
	IORegistryEntry *found = NULL, *e;
	while (it != NULL && found == NULL && (e = it->getNextObject()) != NULL) {
		const char *n = e->getName(gIODTPlane);
		if (n != NULL && strcmp(n, name) == 0) {
			found = e;
			found->retain();
		}
	}
	OSSafeReleaseNULL(it);
	OSSafeReleaseNULL(root);
	return found;
}

/* A child node by name (retained). */
static IORegistryEntry *
dtChild(IORegistryEntry *parent, const char *name)
{
	OSIterator *it = parent->getChildIterator(gIODTPlane);
	IORegistryEntry *found = NULL, *e;
	while (it != NULL && found == NULL && (e = OSDynamicCast(IORegistryEntry, it->getNextObject())) != NULL) {
		const char *n = e->getName(gIODTPlane);
		if (n != NULL && strcmp(n, name) == 0) {
			found = e;
			found->retain();
		}
	}
	OSSafeReleaseNULL(it);
	return found;
}

/* The nub published for the node with that phandle, waited for (retained). */
static IOService *
waitForNub(uint32_t phandle)
{
	uint32_t native = OSSwapHostToLittleInt32(phandle);
	OSData *value = OSData::withBytes(&native, sizeof(native));
	const OSSymbol *key = OSSymbol::withCStringNoCopy("AAPL,phandle");
	OSDictionary *match = value != NULL && key != NULL ? IOService::propertyMatching(key, value) : NULL;
	IOService *nub = match != NULL ? IOService::waitForMatchingService(match, kPieGPIOWaitNs) : NULL;
	OSSafeReleaseNULL(match);
	OSSafeReleaseNULL(key);
	OSSafeReleaseNULL(value);
	return nub;
}

#pragma mark start and stop

bool
RPiThermal::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	fOverride = -1;
	fRPMOffset = -1;
	fAVSMap = provider->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache);
	uint32_t revision = 0;
	if (RPiFirmwareProperty(FW_GET_FIRMWARE_REVISION, &revision, sizeof(revision)) != kIOReturnSuccess ||
	    revision < FW_VIDEOCORE_SINCE) {
		IOLog("RPiThermal: firmware revision %u is not the VideoCore's (QEMU's: 346337); "
		    "the firmware's temperature only\n", revision);
	} else if (fAVSMap == NULL || fAVSMap->getLength() < AVS_RO_TEMP_STATUS + sizeof(uint32_t)) {
		IOLog("RPiThermal: cannot map the AVS monitor; the firmware's temperature only\n");
	} else {
		fAVS = (volatile uint32_t *)fAVSMap->getVirtualAddress();
	}
	IORegistryEntry *root = IORegistryEntry::fromPath("/", gIODTPlane);
	bool bcm2712 = root != NULL && dtString(root, "compatible", "brcm,bcm2712");
	OSSafeReleaseNULL(root);
	fSlope = bcm2712 ? -550 : -487;
	fOffset = bcm2712 ? 450000 : 410040;
	fPollMs = 1000;

	fWorkLoop = IOWorkLoop::workLoop();
	fGate = fWorkLoop != NULL ? IOCommandGate::commandGate(this) : NULL;
	fTimer = fWorkLoop != NULL ? IOTimerEventSource::timerEventSource(this, timerFired) : NULL;
	if (fGate == NULL || fTimer == NULL || fWorkLoop->addEventSource(fGate) != kIOReturnSuccess ||
	    fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess) {
		return false;
	}

	static const char *const kinds[] = { "pwm-fan", "gpio-fan" };
	for (unsigned k = 0; k < 2 && !fHaveFan; k++) {
		OSArray *fans = RPiDTFindCompatible(kinds[k]);
		for (unsigned i = 0; fans != NULL && i < fans->getCount() && !fHaveFan; i++) {
			IORegistryEntry *fan = (IORegistryEntry *)fans->getObject(i);
			if (!dtOkay(fan)) {
				continue;
			}
			strlcpy(fFanName, fan->getName(gIODTPlane), sizeof(fFanName));
			fFanPHandle = dtPHandle(fan);
			fHaveFan = k == 0 ? setUpPWMFan(fan) : setUpGPIOFan(fan);
		}
		OSSafeReleaseNULL(fans);
	}
	readZone(provider);

	int32_t mC = 0;
	int err = temperature(&mC);
	if (err != 0) {
		IOLog("RPiThermal: no temperature (neither the AVS monitor nor the firmware answers)\n");
	}
	registerSysctls();
	registerService();
	if (fHaveFan) {
		IOLog("RPiThermal: %d.%d C (%s); fan %s, states 0-%u; trips %s\n", mC / 1000,
		    (mC < 0 ? -mC : mC) % 1000 / 100, fSource, fFanName, fMaxState, fTripText);
	} else {
		IOLog("RPiThermal: %d.%d C (%s); no fan; trips %s\n", mC / 1000, (mC < 0 ? -mC : mC) % 1000 / 100,
		    fSource, fTripText);
	}
	fGate->runActionBlock(^IOReturn {
		update();
		return kIOReturnSuccess;
	});
	return true;
}

void
RPiThermal::stop(IOService *provider)
{
	unregisterSysctls();
	if (fTimer != NULL) {
		fTimer->cancelTimeout();
	}
	if (fGate != NULL) {
		fGate->runActionBlock(^IOReturn {
			fTimer->cancelTimeout();
			if (fHaveFan) {
				setFanState(fMaxState);
				fState = (int)fMaxState;
			}
			return kIOReturnSuccess;
		});
	}
	if (fHaveFan) {
		PieGPIOReleaseAll(fFanName);
	}
	if (fWorkLoop != NULL) {
		if (fTimer != NULL) {
			fWorkLoop->removeEventSource(fTimer);
		}
		if (fGate != NULL) {
			fWorkLoop->removeEventSource(fGate);
		}
	}
	super::stop(provider);
}

void
RPiThermal::free(void)
{
	for (unsigned i = 0; i < fGPIOCount; i++) {
		OSSafeReleaseNULL(fGPIOs[i].controller);
	}
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fWorkLoop);
	OSSafeReleaseNULL(fPWMMap);
	OSSafeReleaseNULL(fAVSMap);
	super::free();
}

#pragma mark the thermal zone

/*
 * The zone whose sensor is this monitor's (its child node's phandle in
 * thermal-sensors), else cpu-thermal: its coefficients, polling delay,
 * trips and the cooling maps that name the fan.
 */
void
RPiThermal::readZone(IORegistryEntry *provider)
{
	IORegistryEntry *zones = dtFindNamed("thermal-zones");
	IORegistryEntry *zone = NULL;
	OSIterator *it = zones != NULL ? zones->getChildIterator(gIODTPlane) : NULL;
	IORegistryEntry *e;
	while (it != NULL && (e = OSDynamicCast(IORegistryEntry, it->getNextObject())) != NULL) {
		uint32_t sensor;
		bool ours = false;
		if (cellAt(dtData(e, "thermal-sensors"), 0, &sensor)) {
			OSIterator *kids = provider->getChildIterator(gIODTPlane);
			IORegistryEntry *k;
			while (kids != NULL && (k = OSDynamicCast(IORegistryEntry, kids->getNextObject())) != NULL) {
				ours |= dtPHandle(k) == sensor;
			}
			OSSafeReleaseNULL(kids);
		}
		if (ours || (zone == NULL && strcmp(e->getName(gIODTPlane), "cpu-thermal") == 0)) {
			OSSafeReleaseNULL(zone);
			zone = e;
			zone->retain();
			if (ours) {
				break;
			}
		}
	}
	OSSafeReleaseNULL(it);
	OSSafeReleaseNULL(zones);
	snprintf(fTripText, sizeof(fTripText), "none");
	if (zone == NULL) {
		return;
	}

	uint32_t v;
	OSData *coefficients = dtData(zone, "coefficients");
	if (coefficients != NULL && coefficients->getLength() == 8) {
		cellAt(coefficients, 0, &v);
		fSlope = (int32_t)v;
		cellAt(coefficients, 1, &v);
		fOffset = (int32_t)v;
	}
	if (cellAt(dtData(zone, "polling-delay"), 0, &v) && v >= 100) {
		fPollMs = v;
	}

	IORegistryEntry *trips = dtChild(zone, "trips");
	it = trips != NULL ? trips->getChildIterator(gIODTPlane) : NULL;
	while (it != NULL && fTripCount < kMaxTrips && (e = OSDynamicCast(IORegistryEntry, it->getNextObject())) != NULL) {
		Trip &t = fTrips[fTripCount];
		bzero(&t, sizeof(t));
		if (!cellAt(dtData(e, "temperature"), 0, &v)) {
			continue;
		}
		t.temperature = (int32_t)v;
		t.hysteresis = cellAt(dtData(e, "hysteresis"), 0, &v) ? (int32_t)v : 0;
		t.phandle = dtPHandle(e);
		t.state = -1;
		t.critical = dtString(e, "type", "critical");
		fTripCount++;
	}
	OSSafeReleaseNULL(it);
	OSSafeReleaseNULL(trips);

	/* cooling-device = <phandle lowest highest>...; the highest is the state while the trip is on. */
	IORegistryEntry *maps = dtChild(zone, "cooling-maps");
	it = maps != NULL ? maps->getChildIterator(gIODTPlane) : NULL;
	while (fHaveFan && it != NULL && (e = OSDynamicCast(IORegistryEntry, it->getNextObject())) != NULL) {
		uint32_t trip, dev, high;
		OSData *devices = dtData(e, "cooling-device");
		if (!cellAt(dtData(e, "trip"), 0, &trip)) {
			continue;
		}
		for (unsigned i = 0; cellAt(devices, 3 * i, &dev) && cellAt(devices, 3 * i + 2, &high); i++) {
			if (dev != fFanPHandle) {
				continue;
			}
			unsigned state = high > fMaxState ? fMaxState : high;   /* THERMAL_NO_LIMIT: the top */
			for (unsigned t = 0; t < fTripCount; t++) {
				if (fTrips[t].phandle == trip && (int)state > fTrips[t].state) {
					fTrips[t].state = (int)state;
				}
			}
		}
	}
	OSSafeReleaseNULL(it);
	OSSafeReleaseNULL(maps);
	OSSafeReleaseNULL(zone);

	size_t used = 0;
	fTripText[0] = '\0';
	for (unsigned t = 0; t < fTripCount && used < sizeof(fTripText); t++) {
		const Trip &tr = fTrips[t];
		char state[8];
		if (tr.critical) {
			strlcpy(state, "crit", sizeof(state));
		} else if (tr.state >= 0) {
			snprintf(state, sizeof(state), "%d", tr.state);
		} else {
			strlcpy(state, "-", sizeof(state));
		}
		used += (size_t)snprintf(fTripText + used, sizeof(fTripText) - used, "%s%d/%d:%s", t ? " " : "",
		    tr.temperature, tr.hysteresis, state);
	}
	if (fTripCount == 0) {
		strlcpy(fTripText, "none", sizeof(fTripText));
	}
	setProperty("ThermalTrips", fTripText);
	setProperty("ThermalSlope", (unsigned long long)(uint32_t)fSlope, 32);
	setProperty("ThermalOffset", (unsigned long long)(uint32_t)fOffset, 32);
}

#pragma mark fans

bool
RPiThermal::setUpPWMFan(IORegistryEntry *fan)
{
	OSData *pwms = dtData(fan, "pwms");
	OSData *levels = dtData(fan, "cooling-levels");
	uint32_t phandle, channel, period, flags = 0;
	if (!cellAt(pwms, 0, &phandle) || !cellAt(pwms, 1, &channel) || !cellAt(pwms, 2, &period) || channel > 3 ||
	    levels == NULL || levels->getLength() < 8) {
		IOLog("RPiThermal: %s: no PWM or cooling levels it can use\n", fFanName);
		return false;
	}
	cellAt(pwms, 3, &flags);
	IORegistryEntry *node = PieGPIOCopyNodeForPHandle(phandle);
	bool rp1 = node != NULL && dtString(node, "compatible", "raspberrypi,rp1-pwm");
	uint32_t clockPHandle = 0;
	if (node != NULL) {
		cellAt(dtData(node, "clocks"), 0, &clockPHandle);
	}
	OSSafeReleaseNULL(node);
	if (!rp1) {
		IOLog("RPiThermal: %s: its PWM is not RP1's; not driven\n", fFanName);
		return false;
	}
	IOService *pwmNub = waitForNub(phandle);
	IOService *clockNub = clockPHandle != 0 ? waitForNub(clockPHandle) : NULL;
	IOMemoryMap *clockMap = clockNub != NULL ? clockNub->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache) : NULL;
	fPWMMap = pwmNub != NULL ? pwmNub->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache) : NULL;
	bool ok = false;
	do {
		uint64_t range = ((uint64_t)period + CLK_XOSC_NS / 2) / CLK_XOSC_NS;
		if (fPWMMap == NULL || clockMap == NULL || range == 0 || range > UINT32_MAX ||
		    fPWMMap->getLength() < PWM_DUTY(channel) + sizeof(uint32_t)) { break; }
		unsigned states = levels->getLength() / 4;
		if (levels->getLength() % 4 != 0 || states < 2 || states > kMaxStates) { break; }
		bool valid = true;
		for (unsigned i = 0; i < states; i++) {
			cellAt(levels, i, &fLevels[i]);
			valid &= fLevels[i] <= 255 && (i == 0 || fLevels[i] >= fLevels[i - 1]);
		}
		if (!valid || fLevels[states - 1] == 0) { break; }
		unsigned regs = 0;
		uint64_t distance = fPWMMap->getPhysicalAddress() - clockMap->getPhysicalAddress();
		for (unsigned i = 0; i < sizeof(sPWMClocks) / sizeof(sPWMClocks[0]); i++) {
			if (sPWMClocks[i].pwmFromClocks == distance) { regs = sPWMClocks[i].clockRegs; }
		}
		if (regs == 0 || clockMap->getLength() < regs + 3 * sizeof(uint32_t)) { break; }
		OSData *pins = dtData(pwmNub, "pinctrl-0");
		if (pins == NULL || pins->getLength() == 0 || pins->getLength() % 4 != 0) { break; }
		if (PieGPIOApplyPinctrl(pwmNub, fFanName, kPieGPIOWaitNs) != kIOReturnSuccess) { break; }
		volatile uint32_t *clk = (volatile uint32_t *)(clockMap->getVirtualAddress() + regs);
		clk[0] &= ~CLK_CTRL_ENABLE;
		clk[0] = (clk[0] & ~CLK_CTRL_AUXSRC) | CLK_AUXSRC_XOSC << 5;
		clk[1] = 1;
		clk[2] = 0;
		clk[0] |= CLK_CTRL_ENABLE;
		fPWM = (volatile uint32_t *)fPWMMap->getVirtualAddress();
		fChannel = channel;
		fRange = (uint32_t)range;
		fMaxState = states - 1;
		uint32_t regmap, offset;
		if (cellAt(dtData(fan, "rpm-regmap"), 0, &regmap) && regmap == phandle &&
		    cellAt(dtData(fan, "rpm-offset"), 0, &offset) && offset % 4 == 0 &&
		    offset <= fPWMMap->getLength() - sizeof(uint32_t)) { fRPMOffset = (int)offset; }
		fPWM[PWM_CTRL(fChannel) / 4] = PWM_CTRL_FIFO_POP_MASK | PWM_CTRL_MODE_MS | ((flags & 1) ? PWM_CTRL_INVERT : 0);
		fPWM[PWM_RANGE(fChannel) / 4] = fRange;
		fState = setFanState(fMaxState) ? (int)fMaxState : -1;
		fPWM[PWM_GLOBAL_CTRL / 4] |= 1U << fChannel;
		fPWM[PWM_GLOBAL_CTRL / 4] |= PWM_SET_UPDATE;
		ok = true;
	} while (false);
	if (!ok) {
		IOLog("RPiThermal: %s: invalid PWM/clock configuration or pin ownership; fan not driven\n", fFanName);
		PieGPIOReleaseAll(fFanName);
		fPWM = NULL;
		OSSafeReleaseNULL(fPWMMap);
	}

	OSSafeReleaseNULL(clockMap);
	OSSafeReleaseNULL(clockNub);
	OSSafeReleaseNULL(pwmNub);
	return ok;
}

bool
RPiThermal::setUpGPIOFan(IORegistryEntry *fan)
{
	OSData *gpios = dtData(fan, "gpios");
	OSData *map = dtData(fan, "gpio-fan,speed-map");
	unsigned states = map != NULL ? map->getLength() / 8 : 0;
	if (gpios == NULL || gpios->getLength() < 12 || gpios->getLength() % 12 != 0 ||
	    gpios->getLength() / 12 > kMaxFanGPIOs || map == NULL || map->getLength() % 8 != 0 ||
	    states < 2 || states > kMaxStates) {
		IOLog("RPiThermal: %s: no GPIOs or speed map it can use\n", fFanName);
		return false;
	}
	const uint8_t *cells = (const uint8_t *)gpios->getBytesNoCopy();
	for (unsigned i = 0; i < gpios->getLength() / 12 && i < kMaxFanGPIOs; i++) {
		FanGPIO &g = fGPIOs[i];
		g.controller = PieGPIOCopyController(OSReadBigInt32(cells, 12 * i), kPieGPIOWaitNs);
		g.line = OSReadBigInt32(cells, 12 * i + 4);
		g.activeLow = OSReadBigInt32(cells, 12 * i + 8) & 1;
		fGPIOCount = i + 1;
		if (g.controller == NULL) {
			IOLog("RPiThermal: %s: no GPIO controller for its line %u (is piegpio loaded?)\n", fFanName, g.line);
			for (unsigned j = 0; j < fGPIOCount; j++) {
				OSSafeReleaseNULL(fGPIOs[j].controller);
			}
			fGPIOCount = 0;
			return false;
		}
	}
	fMaxState = states - 1 < kMaxStates ? states - 1 : kMaxStates - 1;
	for (unsigned i = 0; i <= fMaxState; i++) {
		cellAt(map, 2 * i + 1, &fLevels[i]);    /* the GPIOs' values */
		if (fLevels[i] >= (1U << fGPIOCount)) {
			for (unsigned j = 0; j < fGPIOCount; j++) { OSSafeReleaseNULL(fGPIOs[j].controller); }
			fGPIOCount = 0;
			return false;
		}
	}
	if (!setFanState(fMaxState)) {
		PieGPIOReleaseAll(fFanName);
		for (unsigned i = 0; i < fGPIOCount; i++) { OSSafeReleaseNULL(fGPIOs[i].controller); }
		fGPIOCount = 0;
		return false;
	}
	fState = (int)fMaxState;
	return true;
}

/* False if a GPIO would not take it (the governor tries again at its next poll). */
bool
RPiThermal::setFanState(unsigned state)
{
	if (fPWM != NULL) {
		fPWM[PWM_DUTY(fChannel) / 4] = (uint32_t)((uint64_t)fLevels[state] * fRange / 255);
		fPWM[PWM_GLOBAL_CTRL / 4] |= PWM_SET_UPDATE;
		return true;
	}
	for (unsigned i = 0; i < fGPIOCount; i++) {
		bool on = (fLevels[state] >> i) & 1;
		if (PieGPIOCall(fGPIOs[i].controller, kPieGPIOSetOutput, (void *)(uintptr_t)fGPIOs[i].line,
		    (void *)(uintptr_t)(on != fGPIOs[i].activeLow), fFanName) != kIOReturnSuccess) {
			return false;
		}
	}
	return true;
}

/* Out of 255: the PWM's duty as the hardware has it, or the GPIO fan's on or off. */
int
RPiThermal::fanDuty(void)
{
	if (fPWM != NULL) {
		uint32_t range = fPWM[PWM_RANGE(fChannel) / 4], duty = fPWM[PWM_DUTY(fChannel) / 4];
		return range != 0 ? (int)((uint64_t)duty * 255 / range) : 0;
	}
	return fState > 0 ? 255 : 0;
}

int
RPiThermal::fanRPM(void)
{
	return fPWM != NULL && fRPMOffset >= 0 ? (int)fPWM[fRPMOffset / 4] : -1;
}

int
RPiThermal::setOverride(int state)
{
	if (state < -1 || state > (int)fMaxState) {
		return EINVAL;
	}
	fGate->runActionBlock(^IOReturn {
		fOverride = state;
		update();
		return kIOReturnSuccess;
	});
	return 0;
}

#pragma mark the governor

int
RPiThermal::firmwareTemperature(int32_t *mC)
{
	uint32_t data[2] = { 0, 0 };    /* sensor id; the reading */
	if (RPiFirmwareProperty(FW_GET_TEMPERATURE, data, sizeof(data)) != kIOReturnSuccess || data[1] == 0) {
		return EIO;
	}
	*mC = (int32_t)data[1];
	return 0;
}

int
RPiThermal::temperature(int32_t *mC)
{
	uint32_t v = fAVS != NULL ? fAVS[AVS_RO_TEMP_STATUS / 4] : 0;
	const char *source = (v & AVS_VALID) ? "avs-monitor" : "firmware";
	if (fSource != source) {
		fSource = source;
		setProperty("TemperatureSource", source);
	}
	if (v & AVS_VALID) {
		*mC = fSlope * (int32_t)(v & AVS_DATA) + fOffset;
		return 0;
	}
	return firmwareTemperature(mC);
}

/* On the work loop: the trips against the temperature, the fan to the state they give. */
void
RPiThermal::update(void)
{
	int32_t mC = 0;
	bool valid = temperature(&mC) == 0;
	if (!valid && !fReadFailed) { IOLog("RPiThermal: the temperature cannot be read; maximum cooling\n"); }
	fReadFailed = !valid;
	unsigned state = thermalNextState(fTrips, fTripCount, valid, mC, fMaxState, fOverride);
	for (unsigned i = 0; valid && i < fTripCount; i++) {
		Trip &trip = fTrips[i];
		if (trip.on && trip.critical && !fCriticalLogged) {
			IOLog("RPiThermal: temperature %d mC above critical trip %d mC\n", mC, trip.temperature);
			fCriticalLogged = true;
		}
	}
	if (fHaveFan && (int)state != fState && setFanState(state)) { fState = (int)state; }

	fTimer->setTimeoutMS(fPollMs);
}

void
RPiThermal::timerFired(OSObject *owner, IOTimerEventSource *)
{
	((RPiThermal *)owner)->update();
}

#pragma mark sysctls

static int
sysctl_temperature(struct sysctl_oid *oidp, void *arg1, int, struct sysctl_req *req)
{
	RPiThermal *t = sThermal;
	int32_t mC = 0;
	int err = t == NULL ? ENOENT : arg1 != NULL ? t->firmwareTemperature(&mC) : t->temperature(&mC);
	if (err != 0) {
		return err;
	}
	int v = mC;
	return sysctl_handle_int(oidp, &v, 0, req);
}

static int
sysctl_trips(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req)
{
	RPiThermal *t = sThermal;
	return t == NULL ? ENOENT : sysctl_handle_string(oidp, t->fTripText, 0, req);
}

enum { FAN_STATE, FAN_MAX_STATE, FAN_DUTY, FAN_RPM, FAN_OVERRIDE };

static int
sysctl_fan(struct sysctl_oid *oidp, void *, int which, struct sysctl_req *req)
{
	RPiThermal *t = sThermal;
	if (t == NULL) {
		return ENOENT;
	}
	int v = which == FAN_STATE ? t->fState : which == FAN_MAX_STATE ? (int)t->fMaxState :
	    which == FAN_DUTY ? t->fanDuty() : which == FAN_RPM ? t->fanRPM() : t->fOverride;
	int err = sysctl_handle_int(oidp, &v, 0, req);
	if (err != 0 || req->newptr == USER_ADDR_NULL || which != FAN_OVERRIDE) {
		return err;
	}
	return t->setOverride(v);
}

SYSCTL_NODE(_hw, OID_AUTO, thermal, CTLFLAG_RW | CTLFLAG_LOCKED, NULL, "SoC temperature and fan");
SYSCTL_PROC(_hw_thermal, OID_AUTO, temperature, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    sysctl_temperature, "I", "SoC temperature, millidegrees Celsius");
SYSCTL_PROC(_hw_thermal, OID_AUTO, temperature_firmware, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, (void *)1, 0,
    sysctl_temperature, "I", "the firmware's reading of it");
SYSCTL_PROC(_hw_thermal, OID_AUTO, trips, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    sysctl_trips, "A", "trips: temperature/hysteresis:fan state");
SYSCTL_PROC(_hw_thermal, OID_AUTO, fan_state, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, FAN_STATE,
    sysctl_fan, "I", "the fan's state");
SYSCTL_PROC(_hw_thermal, OID_AUTO, fan_max_state, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, FAN_MAX_STATE,
    sysctl_fan, "I", "the fan's top state");
SYSCTL_PROC(_hw_thermal, OID_AUTO, fan_duty, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, FAN_DUTY,
    sysctl_fan, "I", "the fan's duty, out of 255");
SYSCTL_PROC(_hw_thermal, OID_AUTO, fan_rpm, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, FAN_RPM,
    sysctl_fan, "I", "the fan's speed, -1 if unknown");
SYSCTL_PROC(_hw_thermal, OID_AUTO, fan_override, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, FAN_OVERRIDE,
    sysctl_fan, "I", "-1: the trips decide; else the state the fan holds");

static struct sysctl_oid *const sCommonOIDs[] = {
	&sysctl__hw_thermal, &sysctl__hw_thermal_temperature, &sysctl__hw_thermal_temperature_firmware,
	&sysctl__hw_thermal_trips,
};
static struct sysctl_oid *const sFanOIDs[] = {
	&sysctl__hw_thermal_fan_state, &sysctl__hw_thermal_fan_max_state, &sysctl__hw_thermal_fan_duty,
	&sysctl__hw_thermal_fan_rpm, &sysctl__hw_thermal_fan_override,
};

void
RPiThermal::registerSysctls(void)
{
	if (!OSCompareAndSwapPtr(NULL, this, &sThermal)) {
		IOLog("RPiThermal: another instance has the sysctls\n");
		return;
	}
	fSysctls = true;
	for (unsigned i = 0; i < sizeof(sCommonOIDs) / sizeof(sCommonOIDs[0]); i++) {
		sysctl_register_oid(sCommonOIDs[i]);
	}
	for (unsigned i = 0; fHaveFan && i < sizeof(sFanOIDs) / sizeof(sFanOIDs[0]); i++) {
		sysctl_register_oid(sFanOIDs[i]);
	}
}

/* Leaves first, then the node; sysctl_unregister_oid waits for handlers that are running. */
void
RPiThermal::unregisterSysctls(void)
{
	if (!fSysctls) {
		return;
	}
	for (unsigned i = sizeof(sFanOIDs) / sizeof(sFanOIDs[0]); fHaveFan && i-- > 0;) {
		sysctl_unregister_oid(sFanOIDs[i]);
	}
	for (unsigned i = sizeof(sCommonOIDs) / sizeof(sCommonOIDs[0]); i-- > 0;) {
		sysctl_unregister_oid(sCommonOIDs[i]);
	}
	sThermal = NULL;
	fSysctls = false;
}
