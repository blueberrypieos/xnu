/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceMemory.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <kern/clock.h>
#include <libkern/OSByteOrder.h>
#include <mach/mach_time.h>
#include <pexpert/pexpert.h>

#include "../rpi/RPiPlatform.h"
#include "../sdmmc/PIESDIO.h"
#include "RKI2C.h"
#include "RKSoC.h"
#include "RKDWMMSDIOCore.h"

enum {
	SDMMC2_CLOCK_SELECT = 0x180, SDMMC2_CLOCK_GATE = 0x344, SDMMC2_RESET = 0x438,
	SDMMC2_DRIVE = 0x590, SDMMC2_SAMPLE = 0x594,
	SDMMC2_CLOCK_MASK = 7U << 8, SDMMC2_CPLL_100M = 3U << 8,
	SDMMC2_GATES = 3, SDMMC2_RESETS = 3U << 10,
	GPIO_DATA_LOW = 0, GPIO_DIRECTION_LOW = 8, GPIO_INPUT = 0x70, GPIO_VERSION = 0x78,
	GPIO_V2 = 0x01000c2b, GPIO_V21 = 0x0101157c, GPIO_V22 = 0x010219c8,
	PMIC_CLOCK_REG = 0xf2, PMIC_CLOCK_ON = 1U << 7,
};

class RKDWMMSDIO : public PIESDIOHost
{
	OSDeclareDefaultStructors(RKDWMMSDIO);

	IOMemoryMap *regsMap;
	volatile uint32_t *regs;
	IOMemoryMap *cruMap;
	IOMemoryMap *powerMap;
	IOMemoryMap *resetMap;
	IORegistryEntry *supplyNode;
	RKRegulators *supply;
	IOWorkLoop *workLoop;
	IOFilterInterruptEventSource *intSource;
	RKDWSdioCore core;
	bool bound;
	bool clocksPrepared;
	bool controllerOn;
	bool busStarted;
	bool stopping;
	uint32_t oldSelect, oldGate, oldReset, oldDrive, oldSample;
	uint32_t busHz;
	uint32_t powerDelay;

	static uint32_t readRegister(void *cookie, uint32_t offset);
	static void writeRegister(void *cookie, uint32_t offset, uint32_t value);
	static uint64_t timeMicroseconds(void *cookie);
	static void pauseMicroseconds(void *cookie, uint32_t usec);
	static bool cell(IORegistryEntry *node, const char *name, unsigned index, uint32_t *value);
	static IORegistryEntry *reference(IORegistryEntry *node, const char *name);
	static bool compatible(IORegistryEntry *node, const char *name);
	static bool gpioSet(IOMemoryMap *map, unsigned pin, bool high);
	bool findPower(IOService *provider);
	bool verifySupply(void);
	bool verifyExternalClock(IORegistryEntry *sequence);
	bool prepareClocks(IOService *provider);
	void releaseClocks(void);
	void removeInterrupt(void);
	bool interruptFilter(IOFilterInterruptEventSource *source);
	void interruptOccurred(IOInterruptEventSource *source, int count);
	static IOReturn result(RKDWSdioCore::Result ret);

protected:
	virtual bool sdioPowerUp(void) APPLE_KEXT_OVERRIDE;
	virtual bool sdioSetClock(uint32_t hz) APPLE_KEXT_OVERRIDE;
	virtual void sdioSetBusWidth4(void) APPLE_KEXT_OVERRIDE;
	virtual bool sdioSetHighSpeed(bool on) APPLE_KEXT_OVERRIDE;
	virtual IOReturn sdioCommand(uint32_t index, uint32_t argument, SDIOResponse type,
	    uint32_t response[4]) APPLE_KEXT_OVERRIDE;
	virtual IOReturn sdioData(uint32_t argument, void *buffer, uint32_t block, uint32_t count,
	    bool write, uint32_t *r5) APPLE_KEXT_OVERRIDE;
	virtual void sdioInterruptEnable(bool on) APPLE_KEXT_OVERRIDE;
	virtual void sdioRecover(void) APPLE_KEXT_OVERRIDE;

public:
	virtual IOService *probe(IOService *provider, SInt32 *score) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
};

#define super PIESDIOHost
OSDefineMetaClassAndStructors(RKDWMMSDIO, PIESDIOHost);

bool
RKDWMMSDIO::cell(IORegistryEntry *node, const char *name, unsigned index, uint32_t *value)
{
	OSData *data = node != NULL ? OSDynamicCast(OSData, node->getProperty(name)) : NULL;
	if (data == NULL || index >= data->getLength() / 4) {
		return false;
	}
	*value = OSReadBigInt32(data->getBytesNoCopy(), index * 4);
	return true;
}

IORegistryEntry *
RKDWMMSDIO::reference(IORegistryEntry *node, const char *name)
{
	uint32_t phandle;
	return cell(node, name, 0, &phandle) ? RKFindPHandle(IORegistryEntry::getRegistryRoot(), phandle) : NULL;
}

bool
RKDWMMSDIO::compatible(IORegistryEntry *node, const char *name)
{
	OSString *key = OSString::withCString(name);
	bool yes = node != NULL && key != NULL && IODTCompareNubName(node, key, NULL);
	OSSafeReleaseNULL(key);
	return yes;
}

uint32_t
RKDWMMSDIO::readRegister(void *cookie, uint32_t offset)
{
	RKDWMMSDIO *host = (RKDWMMSDIO *)cookie;
	return host->regs[offset / 4];
}

void
RKDWMMSDIO::writeRegister(void *cookie, uint32_t offset, uint32_t value)
{
	RKDWMMSDIO *host = (RKDWMMSDIO *)cookie;
	host->regs[offset / 4] = value;
}

uint64_t
RKDWMMSDIO::timeMicroseconds(void *cookie)
{
	(void)cookie;
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000;
}

void
RKDWMMSDIO::pauseMicroseconds(void *cookie, uint32_t usec)
{
	(void)cookie;
	IODelay(usec);
}

bool
RKDWMMSDIO::gpioSet(IOMemoryMap *map, unsigned pin, bool high)
{
	if (map == NULL || pin >= 16) {
		return false;
	}
	uint32_t version = RKRead(map, GPIO_VERSION);
	if (version != GPIO_V2 && version != GPIO_V21 && version != GPIO_V22) {
		return false;
	}
	uint32_t bit = 1U << pin;
	RKWriteMasked(map, GPIO_DATA_LOW, bit, high ? bit : 0);
	RKWriteMasked(map, GPIO_DIRECTION_LOW, bit, bit);
	return (RKRead(map, GPIO_DIRECTION_LOW) & bit) != 0 &&
	       (RKRead(map, GPIO_DATA_LOW) & bit) == (high ? bit : 0);
}

bool
RKDWMMSDIO::verifyExternalClock(IORegistryEntry *sequence)
{
	uint32_t index = 0;
	IORegistryEntry *clock = reference(sequence, "clocks");
	IOService *pmic = supply != NULL ? supply->getProvider() : NULL;
	bool matches = clock != NULL && clock == pmic && compatible(clock, "rockchip,rk817") &&
	    cell(sequence, "clocks", 1, &index) && index == 1;
	RKI2C *bus = matches ? OSDynamicCast(RKI2C, pmic->getProvider()) : NULL;
	OSData *address = pmic != NULL ? OSDynamicCast(OSData, pmic->getProperty("pieboot,bus-reg")) : NULL;
	uint64_t value = address != NULL && address->getLength() >= 16 ?
	    *(const uint64_t *)address->getBytesNoCopy() : ~0ULL;
	uint8_t state = 0;
	bool on = bus != NULL && value <= 0x7f &&
	    bus->readRegisters((uint8_t)value, PMIC_CLOCK_REG, &state, 1) == kIOReturnSuccess &&
	    (state & PMIC_CLOCK_ON) != 0;
	OSSafeReleaseNULL(clock);
	if (!on) {
		IOLog("RKDWMMSDIO: the shared 32 kHz clock is not confirmed on; left unchanged\n");
	}
	return on;
}

bool
RKDWMMSDIO::findPower(IOService *provider)
{
	IORegistryEntry *sequence = reference(provider, "mmc-pwrseq");
	IORegistryEntry *power = reference(provider, "vmmc-supply");
	IORegistryEntry *resetGpio = reference(sequence, "reset-gpios");
	IORegistryEntry *powerGpio = reference(power, "gpio");
	supplyNode = reference(provider, "vqmmc-supply");
	uint32_t resetPin = 0, resetFlags = 0, powerPin = ~0U, powerFlags = ~0U;
	uint32_t minimum = 0, maximum = 0, signalMin = 0, signalMax = 0;
	bool valid = compatible(sequence, "mmc-pwrseq-simple") && compatible(power, "regulator-fixed") &&
	    power->getProperty("regulator-always-on") != NULL &&
	    cell(sequence, "reset-gpios", 1, &resetPin) && resetPin == 2 &&
	    cell(sequence, "reset-gpios", 2, &resetFlags) && resetFlags == 1 &&
	    cell(power, "gpio", 1, &powerPin) && powerPin == 0 &&
	    cell(power, "gpio", 2, &powerFlags) && powerFlags == 0 &&
	    cell(power, "regulator-min-microvolt", 0, &minimum) && minimum == 3300000 &&
	    cell(power, "regulator-max-microvolt", 0, &maximum) && maximum == minimum &&
	    cell(supplyNode, "regulator-min-microvolt", 0, &signalMin) && signalMin == 1800000 &&
	    cell(supplyNode, "regulator-max-microvolt", 0, &signalMax) && signalMax == signalMin;
	if (valid) {
		resetMap = resetGpio != NULL ? RKMapNode(resetGpio) : NULL;
		powerMap = powerGpio != NULL ? RKMapNode(powerGpio) : NULL;
		valid = resetMap != NULL && resetMap->getPhysicalAddress() == 0xfe770000ULL &&
		    powerMap != NULL && powerMap->getPhysicalAddress() == 0xfdd60000ULL;
	}
	if (valid) {
		supply = RKFindRegulators(supplyNode, 5000);
		valid = supply != NULL && verifySupply() && verifyExternalClock(sequence);
	}
	powerDelay = 200;
	(void)cell(sequence, "post-power-on-delay-ms", 0, &powerDelay);
	if (powerDelay < 200) {
		powerDelay = 200;
	}
	if (powerDelay > 1000) {
		valid = false;
	}
	if (valid) {
		RKApplyPinctrlNode(power, "RKDWMMSDIO");
		RKApplyPinctrlNode(sequence, "RKDWMMSDIO");
		RPiReserveGPIOLine(powerGpio, powerPin, "wifi-power");
		RPiReserveGPIOLine(resetGpio, resetPin, "wifi-reset");
	}
	OSSafeReleaseNULL(resetGpio);
	OSSafeReleaseNULL(powerGpio);
	OSSafeReleaseNULL(power);
	OSSafeReleaseNULL(sequence);
	if (!valid) {
		IOLog("RKDWMMSDIO: WiFi power, reset or fixed 1.8 V signaling is not confirmed\n");
	}
	return valid;
}

bool
RKDWMMSDIO::verifySupply(void)
{
	uint64_t started = timeMicroseconds(NULL);
	uint32_t uv = 0;
	IOReturn ret;
	do {
		ret = supply->getVoltage(supplyNode, &uv);
		if (ret != kIOReturnBusy) {
			break;
		}
		IOSleep(50);
	} while (timeMicroseconds(NULL) - started < 3000000);
	return ret == kIOReturnSuccess && uv == 1800000;
}

bool
RKDWMMSDIO::prepareClocks(IOService *provider)
{
	IORegistryEntry *clock = reference(provider, "clocks");
	uint32_t phandle = 0, resetPhandle = 0, resetNumber = 0, clockNumber = 0;
	bool valid = compatible(clock, "rockchip,rk3568-cru") &&
	    cell(provider, "clocks", 0, &phandle) &&
	    cell(provider, "clocks", 1, &clockNumber) && clockNumber == 193 &&
	    cell(provider, "resets", 0, &resetPhandle) && resetPhandle == phandle &&
	    cell(provider, "resets", 1, &resetNumber) && resetNumber == 235;
	for (unsigned i = 1; valid && i < 4; i++) {
		uint32_t p = 0, n = 0;
		valid = cell(provider, "clocks", 2 * i, &p) && p == phandle &&
		    cell(provider, "clocks", 2 * i + 1, &n) && n == (i == 1 ? 194U : 396U + i);
	}
	cruMap = valid ? RKMapNode(clock) : NULL;
	OSSafeReleaseNULL(clock);
	if (cruMap == NULL) {
		return false;
	}
	uint64_t cpll = RKPLLRate(cruMap, RK_CRU_CPLL_CON0, RK_CRU_MODE_CON, RK_CRU_CPLL_MODE_SHIFT);
	uint64_t source = cpll / ((RKRead(cruMap, RK_CRU_CLKSEL(82)) & 0x1f) + 1);
	if (source < 50000000 || source > 300000000) {
		return false;
	}
	oldSelect = RKRead(cruMap, SDMMC2_CLOCK_SELECT) & SDMMC2_CLOCK_MASK;
	oldGate = RKRead(cruMap, SDMMC2_CLOCK_GATE) & SDMMC2_GATES;
	oldReset = RKRead(cruMap, SDMMC2_RESET) & SDMMC2_RESETS;
	oldDrive = RKRead(cruMap, SDMMC2_DRIVE) & 0xffe;
	oldSample = RKRead(cruMap, SDMMC2_SAMPLE) & 0xfff;
	clocksPrepared = true;
	RKWriteMasked(cruMap, SDMMC2_CLOCK_GATE, SDMMC2_GATES, 0);
	RKWriteMasked(cruMap, SDMMC2_RESET, SDMMC2_RESETS, SDMMC2_RESETS);
	IODelay(20);
	RKWriteMasked(cruMap, SDMMC2_CLOCK_SELECT, SDMMC2_CLOCK_MASK, SDMMC2_CPLL_100M);
	RKWriteMasked(cruMap, SDMMC2_DRIVE, 0xffe, 0);
	RKWriteMasked(cruMap, SDMMC2_SAMPLE, 0xfff, 0);
	RKWriteMasked(cruMap, SDMMC2_RESET, SDMMC2_RESETS, 0);
	IODelay(20);
	if ((RKRead(cruMap, SDMMC2_CLOCK_SELECT) & SDMMC2_CLOCK_MASK) != SDMMC2_CPLL_100M ||
	    (RKRead(cruMap, SDMMC2_CLOCK_GATE) & SDMMC2_GATES) != 0 ||
	    (RKRead(cruMap, SDMMC2_RESET) & SDMMC2_RESETS) != 0) {
		return false;
	}
	controllerOn = true;
	busHz = (uint32_t)(source / 2);
	return true;
}

void
RKDWMMSDIO::releaseClocks(void)
{
	if (!clocksPrepared) {
		return;
	}
	if (regs != NULL && controllerOn) {
		regs[RKDWSdioCore::Mask / 4] = 0;
		regs[RKDWSdioCore::ClockEnable / 4] = 0;
		regs[RKDWSdioCore::Command / 4] = RKDWSdioCore::Start | RKDWSdioCore::Hold |
		    RKDWSdioCore::UpdateClock | RKDWSdioCore::WaitData;
		uint64_t started = timeMicroseconds(NULL);
		while ((regs[RKDWSdioCore::Command / 4] & RKDWSdioCore::Start) &&
		    timeMicroseconds(NULL) - started < 100000) {
			IODelay(5);
		}
	}
	RKWriteMasked(cruMap, SDMMC2_CLOCK_SELECT, SDMMC2_CLOCK_MASK, oldSelect);
	RKWriteMasked(cruMap, SDMMC2_DRIVE, 0xffe, oldDrive);
	RKWriteMasked(cruMap, SDMMC2_SAMPLE, 0xfff, oldSample);
	RKWriteMasked(cruMap, SDMMC2_RESET, SDMMC2_RESETS, oldReset);
	IODelay(20);
	RKWriteMasked(cruMap, SDMMC2_CLOCK_GATE, SDMMC2_GATES, oldGate);
	clocksPrepared = false;
	controllerOn = false;
	bound = false;
}

bool
RKDWMMSDIO::sdioPowerUp(void)
{
	if (stopping || !verifySupply()) {
		return false;
	}
	sdioClock = 0;
	core.interruptEnable(false);
	if (!gpioSet(resetMap, 2, false) || !gpioSet(powerMap, 0, true)) {
		return false;
	}
	IOSleep(20);
	if (!core.initialize() || !core.clockOff() || !sdioSetHighSpeed(false) || !gpioSet(resetMap, 2, true)) {
		return false;
	}
	IOSleep(powerDelay);
	return !stopping;
}

bool
RKDWMMSDIO::sdioSetClock(uint32_t hz)
{
	bool ok = !stopping && core.setClock(hz);
	sdioClock = core.clock();
	return ok;
}

void
RKDWMMSDIO::sdioSetBusWidth4(void)
{
	core.width4();
}

bool
RKDWMMSDIO::sdioSetHighSpeed(bool on)
{
	if (on && busHz < kSDIOClockHighSpeedHz) {
		return false;
	}
	RKWriteMasked(cruMap, SDMMC2_DRIVE, 0xffe, on ? 2 : 0);
	RKWriteMasked(cruMap, SDMMC2_SAMPLE, 0xfff, 0);
	return (RKRead(cruMap, SDMMC2_DRIVE) & 0xffe) == (on ? 2U : 0U);
}

IOReturn
RKDWMMSDIO::result(RKDWSdioCore::Result ret)
{
	switch (ret) {
	case RKDWSdioCore::Success: return kIOReturnSuccess;
	case RKDWSdioCore::Timeout: return kIOReturnTimeout;
	case RKDWSdioCore::Busy: return kIOReturnBusy;
	case RKDWSdioCore::Invalid: return kIOReturnBadArgument;
	default: return kIOReturnIOError;
	}
}

IOReturn
RKDWMMSDIO::sdioCommand(uint32_t index, uint32_t argument, SDIOResponse type, uint32_t response[4])
{
	return result(core.command(index, argument, (RKDWSdioCore::Response)type, response));
}

IOReturn
RKDWMMSDIO::sdioData(uint32_t argument, void *buffer, uint32_t block, uint32_t count,
    bool write, uint32_t *r5)
{
	return result(core.transfer(argument, buffer, block, count, write, r5));
}

void
RKDWMMSDIO::sdioInterruptEnable(bool on)
{
	core.interruptEnable(on && !stopping && intSource != NULL);
}

void
RKDWMMSDIO::sdioRecover(void)
{
	if (!core.recover()) {
		IOLog("RKDWMMSDIO: controller recovery did not finish\n");
	}
	sdioClock = core.clock();
}

bool
RKDWMMSDIO::interruptFilter(IOFilterInterruptEventSource *source)
{
	(void)source;
	if (!bound || !core.interruptFilter()) {
		return false;
	}
	return !stopping;
}

void
RKDWMMSDIO::interruptOccurred(IOInterruptEventSource *source, int count)
{
	(void)source;
	(void)count;
	if (!stopping && busStarted) {
		sdioInterrupt();
	}
}

IOService *
RKDWMMSDIO::probe(IOService *provider, SInt32 *score)
{
	uint32_t on = 1;
	if (PE_parse_boot_argn("pie_sdio", &on, sizeof(on)) && on == 0) {
		return NULL;
	}
	IORegistryEntry *root = IORegistryEntry::fromPath("/", gIODTPlane);
	bool board = compatible(root, "anbernic,rg353v");
	OSSafeReleaseNULL(root);
	IODeviceMemory *memory = provider->getDeviceMemoryWithIndex(0);
	uint64_t address = memory != NULL ? memory->getPhysicalAddress() : 0;
	if (!RKDWSdioCore::matches(board, provider->getProperty("non-removable") != NULL,
	    provider->getProperty("no-sd") != NULL, provider->getProperty("no-mmc") != NULL, address)) {
		return NULL;
	}
	return super::probe(provider, score);
}

bool
RKDWMMSDIO::start(IOService *provider)
{
	SInt32 score = 0;
	if (probe(provider, &score) == NULL || !super::start(provider)) {
		return false;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL || regsMap->getPhysicalAddress() != 0xfe000000ULL || regsMap->getLength() < 0x204) {
		return false;
	}
	regs = (volatile uint32_t *)regsMap->getVirtualAddress();
	if (!findPower(provider) || !prepareClocks(provider)) {
		releaseClocks();
		return false;
	}
	RKApplyPinctrl(provider, "RKDWMMSDIO");
	RKDWSdioIO access = { this, readRegister, writeRegister, timeMicroseconds, pauseMicroseconds };
	core.bind(access, 256, busHz);
	bound = true;
	core.interruptEnable(false);
	workLoop = IOWorkLoop::workLoop();
	if (workLoop != NULL) {
		intSource = IOFilterInterruptEventSource::filterInterruptEventSource(this,
		    OSMemberFunctionCast(IOInterruptEventAction, this, &RKDWMMSDIO::interruptOccurred),
		    OSMemberFunctionCast(IOFilterInterruptEventSource::Filter, this, &RKDWMMSDIO::interruptFilter),
		    provider, 0);
	}
	if (intSource == NULL || workLoop->addEventSource(intSource) != kIOReturnSuccess) {
		OSSafeReleaseNULL(intSource);
		uint32_t polling = 0;
		if (!PE_parse_boot_argn("sdio_poll", &polling, sizeof(polling)) || polling == 0) {
			IOLog("RKDWMMSDIO: no interrupt source; the slot stays stopped\n");
			releaseClocks();
			return false;
		}
	} else {
		intSource->enable();
	}
	busStarted = true;
	if (!sdioStartBus()) {
		busStarted = false;
		removeInterrupt();
		releaseClocks();
		return false;
	}
	IOLog("RKDWMMSDIO: RG353V internal WiFi, %u kHz source, PIO, fixed 1.8 V signaling\n", busHz / 1000);
	return true;
}

void
RKDWMMSDIO::removeInterrupt(void)
{
	if (bound) {
		core.interruptEnable(false);
	}
	if (intSource != NULL) {
		intSource->disable();
		if (workLoop != NULL) {
			workLoop->removeEventSource(intSource);
		}
		OSSafeReleaseNULL(intSource);
	}
}

void
RKDWMMSDIO::stop(IOService *provider)
{
	stopping = true;
	sdioStopBus();
	busStarted = false;
	removeInterrupt();
	releaseClocks();
	super::stop(provider);
}

void
RKDWMMSDIO::free(void)
{
	stopping = true;
	sdioStopBus();
	busStarted = false;
	removeInterrupt();
	releaseClocks();
	OSSafeReleaseNULL(workLoop);
	OSSafeReleaseNULL(supply);
	OSSafeReleaseNULL(supplyNode);
	OSSafeReleaseNULL(powerMap);
	OSSafeReleaseNULL(resetMap);
	OSSafeReleaseNULL(cruMap);
	OSSafeReleaseNULL(regsMap);
	super::free();
}
