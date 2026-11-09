/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <os/atomic_private.h>

#include "RPiPlatform.h"
#include "../../pie/watchdog.h"

extern "C" {
#include <kern/thread_call.h>
#include <pexpert/pexpert.h>
}

enum {
	PM_PASSWORD = 0x5a000000, PM_PASSWORD_MASK = 0xff000000,
	PM_RSTC = 0x1c, PM_RSTC_WRCFG = 0x30, PM_RSTC_WRCFG_FULL_RESET = 0x20, PM_RSTC_RESET = 0x102,
	PM_WDOG = 0x24, PM_WDOG_TIME = 0xfffff,
	PM_REGS_SIZE = 0x28,
	FW_GET_FIRMWARE_REVISION = 0x00000001,
	FW_VIDEOCORE_SINCE = 1546300800,
};

#define kTimeoutS       15
#define kPetMs          1000
#define kFirmwareTries  30

enum Owner { kOwnerUndecided, kOwnerNone, kOwnerKernel, kOwnerKext };

static IOService *sNub;
static IOLock *sLock;
static thread_call_t sCall;
static IOMemoryMap *sMap;
static volatile uint32_t *sRegs;
static Owner sOwner = kOwnerUndecided;
static bool sMayArm;
static unsigned sTries;
static bool sStopped;

static void
wr(unsigned off, uint32_t v)
{
	sRegs[off / 4] = PM_PASSWORD | v;
}

static bool
hardwareArmed(void)
{
	return (sRegs[PM_RSTC / 4] & PM_RSTC_WRCFG) == PM_RSTC_WRCFG_FULL_RESET;
}

static void
setCount(void)
{
	wr(PM_WDOG, (kTimeoutS << 16) & PM_WDOG_TIME);
	wr(PM_RSTC, (sRegs[PM_RSTC / 4] & ~(PM_PASSWORD_MASK | PM_RSTC_WRCFG)) | PM_RSTC_WRCFG_FULL_RESET);
}

static void
setOwner(Owner owner)
{
	sOwner = owner;
	sNub->setProperty("WatchdogOwner", owner == kOwnerKernel ? "kernel" : owner == kOwnerKext ? "kext" : "none");
}

static bool
decide(void)
{
	uint32_t revision = 0;
	IOReturn ret = RPiFirmwareProperty(FW_GET_FIRMWARE_REVISION, &revision, sizeof(revision));
	if (ret == kIOReturnNotFound && ++sTries < kFirmwareTries) {
		return false;
	}
	if (ret != kIOReturnSuccess || revision < FW_VIDEOCORE_SINCE) {
		IOLog("RPiWatchdog: firmware revision %u is not the VideoCore's (QEMU's: 346337, whose model "
		    "resets at once when armed); the kernel does not arm it\n", revision);
		return true;
	}
	sMap = sNub->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache);
	if (sMap == NULL || sMap->getLength() < PM_REGS_SIZE) {
		IOLog("RPiWatchdog: cannot map the PM block; the kernel does not arm it\n");
		OSSafeReleaseNULL(sMap);
		return true;
	}
	os_atomic_store(&sRegs, (volatile uint32_t *)sMap->getVirtualAddress(), release);
	uint32_t bootArg = 1;
	PE_parse_boot_argn("pie_watchdog", &bootArg, sizeof(bootArg));
	if (bootArg == 0) {
		if (hardwareArmed()) {
			wr(PM_RSTC, PM_RSTC_RESET);
			IOLog("RPiWatchdog: pie_watchdog=0; found armed, disarmed\n");
		} else {
			IOLog("RPiWatchdog: pie_watchdog=0; the kernel does not arm it\n");
		}
		return true;
	}
	sMayArm = true;
	return true;
}

static void
tick(thread_call_param_t, thread_call_param_t)
{
	IOLockLock(sLock);
	if (os_atomic_load(&sStopped, relaxed)) {
		IOLockUnlock(sLock);
		return;
	}
	if (sOwner == kOwnerUndecided) {
		if (!decide()) {
			uint64_t deadline;
			clock_interval_to_deadline(kPetMs, kMillisecondScale, &deadline);
			thread_call_enter_delayed(sCall, deadline);
			IOLockUnlock(sLock);
			return;
		}
		if (sMayArm) {
			setCount();
			IOLog("RPiWatchdog: armed by the kernel, %u s, set again every %u ms until PIEWatchdog "
			    "takes it\n", kTimeoutS, kPetMs);
			setOwner(kOwnerKernel);
		} else {
			setOwner(kOwnerNone);
		}
	} else if (sOwner == kOwnerKernel && !os_atomic_load(&sStopped, relaxed)) {
		setCount();
	}
	if (sOwner == kOwnerKernel && !os_atomic_load(&sStopped, relaxed)) {
		uint64_t deadline;
		clock_interval_to_deadline(kPetMs, kMillisecondScale, &deadline);
		thread_call_enter_delayed(sCall, deadline);
	}
	IOLockUnlock(sLock);
}

void
rpi_watchdog_early_start(IOService *nub)
{
	if (!os_atomic_cmpxchg(&sNub, (IOService *)NULL, nub, acq_rel)) {
		return;
	}
	nub->retain();
	sLock = IOLockAlloc();
	sCall = thread_call_allocate_with_priority(tick, NULL, THREAD_CALL_PRIORITY_KERNEL_HIGH);
	if (sLock == NULL || sCall == NULL) {
		IOLog("RPiWatchdog: no lock or thread call; the kernel does not arm it\n");
		return;
	}
	thread_call_enter(sCall);
}

void
rpi_watchdog_halt(void)
{
	if (sLock != NULL) {
		IOLockLock(sLock);
	}
	os_atomic_store(&sStopped, true, relaxed);
	volatile uint32_t *regs = os_atomic_load(&sRegs, acquire);
	if (regs != NULL) {
		wr(PM_RSTC, PM_RSTC_RESET);
	}
	if (sLock != NULL) {
		IOLockUnlock(sLock);
	}
}

void
rpi_watchdog_panic(void)
{
	os_atomic_store(&sStopped, true, relaxed);
	volatile uint32_t *regs = os_atomic_load(&sRegs, acquire);
	if (regs != NULL && hardwareArmed()) {
		setCount();
	}
}

bool
pie_watchdog_takeover(void)
{
	if (sLock == NULL || sCall == NULL) {
		return false;
	}
	IOLockLock(sLock);
	bool had = sOwner == kOwnerKernel;
	setOwner(kOwnerKext);
	IOLockUnlock(sLock);
	thread_call_cancel_wait(sCall);
	return had;
}

bool
pie_watchdog_handback(bool armed)
{
	if (sLock == NULL || sCall == NULL) {
		return false;
	}
	IOLockLock(sLock);
	bool kernel = armed && sMayArm && !os_atomic_load(&sStopped, relaxed);
	setOwner(kernel ? kOwnerKernel : kOwnerNone);
	if (kernel) {
		setCount();
		uint64_t deadline;
		clock_interval_to_deadline(kPetMs, kMillisecondScale, &deadline);
		thread_call_enter_delayed(sCall, deadline);
		IOLog("RPiWatchdog: back with the kernel, %u s\n", kTimeoutS);
	}
	IOLockUnlock(sLock);
	return kernel;
}
