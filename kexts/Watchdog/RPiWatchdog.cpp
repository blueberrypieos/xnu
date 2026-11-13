/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * RPiWatchdog: the hardware watchdog of the Raspberry Pi 4 (BCM2711) and 5
 * (BCM2712), the power-management block's (brcm,bcm2711-pm with
 * brcm,bcm2835-pm-wdt; brcm,bcm2712-pm). Armed, it resets the board when
 * its count runs out; the kernel sets the count again every second, so a
 * kernel whose timers stop (a hang, or a panic that never gets to its
 * reboot) resets within WatchdogTimeout seconds (the personality's, 15,
 * the most the count holds) of the last time. The reset is the one the
 * kernel's reboot makes (TF-A's PSCI SYSTEM_RESET arms this watchdog with a
 * few ticks): the panic region survives it.
 *
 * The kernel itself arms it from the moment IOKit publishes the PM block
 * (<pie/watchdog.h>, xnu's RPiWatchdogEarly.cpp), so that a hang before
 * rc loads this kext resets the board too: start takes it over
 * (pie_watchdog_takeover), and stop gives it back, armed if it was, for
 * the kernel to set again every second (pie_watchdog_handback). The
 * kernel also disarms it at a halt and sets a full count as a panic
 * begins, whoever has it.
 *
 * Its registers, written with the password 0x5a000000 in the top byte:
 * RSTC 0x1c, whose WRCFG (bits 5:4) 2 makes the count's end a full reset;
 * WDOG 0x24, the count (bits 19:0) in 1/65536 s. Armed: WDOG set, then
 * WRCFG 2. Disarmed: RSTC 0x102. RSTS (0x20), whose partition bits tell
 * the firmware to halt, is not touched.
 *
 * QEMU's raspi4b models the block without a count: RSTC's WRCFG 2 resets
 * the machine at once. QEMU is told apart as PIEThermal does it, by the
 * firmware's revision (QEMU's mailbox gives 346337 on every machine; the
 * VideoCore's is its build time, 2019 or later on these SoCs), and the
 * watchdog is not armed there unless hw.watchdog.enabled is set. The
 * boot-arg pie_watchdog=0 leaves it unarmed on a Pi, and disarms one the
 * bootloader armed (the EEPROM's BOOT_WATCHDOG_TIMEOUT).
 *
 * User space checks in through IOWatchDogTimer: an administrator
 * sets the service's properties to a number of seconds N
 * (IORegistryEntrySetCFProperties with a CFNumber), and must set it again
 * within N seconds, or the kernel panics, naming the process that set it
 * last; 0 ends the check-ins. That is the kernel's timer too, with the
 * hardware armed or not. A halt (kIOMessageSystemWillPowerOff) disarms the
 * hardware, so that it stays halted; a halt or a restart ends the
 * check-ins as the kernel's reboot begins (kIOMessageSystemPagingOff),
 * before it kills what is left. init has sent every process SIGTERM before
 * that (SIGKILL 2 s later, then a sync): a daemon must set 0 on SIGTERM.
 * With the boot-arg pie_watchdog_boot=N, the kernel's deadline of N s
 * from boot becomes a check-in due when this starts: user space must set
 * a number, or 0, by then.
 *
 * Registry property WatchdogState: armed, disarmed, qemu (not armed: QEMU),
 * expiring (hw.watchdog.test_expire). The hw.watchdog sysctls: enabled
 * (root: 1 arms, 0 disarms), timeout (root: 2-15 s), remaining (ms, -1 when
 * not armed), pets, user_timeout (s, 0 when off), user_remaining (ms, -1
 * when off), user_pid, and test_expire (root: 1 stops setting the count,
 * as a hung kernel would, with the hardware armed: the board resets
 * timeout seconds later; 0 goes back to setting it).
 */

#include <IOKit/IOCommandGate.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMessage.h>
#include <IOKit/IOService.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <IOKit/system_management/IOWatchDogTimer.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <string.h>
#include <sys/sysctl.h>

#include <pie/rpi.h>
#include <pie/watchdog.h>

enum {
	PM_PASSWORD = 0x5a000000, PM_PASSWORD_MASK = 0xff000000,
	PM_RSTC = 0x1c, PM_RSTC_WRCFG = 0x30, PM_RSTC_WRCFG_FULL_RESET = 0x20, PM_RSTC_RESET = 0x102,
	PM_WDOG = 0x24, PM_WDOG_TIME = 0xfffff,
	PM_REGS_SIZE = 0x28,
	FW_GET_FIRMWARE_REVISION = 0x00000001,
	FW_VIDEOCORE_SINCE = 1546300800,        /* 2019-01-01 UTC: older than any BCM2711 or BCM2712 firmware */
};

#define kMinTimeoutS    2               /* a pet a second: 1 s would run out between them */
#define kMaxTimeoutS    15              /* PM_WDOG_TIME >> 16 */
#define kPetMs          1000
#define kMaxUserS       86400

class RPiWatchdog : public IOWatchDogTimer
{
	OSDeclareDefaultStructors(RPiWatchdog);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual void setWatchDogTimer(UInt32 timeOut) APPLE_KEXT_OVERRIDE;

	int setEnabled(bool on);
	int setTimeout(int s);
	int setExpiring(bool on);
	int remainingMs(void);
	int userRemainingMs(void);

	uint32_t fTimeoutS, fUserS;
	uint64_t fPets;
	int fUserPid;
	bool fArmed;

private:
	IOMemoryMap *fMap;
	volatile uint32_t *fRegs;
	IOWorkLoop *fWorkLoop;
	IOCommandGate *fGate;
	IOTimerEventSource *fTimer;
	IONotifier *fHaltNotifier;
	uint64_t fUserDeadline;
	char fUserName[32];
	bool fQEMU, fExpiring, fSysctls;
	bool fOwnInstance, fGateAdded, fTimerAdded, fUserShutdown, fUserSet, fBootDue;
	volatile bool fStopping;
	thread_t volatile fInternal;

	uint32_t rd(unsigned off) { return fRegs[off / 4]; }
	void wr(unsigned off, uint32_t v) { fRegs[off / 4] = PM_PASSWORD | v; }
	bool hardwareArmed(void) { return (rd(PM_RSTC) & PM_RSTC_WRCFG) == PM_RSTC_WRCFG_FULL_RESET; }
	void pet(void);
	void arm(void);
	void disarm(void);
	void updateState(void);
	void tick(void);
	void registerSysctls(void);
	void unregisterSysctls(void);
	static void timerFired(OSObject *owner, IOTimerEventSource *sender);
	static IOReturn haltRestart(void *target, void *refCon, UInt32 messageType, IOService *provider,
	    void *messageArgument, vm_size_t argSize);
};

OSDefineMetaClassAndStructors(RPiWatchdog, IOWatchDogTimer);
#define super IOWatchDogTimer

/* The instance the sysctls reach; set while they are registered. */
static RPiWatchdog *sWatchdog;

bool
RPiWatchdog::start(IOService *provider)
{
	fMap = provider->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache);
	if (fMap == NULL || fMap->getLength() < PM_REGS_SIZE) {
		IOLog("RPiWatchdog: cannot map its registers\n");
		return false;
	}
	fRegs = (volatile uint32_t *)fMap->getVirtualAddress();
	/* Reject duplicates before either instance can alter the PM block. */
	if (!OSCompareAndSwapPtr(NULL, this, &sWatchdog)) {
		IOLog("RPiWatchdog: another instance owns the hardware\n");
		return false;
	}
	fOwnInstance = true;

	OSNumber *timeout = OSDynamicCast(OSNumber, getProperty("WatchdogTimeout"));
	fTimeoutS = timeout != NULL ? timeout->unsigned32BitValue() : kMaxTimeoutS;
	fTimeoutS = fTimeoutS < kMinTimeoutS ? kMinTimeoutS : fTimeoutS > kMaxTimeoutS ? kMaxTimeoutS : fTimeoutS;
	uint32_t bootArg = 1;
	PE_parse_boot_argn("pie_watchdog", &bootArg, sizeof(bootArg));
	uint32_t revision = 0;
	fQEMU = RPiFirmwareProperty(FW_GET_FIRMWARE_REVISION, &revision, sizeof(revision)) != kIOReturnSuccess ||
	    revision < FW_VIDEOCORE_SINCE;

	fWorkLoop = IOWorkLoop::workLoop();
	fGate = fWorkLoop != NULL ? IOCommandGate::commandGate(this) : NULL;
	/* Kernel-high: busy threads must not hold off a pet. */
	fTimer = fWorkLoop != NULL ?
	    IOTimerEventSource::timerEventSource(kIOTimerEventSourceOptionsPriorityKernelHigh, this, timerFired) : NULL;
	if (fGate == NULL || fTimer == NULL || fWorkLoop->addEventSource(fGate) != kIOReturnSuccess) {
		return false;
	}
	fGateAdded = true;
	if (fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess) {
		return false;
	}
	fTimerAdded = true;
	/* A halt must be able to disarm us before the superclass publishes. */
	fHaltNotifier = registerPrioritySleepWakeInterest(haltRestart, this, NULL);
	if (fHaltNotifier == NULL) {
		return false;
	}
	fInternal = current_thread();
	bool started = super::start(provider);
	fInternal = NULL;
	if (!started) {
		return false;
	}

	/* Stop kernel pets only after our timer and required notifier are ready. */
	bool kernelHad = pie_watchdog_takeover();
	uint32_t bootS = 0;
	uint64_t bootDeadline = pie_watchdog_boot_deadline(&bootS);
	if (kernelHad) {
		/* A later mailbox failure must not abandon the kernel's armed count. */
		fQEMU = false;
	}

	fGate->runActionBlock(^IOReturn {
		uint32_t rstc = rd(PM_RSTC), wdog = rd(PM_WDOG);
		if (fQEMU) {
			IOLog("RPiWatchdog: firmware revision %u is not the VideoCore's (QEMU's: 346337, whose model "
			    "resets at once when armed); not armed\n", revision);
		} else if (bootArg == 0) {
			IOLog("RPiWatchdog: pie_watchdog=0; not armed\n");
			/* One the bootloader armed (BOOT_WATCHDOG_TIMEOUT) would reset the board unpetted. */
			if ((rstc & PM_RSTC_WRCFG) == PM_RSTC_WRCFG_FULL_RESET) {
				disarm();
				IOLog("RPiWatchdog: found armed (RSTC 0x%08x, WDOG 0x%08x); disarmed\n", rstc, wdog);
			}
		} else {
			if (kernelHad) {
				IOLog("RPiWatchdog: taken over from the kernel\n");
			} else if ((rstc & PM_RSTC_WRCFG) == PM_RSTC_WRCFG_FULL_RESET) {
				IOLog("RPiWatchdog: found armed (RSTC 0x%08x, WDOG 0x%08x)\n", rstc, wdog);
			}
			arm();
			IOLog("RPiWatchdog: armed, %u s, set again every %u ms\n", fTimeoutS, kPetMs);
		}
		if (bootDeadline != 0 && !fUserSet && !fUserShutdown && !fStopping) {
			fUserS = bootS;
			fUserDeadline = bootDeadline;
			fUserPid = 0;
			fBootDue = true;
			strlcpy(fUserName, "pie_watchdog_boot", sizeof(fUserName));
			IOLog("RPiWatchdog: the boot deadline of %u s is now a check-in due\n", bootS);
		}
		updateState();
		fTimer->setTimeoutMS(kPetMs);
		return kIOReturnSuccess;
	});
	registerSysctls();
	return true;
}

void
RPiWatchdog::stop(IOService *provider)
{
	unregisterSysctls();
	if (fHaltNotifier != NULL) {
		fHaltNotifier->remove();
		fHaltNotifier = NULL;
	}
	fStopping = true;
	fTimer->cancelTimeout();
	fGate->runActionBlock(^IOReturn {
		fTimer->cancelTimeout();
		if (fUserS != 0 && fBootDue && !fUserShutdown) {
			/* No one checked in yet: the kernel keeps the boot deadline. */
			pie_watchdog_boot_restore(fUserDeadline, fUserS);
			IOLog("RPiWatchdog: unloaded; the boot deadline goes back to the kernel\n");
			fUserS = 0;
			fBootDue = false;
		} else if (fUserS != 0) {
			IOLog("RPiWatchdog: unloaded with check-ins due from pid %d (%s); they end\n", fUserPid, fUserName);
			fUserS = 0;
		}
		/* Armed, it goes back to the kernel, which sets it from then on; else it stays disarmed. */
		if (!(fArmed && !fExpiring && pie_watchdog_handback(true))) {
			if (fArmed) {
				disarm();
			}
			pie_watchdog_handback(false);
		}
		return kIOReturnSuccess;
	});
	/* The check-ins off; IOWatchDogTimer's notifier removed. */
	fInternal = current_thread();
	super::stop(provider);
	fInternal = NULL;
	fWorkLoop->removeEventSource(fTimer);
	fTimerAdded = false;
	fWorkLoop->removeEventSource(fGate);
	fGateAdded = false;
	OSCompareAndSwapPtr(this, NULL, &sWatchdog);
	fOwnInstance = false;
	/* IOWatchDogTimer::stop does not call it. */
	IOService::stop(provider);
}

void
RPiWatchdog::free(void)
{
	/* Also covers partial startup, before the hardware was armed. */
	if (fHaltNotifier != NULL) {
		fHaltNotifier->remove();
		fHaltNotifier = NULL;
	}
	if (fTimer != NULL) {
		fTimer->cancelTimeout();
	}
	if (fTimerAdded) {
		fWorkLoop->removeEventSource(fTimer);
	}
	if (fGateAdded) {
		fWorkLoop->removeEventSource(fGate);
	}
	if (fOwnInstance) {
		OSCompareAndSwapPtr(this, NULL, &sWatchdog);
	}
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fWorkLoop);
	OSSafeReleaseNULL(fMap);
	super::free();
}

/* The rest on the work loop. */

void
RPiWatchdog::pet(void)
{
	wr(PM_WDOG, (fTimeoutS << 16) & PM_WDOG_TIME);
	wr(PM_RSTC, (rd(PM_RSTC) & ~(PM_PASSWORD_MASK | PM_RSTC_WRCFG)) | PM_RSTC_WRCFG_FULL_RESET);
	fPets++;
}

void
RPiWatchdog::arm(void)
{
	pet();
	fArmed = true;
	fExpiring = false;
}

void
RPiWatchdog::disarm(void)
{
	wr(PM_RSTC, PM_RSTC_RESET);
	fArmed = false;
	fExpiring = false;
}

void
RPiWatchdog::updateState(void)
{
	setProperty("WatchdogState", fExpiring ? "expiring" : fArmed ? "armed" : fQEMU ? "qemu" : "disarmed");
	OSNumber *n = OSNumber::withNumber(fTimeoutS, 32);
	if (n != NULL) {
		setProperty("WatchdogTimeout", n);
		n->release();
	}
}

void
RPiWatchdog::tick(void)
{
	if (fStopping) {
		return;
	}
	if (fArmed && !fExpiring) {
		pet();
	}
	if (fUserS != 0 && mach_absolute_time() >= fUserDeadline) {
		panic("RPiWatchdog: no check-in from user space for %u s (IOWatchDogTimer, set by pid %d, %s)",
		    fUserS, fUserPid, fUserName);
	}
	fTimer->setTimeoutMS(kPetMs);
}

void
RPiWatchdog::timerFired(OSObject *owner, IOTimerEventSource *)
{
	((RPiWatchdog *)owner)->tick();
}

/* IOWatchDogTimer's: from setProperties (an administrator's process), start, stop. */
void
RPiWatchdog::setWatchDogTimer(UInt32 timeOut)
{
	uint32_t s = timeOut > kMaxUserS ? kMaxUserS : (uint32_t)timeOut;
	/*
	 * IOWatchDogTimer's start, stop and sleep handler call this too. They
	 * must not end the boot deadline: only user space can. The sleep
	 * handler runs in the kernel, pid 0. start and stop may run on a user
	 * thread, so their own thread's calls do not count as user calls.
	 */
	bool fromUser = proc_selfpid() != 0 && current_thread() != fInternal;
	int pid = s != 0 ? proc_selfpid() : 0;
	char buf[sizeof(fUserName)] = "";
	if (s != 0) {
		proc_selfname(buf, sizeof(buf));
	}
	const char *name = buf;         /* blocks take no arrays */
	fGate->runActionBlock(^IOReturn {
		if (fromUser) {
			/* Gated, as stop's restore: a check-in before the takeover ends the kernel's deadline too. */
			uint32_t unused;
			(void)pie_watchdog_boot_deadline(&unused);
			fUserSet = true;
			fBootDue = false;
		} else if (fBootDue) {
			return kIOReturnSuccess;
		}
		/* A queued daemon call must not recreate a deadline after PagingOff. */
		if (s != 0 && (fUserShutdown || fStopping)) {
			setProperty("IOWatchDogEnabled", kOSBooleanFalse);
			return kIOReturnSuccess;
		}
		if (s == 0) {
			if (fUserS != 0) {
				IOLog("RPiWatchdog: check-ins end (pid %d, %s)\n", pid != 0 ? pid : fUserPid, fUserName);
			}
			fUserS = 0;
			fUserPid = 0;
			fUserName[0] = '\0';
			return kIOReturnSuccess;
		}
		if (fUserS == 0 || fUserPid != pid) {
			IOLog("RPiWatchdog: check-ins every %u s from pid %d (%s)\n", s, pid, name);
		}
		clock_interval_to_deadline(s, kSecondScale, &fUserDeadline);
		fUserS = s;
		fUserPid = pid;
		strlcpy(fUserName, name, sizeof(fUserName));
		return kIOReturnSuccess;
	});
}

/*
 * A halt or a restart: the check-ins end, at kIOMessageSystemPagingOff
 * already (the kernel's reboot begins, before it kills every process: a
 * daemon killed without setting 0 must not panic the shutdown); a halt
 * disarms the hardware.
 */
IOReturn
RPiWatchdog::haltRestart(void *target, void *, UInt32 messageType, IOService *, void *, vm_size_t)
{
	RPiWatchdog *me = (RPiWatchdog *)target;
	if (messageType != kIOMessageSystemWillPowerOff && messageType != kIOMessageSystemWillRestart &&
	    messageType != kIOMessageSystemPagingOff) {
		return kIOReturnUnsupported;
	}
	me->fGate->runActionBlock(^IOReturn {
		me->fUserShutdown = true;
		me->fUserS = 0;
		me->fUserPid = 0;
		me->fUserName[0] = '\0';
		me->setProperty("IOWatchDogEnabled", kOSBooleanFalse);
		if (messageType == kIOMessageSystemWillPowerOff && me->fArmed) {
			me->disarm();
			me->updateState();
		}
		return kIOReturnSuccess;
	});
	return kIOReturnSuccess;
}

int
RPiWatchdog::setEnabled(bool on)
{
	fGate->runActionBlock(^IOReturn {
		if (on && !fArmed) {
			if (fQEMU) {
				IOLog("RPiWatchdog: armed by hw.watchdog.enabled; QEMU's model resets at once\n");
			}
			arm();
			IOLog("RPiWatchdog: armed, %u s\n", fTimeoutS);
		} else if (!on && fArmed) {
			disarm();
			IOLog("RPiWatchdog: disarmed by hw.watchdog.enabled\n");
		}
		updateState();
		return kIOReturnSuccess;
	});
	return 0;
}

int
RPiWatchdog::setTimeout(int s)
{
	if (s < kMinTimeoutS || s > kMaxTimeoutS) {
		return EINVAL;
	}
	fGate->runActionBlock(^IOReturn {
		fTimeoutS = (uint32_t)s;
		if (fArmed && !fExpiring) {
			pet();
		}
		updateState();
		return kIOReturnSuccess;
	});
	return 0;
}

int
RPiWatchdog::setExpiring(bool on)
{
	IOReturn ret = fGate->runActionBlock(^IOReturn {
		if (!fArmed) {
			return kIOReturnNotReady;
		}
		if (on && !fExpiring) {
			IOLog("RPiWatchdog: no longer set again (hw.watchdog.test_expire); the board resets in %u s\n",
			    fTimeoutS);
		} else if (!on && fExpiring) {
			pet();
		}
		fExpiring = on;
		updateState();
		return kIOReturnSuccess;
	});
	return ret == kIOReturnSuccess ? 0 : ENXIO;
}

int
RPiWatchdog::remainingMs(void)
{
	int ms = -1, *out = &ms;        /* by pointer: kxld has no block runtime for byref captures */
	fGate->runActionBlock(^IOReturn {
		if (fArmed && hardwareArmed()) {
			*out = (int)(((uint64_t)(rd(PM_WDOG) & PM_WDOG_TIME) * 1000) >> 16);
		}
		return kIOReturnSuccess;
	});
	return ms;
}

int
RPiWatchdog::userRemainingMs(void)
{
	int ms = -1, *out = &ms;
	fGate->runActionBlock(^IOReturn {
		if (fUserS != 0) {
			uint64_t now = mach_absolute_time(), ns = 0;
			if (fUserDeadline > now) {
				absolutetime_to_nanoseconds(fUserDeadline - now, &ns);
			}
			*out = (int)(ns / 1000000);
		}
		return kIOReturnSuccess;
	});
	return ms;
}

#pragma mark sysctls

enum { WD_ENABLED, WD_TIMEOUT, WD_REMAINING, WD_PETS, WD_USER_TIMEOUT, WD_USER_REMAINING, WD_USER_PID, WD_TEST_EXPIRE };

static int
sysctl_watchdog(struct sysctl_oid *oidp, void *, int which, struct sysctl_req *req)
{
	RPiWatchdog *w = sWatchdog;
	if (w == NULL) {
		return ENOENT;
	}
	int v;
	switch (which) {
	case WD_ENABLED: v = w->fArmed; break;
	case WD_TIMEOUT: v = (int)w->fTimeoutS; break;
	case WD_REMAINING: v = w->remainingMs(); break;
	case WD_PETS: v = (int)w->fPets; break;
	case WD_USER_TIMEOUT: v = (int)w->fUserS; break;
	case WD_USER_REMAINING: v = w->userRemainingMs(); break;
	case WD_USER_PID: v = w->fUserPid; break;
	default: v = 0; break;
	}
	int err = sysctl_handle_int(oidp, &v, 0, req);
	if (err != 0 || req->newptr == USER_ADDR_NULL) {
		return err;
	}
	switch (which) {
	case WD_ENABLED: return v == 0 || v == 1 ? w->setEnabled(v) : EINVAL;
	case WD_TIMEOUT: return w->setTimeout(v);
	case WD_TEST_EXPIRE: return v == 0 || v == 1 ? w->setExpiring(v) : EINVAL;
	default: return EPERM;
	}
}

#define WD_OID(name, flags, which, descr) \
	SYSCTL_PROC(_hw_watchdog, OID_AUTO, name, CTLTYPE_INT | (flags) | CTLFLAG_LOCKED, NULL, which, \
	    sysctl_watchdog, "I", descr)

SYSCTL_NODE(_hw, OID_AUTO, watchdog, CTLFLAG_RW | CTLFLAG_LOCKED, NULL, "hardware watchdog");
WD_OID(enabled, CTLFLAG_RW, WD_ENABLED, "1: armed; 0: not");
WD_OID(timeout, CTLFLAG_RW, WD_TIMEOUT, "seconds without the kernel before a reset, 2-15");
WD_OID(remaining, CTLFLAG_RD, WD_REMAINING, "ms the count has left, -1 when not armed");
WD_OID(pets, CTLFLAG_RD, WD_PETS, "times the count was set");
WD_OID(user_timeout, CTLFLAG_RD, WD_USER_TIMEOUT, "seconds between user space's check-ins, 0: none due");
WD_OID(user_remaining, CTLFLAG_RD, WD_USER_REMAINING, "ms to the next check-in's deadline, -1: none");
WD_OID(user_pid, CTLFLAG_RD, WD_USER_PID, "the process that checked in last");
WD_OID(test_expire, CTLFLAG_RW, WD_TEST_EXPIRE, "1: stop setting the count, the board resets; 0: go on");

static struct sysctl_oid *const sOIDs[] = {
	&sysctl__hw_watchdog, &sysctl__hw_watchdog_enabled, &sysctl__hw_watchdog_timeout,
	&sysctl__hw_watchdog_remaining, &sysctl__hw_watchdog_pets, &sysctl__hw_watchdog_user_timeout,
	&sysctl__hw_watchdog_user_remaining, &sysctl__hw_watchdog_user_pid, &sysctl__hw_watchdog_test_expire,
};

void
RPiWatchdog::registerSysctls(void)
{
	if (!fOwnInstance) {
		return;
	}
	fSysctls = true;
	for (unsigned i = 0; i < sizeof(sOIDs) / sizeof(sOIDs[0]); i++) {
		sysctl_register_oid(sOIDs[i]);
	}
}

/* Leaves first, then the node; sysctl_unregister_oid waits for handlers that are running. */
void
RPiWatchdog::unregisterSysctls(void)
{
	if (!fSysctls) {
		return;
	}
	for (unsigned i = sizeof(sOIDs) / sizeof(sOIDs[0]); i-- > 0;) {
		sysctl_unregister_oid(sOIDs[i]);
	}
	fSysctls = false;
}
