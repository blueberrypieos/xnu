/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * RPiCPUFreq: the Raspberry Pi's CPU clock. Every core runs on one clock,
 * the firmware's ARM clock (id 3), which this driver sets through the
 * VideoCore firmware's mailbox: GET_MIN_CLOCK_RATE 0x00030007,
 * GET_MAX_CLOCK_RATE 0x00030004, GET_CLOCK_RATE 0x00030002,
 * GET_CLOCK_MEASURED 0x00030047 (each {clock, rate in Hz}) and
 * SET_CLOCK_RATE 0x00038002 ({clock, rate, skip turbo}: 1, as Linux's
 * clock driver sends it, else the firmware's turbo mode would move the
 * core, V3D and SDRAM clocks with the ARM's); the firmware sets the core
 * voltage to go with the rate, and caps the rate itself when the
 * SoC is too hot or the supply too low. GET_THROTTLED 0x00030046 says
 * whether it does: bits 0-3 now (undervoltage, frequency capped,
 * throttled, soft temperature limit), bits 16-19 the same since boot.
 *
 * A governor (CPUFreqPolicy.h) picks the rate every sample_ms (100) from
 * the busiest CPU's load over the last interval (<pie/cpu.h>, the
 * scheduler's own counts): ondemand by default, performance or powersave.
 * The driver logs each change in the firmware's throttling, fills in
 * hw.cpufrequency, _min and _max, and has the hw.cpufreq sysctls: current,
 * measured, min, max, load, throttled, throttled_since_boot, transitions,
 * and, for root, governor, max_limit (Hz, 0: none) and sample_ms. As it
 * unloads, the clock goes back to the rate it found.
 *
 * Under QEMU (its firmware's revision is not the VideoCore's), or with the
 * boot-arg pie_cpufreq=0, the governor follows the load but sets no
 * rate: the registry's CPUFreqControl says firmware or none.
 */

#include <IOKit/IOCommandGate.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOWorkLoop.h>
#include <pexpert/pexpert.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/sysctl.h>

#include <pie/cpu.h>
#include <pie/rpi.h>

#include "CPUFreqPolicy.h"

#define kMaxCPUs        16

enum {
	FW_GET_FIRMWARE_REVISION = 0x00000001,
	FW_VIDEOCORE_SINCE = 1546300800,        /* 2019-01-01 UTC: older than any BCM2711 or BCM2712 firmware */
	FW_GET_CLOCK_RATE = 0x00030002, FW_GET_MAX_CLOCK_RATE = 0x00030004, FW_GET_MIN_CLOCK_RATE = 0x00030007,
	FW_GET_CLOCK_MEASURED = 0x00030047, FW_SET_CLOCK_RATE = 0x00038002,
	FW_GET_THROTTLED = 0x00030046,
	FW_CLOCK_ARM = 3,
	THROTTLED_NOW = 0xf, THROTTLED_SINCE_SHIFT = 16,
};

static const char *const kThrottledNames[] = {
	"undervoltage", "frequency capped", "throttled", "soft temperature limit",
};

class RPiCPUFreq : public IOService
{
	OSDeclareDefaultStructors(RPiCPUFreq);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	int setGovernor(int governor);
	int setLimit(uint32_t hz);
	int setSampleMs(unsigned ms);

	bool fControl;
	int fGovernor;
	uint32_t fMin, fMax, fCur, fTarget, fLimit;
	unsigned fSampleMs, fLoad;
	uint64_t fTransitions;

private:
	IOWorkLoop *fWorkLoop;
	IOCommandGate *fGate;
	IOTimerEventSource *fTimer;
	bool fSysctls, fPublished, fSetFailed;
	bool fGateAdded, fTimerAdded, fStopping, fClaimed;
	uint32_t fInitial, fThrottled;
	unsigned fCPUs, fSinceThrottled, fRetryMs;
	uint64_t fBusy[kMaxCPUs], fIdle[kMaxCPUs];
	uint64_t fSavedHz, fSavedMinHz, fSavedMaxHz;
	unsigned long fSavedClockRate;

	void cleanup(void);
	unsigned load(void);
	void sample(void);
	void apply(void);
	void setRate(uint32_t hz);
	void checkThrottled(void);
	void publishFrequency(void);
	void registerSysctls(void);
	void unregisterSysctls(void);
	static void timerFired(OSObject *owner, IOTimerEventSource *sender);
};

OSDefineMetaClassAndStructors(RPiCPUFreq, IOService);
#define super IOService

/* The instance the sysctls reach; set while they are registered. */
static RPiCPUFreq *sCPUFreq;

/* A firmware clock rate of the ARM clock, or 0. */
static uint32_t
armRate(uint32_t tag)
{
	uint32_t data[2] = { FW_CLOCK_ARM, 0 };
	if (RPiFirmwareProperty(tag, data, sizeof(data)) != kIOReturnSuccess) {
		return 0;
	}
	return data[1];
}

/* The firmware's throttling word, or 0. */
static uint32_t
throttledWord(void)
{
	uint32_t data[1] = { 0 };
	if (RPiFirmwareProperty(FW_GET_THROTTLED, data, sizeof(data)) != kIOReturnSuccess) {
		return 0;
	}
	return data[0];
}

#pragma mark start and stop

bool
RPiCPUFreq::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	/* Claim before reading/changing shared clock state, including startup failures. */
	if (!OSCompareAndSwapPtr(NULL, this, &sCPUFreq)) { return false; }
	fClaimed = true;
	fCPUs = pie_cpu_count();
	if (fCPUs > kMaxCPUs) {
		fCPUs = kMaxCPUs;
	}
	uint64_t t[3];
	if (fCPUs == 0 || pie_cpu_times(0, t) != KERN_SUCCESS) {
		IOLog("RPiCPUFreq: no CPU times from the kernel\n");
		return false;
	}
	fGovernor = kCPUFreqOndemand;
	fSampleMs = 100;

	uint32_t revision = 0, on = 1;
	PE_parse_boot_argn("pie_cpufreq", &on, sizeof(on));
	if (RPiFirmwareProperty(FW_GET_FIRMWARE_REVISION, &revision, sizeof(revision)) != kIOReturnSuccess ||
	    revision < FW_VIDEOCORE_SINCE) {
		IOLog("RPiCPUFreq: firmware revision %u is not the VideoCore's (QEMU's: 346337); "
		    "the load only, no rates set\n", revision);
	} else if (on == 0) {
		IOLog("RPiCPUFreq: pie_cpufreq=0: the load only, no rates set\n");
	} else {
		fMin = armRate(FW_GET_MIN_CLOCK_RATE);
		fMax = armRate(FW_GET_MAX_CLOCK_RATE);
		fCur = fTarget = fInitial = armRate(FW_GET_CLOCK_RATE);
		if (fMin == 0 || fMax < fMin || fCur == 0) {
			IOLog("RPiCPUFreq: the firmware's ARM clock rates (min %u, max %u, now %u) make no sense; "
			    "no rates set\n", fMin, fMax, fCur);
			fMin = fMax = fCur = fTarget = fInitial = 0;
		} else {
			fControl = true;
		}
	}
	setProperty("CPUFreqControl", fControl ? "firmware" : "none");
	if (fControl) {
		setProperty("CPUFreqMinHz", fMin, 32);
		setProperty("CPUFreqMaxHz", fMax, 32);
	}

	fWorkLoop = IOWorkLoop::workLoop();
	fGate = fWorkLoop != NULL ? IOCommandGate::commandGate(this) : NULL;
	fTimer = fWorkLoop != NULL ? IOTimerEventSource::timerEventSource(this, timerFired) : NULL;
	if (fGate == NULL || fTimer == NULL) { return false; }
	if (fWorkLoop->addEventSource(fGate) != kIOReturnSuccess) { return false; }
	fGateAdded = true;
	if (fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess) { return false; }
	fTimerAdded = true;
	load();         /* the first interval starts now */
	if (fControl) {
		fSavedHz = gPEClockFrequencyInfo.cpu_frequency_hz;
		fSavedMinHz = gPEClockFrequencyInfo.cpu_frequency_min_hz;
		fSavedMaxHz = gPEClockFrequencyInfo.cpu_frequency_max_hz;
		fSavedClockRate = gPEClockFrequencyInfo.cpu_clock_rate_hz;
		fPublished = true;
		publishFrequency();
		fThrottled = throttledWord();
		uint32_t since = fThrottled >> THROTTLED_SINCE_SHIFT;
		IOLog("RPiCPUFreq: ARM clock %u-%u MHz, now %u MHz; governor %s; throttling 0x%x\n",
		    fMin / 1000000, fMax / 1000000, fCur / 1000000, kCPUFreqGovernorNames[fGovernor], fThrottled);
		for (unsigned b = 0; b < 4; b++) {
			if (since & (1U << b)) {
				IOLog("RPiCPUFreq: since boot: %s\n", kThrottledNames[b]);
			}
		}
	}
	registerSysctls();
	registerService();
	fGate->runActionBlock(^IOReturn {
		fTimer->setTimeoutMS(fSampleMs);
		return kIOReturnSuccess;
	});
	return true;
}

void
RPiCPUFreq::cleanup(void)
{
	unregisterSysctls();
	if (fGateAdded) {
		fGate->runActionBlock(^IOReturn {
			fStopping = true;
			if (fTimer != NULL) { fTimer->cancelTimeout(); }
			if (fControl && (fTarget != fInitial || fSetFailed)) {
				for (unsigned attempt = 0; attempt < 3; attempt++) {
					setRate(fInitial);
					if (!fSetFailed) { break; }
					if (attempt < 2) { IOSleep(10); }
				}
				if (fSetFailed) {
					uint32_t actual = armRate(FW_GET_CLOCK_MEASURED);
					if (actual != 0) { fCur = actual; }
					IOLog("RPiCPUFreq: could not restore initial ARM clock %u Hz; "
					    "last known rate %u Hz (readback %u)\n", fInitial, fCur, actual);
				}
			}
			return kIOReturnSuccess;
		});
	} else {
		fStopping = true;
		if (fTimer != NULL) { fTimer->cancelTimeout(); }
	}
	if (fPublished) {
		gPEClockFrequencyInfo.cpu_frequency_hz = fSetFailed ? fCur : fSavedHz;
		gPEClockFrequencyInfo.cpu_frequency_min_hz = fSavedMinHz;
		gPEClockFrequencyInfo.cpu_frequency_max_hz = fSavedMaxHz;
		gPEClockFrequencyInfo.cpu_clock_rate_hz = fSetFailed ? fCur : fSavedClockRate;
		fPublished = false;
	}
	if (fTimerAdded) { fWorkLoop->removeEventSource(fTimer); fTimerAdded = false; }
	if (fGateAdded) { fWorkLoop->removeEventSource(fGate); fGateAdded = false; }
	if (fClaimed) { OSCompareAndSwapPtr(this, NULL, &sCPUFreq); fClaimed = false; }
}

void
RPiCPUFreq::stop(IOService *provider)
{
	cleanup();
	super::stop(provider);
}

void
RPiCPUFreq::free(void)
{
	cleanup();
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fWorkLoop);
	super::free();
}

#pragma mark the governor

/* The busiest CPU's load since the last call, in percent. */
unsigned
RPiCPUFreq::load(void)
{
	unsigned top = 0;
	for (unsigned cpu = 0; cpu < fCPUs; cpu++) {
		uint64_t t[3];
		if (pie_cpu_times(cpu, t) != KERN_SUCCESS) {
			continue;
		}
		uint64_t busy = t[PIE_CPU_USER] + t[PIE_CPU_SYSTEM], idle = t[PIE_CPU_IDLE];
		if (busy >= fBusy[cpu] && idle >= fIdle[cpu]) {
			unsigned pct = CPUFreqLoadPct(busy - fBusy[cpu], idle - fIdle[cpu]);
			top = pct > top ? pct : top;
		}
		fBusy[cpu] = busy;
		fIdle[cpu] = idle;
	}
	return top;
}

/* On the work loop. */
void
RPiCPUFreq::sample(void)
{
	if (fStopping) { return; }
	fLoad = load();
	if (fSetFailed && fRetryMs < 1000) { fRetryMs += fSampleMs; }
	apply();
	fSinceThrottled += fSampleMs;
	if (fControl && fSinceThrottled >= 1000) {
		fSinceThrottled = 0;
		checkThrottled();
	}
	fTimer->setTimeoutMS(fSampleMs);
}

/* The governor's rate for the load last measured; on the work loop. */
void
RPiCPUFreq::apply(void)
{
	if (!fControl || (fSetFailed && fRetryMs < 1000)) {
		return;
	}
	/* From the rate asked for: the firmware may round it, or cap it (fCur, what it says it set). */
	uint32_t next = CPUFreqNext(fGovernor, fTarget, fMin, fMax, fLimit, fLoad);
	if (next != fTarget || fSetFailed) {
		setRate(next);
	}
}

/*
 * On the work loop: hz asked for (the governor's next step is from it,
 * after a successful request; failed requests retry at most once a second); fCur the rate
 * the firmware says it set, else the one it reads back.
 */
void
RPiCPUFreq::setRate(uint32_t hz)
{
	uint32_t data[3] = { FW_CLOCK_ARM, hz, 1 };
	fRetryMs = 0;
	fTransitions++;
	if (RPiFirmwareProperty(FW_SET_CLOCK_RATE, data, sizeof(data)) != kIOReturnSuccess) {
		if (!fSetFailed) {
			IOLog("RPiCPUFreq: setting the ARM clock to %u MHz failed\n", hz / 1000000);
		}
		fSetFailed = true;
		return;
	}
	fSetFailed = false;
	fTarget = hz;
	uint32_t now = data[1] != 0 ? data[1] : armRate(FW_GET_CLOCK_RATE);
	if (now != 0 && now != fCur) {
		fCur = now;
		publishFrequency();
	}
}

void
RPiCPUFreq::checkThrottled(void)
{
	uint32_t word = throttledWord();
	uint32_t changed = (word ^ fThrottled) & THROTTLED_NOW;
	for (unsigned b = 0; b < 4; b++) {
		if (changed & (1U << b)) {
			IOLog("RPiCPUFreq: %s: %s (ARM clock %u MHz)\n", kThrottledNames[b],
			    (word & (1U << b)) ? "yes" : "no longer", fCur / 1000000);
		}
	}
	fThrottled = word;
}

void
RPiCPUFreq::publishFrequency(void)
{
	gPEClockFrequencyInfo.cpu_frequency_min_hz = fMin;
	gPEClockFrequencyInfo.cpu_frequency_max_hz = fMax;
	gPEClockFrequencyInfo.cpu_frequency_hz = fCur;
	gPEClockFrequencyInfo.cpu_clock_rate_hz = fCur;
}

void
RPiCPUFreq::timerFired(OSObject *owner, IOTimerEventSource *)
{
	((RPiCPUFreq *)owner)->sample();
}

int
RPiCPUFreq::setGovernor(int governor)
{
	if (governor < 0 || governor >= kCPUFreqGovernors) {
		return EINVAL;
	}
	fGate->runActionBlock(^IOReturn {
		fGovernor = governor;
		apply();
		return kIOReturnSuccess;
	});
	IOLog("RPiCPUFreq: governor %s (hw.cpufreq.governor)\n", kCPUFreqGovernorNames[governor]);
	return 0;
}

int
RPiCPUFreq::setLimit(uint32_t hz)
{
	fGate->runActionBlock(^IOReturn {
		fLimit = hz;
		apply();
		return kIOReturnSuccess;
	});
	return 0;
}

int
RPiCPUFreq::setSampleMs(unsigned ms)
{
	if (ms < 10 || ms > 1000) {
		return EINVAL;
	}
	fGate->runActionBlock(^IOReturn {
		fSampleMs = ms;
		return kIOReturnSuccess;
	});
	return 0;
}

#pragma mark sysctls

enum { CF_CURRENT, CF_MEASURED, CF_MIN, CF_MAX, CF_LIMIT, CF_TRANSITIONS };

static int
sysctl_rate(struct sysctl_oid *oidp, void *, int which, struct sysctl_req *req)
{
	RPiCPUFreq *c = sCPUFreq;
	if (c == NULL) {
		return ENOENT;
	}
	int64_t v = which == CF_CURRENT ? c->fCur : which == CF_MEASURED ? (c->fControl ? armRate(FW_GET_CLOCK_MEASURED) : 0) :
	    which == CF_MIN ? c->fMin : which == CF_MAX ? c->fMax : which == CF_LIMIT ? c->fLimit :
	    (int64_t)c->fTransitions;
	int err = sysctl_handle_quad(oidp, &v, 0, req);
	if (err != 0 || req->newptr == USER_ADDR_NULL) {
		return err;
	}
	if (which != CF_LIMIT) {
		return EPERM;
	}
	if (v < 0 || v > UINT32_MAX) {
		return EINVAL;
	}
	return c->setLimit((uint32_t)v);
}

enum { CF_LOAD, CF_THROTTLED, CF_THROTTLED_SINCE, CF_SAMPLE_MS };

static int
sysctl_int(struct sysctl_oid *oidp, void *, int which, struct sysctl_req *req)
{
	RPiCPUFreq *c = sCPUFreq;
	if (c == NULL) {
		return ENOENT;
	}
	int v;
	switch (which) {
	case CF_LOAD: v = (int)c->fLoad; break;
	case CF_THROTTLED: v = c->fControl ? (int)(throttledWord() & THROTTLED_NOW) : 0; break;
	case CF_THROTTLED_SINCE: v = c->fControl ? (int)((throttledWord() >> THROTTLED_SINCE_SHIFT) & THROTTLED_NOW) : 0; break;
	default: v = (int)c->fSampleMs; break;
	}
	int err = sysctl_handle_int(oidp, &v, 0, req);
	if (err != 0 || req->newptr == USER_ADDR_NULL) {
		return err;
	}
	if (which != CF_SAMPLE_MS) {
		return EPERM;
	}
	return v < 0 ? EINVAL : c->setSampleMs((unsigned)v);
}

static int
sysctl_governor(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req)
{
	RPiCPUFreq *c = sCPUFreq;
	if (c == NULL) {
		return ENOENT;
	}
	char name[16];
	strlcpy(name, kCPUFreqGovernorNames[c->fGovernor], sizeof(name));
	int err = sysctl_handle_string(oidp, name, sizeof(name), req);
	if (err != 0 || req->newptr == USER_ADDR_NULL) {
		return err;
	}
	for (int g = 0; g < kCPUFreqGovernors; g++) {
		if (strcmp(name, kCPUFreqGovernorNames[g]) == 0) {
			return c->setGovernor(g);
		}
	}
	return EINVAL;
}

#define RATE_OID(name, flags, which, descr) \
	SYSCTL_PROC(_hw_cpufreq, OID_AUTO, name, CTLTYPE_QUAD | (flags) | CTLFLAG_LOCKED, NULL, which, \
	    sysctl_rate, "Q", descr)
#define INT_OID(name, flags, which, descr) \
	SYSCTL_PROC(_hw_cpufreq, OID_AUTO, name, CTLTYPE_INT | (flags) | CTLFLAG_LOCKED, NULL, which, \
	    sysctl_int, "I", descr)

SYSCTL_NODE(_hw, OID_AUTO, cpufreq, CTLFLAG_RW | CTLFLAG_LOCKED, NULL, "CPU clock");
RATE_OID(current, CTLFLAG_RD, CF_CURRENT, "the ARM clock's rate, Hz (0: not set by this driver)");
RATE_OID(measured, CTLFLAG_RD, CF_MEASURED, "the firmware's measure of it, Hz");
RATE_OID(min, CTLFLAG_RD, CF_MIN, "the firmware's lowest rate, Hz");
RATE_OID(max, CTLFLAG_RD, CF_MAX, "the firmware's highest rate, Hz");
RATE_OID(max_limit, CTLFLAG_RW, CF_LIMIT, "the governor's highest rate, Hz; 0: max");
RATE_OID(transitions, CTLFLAG_RD, CF_TRANSITIONS, "rates asked of the firmware since the driver loaded");
INT_OID(load, CTLFLAG_RD, CF_LOAD, "the busiest CPU's load over the last sample, percent");
INT_OID(throttled, CTLFLAG_RD, CF_THROTTLED,
    "the firmware now: 1 undervoltage, 2 frequency capped, 4 throttled, 8 soft temperature limit");
INT_OID(throttled_since_boot, CTLFLAG_RD, CF_THROTTLED_SINCE, "the same bits, since boot");
INT_OID(sample_ms, CTLFLAG_RW, CF_SAMPLE_MS, "the governor's interval, 10-1000 ms");
SYSCTL_PROC(_hw_cpufreq, OID_AUTO, governor, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    sysctl_governor, "A", "ondemand, performance or powersave");

static struct sysctl_oid *const sOIDs[] = {
	&sysctl__hw_cpufreq, &sysctl__hw_cpufreq_current, &sysctl__hw_cpufreq_measured, &sysctl__hw_cpufreq_min,
	&sysctl__hw_cpufreq_max, &sysctl__hw_cpufreq_max_limit, &sysctl__hw_cpufreq_transitions,
	&sysctl__hw_cpufreq_load, &sysctl__hw_cpufreq_throttled, &sysctl__hw_cpufreq_throttled_since_boot,
	&sysctl__hw_cpufreq_sample_ms, &sysctl__hw_cpufreq_governor,
};

void
RPiCPUFreq::registerSysctls(void)
{
	fSysctls = true;
	for (unsigned i = 0; i < sizeof(sOIDs) / sizeof(sOIDs[0]); i++) {
		sysctl_register_oid(sOIDs[i]);
	}
}

/* Leaves first, then the node; sysctl_unregister_oid waits for handlers that are running. */
void
RPiCPUFreq::unregisterSysctls(void)
{
	if (!fSysctls) {
		return;
	}
	for (unsigned i = sizeof(sOIDs) / sizeof(sOIDs[0]); i-- > 0;) {
		sysctl_unregister_oid(sOIDs[i]);
	}
	fSysctls = false;
}
