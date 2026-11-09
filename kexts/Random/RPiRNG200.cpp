/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * RPiRNG200: the hardware random-number generator of the Raspberry Pi 4
 * (BCM2711) and 5 (BCM2712), Broadcom's iproc RNG200
 * (brcm,bcm2711-rng200 on both), as an entropy source for the kernel's
 * PRNG: every RNGReseedInterval seconds (the personality's, 60) it reseeds
 * that PRNG with 64 bytes of the RNG's through write_random(), as xnu's own
 * sources do. /dev/random, read_random() and arc4random are the PRNG's;
 * the RNG's own bytes go nowhere else.
 *
 * Its registers: CTRL 0x00 (RBGEN, bits 12:0, 1 enables the generator),
 * RNG_SOFT_RESET 0x04 and RBG_SOFT_RESET 0x08 (bit 0), INT_STATUS 0x18
 * (bit 31 MASTER_FAIL_LOCKOUT and bit 5 NIST_FAIL, its own health tests;
 * write 1s to clear), FIFO_DATA 0x20 (a 32-bit word) and FIFO_COUNT 0x24
 * (bits 7:0, the words ready). It has no interrupt in the device trees: the
 * driver polls, on its own work loop.
 *
 * Nothing it reads reaches the PRNG unless:
 * - the generator's health tests show no failure, before and after each
 *   batch;
 * - no word equals the one before it (FIPS 140-2's continuous test);
 * - FIPS 140-2's power-up tests (<RPiRNGTests.h>) passed on 20,000 bits,
 *   read and thrown away after the driver started or last reset the RNG.
 * A failure resets the RNG (the generator off, the status cleared, both
 * soft resets pulsed, on again) and starts over with the power-up tests at
 * the next tick. An RNG that gives no data is logged once and tried again
 * every tick; the kernel goes on with its other entropy, the boot seed and
 * interrupt timing.
 *
 * QEMU's raspi4b has the older BCM2835 RNG's model at that address, 16
 * bytes of registers with nothing behind them: a read past them aborts.
 * probe() tells the two apart by the word at 0x04, which only reads it
 * may touch: the RNG200's RNG_SOFT_RESET (bit 0), the BCM2835 RNG's
 * status, whose top byte is its words ready (QEMU's always 1 or more).
 *
 * Registry properties, for ioreg: RNGState (starting, running, no-data,
 * failed), RNGSelfTest (pending, passed, failed: TEST), RNGBytesFed,
 * RNGResets, RNGReseedInterval.
 */

#include <IOKit/IOCommandGate.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOWorkLoop.h>
#include <kern/clock.h>
#include <string.h>
#include <sys/random.h>

#include "RPiRNGTests.h"

enum {
	RNG_CTRL = 0x00, RNG_CTRL_RBGEN = 0x1fff, RNG_CTRL_RBGEN_ENABLE = 1,
	RNG_SOFT_RESET = 0x04, RBG_SOFT_RESET = 0x08, SOFT_RESET = 1,
	BCM2835_RNG_STATUS_WORDS = 0xffU << 24,         /* at 0x04 on the BCM2835's RNG */
	RNG_INT_STATUS = 0x18,
	RNG_INT_FAILED = (1U << 31) | (1U << 5),        /* MASTER_FAIL_LOCKOUT, NIST_FAIL */
	RNG_FIFO_DATA = 0x20,
	RNG_FIFO_COUNT = 0x24, RNG_FIFO_COUNT_WORDS = 0xff,
	RNG_REGS_SIZE = 0x28,
};

#define kSeedBytes      64              /* write_random() takes up to a SHA-512's as it is */
#define kWaitMs         1000            /* with no new word, before it is no-data */

class RPiRNG200 : public IOService
{
	OSDeclareDefaultStructors(RPiRNG200);

public:
	virtual IOService *probe(IOService *provider, SInt32 *score) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

private:
	enum Result { kOK, kNoData, kFailed, kRepeated };

	IOMemoryMap *fMap;
	volatile uint32_t *fRegs;
	IOWorkLoop *fWorkLoop;
	IOCommandGate *fGate;
	IOTimerEventSource *fTimer;
	uint32_t fIntervalS;
	uint64_t fBytesFed;
	uint32_t fResets;
	uint32_t fLastWord;
	bool fHaveLast, fTested, fAnnounced, fNoDataLogged;
	volatile bool fStopping;

	uint32_t rd(unsigned off) { return fRegs[off / 4]; }
	void wr(unsigned off, uint32_t v) { fRegs[off / 4] = v; }
	void reset(void);
	Result collect(uint8_t *buf, unsigned len);
	void tick(void);
	void noData(void);
	void setNumber(const char *key, uint64_t value, unsigned bits);
	static void timerFired(OSObject *owner, IOTimerEventSource *sender);
};

OSDefineMetaClassAndStructors(RPiRNG200, IOService);
#define super IOService

IOService *
RPiRNG200::probe(IOService *provider, SInt32 *score)
{
	if (super::probe(provider, score) == NULL) {
		return NULL;
	}
	IOMemoryMap *map = provider->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache);
	if (map == NULL || map->getLength() < RNG_REGS_SIZE) {
		IOLog("RPiRNG200: cannot map its registers\n");
		OSSafeReleaseNULL(map);
		return NULL;
	}
	uint32_t word = ((volatile uint32_t *)map->getVirtualAddress())[RNG_SOFT_RESET / 4];
	map->release();
	if ((word & BCM2835_RNG_STATUS_WORDS) != 0) {
		IOLog("RPiRNG200: not an RNG200 (0x%08x at 0x04: the BCM2835 RNG's status, as QEMU's raspi4b has); "
		    "not driven\n", word);
		return NULL;
	}
	return this;
}

bool
RPiRNG200::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	fMap = provider->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache);
	if (fMap == NULL || fMap->getLength() < RNG_REGS_SIZE) {
		IOLog("RPiRNG200: cannot map its registers\n");
		return false;
	}
	fRegs = (volatile uint32_t *)fMap->getVirtualAddress();

	OSNumber *interval = OSDynamicCast(OSNumber, getProperty("RNGReseedInterval"));
	fIntervalS = interval != NULL ? interval->unsigned32BitValue() : 60;
	fIntervalS = fIntervalS < 1 ? 1 : fIntervalS > 3600 ? 3600 : fIntervalS;
	setNumber("RNGReseedInterval", fIntervalS, 32);
	setNumber("RNGBytesFed", 0, 64);
	setNumber("RNGResets", 0, 32);
	setProperty("RNGState", "starting");
	setProperty("RNGSelfTest", "pending");

	fWorkLoop = IOWorkLoop::workLoop();
	fGate = fWorkLoop != NULL ? IOCommandGate::commandGate(this) : NULL;
	fTimer = fWorkLoop != NULL ? IOTimerEventSource::timerEventSource(this, timerFired) : NULL;
	if (fGate == NULL || fTimer == NULL || fWorkLoop->addEventSource(fGate) != kIOReturnSuccess ||
	    fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess) {
		return false;
	}
	fGate->runActionBlock(^IOReturn {
		/* The firmware leaves it running; a generator that is off, or failed, starts over. */
		if ((rd(RNG_INT_STATUS) & RNG_INT_FAILED) != 0) {
			IOLog("RPiRNG200: its health tests had failed (INT_STATUS 0x%08x); the RNG reset\n",
			    rd(RNG_INT_STATUS));
			reset();
		} else if ((rd(RNG_CTRL) & RNG_CTRL_RBGEN) == 0) {
			wr(RNG_CTRL, (rd(RNG_CTRL) & ~RNG_CTRL_RBGEN) | RNG_CTRL_RBGEN_ENABLE);
		}
		fTimer->setTimeoutMS(1);
		return kIOReturnSuccess;
	});
	registerService();
	return true;
}

void
RPiRNG200::stop(IOService *provider)
{
	fStopping = true;
	if (fTimer != NULL) {
		fTimer->cancelTimeout();
	}
	if (fGate != NULL) {
		/* Waits for a tick that is running (it sees fStopping within a millisecond). */
		fGate->runActionBlock(^IOReturn {
			fTimer->cancelTimeout();
			return kIOReturnSuccess;
		});
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
RPiRNG200::free(void)
{
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fWorkLoop);
	OSSafeReleaseNULL(fMap);
	fLastWord = 0;
	super::free();
}

void
RPiRNG200::setNumber(const char *key, uint64_t value, unsigned bits)
{
	OSNumber *n = OSNumber::withNumber(value, bits);
	if (n != NULL) {
		setProperty(key, n);
		n->release();
	}
}

/* On the work loop: the generator off, its status cleared, both soft resets pulsed, on again. */
void
RPiRNG200::reset(void)
{
	uint32_t ctrl = rd(RNG_CTRL) & ~RNG_CTRL_RBGEN;
	wr(RNG_CTRL, ctrl);
	wr(RNG_INT_STATUS, 0xffffffff);
	wr(RBG_SOFT_RESET, rd(RBG_SOFT_RESET) | SOFT_RESET);
	wr(RNG_SOFT_RESET, rd(RNG_SOFT_RESET) | SOFT_RESET);
	wr(RNG_SOFT_RESET, rd(RNG_SOFT_RESET) & ~SOFT_RESET);
	wr(RBG_SOFT_RESET, rd(RBG_SOFT_RESET) & ~SOFT_RESET);
	wr(RNG_CTRL, ctrl | RNG_CTRL_RBGEN_ENABLE);
	fHaveLast = false;
	fTested = false;
	setProperty("RNGSelfTest", "pending");
	setNumber("RNGResets", ++fResets, 32);
}

/*
 * On the work loop: len bytes from the FIFO, the generator's health and
 * the continuous test checked; kNoData when a second goes by with no new
 * word (or the driver is stopping). Each wait spins up to 50 us, then
 * sleeps a millisecond at a time.
 */
RPiRNG200::Result
RPiRNG200::collect(uint8_t *buf, unsigned len)
{
	uint64_t deadline;
	clock_interval_to_deadline(kWaitMs, kMillisecondScale, &deadline);
	unsigned got = 0, spins = 0;
	while (got < len) {
		if (fStopping) {
			return kNoData;
		}
		if ((rd(RNG_INT_STATUS) & RNG_INT_FAILED) != 0) {
			return kFailed;
		}
		unsigned words = rd(RNG_FIFO_COUNT) & RNG_FIFO_COUNT_WORDS;
		if (words == 0) {
			if (mach_absolute_time() >= deadline) {
				return kNoData;
			}
			if (spins++ < 50) {
				IODelay(1);
			} else {
				IOSleep(1);
			}
			continue;
		}
		for (; words > 0 && got < len; words--) {
			uint32_t w = rd(RNG_FIFO_DATA);
			if (fHaveLast && w == fLastWord) {
				return kRepeated;
			}
			fLastWord = w;
			fHaveLast = true;
			unsigned n = len - got < 4 ? len - got : 4;
			memcpy(buf + got, &w, n);
			got += n;
		}
		clock_interval_to_deadline(kWaitMs, kMillisecondScale, &deadline);
		spins = 0;
	}
	return (rd(RNG_INT_STATUS) & RNG_INT_FAILED) != 0 ? kFailed : kOK;
}

void
RPiRNG200::noData(void)
{
	setProperty("RNGState", "no-data");
	if (!fNoDataLogged) {
		IOLog("RPiRNG200: no data from the RNG in %u ms (FIFO_COUNT 0x%08x); trying again every %u s\n", kWaitMs,
		    rd(RNG_FIFO_COUNT), fIntervalS);
		fNoDataLogged = true;
	}
}

/* On the work loop: the power-up tests if they are due, then a reseed; the next tick. */
void
RPiRNG200::tick(void)
{
	if (fStopping) {
		return;
	}
	if (!fTested) {
		uint8_t *bits = (uint8_t *)IOMallocData(kRNGTestBytes);
		if (bits == NULL) {
			fTimer->setTimeoutMS(1000);
			return;
		}
		Result r = collect(bits, kRNGTestBytes);
		const char *failed = r == kFailed ? "health" : r == kRepeated ? "continuous" :
		    r == kOK ? RNGTestFIPS140(bits) : NULL;
		memset_s(bits, kRNGTestBytes, 0, kRNGTestBytes);
		IOFreeData(bits, kRNGTestBytes);
		if (fStopping) {
			return;
		}
		if (r == kNoData) {
			noData();
			fTimer->setTimeoutMS(fIntervalS * 1000);
			return;
		}
		if (failed != NULL) {
			char text[32];
			snprintf(text, sizeof(text), "failed: %s", failed);
			IOLog("RPiRNG200: power-up test %s (INT_STATUS 0x%08x); the RNG reset\n", text,
			    rd(RNG_INT_STATUS));
			setProperty("RNGState", "failed");
			reset();
			setProperty("RNGSelfTest", text);
			fTimer->setTimeoutMS(fIntervalS * 1000);
			return;
		}
		fTested = true;
		setProperty("RNGSelfTest", "passed");
	}

	uint8_t seed[kSeedBytes];
	Result r = collect(seed, sizeof(seed));
	if (fStopping) {
		memset_s(seed, sizeof(seed), 0, sizeof(seed));
		return;
	}
	if (r == kOK) {
		write_random(seed, sizeof(seed));
		setNumber("RNGBytesFed", fBytesFed += sizeof(seed), 64);
		setProperty("RNGState", "running");
		if (!fAnnounced) {
			IOLog("RPiRNG200: power-up tests passed; reseeding the kernel's PRNG every %u s\n", fIntervalS);
			fAnnounced = true;
		}
		fNoDataLogged = false;
	} else if (r == kNoData) {
		noData();
	} else {
		IOLog("RPiRNG200: %s test failed (INT_STATUS 0x%08x); the RNG reset\n",
		    r == kRepeated ? "continuous" : "health", rd(RNG_INT_STATUS));
		setProperty("RNGState", "failed");
		reset();
	}
	memset_s(seed, sizeof(seed), 0, sizeof(seed));
	fTimer->setTimeoutMS(fIntervalS * 1000);
}

void
RPiRNG200::timerFired(OSObject *owner, IOTimerEventSource *)
{
	((RPiRNG200 *)owner)->tick();
}
