/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOPlatformExpert.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <libkern/OSByteOrder.h>
#include <mach/mach_time.h>

#include "../rpi/RPiPlatform.h"
#include "../sdmmc/PIESDCard.h"
#include "RKSoC.h"

#include <pexpert/pexpert.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>
#pragma clang diagnostic pop

/* DesignWare Mobile Storage Host registers. */
enum {
	DW_CTRL = 0x00, DW_PWREN = 0x04, DW_CLKDIV = 0x08, DW_CLKSRC = 0x0c, DW_CLKENA = 0x10,
	DW_TMOUT = 0x14, DW_CTYPE = 0x18, DW_BLKSIZ = 0x1c, DW_BYTCNT = 0x20, DW_INTMASK = 0x24,
	DW_CMDARG = 0x28, DW_CMD = 0x2c, DW_RESP0 = 0x30, DW_RINTSTS = 0x44, DW_STATUS = 0x48,
	DW_FIFOTH = 0x4c, DW_TCBCNT = 0x5c, DW_TBBCNT = 0x60, DW_VERID = 0x6c, DW_BMOD = 0x80, DW_DATA = 0x200,
};

enum {
	CTRL_RESET = 1U << 0, CTRL_FIFO_RESET = 1U << 1, CTRL_DMA_RESET = 1U << 2,
	CMD_START = 1U << 31, CMD_USE_HOLD = 1U << 29, CMD_UPD_CLK = 1U << 21, CMD_INIT = 1U << 15,
	CMD_STOP_ABORT = 1U << 14, CMD_PRV_DAT_WAIT = 1U << 13, CMD_AUTO_STOP = 1U << 12, CMD_WRITE = 1U << 10, CMD_DATA = 1U << 9,
	CMD_RESP_CRC = 1U << 8, CMD_RESP_LONG = 1U << 7, CMD_RESP_EXP = 1U << 6,
	INT_RE = 1U << 1, INT_CMD_DONE = 1U << 2, INT_DTO = 1U << 3, INT_TXDR = 1U << 4, INT_RXDR = 1U << 5,
	INT_RCRC = 1U << 6, INT_DCRC = 1U << 7, INT_RTO = 1U << 8, INT_DRTO = 1U << 9, INT_HTO = 1U << 10,
	INT_FRUN = 1U << 11, INT_HLE = 1U << 12, INT_SBE = 1U << 13, INT_ACD = 1U << 14, INT_EBE = 1U << 15,
	INT_DATA_ERR = INT_DCRC | INT_DRTO | INT_HTO | INT_FRUN | INT_SBE | INT_EBE,
	INT_CMD_ERR = INT_RE | INT_RCRC | INT_RTO | INT_HLE,
	STATUS_DATA_BUSY = 1U << 9,
	PIO_STALL_MS = 2000,
	WRITE_BUSY_MS = 10000,
	PROGRAMMING_MS = 10000,
	DAT0_WAIT_MS = 500,
	WATCH_MS = 5000,
	LINE_BYTES = 256, HELD_STEPS = 2,
};

/*
 * RK3568 clock unit: the SD clocks' muxes, CLKSEL 30, their parents'
 * dividers, CLKSEL 75, 81 and 82, and the card clock phases.
 */
enum {
	CRU_SDMMC_CON0 = 0x580,
	CRU_PHASE_MASK = 0x6, CRU_PHASE_90 = 0x2,
	CRU_PARENT_CPLL_100M = 3,
};

/* RK3568 PMU GRF: the IO domains' 1.8 V and 3.3 V selects, bit N for VCCIO N. */
enum {
	PMUGRF_IO_VSEL0 = 0x140, PMUGRF_IO_VSEL1 = 0x144,
	IO_1V8 = 1800000, IO_3V3 = 3300000,
};

class RKDWMMC : public PIESDCard
{
	OSDeclareDefaultStructors(RKDWMMC);

	IOMemoryMap *regsMap;
	volatile uint32_t *regs;
	IOBufferMemoryDescriptor *bounceMem;
	uint32_t fifoDepth;
	uint32_t busHz;
	bool dead;
	IOMemoryMap *cruMap;
	uint32_t cruIndex;
	uint32_t drivePhase;
	IOMemoryMap *pmuGrfMap;
	RKRegulators *supply;
	IORegistryEntry *supplyNode;
	uint32_t ioDomains;
	int sdSpec;
	bool noDataSinceReset;
	char slot[24];
	thread_call_t watchCall;
	IOLock *watchLock;
	uint64_t watchStart;
	uint32_t watchIndex, watchArg, watchCount;
	bool watchWrite, watchOut;
	char heldStep[HELD_STEPS][LINE_BYTES];
	unsigned heldSteps;

	uint32_t rd(uint32_t off) { return regs[off / 4]; }
	void wr(uint32_t off, uint32_t v) { regs[off / 4] = v; }
	volatile uint32_t *cru(uint32_t off) { return (volatile uint32_t *)(cruMap->getVirtualAddress() + off); }
	uint64_t parentRate(unsigned sel);

	bool waitClear(uint32_t off, uint32_t mask, uint32_t usec);
	uint32_t waitInt(uint32_t mask, uint32_t usec);
	bool resetFifo(void);
	bool updateClock(uint64_t deadline = 0);
	bool clockRegisters(uint32_t enable, uint32_t divider);
	void waitDat0(const char *what);
	IOReturn command(uint32_t index, uint32_t arg, uint32_t flags, uint32_t data, uint32_t resp[4],
	    uint64_t until = 0);
	bool resetController(void);
	uint32_t cardState(uint64_t until = 0);
	uint32_t pollCardState(void);
	void abortTransfer(void);
	IOReturn readData(uint32_t index, uint32_t arg, void *buf, uint32_t blksz, uint32_t nblk);
	IOReturn writeData(uint32_t index, uint32_t arg, const void *buf, uint32_t nblk);
	uint32_t ciuClock(IOService *provider);
	bool findSupply(IOService *provider);
	IOReturn readSupply(uint32_t *microvolt);
	void setDomains(uint32_t microvolt);
	bool domainsAt(uint32_t microvolt);
	bool gateClock(void);
	bool raiseSupply(void);
	int readSpec(void);
	int cardSignal(void);
	bool setUp(void);
	void setUpHighSpeed(void);
	bool setUpCard(void);
	void stateLine(char *line, size_t size, const char *what, uint64_t since);
	void logState(const char *what, uint64_t since);
	void recoveryStep(const char *what, uint64_t since, bool hold);
	void watchBegin(uint32_t index, uint32_t arg, uint32_t count, bool write);
	void watchEnd(void);
	static void watchFire(thread_call_param_t p0, thread_call_param_t p1);

protected:
	virtual bool sdPowerUp(void) APPLE_KEXT_OVERRIDE;
	virtual bool sdSetClock(uint32_t hz) APPLE_KEXT_OVERRIDE;
	virtual void sdSetBusWidth4(void) APPLE_KEXT_OVERRIDE;
	virtual bool sdHighSpeedTiming(bool on) APPLE_KEXT_OVERRIDE;
	virtual IOReturn sdCommand(uint32_t index, uint32_t arg, SDResponse type, uint32_t resp[4]) APPLE_KEXT_OVERRIDE;
	virtual IOReturn sdReadSmall(uint32_t index, uint32_t arg, uint8_t *data, uint32_t bytes) APPLE_KEXT_OVERRIDE;
	virtual IOReturn sdTransfer(uint32_t index, uint32_t arg, uint32_t count, bool write) APPLE_KEXT_OVERRIDE;

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
};

#define super PIESDCard
OSDefineMetaClassAndStructors(RKDWMMC, PIESDCard);

static uint64_t
microsecondsSince(uint64_t since)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - since, &ns);
	return ns / 1000;
}

#pragma mark Controller

/*
 * waitClear is true once none of mask's bits is set at off, and false if
 * one still is at usec. waitInt returns RINTSTS once one of mask's bits or
 * an error bit is set in it, or as it reads at usec. Both pause between
 * reads, and read once more at their deadline.
 */
bool
RKDWMMC::waitClear(uint32_t off, uint32_t mask, uint32_t usec)
{
	uint64_t deadline;
	clock_interval_to_deadline(usec, kMicrosecondScale, &deadline);
	while (rd(off) & mask) {
		if (mach_absolute_time() >= deadline) {
			return !(rd(off) & mask);
		}
		IODelay(10);
	}
	return true;
}

uint32_t
RKDWMMC::waitInt(uint32_t mask, uint32_t usec)
{
	uint64_t deadline;
	clock_interval_to_deadline(usec, kMicrosecondScale, &deadline);
	for (;;) {
		uint32_t st = rd(DW_RINTSTS);
		if (st & (mask | INT_CMD_ERR | INT_DATA_ERR)) {
			return st;
		}
		if (mach_absolute_time() >= deadline) {
			return rd(DW_RINTSTS);
		}
		IODelay(10);
	}
}

bool
RKDWMMC::resetFifo(void)
{
	wr(DW_CTRL, rd(DW_CTRL) | CTRL_FIFO_RESET);
	return waitClear(DW_CTRL, CTRL_FIFO_RESET, 100000);
}

/* The microseconds left before deadline, rounded up, so that a wait for them ends at it; 0 once it has passed. */
static uint32_t
microsecondsLeft(uint64_t deadline)
{
	uint64_t now = mach_absolute_time(), ns = 0;
	if (now < deadline) {
		absolutetime_to_nanoseconds(deadline - now, &ns);
	}
	uint64_t us = (ns + 999) / 1000;
	return us < UINT32_MAX ? (uint32_t)us : UINT32_MAX;
}

/*
 * Up to 500 ms for the card to release DAT0, as TRM part 2 asks before
 * SDCLK stops. If it does not, the caller goes on, and what it does is
 * logged.
 */
void
RKDWMMC::waitDat0(const char *what)
{
	if (!waitClear(DW_STATUS, STATUS_DATA_BUSY, DAT0_WAIT_MS * 1000)) {
		IOLog("RKDWMMC: %s: DAT0 still reads busy after 500 ms; %s anyway\n", slot, what);
	}
}

/*
 * The clock registers taken by the controller by deadline, 100 ms from now
 * if it is 0. The update is sent once no command is pending, and again
 * 10 us after the controller refuses it with a hardware lock error; none
 * is sent once the deadline has passed. The wait for it ends at the
 * deadline, with a last look there. The update carries
 * wait_prvdata_complete, as TRM part 2 section 6.6.4.3 sends it, unless
 * the controller has been reset and has sent no data command since: such a
 * reset leaves no transfer to wait for, and the update carries
 * update_clock_registers_only alone.
 */
bool
RKDWMMC::updateClock(uint64_t deadline)
{
	if (deadline == 0) {
		clock_interval_to_deadline(100, kMillisecondScale, &deadline);
	}
	for (;;) {
		bool late = mach_absolute_time() >= deadline;
		if (rd(DW_CMD) & CMD_START) {
			if (late) {
				return false;
			}
			IODelay(10);
			continue;
		}
		if (late) {
			return false;
		}
		wr(DW_RINTSTS, INT_HLE);
		wr(DW_CMD, CMD_START | CMD_UPD_CLK | (noDataSinceReset ? 0 : CMD_PRV_DAT_WAIT | CMD_USE_HOLD));
		if (!waitClear(DW_CMD, CMD_START, microsecondsLeft(deadline))) {
			return false;
		}
		if (!(rd(DW_RINTSTS) & INT_HLE)) {
			return true;
		}
		IODelay(10);
	}
}

/*
 * CLKENA set to enable, and with a divider other than ~0, CLKSRC to 0 and
 * CLKDIV to divider, then taken by the controller, all within 100 ms. The
 * writes come once no command is pending, and must read back with no
 * hardware lock error, or they are made again; then updateClock must take
 * them.
 */
bool
RKDWMMC::clockRegisters(uint32_t enable, uint32_t divider)
{
	uint64_t deadline;
	clock_interval_to_deadline(100, kMillisecondScale, &deadline);
	for (;;) {
		bool late = mach_absolute_time() >= deadline;
		if (!(rd(DW_CMD) & CMD_START)) {
			wr(DW_RINTSTS, INT_HLE);
			wr(DW_CLKENA, enable);
			if (divider != ~0U) {
				wr(DW_CLKSRC, 0);
				wr(DW_CLKDIV, divider);
			}
			if (!(rd(DW_RINTSTS) & INT_HLE) && rd(DW_CLKENA) == enable &&
			    (divider == ~0U || (rd(DW_CLKSRC) == 0 && rd(DW_CLKDIV) == divider))) {
				return updateClock(deadline);
			}
		}
		if (late) {
			return false;
		}
		IODelay(10);
	}
}

/*
 * The card clock set to hz, or the next rate below it, as TRM part 2
 * section 6.6.4.3 sets it: gateClock waits for DAT0 and stops SDCLK, then
 * the divider is set and SDCLK started again, each step confirmed. After
 * a failed step, gateClock stops SDCLK again.
 */
bool
RKDWMMC::sdSetClock(uint32_t hz)
{
	uint32_t div = busHz <= hz ? 0 : (busHz + 2 * hz - 1) / (2 * hz);
	if (div > 0xff) {
		div = 0xff;
	}
	if (!gateClock() || !clockRegisters(0, div) || !clockRegisters(1, ~0U)) {
		IOLog("RKDWMMC: %s: the clock did not change to %u kHz\n", slot, hz / 1000);
		gateClock();
		return false;
	}
	sdClockHz = div ? busHz / (2 * div) : busHz;
	return true;
}

/* usec, or the time left before until if that is less; until 0 sets no limit. */
static uint32_t
capWait(uint32_t usec, uint64_t until)
{
	if (until == 0) {
		return usec;
	}
	uint32_t left = microsecondsLeft(until);
	return left < usec ? left : usec;
}

/* A command and its response; each of its waits ends by until, if it is not 0. */
IOReturn
RKDWMMC::command(uint32_t index, uint32_t arg, uint32_t flags, uint32_t data, uint32_t resp[4], uint64_t until)
{
	if (!waitClear(DW_CMD, CMD_START, capWait(100000, until))) {
		return kIOReturnTimeout;
	}
	if (index != 0 && index != 12 && index != 13 &&
	    !waitClear(DW_STATUS, STATUS_DATA_BUSY, capWait(500000, until))) {
		return kIOReturnBusy;
	}
	wr(DW_RINTSTS, ~0U);
	wr(DW_CMDARG, arg);
	uint32_t cmd = CMD_START | CMD_USE_HOLD | (index & 0x3f) | flags | data;
	if (index == 0) {
		cmd |= CMD_INIT;
	}
	if (data & CMD_DATA) {
		cmd |= CMD_PRV_DAT_WAIT;
	}
	wr(DW_CMD, cmd);
	if (!waitClear(DW_CMD, CMD_START, capWait(100000, until))) {
		return kIOReturnTimeout;
	}
	uint32_t st = waitInt(INT_CMD_DONE, capWait(500000, until));
	if (st & INT_RTO) {
		return kIOReturnTimeout;
	}
	if ((st & (INT_RE | INT_HLE)) || ((flags & CMD_RESP_CRC) && (st & INT_RCRC)) || !(st & INT_CMD_DONE)) {
		return kIOReturnIOError;
	}
	wr(DW_RINTSTS, INT_CMD_DONE);
	if (resp != NULL) {
		for (int i = 0; i < 4; i++) {
			resp[i] = (flags & CMD_RESP_LONG) || i == 0 ? rd(DW_RESP0 + 4 * i) : 0;
		}
	}
	return kIOReturnSuccess;
}

IOReturn
RKDWMMC::sdCommand(uint32_t index, uint32_t arg, SDResponse type, uint32_t resp[4])
{
	static const uint32_t flags[] = {
		0,
		CMD_RESP_EXP | CMD_RESP_CRC,
		CMD_RESP_EXP | CMD_RESP_CRC,
		CMD_RESP_EXP | CMD_RESP_LONG | CMD_RESP_CRC,
		CMD_RESP_EXP,
	};
	return command(index, arg, flags[type], 0, resp);
}

/* The controller reset, FIFO and DMA too, and its clock set again. */
bool
RKDWMMC::resetController(void)
{
	wr(DW_CTRL, rd(DW_CTRL) | CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET);
	if (!waitClear(DW_CTRL, CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET, 100000)) {
		return false;
	}
	noDataSinceReset = true;
	return updateClock();
}

/* The card's state from CMD13's status: 4 transfer, 5 data, 6 receive, 7 programming; ~0 if no answer. */
uint32_t
RKDWMMC::cardState(uint64_t until)
{
	uint32_t resp[4];
	if (command(13, sdRCA << 16, CMD_RESP_EXP | CMD_RESP_CRC, 0, resp, until) != kIOReturnSuccess) {
		return ~0U;
	}
	return (resp[0] >> 9) & 0xf;
}

/*
 * The card's state from CMD13 every millisecond while it is in its
 * programming state, for up to 10 s: each CMD13's waits end by then, and
 * none is sent after it. Once the card has answered in its programming
 * state, a CMD13 that fails counts as that state.
 */
uint32_t
RKDWMMC::pollCardState(void)
{
	uint64_t deadline;
	clock_interval_to_deadline(PROGRAMMING_MS, kMillisecondScale, &deadline);
	bool programming = false;
	for (;;) {
		uint32_t state = cardState(deadline);
		if (state == ~0U && programming) {
			state = 7;
		}
		if (state != 7 || mach_absolute_time() >= deadline) {
			return state;
		}
		programming = true;
		IODelay(1000);
		if (mach_absolute_time() >= deadline) {
			return state;
		}
	}
}

/*
 * After a failed transfer: the FIFO emptied and the card stopped, until
 * CMD13 finds it in its transfer state and DAT0 is released. CMD13 polls
 * a card in its programming state for up to 10 s; one still there is set
 * up again, with no more stops. A card that stays busy in its transfer
 * state gets a controller reset on the next try. A card already out of
 * its data state does not answer the stop. A controller that does not
 * take a command is reset. A card that does not come back is set up
 * again; if that fails too, no more I/O goes to it. Each step is logged
 * with the controller's state, but a recovery that ends at its first try
 * logs only that it is done, and a FIFO or command reset that failed.
 */
void
RKDWMMC::abortTransfer(void)
{
	uint64_t since = mach_absolute_time();
	heldSteps = 0;
	bool host = resetFifo() && !(rd(DW_CMD) & CMD_START);
	recoveryStep(host ? "recovery: the FIFO reset" : "recovery: the FIFO or the command did not reset", since,
	    host);
	for (int tries = 0; tries < 3; tries++) {
		if (!host && !(host = resetController())) {
			recoveryStep("recovery: the controller did not reset", since, false);
			break;
		}
		IOReturn stop = command(12, 0, CMD_RESP_EXP | CMD_RESP_CRC | CMD_STOP_ABORT, 0, NULL);
		if (rd(DW_CMD) & CMD_START) {
			recoveryStep("recovery: CMD12 was not taken", since, false);
			host = false;
			continue;
		}
		uint32_t state = pollCardState();
		char what[64];
		snprintf(what, sizeof(what), "recovery: CMD12 0x%x, card state %d", stop, (int)state);
		recoveryStep(what, since, tries == 0 && state == 4);
		if (state == 7) {
			break;
		}
		if (state == 4 && !waitClear(DW_STATUS, STATUS_DATA_BUSY, 100000)) {
			recoveryStep("recovery: the card stayed busy", since, false);
			host = false;
			continue;
		}
		if (state == 4 && (resetFifo() || resetController())) {
			wr(DW_RINTSTS, ~0U);
			heldSteps = 0;
			logState("recovery: done", since);
			return;
		}
		host = !(rd(DW_CMD) & CMD_START);
	}
	recoveryStep("the card did not come back after a failed transfer; setting it up again", since, false);
	wr(DW_RINTSTS, ~0U);
	if (!sdInitCard()) {
		IOLog("RKDWMMC: %s: the card did not answer after %llu us; no more I/O to it\n", slot,
		    microsecondsSince(since));
		dead = true;
	} else {
		IOLog("RKDWMMC: %s: the card was set up again after %llu us\n", slot, microsecondsSince(since));
	}
}

IOReturn
RKDWMMC::readData(uint32_t index, uint32_t arg, void *buf, uint32_t blksz, uint32_t nblk)
{
	uint64_t since = mach_absolute_time();
	char what[96];
	if (!resetFifo()) {
		snprintf(what, sizeof(what), "CMD%u read: the FIFO did not reset", index);
		logState(what, since);
		abortTransfer();
		return kIOReturnIOError;
	}
	wr(DW_BLKSIZ, blksz);
	wr(DW_BYTCNT, blksz * nblk);
	noDataSinceReset = false;
	IOReturn ret = command(index, arg, CMD_RESP_EXP | CMD_RESP_CRC, CMD_DATA | (nblk > 1 ? CMD_AUTO_STOP : 0), NULL);
	if (ret != kIOReturnSuccess) {
		snprintf(what, sizeof(what), "CMD%u read of %u bytes: the command failed, 0x%x", index, blksz * nblk,
		    ret);
		logState(what, since);
		abortTransfer();
		return ret;
	}
	uint32_t *p = (uint32_t *)buf, words = blksz * nblk / 4, got = 0, st = 0;
	uint64_t deadline;
	clock_interval_to_deadline(PIO_STALL_MS, kMillisecondScale, &deadline);
	while (got < words) {
		bool late = mach_absolute_time() >= deadline;
		st = rd(DW_RINTSTS);
		if (st & (INT_DATA_ERR | INT_CMD_ERR)) {
			break;
		}
		uint32_t n = (rd(DW_STATUS) >> 17) & 0x1fff;
		if (n == 0 && late) {
			break;
		}
		if (n == 0) {
			IODelay(5);
			continue;
		}
		for (; n > 0 && got < words; n--) {
			p[got++] = rd(DW_DATA);
		}
		wr(DW_RINTSTS, INT_RXDR);
		clock_interval_to_deadline(PIO_STALL_MS, kMillisecondScale, &deadline);
	}
	if (got == words) {
		st = waitInt(INT_DTO, 1000000);
		if ((st & INT_DTO) && nblk > 1 && !(st & INT_ACD)) {
			st = waitInt(INT_ACD, 1000000);
		}
	}
	if (got < words || !(st & INT_DTO) || (nblk > 1 && !(st & INT_ACD)) || (st & (INT_DATA_ERR | INT_CMD_ERR))) {
		snprintf(what, sizeof(what), "CMD%u read of %u bytes failed at %u, status 0x%08x", index, blksz * nblk,
		    got * 4, st);
		logState(what, since);
		abortTransfer();
		return kIOReturnIOError;
	}
	wr(DW_RINTSTS, ~0U);
	return kIOReturnSuccess;
}

IOReturn
RKDWMMC::writeData(uint32_t index, uint32_t arg, const void *buf, uint32_t nblk)
{
	uint64_t since = mach_absolute_time();
	char what[96];
	if (!resetFifo()) {
		snprintf(what, sizeof(what), "CMD%u write: the FIFO did not reset", index);
		logState(what, since);
		abortTransfer();
		return kIOReturnIOError;
	}
	wr(DW_BLKSIZ, kSDBlockSize);
	wr(DW_BYTCNT, kSDBlockSize * nblk);
	noDataSinceReset = false;
	IOReturn ret = command(index, arg, CMD_RESP_EXP | CMD_RESP_CRC, CMD_DATA | CMD_WRITE | (nblk > 1 ? CMD_AUTO_STOP : 0),
	    NULL);
	if (ret != kIOReturnSuccess) {
		snprintf(what, sizeof(what), "CMD%u write of %u blocks: the command failed, 0x%x", index, nblk, ret);
		logState(what, since);
		abortTransfer();
		return ret;
	}
	const uint32_t *p = (const uint32_t *)buf;
	uint32_t words = kSDBlockSize * nblk / 4, sent = 0, st = 0;
	uint64_t deadline;
	clock_interval_to_deadline(PIO_STALL_MS, kMillisecondScale, &deadline);
	while (sent < words) {
		bool late = mach_absolute_time() >= deadline;
		st = rd(DW_RINTSTS);
		if (st & (INT_DATA_ERR | INT_CMD_ERR)) {
			break;
		}
		uint32_t used = (rd(DW_STATUS) >> 17) & 0x1fff;
		uint32_t space = used < fifoDepth ? fifoDepth - used : 0;
		if (space == 0 && late) {
			break;
		}
		if (space == 0) {
			IODelay(5);
			continue;
		}
		for (; space > 0 && sent < words; space--) {
			wr(DW_DATA, p[sent++]);
		}
		wr(DW_RINTSTS, INT_TXDR);
		clock_interval_to_deadline(PIO_STALL_MS, kMillisecondScale, &deadline);
	}
	if (sent == words) {
		st = waitInt(INT_DTO, 2000000);
		if ((st & INT_DTO) && nblk > 1 && !(st & INT_ACD)) {
			st = waitInt(INT_ACD, 1000000);
		}
	}
	if (sent < words || !(st & INT_DTO) || (nblk > 1 && !(st & INT_ACD)) || (st & (INT_DATA_ERR | INT_CMD_ERR))) {
		snprintf(what, sizeof(what), "CMD%u write of %u blocks failed at %u bytes, status 0x%08x", index, nblk,
		    sent * 4, st);
		logState(what, since);
		abortTransfer();
		return kIOReturnIOError;
	}
	wr(DW_RINTSTS, ~0U);
	if (!waitClear(DW_STATUS, STATUS_DATA_BUSY, WRITE_BUSY_MS * 1000)) {
		snprintf(what, sizeof(what), "CMD%u write of %u blocks: the card stayed busy", index, nblk);
		logState(what, since);
		abortTransfer();
		return kIOReturnTimeout;
	}
	return kIOReturnSuccess;
}

IOReturn
RKDWMMC::sdTransfer(uint32_t index, uint32_t arg, uint32_t count, bool write)
{
	if (dead) {
		return kIOReturnNotResponding;
	}
	watchBegin(index, arg, count, write);
	IOReturn ret = write ? writeData(index, arg, sdBounce, count) :
	    readData(index, arg, sdBounce, kSDBlockSize, count);
	watchEnd();
	return ret;
}

IOReturn
RKDWMMC::sdReadSmall(uint32_t index, uint32_t arg, uint8_t *data, uint32_t bytes)
{
	watchBegin(index, arg, 1, false);
	IOReturn ret = readData(index, arg, data, bytes, 1);
	watchEnd();
	return ret;
}

#pragma mark Logging

/* The controller's state, with what happened and the time since since, as a log line in line. */
void
RKDWMMC::stateLine(char *line, size_t size, const char *what, uint64_t since)
{
	snprintf(line, size, "RKDWMMC: %s: %s after %llu us; RINTSTS 0x%08x, STATUS 0x%08x, BYTCNT %u, TCBCNT %u, "
	    "TBBCNT %u\n", slot, what, microsecondsSince(since), rd(DW_RINTSTS), rd(DW_STATUS), rd(DW_BYTCNT),
	    rd(DW_TCBCNT), rd(DW_TBBCNT));
}

/* The controller's state logged, with what happened and the time since since. */
void
RKDWMMC::logState(const char *what, uint64_t since)
{
	char line[LINE_BYTES];
	stateLine(line, sizeof(line), what, since);
	IOLog("%s", line);
}

/*
 * A step of abortTransfer logged as logState logs it, or held if hold is
 * set. Held steps are logged before the next step that is not, and a
 * recovery that ends drops them.
 */
void
RKDWMMC::recoveryStep(const char *what, uint64_t since, bool hold)
{
	if (hold) {
		if (heldSteps < HELD_STEPS) {
			stateLine(heldStep[heldSteps++], LINE_BYTES, what, since);
		}
		return;
	}
	for (unsigned i = 0; i < heldSteps; i++) {
		IOLog("%s", heldStep[i]);
	}
	heldSteps = 0;
	logState(what, since);
}

/*
 * While a transfer is out, a thread call logs the controller's registers
 * 5 s after it began and every 5 s after that. The call takes only
 * watchLock, which the transfer's thread holds only to arm and cancel it.
 */
void
RKDWMMC::watchBegin(uint32_t index, uint32_t arg, uint32_t count, bool write)
{
	if (watchCall == NULL) {
		return;
	}
	uint64_t deadline;
	IOLockLock(watchLock);
	watchStart = mach_absolute_time();
	watchIndex = index;
	watchArg = arg;
	watchCount = count;
	watchWrite = write;
	watchOut = true;
	clock_interval_to_deadline(WATCH_MS, kMillisecondScale, &deadline);
	thread_call_enter_delayed(watchCall, deadline);
	IOLockUnlock(watchLock);
}

void
RKDWMMC::watchEnd(void)
{
	if (watchCall == NULL) {
		return;
	}
	IOLockLock(watchLock);
	watchOut = false;
	thread_call_cancel(watchCall);
	IOLockUnlock(watchLock);
}

void
RKDWMMC::watchFire(thread_call_param_t p0, thread_call_param_t p1)
{
	(void)p1;
	RKDWMMC *me = (RKDWMMC *)p0;
	IOLockLock(me->watchLock);
	if (me->watchOut) {
		IOLog("RKDWMMC: %s: CMD%u %s of %u blocks at 0x%x out for %llu ms; CTRL 0x%x, CMD 0x%08x, CMDARG 0x%x, "
		    "RINTSTS 0x%08x, STATUS 0x%08x, BYTCNT %u, TCBCNT %u, TBBCNT %u, CLKENA 0x%x, CLKDIV 0x%x\n",
		    me->slot, me->watchIndex, me->watchWrite ? "write" : "read", me->watchCount,
		    me->watchArg, microsecondsSince(me->watchStart) / 1000, me->rd(DW_CTRL), me->rd(DW_CMD),
		    me->rd(DW_CMDARG), me->rd(DW_RINTSTS), me->rd(DW_STATUS), me->rd(DW_BYTCNT), me->rd(DW_TCBCNT),
		    me->rd(DW_TBBCNT), me->rd(DW_CLKENA), me->rd(DW_CLKDIV));
		uint64_t deadline;
		clock_interval_to_deadline(WATCH_MS, kMillisecondScale, &deadline);
		thread_call_enter_delayed(me->watchCall, deadline);
	}
	IOLockUnlock(me->watchLock);
}

#pragma mark Card

bool
RKDWMMC::sdPowerUp(void)
{
	wr(DW_CTRL, CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET);
	if (!waitClear(DW_CTRL, CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET, 100000)) {
		IOLog("RKDWMMC: controller reset timed out\n");
		return false;
	}
	noDataSinceReset = true;
	wr(DW_PWREN, 1);
	IODelay(10000);
	wr(DW_INTMASK, 0);
	wr(DW_RINTSTS, ~0U);
	wr(DW_TMOUT, ~0U);
	wr(DW_BMOD, 0);
	wr(DW_FIFOTH, ((fifoDepth / 2 - 1) << 16) | (fifoDepth / 2));
	wr(DW_CTYPE, 0);
	return true;
}

void
RKDWMMC::sdSetBusWidth4(void)
{
	wr(DW_CTYPE, 1);
}

bool
RKDWMMC::sdHighSpeedTiming(bool on)
{
	if (on && busHz < kSDClockHighSpeedHz) {
		return false;
	}
	if (cruMap != NULL) {
		*cru(CRU_SDMMC_CON0 + 8 * cruIndex) = (CRU_PHASE_MASK << 16) | (on ? CRU_PHASE_90 : drivePhase);
	}
	return true;
}

#pragma mark Rockchip

uint64_t
RKDWMMC::parentRate(unsigned sel)
{
	uint32_t c75 = *cru(RK_CRU_CLKSEL(75)), c81 = *cru(RK_CRU_CLKSEL(81)), c82 = *cru(RK_CRU_CLKSEL(82));
	uint64_t gpll = RKPLLRate(cruMap, RK_CRU_GPLL_CON0, RK_CRU_MODE_CON, RK_CRU_GPLL_MODE_SHIFT);
	uint64_t cpll = RKPLLRate(cruMap, RK_CRU_CPLL_CON0, RK_CRU_MODE_CON, RK_CRU_CPLL_MODE_SHIFT);
	switch (sel) {
	case 0: return 24000000;
	case 1: return gpll / ((c75 & 0x1f) + 1);
	case 2: return gpll / (((c75 >> 8) & 0x1f) + 1);
	case 3: return cpll / ((c82 & 0x1f) + 1);
	case 4: return cpll / ((c81 & 0x1f) + 1);
	case 5: return 24000000 / (((c82 >> 8) & 0x3f) + 1);
	default: return 0;
	}
}

/*
 * The rate the clock unit gives the controller, which halves it. Below
 * what high speed needs, the card clock moves to the 100 MHz parent while
 * the controller is still idle.
 */
uint32_t
RKDWMMC::ciuClock(IOService *provider)
{
	uint64_t pa = regsMap->getPhysicalAddress();
	uint32_t shift = pa == 0xfe2b0000ULL ? 8 : pa == 0xfe2c0000ULL ? 12 : 0;
	cruIndex = shift == 8 ? 0 : 1;
	cruMap = shift ? RKMapCompatible("rockchip,rk3568-cru") : NULL;
	if (cruMap == NULL) {
		IOLog("RKDWMMC: %s: unknown clock; assuming 24 MHz\n", provider->getName());
		return 24000000;
	}
	drivePhase = *cru(CRU_SDMMC_CON0 + 8 * cruIndex) & CRU_PHASE_MASK;
	unsigned sel = (*cru(RK_CRU_CLKSEL(30)) >> shift) & 7;
	uint64_t hz = parentRate(sel);
	uint32_t want = CRU_PARENT_CPLL_100M;
	uint64_t wantHz = parentRate(want);
	IOLog("RKDWMMC: %s: clock parent %u, %llu Hz; parent %u is %llu Hz; phases 0x%x 0x%x\n", provider->getName(),
	    sel, hz, want, wantHz, *cru(CRU_SDMMC_CON0 + 8 * cruIndex), *cru(CRU_SDMMC_CON0 + 8 * cruIndex + 4));
	if (want != sel && wantHz >= 24000000 && wantHz <= 300000000 && hz / 2 < kSDClockHighSpeedHz) {
		*cru(RK_CRU_CLKSEL(30)) = (7U << (shift + 16)) | (want << shift);
		IODelay(100);
		sel = (*cru(RK_CRU_CLKSEL(30)) >> shift) & 7;
		hz = parentRate(sel);
		IOLog("RKDWMMC: %s: clock parent now %u, %llu Hz\n", provider->getName(), sel, hz);
	}
	return hz != 0 ? (uint32_t)hz : 24000000;
}

/*
 * The card's IO supply, from vqmmc-supply, and the IO domains it feeds,
 * from the io-domains node's vccioN-supply. VCCIO2 is left out: a pin can
 * select its voltage. Without a domain, or a driver for the supply within
 * 3 s, the supply and its domains are left as they are.
 */
bool
RKDWMMC::findSupply(IOService *provider)
{
	OSData *vq = OSDynamicCast(OSData, provider->getProperty("vqmmc-supply"));
	uint32_t ph = vq != NULL && vq->getLength() == 4 ?
	    OSSwapBigToHostInt32(*(const uint32_t *)vq->getBytesNoCopy()) : 0;
	OSArray *nodes = ph != 0 ? RPiDTFindCompatible("rockchip,rk3568-pmu-io-voltage-domain") : NULL;
	IORegistryEntry *domains = nodes != NULL ? OSDynamicCast(IORegistryEntry, nodes->getObject(0)) : NULL;
	for (unsigned n = 1; domains != NULL && n <= 7; n++) {
		char key[16];
		snprintf(key, sizeof(key), "vccio%u-supply", n);
		OSData *d = OSDynamicCast(OSData, domains->getProperty(key));
		if (n != 2 && d != NULL && d->getLength() == 4 &&
		    OSSwapBigToHostInt32(*(const uint32_t *)d->getBytesNoCopy()) == ph) {
			ioDomains |= 1U << n;
		}
	}
	OSSafeReleaseNULL(nodes);

	const char *why = NULL;
	if (ph == 0) {
		why = "no vqmmc-supply";
	} else if (ioDomains == 0 || pmuGrfMap == NULL) {
		why = "no IO domain for its IO supply";
	} else {
		supplyNode = RKFindPHandle(IORegistryEntry::getRegistryRoot(), ph);
		supply = supplyNode != NULL ? RKFindRegulators(supplyNode, 3000) : NULL;
		why = supply == NULL ? "no driver for its IO supply within 3 s" : NULL;
	}
	if (why != NULL) {
		IOLog("RKDWMMC: %s: %s; the IO supply is left as it is\n", slot, why);
		OSSafeReleaseNULL(supply);
		OSSafeReleaseNULL(supplyNode);
		return false;
	}
	return true;
}

/* The supply's voltage as its driver reads it, asked again every 50 ms for up to 3 s while it refuses for a time. */
IOReturn
RKDWMMC::readSupply(uint32_t *microvolt)
{
	uint64_t deadline;
	clock_interval_to_deadline(3000, kMillisecondScale, &deadline);
	for (;;) {
		IOReturn ret = supply->getVoltage(supplyNode, microvolt);
		if (ret != kIOReturnBusy || mach_absolute_time() >= deadline) {
			return ret;
		}
		IOSleep(50);
	}
}

/* The IO domains in 3.3 V mode for a supply above 1.8 V, in 1.8 V mode otherwise. */
void
RKDWMMC::setDomains(uint32_t microvolt)
{
	bool high = microvolt > IO_1V8;
	RKWriteMasked(pmuGrfMap, PMUGRF_IO_VSEL0, ioDomains, high ? 0 : ioDomains);
	RKWriteMasked(pmuGrfMap, PMUGRF_IO_VSEL1, ioDomains, high ? ioDomains : 0);
}

/* True if every IO domain is in the mode setDomains gives a supply at microvolt. */
bool
RKDWMMC::domainsAt(uint32_t microvolt)
{
	bool high = microvolt > IO_1V8;
	return (RKRead(pmuGrfMap, PMUGRF_IO_VSEL0) & ioDomains) == (high ? 0 : ioDomains) &&
	       (RKRead(pmuGrfMap, PMUGRF_IO_VSEL1) & ioDomains) == (high ? ioDomains : 0);
}

/*
 * SDCLK stopped, and the stop taken by the controller, as clockRegisters
 * sets CLKENA to 0, once waitDat0 returns; a controller reset and a second
 * try follow if the first fails. False if the stop is not confirmed, and
 * then nothing is to change.
 */
bool
RKDWMMC::gateClock(void)
{
	waitDat0("stopping SDCLK");
	for (int tries = 0; tries < 2; tries++) {
		if (tries > 0) {
			wr(DW_CTRL, CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET);
			if (waitClear(DW_CTRL, CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET, 100000)) {
				noDataSinceReset = true;
			}
		}
		if (clockRegisters(0, ~0U)) {
			return true;
		}
	}
	IOLog("RKDWMMC: %s: SDCLK did not stop\n", slot);
	return false;
}

/*
 * The domains to 3.3 V mode, then the supply to 3.3 V, with SDCLK stopped
 * through both and the 10 ms after. True once the supply reads back
 * 3.3 V; otherwise the domains stay in 3.3 V mode, at or above the supply.
 */
bool
RKDWMMC::raiseSupply(void)
{
	if (!gateClock()) {
		return false;
	}
	setDomains(IO_3V3);
	IOReturn ret = supply->setVoltage(supplyNode, IO_3V3);
	IOSleep(10);
	uint32_t now = 0;
	IOReturn back = supply->getVoltage(supplyNode, &now);
	if (ret == kIOReturnSuccess && back == kIOReturnSuccess && now == IO_3V3 && domainsAt(IO_3V3)) {
		IOLog("RKDWMMC: %s: IO at 3300 mV; IO_VSEL0 0x%x, IO_VSEL1 0x%x\n", slot,
		    RKRead(pmuGrfMap, PMUGRF_IO_VSEL0) & 0xffff, RKRead(pmuGrfMap, PMUGRF_IO_VSEL1) & 0xffff);
		return true;
	}
	IOLog("RKDWMMC: %s: the IO supply did not change to 3300 mV (0x%x), and reads %u mV (0x%x); its domains stay "
	    "in 3.3 V mode\n", slot, ret, back == kIOReturnSuccess ? now / 1000 : 0, back);
	return false;
}

/* The card's SD_SPEC from ACMD51's SCR, under sdLock; -1 if the SCR does not read or has another structure. */
int
RKDWMMC::readSpec(void)
{
	uint8_t scr[8];
	uint32_t resp[4];
	IOReturn ret = sdCommand(55, sdRCA << 16, kSDRespR1, resp);
	if (ret == kIOReturnSuccess) {
		ret = sdReadSmall(51, 0, scr, sizeof(scr));
	}
	if (ret != kIOReturnSuccess || (scr[0] >> 4) != 0) {
		IOLog("RKDWMMC: %s: no SCR (0x%x)\n", slot, ret);
		return -1;
	}
	return scr[0] & 0xf;
}

/*
 * 1 if the card is in 1.8 V signaling, 0 if it is in 3.3 V signaling, -1
 * if that is not known, with sdSpec set from the SCR. A card of SD_SPEC 0,
 * SD 1.0 or 1.01, has no switch function and no 1.8 V signaling, and is
 * sent no CMD6. Any later card offers SDR50, SDR104 or DDR50 in its switch
 * status only in 1.8 V signaling.
 */
int
RKDWMMC::cardSignal(void)
{
	uint8_t status[64];
	IOReturn ret = kIOReturnSuccess;
	IOLockLock(sdLock);
	sdSpec = readSpec();
	if (sdSpec > 0) {
		ret = sdReadSmall(6, 0x00FFFFFF, status, sizeof(status));
	}
	IOLockUnlock(sdLock);
	if (sdSpec < 0) {
		return -1;
	}
	if (sdSpec == 0) {
		IOLog("RKDWMMC: %s: SD_SPEC 0, with no switch function\n", slot);
		return 0;
	}
	if (ret != kIOReturnSuccess) {
		IOLog("RKDWMMC: %s: SD_SPEC %d; no switch status (0x%x)\n", slot, sdSpec, ret);
		return -1;
	}
	IOLog("RKDWMMC: %s: SD_SPEC %d, bus speed modes 0x%02x\n", slot, sdSpec, status[13]);
	return (status[13] & 0x1c) != 0 ? 1 : 0;
}

/* The card set up from CMD0 at the IO voltage as it is now; an earlier setup, and its SCR, no longer count. */
bool
RKDWMMC::setUp(void)
{
	dead = false;
	sdSpec = -1;
	return sdStart();
}

/*
 * High speed set up as PIESDCard sets it, with CMD6, for a card whose SCR
 * gives SD_SPEC 1 or later. A card of SD_SPEC 0 has no switch function,
 * and stays at its default speed, as does one whose SCR does not read.
 * The SCR is read here if setUpCard has not read it.
 */
void
RKDWMMC::setUpHighSpeed(void)
{
	uint8_t *reference = (uint8_t *)IOMallocData(kSDCheckBlocks * kSDBlockSize);
	IOLockLock(sdLock);
	if (sdSpec < 0) {
		sdSpec = readSpec();
	}
	if (sdSpec <= 0) {
		IOLog("RKDWMMC: %s: %s; no high speed\n", slot, sdSpec == 0 ? "SD_SPEC 0" : "no SCR");
	} else if (reference != NULL && busHz >= kSDClockHighSpeedHz && sdReadReference(reference)) {
		sdSetUpHighSpeed(reference, "dwmmc_hs");
	}
	IOLockUnlock(sdLock);
	if (reference != NULL) {
		IOFreeData(reference, kSDCheckBlocks * kSDBlockSize);
	}
}

/*
 * The card set up at an IO voltage it can be used at; false if there is
 * none. The supply is never lowered toward 1.8 V. One the boot left at
 * 1.8 V has had that voltage since before the kernel started: the domains
 * go to 1.8 V mode, and the card is set up there without CMD11. It is kept
 * if cardSignal finds it in 1.8 V signaling. Otherwise, and from any other
 * supply, the domains go to 3.3 V mode and the supply to 3.3 V. A card is
 * then kept if cardSignal finds it in 3.3 V signaling. One in 1.8 V
 * signaling is left unused, as is one whose signaling is not known. SDCLK
 * is stopped through every change and the 10 ms after it, and the card is
 * set up again from CMD0. A card is used only once a setup has followed
 * the last change, at a supply that read back and domains in its mode. If
 * the supply cannot be read or set, the domains go to 3.3 V mode and the
 * card is not used. SDCLK stays stopped for a card that is not used.
 * Without a supply to control, the card is set up as the boot left it.
 */
bool
RKDWMMC::setUpCard(void)
{
	uint32_t uv = 0;
	if (!findSupply(getProvider())) {
		return sdStart();
	}
	IOReturn ret = readSupply(&uv);
	if (ret != kIOReturnSuccess) {
		IOLog("RKDWMMC: %s: the IO supply cannot be read (0x%x); the card is not used\n", slot, ret);
		if (gateClock()) {
			setDomains(IO_3V3);
		}
		return false;
	}
	IOLog("RKDWMMC: %s: IO supply %s at %u mV, IO domains 0x%x; IO_VSEL0 0x%x, IO_VSEL1 0x%x; STATUS 0x%08x\n",
	    slot, supplyNode->getName(gIODTPlane), uv / 1000, ioDomains, RKRead(pmuGrfMap, PMUGRF_IO_VSEL0) & 0xffff,
	    RKRead(pmuGrfMap, PMUGRF_IO_VSEL1) & 0xffff, rd(DW_STATUS));
	if (uv == IO_1V8) {
		if (!domainsAt(IO_1V8)) {
			if (!gateClock()) {
				return false;
			}
			setDomains(IO_1V8);
			IOSleep(10);
		}
		bool present = setUp();
		int signal = present ? cardSignal() : -1;
		if (signal == 1 && domainsAt(IO_1V8)) {
			return true;
		}
		IOLog("RKDWMMC: %s: %s at 1.8 V; raising the IO supply to 3.3 V\n", slot,
		    !present ? "no card is set up" : signal == 0 ? "the card is not in 1.8 V signaling" :
		    signal < 0 ? "the card's signaling is not known" : "the IO domains are not in 1.8 V mode");
	}
	if (!raiseSupply()) {
		return false;
	}
	if (!setUp()) {
		IOLog("RKDWMMC: %s: no card is set up at 3.3 V\n", slot);
		gateClock();
		return false;
	}
	int signal = cardSignal();
	if (signal == 1) {
		IOLog("RKDWMMC: %s: the card is in 1.8 V signaling, and the IO supply is not lowered to 1.8 V; "
		    "switch the board off and on to use it\n", slot);
	} else if (signal < 0) {
		IOLog("RKDWMMC: %s: the card's signaling voltage is not known; it is not used\n", slot);
	}
	if (signal != 0) {
		gateClock();
		return false;
	}
	return true;
}

#pragma mark IOService

bool
RKDWMMC::start(IOService *provider)
{
	if (provider->getProperty("non-removable") != NULL) {
		return false;
	}
	if (!super::start(provider)) {
		return false;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		IOLog("RKDWMMC: cannot map the registers\n");
		return false;
	}
	regs = (volatile uint32_t *)regsMap->getVirtualAddress();
	snprintf(slot, sizeof(slot), "%s@%llx", provider->getName(), regsMap->getPhysicalAddress());
	pmuGrfMap = RKMapCompatible("rockchip,rk3568-pmugrf");
	if (pmuGrfMap != NULL) {
		IOLog("RKDWMMC: %s: IO_VSEL0 0x%x, IO_VSEL1 0x%x\n", slot, RKRead(pmuGrfMap, PMUGRF_IO_VSEL0) & 0xffff,
		    RKRead(pmuGrfMap, PMUGRF_IO_VSEL1) & 0xffff);
	}
	uint64_t base = 0;
	if (PE_parse_boot_argn("dwmmc_skip", &base, sizeof(base)) && base == regsMap->getPhysicalAddress()) {
		IOLog("RKDWMMC: %s: skipped\n", provider->getName());
		return false;
	}
	sdReadOnly = PE_parse_boot_argn("dwmmc_ro", &base, sizeof(base)) && base == regsMap->getPhysicalAddress();
	bounceMem = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut, kSDBounceBytes, 4);
	sdBounce = bounceMem != NULL ? (uint32_t *)bounceMem->getBytesNoCopy() : NULL;
	watchLock = IOLockAlloc();
	watchCall = watchLock != NULL ? thread_call_allocate_with_options(&RKDWMMC::watchFire, this,
	    THREAD_CALL_PRIORITY_KERNEL, THREAD_CALL_OPTIONS_ONCE) : NULL;
	fifoDepth = 256;
	OSData *fd = OSDynamicCast(OSData, provider->getProperty("fifo-depth"));
	if (fd != NULL && fd->getLength() == 4) {
		fifoDepth = OSSwapBigToHostInt32(*(const uint32_t *)fd->getBytesNoCopy());
	}

	RKApplyPinctrl(provider, "RKDWMMC");
	busHz = ciuClock(provider) / 2;
	IOLog("RKDWMMC: %s at 0x%llx, version 0x%x, FIFO %u words, controller clock %u MHz%s\n",
	    provider->getName(), regsMap->getPhysicalAddress(), rd(DW_VERID), fifoDepth, busHz / 1000000,
	    sdReadOnly ? ", read-only" : "");

	sdSpec = -1;
	bool present = setUpCard();
	OSSafeReleaseNULL(supply);
	OSSafeReleaseNULL(supplyNode);
	if (!present) {
		return false;
	}
	setUpHighSpeed();
	IOLog("RKDWMMC: %u kHz, %s bus\n", sdClockHz / 1000, rd(DW_CTYPE) ? "4-bit" : "1-bit");
	sdPublish(kIOPropertyInternalKey);
	return true;
}

void
RKDWMMC::free(void)
{
	if (watchCall != NULL) {
		thread_call_cancel_wait(watchCall);
		thread_call_free(watchCall);
		watchCall = NULL;
	}
	if (watchLock != NULL) {
		IOLockFree(watchLock);
		watchLock = NULL;
	}
	OSSafeReleaseNULL(bounceMem);
	OSSafeReleaseNULL(cruMap);
	OSSafeReleaseNULL(pmuGrfMap);
	OSSafeReleaseNULL(supply);
	OSSafeReleaseNULL(supplyNode);
	OSSafeReleaseNULL(regsMap);
	super::free();
}
