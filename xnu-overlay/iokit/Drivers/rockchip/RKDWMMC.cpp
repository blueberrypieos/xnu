/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/OSByteOrder.h>

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
	DW_FIFOTH = 0x4c, DW_VERID = 0x6c, DW_BMOD = 0x80, DW_DATA = 0x200,
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

	uint32_t rd(uint32_t off) { return regs[off / 4]; }
	void wr(uint32_t off, uint32_t v) { regs[off / 4] = v; }
	volatile uint32_t *cru(uint32_t off) { return (volatile uint32_t *)(cruMap->getVirtualAddress() + off); }
	uint64_t parentRate(unsigned sel);

	bool waitClear(uint32_t off, uint32_t mask, uint32_t usec);
	uint32_t waitInt(uint32_t mask, uint32_t usec);
	bool resetFifo(void);
	bool updateClock(void);
	IOReturn command(uint32_t index, uint32_t arg, uint32_t flags, uint32_t data, uint32_t resp[4]);
	bool resetController(void);
	uint32_t cardState(void);
	void abortTransfer(void);
	IOReturn readData(uint32_t index, uint32_t arg, void *buf, uint32_t blksz, uint32_t nblk);
	IOReturn writeData(uint32_t index, uint32_t arg, const void *buf, uint32_t nblk);
	uint32_t ciuClock(IOService *provider);

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

#pragma mark Controller

bool
RKDWMMC::waitClear(uint32_t off, uint32_t mask, uint32_t usec)
{
	for (uint32_t t = 0; t < usec; t += 10) {
		if (!(rd(off) & mask)) {
			return true;
		}
		IODelay(10);
	}
	return !(rd(off) & mask);
}

uint32_t
RKDWMMC::waitInt(uint32_t mask, uint32_t usec)
{
	uint32_t st = 0;
	for (uint32_t t = 0; t <= usec; t += 10) {
		st = rd(DW_RINTSTS);
		if (st & (mask | INT_CMD_ERR | INT_DATA_ERR)) {
			return st;
		}
		IODelay(10);
	}
	return st;
}

bool
RKDWMMC::resetFifo(void)
{
	wr(DW_CTRL, rd(DW_CTRL) | CTRL_FIFO_RESET);
	return waitClear(DW_CTRL, CTRL_FIFO_RESET, 100000);
}

bool
RKDWMMC::updateClock(void)
{
	wr(DW_CMD, CMD_START | CMD_UPD_CLK | CMD_USE_HOLD);
	return waitClear(DW_CMD, CMD_START, 100000);
}

bool
RKDWMMC::sdSetClock(uint32_t hz)
{
	uint32_t div = busHz <= hz ? 0 : (busHz + 2 * hz - 1) / (2 * hz);
	if (div > 0xff) {
		div = 0xff;
	}
	wr(DW_CLKENA, 0);
	wr(DW_CLKSRC, 0);
	if (!updateClock()) {
		IOLog("RKDWMMC: the clock update did not finish\n");
		return false;
	}
	wr(DW_CLKDIV, div);
	if (!updateClock()) {
		return false;
	}
	wr(DW_CLKENA, 1);
	if (!updateClock()) {
		return false;
	}
	sdClockHz = div ? busHz / (2 * div) : busHz;
	return true;
}

IOReturn
RKDWMMC::command(uint32_t index, uint32_t arg, uint32_t flags, uint32_t data, uint32_t resp[4])
{
	if (!waitClear(DW_CMD, CMD_START, 100000)) {
		return kIOReturnTimeout;
	}
	if (index != 0 && index != 12 && index != 13 && !waitClear(DW_STATUS, STATUS_DATA_BUSY, 500000)) {
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
	if (!waitClear(DW_CMD, CMD_START, 100000)) {
		return kIOReturnTimeout;
	}
	uint32_t st = waitInt(INT_CMD_DONE, 500000);
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
	return waitClear(DW_CTRL, CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET, 100000) && updateClock();
}

/* The card's state from CMD13's status: 4 transfer, 5 data, 6 receive, 7 programming; ~0 if no answer. */
uint32_t
RKDWMMC::cardState(void)
{
	uint32_t resp[4];
	if (command(13, sdRCA << 16, CMD_RESP_EXP | CMD_RESP_CRC, 0, resp) != kIOReturnSuccess) {
		return ~0U;
	}
	return (resp[0] >> 9) & 0xf;
}

/*
 * After a failed transfer: the FIFO emptied and the card stopped, until
 * CMD13 finds it in its transfer state and DAT0 is released. A card that
 * stays busy there gets a controller reset on the next try. A card
 * already out of its data state does not answer the stop. A controller
 * that does not take a command is reset. A card that does not come back
 * is set up again; if that fails too, no more I/O goes to it.
 */
void
RKDWMMC::abortTransfer(void)
{
	bool host = resetFifo() && !(rd(DW_CMD) & CMD_START);
	for (int tries = 0; tries < 3; tries++) {
		if (!host && !(host = resetController())) {
			break;
		}
		command(12, 0, CMD_RESP_EXP | CMD_RESP_CRC | CMD_STOP_ABORT, 0, NULL);
		if (rd(DW_CMD) & CMD_START) {
			host = false;
			continue;
		}
		uint32_t state = ~0U;
		for (int i = 0; i < 1000; i++) {
			state = cardState();
			if (state != 7) {
				break;
			}
			IODelay(1000);
		}
		if (state == 4 && !waitClear(DW_STATUS, STATUS_DATA_BUSY, 100000)) {
			host = false;
			continue;
		}
		if (state == 4 && (resetFifo() || resetController())) {
			wr(DW_RINTSTS, ~0U);
			return;
		}
		host = !(rd(DW_CMD) & CMD_START);
	}
	IOLog("RKDWMMC: the card did not come back after a failed transfer; setting it up again\n");
	wr(DW_RINTSTS, ~0U);
	if (!sdInitCard()) {
		IOLog("RKDWMMC: the card did not answer; no more I/O to it\n");
		dead = true;
	}
}

IOReturn
RKDWMMC::readData(uint32_t index, uint32_t arg, void *buf, uint32_t blksz, uint32_t nblk)
{
	if (!resetFifo()) {
		abortTransfer();
		return kIOReturnIOError;
	}
	wr(DW_BLKSIZ, blksz);
	wr(DW_BYTCNT, blksz * nblk);
	IOReturn ret = command(index, arg, CMD_RESP_EXP | CMD_RESP_CRC, CMD_DATA | (nblk > 1 ? CMD_AUTO_STOP : 0), NULL);
	if (ret != kIOReturnSuccess) {
		abortTransfer();
		return ret;
	}
	uint32_t *p = (uint32_t *)buf, words = blksz * nblk / 4, got = 0, st = 0;
	for (uint32_t t = 0; got < words && t < 2000000; t += 5) {
		st = rd(DW_RINTSTS);
		if (st & (INT_DATA_ERR | INT_CMD_ERR)) {
			break;
		}
		uint32_t n = (rd(DW_STATUS) >> 17) & 0x1fff;
		if (n == 0) {
			IODelay(5);
			continue;
		}
		for (; n > 0 && got < words; n--) {
			p[got++] = rd(DW_DATA);
		}
		wr(DW_RINTSTS, INT_RXDR);
	}
	if (got == words) {
		st = waitInt(INT_DTO, 1000000);
		if ((st & INT_DTO) && nblk > 1 && !(st & INT_ACD)) {
			st = waitInt(INT_ACD, 1000000);
		}
	}
	if (got < words || !(st & INT_DTO) || (nblk > 1 && !(st & INT_ACD)) || (st & (INT_DATA_ERR | INT_CMD_ERR))) {
		IOLog("RKDWMMC: CMD%u read of %u bytes failed after %u (status 0x%08x)\n", index, blksz * nblk,
		    got * 4, st);
		abortTransfer();
		return kIOReturnIOError;
	}
	wr(DW_RINTSTS, ~0U);
	return kIOReturnSuccess;
}

IOReturn
RKDWMMC::writeData(uint32_t index, uint32_t arg, const void *buf, uint32_t nblk)
{
	if (!resetFifo()) {
		abortTransfer();
		return kIOReturnIOError;
	}
	wr(DW_BLKSIZ, kSDBlockSize);
	wr(DW_BYTCNT, kSDBlockSize * nblk);
	IOReturn ret = command(index, arg, CMD_RESP_EXP | CMD_RESP_CRC, CMD_DATA | CMD_WRITE | (nblk > 1 ? CMD_AUTO_STOP : 0),
	    NULL);
	if (ret != kIOReturnSuccess) {
		abortTransfer();
		return ret;
	}
	const uint32_t *p = (const uint32_t *)buf;
	uint32_t words = kSDBlockSize * nblk / 4, sent = 0, st = 0;
	for (uint32_t t = 0; sent < words && t < 2000000; t += 5) {
		st = rd(DW_RINTSTS);
		if (st & (INT_DATA_ERR | INT_CMD_ERR)) {
			break;
		}
		uint32_t used = (rd(DW_STATUS) >> 17) & 0x1fff;
		uint32_t space = used < fifoDepth ? fifoDepth - used : 0;
		if (space == 0) {
			IODelay(5);
			continue;
		}
		for (; space > 0 && sent < words; space--) {
			wr(DW_DATA, p[sent++]);
		}
		wr(DW_RINTSTS, INT_TXDR);
	}
	if (sent == words) {
		st = waitInt(INT_DTO, 2000000);
		if ((st & INT_DTO) && nblk > 1 && !(st & INT_ACD)) {
			st = waitInt(INT_ACD, 1000000);
		}
	}
	if (sent < words || !(st & INT_DTO) || (nblk > 1 && !(st & INT_ACD)) || (st & (INT_DATA_ERR | INT_CMD_ERR))) {
		IOLog("RKDWMMC: CMD%u write of %u blocks failed after %u bytes (status 0x%08x)\n", index, nblk,
		    sent * 4, st);
		abortTransfer();
		return kIOReturnIOError;
	}
	wr(DW_RINTSTS, ~0U);
	if (!waitClear(DW_STATUS, STATUS_DATA_BUSY, 1000000)) {
		IOLog("RKDWMMC: CMD%u write of %u blocks: the card stayed busy\n", index, nblk);
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
	if (write) {
		return writeData(index, arg, sdBounce, count);
	}
	return readData(index, arg, sdBounce, kSDBlockSize, count);
}

IOReturn
RKDWMMC::sdReadSmall(uint32_t index, uint32_t arg, uint8_t *data, uint32_t bytes)
{
	return readData(index, arg, data, bytes, 1);
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
	uint64_t base = 0;
	if (PE_parse_boot_argn("dwmmc_skip", &base, sizeof(base)) && base == regsMap->getPhysicalAddress()) {
		IOLog("RKDWMMC: %s: skipped\n", provider->getName());
		return false;
	}
	sdReadOnly = PE_parse_boot_argn("dwmmc_ro", &base, sizeof(base)) && base == regsMap->getPhysicalAddress();
	bounceMem = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut, kSDBounceBytes, 4);
	sdBounce = bounceMem != NULL ? (uint32_t *)bounceMem->getBytesNoCopy() : NULL;
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

	if (!sdStart()) {
		return false;
	}
	uint8_t *reference = (uint8_t *)IOMallocData(kSDCheckBlocks * kSDBlockSize);
	IOLockLock(sdLock);
	if (reference != NULL && busHz >= kSDClockHighSpeedHz && sdReadReference(reference)) {
		sdSetUpHighSpeed(reference, "dwmmc_hs");
	}
	IOLockUnlock(sdLock);
	if (reference != NULL) {
		IOFreeData(reference, kSDCheckBlocks * kSDBlockSize);
	}
	IOLog("RKDWMMC: %u kHz, %s bus\n", sdClockHz / 1000, rd(DW_CTYPE) ? "4-bit" : "1-bit");
	sdPublish(kIOPropertyInternalKey);
	return true;
}

void
RKDWMMC::free(void)
{
	OSSafeReleaseNULL(bounceMem);
	OSSafeReleaseNULL(cruMap);
	OSSafeReleaseNULL(regsMap);
	super::free();
}
