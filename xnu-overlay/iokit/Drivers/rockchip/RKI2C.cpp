/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <libkern/OSByteOrder.h>
#include <mach/mach_time.h>

#include "../rpi/RPiPlatform.h"
#include "RKI2C.h"
#include "RKSoC.h"

/* RK3568 I2C controller registers. */
enum {
	I2C_CON = 0x000, I2C_CLKDIV = 0x004, I2C_MRXADDR = 0x008, I2C_MRXRADDR = 0x00c,
	I2C_MTXCNT = 0x010, I2C_MRXCNT = 0x014, I2C_IEN = 0x018, I2C_IPD = 0x01c,
	I2C_TXDATA0 = 0x100, I2C_RXDATA0 = 0x200,
};

enum {
	CON_EN = 1U << 0, CON_MODE_TX = 0U << 1, CON_MODE_REGISTER_RX = 1U << 1,
	CON_START = 1U << 3, CON_STOP = 1U << 4, CON_LAST_NAK = 1U << 5, CON_ACT2NAK = 1U << 6,
	IPD_MBTF = 1U << 2, IPD_MBRF = 1U << 3, IPD_START = 1U << 4, IPD_STOP = 1U << 5, IPD_NAK = 1U << 6,
	IPD_ALL = 0xff,
	ADDR_LOW_VALID = 1U << 24,
	FIFO_BYTES = 32,
};

/*
 * I2C0, in the PMU domain: its clock gates in PMUCRU GATE_CON01, its resets
 * in SOFTRST_CON00, its parent's mux in CLKSEL_CON02 and its divider in
 * CLKSEL_CON03.
 */
enum {
	I2C0_BASE = 0xfdd40000,
	PMUGATE1_I2C0 = 0x3, PMUSOFTRST0_I2C0 = 0x18,
	PMUCLKSEL2_PDPMU_GPLL = 1U << 15, PMUCLKSEL3_I2C0_DIV = 0x7f,
};

#define BUS_HZ_MAX      100000
#define BUS_HZ_MIN      10000
#define START_US        2000

#define super IOService
OSDefineMetaClassAndStructors(RKI2C, IOService);

uint32_t
RKI2C::rd(uint32_t off)
{
	return RKRead(regsMap, off);
}

void
RKI2C::wr(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(regsMap->getVirtualAddress() + off) = v;
}

/*
 * The pending bits among bits, read without a pause until one is pending
 * or deadline has passed, then once more: 0 if none is pending then.
 */
uint32_t
RKI2C::waitFor(uint32_t bits, uint64_t deadline)
{
	for (;;) {
		uint32_t ipd = rd(I2C_IPD) & bits;
		if (ipd != 0) {
			return ipd;
		}
		if (mach_absolute_time() >= deadline) {
			return rd(I2C_IPD) & bits;
		}
	}
}

/*
 * One transaction: a start, the address and reg, then data written or,
 * after a restart, read, then a stop. A NAK ends it early, and the stop
 * is still waited for before the controller is turned off. The start has
 * 2 ms, the data four times the bus time of the whole transaction from
 * when it is sent, and the stop as long as the two, each to a deadline. A
 * one-byte transaction at 100 kHz ends within 6.88 ms, failed or not, but
 * for time its thread is off the CPU.
 */
IOReturn
RKI2C::transfer(uint8_t address, uint8_t reg, uint8_t *data, uint32_t length, bool read)
{
	if (address > 0x7f || length == 0 || length > (read ? FIFO_BYTES : FIFO_BYTES - 2)) {
		return kIOReturnBadArgument;
	}
	IOReturn ret = kIOReturnSuccess;
	uint32_t con = CON_EN | CON_ACT2NAK | (read ? CON_MODE_REGISTER_RX | CON_LAST_NAK : CON_MODE_TX);
	uint32_t done = read ? IPD_MBRF : IPD_MBTF, ipd;
	uint32_t dataUs = (uint32_t)((length + 3) * 9 * 4 * 1000000ULL / busHz);
	uint64_t deadline;

	IOLockLock(lock);
	wr(I2C_IPD, IPD_ALL);
	wr(I2C_CON, con | CON_START);
	clock_interval_to_deadline(START_US, kMicrosecondScale, &deadline);
	ipd = waitFor(IPD_START, deadline);
	wr(I2C_CON, con);
	if (!(ipd & IPD_START)) {
		ret = kIOReturnTimeout;
		goto stop;
	}
	if (read) {
		wr(I2C_MRXADDR, ADDR_LOW_VALID | (uint32_t)address << 1);
		wr(I2C_MRXRADDR, ADDR_LOW_VALID | reg);
		wr(I2C_MRXCNT, length);
	} else {
		uint8_t out[FIFO_BYTES] = { (uint8_t)(address << 1), reg };
		bcopy(data, out + 2, length);
		for (uint32_t i = 0; i < length + 2; i += 4) {
			wr(I2C_TXDATA0 + i, OSReadLittleInt32(out, i));
		}
		wr(I2C_MTXCNT, length + 2);
	}
	clock_interval_to_deadline(dataUs, kMicrosecondScale, &deadline);
	ipd = waitFor(done | IPD_NAK, deadline);
	if (ipd & IPD_NAK) {
		ret = kIOReturnNoDevice;
	} else if (!(ipd & done)) {
		ret = kIOReturnTimeout;
	} else if (read) {
		for (uint32_t i = 0; i < length; i++) {
			data[i] = (uint8_t)(rd(I2C_RXDATA0 + (i & ~3U)) >> ((i & 3) * 8));
		}
	}
stop:
	wr(I2C_IPD, IPD_STOP | IPD_NAK);
	wr(I2C_CON, con | CON_STOP);
	clock_interval_to_deadline(START_US + dataUs, kMicrosecondScale, &deadline);
	if (!(waitFor(IPD_STOP, deadline) & IPD_STOP) && ret == kIOReturnSuccess) {
		ret = kIOReturnTimeout;
	}
	wr(I2C_CON, 0);
	wr(I2C_IPD, IPD_ALL);
	IOLockUnlock(lock);
	return ret;
}

IOReturn
RKI2C::readRegisters(uint8_t address, uint8_t reg, uint8_t *data, uint32_t length)
{
	return transfer(address, reg, data, length, true);
}

IOReturn
RKI2C::writeRegisters(uint8_t address, uint8_t reg, const uint8_t *data, uint32_t length)
{
	return transfer(address, reg, const_cast<uint8_t *>(data), length, false);
}

/* I2C0's clock: the PMU's PPLL or the GPLL, divided. */
uint64_t
RKI2C::clockRate(IOMemoryMap *pmucru)
{
	uint64_t parent;
	if (RKRead(pmucru, RK_PMUCRU_CLKSEL(2)) & PMUCLKSEL2_PDPMU_GPLL) {
		IOMemoryMap *cru = RKMapCompatible("rockchip,rk3568-cru");
		parent = cru != NULL ? RKPLLRate(cru, RK_CRU_GPLL_CON0, RK_CRU_MODE_CON, RK_CRU_GPLL_MODE_SHIFT) : 0;
		OSSafeReleaseNULL(cru);
	} else {
		parent = RKPLLRate(pmucru, RK_PMUCRU_PPLL_CON0, RK_PMUCRU_MODE_CON, RK_PMUCRU_PPLL_MODE_SHIFT);
	}
	return parent / ((RKRead(pmucru, RK_PMUCRU_CLKSEL(3)) & PMUCLKSEL3_I2C0_DIV) + 1);
}

/*
 * SCL low and high each last 8 * (CLKDIVL + 1) and 8 * (CLKDIVH + 1)
 * controller clocks. The bus runs at clock-frequency, at most 100 kHz. A
 * clock-frequency below 10 kHz is refused: a start's setup and hold take
 * three SCL high times, 150 us at 10 kHz, and must fit its 2 ms wait.
 */
bool
RKI2C::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		IOLog("RKI2C: %s: cannot map the registers\n", provider->getName());
		return false;
	}
	if (regsMap->getPhysicalAddress() != I2C0_BASE) {
		IOLog("RKI2C: %s at 0x%llx: unknown clock; not started\n", provider->getName(),
		    regsMap->getPhysicalAddress());
		return false;
	}
	IOMemoryMap *pmucru = RKMapCompatible("rockchip,rk3568-pmucru");
	if (pmucru == NULL) {
		IOLog("RKI2C: %s: no PMU clock unit\n", provider->getName());
		return false;
	}
	RKWriteMasked(pmucru, RK_PMUCRU_GATE(1), PMUGATE1_I2C0, 0);
	RKWriteMasked(pmucru, RK_PMUCRU_SOFTRST(0), PMUSOFTRST0_I2C0, 0);
	uint64_t hz = clockRate(pmucru);
	OSSafeReleaseNULL(pmucru);
	RKApplyPinctrl(provider, "RKI2C");

	busHz = BUS_HZ_MAX;
	OSData *freq = OSDynamicCast(OSData, provider->getProperty("clock-frequency"));
	if (freq != NULL && freq->getLength() == 4) {
		uint32_t want = OSSwapBigToHostInt32(*(const uint32_t *)freq->getBytesNoCopy());
		if (want != 0 && want < BUS_HZ_MIN) {
			IOLog("RKI2C: %s: clock-frequency %u Hz is below %u Hz; not started\n", provider->getName(), want,
			    BUS_HZ_MIN);
			return false;
		}
		if (want != 0 && want < busHz) {
			busHz = want;
		}
	}
	uint64_t total = (hz + 8ULL * busHz - 1) / (8ULL * busHz);
	if (total < 4 || total > 0x10000) {
		IOLog("RKI2C: %s: clock %llu Hz cannot give %u Hz\n", provider->getName(), hz, busHz);
		return false;
	}
	uint32_t low = (uint32_t)(total + 1) / 2, high = (uint32_t)total - low;
	lock = IOLockAlloc();
	if (lock == NULL) {
		return false;
	}
	wr(I2C_CON, 0);
	wr(I2C_IEN, 0);
	wr(I2C_IPD, IPD_ALL);
	wr(I2C_CLKDIV, (high - 1) << 16 | (low - 1));
	IOLog("RKI2C: %s at 0x%llx, clock %llu Hz, bus %llu Hz\n", provider->getName(),
	    regsMap->getPhysicalAddress(), hz, hz / (8 * total));

	registerService();
	RPiPublishDTChildren(this, provider);
	return true;
}

void
RKI2C::free(void)
{
	if (lock != NULL) {
		IOLockFree(lock);
		lock = NULL;
	}
	OSSafeReleaseNULL(regsMap);
	super::free();
}
