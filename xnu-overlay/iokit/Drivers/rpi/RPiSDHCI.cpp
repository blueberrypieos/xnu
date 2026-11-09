/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOWorkLoop.h>
#include <libkern/OSByteOrder.h>
#include <sys/errno.h>
#include <sys/sysctl.h>

#include "RPiPlatform.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#include <IOKit/storage/IOBlockStorageDevice.h>
#include <IOKit/storage/IOStorageDeviceCharacteristics.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>
#pragma clang diagnostic pop

enum {
	SDHCI_ARGUMENT2       = 0x00,
	SDHCI_BLOCK           = 0x04,
	SDHCI_ARGUMENT        = 0x08,
	SDHCI_CMD_XFER        = 0x0C,
	SDHCI_RESPONSE        = 0x10,
	SDHCI_BUFFER          = 0x20,
	SDHCI_PRESENT_STATE   = 0x24,
	SDHCI_HOST_CONTROL    = 0x28,
	SDHCI_CLOCK_CONTROL   = 0x2C,
	SDHCI_INT_STATUS      = 0x30,
	SDHCI_INT_ENABLE      = 0x34,
	SDHCI_SIGNAL_ENABLE   = 0x38,
	SDHCI_HOST_CONTROL2   = 0x3C,
	SDHCI_CAPABILITIES    = 0x40,
	SDHCI_ADMA_ADDRESS    = 0x58,
	SDHCI_VERSION         = 0xFC,
	SDHCI_SDMA_ADDRESS    = SDHCI_ARGUMENT2,
};

enum {
	PRESENT_CMD_INHIBIT = 1U << 0,
	PRESENT_DAT_INHIBIT = 1U << 1,
	PRESENT_CARD        = 1U << 16,

	HC_DATA_4BIT        = 1U << 1,
	HC_HIGH_SPEED       = 1U << 2,
	HC_DMA_MASK         = 3U << 3,
	HC_DMA_SDMA         = 0U << 3,
	HC_DMA_ADMA2        = 2U << 3,
	POWER_ON_3V3        = (0x7U << 1 | 1U) << 8,
	HC2_UHS_MASK        = 7U << 16,
	HC2_UHS_SDR25       = 1U << 16,
	HC2_SIGNAL_1V8      = 1U << 19,

	CAP_ADMA2           = 1U << 19,
	CAP_HIGH_SPEED      = 1U << 21,
	CAP_SDMA            = 1U << 22,

	CLOCK_INTERNAL_EN   = 1U << 0,
	CLOCK_STABLE        = 1U << 1,
	CLOCK_CARD_EN       = 1U << 2,
	TIMEOUT_MAX         = 0xEU << 16,
	RESET_ALL           = 1U << 24,
	RESET_CMD           = 1U << 25,
	RESET_DATA          = 1U << 26,

	INT_CMD_DONE        = 1U << 0,
	INT_XFER_DONE       = 1U << 1,
	INT_DMA             = 1U << 3,
	INT_WRITE_READY     = 1U << 4,
	INT_READ_READY      = 1U << 5,
	INT_ERROR           = 1U << 15,
	INT_ERRORS          = 0xFFFFU << 16,
	INT_ADMA_ERROR      = 1U << 25,

	BLOCK_SDMA_512K     = 7U << 12,

	ADMA_VALID          = 1U << 0,
	ADMA_END            = 1U << 1,
	ADMA_TRAN           = 2U << 4,

	XFER_DMA_EN         = 1U << 0,
	XFER_BLOCK_COUNT_EN = 1U << 1,
	XFER_AUTO_CMD12     = 1U << 2,
	XFER_READ           = 1U << 4,
	XFER_MULTI          = 1U << 5,

	CMD_RESP_NONE       = 0,
	CMD_RESP_136        = 1,
	CMD_RESP_48         = 2,
	CMD_RESP_48_BUSY    = 3,
	CMD_CRC_CHECK       = 1U << 3,
	CMD_INDEX_CHECK     = 1U << 4,
	CMD_DATA            = 1U << 5,
};

enum {
	R_NONE = CMD_RESP_NONE,
	R1     = CMD_RESP_48 | CMD_CRC_CHECK | CMD_INDEX_CHECK,
	R1B    = CMD_RESP_48_BUSY | CMD_CRC_CHECK | CMD_INDEX_CHECK,
	R2     = CMD_RESP_136 | CMD_CRC_CHECK,
	R3     = CMD_RESP_48,
	R6     = R1,
	R7     = R1,
};

#define BLOCK_SIZE      512
#define MAX_BLOCKS      128
#define BOUNCE_BYTES    (MAX_BLOCKS * BLOCK_SIZE)
#define ADMA_CHUNK      32768
#define CHECK_BLOCKS    8
#define CLOCK_INIT_HZ   400000
#define CLOCK_RUN_HZ    25000000
#define CLOCK_HS_HZ     50000000
#define BASE_CLOCK_HZ   100000000
#define POLL_USEC       50

class RPiSDHCI;
static RPiSDHCI *gSDHCI;

class RPiSDHCI : public IOBlockStorageDevice
{
	OSDeclareDefaultStructors(RPiSDHCI);

	IOMemoryMap *regsMap;
	volatile uint32_t *regs;
	IOLock *lock;
	uint32_t *bounce;
	IOBufferMemoryDescriptor *bounceMem;
	IOBufferMemoryDescriptor *descMem;
	uint32_t bounceBus, descBus;
	enum { DMA_NONE, DMA_SDMA, DMA_ADMA2 } dma;
	IOWorkLoop *workLoop;
	IOFilterInterruptEventSource *intSource;
	IOLock *waitLock;
	bool irqFired;
	bool useIRQ;
	uint32_t caps;
	uint32_t baseClock;
	uint32_t clockHz;
	bool slowWrites;
	bool present;
	bool highCapacity;
	uint32_t rca;
	uint64_t blocks;
	char vendor[16];
	char product[16];
	char revision[8];

	uint32_t rd(uint32_t off) { return regs[off / 4]; }
	void
	wr(uint32_t off, uint32_t v)
	{
		regs[off / 4] = v;
		if (slowWrites && clockHz <= CLOCK_INIT_HZ) {
			IODelay(10);
		}
	}

	bool waitFor(uint32_t off, uint32_t mask, uint32_t want, uint32_t usec);
	uint32_t waitStatus(uint32_t mask, uint32_t usec);
	void failed(uint32_t status, uint32_t lines);
	void resetLines(uint32_t which);
	bool setClock(uint32_t hz);
	IOReturn command(uint32_t index, uint32_t arg, uint32_t flags, uint32_t xfer, uint32_t resp[4]);
	IOReturn appCommand(uint32_t index, uint32_t arg, uint32_t flags, uint32_t resp[4]);
	IOReturn transfer(uint64_t block, uint32_t count, bool write);
	IOReturn transferPIO(uint64_t block, uint32_t count, bool write);
	IOReturn transferDMA(uint64_t block, uint32_t count, bool write);
	IOReturn readSmall(uint32_t index, uint32_t arg, uint8_t *data, uint32_t bytes);
	bool initCard(void);
	bool sameAs(const uint8_t *reference, const char *what);
	void setUpHighSpeed(const uint8_t *reference);
	void setUpDMA(IOService *provider, const uint8_t *reference, uint32_t mode);
	void setUpInterrupt(IOService *provider);
	bool interruptFilter(IOFilterInterruptEventSource *src);
	void interruptOccurred(IOInterruptEventSource *src, int count);

public:
	int setDMAMode(uint32_t mode);
	uint32_t dmaMode(void) const { return dma == DMA_ADMA2 ? 1 : dma == DMA_SDMA ? 2 : 0; }

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	virtual IOReturn doEjectMedia(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn doFormatMedia(UInt64 byteCapacity) APPLE_KEXT_OVERRIDE;
	virtual UInt32 doGetFormatCapacities(UInt64 *capacities, UInt32 capacitiesMaxCount) const APPLE_KEXT_OVERRIDE;
	virtual char *getVendorString(void) APPLE_KEXT_OVERRIDE;
	virtual char *getProductString(void) APPLE_KEXT_OVERRIDE;
	virtual char *getRevisionString(void) APPLE_KEXT_OVERRIDE;
	virtual char *getAdditionalDeviceInfoString(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportBlockSize(UInt64 *blockSize) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportEjectability(bool *isEjectable) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportMaxValidBlock(UInt64 *maxBlock) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportMediaState(bool *mediaPresent, bool *changedState) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportRemovability(bool *isRemovable) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportWriteProtection(bool *isWriteProtected) APPLE_KEXT_OVERRIDE;
	virtual IOReturn doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 nblks,
	    IOStorageAttributes *attributes, IOStorageCompletion *completion) APPLE_KEXT_OVERRIDE;
	virtual IOReturn doSynchronize(UInt64 block, UInt64 nblks, IOStorageSynchronizeOptions options) APPLE_KEXT_OVERRIDE;
};

#define super IOBlockStorageDevice
OSDefineMetaClassAndStructors(RPiSDHCI, IOBlockStorageDevice);

#pragma mark Controller

bool
RPiSDHCI::waitFor(uint32_t off, uint32_t mask, uint32_t want, uint32_t usec)
{
	for (uint32_t t = 0; t < usec; t += 10) {
		if ((rd(off) & mask) == want) {
			return true;
		}
		IODelay(10);
	}
	return (rd(off) & mask) == want;
}

uint32_t
RPiSDHCI::waitStatus(uint32_t mask, uint32_t usec)
{
	for (uint32_t t = 0;; t += 10) {
		uint32_t st = rd(SDHCI_INT_STATUS);
		if (st & (mask | INT_ERROR)) {
			return st;
		}
		if (t >= usec) {
			return 0;
		}
		if (useIRQ && t >= POLL_USEC) {
			break;
		}
		IODelay(10);
	}

	uint64_t deadline;
	clock_interval_to_deadline(usec, kMicrosecondScale, &deadline);
	uint32_t st;
	IOLockLock(waitLock);
	for (;;) {
		st = rd(SDHCI_INT_STATUS);
		if (st & (mask | INT_ERROR)) {
			break;
		}
		irqFired = false;
		regs[SDHCI_SIGNAL_ENABLE / 4] = mask | INT_ERRORS;
		if (IOLockSleepDeadline(waitLock, &irqFired, deadline, THREAD_UNINT) == THREAD_TIMED_OUT) {
			st = rd(SDHCI_INT_STATUS);
			if (st & (mask | INT_ERROR)) {
				IOLog("RPiSDHCI: no interrupt from the controller; polling\n");
				useIRQ = false;
			} else {
				st = 0;
			}
			break;
		}
	}
	regs[SDHCI_SIGNAL_ENABLE / 4] = 0;
	IOLockUnlock(waitLock);
	return st;
}

bool
RPiSDHCI::interruptFilter(IOFilterInterruptEventSource *src)
{
	(void)src;
	uint32_t enabled = regs[SDHCI_SIGNAL_ENABLE / 4];
	if (enabled == 0 || !(regs[SDHCI_INT_STATUS / 4] & enabled)) {
		return false;
	}
	regs[SDHCI_SIGNAL_ENABLE / 4] = 0;
	return true;
}

void
RPiSDHCI::interruptOccurred(IOInterruptEventSource *src, int count)
{
	(void)src;
	(void)count;
	IOLockLock(waitLock);
	irqFired = true;
	IOLockWakeup(waitLock, &irqFired, false);
	IOLockUnlock(waitLock);
}

void
RPiSDHCI::failed(uint32_t status, uint32_t lines)
{
	wr(SDHCI_INT_STATUS, status ? status : 0xFFFFFFFFU);
	resetLines(lines);
}

void
RPiSDHCI::resetLines(uint32_t which)
{
	wr(SDHCI_CLOCK_CONTROL, rd(SDHCI_CLOCK_CONTROL) | which);
	waitFor(SDHCI_CLOCK_CONTROL, which, 0, 100000);
}

bool
RPiSDHCI::setClock(uint32_t hz)
{
	uint32_t n = 0;
	if (baseClock > hz) {
		n = (baseClock + 2 * hz - 1) / (2 * hz);
		if (n > 0x3FF) {
			n = 0x3FF;
		}
	}
	uint32_t cc = rd(SDHCI_CLOCK_CONTROL) & ~0xFFFFU;
	clockHz = 0;
	wr(SDHCI_CLOCK_CONTROL, cc);
	cc |= ((n & 0xFF) << 8) | (((n >> 8) & 0x3) << 6) | CLOCK_INTERNAL_EN;
	wr(SDHCI_CLOCK_CONTROL, cc);
	if (!waitFor(SDHCI_CLOCK_CONTROL, CLOCK_STABLE, CLOCK_STABLE, 20000)) {
		IOLog("RPiSDHCI: clock not stable\n");
		return false;
	}
	wr(SDHCI_CLOCK_CONTROL, cc | CLOCK_CARD_EN);
	clockHz = n ? baseClock / (2 * n) : baseClock;
	IODelay(1000);
	return true;
}

IOReturn
RPiSDHCI::command(uint32_t index, uint32_t arg, uint32_t flags, uint32_t xfer, uint32_t resp[4])
{
	uint32_t inhibit = PRESENT_CMD_INHIBIT;
	if ((flags & CMD_DATA) || (flags & 3) == CMD_RESP_48_BUSY) {
		inhibit |= PRESENT_DAT_INHIBIT;
	}
	if (!waitFor(SDHCI_PRESENT_STATE, inhibit, 0, 100000)) {
		resetLines(RESET_CMD | RESET_DATA);
		return kIOReturnBusy;
	}
	wr(SDHCI_INT_STATUS, 0xFFFFFFFFU);
	wr(SDHCI_ARGUMENT, arg);
	wr(SDHCI_CMD_XFER, ((index << 8 | flags) << 16) | xfer);

	uint32_t st = waitStatus(INT_CMD_DONE, 1000000);
	if (!(st & INT_CMD_DONE) || (st & INT_ERROR)) {
		failed(st, RESET_CMD | RESET_DATA);
		return (st == 0 || (st & (1U << 16))) ? kIOReturnTimeout : kIOReturnIOError;
	}
	wr(SDHCI_INT_STATUS, INT_CMD_DONE);
	if (resp != NULL) {
		for (int i = 0; i < 4; i++) {
			resp[i] = rd(SDHCI_RESPONSE + 4 * i);
		}
	}
	if ((flags & 3) == CMD_RESP_48_BUSY) {
		st = waitStatus(INT_XFER_DONE, 1000000);
		if (!(st & INT_XFER_DONE) || (st & INT_ERROR)) {
			failed(st, RESET_DATA);
			return kIOReturnTimeout;
		}
		wr(SDHCI_INT_STATUS, INT_XFER_DONE);
	}
	return kIOReturnSuccess;
}

IOReturn
RPiSDHCI::appCommand(uint32_t index, uint32_t arg, uint32_t flags, uint32_t resp[4])
{
	uint32_t r[4];
	IOReturn ret = command(55, rca << 16, R1, 0, r);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	return command(index, arg, flags, 0, resp);
}

IOReturn
RPiSDHCI::transfer(uint64_t block, uint32_t count, bool write)
{
	if (dma != DMA_NONE) {
		IOReturn ret = transferDMA(block, count, write);
		if (ret != kIOReturnIOError) {
			return ret;
		}
		IOLog("RPiSDHCI: DMA failed; programmed I/O from now on\n");
		dma = DMA_NONE;
	}
	return transferPIO(block, count, write);
}

IOReturn
RPiSDHCI::transferDMA(uint64_t block, uint32_t count, bool write)
{
	uint32_t bytes = count * BLOCK_SIZE;
	uint32_t arg = (uint32_t)(highCapacity ? block : block * BLOCK_SIZE);
	uint32_t xfer = XFER_DMA_EN | XFER_BLOCK_COUNT_EN | (write ? 0 : XFER_READ);
	uint32_t index;
	if (count > 1) {
		xfer |= XFER_MULTI | XFER_AUTO_CMD12;
		index = write ? 25 : 18;
	} else {
		index = write ? 24 : 17;
	}
	bounceMem->performOperation(write ? kIOMemoryIncoherentIOStore : kIOMemoryIncoherentIOFlush, 0, bytes);
	if (dma == DMA_ADMA2) {
		volatile uint32_t *d = (volatile uint32_t *)descMem->getBytesNoCopy();
		unsigned n = 0;
		for (uint32_t off = 0; off < bytes; off += ADMA_CHUNK, n++) {
			uint32_t len = bytes - off < ADMA_CHUNK ? bytes - off : ADMA_CHUNK;
			d[2 * n] = len << 16 | ADMA_VALID | ADMA_TRAN | (off + len == bytes ? ADMA_END : 0);
			d[2 * n + 1] = bounceBus + off;
		}
		descMem->performOperation(kIOMemoryIncoherentIOStore, 0, n * 8);
		wr(SDHCI_ADMA_ADDRESS, descBus);
		wr(SDHCI_ADMA_ADDRESS + 4, 0);
	} else {
		wr(SDHCI_SDMA_ADDRESS, bounceBus);
	}
	wr(SDHCI_BLOCK, (count << 16) | BLOCK_SDMA_512K | BLOCK_SIZE);

	uint32_t resp[4];
	IOReturn ret = command(index, arg, R1 | CMD_DATA, xfer, resp);
	if (ret != kIOReturnSuccess) {
		return ret == kIOReturnTimeout ? kIOReturnIOError : ret;
	}
	uint32_t st;
	for (;;) {
		st = waitStatus(INT_XFER_DONE | INT_DMA, 2000000);
		if ((st & INT_DMA) && !(st & (INT_XFER_DONE | INT_ERROR))) {
			wr(SDHCI_INT_STATUS, INT_DMA);
			wr(SDHCI_SDMA_ADDRESS, rd(SDHCI_SDMA_ADDRESS));
			continue;
		}
		break;
	}
	if (!(st & INT_XFER_DONE) || (st & INT_ERROR)) {
		failed(st, RESET_CMD | RESET_DATA);
		IOLog("RPiSDHCI: DMA %s of blocks %llu-%llu failed (status 0x%08x%s)\n", write ? "write" : "read",
		    block, block + count - 1, st, (st & INT_ADMA_ERROR) ? ", ADMA error" : "");
		return kIOReturnIOError;
	}
	wr(SDHCI_INT_STATUS, INT_XFER_DONE | INT_DMA);
	if (!write) {
		bounceMem->performOperation(kIOMemoryIncoherentIOFlush, 0, bytes);
	}
	return kIOReturnSuccess;
}

IOReturn
RPiSDHCI::transferPIO(uint64_t block, uint32_t count, bool write)
{
	uint32_t arg = (uint32_t)(highCapacity ? block : block * BLOCK_SIZE);
	uint32_t xfer = XFER_BLOCK_COUNT_EN | (write ? 0 : XFER_READ);
	uint32_t index;
	if (count > 1) {
		xfer |= XFER_MULTI | XFER_AUTO_CMD12;
		index = write ? 25 : 18;
	} else {
		index = write ? 24 : 17;
	}
	wr(SDHCI_BLOCK, (count << 16) | BLOCK_SIZE);

	uint32_t resp[4];
	IOReturn ret = command(index, arg, R1 | CMD_DATA, xfer, resp);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint32_t ready = write ? INT_WRITE_READY : INT_READ_READY;
	uint32_t *p = bounce;
	for (uint32_t b = 0; b < count; b++) {
		uint32_t st = waitStatus(ready, 1000000);
		if (!(st & ready) || (st & INT_ERROR)) {
			failed(st, RESET_CMD | RESET_DATA);
			IOLog("RPiSDHCI: %s of block %llu failed (status 0x%08x)\n",
			    write ? "write" : "read", block + b, st);
			return kIOReturnIOError;
		}
		wr(SDHCI_INT_STATUS, ready);
		for (uint32_t w = 0; w < BLOCK_SIZE / 4; w++, p++) {
			if (write) {
				wr(SDHCI_BUFFER, *p);
			} else {
				*p = rd(SDHCI_BUFFER);
			}
		}
	}
	uint32_t st = waitStatus(INT_XFER_DONE, 1000000);
	if (!(st & INT_XFER_DONE) || (st & INT_ERROR)) {
		failed(st, RESET_CMD | RESET_DATA);
		IOLog("RPiSDHCI: %s of blocks %llu-%llu failed (status 0x%08x)\n",
		    write ? "write" : "read", block, block + count - 1, st);
		return kIOReturnIOError;
	}
	wr(SDHCI_INT_STATUS, INT_XFER_DONE);
	return kIOReturnSuccess;
}

IOReturn
RPiSDHCI::readSmall(uint32_t index, uint32_t arg, uint8_t *data, uint32_t bytes)
{
	wr(SDHCI_BLOCK, (1U << 16) | bytes);
	uint32_t resp[4];
	IOReturn ret = command(index, arg, R1 | CMD_DATA, XFER_BLOCK_COUNT_EN | XFER_READ, resp);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint32_t st = waitStatus(INT_READ_READY, 1000000);
	if (!(st & INT_READ_READY) || (st & INT_ERROR)) {
		failed(st, RESET_CMD | RESET_DATA);
		return kIOReturnIOError;
	}
	wr(SDHCI_INT_STATUS, INT_READ_READY);
	for (uint32_t i = 0; i < bytes; i += 4) {
		uint32_t w = rd(SDHCI_BUFFER);
		bcopy(&w, data + i, 4);
	}
	st = waitStatus(INT_XFER_DONE, 1000000);
	if (!(st & INT_XFER_DONE) || (st & INT_ERROR)) {
		failed(st, RESET_CMD | RESET_DATA);
		return kIOReturnIOError;
	}
	wr(SDHCI_INT_STATUS, INT_XFER_DONE);
	return kIOReturnSuccess;
}

#pragma mark Card

static uint32_t
r2_bits(const uint32_t resp[4], unsigned hi, unsigned lo)
{
	uint32_t v = 0;
	for (unsigned bit = hi; ; bit--) {
		unsigned b = bit - 8;
		v = (v << 1) | ((resp[b / 32] >> (b % 32)) & 1);
		if (bit == lo) {
			break;
		}
	}
	return v;
}

bool
RPiSDHCI::initCard(void)
{
	uint32_t resp[4];

	bool signal1v8 = (rd(SDHCI_HOST_CONTROL2) & HC2_SIGNAL_1V8) != 0;

	wr(SDHCI_CLOCK_CONTROL, RESET_ALL);
	if (!waitFor(SDHCI_CLOCK_CONTROL, RESET_ALL, 0, 100000)) {
		IOLog("RPiSDHCI: controller reset timed out\n");
		return false;
	}
	wr(SDHCI_HOST_CONTROL, POWER_ON_3V3);
	if (signal1v8) {
		wr(SDHCI_HOST_CONTROL2, HC2_SIGNAL_1V8);
		IOLog("RPiSDHCI: the card signals at 1.8 V (left so by the firmware)\n");
	}
	IODelay(10000);
	wr(SDHCI_CLOCK_CONTROL, (rd(SDHCI_CLOCK_CONTROL) & ~(0xFU << 16)) | TIMEOUT_MAX);
	wr(SDHCI_INT_ENABLE, 0xFFFF00FFU);
	wr(SDHCI_SIGNAL_ENABLE, 0);
	if (!setClock(CLOCK_INIT_HZ)) {
		return false;
	}
	if (!(rd(SDHCI_PRESENT_STATE) & PRESENT_CARD)) {
		IOLog("RPiSDHCI: no card reported; trying anyway\n");
	}

	command(0, 0, R_NONE, 0, NULL);
	IODelay(1000);
	bool v2 = command(8, 0x1AA, R7, 0, resp) == kIOReturnSuccess && (resp[0] & 0xFFF) == 0x1AA;
	rca = 0;
	uint32_t ocr = 0;
	for (int i = 0; i < 100; i++) {
		if (appCommand(41, 0x00FF8000U | (v2 ? 0x40000000U : 0), R3, resp) != kIOReturnSuccess) {
			IOLog("RPiSDHCI: no SD card answers\n");
			return false;
		}
		ocr = resp[0];
		if (ocr & 0x80000000U) {
			break;
		}
		IOSleep(10);
	}
	if (!(ocr & 0x80000000U)) {
		IOLog("RPiSDHCI: card did not power up (OCR 0x%08x)\n", ocr);
		return false;
	}
	highCapacity = (ocr & 0x40000000U) != 0;

	if (command(2, 0, R2, 0, resp) != kIOReturnSuccess) {
		return false;
	}
	vendor[0] = (char)r2_bits(resp, 119, 112);
	vendor[1] = (char)r2_bits(resp, 111, 104);
	vendor[2] = '\0';
	for (int i = 0; i < 5; i++) {
		product[i] = (char)r2_bits(resp, 103 - 8 * i, 96 - 8 * i);
	}
	product[5] = '\0';
	uint32_t prv = r2_bits(resp, 63, 56);
	snprintf(revision, sizeof(revision), "%u.%u", prv >> 4, prv & 0xF);

	if (command(3, 0, R6, 0, resp) != kIOReturnSuccess) {
		return false;
	}
	rca = resp[0] >> 16;

	if (command(9, rca << 16, R2, 0, resp) != kIOReturnSuccess) {
		return false;
	}
	if (r2_bits(resp, 127, 126) == 1) {
		blocks = ((uint64_t)r2_bits(resp, 69, 48) + 1) * 1024;
	} else {
		uint32_t c_size = r2_bits(resp, 73, 62);
		uint32_t mult = r2_bits(resp, 49, 47);
		uint32_t bl_len = r2_bits(resp, 83, 80);
		blocks = ((uint64_t)(c_size + 1) << (mult + 2)) << bl_len >> 9;
	}

	if (command(7, rca << 16, R1B, 0, resp) != kIOReturnSuccess) {
		return false;
	}
	if (!setClock(CLOCK_RUN_HZ)) {
		return false;
	}
	if (appCommand(6, 2, R1, resp) == kIOReturnSuccess) {
		wr(SDHCI_HOST_CONTROL, rd(SDHCI_HOST_CONTROL) | HC_DATA_4BIT);
	}
	if (!highCapacity) {
		command(16, BLOCK_SIZE, R1, 0, resp);
	}
	return true;
}

#pragma mark Faster modes

bool
RPiSDHCI::sameAs(const uint8_t *reference, const char *what)
{
	memset(bounce, 0xA5, CHECK_BLOCKS * BLOCK_SIZE);
	IOReturn ret = dma != DMA_NONE ? transferDMA(0, CHECK_BLOCKS, false) : transferPIO(0, CHECK_BLOCKS, false);
	if (ret == kIOReturnSuccess && memcmp(bounce, reference, CHECK_BLOCKS * BLOCK_SIZE) == 0) {
		return true;
	}
	IOLog("RPiSDHCI: %s reads the card %s; left out\n", what, ret == kIOReturnSuccess ? "wrongly" : "with errors");
	return false;
}

void
RPiSDHCI::setUpHighSpeed(const uint8_t *reference)
{
	uint32_t on = 1;
	if ((PE_parse_boot_argn("sdhci_hs", &on, sizeof(on)) && on == 0) || !(caps & CAP_HIGH_SPEED)) {
		return;
	}
	uint8_t status[64];
	if (readSmall(6, 0x00FFFFF1, status, sizeof(status)) != kIOReturnSuccess || !(status[13] & 0x02)) {
		return;
	}
	if (readSmall(6, 0x80FFFFF1, status, sizeof(status)) != kIOReturnSuccess || (status[16] & 0xF) != 1) {
		IOLog("RPiSDHCI: the card did not switch to high speed\n");
		return;
	}
	bool v18 = (rd(SDHCI_HOST_CONTROL2) & HC2_SIGNAL_1V8) != 0;
	if (v18) {
		wr(SDHCI_HOST_CONTROL2, (rd(SDHCI_HOST_CONTROL2) & ~HC2_UHS_MASK) | HC2_UHS_SDR25);
	} else {
		wr(SDHCI_HOST_CONTROL, rd(SDHCI_HOST_CONTROL) | HC_HIGH_SPEED);
	}
	if (setClock(CLOCK_HS_HZ) && sameAs(reference, "high speed")) {
		return;
	}
	setClock(CLOCK_RUN_HZ);
	if (v18) {
		wr(SDHCI_HOST_CONTROL2, rd(SDHCI_HOST_CONTROL2) & ~HC2_UHS_MASK);
	} else {
		wr(SDHCI_HOST_CONTROL, rd(SDHCI_HOST_CONTROL) & ~HC_HIGH_SPEED);
	}
	readSmall(6, 0x80FFFFF0, status, sizeof(status));
}

void
RPiSDHCI::setUpInterrupt(IOService *provider)
{
	uint32_t on = 1;
	if (PE_parse_boot_argn("sdhci_irq", &on, sizeof(on)) && on == 0) {
		return;
	}
	waitLock = IOLockAlloc();
	workLoop = IOWorkLoop::workLoop();
	if (waitLock != NULL && workLoop != NULL) {
		intSource = IOFilterInterruptEventSource::filterInterruptEventSource(this,
		    OSMemberFunctionCast(IOInterruptEventAction, this, &RPiSDHCI::interruptOccurred),
		    OSMemberFunctionCast(IOFilterInterruptEventSource::Filter, this, &RPiSDHCI::interruptFilter),
		    provider, 0);
	}
	if (intSource == NULL || workLoop->addEventSource(intSource) != kIOReturnSuccess) {
		IOLog("RPiSDHCI: no interrupt; polling\n");
		OSSafeReleaseNULL(intSource);
		return;
	}
	regs[SDHCI_SIGNAL_ENABLE / 4] = 0;
	intSource->enable();
	useIRQ = true;
}

void
RPiSDHCI::setUpDMA(IOService *provider, const uint8_t *reference, uint32_t on)
{
	uint64_t pa = bounceMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), ba = 0;
	IOLog("RPiSDHCI: %s: bounce buffer at 0x%llx, bus 0x%llx%s\n", provider->getName(), pa,
	    RPiDMAAddress(provider, pa, &ba) ? ba : 0ULL, RPiDMAAddress(provider, pa, &ba) ? "" : " (outside the DMA window)");
	dma = DMA_NONE;
	if (on == 0 || !(caps & (CAP_ADMA2 | CAP_SDMA))) {
		return;
	}
	if (descMem == NULL) {
		descMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryPhysicallyContiguous, 4096, 0x3FFFF000ULL);
	}
	uint64_t b, d;
	if (descMem == NULL ||
	    !RPiDMAAddress(provider, bounceMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &b) ||
	    !RPiDMAAddress(provider, descMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &d) ||
	    b + BOUNCE_BYTES > (1ULL << 32) || d + 4096 > (1ULL << 32)) {
		IOLog("RPiSDHCI: no DMA: the controller cannot reach its buffers\n");
		return;
	}
	bounceBus = (uint32_t)b;
	descBus = (uint32_t)d;
	uint32_t hc = rd(SDHCI_HOST_CONTROL) & ~HC_DMA_MASK;
	if ((caps & CAP_ADMA2) && on != 2) {
		dma = DMA_ADMA2;
		wr(SDHCI_HOST_CONTROL, hc | HC_DMA_ADMA2);
		if (sameAs(reference, "ADMA2")) {
			return;
		}
	}
	if (caps & CAP_SDMA) {
		dma = DMA_SDMA;
		wr(SDHCI_HOST_CONTROL, hc | HC_DMA_SDMA);
		if (sameAs(reference, "SDMA")) {
			return;
		}
	}
	dma = DMA_NONE;
}

#pragma mark IOService

bool
RPiSDHCI::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		IOLog("RPiSDHCI: cannot map the registers\n");
		return false;
	}
	regs = (volatile uint32_t *)regsMap->getVirtualAddress();
	slowWrites = IODTMatchNubWithKeys(provider, "'brcm,bcm2835-sdhci'");
	lock = IOLockAlloc();
	bounceMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, BOUNCE_BYTES, 0x3FFF0000ULL);
	bounce = bounceMem != NULL ? (uint32_t *)bounceMem->getBytesNoCopy() : NULL;
	if (lock == NULL || bounce == NULL) {
		return false;
	}

	caps = rd(SDHCI_CAPABILITIES);
	baseClock = ((caps >> 8) & 0xFF) * 1000000;
	OSData *freq = OSDynamicCast(OSData, provider->getProperty("clock-frequency"));
	if (freq != NULL && freq->getLength() == sizeof(uint32_t)) {
		baseClock = *(const uint32_t *)freq->getBytesNoCopy();
	}
	uint32_t clk;
	if (PE_parse_boot_argn("sdhci_clk", &clk, sizeof(clk))) {
		baseClock = clk;
	}
	if (baseClock == 0) {
		baseClock = BASE_CLOCK_HZ;
	}
	IOLog("RPiSDHCI: %s, SDHCI version %u, base clock %u MHz\n", provider->getName(),
	    (rd(SDHCI_VERSION) >> 16) & 0xFF, baseClock / 1000000);

	IOLockLock(lock);
	present = initCard();
	IOLockUnlock(lock);
	if (!present) {
		return false;
	}
	IOLog("RPiSDHCI: %s %s rev %s, %llu blocks (%llu MB)%s\n", vendor, product, revision,
	    blocks, blocks / 2048, highCapacity ? ", high capacity" : "");

	uint8_t *reference = (uint8_t *)IOMallocData(CHECK_BLOCKS * BLOCK_SIZE);
	IOLockLock(lock);
	if (reference != NULL && blocks >= CHECK_BLOCKS && transferPIO(0, CHECK_BLOCKS, false) == kIOReturnSuccess) {
		bcopy(bounce, reference, CHECK_BLOCKS * BLOCK_SIZE);
		setUpHighSpeed(reference);
		setUpInterrupt(provider);
		uint32_t mode = 1;
		PE_parse_boot_argn("sdhci_dma", &mode, sizeof(mode));
		setUpDMA(provider, reference, mode);
	}
	IOLockUnlock(lock);
	if (reference != NULL) {
		IOFreeData(reference, CHECK_BLOCKS * BLOCK_SIZE);
	}
	IOLog("RPiSDHCI: %u MHz, %s%s\n", clockHz / 1000000,
	    dma == DMA_ADMA2 ? "ADMA2" : dma == DMA_SDMA ? "SDMA" : "programmed I/O", useIRQ ? ", interrupts" : "");

	OSDictionary *proto = OSDictionary::withCapacity(2);
	if (proto != NULL) {
		OSString *s = OSString::withCString("SD");
		proto->setObject(kIOPropertyPhysicalInterconnectTypeKey, s);
		s->release();
		s = OSString::withCString(kIOPropertyInternalKey);
		proto->setObject(kIOPropertyPhysicalInterconnectLocationKey, s);
		s->release();
		setProperty(kIOPropertyProtocolCharacteristicsKey, proto);
		proto->release();
	}
	setProperty(kIOMaximumBlockCountReadKey, MAX_BLOCKS, 64);
	setProperty(kIOMaximumBlockCountWriteKey, MAX_BLOCKS, 64);

	registerService();
	if (gSDHCI == NULL) {
		gSDHCI = this;
	}
	return true;
}

int
RPiSDHCI::setDMAMode(uint32_t mode)
{
	uint8_t *reference = (uint8_t *)IOMallocData(CHECK_BLOCKS * BLOCK_SIZE);
	if (reference == NULL) {
		return ENOMEM;
	}
	int err = EIO;
	IOLockLock(lock);
	dma = DMA_NONE;
	if (transferPIO(0, CHECK_BLOCKS, false) == kIOReturnSuccess) {
		bcopy(bounce, reference, CHECK_BLOCKS * BLOCK_SIZE);
		IOLog("RPiSDHCI: trying DMA mode %u at %u MHz%s\n", mode, clockHz / 1000000, useIRQ ? ", interrupts" : "");
		setUpDMA(getProvider(), reference, mode);
		err = 0;
	}
	IOLockUnlock(lock);
	IOFreeData(reference, CHECK_BLOCKS * BLOCK_SIZE);
	IOLog("RPiSDHCI: %s\n", dma == DMA_ADMA2 ? "ADMA2" : dma == DMA_SDMA ? "SDMA" : "programmed I/O");
	return err;
}

static int
sysctl_sdhci_dma SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	if (gSDHCI == NULL) {
		return ENODEV;
	}
	int value = (int)gSDHCI->dmaMode();
	int changed = 0;
	int err = sysctl_io_number(req, value, sizeof(value), &value, &changed);
	if (err != 0 || !changed) {
		return err;
	}
	if (value < 0 || value > 2) {
		return EINVAL;
	}
	return gSDHCI->setDMAMode((uint32_t)value);
}
SYSCTL_PROC(_debug, OID_AUTO, sdhci_dma, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    sysctl_sdhci_dma, "I", "SD controller DMA: 0 none, 1 ADMA2 (else SDMA), 2 SDMA");

void
RPiSDHCI::free(void)
{
	if (intSource != NULL) {
		intSource->disable();
		workLoop->removeEventSource(intSource);
		intSource->release();
	}
	OSSafeReleaseNULL(workLoop);
	OSSafeReleaseNULL(bounceMem);
	OSSafeReleaseNULL(descMem);
	if (waitLock != NULL) {
		IOLockFree(waitLock);
	}
	if (lock != NULL) {
		IOLockFree(lock);
	}
	OSSafeReleaseNULL(regsMap);
	super::free();
}

#pragma mark IOBlockStorageDevice

IOReturn
RPiSDHCI::doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 nblks,
    IOStorageAttributes *attributes, IOStorageCompletion *completion)
{
	(void)attributes;
	bool write = buffer->getDirection() == kIODirectionOut;
	IOReturn ret = kIOReturnSuccess;

	if (block + nblks > blocks) {
		ret = kIOReturnBadArgument;
	}
	IOLockLock(lock);
	for (UInt64 done = 0; ret == kIOReturnSuccess && done < nblks;) {
		uint32_t n = (uint32_t)(nblks - done < MAX_BLOCKS ? nblks - done : MAX_BLOCKS);
		IOByteCount offset = done * BLOCK_SIZE, length = (IOByteCount)n * BLOCK_SIZE;
		if (write) {
			if (buffer->readBytes(offset, bounce, length) != length) {
				ret = kIOReturnIOError;
				break;
			}
			ret = transfer(block + done, n, true);
		} else {
			ret = transfer(block + done, n, false);
			if (ret == kIOReturnSuccess && buffer->writeBytes(offset, bounce, length) != length) {
				ret = kIOReturnIOError;
			}
		}
		done += n;
	}
	IOLockUnlock(lock);

	IOStorage::complete(completion, ret, ret == kIOReturnSuccess ? nblks * BLOCK_SIZE : 0);
	return kIOReturnSuccess;
}

IOReturn
RPiSDHCI::doSynchronize(UInt64 block, UInt64 nblks, IOStorageSynchronizeOptions options)
{
	(void)block;
	(void)nblks;
	(void)options;
	return kIOReturnSuccess;
}

IOReturn RPiSDHCI::doEjectMedia(void) { return kIOReturnUnsupported; }
IOReturn RPiSDHCI::doFormatMedia(UInt64 byteCapacity) { (void)byteCapacity; return kIOReturnUnsupported; }

UInt32
RPiSDHCI::doGetFormatCapacities(UInt64 *capacities, UInt32 capacitiesMaxCount) const
{
	if (capacities != NULL && capacitiesMaxCount > 0) {
		capacities[0] = blocks * BLOCK_SIZE;
	}
	return 1;
}

char *RPiSDHCI::getVendorString(void) { return vendor; }
char *RPiSDHCI::getProductString(void) { return product; }
char *RPiSDHCI::getRevisionString(void) { return revision; }
char *RPiSDHCI::getAdditionalDeviceInfoString(void) { return (char *)"SD card"; }

IOReturn RPiSDHCI::reportBlockSize(UInt64 *blockSize) { *blockSize = BLOCK_SIZE; return kIOReturnSuccess; }
IOReturn RPiSDHCI::reportEjectability(bool *isEjectable) { *isEjectable = false; return kIOReturnSuccess; }
IOReturn RPiSDHCI::reportMaxValidBlock(UInt64 *maxBlock) { *maxBlock = blocks - 1; return kIOReturnSuccess; }

IOReturn
RPiSDHCI::reportMediaState(bool *mediaPresent, bool *changedState)
{
	*mediaPresent = present;
	if (changedState != NULL) {
		*changedState = false;
	}
	return kIOReturnSuccess;
}

IOReturn RPiSDHCI::reportRemovability(bool *isRemovable) { *isRemovable = false; return kIOReturnSuccess; }
IOReturn RPiSDHCI::reportWriteProtection(bool *isWriteProtected) { *isWriteProtected = false; return kIOReturnSuccess; }
