/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOLib.h>
#include <libkern/OSByteOrder.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/param.h>

#include "../../pie/sdio.h"
#include "PIESDIO.h"

/* The card's common registers, its functions' basic registers and the CIS tuples used here. */
enum {
	CCCR_REVISION = 0x00, CCCR_IO_ENABLE = 0x02, CCCR_IO_READY = 0x03, CCCR_INT_ENABLE = 0x04,
	CCCR_INT_PENDING = 0x05, CCCR_ABORT = 0x06, CCCR_BUS_IF = 0x07, CCCR_CAPS = 0x08, CCCR_CIS = 0x09,
	CCCR_SPEED = 0x13,
	ABORT_RES = 1U << 3, BUS_IF_WIDTH_MASK = 3, BUS_IF_WIDTH_4 = 2, CAPS_SMB = 1U << 1,
	SPEED_SHS = 1U << 0, SPEED_EHS = 1U << 1,
	FBR_INTERFACE = 0x00, FBR_CIS = 0x09, FBR_BLOCK_SIZE = 0x10,
	CIS_FIRST = 0x1000, CIS_LAST = 0x17fff,
	CISTPL_NULL = 0x00, CISTPL_MANFID = 0x20, CISTPL_FUNCE = 0x22, CISTPL_END = 0xff,
	R4_READY = 1U << 31, R4_MEMORY = 1U << 27, R4_OCR = 0x00ffffff,
	OCR_3V3 = 0x00300000, OCR_ANY = 0x00ff8000,
	R5_ERRORS = 0xcb00,
	CMD53_BLOCKS_MAX = 511, CMD53_BYTES_MAX = 512,
};
#define FBR(fn, reg)	((fn) * 0x100 + (reg))

OSDefineMetaClassAndStructors(PIESDIOFunction, IOService);
OSDefineMetaClassAndAbstractStructors(PIESDIOHost, IOService);

#pragma mark Bus

int
PIESDIOHost::claimBus(void)
{
	IOLockLock(busLock);
	busOwner = current_thread();
	return 0;
}

void
PIESDIOHost::releaseBus(void)
{
	busOwner = NULL;
	IOLockUnlock(busLock);
}

bool
PIESDIOHost::busClaimed(void)
{
	return busOwner == current_thread();
}

int
PIESDIOHost::io52(unsigned fn, uint32_t addr, bool write, uint8_t value, uint8_t *out)
{
	uint32_t resp[4] = { 0, 0, 0, 0 };
	uint32_t arg = (write ? 1U << 31 : 0) | ((fn & 7) << 28) | ((addr & 0x1ffff) << 9) | (write ? value : 0);

	if (dead) {
		return ENXIO;
	}
	IOReturn ret = sdioCommand(52, arg, kSDIORespR5, resp);
	if (ret != kIOReturnSuccess) {
		return ret == kIOReturnTimeout ? ETIMEDOUT : EIO;
	}
	if (resp[0] & R5_ERRORS) {
		return EIO;
	}
	if (out != NULL) {
		*out = (uint8_t)resp[0];
	}
	return 0;
}

/* One CMD53: blksz 0 moves len bytes in byte mode, otherwise len / blksz blocks. */
int
PIESDIOHost::transfer(PIESDIOFunction *f, uint32_t addr, uint8_t *buf, uint32_t len, uint32_t blksz, bool write,
    bool incr)
{
	uint32_t count = blksz != 0 ? len / blksz : len;
	uint32_t arg = (write ? 1U << 31 : 0) | ((uint32_t)(f->number & 7) << 28) | (blksz != 0 ? 1U << 27 : 0) |
	    (incr ? 1U << 26 : 0) | ((addr & 0x1ffff) << 9) | (count & 0x1ff);
	uint32_t r5 = 0;
	IOReturn ret = sdioData(arg, buf, blksz != 0 ? blksz : len, blksz != 0 ? count : 1, write, &r5);
	if (ret == kIOReturnSuccess && !(r5 & R5_ERRORS)) {
		return 0;
	}
	IOLog("%s: CMD53 %s of %u bytes, function %u at 0x%x, failed: 0x%x, R5 0x%x\n", getName(),
	    write ? "write" : "read", len, f->number, addr, ret, r5);
	recover(f->number);
	return ret == kIOReturnTimeout ? ETIMEDOUT : EIO;
}

int
PIESDIOHost::io53(PIESDIOFunction *f, uint32_t addr, void *buf, size_t len, bool write, bool incr)
{
	uint8_t *p = (uint8_t *)buf;
	uint32_t bs = f->blockSize;

	if (dead) {
		return ENXIO;
	}
	if (bs == 0) {
		return EINVAL;
	}
	while (len > 0) {
		uint32_t n, blk = 0;
		if ((cardCaps & CAPS_SMB) && len >= bs) {
			n = (uint32_t)MIN(len / bs, (size_t)CMD53_BLOCKS_MAX) * bs;
			blk = bs;
		} else {
			n = (uint32_t)MIN(len, (size_t)MIN(bs, (uint32_t)CMD53_BYTES_MAX));
		}
		int err = transfer(f, addr, p, n, blk, write, incr);
		if (err != 0) {
			return err;
		}
		p += n;
		len -= n;
		if (incr) {
			addr += n;
		}
	}
	return 0;
}

/* After a failed transfer: the host's own reset, then the function's abort, else the card set up again. */
void
PIESDIOHost::recover(unsigned fn)
{
	sdioRecover();
	if (io52(0, CCCR_ABORT, true, (uint8_t)(fn & 7), NULL) == 0) {
		return;
	}
	IOLog("%s: the card did not answer the abort; setting it up again\n", getName());
	(void)reinitCard();
}

/*
 * The card powered up and set up again, with the functions' block sizes,
 * enables and interrupt enables as they were. The host's card interrupt
 * stays masked until they are; then the functions' reset callbacks run,
 * but the asker's when a function's driver asked for this one, unless
 * another new set-up waits for its callbacks too. A try counts as failed
 * unless all of them are set. After three failed tries no more I/O goes
 * to the card, and a card given up is not tried again. Called with the
 * bus claimed. The set-up count and the card given up change under
 * irqLock too, as a reset callback reads them without the claim.
 */
bool
PIESDIOHost::reinitCard(PIESDIOFunction *asker)
{
	if (dead) {
		return false;
	}
	if (!polling) {
		sdioInterruptEnable(false);
	}
	for (int tries = 0; tries < 3; tries++) {
		recoveries++;
		if (!initCard()) {
			continue;
		}
		int err = restoreFunctions();
		if (err != 0) {
			IOLog("%s: the functions' settings did not go back to the card: %d\n", getName(), err);
			continue;
		}
		if (!polling) {
			sdioInterruptEnable(interrupts != 0);
		}
		if (asker != NULL) {
			IOLog("%s: function %u's driver had the card set up again, in %d %s\n", getName(), asker->number,
			    tries + 1, tries == 0 ? "try" : "tries");
		} else {
			IOLog("%s: the card is set up again, in %d %s\n", getName(), tries + 1, tries == 0 ? "try" : "tries");
		}
		IOLockLock(irqLock);
		generation++;
		resetAsked = asker != NULL && !resetPending ? (uint8_t)(1U << asker->number) : 0;
		resetPending = true;
		IOLockWakeup(irqLock, &irqPending, true);
		IOLockUnlock(irqLock);
		return true;
	}
	IOLog("%s: the card did not come back; no more I/O to it\n", getName());
	IOLockLock(irqLock);
	dead = true;
	IOLockUnlock(irqLock);
	return false;
}

int
PIESDIOHost::restoreFunctions(void)
{
	int err = 0;
	for (unsigned fn = 1; fn <= functions && fn <= kSDIOMaxFunctions && err == 0; fn++) {
		PIESDIOFunction *f = funcs[fn];
		if (f != NULL && f->blockSize != 0) {
			err = io52(0, FBR(fn, FBR_BLOCK_SIZE), true, (uint8_t)f->blockSize, NULL);
			if (err == 0) {
				err = io52(0, FBR(fn, FBR_BLOCK_SIZE + 1), true, (uint8_t)(f->blockSize >> 8), NULL);
			}
		}
	}
	if (err == 0 && enabled != 0) {
		err = io52(0, CCCR_IO_ENABLE, true, enabled, NULL);
	}
	if (err == 0 && interrupts != 0) {
		err = io52(0, CCCR_INT_ENABLE, true, (uint8_t)(interrupts | 1), NULL);
	}
	return err;
}

int
PIESDIOHost::enableFunction(PIESDIOFunction *f, bool on)
{
	uint8_t v = 0, bit = (uint8_t)(1U << f->number);
	int err = io52(0, CCCR_IO_ENABLE, false, 0, &v);
	if (err == 0) {
		err = io52(0, CCCR_IO_ENABLE, true, on ? (uint8_t)(v | bit) : (uint8_t)(v & ~bit), NULL);
	}
	if (err != 0) {
		return err;
	}
	enabled = on ? (uint8_t)(enabled | bit) : (uint8_t)(enabled & ~bit);
	for (int i = 0; on && i < 100; i++) {
		if ((err = io52(0, CCCR_IO_READY, false, 0, &v)) != 0) {
			return err;
		}
		if (v & bit) {
			return 0;
		}
		IOSleep(10);
	}
	return on ? ETIMEDOUT : 0;
}

int
PIESDIOHost::setBlockSize(PIESDIOFunction *f, unsigned size)
{
	if (size == 0) {
		size = MIN(f->maxBlockSize, 512);
	}
	if (size == 0 || size > f->maxBlockSize) {
		return EINVAL;
	}
	int err = io52(0, FBR(f->number, FBR_BLOCK_SIZE), true, (uint8_t)size, NULL);
	if (err == 0) {
		err = io52(0, FBR(f->number, FBR_BLOCK_SIZE + 1), true, (uint8_t)(size >> 8), NULL);
	}
	if (err == 0) {
		f->blockSize = (uint16_t)size;
	}
	return err;
}

size_t
PIESDIOHost::alignSize(PIESDIOFunction *f, size_t len)
{
	size_t n = (len + 3) & ~(size_t)3;
	if (f->blockSize != 0 && n > f->blockSize) {
		n = (n + f->blockSize - 1) / f->blockSize * f->blockSize;
	}
	return n;
}

size_t
PIESDIOHost::maxRequest(PIESDIOFunction *f)
{
	if (cardCaps & CAPS_SMB) {
		return (size_t)CMD53_BLOCKS_MAX * f->blockSize;
	}
	return MIN(f->blockSize, (uint16_t)CMD53_BYTES_MAX);
}

/*
 * Called with the bus claimed, so no handler runs meanwhile. A handler
 * removed is never called again, even when the card's interrupt enable
 * does not clear: the card is then set up again without it, with the
 * host's card interrupt masked until it is. On a card given up, the
 * handler goes without any I/O.
 */
int
PIESDIOHost::setHandler(PIESDIOFunction *f, void (*handler)(void *), void *arg)
{
	uint8_t bit = (uint8_t)(1U << f->number);
	uint8_t want = handler != NULL ? (uint8_t)(interrupts | bit) : (uint8_t)(interrupts & ~bit);
	uint8_t reg = want != 0 ? (uint8_t)(want | 1) : 0;

	if (handler == NULL) {
		f->handler = NULL;
		f->handlerArg = NULL;
		interrupts = want;
		if (dead) {
			return ENXIO;
		}
		int err = io52(0, CCCR_INT_ENABLE, true, reg, NULL);
		if (err != 0) {
			sdioRecover();
			err = io52(0, CCCR_INT_ENABLE, true, reg, NULL);
		}
		if (err != 0) {
			IOLog("%s: function %u's interrupt enable did not clear: %d; setting the card up again\n", getName(),
			    f->number, err);
			return reinitCard() ? 0 : err;
		}
	} else {
		f->handlerArg = arg;
		f->handler = handler;
		int err = io52(0, CCCR_INT_ENABLE, true, reg, NULL);
		if (err != 0) {
			f->handler = NULL;
			f->handlerArg = NULL;
			return err;
		}
		interrupts = want;
	}
	if (!polling) {
		sdioInterruptEnable(want != 0);
	}
	return 0;
}

/*
 * Under irqLock, so the interrupt thread reads a whole pair. A callback
 * running on another thread is waited for, so its argument may be freed
 * once this returns.
 */
void
PIESDIOHost::setResetHandler(PIESDIOFunction *f, void (*handler)(void *), void *arg)
{
	IOLockLock(irqLock);
	f->resetArg = handler != NULL ? arg : NULL;
	f->resetHandler = handler;
	while (f->resetThread != NULL && f->resetThread != current_thread()) {
		IOLockSleep(irqLock, &f->resetThread, THREAD_UNINT);
	}
	IOLockUnlock(irqLock);
}

/* A function driver's request: the card powered up and set up again, as after an error. Called with the bus claimed. */
int
PIESDIOHost::resetCard(PIESDIOFunction *f)
{
	return reinitCard(f) ? 0 : ENXIO;
}

/* The count of the card's set-ups, or 0 for a card given up. Under irqLock, so it needs no claim. */
unsigned
PIESDIOHost::cardGeneration(void)
{
	IOLockLock(irqLock);
	unsigned g = dead ? 0 : generation;
	IOLockUnlock(irqLock);
	return g;
}

/*
 * On the interrupt thread, the bus not claimed: each function's reset
 * callback, after a new set-up, but those of the functions in skip.
 */
void
PIESDIOHost::announceReset(uint8_t skip)
{
	for (unsigned fn = 1; fn <= kSDIOMaxFunctions; fn++) {
		IOLockLock(irqLock);
		PIESDIOFunction *f = funcs[fn];
		void (*handler)(void *) = f != NULL && !(skip & (1U << fn)) ? f->resetHandler : NULL;
		void *arg = f != NULL ? f->resetArg : NULL;
		if (handler != NULL) {
			f->resetThread = current_thread();
		}
		IOLockUnlock(irqLock);
		if (handler == NULL) {
			continue;
		}
		handler(arg);
		IOLockLock(irqLock);
		f->resetThread = NULL;
		IOLockWakeup(irqLock, &f->resetThread, false);
		IOLockUnlock(irqLock);
	}
}

#pragma mark Card

/* The tuples from addr: the IDs of CISTPL_MANFID, and the largest block of CISTPL_FUNCE. */
bool
PIESDIOHost::readCIS(uint32_t addr, uint16_t *vendorOut, uint16_t *deviceOut, uint16_t *maxBlock, bool function)
{
	for (int tuples = 0; tuples < 64; tuples++) {
		uint8_t code = 0, link = 0, b[16] = { 0 };
		if (addr < CIS_FIRST || addr > CIS_LAST || io52(0, addr, false, 0, &code) != 0) {
			return false;
		}
		if (code == CISTPL_END) {
			return true;
		}
		if (code == CISTPL_NULL) {
			addr++;
			continue;
		}
		if (addr + 1 > CIS_LAST || io52(0, addr + 1, false, 0, &link) != 0) {
			return false;
		}
		if (link == 0xff) {
			return true;
		}
		if (addr + 1 + link > CIS_LAST) {
			return false;
		}
		unsigned n = MIN((unsigned)link, (unsigned)sizeof(b));
		for (unsigned i = 0; i < n; i++) {
			if (io52(0, addr + 2 + i, false, 0, &b[i]) != 0) {
				return false;
			}
		}
		if (code == CISTPL_MANFID && n >= 4 && vendorOut != NULL && deviceOut != NULL) {
			*vendorOut = (uint16_t)(b[0] | (b[1] << 8));
			*deviceOut = (uint16_t)(b[2] | (b[3] << 8));
		}
		if (code == CISTPL_FUNCE && maxBlock != NULL) {
			if (!function && n >= 3 && b[0] == 0x00) {
				*maxBlock = (uint16_t)(b[1] | (b[2] << 8));
			} else if (function && n >= 14 && b[0] == 0x01) {
				*maxBlock = (uint16_t)(b[12] | (b[13] << 8));
			}
		}
		addr += 2 + link;
	}
	return true;
}

/*
 * Power-up and set-up, following the SDIO simplified specification: the
 * card's reset and CMD0, CMD5 until it is ready, CMD3 and CMD7, the CCCR
 * and the common CIS, then a 4-bit bus and high speed where the card has
 * it. Called with the bus claimed.
 */
bool
PIESDIOHost::initCard(void)
{
	uint32_t resp[4];
	uint8_t v = 0;

	if (!sdioPowerUp() || !sdioSetClock(kSDIOClockInitHz)) {
		IOLog("%s: the host did not power up\n", getName());
		return false;
	}
	(void)sdioCommand(52, (1U << 31) | (CCCR_ABORT << 9) | ABORT_RES, kSDIORespR5, resp);
	(void)sdioCommand(0, 0, kSDIORespNone, NULL);
	IOSleep(1);
	if (sdioCommand(5, 0, kSDIORespR4, resp) != kIOReturnSuccess) {
		IOLog("%s: no SDIO card answers\n", getName());
		return false;
	}
	uint32_t ocr = resp[0] & R4_OCR;
	functions = (uint8_t)((resp[0] >> 28) & 7);
	bool memory = (resp[0] & R4_MEMORY) != 0;
	uint32_t want = (ocr & OCR_3V3) != 0 ? ocr & OCR_3V3 : ocr & OCR_ANY;
	if (want == 0) {
		IOLog("%s: no common voltage (OCR 0x%06x)\n", getName(), ocr);
		return false;
	}
	bool ready = false;
	for (int i = 0; i < 100 && !ready; i++) {
		if (sdioCommand(5, want, kSDIORespR4, resp) != kIOReturnSuccess) {
			IOLog("%s: CMD5 failed\n", getName());
			return false;
		}
		ready = (resp[0] & R4_READY) != 0;
		if (!ready) {
			IOSleep(10);
		}
	}
	if (!ready) {
		IOLog("%s: the card did not become ready (OCR 0x%08x)\n", getName(), resp[0]);
		return false;
	}
	if (sdioCommand(3, 0, kSDIORespR1, resp) != kIOReturnSuccess) {
		IOLog("%s: CMD3 failed\n", getName());
		return false;
	}
	rca = resp[0] >> 16;
	if (sdioCommand(7, rca << 16, kSDIORespR1, resp) != kIOReturnSuccess) {
		IOLog("%s: CMD7 failed\n", getName());
		return false;
	}
	commonCIS = 0;
	if (io52(0, CCCR_REVISION, false, 0, &cccrRevision) != 0 || io52(0, CCCR_CAPS, false, 0, &cardCaps) != 0) {
		IOLog("%s: the CCCR does not read\n", getName());
		return false;
	}
	for (unsigned i = 0; i < 3; i++) {
		if (io52(0, CCCR_CIS + i, false, 0, &v) != 0) {
			return false;
		}
		commonCIS |= (uint32_t)v << (8 * i);
	}
	if (!readCIS(commonCIS, &vendor, &device, &f0MaxBlock, false)) {
		IOLog("%s: the common CIS at 0x%x does not read\n", getName(), commonCIS);
	}

	bool wide = false;
	if (io52(0, CCCR_BUS_IF, false, 0, &v) == 0 &&
	    io52(0, CCCR_BUS_IF, true, (uint8_t)((v & ~BUS_IF_WIDTH_MASK) | BUS_IF_WIDTH_4), NULL) == 0) {
		sdioSetBusWidth4();
		wide = true;
	}
	if (!sdioSetClock(kSDIOClockRunHz)) {
		return false;
	}
	uint32_t hsOn = 1;
	bool hs = false;
	if (!(PE_parse_boot_argn("sdio_hs", &hsOn, sizeof(hsOn)) && hsOn == 0) &&
	    io52(0, CCCR_SPEED, false, 0, &v) == 0 && (v & SPEED_SHS) &&
	    io52(0, CCCR_SPEED, true, (uint8_t)(v | SPEED_EHS), NULL) == 0) {
		hs = sdioSetHighSpeed(true) && sdioSetClock(kSDIOClockHighSpeedHz) &&
		    io52(0, CCCR_SPEED, false, 0, &v) == 0 && (v & SPEED_EHS);
		if (!hs) {
			IOLog("%s: high speed did not work; %u kHz\n", getName(), kSDIOClockRunHz / 1000);
			if (!sdioSetHighSpeed(false) || !sdioSetClock(kSDIOClockRunHz) ||
			    io52(0, CCCR_SPEED, true, (uint8_t)(v & ~SPEED_EHS), NULL) != 0 ||
			    io52(0, CCCR_SPEED, false, 0, &v) != 0 || (v & SPEED_EHS)) {
				IOLog("%s: the card did not go back to %u kHz\n", getName(), kSDIOClockRunHz / 1000);
				return false;
			}
		}
	}
	IOLog("%s: SDIO card, RCA 0x%04x, CCCR 0x%02x, capabilities 0x%02x, %u function%s%s, CIS 0x%x, "
	    "ID %04x:%04x, %u kHz, %s bus%s\n", getName(), rca, cccrRevision, cardCaps, functions,
	    functions == 1 ? "" : "s", memory ? " and memory" : "", commonCIS, vendor, device, sdioClock / 1000,
	    wide ? "4-bit" : "1-bit", hs ? ", high speed" : "");
	return true;
}

/* A nub for each function, attached; registered once the bus is released. Function 1 is enabled by its driver. */
void
PIESDIOHost::publishFunctions(void)
{
	for (unsigned fn = 1; fn <= functions && fn <= kSDIOMaxFunctions; fn++) {
		uint8_t iface = 0, v = 0;
		uint32_t cis = 0;
		uint16_t maxBlock = 0, fv = 0, fd = 0;
		(void)io52(0, FBR(fn, FBR_INTERFACE), false, 0, &iface);
		for (unsigned i = 0; i < 3; i++) {
			if (io52(0, FBR(fn, FBR_CIS) + i, false, 0, &v) == 0) {
				cis |= (uint32_t)v << (8 * i);
			}
		}
		(void)readCIS(cis, &fv, &fd, &maxBlock, true);
		if (fv == 0) {
			fv = vendor;
			fd = device;
		}
		if (maxBlock == 0) {
			maxBlock = 512;
		}
		PIESDIOFunction *f = new PIESDIOFunction;
		if (f == NULL || !f->init()) {
			OSSafeReleaseNULL(f);
			continue;
		}
		f->host = this;
		f->number = (uint8_t)fn;
		f->vendor = fv;
		f->device = fd;
		f->maxBlockSize = maxBlock;
		if (setBlockSize(f, 0) != 0) {
			f->blockSize = 0;
		}
		f->setProperty("SDIOVendorID", fv, 32);
		f->setProperty("SDIODeviceID", fd, 32);
		f->setProperty("SDIOFunction", fn, 32);
		f->setProperty("SDIOInterfaceClass", iface & 0xf, 32);
		IOLog("%s: function %u: interface class %u, CIS 0x%x, ID %04x:%04x, block %u of at most %u\n", getName(),
		    fn, iface & 0xf, cis, fv, fd, f->blockSize, maxBlock);
		if (!f->attach(this)) {
			f->release();
			continue;
		}
		funcs[fn] = f;
	}
}

#pragma mark Threads

void
PIESDIOHost::threadDone(void)
{
	IOLockLock(irqLock);
	if (--threadsRunning == 0) {
		IOLockWakeup(irqLock, &threadsRunning, false);
	}
	IOLockUnlock(irqLock);
}

/*
 * The handlers run with the bus claimed. With one handler it runs at
 * once; with more, or when polling, Int Pending says whose. A new set-up
 * made while this waited for the claim has its reset callbacks run first:
 * the interrupt stays pending for the next pass. One made by a handler's
 * own I/O stops the dispatch too, but the interrupt is not kept pending:
 * the card's is level-triggered and the new set-up unmasks it, so a real
 * one signals again. A card given up runs no further handler.
 */
void
PIESDIOHost::dispatch(void)
{
	uint8_t pending = 0;
	unsigned only = 0, count = 0;

	claimBus();
	IOLockLock(irqLock);
	bool reset = resetPending;
	if (reset) {
		irqPending = true;
	}
	IOLockUnlock(irqLock);
	if (reset) {
		releaseBus();
		return;
	}
	for (unsigned fn = 1; fn <= functions; fn++) {
		if (funcs[fn] != NULL && funcs[fn]->handler != NULL) {
			only = fn;
			count++;
		}
	}
	if (!dead && count != 0) {
		if (count == 1 && !polling) {
			pending = (uint8_t)(1U << only);
		} else if (io52(0, CCCR_INT_PENDING, false, 0, &pending) != 0) {
			pending = 0;
		}
		for (unsigned fn = 1; fn <= functions; fn++) {
			PIESDIOFunction *f = funcs[fn];
			if (!(pending & (1U << fn)) || f == NULL || f->handler == NULL) {
				continue;
			}
			f->handler(f->handlerArg);
			IOLockLock(irqLock);
			reset = resetPending;
			bool stop = reset || dead;
			IOLockUnlock(irqLock);
			if (stop) {
				break;
			}
		}
	}
	if (!reset && !polling && !dead && interrupts != 0) {
		sdioInterruptEnable(true);
	}
	releaseBus();
}

void
PIESDIOHost::irqThread(void *arg, wait_result_t wr)
{
	(void)wr;
	PIESDIOHost *self = (PIESDIOHost *)arg;
	for (;;) {
		bool stop, reset, irq;
		uint8_t asked;
		if (self->polling) {
			IOSleep(2);
		}
		IOLockLock(self->irqLock);
		while (!self->polling && !self->irqPending && !self->resetPending && !self->stopping) {
			IOLockSleep(self->irqLock, &self->irqPending, THREAD_UNINT);
		}
		stop = self->stopping;
		reset = self->resetPending;
		asked = reset ? self->resetAsked : 0;
		irq = self->irqPending || self->polling;
		self->irqPending = false;
		self->resetPending = false;
		self->resetAsked = 0;
		IOLockUnlock(self->irqLock);
		if (stop) {
			break;
		}
		if (reset) {
			self->announceReset(asked);
		}
		if (irq) {
			self->dispatch();
		}
	}
	self->threadDone();
	self->release();
}

void
PIESDIOHost::initThread(void *arg, wait_result_t wr)
{
	(void)wr;
	PIESDIOHost *self = (PIESDIOHost *)arg;
	thread_t thread;

	bool ok = false;
	for (int tries = 0; tries < 3 && !ok; tries++) {
		if (tries > 0) {
			IOSleep(1000);
			IOLockLock(self->irqLock);
			bool stop = self->stopping;
			IOLockUnlock(self->irqLock);
			if (stop) {
				break;
			}
		}
		self->claimBus();
		ok = self->initCard();
		if (ok) {
			IOLockLock(self->irqLock);
			self->generation = 1;
			IOLockUnlock(self->irqLock);
			self->publishFunctions();
		}
		self->releaseBus();
	}
	if (ok) {
		for (unsigned fn = 1; fn <= kSDIOMaxFunctions; fn++) {
			if (self->funcs[fn] != NULL) {
				self->funcs[fn]->registerService();
			}
		}
		IOLockLock(self->irqLock);
		bool start = !self->stopping;
		if (start) {
			self->threadsRunning++;
		}
		IOLockUnlock(self->irqLock);
		self->retain();
		if (start && kernel_thread_start(&PIESDIOHost::irqThread, self, &thread) == KERN_SUCCESS) {
			thread_deallocate(thread);
		} else {
			if (start) {
				self->threadDone();
			}
			self->release();
		}
	} else {
		IOLog("%s: no SDIO card set up\n", self->getName());
	}
	self->threadDone();
	self->release();
}

/*
 * Power-up and enumeration run on a thread of their own, so the host's
 * start returns at once. A first set-up that fails is tried again, three
 * times in all, a second apart.
 */
bool
PIESDIOHost::sdioStartBus(void)
{
	thread_t thread;
	uint32_t poll = 0;

	busLock = IOLockAlloc();
	irqLock = IOLockAlloc();
	if (busLock == NULL || irqLock == NULL) {
		return false;
	}
	polling = PE_parse_boot_argn("sdio_poll", &poll, sizeof(poll)) && poll != 0;
	threadsRunning = 1;
	retain();
	if (kernel_thread_start(&PIESDIOHost::initThread, this, &thread) != KERN_SUCCESS) {
		threadsRunning = 0;
		release();
		return false;
	}
	thread_deallocate(thread);
	return true;
}

void
PIESDIOHost::sdioStopBus(void)
{
	if (irqLock == NULL) {
		return;
	}
	IOLockLock(irqLock);
	stopping = true;
	IOLockWakeup(irqLock, &irqPending, false);
	while (threadsRunning > 0) {
		IOLockSleep(irqLock, &threadsRunning, THREAD_UNINT);
	}
	IOLockUnlock(irqLock);
	for (unsigned fn = 1; fn <= kSDIOMaxFunctions; fn++) {
		OSSafeReleaseNULL(funcs[fn]);
	}
}

void
PIESDIOHost::sdioInterrupt(void)
{
	IOLockLock(irqLock);
	irqPending = true;
	IOLockWakeup(irqLock, &irqPending, true);
	IOLockUnlock(irqLock);
}

void
PIESDIOHost::free(void)
{
	if (busLock != NULL) {
		IOLockFree(busLock);
	}
	if (irqLock != NULL) {
		IOLockFree(irqLock);
	}
	IOService::free();
}

#pragma mark <pie/sdio.h>

static PIESDIOFunction *
sdio_function(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = OSDynamicCast(PIESDIOFunction, (OSObject *)f);
	return s != NULL && s->host != NULL ? s : NULL;
}

static PIESDIOFunction *
sdio_claimed(struct pie_sdio_func *f, int *err)
{
	PIESDIOFunction *s = sdio_function(f);
	*err = s == NULL ? ENXIO : !s->host->busClaimed() ? EPERM : 0;
	return *err == 0 ? s : NULL;
}

int
pie_sdio_claim(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? s->host->claimBus() : ENXIO;
}

void
pie_sdio_release(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = sdio_function(f);
	if (s != NULL && s->host->busClaimed()) {
		s->host->releaseBus();
	}
}

int
pie_sdio_read1(struct pie_sdio_func *f, unsigned int fn, uint32_t addr, uint8_t *val)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	if (s != NULL && fn != 0 && fn != s->number) {
		err = EINVAL;
	}
	return err == 0 ? s->host->io52(fn, addr, false, 0, val) : err;
}

int
pie_sdio_write1(struct pie_sdio_func *f, unsigned int fn, uint32_t addr, uint8_t val)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	if (s != NULL && fn != 0 && fn != s->number) {
		err = EINVAL;
	}
	return err == 0 ? s->host->io52(fn, addr, true, val, NULL) : err;
}

int
pie_sdio_read4(struct pie_sdio_func *f, uint32_t addr, uint32_t *val)
{
	uint8_t b[4];
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	if (s != NULL && (err = s->host->io53(s, addr, b, sizeof(b), false, true)) == 0) {
		*val = OSReadLittleInt32(b, 0);
	}
	return err;
}

int
pie_sdio_write4(struct pie_sdio_func *f, uint32_t addr, uint32_t val)
{
	uint8_t b[4];
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	OSWriteLittleInt32(b, 0, val);
	return s != NULL ? s->host->io53(s, addr, b, sizeof(b), true, true) : err;
}

int
pie_sdio_read(struct pie_sdio_func *f, uint32_t addr, void *buf, size_t len, bool incr)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	return s != NULL ? s->host->io53(s, addr, buf, len, false, incr) : err;
}

int
pie_sdio_write(struct pie_sdio_func *f, uint32_t addr, const void *buf, size_t len, bool incr)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	return s != NULL ? s->host->io53(s, addr, (void *)(uintptr_t)buf, len, true, incr) : err;
}

int
pie_sdio_enable(struct pie_sdio_func *f)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	return s != NULL ? s->host->enableFunction(s, true) : err;
}

int
pie_sdio_disable(struct pie_sdio_func *f)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	return s != NULL ? s->host->enableFunction(s, false) : err;
}

int
pie_sdio_set_block_size(struct pie_sdio_func *f, unsigned int size)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	return s != NULL ? s->host->setBlockSize(s, size) : err;
}

size_t
pie_sdio_align_size(struct pie_sdio_func *f, size_t len)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? s->host->alignSize(s, len) : len;
}

size_t
pie_sdio_max_request(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? s->host->maxRequest(s) : 0;
}

int
pie_sdio_interrupt_register(struct pie_sdio_func *f, void (*handler)(void *), void *arg)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	if (s != NULL && handler == NULL) {
		err = EINVAL;
	}
	return err == 0 ? s->host->setHandler(s, handler, arg) : err;
}

int
pie_sdio_reset_register(struct pie_sdio_func *f, void (*handler)(void *), void *arg)
{
	PIESDIOFunction *s = sdio_function(f);
	if (s == NULL) {
		return ENXIO;
	}
	if (s->host->busClaimed()) {
		return EDEADLK;
	}
	s->host->setResetHandler(s, handler, arg);
	return 0;
}

int
pie_sdio_reset_card(struct pie_sdio_func *f)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	return s != NULL ? s->host->resetCard(s) : err;
}

unsigned int
pie_sdio_generation(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? s->host->cardGeneration() : 0;
}

int
pie_sdio_interrupt_unregister(struct pie_sdio_func *f)
{
	int err;
	PIESDIOFunction *s = sdio_claimed(f, &err);
	return s != NULL ? s->host->setHandler(s, NULL, NULL) : err;
}

bool
pie_sdio_is_uhs(struct pie_sdio_func *f)
{
	(void)f;
	return false;
}

uint32_t
pie_sdio_clock(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? s->host->clockHz() : 0;
}

unsigned int
pie_sdio_number(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? s->number : 0;
}

uint16_t
pie_sdio_vendor(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? s->vendor : 0;
}

uint16_t
pie_sdio_device(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? s->device : 0;
}

unsigned int
pie_sdio_block_size(struct pie_sdio_func *f)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? s->blockSize : 0;
}

struct pie_sdio_func *
pie_sdio_function(struct pie_sdio_func *f, unsigned int fn)
{
	PIESDIOFunction *s = sdio_function(f);
	return s != NULL ? (struct pie_sdio_func *)s->host->function(fn) : NULL;
}
