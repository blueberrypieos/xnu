/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PIEDMABCM2835: the Broadcom DMA controllers of the BCM2711 and the
 * BCM2712, from their device tree nodes (brcm,bcm2835-dma, brcm,bcm2711-dma,
 * brcm,bcm2712-dma) and the BCM2711 peripherals datasheet's register facts.
 *
 * Each channel's registers are 0x100 apart from channel 0's; a node's
 * registers start at its first channel ((address >> 8) & 0xf), and its
 * brcm,dma-channel-mask says which channels are the OS's (the firmware's
 * are not). Three layouts:
 * - Legacy and LITE (the BCM2711's /soc node, channels 0-10): CS 0x00,
 *   CONBLK_AD 0x04, NEXTCONBK 0x1c, DEBUG 0x20 (bit 28: a LITE channel,
 *   whose blocks hold 65532 bytes at most); 32-byte control blocks (TI,
 *   SOURCE_AD, DEST_AD, TXFR_LEN, STRIDE, NEXTCONBK); bus addresses, RAM at
 *   0xc0000000 + its physical address (the low GB only) and the
 *   peripherals at 0x7exxxxxx, as /soc's dma-ranges say.
 * - DMA4, 40-bit (the BCM2711's /scb node, channels 11-14; the BCM2712's
 *   dma@10600, channels 6-11): CS 0x00, CB 0x04 (address >> 5), DEBUG
 *   0x0c, NEXT_CB 0x28; 32-byte control blocks (TI, SRC, SRCI, DEST,
 *   DESTI, LEN, NEXT_CB), SRCI and DESTI holding address bits 39:32 and the
 *   increment; RAM at its physical address, the BCM2711's peripherals at
 *   0x4_7exxxxxx (/scb's dma-ranges), the BCM2712's at their physical
 *   address.
 * - The BCM2712's legacy channels (dma@10000, 0-5): the legacy layout, with
 *   the control blocks' addresses >> 5 and STRIDE holding the source's and
 *   destination's address bits 39:32 (so no 2D transfers).
 * A transfer's last block interrupts; the channel is done when its
 * control block address reads 0, failed when CS says ERROR. A LITE
 * channel (DEBUG bit 28, legacy layouts) takes 65532 bytes a block. The
 * device's register must be 32 bits wide; the dmas cell's low 5 bits are
 * its DREQ (PERMAP). Two channels may share an interrupt (the BCM2711's
 * 7 and 8, 9 and 10): each line's action looks at its own channels'
 * pending bits only.
 *
 * Under QEMU (its firmware's revision says) only the legacy node is
 * driven: QEMU models those channels (without DREQ pacing), and takes
 * DMA4's registers for legacy ones.
 */

#include <IOKit/IOLib.h>
#include <libkern/OSByteOrder.h>
#include <string.h>

#include "DMAController.h"

enum {
	KIND_LEGACY, KIND_DMA4, KIND_2712,
	FLAG_LITE = 1U << 0,

	/* Legacy and LITE. */
	CS = 0x00, CONBLK_AD = 0x04, NEXTCONBK = 0x1c, DEBUG = 0x20,
	CS_ACTIVE = 1U << 0, CS_END = 1U << 1, CS_INT = 1U << 2, CS_ERROR = 1U << 8,
	CS_WRITES_PENDING = 1U << 6, CS_WAIT_FOR_WRITES = 1U << 28, CS_ABORT = 1U << 30, CS_RESET = 1U << 31,
	TI_INTEN = 1U << 0, TI_WAIT_RESP = 1U << 3, TI_DEST_INC = 1U << 4, TI_DEST_DREQ = 1U << 6,
	TI_SRC_INC = 1U << 8, TI_SRC_DREQ = 1U << 10, TI_PERMAP_SHIFT = 16,
	DEBUG_LITE = 1U << 28,

	/* DMA4. */
	D4_CS = 0x00, D4_CB = 0x04, D4_DEBUG = 0x0c, D4_NEXT_CB = 0x28,
	D4_CS_ERR = 1U << 10, D4_CS_PROT = 3U << 8, D4_CS_TRANSACTIONS = 1U << 25,
	D4_TI_INTEN = 1U << 0, D4_TI_WAIT_RESP = 1U << 2, D4_TI_WAIT_RD_RESP = 1U << 3,
	D4_TI_PERMAP_SHIFT = 9, D4_TI_S_DREQ = 1U << 14, D4_TI_D_DREQ = 1U << 15,
	D4_INFO_INC = 1U << 12, D4_DEBUG_RESET = 1U << 23,

	CB_BYTES = 32,
	LITE_MAX = 65532,
	NORMAL_MAX = 0x3ffffffc,      /* TXFR_LEN's and LEN's 30 bits, whole words */
	LEGACY_RAM_BUS = 0xc0000000U,
	PERIPH_CPU = 0xfc000000U, PERIPH_END = 0xff800000U, PERIPH_LEGACY_BUS = 0x7c000000U,
};
#define BCM2711_PERIPH_DMA4_BUS 0x47c000000ULL
#define kMaxTransfer (64U * 1024 * 1024)

class PIEDMABCM2835 : public PIEDMAController
{
	OSDeclareDefaultStructors(PIEDMABCM2835);

protected:
	virtual bool hwStart(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual uint64_t hwMaxAddress(void) APPLE_KEXT_OVERRIDE;
	virtual uint32_t hwMaxLength(Channel &c) APPLE_KEXT_OVERRIDE;
	virtual bool hwMemoryAddress(uint64_t pa, uint64_t *bus) APPLE_KEXT_OVERRIDE;
	virtual bool hwDeviceAddress(IORegistryEntry *client, uint32_t offset, uint64_t *bus) APPLE_KEXT_OVERRIDE;
	virtual int hwFit(Channel &c, const PIEDMARequest *r) APPLE_KEXT_OVERRIDE;
	virtual IOReturn hwTransfer(Channel &c, int direction, uint64_t src, uint64_t dst, uint32_t length) APPLE_KEXT_OVERRIDE;
	virtual bool hwAbort(Channel &c) APPLE_KEXT_OVERRIDE;
	virtual void hwFilter(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual void hwService(unsigned line) APPLE_KEXT_OVERRIDE;

private:
	bool fLegacyNode;               /* brcm,bcm2835-dma: the BCM2711's /soc view */
	bool fBCM2712;
	unsigned fLine[kDMAMaxChannels];
	bool fHasLine[kDMAMaxChannels];   /* fLine is one of the added interrupts */
	uint32_t fLineMask[kDMAMaxInterrupts];
	_Atomic uint32_t fPending;

	static uint32_t rd(Channel &c, unsigned off) { return *(volatile uint32_t *)(c.regs + off); }
	static void wr(Channel &c, unsigned off, uint32_t v) { *(volatile uint32_t *)(c.regs + off) = v; }
	static void cbWrite(Channel &c, unsigned block, unsigned word, uint32_t v)
	{
		*(volatile uint32_t *)(c.cb + block * CB_BYTES + 4 * word) = v;
	}
	uint32_t perBlock(Channel &c) { return (c.flags & FLAG_LITE) ? LITE_MAX : NORMAL_MAX; }
	bool ended(Channel &c, bool *error);
};

OSDefineMetaClassAndStructors(PIEDMABCM2835, PIEDMAController);

static bool
dtHas(IORegistryEntry *e, const char *key, const char *value)
{
	OSData *d = OSDynamicCast(OSData, e->getProperty(key));
	if (d == NULL) {
		return false;
	}
	const char *p = (const char *)d->getBytesNoCopy(), *end = p + d->getLength();
	for (; p < end; p += strnlen(p, (size_t)(end - p)) + 1) {
		if (strncmp(p, value, (size_t)(end - p)) == 0) {
			return true;
		}
	}
	return false;
}

/* The index of "dma<n>" in the node's interrupt-names, else -1. */
static int
interruptIndex(IORegistryEntry *e, unsigned n)
{
	OSData *d = OSDynamicCast(OSData, e->getProperty("interrupt-names"));
	if (d == NULL) {
		return -1;
	}
	char want[8];
	snprintf(want, sizeof(want), "dma%u", n);
	const char *p = (const char *)d->getBytesNoCopy(), *end = p + d->getLength();
	for (int i = 0; p < end; p += strnlen(p, (size_t)(end - p)) + 1, i++) {
		if (strncmp(p, want, (size_t)(end - p)) == 0) {
			return i;
		}
	}
	return -1;
}

bool
PIEDMABCM2835::hwStart(IOService *provider)
{
	fLegacyNode = dtHas(provider, "compatible", "brcm,bcm2835-dma");
	fBCM2712 = dtHas(provider, "compatible", "brcm,bcm2712-dma");
	bool dma4Node = dtHas(provider, "compatible", "brcm,bcm2711-dma");
	if (!fLegacyNode && !fBCM2712 && !dma4Node) {
		return false;
	}
	if (!fLegacyNode && isQEMU()) {
		IOLog("PIEDMA: %s: QEMU (its firmware's revision) models no 40-bit channels; declined\n", fName);
		return false;
	}
	OSData *maskData = OSDynamicCast(OSData, provider->getProperty("brcm,dma-channel-mask"));
	uint32_t mask = maskData != NULL && maskData->getLength() >= 4 ? OSReadBigInt32(maskData->getBytesNoCopy(), 0) : 0;
	unsigned first = (unsigned)(fPhys >> 8) & 0xf;
	unsigned count = (unsigned)(fMap->getLength() / 0x100);
	if (count > kDMAMaxChannels) {
		count = kDMAMaxChannels;
	}
	if (first + count > 15) {
		count = 15 - first;
	}
	fCount = count;
	bool lines[kDMAMaxInterrupts] = { false };
	for (unsigned i = 0; i < count; i++) {
		Channel &c = fChannels[i];
		c.number = first + i;
		c.regs = fRegs + 0x100 * i;
		c.usable = (mask >> c.number) & 1;
		if (dma4Node || (fBCM2712 && c.number >= 6)) {
			c.kind = KIND_DMA4;
		} else {
			c.kind = fBCM2712 ? KIND_2712 : KIND_LEGACY;
			c.flags = c.usable && (rd(c, DEBUG) & DEBUG_LITE) ? FLAG_LITE : 0;
		}
		if (!c.usable) {
			continue;
		}
		int index = interruptIndex(provider, c.number);
		fLine[i] = index >= 0 ? (unsigned)index : i;
		if (fLine[i] >= kDMAMaxInterrupts) {
			c.usable = false;
			continue;
		}
		if (!lines[fLine[i]]) {
			if (!addInterrupt(provider, fLine[i], fLine[i])) {
				IOLog("PIEDMA: %s: channel %u has no interrupt\n", fName, c.number);
				c.usable = false;
				continue;
			}
			lines[fLine[i]] = true;
		}
		fHasLine[i] = true;
		fLineMask[fLine[i]] |= 1U << i;
	}
	return true;
}

uint64_t
PIEDMABCM2835::hwMaxAddress(void)
{
	/* The legacy view: the low GB; DMA4: 36 bits on the BCM2711, 40 on the BCM2712. */
	return fLegacyNode ? 0x3fffffffULL : fBCM2712 ? 0xffffffffffULL : 0xfffffffffULL;
}

uint32_t
PIEDMABCM2835::hwMaxLength(Channel &c)
{
	uint64_t max = (uint64_t)perBlock(c) * (hwCBBytes() / CB_BYTES);
	return max > kMaxTransfer ? kMaxTransfer : (uint32_t)max;
}

bool
PIEDMABCM2835::hwMemoryAddress(uint64_t pa, uint64_t *bus)
{
	if (fLegacyNode) {
		/* /soc's dma-ranges: the low GB of RAM at 0xc0000000. */
		if (pa >= 0x40000000ULL) {
			return false;
		}
		*bus = LEGACY_RAM_BUS + pa;
		return true;
	}
	*bus = pa;
	return true;
}

bool
PIEDMABCM2835::hwDeviceAddress(IORegistryEntry *client, uint32_t offset, uint64_t *bus)
{
	IOService *nub = OSDynamicCast(IOService, client);
	IODeviceMemory *mem = nub != NULL ? nub->getDeviceMemoryWithIndex(0) : NULL;
	if (mem == NULL) {
		return false;
	}
	uint64_t pa = mem->getPhysicalAddress() + offset;
	if (fBCM2712) {
		*bus = pa;
		return true;
	}
	if (pa < PERIPH_CPU || pa >= PERIPH_END) {
		return false;
	}
	*bus = fLegacyNode ? pa - PERIPH_CPU + PERIPH_LEGACY_BUS : pa - PERIPH_CPU + BCM2711_PERIPH_DMA4_BUS;
	return true;
}

/* 32-bit registers only; a full channel before a LITE one. */
int
PIEDMABCM2835::hwFit(Channel &c, const PIEDMARequest *r)
{
	if (r->width != 4) {
		return -1;
	}
	return (c.flags & FLAG_LITE) ? 1 : 2;
}

IOReturn
PIEDMABCM2835::hwTransfer(Channel &c, int direction, uint64_t src, uint64_t dst, uint32_t length)
{
	uint32_t per = perBlock(c), blocks = (length + per - 1) / per;
	if (blocks == 0 || blocks > hwCBBytes() / CB_BYTES) {
		return kIOReturnBadArgument;
	}
	bool srcInc = direction != kDMADevToMem, dstInc = direction != kDMAMemToDev;
	uint32_t dreq = c.cell & 0x1f;
	for (uint32_t b = 0; b < blocks; b++) {
		uint64_t off = (uint64_t)b * per;
		uint32_t len = length - (uint32_t)off < per ? length - (uint32_t)off : per;
		uint64_t s = src + (srcInc ? off : 0), d = dst + (dstInc ? off : 0);
		uint64_t next = b + 1 < blocks ? c.cbBus + (b + 1) * CB_BYTES : 0;
		bool last = b + 1 == blocks;
		if (c.kind == KIND_DMA4) {
			uint32_t ti = D4_TI_WAIT_RESP | (last ? D4_TI_INTEN : 0);
			if (direction == kDMAMemToDev) {
				ti |= D4_TI_D_DREQ | dreq << D4_TI_PERMAP_SHIFT;
			} else if (direction == kDMADevToMem) {
				ti |= D4_TI_S_DREQ | D4_TI_WAIT_RD_RESP | dreq << D4_TI_PERMAP_SHIFT;
			}
			cbWrite(c, b, 0, ti);
			cbWrite(c, b, 1, (uint32_t)s);
			cbWrite(c, b, 2, ((uint32_t)(s >> 32) & 0xff) | (srcInc ? D4_INFO_INC : 0));
			cbWrite(c, b, 3, (uint32_t)d);
			cbWrite(c, b, 4, ((uint32_t)(d >> 32) & 0xff) | (dstInc ? D4_INFO_INC : 0));
			cbWrite(c, b, 5, len);
			cbWrite(c, b, 6, (uint32_t)(next >> 5));
			cbWrite(c, b, 7, 0);
		} else {
			uint32_t ti = TI_WAIT_RESP | (last ? TI_INTEN : 0) | (srcInc ? TI_SRC_INC : 0) | (dstInc ? TI_DEST_INC : 0);
			if (direction == kDMAMemToDev) {
				ti |= TI_DEST_DREQ | dreq << TI_PERMAP_SHIFT;
			} else if (direction == kDMADevToMem) {
				ti |= TI_SRC_DREQ | dreq << TI_PERMAP_SHIFT;
			}
			bool shifted = c.kind == KIND_2712;
			cbWrite(c, b, 0, ti);
			cbWrite(c, b, 1, (uint32_t)s);
			cbWrite(c, b, 2, (uint32_t)d);
			cbWrite(c, b, 3, len);
			cbWrite(c, b, 4, shifted ? ((uint32_t)(s >> 32) & 0xff) | ((uint32_t)(d >> 32) & 0xff) << 8 : 0);
			cbWrite(c, b, 5, shifted ? (uint32_t)(next >> 5) : (uint32_t)next);
			cbWrite(c, b, 6, 0);
			cbWrite(c, b, 7, 0);
		}
	}
	/* No old acknowledgement or queued completion may modify this start. */
	if (!hwAbort(c)) { return kIOReturnNotResponding; }
	__builtin_arm_dsb(0xf);
	IOInterruptState irq = IOSimpleLockLockDisableInterrupt(fHardwareLock);
	__c11_atomic_fetch_and(&fPending, ~(1U << (unsigned)(&c - fChannels)), __ATOMIC_RELAXED);
	if (c.kind == KIND_DMA4) {
		wr(c, D4_CB, (uint32_t)(c.cbBus >> 5));
		wr(c, D4_CS, CS_ACTIVE | D4_CS_PROT | CS_WAIT_FOR_WRITES);
	} else {
		wr(c, CONBLK_AD, c.kind == KIND_2712 ? (uint32_t)(c.cbBus >> 5) : (uint32_t)c.cbBus);
		wr(c, CS, CS_ACTIVE | CS_WAIT_FOR_WRITES);
	}
	IOSimpleLockUnlockEnableInterrupt(fHardwareLock, irq);
	return kIOReturnSuccess;
}

bool
PIEDMABCM2835::hwAbort(Channel &c)
{
	IOInterruptState irq = IOSimpleLockLockDisableInterrupt(fHardwareLock);
	bool idle = false;
	if (c.kind == KIND_DMA4) {
		wr(c, D4_CS, D4_CS_PROT);
		for (unsigned i = 0; i < 1000 && (rd(c, D4_CS) & D4_CS_TRANSACTIONS); i++) { IODelay(1); }
		wr(c, D4_DEBUG, D4_DEBUG_RESET);
		for (unsigned i = 0; i < 1000; i++) {
			if (!(rd(c, D4_CS) & (CS_ACTIVE | D4_CS_TRANSACTIONS)) && !(rd(c, D4_DEBUG) & D4_DEBUG_RESET)) {
				idle = true; break;
			}
			IODelay(1);
		}
	} else {
		if (rd(c, CONBLK_AD) != 0) {
			wr(c, NEXTCONBK, 0);
			wr(c, CS, CS_ABORT | CS_ACTIVE);
			for (unsigned i = 0; i < 100 && (rd(c, CS) & CS_ABORT); i++) { IODelay(1); }
		}
		wr(c, CS, CS_RESET);
		for (unsigned i = 0; i < 1000; i++) {
			if (!(rd(c, CS) & (CS_ACTIVE | CS_ABORT | CS_RESET | CS_WRITES_PENDING))) { idle = true; break; }
			IODelay(1);
		}
	}
	__c11_atomic_fetch_and(&fPending, ~(1U << (unsigned)(&c - fChannels)), __ATOMIC_RELAXED);
	__builtin_arm_dsb(0xf);
	IOSimpleLockUnlockEnableInterrupt(fHardwareLock, irq);
	return idle;
}

/* Primary interrupt context: each of the line's channels' INT cleared (ACTIVE kept), noted. */
void
PIEDMABCM2835::hwFilter(unsigned line)
{
	IOInterruptState irq = IOSimpleLockLockDisableInterrupt(fHardwareLock);
	for (unsigned i = 0; i < fCount; i++) {
		Channel &c = fChannels[i];
		/* Every channel on the line, usable or not: an INT left set would raise it for ever. */
		if (!fHasLine[i] || fLine[i] != line) {
			continue;
		}
		uint32_t cs = rd(c, CS);
		if (cs & CS_INT) {
			wr(c, CS, CS_INT | (cs & CS_ACTIVE) | (c.kind == KIND_DMA4 ? D4_CS_PROT : 0));
			__c11_atomic_fetch_or(&fPending, 1U << i, __ATOMIC_RELAXED);
		}
	}
	IOSimpleLockUnlockEnableInterrupt(fHardwareLock, irq);
}

/* Whether a running channel's transfer has ended (its block address 0), or failed. */
bool
PIEDMABCM2835::ended(Channel &c, bool *error)
{
	uint32_t cs = rd(c, CS);
	if (c.kind == KIND_DMA4) {
		*error = (cs & D4_CS_ERR) != 0;
		return *error || rd(c, D4_CB) == 0;
	}
	*error = (cs & CS_ERROR) != 0;
	return *error || rd(c, CONBLK_AD) == 0;
}

void
PIEDMABCM2835::hwService(unsigned line)
{
	/* This line's channels' bits only: another line's action may not have run yet. */
	uint32_t mask = line < kDMAMaxInterrupts ? fLineMask[line] : 0;
	uint32_t pending = __c11_atomic_fetch_and(&fPending, ~mask, __ATOMIC_RELAXED) & mask;
	for (unsigned i = 0; i < fCount; i++) {
		Channel &c = fChannels[i];
		if (!c.running || !(pending & (1U << i))) {
			continue;
		}
		/* The last block's interrupt comes as its writes are acknowledged: the address is 0 by now, or soon. */
		bool error = false, done = false;
		for (int t = 0; t < 1000 && !(done = ended(c, &error)); t++) {
			IODelay(1);
		}
		if (error) {
			IOLog("PIEDMA: %s: channel %u: error (CS 0x%08x, DEBUG 0x%08x)\n", fName, c.number, rd(c, CS),
			    rd(c, c.kind == KIND_DMA4 ? D4_DEBUG : DEBUG));
			complete(c, kIOReturnIOError);
		} else if (done) {
			complete(c, kIOReturnSuccess);
		}
	}
}
