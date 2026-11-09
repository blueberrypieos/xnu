/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PIEDMADWAxi: RP1's DMA controller (dma@188000, snps,axi-dma-1.01a), a
 * Synopsys DesignWare AXI DMAC with dma-channels (8) channels, one
 * interrupt, a 128-bit master (snps,data-width 4) and up to
 * snps,block-size items a block.
 *
 * Common registers: CFG 0x010 (bit 0 DMAC_EN, bit 1 INT_EN), CHEN 0x018
 * (8 channels or fewer: bits 7:0 enable, 15:8 their write enables),
 * RESET 0x058. Channel i's at
 * 0x100 + 0x100 * i: CFG 0x20 (64-bit), LLP 0x28, INTSTATUS_ENA 0x80,
 * INTSTATUS 0x88, INTSIGNAL_ENA 0x90, INTCLEAR 0x98. A transfer is a chain
 * of 64-byte linked-list items (SAR, DAR, BLOCK_TS (source items - 1),
 * LLP, CTL), each VALID, the last LAST; the chain's end is DMA_TFR (bit 1)
 * in the channel's INTSTATUS, an error one of bits 5-14 and 16-21. With
 * more than 16 handshake targets (snps,dma-targets: RP1's 64) CFG's layout
 * is the second one: the low word SRC_PER from bit 4 and DST_PER from bit
 * 11 under the multi-block types, the high word TT_FC (0 memory to memory,
 * 1 to a device, 2 from one), the handshake selects (0: hardware) and
 * PRIORITY from bit 20; else the first: all but the multi-block types in
 * the high word, SRC_PER from bit 7, DST_PER from 12, PRIORITY from 17.
 *
 * RP1 sees RAM and peripherals through the nub's native pieboot,dma-ranges.
 * DMA uses the 40-bit system view (0xc040000000), not the management
 * processors' 32-bit view (0x40000000); see RP1 Peripherals 2.2 and 2.3.1.
 * It is driven only if its ID and version registers read
 * as a controller's (neither 0 nor all ones). A dmas cell's low 8 bits are the
 * handshake; its higher bits, if any, pick a channel whose snps,chan-flags
 * match. RP1's DMA clocks are left to the firmware, which runs them.
 */

#include <IOKit/IOLib.h>
#include <libkern/OSByteOrder.h>
#include <string.h>

#include <pie/rpi.h>

#include "DMAController.h"

enum {
	DMAC_ID = 0x000, DMAC_COMPVER = 0x008, DMAC_CFG = 0x010, CHEN = 0x018, RESET = 0x058,
	RP1_DMAC_OFFSET = 0x188000, RP1_BAR_ALIGN = 0x400000,
	RP1_PERIPHERALS_SIZE = 0x410000,      /* rp1's ranges: 0xc0_40000000 to its BAR, this long */
	CFG_DMAC_EN = 1U << 0, CFG_INT_EN = 1U << 1,
	CH_BASE = 0x100, CH_CFG = 0x20, CH_LLP = 0x28, CH_INTSTATUS_ENA = 0x80, CH_INTSTATUS = 0x88,
	CH_INTSIGNAL_ENA = 0x90, CH_INTCLEAR = 0x98,
	INT_DMA_TFR = 1U << 1,
	INT_ERRORS = (0x3ffU << 5) | (0x3fU << 16),         /* bits 5-14, 16-21 */
	CTL_SINC_FIXED = 1U << 4, CTL_DINC_FIXED = 1U << 6, CTL_SRC_WIDTH = 8, CTL_DST_WIDTH = 11,
	CTL_SRC_MSIZE = 14, CTL_DST_MSIZE = 18,
	CTLH_ARLEN_EN = 1U << 6, CTLH_ARLEN = 7, CTLH_AWLEN_EN = 1U << 15, CTLH_AWLEN = 16,
	CTLH_LAST = 1U << 30, CTLH_VALID = 1U << 31,
	MULTBLK_LL = 3,
	LLI_BYTES = 64,
	MAX_CHANNELS = 8,
};
#define kMaxTransfer (16U * 1024 * 1024)

class PIEDMADWAxi : public PIEDMAController
{
	OSDeclareDefaultStructors(PIEDMADWAxi);

protected:
	virtual bool hwStart(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual uint64_t hwMaxAddress(void) APPLE_KEXT_OVERRIDE { return 0xfffffffffULL; }
	virtual uint32_t hwMaxLength(Channel &c) APPLE_KEXT_OVERRIDE;
	virtual bool hwMemoryAddress(uint64_t pa, uint64_t *bus) APPLE_KEXT_OVERRIDE;
	virtual bool hwDeviceAddress(IORegistryEntry *client, uint32_t offset, uint64_t *bus) APPLE_KEXT_OVERRIDE;
	virtual int hwFit(Channel &c, const PIEDMARequest *r) APPLE_KEXT_OVERRIDE;
	virtual IOReturn hwTransfer(Channel &c, int direction, uint64_t src, uint64_t dst, uint32_t length) APPLE_KEXT_OVERRIDE;
	virtual bool hwAbort(Channel &c) APPLE_KEXT_OVERRIDE;
	virtual void hwFilter(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual void hwService(unsigned line) APPLE_KEXT_OVERRIDE;

private:
	IOService *fNub;
	bool fCFG2;
	uint32_t fBlockSize[MAX_CHANNELS], fPriority[MAX_CHANNELS], fMaxBurst[MAX_CHANNELS];

	uint32_t rd(unsigned off) { return *(volatile uint32_t *)(fRegs + off); }
	void wr(unsigned off, uint32_t v) { *(volatile uint32_t *)(fRegs + off) = v; }
	static uint32_t crd(Channel &c, unsigned off) { return *(volatile uint32_t *)(c.regs + off); }
	static void cwr(Channel &c, unsigned off, uint32_t v) { *(volatile uint32_t *)(c.regs + off) = v; }
	static void cwr64(Channel &c, unsigned off, uint64_t v)
	{
		cwr(c, off, (uint32_t)v);
		cwr(c, off + 4, (uint32_t)(v >> 32));
	}
	static void lli(Channel &c, unsigned item, unsigned off, uint32_t v)
	{
		*(volatile uint32_t *)(c.cb + item * LLI_BYTES + off) = v;
	}
	unsigned index(Channel &c) { return (unsigned)(&c - fChannels); }
	bool disable(Channel &c);
};

OSDefineMetaClassAndStructors(PIEDMADWAxi, PIEDMAController);

/* A u32 property's cell i (its last if it has fewer), or def. */
static uint32_t
dtCell(IORegistryEntry *e, const char *key, unsigned i, uint32_t def)
{
	OSData *d = OSDynamicCast(OSData, e->getProperty(key));
	unsigned n = d != NULL ? d->getLength() / 4 : 0;
	if (n == 0) {
		return def;
	}
	return OSReadBigInt32(d->getBytesNoCopy(), 4 * (i < n ? i : n - 1));
}

/* MSIZE's encoding of a burst of n items: 1, 4, 8, 16, ... 1024. */
static uint32_t
msize(uint32_t n)
{
	uint32_t e = 0;
	for (uint32_t items = 4; e < 9 && items <= n; items <<= 1) {
		e++;
	}
	return e;
}

bool
PIEDMADWAxi::hwStart(IOService *provider)
{
	fNub = provider;
	unsigned count = dtCell(provider, "dma-channels", 0, 0);
	if (count == 0 || count > MAX_CHANNELS || fMap->getLength() < CH_BASE + 0x100 * count) {
		IOLog("PIEDMA: %s: %u channels: only 1-%u are driven\n", fName, count, MAX_CHANNELS);
		return false;
	}
	uint32_t id = rd(DMAC_ID), version = rd(DMAC_COMPVER);
	if (id == 0 || id == 0xffffffff || version == 0 || version == 0xffffffff ||
	    (fPhys & (RP1_BAR_ALIGN - 1)) != RP1_DMAC_OFFSET) {
		IOLog("PIEDMA: %s: not RP1's DMAC as expected (ID 0x%x, version 0x%x, at 0x%llx): left alone\n", fName,
		    id, version, fPhys);
		return false;
	}
	fCFG2 = dtCell(provider, "snps,dma-targets", 0, 16) > 16;
	/* Reset, then enabled with its interrupt; every channel off and quiet. */
	wr(RESET, 1);
	for (int i = 0; i < 1000 && (rd(RESET) & 1); i++) {
		IODelay(1);
	}
	if (rd(RESET) & 1) {
		IOLog("PIEDMA: %s: the reset does not end\n", fName);
		return false;
	}
	fCount = count;
	for (unsigned i = 0; i < count; i++) {
		Channel &c = fChannels[i];
		c.number = i;
		c.regs = fRegs + CH_BASE + 0x100 * i;
		c.usable = true;
		c.flags = dtCell(provider, "snps,chan-flags", i, 0);
		fBlockSize[i] = dtCell(provider, "snps,block-size", i, 0x40000);
		fPriority[i] = dtCell(provider, "snps,priority", i, i);
		fMaxBurst[i] = dtCell(provider, "snps,axi-max-burst-len", i, 0);
		cwr(c, CH_INTSTATUS_ENA, 0);
		cwr(c, CH_INTSIGNAL_ENA, 0);
		cwr(c, CH_INTCLEAR, 0xffffffff);
	}
	wr(CHEN, ((1U << count) - 1) << 8);
	wr(DMAC_CFG, CFG_DMAC_EN | CFG_INT_EN);
	return addInterrupt(provider, 0, 0);
}

uint32_t
PIEDMADWAxi::hwMaxLength(Channel &c)
{
	/* A device's items are its register's width; a copy's at least a byte. */
	uint64_t max = (uint64_t)fBlockSize[index(c)] * (c.width != 0 ? c.width : 1) * (hwCBBytes() / LLI_BYTES);
	return max > kMaxTransfer ? kMaxTransfer : (uint32_t)max;
}

bool
PIEDMADWAxi::hwMemoryAddress(uint64_t pa, uint64_t *bus)
{
	return RPiDMAAddress(fNub, pa, bus);
}

/* Translate a bounded register through RP1's 40-bit system address view. */
bool
PIEDMADWAxi::hwDeviceAddress(IORegistryEntry *client, uint32_t offset, uint64_t *bus)
{
	IOService *nub = OSDynamicCast(IOService, client);
	IODeviceMemory *mem = nub != NULL ? nub->getDeviceMemoryWithIndex(0) : NULL;
	OSData *ranges = OSDynamicCast(OSData, fNub->getProperty("pieboot,dma-ranges"));
	uint64_t base = fPhys - RP1_DMAC_OFFSET;
	if (mem == NULL || offset >= mem->getLength() || mem->getPhysicalAddress() < base ||
	    mem->getPhysicalAddress() - base >= RP1_PERIPHERALS_SIZE ||
	    offset >= RP1_PERIPHERALS_SIZE - (mem->getPhysicalAddress() - base) ||
	    ranges == NULL || ranges->getLength() == 0 || ranges->getLength() % 24 != 0) {
		return false;
	}
	if (!RPiDMAAddress(fNub, mem->getPhysicalAddress() + offset, bus)) { return false; }
	/* Missing or inappropriate ranges must not fall back to a CPU/32-bit address. */
	return *bus >= 0xc040000000ULL && *bus - 0xc040000000ULL < RP1_PERIPHERALS_SIZE;
}

/* A channel whose chan-flags are the cell's high bits; any other, if none is free. */
int
PIEDMADWAxi::hwFit(Channel &c, const PIEDMARequest *r)
{
	return (c.flags & ~0xffU) == (r->cell & ~0xffU) ? 2 : 1;
}

IOReturn
PIEDMADWAxi::hwTransfer(Channel &c, int direction, uint64_t src, uint64_t dst, uint32_t length)
{
	unsigned i = index(c);
	/* Items: the device's register's width; a copy's as wide as both addresses and the length allow. */
	uint32_t width = c.width;
	if (direction == kDMAMemToMem) {
		uint64_t a = 16 | src | dst | length;
		width = (uint32_t)(a & -a);
	}
	if (width == 0 || length % width != 0) {
		return kIOReturnBadArgument;
	}
	uint32_t log2w = (uint32_t)__builtin_ctz(width);
	uint64_t perItem = (uint64_t)fBlockSize[i] * width;
	uint32_t items = (uint32_t)((length + perItem - 1) / perItem);
	if (items == 0 || items > hwCBBytes() / LLI_BYTES) {
		return kIOReturnBadArgument;
	}
	bool srcInc = direction != kDMADevToMem, dstInc = direction != kDMAMemToDev;
	uint32_t devBurst = msize(c.burst), memBurst = msize(16);
	uint32_t ctl = (srcInc ? 0 : CTL_SINC_FIXED) | (dstInc ? 0 : CTL_DINC_FIXED) | log2w << CTL_SRC_WIDTH |
	    log2w << CTL_DST_WIDTH | (direction == kDMADevToMem ? devBurst : memBurst) << CTL_SRC_MSIZE |
	    (direction == kDMAMemToDev ? devBurst : memBurst) << CTL_DST_MSIZE;
	uint32_t ctlh = CTLH_VALID;
	if (fMaxBurst[i] != 0) {
		/* AXI bursts of at most that many beats (the field holds beats - 1). */
		uint32_t beats = (fMaxBurst[i] - 1) & 0xff;
		ctlh |= CTLH_ARLEN_EN | beats << CTLH_ARLEN | CTLH_AWLEN_EN | beats << CTLH_AWLEN;
	}
	for (uint32_t k = 0; k < items; k++) {
		uint64_t off = k * perItem;
		uint32_t len = length - (uint32_t)off < perItem ? length - (uint32_t)off : (uint32_t)perItem;
		uint64_t s = src + (srcInc ? off : 0), d = dst + (dstInc ? off : 0);
		uint64_t next = k + 1 < items ? c.cbBus + (k + 1) * LLI_BYTES : 0;
		lli(c, k, 0x00, (uint32_t)s);
		lli(c, k, 0x04, (uint32_t)(s >> 32));
		lli(c, k, 0x08, (uint32_t)d);
		lli(c, k, 0x0c, (uint32_t)(d >> 32));
		lli(c, k, 0x10, (len >> log2w) - 1);
		lli(c, k, 0x14, 0);
		lli(c, k, 0x18, (uint32_t)next);
		lli(c, k, 0x1c, (uint32_t)(next >> 32));
		lli(c, k, 0x20, ctl);
		lli(c, k, 0x24, ctlh | (k + 1 == items ? CTLH_LAST : 0));
		for (unsigned w = 0x28; w < LLI_BYTES; w += 4) {
			lli(c, k, w, 0);
		}
	}
	uint32_t ttfc = direction == kDMAMemToDev ? 1 : direction == kDMADevToMem ? 2 : 0;
	uint32_t handshake = c.cell & 0xff;
	uint32_t srcPer = direction == kDMADevToMem ? handshake : 0, dstPer = direction == kDMAMemToDev ? handshake : 0;
	uint32_t cfgl = MULTBLK_LL | MULTBLK_LL << 2, cfgh;
	if (fCFG2) {
		cfgl |= srcPer << 4 | dstPer << 11;
		cfgh = ttfc | fPriority[i] << 20;
	} else {
		cfgh = ttfc | srcPer << 7 | dstPer << 12 | fPriority[i] << 17;
	}
	/* The items in memory before the channel reads them. */
	__builtin_arm_dsb(0xf);
	cwr(c, CH_CFG, cfgl);
	cwr(c, CH_CFG + 4, cfgh);
	cwr64(c, CH_LLP, c.cbBus);
	cwr(c, CH_INTCLEAR, 0xffffffff);
	cwr(c, CH_INTSTATUS_ENA, INT_DMA_TFR | INT_ERRORS);
	cwr(c, CH_INTSIGNAL_ENA, INT_DMA_TFR | INT_ERRORS);
	wr(CHEN, 1U << i | 1U << (i + 8));
	return kIOReturnSuccess;
}

/* The channel off (its write enable with a 0), waited for: 50 ms at most. */
bool
PIEDMADWAxi::disable(Channel &c)
{
	unsigned i = index(c);
	wr(CHEN, 1U << (i + 8));
	for (int t = 0; t < 50000 && (rd(CHEN) & (1U << i)); t++) {
		IODelay(1);
	}
	cwr(c, CH_INTSIGNAL_ENA, 0);
	cwr(c, CH_INTSTATUS_ENA, 0);
	cwr(c, CH_INTCLEAR, 0xffffffff);
	return !(rd(CHEN) & (1U << i));
}

bool
PIEDMADWAxi::hwAbort(Channel &c)
{
	cwr(c, CH_INTSIGNAL_ENA, 0);
	bool idle = disable(c);
	if (idle) { cwr(c, CH_INTCLEAR, 0xffffffff); }
	__builtin_arm_dsb(0xf);
	return idle;
}

/* Primary interrupt context: the controller's interrupt off until the action has looked. */
void
PIEDMADWAxi::hwFilter(unsigned)
{
	wr(DMAC_CFG, CFG_DMAC_EN);
}

void
PIEDMADWAxi::hwService(unsigned)
{
	/* Each channel's own status (the common INTSTATUS's summary is not relied on). */
	for (unsigned i = 0; i < fCount; i++) {
		Channel &c = fChannels[i];
		uint32_t st = crd(c, CH_INTSTATUS);
		if (st == 0) {
			continue;
		}
		cwr(c, CH_INTCLEAR, st);
		if (!c.running) {
			continue;
		}
		if (st & INT_ERRORS) {
			IOLog("PIEDMA: %s: channel %u: error (INTSTATUS 0x%08x)\n", fName, c.number, st);
			complete(c, kIOReturnIOError);
		} else if (st & INT_DMA_TFR) {
			complete(c, kIOReturnSuccess);
		}
	}
	wr(DMAC_CFG, CFG_DMAC_EN | CFG_INT_EN);
}
