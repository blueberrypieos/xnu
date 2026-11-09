/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PieGPIOBrcmstb: the BCM2712's own GPIO blocks (brcm,brcmstb-gpio, the
 * Broadcom set-top box GIO), whose banks of up to 32 lines
 * (brcm,gpio-bank-widths) are 0x20 bytes apart: open-drain enable (ODEN),
 * data, direction (IODIR, 1: input), edge configuration (EC: rising or
 * high; EI: both edges), interrupt mask, level (or edge) and status
 * (written 1 to clear), one interrupt for the node's banks. No pulls (the
 * pin controller's): a bias is left as it is, as Linux's optional bias
 * setting leaves it. Each bank is a chip, as in Linux: a PieGPIOBrcmstbBank,
 * gpiochip10 and on in device tree order (IOGPIOChipBase), labelled
 * gpio-brcmstb@<the bank's registers' physical address>. The banks share
 * this driver's work loop and interrupt.
 *
 * A line the kernel drives (M2_PWR_EN, the M.2 slot's power, RPiPCIe.cpp)
 * is in the node's IOGPIOReservedLines: used, and not requested.
 *
 * Not yet run on a Raspberry Pi 5.
 */

#include <IOKit/IOLib.h>
#include <pie/rpi.h>
#include <libkern/OSByteOrder.h>

#include "PieGPIOController.h"

class PieGPIOBrcmstbBank;

class PieGPIOBrcmstb : public IOService
{
	OSDeclareDefaultStructors(PieGPIOBrcmstb);

	IOMemoryMap *fMap;
	IOWorkLoop *fWorkLoop;
	IOFilterInterruptEventSource *fInterrupt;
	OSArray *fBanks;
	_Atomic uint64_t fStamp;

	static bool filter(OSObject *owner, IOFilterInterruptEventSource *source);
	static void action(OSObject *owner, IOInterruptEventSource *source, int count);
	unsigned firstBankIndex(IOService *provider);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE { return fWorkLoop; }

	volatile uint32_t *regs;
	bool hasInterrupt;
};

class PieGPIOBrcmstbBank : public PieGPIOController
{
	OSDeclareDefaultStructors(PieGPIOBrcmstbBank);

	PieGPIOBrcmstb *fParent;
	IOService *fNode;
	unsigned fBank, fFirst, fChip;
	char fLabel[32];

	volatile uint32_t *reg(unsigned off) { return fParent->regs + (0x20 * fBank + off) / 4; }
	void setBit(unsigned off, unsigned line, bool on)
	{
		*reg(off) = on ? *reg(off) | (1U << line) : *reg(off) & ~(1U << line);
	}

protected:
	virtual bool hwStart(IOService *provider) APPLE_KEXT_OVERRIDE { (void)provider; return true; }
	virtual void hwSetOutput(unsigned line, bool output) APPLE_KEXT_OVERRIDE;
	virtual bool hwIsOutput(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual bool hwRead(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual void hwWrite(unsigned line, bool high) APPLE_KEXT_OVERRIDE;
	virtual bool hwSetOpenDrain(unsigned line, bool openDrain) APPLE_KEXT_OVERRIDE;
	virtual void hwSetEdges(unsigned line, bool rising, bool falling) APPLE_KEXT_OVERRIDE;
	virtual bool hwCanDetectEdges(void) APPLE_KEXT_OVERRIDE { return fParent->hasInterrupt; }
	virtual uint64_t hwTakeEdges(void) APPLE_KEXT_OVERRIDE;
	virtual IOService *dtNode(void) APPLE_KEXT_OVERRIDE { return fNode; }
	virtual unsigned chipNumber(void) APPLE_KEXT_OVERRIDE { return fChip; }
	virtual const char *chipLabel(void) APPLE_KEXT_OVERRIDE { return fLabel; }
	virtual unsigned lineNamesFirst(void) APPLE_KEXT_OVERRIDE { return fFirst; }

public:
	bool initBank(PieGPIOBrcmstb *parent, IOService *node, unsigned bank, unsigned first, unsigned width,
	    unsigned chip, uint64_t physical, OSObject *groupID);
};

OSDefineMetaClassAndStructors(PieGPIOBrcmstb, IOService);
OSDefineMetaClassAndStructors(PieGPIOBrcmstbBank, PieGPIOController);

enum { ODEN = 0x00, DATA = 0x04, IODIR = 0x08, EC = 0x0c, EI = 0x10, MASK = 0x14, LEVEL = 0x18, STAT = 0x1c };

/* The node's first bank's index among all brcmstb GPIO banks, in device tree order. */
unsigned
PieGPIOBrcmstb::firstBankIndex(IOService *provider)
{
	OSArray *nodes = RPiDTFindCompatible("brcm,brcmstb-gpio");
	unsigned index = 0;
	for (unsigned n = 0; nodes != NULL && n < nodes->getCount(); n++) {
		IORegistryEntry *node = OSDynamicCast(IORegistryEntry, nodes->getObject(n));
		if (node == provider) {
			break;
		}
		OSData *widths = node != NULL ? OSDynamicCast(OSData, node->getProperty("brcm,gpio-bank-widths")) : NULL;
		index += widths != NULL ? widths->getLength() / sizeof(uint32_t) : 1;
	}
	OSSafeReleaseNULL(nodes);
	return index;
}

bool
PieGPIOBrcmstb::start(IOService *provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	fMap = provider->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache);
	fWorkLoop = IOWorkLoop::workLoop();
	fBanks = OSArray::withCapacity(2);
	if (fMap == NULL || fWorkLoop == NULL || fBanks == NULL) {
		return false;
	}
	regs = (volatile uint32_t *)fMap->getVirtualAddress();
	fInterrupt = IOFilterInterruptEventSource::filterInterruptEventSource(this, action, filter, provider, 0);
	if (fInterrupt != NULL && fWorkLoop->addEventSource(fInterrupt) == kIOReturnSuccess) {
		hasInterrupt = true;
	} else {
		OSSafeReleaseNULL(fInterrupt);
	}

	OSData *widths = OSDynamicCast(OSData, provider->getProperty("brcm,gpio-bank-widths"));
	unsigned banks = widths != NULL ? widths->getLength() / sizeof(uint32_t) : 1;
	OSNumber *base = OSDynamicCast(OSNumber, getProperty("IOGPIOChipBase"));
	unsigned chip = (base != NULL ? base->unsigned32BitValue() : 10) + firstBankIndex(provider);
	uint64_t physical = fMap->getPhysicalAddress();
	for (unsigned b = 0, first = 0; b < banks; b++) {
		unsigned width = widths != NULL ? OSReadBigInt32(widths->getBytesNoCopy(), 4 * b) : 32;
		/* Nothing armed from before. */
		regs[(0x20 * b + MASK) / 4] = 0;
		regs[(0x20 * b + STAT) / 4] = ~0U;
		PieGPIOBrcmstbBank *bank = OSTypeAlloc(PieGPIOBrcmstbBank);
		if (bank != NULL && bank->initBank(this, provider, b, first, width, chip + b, physical + 0x20 * b,
		    getProperty("GPIOGroupID")) && bank->attach(this)) {
			if (bank->start(this)) {
				fBanks->setObject(bank);
			} else {
				bank->detach(this);
			}
		}
		OSSafeReleaseNULL(bank);
		first += width;
	}
	if (fInterrupt != NULL) {
		fInterrupt->enable();
	}
	registerService();
	return true;
}

void
PieGPIOBrcmstb::stop(IOService *provider)
{
	if (fInterrupt != NULL) {
		fInterrupt->disable();
		fWorkLoop->removeEventSource(fInterrupt);
	}
	IOService::stop(provider);
}

void
PieGPIOBrcmstb::free(void)
{
	OSSafeReleaseNULL(fInterrupt);
	OSSafeReleaseNULL(fBanks);
	OSSafeReleaseNULL(fWorkLoop);
	OSSafeReleaseNULL(fMap);
	IOService::free();
}

bool
PieGPIOBrcmstb::filter(OSObject *owner, IOFilterInterruptEventSource *)
{
	PieGPIOBrcmstb *self = (PieGPIOBrcmstb *)owner;
	uint64_t none = 0;
	__c11_atomic_compare_exchange_strong(&self->fStamp, &none, mach_continuous_time(),
	    __ATOMIC_RELAXED, __ATOMIC_RELAXED);
	return true;
}

/* The node's interrupt: each bank's edges, stamped as PieGPIOController's own are. */
void
PieGPIOBrcmstb::action(OSObject *owner, IOInterruptEventSource *, int)
{
	PieGPIOBrcmstb *self = (PieGPIOBrcmstb *)owner;
	uint64_t at = __c11_atomic_exchange(&self->fStamp, 0, __ATOMIC_RELAXED);
	uint64_t now = mach_continuous_time();
	if (at == 0 || at > now) {
		at = now;
	}
	uint64_t atNs, nowNs;
	absolutetime_to_nanoseconds(at, &atNs);
	absolutetime_to_nanoseconds(now, &nowNs);
	clock_sec_t s;
	clock_nsec_t ns;
	clock_get_calendar_nanotime(&s, &ns);
	uint64_t real = (uint64_t)s * NSEC_PER_SEC + ns - (nowNs - atNs);
	for (unsigned b = 0; b < self->fBanks->getCount(); b++) {
		((PieGPIOBrcmstbBank *)self->fBanks->getObject(b))->handleInterrupt(atNs, real);
	}
}

bool
PieGPIOBrcmstbBank::initBank(PieGPIOBrcmstb *parent, IOService *node, unsigned bank, unsigned first,
    unsigned width, unsigned chip, uint64_t physical, OSObject *groupID)
{
	if (!init() || width == 0 || width > 32) {
		return false;
	}
	fParent = parent;
	fNode = node;
	fBank = bank;
	fFirst = first;
	fChip = chip;
	fLineCount = width;
	snprintf(fLabel, sizeof(fLabel), "gpio-brcmstb@%llx", physical);
	fWorkLoop = parent->getWorkLoop();
	fWorkLoop->retain();
	if (groupID != NULL) {
		setProperty("GPIOGroupID", groupID);
	}
	return true;
}

void
PieGPIOBrcmstbBank::hwSetOutput(unsigned line, bool output)
{
	setBit(IODIR, line, !output);
}

bool
PieGPIOBrcmstbBank::hwIsOutput(unsigned line)
{
	return !(*reg(IODIR) & (1U << line));
}

bool
PieGPIOBrcmstbBank::hwRead(unsigned line)
{
	return (*reg(DATA) >> line) & 1;
}

void
PieGPIOBrcmstbBank::hwWrite(unsigned line, bool high)
{
	setBit(DATA, line, high);
}

bool
PieGPIOBrcmstbBank::hwSetOpenDrain(unsigned line, bool openDrain)
{
	setBit(ODEN, line, openDrain);
	return true;
}

void
PieGPIOBrcmstbBank::hwSetEdges(unsigned line, bool rising, bool falling)
{
	setBit(MASK, line, false);
	setBit(LEVEL, line, false);
	setBit(EI, line, rising && falling);
	setBit(EC, line, rising && !falling);
	*reg(STAT) = 1U << line;
	setBit(MASK, line, rising || falling);
}

uint64_t
PieGPIOBrcmstbBank::hwTakeEdges(void)
{
	uint32_t st = *reg(STAT) & *reg(MASK);
	*reg(STAT) = st;
	return st;
}
