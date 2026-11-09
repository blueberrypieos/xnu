/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/OSByteOrder.h>

#include "RPiPlatform.h"

#define RP1_VENDOR              0x1de4
#define RP1_DEVICE              0x0001
#define RP1_C0_CHIP_ID          0x20001927
#define RP1_IRQS                61
#define RP1_PCIE_APBS_BASE      0x108000
#define REG_SET                 0x800
#define REG_CLR                 0xc00
#define MSIX_CFG(x)             (0x8 + 4 * (x))
#define MSIX_CFG_IACK_EN        (1U << 3)
#define MSIX_CFG_IACK           (1U << 2)
#define MSIX_CFG_ENABLE         (1U << 0)

#define PCI_CAP_MSIX            0x11
#define MSIX_CTRL_ENABLE        (1U << 15)
#define MSIX_CTRL_FUNC_MASK     (1U << 14)
#define MSIX_ENTRY_SIZE         16
#define MSIX_ENTRY_CTRL_MASKED  1U

class RP1 : public IOInterruptController
{
	OSDeclareDefaultStructors(RP1);

	RPiPCIDevice *pci;
	IOMemoryMap *bar1Map;
	uintptr_t bar1;
	IOMemoryMap *tableMap;
	volatile uint32_t *msixTable;
	RPiMIP *mip;
	int firstVector;
	uint32_t numVectors;
	bool level[RP1_IRQS];

	void msixCfg(unsigned irq, uint32_t bits, bool set)
	{
		*(volatile uint32_t *)(bar1 + RP1_PCIE_APBS_BASE + (set ? REG_SET : REG_CLR) + MSIX_CFG(irq)) = bits;
	}
	void maskEntry(unsigned irq, bool masked)
	{
		msixTable[irq * MSIX_ENTRY_SIZE / 4 + 3] = masked ? MSIX_ENTRY_CTRL_MASKED : 0;
	}
	IORegistryEntry *findRP1Node(void);
	bool setUpMSIX(void);
	RPiMIP *findMIP(void);

	void msixInterrupt(void *refCon, IOService *nub, int source);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;

	virtual IOReturn registerInterrupt(IOService *nub, int source, void *target, IOInterruptHandler handler,
	    void *refCon) APPLE_KEXT_OVERRIDE;
	virtual bool vectorCanBeShared(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual int getVectorType(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void disableVectorHard(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void enableVector(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
};

#undef super
#define super IOInterruptController
OSDefineMetaClassAndStructors(RP1, IOInterruptController);

IORegistryEntry *
RP1::findRP1Node(void)
{
	for (IORegistryEntry *e = pci->getProvider(); e != NULL; e = e->getParentEntry(gIOServicePlane)) {
		if (e->inPlane(gIODTPlane)) {
			OSIterator *it = e->getChildIterator(gIODTPlane);
			IORegistryEntry *found = NULL;
			while (it != NULL && found == NULL) {
				IORegistryEntry *c = OSDynamicCast(IORegistryEntry, it->getNextObject());
				if (c == NULL) {
					break;
				}
				if (!strcmp(c->getName(gIODTPlane), "rp1")) {
					found = c;
				}
			}
			OSSafeReleaseNULL(it);
			return found;
		}
	}
	return NULL;
}

RPiMIP *
RP1::findMIP(void)
{
	uint32_t want = 0;
	IOService *bridge = pci->getProvider();
	IOService *node = bridge != NULL ? bridge->getProvider() : NULL;
	OSData *parent = node != NULL ? OSDynamicCast(OSData, node->getProperty("msi-parent")) : NULL;
	if (parent != NULL && parent->getLength() >= sizeof(uint32_t)) {
		want = OSReadBigInt32(parent->getBytesNoCopy(), 0);
	}
	OSDictionary *matching = serviceMatching("RPiMIP");
	if (matching == NULL) {
		return NULL;
	}
	RPiMIP *found = NULL;
	for (int tries = 0; tries < 50 && found == NULL; tries++) {
		OSIterator *iter = getMatchingServices(matching);
		while (iter != NULL && found == NULL) {
			RPiMIP *m = OSDynamicCast(RPiMIP, iter->getNextObject());
			if (m == NULL) {
				break;
			}
			IOService *mnode = m->getProvider();
			OSData *ph = mnode != NULL ? OSDynamicCast(OSData, mnode->getProperty("AAPL,phandle")) : NULL;
			if (want == 0 || (ph != NULL && ph->getLength() >= sizeof(uint32_t) &&
			    *(const uint32_t *)ph->getBytesNoCopy() == want)) {
				found = m;
				found->retain();
			}
		}
		OSSafeReleaseNULL(iter);
		if (found == NULL) {
			IOSleep(100);
		}
	}
	matching->release();
	if (found == NULL) {
		IOLog("RP1: no MIP with phandle 0x%x (its PCIe controller's msi-parent)\n", want);
	}
	return found;
}

bool
RP1::setUpMSIX(void)
{
	uint8_t cap = pci->findCapability(PCI_CAP_MSIX);
	if (cap == 0) {
		IOLog("RP1: no MSI-X capability\n");
		return false;
	}
	uint16_t ctrl = pci->configRead16(cap + 2);
	unsigned tableSize = (ctrl & 0x7ff) + 1U;
	uint32_t table = pci->configRead32(cap + 4);
	int index = pci->barIndex(table & 7);
	if (tableSize < RP1_IRQS || index < 0) {
		IOLog("RP1: MSI-X table of %u entries in BAR%u\n", tableSize, table & 7);
		return false;
	}
	tableMap = pci->mapDeviceMemoryWithIndex((unsigned)index);
	if (tableMap == NULL) {
		return false;
	}
	msixTable = (volatile uint32_t *)(tableMap->getVirtualAddress() + (table & ~7U));

	mip = findMIP();
	firstVector = mip != NULL ? mip->allocate(RP1_IRQS) : -1;
	if (firstVector < 0) {
		IOLog("RP1: no MSI vectors (MIP %s)\n", mip != NULL ? "full" : "missing");
		return false;
	}
	uint64_t doorbell = mip->doorbell();
	OSArray *controllers = OSArray::withCapacity(RP1_IRQS);
	OSArray *specifiers = OSArray::withCapacity(RP1_IRQS);
	if (controllers == NULL || specifiers == NULL) {
		OSSafeReleaseNULL(controllers);
		OSSafeReleaseNULL(specifiers);
		return false;
	}
	for (unsigned i = 0; i < RP1_IRQS; i++) {
		volatile uint32_t *e = msixTable + i * MSIX_ENTRY_SIZE / 4;
		e[0] = (uint32_t)doorbell;
		e[1] = (uint32_t)(doorbell >> 32);
		e[2] = (uint32_t)firstVector + i;
		e[3] = MSIX_ENTRY_CTRL_MASKED;
		uint32_t intid = mip->gicInterrupt((unsigned)firstVector + i);
		OSData *spec = OSData::withBytes(&intid, sizeof(intid));
		controllers->setObject(mip->gicControllerName());
		specifiers->setObject(spec);
		OSSafeReleaseNULL(spec);
	}
	pci->setProperty(gIOInterruptControllersKey, controllers);
	pci->setProperty(gIOInterruptSpecifiersKey, specifiers);
	controllers->release();
	specifiers->release();

	pci->configWrite16(cap + 2, (uint16_t)((ctrl | MSIX_CTRL_ENABLE) & ~MSIX_CTRL_FUNC_MASK));
	pci->configWrite16(0x04, (uint16_t)(pci->configRead16(0x04) | 0x0406));

	IOInterruptAction action = OSMemberFunctionCast(IOInterruptAction, this, &RP1::msixInterrupt);
	for (unsigned i = 0; i < RP1_IRQS; i++) {
		if (pci->registerInterrupt((int)i, this, action, (void *)(uintptr_t)i) != kIOReturnSuccess) {
			IOLog("RP1: cannot take MSI-X vector %u\n", i);
			return false;
		}
		pci->enableInterrupt((int)i);
	}
	return true;
}

bool
RP1::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	pci = OSDynamicCast(RPiPCIDevice, provider);
	if (pci == NULL) {
		return false;
	}
	int index = pci->barIndex(1);
	bar1Map = index >= 0 ? pci->mapDeviceMemoryWithIndex((unsigned)index) : NULL;
	if (bar1Map == NULL || bar1Map->getLength() <= 0x10000) {
		IOLog("RP1: BAR1 not set up (is config.txt's pciex4_reset=0?)\n");
		return false;
	}
	bar1 = (uintptr_t)bar1Map->getVirtualAddress();
	uint32_t chip = *(volatile uint32_t *)bar1;
	IOLog("RP1: chip ID 0x%08x%s, peripherals at 0x%llx (BAR1 PCI 0x%08x)\n", chip,
	    chip == RP1_C0_CHIP_ID ? "" : " (not the C0 this driver knows)", bar1Map->getPhysicalAddress(),
	    pci->configRead32(0x14) & ~0xfU);
	if (chip == 0xffffffff || chip == 0) {
		IOLog("RP1: no answer through BAR1; leaving its devices alone\n");
		return false;
	}

	numVectors = RP1_IRQS;
	vectors = IONewZero(IOInterruptVector, numVectors);
	if (vectors == NULL) {
		return false;
	}
	for (uint32_t i = 0; i < numVectors; i++) {
		vectors[i].interruptLock = IOLockAlloc();
		if (vectors[i].interruptLock == NULL) {
			return false;
		}
	}
	IORegistryEntry *node = findRP1Node();
	if (node == NULL) {
		IOLog("RP1: no rp1 node in the device tree\n");
		return false;
	}
	if (!setUpMSIX()) {
		return false;
	}
	const OSSymbol *name = IODTInterruptControllerName(node);
	if (name == NULL) {
		return false;
	}
	setProperty("InterruptControllerName", (OSObject *)name);
	getPlatform()->registerInterruptController((OSSymbol *)name, this);
	name->release();

	IOLog("RP1: %u interrupts on MSI-X vectors %d-%d\n", RP1_IRQS, firstVector, firstVector + RP1_IRQS - 1);
	registerService();
	RPiPublishDTChildren(this, node);
	return true;
}

IOReturn
RP1::registerInterrupt(IOService *nub, int source, void *target, IOInterruptHandler handler, void *refCon)
{
	OSArray *specs = OSDynamicCast(OSArray, nub->getProperty(gIOInterruptSpecifiersKey));
	OSData *spec = specs ? OSDynamicCast(OSData, specs->getObject((unsigned)source)) : NULL;
	if (spec != NULL && spec->getLength() >= 2 * sizeof(uint32_t)) {
		const uint32_t *cells = (const uint32_t *)spec->getBytesNoCopy();
		if (cells[0] < RP1_IRQS) {
			level[cells[0]] = (cells[1] & 0xc) != 0;
			msixCfg(cells[0], MSIX_CFG_IACK_EN, level[cells[0]]);
		}
	}
	return super::registerInterrupt(nub, source, target, handler, refCon);
}

void
RP1::msixInterrupt(void *refCon, IOService *  , int  )
{
	unsigned irq = (unsigned)(uintptr_t)refCon;
	if (irq >= numVectors) {
		return;
	}
	IOInterruptVector *vector = &vectors[irq];
	vector->interruptActive = 1;
	OSMemoryBarrier();
	if (!vector->interruptDisabledSoft && vector->interruptRegistered) {
		vector->handler(vector->target, vector->refCon, vector->nub, vector->source);
	} else {
		vector->interruptDisabledHard = 1;
		disableVectorHard(irq, vector);
	}
	vector->interruptActive = 0;
	if (level[irq]) {
		msixCfg(irq, MSIX_CFG_IACK, true);
	}
}

bool
RP1::vectorCanBeShared(IOInterruptVectorNumber  , IOInterruptVector *  )
{
	return true;
}

int
RP1::getVectorType(IOInterruptVectorNumber vectorNumber, IOInterruptVector *  )
{
	return vectorNumber < RP1_IRQS && level[vectorNumber] ? kIOInterruptTypeLevel : kIOInterruptTypeEdge;
}

void
RP1::disableVectorHard(IOInterruptVectorNumber vectorNumber, IOInterruptVector *  )
{
	if (vectorNumber < RP1_IRQS) {
		maskEntry(vectorNumber, true);
	}
}

void
RP1::enableVector(IOInterruptVectorNumber vectorNumber, IOInterruptVector *  )
{
	if (vectorNumber < RP1_IRQS) {
		msixCfg(vectorNumber, MSIX_CFG_ENABLE, true);
		maskEntry(vectorNumber, false);
	}
}
