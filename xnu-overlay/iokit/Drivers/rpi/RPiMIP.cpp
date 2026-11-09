/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <libkern/OSByteOrder.h>

#include "RPiPlatform.h"

extern "C" {
#include <pexpert/arm64/gicv2.h>
}

enum {
	MIP_INT_CFGL_HOST  = 0x20,
	MIP_INT_CFGH_HOST  = 0x30,
	MIP_INT_MASKL_HOST = 0x40,
	MIP_INT_MASKH_HOST = 0x50,
	MIP_INT_MASKL_VPU  = 0x60,
	MIP_INT_MASKH_VPU  = 0x70,
};

#undef super
#define super IOService
OSDefineMetaClassAndStructors(RPiMIP, IOService);

bool
RPiMIP::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	lock = IOLockAlloc();
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (lock == NULL || regsMap == NULL) {
		IOLog("RPiMIP: cannot map the registers\n");
		return false;
	}

	OSData *busReg = OSDynamicCast(OSData, provider->getProperty("pieboot,bus-reg"));
	if (busReg == NULL || busReg->getLength() < 4 * sizeof(uint64_t)) {
		IOLog("RPiMIP: no doorbell address\n");
		return false;
	}
	doorbellAddress = ((const uint64_t *)busReg->getBytesNoCopy())[2];

	OSData *ranges = OSDynamicCast(OSData, provider->getProperty("msi-ranges"));
	if (ranges == NULL || ranges->getLength() < 5 * sizeof(uint32_t)) {
		IOLog("RPiMIP: no msi-ranges\n");
		return false;
	}
	const uint8_t *r = (const uint8_t *)ranges->getBytesNoCopy();
	uint32_t gicPhandle = OSReadBigInt32(r, 0);
	spiBase = OSReadBigInt32(r, 8);
	count = OSReadBigInt32(r, 16);
	if (count > 64) {
		count = 64;
	}
	char name[48];
	snprintf(name, sizeof(name), "IOInterruptController%08X", gicPhandle);
	gicName = OSSymbol::withCString(name);

	volatile uint32_t *regs = (volatile uint32_t *)regsMap->getVirtualAddress();
	regs[MIP_INT_MASKL_HOST / 4] = 0;
	regs[MIP_INT_MASKH_HOST / 4] = 0;
	regs[MIP_INT_MASKL_VPU / 4] = ~0U;
	regs[MIP_INT_MASKH_VPU / 4] = ~0U;
	regs[MIP_INT_CFGL_HOST / 4] = ~0U;
	regs[MIP_INT_CFGH_HOST / 4] = ~0U;

	IORegistryIterator *iter = IORegistryIterator::iterateOver(gIODTPlane, kIORegistryIterateRecursively);
	if (iter == NULL) {
		IOLog("RPiMIP: cannot walk the device tree\n");
		return false;
	}
	unsigned walks = 0;
	do {
		walks++;
		iter->reset();
		used = 0;
		for (unsigned v = 0; v < count; v++) {
			OSSafeReleaseNULL(owner[v]);
		}
		IORegistryEntry *entry;
		while ((entry = iter->getNextObject()) != NULL) {
			OSArray *controllers = OSDynamicCast(OSArray, entry->getProperty(gIOInterruptControllersKey));
			OSArray *specifiers = OSDynamicCast(OSArray, entry->getProperty(gIOInterruptSpecifiersKey));
			if (controllers == NULL || specifiers == NULL) {
				continue;
			}
			for (unsigned i = 0; i < specifiers->getCount() && i < controllers->getCount(); i++) {
				OSSymbol *controller = OSDynamicCast(OSSymbol, controllers->getObject(i));
				OSData *specifier = OSDynamicCast(OSData, specifiers->getObject(i));
				if (controller == NULL || specifier == NULL || specifier->getLength() < sizeof(uint32_t) ||
				    !controller->isEqualTo(gicName)) {
					continue;
				}
				uint32_t intid = *(const uint32_t *)specifier->getBytesNoCopy();
				if (intid >= gicInterrupt(0) && intid < gicInterrupt(count)) {
					used |= 1ULL << (intid - gicInterrupt(0));
					if (owner[intid - gicInterrupt(0)] == NULL) {
						owner[intid - gicInterrupt(0)] = OSSymbol::withCString(entry->getName());
					}
				}
			}
		}
	} while (!iter->isValid());
	iter->release();
	if (walks > 1) {
		IOLog("RPiMIP: the device tree changed while it was walked; walked it %u times\n", walks);
	}
	for (unsigned v = 0; v < count; v++) {
		if ((used >> v) & 1) {
			IOLog("RPiMIP: vector %u (GIC SPI %u) is %s's interrupt: not handed out\n", v, spiBase + v,
			    owner[v] != NULL ? owner[v]->getCStringNoCopy() : "a device");
		}
	}

	IOLog("RPiMIP: %u MSI vectors, doorbell PCI 0x%llx, GIC SPIs %u-%u\n", count, doorbellAddress,
	    spiBase, spiBase + count - 1);
	OSMemoryBarrier();
	started = true;
	registerService();
	return true;
}

bool
RPiMIP::serializeProperties(OSSerialize *s) const
{
	bool ready = started;
	OSMemoryBarrier();
	OSArray *vectors = ready ? OSArray::withCapacity(count) : NULL;
	if (vectors != NULL) {
		IOLockLock(lock);
		uint64_t inUse = used;
		IOLockUnlock(lock);
		for (unsigned v = 0; v < count; v++) {
			char text[80];
			const char *who = owner[v] != NULL ? owner[v]->getCStringNoCopy() : ((inUse >> v) & 1) ? "msi" : "free";
			snprintf(text, sizeof(text), "SPI %u: %s, %s", spiBase + v, who,
			    pe_gicv2_is_edge(gicInterrupt(v)) ? "edge" : "level");
			OSString *str = OSString::withCString(text);
			if (str != NULL) {
				vectors->setObject(str);
				str->release();
			}
		}
		const_cast<RPiMIP *>(this)->setProperty("MSIVectors", vectors);
		vectors->release();
	}
	return super::serializeProperties(s);
}

int
RPiMIP::allocate(unsigned n)
{
	int first = -1;
	IOLockLock(lock);
	for (unsigned v = 0; v + n <= count && first < 0; v++) {
		uint64_t want = (n >= 64 ? ~0ULL : ((1ULL << n) - 1)) << v;
		if ((used & want) == 0) {
			used |= want;
			first = (int)v;
		}
	}
	IOLockUnlock(lock);
	for (unsigned i = 0; first >= 0 && i < n; i++) {
		pe_gicv2_set_edge(gicInterrupt((unsigned)first + i), true);
	}
	return first;
}
