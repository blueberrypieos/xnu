/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceMemory.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <libkern/OSByteOrder.h>
#include <mach/mach_time.h>

#include "../rpi/RPiPlatform.h"
#include "RKSoC.h"

/*
 * The first register range of a device tree node, uncached. The device
 * memory array belongs to the node's property, and is used through a
 * reference from copyProperty. Without that property the node's first
 * reg entry is resolved here, through its parent, and nothing is written
 * to the node: only its nub's resources set the property.
 */
IOMemoryMap *
RKMapNode(IORegistryEntry *node)
{
	OSObject *prop = node->copyProperty(gIODeviceMemoryKey);
	OSArray *mem = OSDynamicCast(OSArray, prop);
	IOMemoryDescriptor *range = mem != NULL ? OSDynamicCast(IOMemoryDescriptor, mem->getObject(0)) : NULL;
	IOMemoryMap *map = range != NULL ? range->map(kIOMapInhibitCache) : NULL;
	if (prop == NULL) {
		IORegistryEntry *parent = node->copyParentEntry(gIODTPlane);
		OSObject *regProp = node->copyProperty("reg");
		OSData *reg = OSDynamicCast(OSData, regProp);
		UInt32 sizeCells = 0, addressCells = 0;
		IOPhysicalAddress phys = 0;
		IOPhysicalLength len = 0;
		if (parent != NULL) {
			IODTGetCellCounts(parent, &sizeCells, &addressCells);
		}
		if (parent != NULL && reg != NULL && sizeCells != 0 && reg->getLength() >= 4 * (sizeCells + addressCells) &&
		    IODTResolveAddressCell(parent, (UInt32 *)(uintptr_t)reg->getBytesNoCopy(), &phys, &len)) {
			IODeviceMemory *one = IODeviceMemory::withRange(phys, len);
			map = one != NULL ? one->map(kIOMapInhibitCache) : NULL;
			OSSafeReleaseNULL(one);
		}
		OSSafeReleaseNULL(regProp);
		OSSafeReleaseNULL(parent);
	}
	OSSafeReleaseNULL(prop);
	return map;
}

/*
 * The registers of the first node compatible with compat. A node that
 * its nub has replaced has no parent; the second lookup finds the nub.
 */
IOMemoryMap *
RKMapCompatible(const char *compat)
{
	IOMemoryMap *map = NULL;
	for (int tries = 0; map == NULL && tries < 2; tries++) {
		OSArray *nodes = RPiDTFindCompatible(compat);
		IORegistryEntry *node = nodes != NULL ? OSDynamicCast(IORegistryEntry, nodes->getObject(0)) : NULL;
		map = node != NULL ? RKMapNode(node) : NULL;
		OSSafeReleaseNULL(nodes);
	}
	return map;
}

/* The registers of the node with this phandle, looked up a second time as in RKMapCompatible. */
IOMemoryMap *
RKMapPHandle(uint32_t phandle)
{
	IOMemoryMap *map = NULL;
	for (int tries = 0; map == NULL && tries < 2; tries++) {
		IORegistryEntry *node = RKFindPHandle(IORegistryEntry::getRegistryRoot(), phandle);
		map = node != NULL ? RKMapNode(node) : NULL;
		OSSafeReleaseNULL(node);
	}
	return map;
}

/* The node below root with this phandle, retained; the walk starts again if the plane changes under it. */
IORegistryEntry *
RKFindPHandle(IORegistryEntry *root, uint32_t phandle)
{
	IORegistryIterator *it = IORegistryIterator::iterateOver(root, gIODTPlane, kIORegistryIterateRecursively);
	IORegistryEntry *found = NULL;
	if (it == NULL) {
		return NULL;
	}
	do {
		OSSafeReleaseNULL(found);
		it->reset();
		for (IORegistryEntry *e; (e = it->getNextObject()) != NULL;) {
			OSData *ph = OSDynamicCast(OSData, e->getProperty("AAPL,phandle"));
			if (ph != NULL && ph->getLength() == 4 && *(const uint32_t *)ph->getBytesNoCopy() == phandle) {
				found = e;
				found->retain();
				break;
			}
		}
	} while (found == NULL && !it->isValid());
	it->release();
	return found;
}

/* The child of parent in the device tree plane named name, retained. */
IORegistryEntry *
RKChildNamed(IORegistryEntry *parent, const char *name)
{
	OSIterator *it = parent != NULL ? parent->getChildIterator(gIODTPlane) : NULL;
	IORegistryEntry *found = NULL;
	if (it == NULL) {
		return NULL;
	}
	do {
		OSSafeReleaseNULL(found);
		it->reset();
		for (OSObject *o; (o = it->getNextObject()) != NULL;) {
			IORegistryEntry *e = OSDynamicCast(IORegistryEntry, o);
			const char *n = e != NULL ? e->getName(gIODTPlane) : NULL;
			if (n != NULL && strcmp(n, name) == 0) {
				found = e;
				found->retain();
				break;
			}
		}
	} while (found == NULL && !it->isValid());
	it->release();
	return found;
}

/*
 * The pins of pinctrl-0, set to their functions: GPIO0's in the PMU GRF,
 * GPIO1-4's in the GRF, four bits per pin and four pins per register,
 * written through the upper half's mask. The groups and their settings
 * are found from the device tree's root. The pin controller's node is
 * replaced by its nub while drivers start, and the node it replaces has
 * no children.
 */
void
RKApplyPinctrl(IOService *provider, const char *driver)
{
	OSData *pc = OSDynamicCast(OSData, provider->getProperty("pinctrl-0"));
	IORegistryEntry *root = IORegistryEntry::getRegistryRoot();
	IOMemoryMap *grfMap = RKMapCompatible("rockchip,rk3568-grf");
	IOMemoryMap *pmuMap = RKMapCompatible("rockchip,rk3568-pmugrf");
	if (pc == NULL || grfMap == NULL || pmuMap == NULL) {
		IOLog("%s: %s: no pin configuration\n", driver, provider->getName());
		goto out;
	}
	for (unsigned i = 0; i < pc->getLength() / 4; i++) {
		uint32_t ph = OSSwapBigToHostInt32(((const uint32_t *)pc->getBytesNoCopy())[i]);
		IORegistryEntry *group = RKFindPHandle(root, ph);
		OSData *pins = group ? OSDynamicCast(OSData, group->getProperty("rockchip,pins")) : NULL;
		if (pins == NULL) {
			IOLog("%s: %s: no pin group 0x%x\n", driver, provider->getName(), ph);
		}
		for (unsigned j = 0; pins != NULL && j + 4 <= pins->getLength() / 4; j += 4) {
			const uint32_t *q = (const uint32_t *)pins->getBytesNoCopy() + j;
			uint32_t bank = OSSwapBigToHostInt32(q[0]), pin = OSSwapBigToHostInt32(q[1]);
			uint32_t func = OSSwapBigToHostInt32(q[2]);
			if (bank > 4 || pin > 31 || func > 7) {
				continue;
			}
			volatile uint32_t *reg = bank == 0 ?
			    (volatile uint32_t *)(pmuMap->getVirtualAddress() + (pin / 4) * 4) :
			    (volatile uint32_t *)(grfMap->getVirtualAddress() + (bank - 1) * 0x20 + (pin / 4) * 4);
			uint32_t shift = (pin % 4) * 4;
			*reg = (0x7U << (shift + 16)) | (func << shift);

			if (bank == 0) {
				continue;
			}
			IORegistryEntry *conf = RKFindPHandle(root, OSSwapBigToHostInt32(q[3]));
			if (conf == NULL) {
				IOLog("%s: %s: no pin settings 0x%x for GPIO%u pin %u\n", driver, provider->getName(),
				    OSSwapBigToHostInt32(q[3]), bank, pin);
				continue;
			}
			int pull = conf->getProperty("bias-pull-up") ? 1 : conf->getProperty("bias-pull-down") ? 2 :
			    conf->getProperty("bias-disable") ? 0 : -1;
			if (pull >= 0) {
				reg = (volatile uint32_t *)(grfMap->getVirtualAddress() + 0x80 + (bank - 1) * 0x10 + (pin / 8) * 4);
				shift = (pin % 8) * 2;
				*reg = (0x3U << (shift + 16)) | ((uint32_t)pull << shift);
			}
			OSData *ds = OSDynamicCast(OSData, conf->getProperty("drive-strength"));
			if (ds != NULL && ds->getLength() == 4) {
				uint32_t level = OSSwapBigToHostInt32(*(const uint32_t *)ds->getBytesNoCopy());
				reg = (volatile uint32_t *)(grfMap->getVirtualAddress() + 0x200 + (bank - 1) * 0x40 + (pin / 2) * 4);
				shift = (pin % 2) * 8;
				*reg = (0x3fU << (shift + 16)) | ((((1U << (level + 1)) - 1) & 0x3f) << shift);
			}
			conf->release();
		}
		OSSafeReleaseNULL(group);
	}
out:
	OSSafeReleaseNULL(grfMap);
	OSSafeReleaseNULL(pmuMap);
}

/*
 * The rate of an RK3568 PLL from its three CON registers at con and its
 * mode bits; 24 MHz while the mode selects the oscillator.
 */
uint64_t
RKPLLRate(IOMemoryMap *cru, uint32_t con, uint32_t mode, unsigned modeShift)
{
	if (((RKRead(cru, mode) >> modeShift) & 3) != 1) {
		return 24000000;
	}
	uint32_t c0 = RKRead(cru, con), c1 = RKRead(cru, con + 4), c2 = RKRead(cru, con + 8);
	uint64_t fbdiv = c0 & 0xfff, postdiv1 = (c0 >> 12) & 7, refdiv = c1 & 0x3f, postdiv2 = (c1 >> 6) & 7;
	if (postdiv1 == 0 || postdiv2 == 0 || refdiv == 0) {
		return 0;
	}
	uint64_t hz = 24000000ULL * fbdiv / refdiv;
	if (!(c1 & (1U << 12))) {
		hz += (24000000ULL * (c2 & 0xffffff) / refdiv) >> 24;
	}
	return hz / postdiv1 / postdiv2;
}

/*
 * Waits until the boot volume's media has appeared, or for seconds, a
 * minute unless given; 0 looks without waiting. USB power and PHY bring-up
 * wait for it, so that they stay out of the boot SD card's setup.
 */
bool
RKWaitForBootVolume(uint32_t seconds)
{
	OSDictionary *match = IOService::resourceMatching("boot-uuid-media");
	IOService *media = match != NULL ? IOService::waitForMatchingService(match, seconds * NSEC_PER_SEC) : NULL;
	bool found = media != NULL;
	OSSafeReleaseNULL(media);
	OSSafeReleaseNULL(match);
	return found;
}

/* Runs fn(owner) on a new kernel thread, which owns a reference to owner and must release it. */
bool
RKStartThread(OSObject *owner, void (*fn)(void *, wait_result_t))
{
	thread_t thread;
	owner->retain();
	if (kernel_thread_start(fn, owner, &thread) != KERN_SUCCESS) {
		owner->release();
		return false;
	}
	thread_deallocate(thread);
	return true;
}

OSDefineMetaClassAndAbstractStructors(RKRegulators, IOService);

/*
 * The driver whose provider is the regulator node's PMIC, the parent of
 * the node's parent, looked for every 50 ms and once more at the end. The
 * PMIC's node is replaced by its nub when its bus starts, so it is found
 * again each time.
 */
RKRegulators *
RKFindRegulators(IORegistryEntry *node, uint32_t ms)
{
	OSDictionary *match = IOService::serviceMatching("RKRegulators");
	RKRegulators *found = NULL;
	uint64_t deadline;
	clock_interval_to_deadline(ms, kMillisecondScale, &deadline);
	for (bool last = false; match != NULL && found == NULL && !last;) {
		last = mach_absolute_time() >= deadline;
		IORegistryEntry *group = node->copyParentEntry(gIODTPlane);
		IORegistryEntry *pmic = group != NULL ? group->copyParentEntry(gIODTPlane) : NULL;
		OSIterator *it = pmic != NULL ? IOService::getMatchingServices(match) : NULL;
		for (OSObject *o; it != NULL && found == NULL && (o = it->getNextObject()) != NULL;) {
			RKRegulators *r = OSDynamicCast(RKRegulators, o);
			if (r != NULL && r->getProvider() == pmic) {
				found = r;
				found->retain();
			}
		}
		OSSafeReleaseNULL(it);
		OSSafeReleaseNULL(pmic);
		OSSafeReleaseNULL(group);
		if (found == NULL && !last) {
			IOSleep(50);
		}
	}
	OSSafeReleaseNULL(match);
	return found;
}
