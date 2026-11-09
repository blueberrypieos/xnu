/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <PieGPIOKernel.h>: for kernel drivers that use piegpio's GPIO
 * controllers (<PieGPIO.h>'s platform functions) from what their device
 * tree node says: a line by its phandle and number (cs-gpios, gpios), and
 * the node's pin configuration (pinctrl-0). Inline, so that a driver has no
 * link-time dependency on piegpio.kext: a controller that is not there
 * (piegpio not loaded) is waited for, then reported as kIOReturnNoDevice.
 */

#ifndef _PIEGPIO_KERNEL_H_
#define _PIEGPIO_KERNEL_H_

#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOService.h>
#include <libkern/OSByteOrder.h>

#include "PieGPIO.h"

/* How long a driver waits for a controller when it starts. */
#define kPieGPIOWaitNs  (5ULL * 1000 * 1000 * 1000)

/* pieboot's device tree keeps handles little-endian; FDT's are big-endian. */
static inline bool
PieGPIOReadPHandle(IORegistryEntry *node, uint32_t *value)
{
	if (node == NULL || value == NULL) { return false; }
	OSData *p = OSDynamicCast(OSData, node->getProperty("AAPL,phandle"));
	if (p != NULL && p->getLength() == sizeof(uint32_t)) {
		*value = OSReadLittleInt32(p->getBytesNoCopy(), 0);
		return *value != 0;
	}
	p = OSDynamicCast(OSData, node->getProperty("phandle"));
	if (p == NULL || p->getLength() != sizeof(uint32_t)) { return false; }
	*value = OSReadBigInt32(p->getBytesNoCopy(), 0);
	return *value != 0;
}

/* The controller whose device tree node has that phandle (retained), waited for; NULL if none came. */
static inline IOService *
PieGPIOCopyController(uint32_t phandle, uint64_t waitNs)
{
	const OSSymbol *key = OSSymbol::withCStringNoCopy(kPieGPIOPHandleKey);
	OSNumber *value = OSNumber::withNumber(phandle, 32);
	OSDictionary *match = key != NULL && value != NULL ? IOService::propertyMatching(key, value) : NULL;
	IOService *controller = match != NULL ? IOService::waitForMatchingService(match, waitNs) : NULL;
	OSSafeReleaseNULL(match);
	OSSafeReleaseNULL(value);
	OSSafeReleaseNULL(key);
	return controller;
}

static inline IOReturn
PieGPIOCall(IOService *controller, const char *function, void *p1, void *p2, void *p3)
{
	const OSSymbol *name = OSSymbol::withCStringNoCopy(function);
	IOReturn ret = name != NULL ? controller->callPlatformFunction(name, false, p1, p2, p3, NULL) : kIOReturnNoMemory;
	OSSafeReleaseNULL(name);
	return ret;
}

/* The device tree node with that phandle (retained), or NULL. */
static inline IORegistryEntry *
PieGPIOCopyNodeForPHandle(uint32_t phandle)
{
	IORegistryEntry *root = IORegistryEntry::fromPath("/", gIODTPlane);
	IORegistryIterator *it = root != NULL ?
	    IORegistryIterator::iterateOver(root, gIODTPlane, kIORegistryIterateRecursively) : NULL;
	IORegistryEntry *found = NULL, *e;
	while (it != NULL && found == NULL && (e = it->getNextObject()) != NULL) {
		uint32_t handle;
		if (PieGPIOReadPHandle(e, &handle) && handle == phandle) {
			found = e;
			found->retain();
		}
	}
	OSSafeReleaseNULL(it);
	OSSafeReleaseNULL(root);
	return found;
}

/*
 * Drives a line the way Linux's GPIO descriptors do: cells is a
 * <phandle line flags> triple (flags bit 0: active low), value the
 * logical level. The line stays the consumer's.
 */
static inline IOReturn
PieGPIOSetLine(const uint8_t *cells, bool value, const char *consumer, uint64_t waitNs)
{
	uint32_t phandle = OSReadBigInt32(cells, 0), line = OSReadBigInt32(cells, 4);
	bool activeLow = OSReadBigInt32(cells, 8) & 1;
	IOService *controller = PieGPIOCopyController(phandle, waitNs);
	if (controller == NULL) {
		return kIOReturnNoDevice;
	}
	IOReturn ret = PieGPIOCall(controller, kPieGPIOSetOutput, (void *)(uintptr_t)line,
	    (void *)(uintptr_t)(value != activeLow), (void *)consumer);
	controller->release();
	return ret;
}

/*
 * Applies a device's pinctrl-0: each configuration node it names, given
 * to the controller that is its parent node.
 */
static inline IOReturn
PieGPIOApplyPinctrl(IORegistryEntry *device, const char *consumer, uint64_t waitNs)
{
	OSData *list = OSDynamicCast(OSData, device->getProperty("pinctrl-0"));
	unsigned n = list != NULL ? list->getLength() / 4 : 0;
	IOReturn ret = kIOReturnSuccess;
	for (unsigned i = 0; i < n && ret == kIOReturnSuccess; i++) {
		IORegistryEntry *config = PieGPIOCopyNodeForPHandle(OSReadBigInt32(list->getBytesNoCopy(), 4 * i));
		IORegistryEntry *parent = config != NULL ? config->getParentEntry(gIODTPlane) : NULL;
		uint32_t handle;
		IOService *controller = PieGPIOReadPHandle(parent, &handle) ?
		    PieGPIOCopyController(handle, waitNs) : NULL;
		if (config == NULL) {
			ret = kIOReturnNotFound;
		} else if (controller == NULL) {
			ret = kIOReturnNoDevice;
		} else {
			ret = PieGPIOCall(controller, kPieGPIOSetPinctrl, config, (void *)consumer, NULL);
		}
		OSSafeReleaseNULL(controller);
		OSSafeReleaseNULL(config);
	}
	return ret;
}

/*
 * Calls fn back for the line's edges (cells: <phandle line flags>, flags
 * bit 0 active low), logical rising and/or falling, until the consumer
 * releases it (PieGPIOReleaseAll). kIOReturnUnsupported if the
 * controller cannot see edges (no interrupt).
 */
static inline IOReturn
PieGPIORequestLineEdges(const uint8_t *cells, bool rising, bool falling, const char *consumer, PieGPIOEdgeFn fn,
    void *refcon, uint64_t waitNs)
{
	PieGPIOKernelEdges e = {
		.line = OSReadBigInt32(cells, 4), .activeLow = (OSReadBigInt32(cells, 8) & 1) != 0,
		.rising = rising, .falling = falling, .consumer = consumer, .fn = fn, .refcon = refcon,
	};
	IOService *controller = PieGPIOCopyController(OSReadBigInt32(cells, 0), waitNs);
	if (controller == NULL) {
		return kIOReturnNoDevice;
	}
	IOReturn ret = PieGPIOCall(controller, kPieGPIORequestEdges, &e, NULL, NULL);
	controller->release();
	return ret;
}

/* A line's logical value (cells as above): its pin, active low applied. */
static inline IOReturn
PieGPIOGetLine(const uint8_t *cells, bool *value, uint64_t waitNs)
{
	IOService *controller = PieGPIOCopyController(OSReadBigInt32(cells, 0), waitNs);
	if (controller == NULL) {
		return kIOReturnNoDevice;
	}
	uint32_t pin = 0;
	IOReturn ret = PieGPIOCall(controller, kPieGPIOGetValue, (void *)(uintptr_t)OSReadBigInt32(cells, 4), &pin, NULL);
	controller->release();
	*value = (pin != 0) != ((OSReadBigInt32(cells, 8) & 1) != 0);
	return ret;
}

/* Gives back every line the consumer holds, on every controller. */
static inline void
PieGPIOReleaseAll(const char *consumer)
{
	OSDictionary *match = IOService::serviceMatching(kPieGPIOControllerClass);
	OSIterator *it = match != NULL ? IOService::getMatchingServices(match) : NULL;
	IOService *controller;
	while (it != NULL && (controller = OSDynamicCast(IOService, it->getNextObject())) != NULL) {
		PieGPIOCall(controller, kPieGPIORelease, (void *)consumer, NULL, NULL);
	}
	OSSafeReleaseNULL(it);
	OSSafeReleaseNULL(match);
}

#endif /* _PIEGPIO_KERNEL_H_ */
