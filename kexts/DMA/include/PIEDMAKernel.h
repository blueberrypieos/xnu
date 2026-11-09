/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <PIEDMAKernel.h>: for kernel drivers that use PIEDMA.kext's
 * controllers (<PIEDMA.h>) from what their device tree node says (dmas,
 * dma-names). Inline, so that a driver has no link-time dependency on
 * PIEDMA.kext: a controller that is not there (the kext not loaded) is
 * reported as kIOReturnNoDevice, and the driver goes on without DMA.
 */

#ifndef _PIEDMA_KERNEL_H_
#define _PIEDMA_KERNEL_H_

#include <IOKit/IOService.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <libkern/OSByteOrder.h>
#include <string.h>

#include "PIEDMA.h"

/* Poison is attached to kernel-owned device nodes and lasts until reboot. */
static inline bool
PIEDMAIsQuarantined(IORegistryEntry *node, const char *key)
{
	OSObject *value = node != NULL ? node->getProperty(key) : NULL;
	OSBoolean *flag = OSDynamicCast(OSBoolean, value);
	return value != NULL && (flag == NULL || flag->isTrue());
}

/* Called under the DMA registry lock, before hardware ownership. Preparing
 * the key with a shared Boolean makes the later false->true replacement
 * allocation-free; a live flag is never reset by a reprobe. */
static inline bool
PIEDMAPrepareQuarantineFlag(IORegistryEntry *node, const char *key)
{
	if (node == NULL || PIEDMAIsQuarantined(node, key)) { return false; }
	return node->getProperty(key) != NULL || node->setProperty(key, kOSBooleanFalse);
}

/*
 * The controller (retained) and cell of the client's dma-names entry name
 * (dmas = <phandle cell> pairs, #dma-cells 1), waited for up to waitNs;
 * NULL if it has none or the controller does not come.
 */
static inline IOService *
PIEDMACopyController(IORegistryEntry *client, const char *name, uint32_t *cell, uint64_t waitNs)
{
	OSData *dmas = OSDynamicCast(OSData, client->getProperty("dmas"));
	OSData *names = OSDynamicCast(OSData, client->getProperty("dma-names"));
	if (dmas == NULL || names == NULL) {
		return NULL;
	}
	const char *p = (const char *)names->getBytesNoCopy(), *end = p + names->getLength();
	unsigned index = 0;
	for (; p < end && strncmp(p, name, (size_t)(end - p)) != 0; p += strnlen(p, (size_t)(end - p)) + 1) {
		index++;
	}
	if (p >= end || dmas->getLength() < 8 * (index + 1)) {
		return NULL;
	}
	const uint8_t *d = (const uint8_t *)dmas->getBytesNoCopy() + 8 * index;
	*cell = OSReadBigInt32(d, 4);
	const OSSymbol *key = OSSymbol::withCStringNoCopy(kPIEDMAPHandleKey);
	OSNumber *value = OSNumber::withNumber(OSReadBigInt32(d, 0), 32);
	OSDictionary *match = key != NULL && value != NULL ? IOService::propertyMatching(key, value) : NULL;
	IOService *controller = match != NULL ? IOService::waitForMatchingService(match, waitNs) : NULL;
	OSSafeReleaseNULL(match);
	OSSafeReleaseNULL(value);
	OSSafeReleaseNULL(key);
	return controller;
}

static inline IOReturn
PIEDMACall(IOService *controller, const char *function, void *p1, void *p2, void *p3, void *p4 = NULL)
{
	const OSSymbol *name = OSSymbol::withCStringNoCopy(function);
	IOReturn ret = name != NULL ? controller->callPlatformFunction(name, false, p1, p2, p3, p4) : kIOReturnNoMemory;
	OSSafeReleaseNULL(name);
	return ret;
}

/* A channel for r (r->cell set from the client's dma-names entry name); *controller retained. */
static inline IOReturn
PIEDMAAllocateChannel(PIEDMARequest *r, const char *name, IOService **controller, uint64_t waitNs)
{
	*controller = PIEDMACopyController(r->client, name, &r->cell, waitNs);
	if (*controller == NULL) {
		return kIOReturnNoDevice;
	}
	IOReturn ret = PIEDMACall(*controller, kPIEDMAAllocate, r, NULL, NULL);
	if (ret != kIOReturnSuccess) {
		OSSafeReleaseNULL(*controller);
	}
	return ret;
}

static inline IOReturn
PIEDMAStartChannel(IOService *controller, void *channel, uint64_t pa, uint32_t length,
    IOBufferMemoryDescriptor *memory)
{
	return PIEDMACall(controller, kPIEDMAStart, channel, &pa, &length, memory);
}

static inline IOReturn
PIEDMAWaitChannel(IOService *controller, void *channel, uint64_t timeoutNs)
{
	return PIEDMACall(controller, kPIEDMAWait, channel, &timeoutNs, NULL);
}

static inline IOReturn
PIEDMAReleaseChannel(IOService *controller, void *channel)
{
	return PIEDMACall(controller, kPIEDMARelease, channel, NULL, NULL);
}

#endif /* _PIEDMA_KERNEL_H_ */
