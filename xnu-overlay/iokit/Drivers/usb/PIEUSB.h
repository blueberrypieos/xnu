/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_PIE_USB_H
#define _IOKIT_PIE_USB_H

#include <IOKit/IOService.h>

class XHCIController;

typedef void (*PIEUSBInterruptAction)(OSObject *target, const uint8_t *data, uint32_t length);

struct PIEUSBEndpoint {
	uint8_t address;
	uint8_t attributes;
	uint8_t interval;
	uint16_t maxPacket;
};

class PIEUSBInterface : public IOService
{
	OSDeclareDefaultStructors(PIEUSBInterface);

public:
	XHCIController *controller;
	uint32_t slot;
	uint32_t generation;
	uint8_t number;
	uint8_t numEndpoints;
	struct PIEUSBEndpoint endpoints[16];

	IOReturn controlRequest(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
	    uint16_t wIndex, uint16_t wLength, void *data);

	IOReturn startInterruptIn(uint8_t endpointAddress, OSObject *target, PIEUSBInterruptAction action);

	IOReturn bulkTransfer(uint8_t endpointAddress, void *data, uint32_t length, uint32_t *actual);
	IOReturn clearHalt(uint8_t endpointAddress);
};

#endif
