/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_PIE_USB_H
#define _IOKIT_PIE_USB_H

#include "../../pie/usb.h"

/*
 * A USB host controller, as PIEUSBInterface sees it: the xHCI and EHCI
 * drivers implement it. A device is a slot number and the generation it
 * had when its interfaces were published. Once it has gone, current is
 * false and a call for it returns kIOReturnNoDevice. publishInterfaces
 * and terminateInterfaces are shared by the controllers.
 */
class PIEUSBHost : public IOService
{
	OSDeclareAbstractStructors(PIEUSBHost);

public:
	virtual bool current(uint32_t slot, uint32_t generation) = 0;
	virtual IOReturn controlTransfer(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
	    uint16_t wIndex, uint16_t wLength, void *data, uint32_t *actual, uint32_t generation = 0) = 0;
	virtual IOReturn startInterruptIn(uint32_t slot, const struct PIEUSBEndpoint *ep, OSObject *target,
	    PIEUSBInterruptAction action, uint32_t generation = 0) = 0;
	virtual IOReturn bulkTransfer(uint32_t slot, const struct PIEUSBEndpoint *ep, void *data, uint32_t length,
	    uint32_t *actual, uint32_t generation = 0) = 0;
	virtual IOReturn resetPipe(uint32_t slot, uint8_t endpointAddress, uint32_t generation = 0) = 0;
	virtual IOReturn startQueue(uint32_t slot, const struct PIEUSBEndpoint *ep, uint32_t size, uint32_t count,
	    OSObject *target, PIEUSBQueueAction action, uint32_t generation) = 0;
	virtual IOReturn queueOut(uint32_t slot, uint8_t endpointAddress, const void *data, uint32_t length,
	    uint32_t generation) = 0;
	virtual IOReturn stopQueue(uint32_t slot, uint8_t endpointAddress, uint32_t generation) = 0;

	void publishInterfaces(uint32_t slot, uint32_t generation, uint8_t port, uint8_t speed, const uint8_t *dev,
	    const uint8_t *cfg, uint32_t cfgLength);
	void terminateInterfaces(uint32_t slot, uint32_t generation);
};

#endif
