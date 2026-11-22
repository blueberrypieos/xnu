/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <pie/usb.h>: the kernel's USB interfaces, for USB drivers in kexts, from
 * the com.blueberrypieos.kpi.platform symbol set (config/PIE.exports).
 * iokit/Drivers/usb/XHCIController.cpp implements them.
 *
 * The xHCI driver configures each device's first configuration and
 * publishes a PIEUSBInterface for each of its interfaces, alternate
 * setting 0. A driver matches one with IOProviderClass PIEUSBInterface and
 * the properties idVendor, idProduct, bInterfaceNumber, bInterfaceClass,
 * bInterfaceSubClass and bInterfaceProtocol. port and speed say where the
 * device is: speed 1 is full, 2 low, 3 high and 4 super speed. When the
 * device goes, the interface terminates, and every call returns
 * kIOReturnNoDevice. Every call also returns it once ep0 cannot be reset
 * after a failed request, once an endpoint that bulkTransfer uses does not
 * reset, and once the controller takes no more commands because one could
 * neither finish nor be aborted.
 *
 * A kext may call controlRequest, clearHalt and the bulk queue calls.
 * startInterruptIn and bulkTransfer are for the kernel's own drivers.
 * Every call but queueBulkOut waits for the device or the controller, so
 * a driver makes them from its own threads, never from an action.
 */

#ifndef _PIE_USB_H_
#define _PIE_USB_H_

#include <IOKit/IOService.h>

class XHCIController;

typedef void (*PIEUSBInterruptAction)(OSObject *target, const uint8_t *data, uint32_t length);

/*
 * A bulk queue's action, on the controller's work loop. On an IN endpoint
 * data holds the length bytes received, until the action returns. On an
 * OUT endpoint data is NULL, and length is the transfer's. A status other
 * than kIOReturnSuccess means the queue has halted. A controller runs one
 * action at a time, under its work loop's gate, whichever thread it runs
 * on. That can be the thread of any call that waits for the device, so an
 * action must not block, wait for the device or take a lock that a driver
 * holds around a call.
 */
typedef void (*PIEUSBQueueAction)(OSObject *target, void *data, uint32_t length, IOReturn status);

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

	/*
	 * After an IN request that returns kIOReturnSuccess, data holds the
	 * bytes the device sent, and the rest of its wLength bytes are zero.
	 */
	IOReturn controlRequest(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
	    uint16_t wIndex, uint16_t wLength, void *data);

	IOReturn startInterruptIn(uint8_t endpointAddress, OSObject *target, PIEUSBInterruptAction action);

	IOReturn bulkTransfer(uint8_t endpointAddress, void *data, uint32_t length, uint32_t *actual);
	IOReturn clearHalt(uint8_t endpointAddress);

	/*
	 * A bulk queue on a bulk endpoint: count buffers, 1 to 32, of size
	 * bytes, a power of two from 512 to 65536, at most 256 KiB in all. On
	 * an IN endpoint size must be a whole number of the endpoint's packets;
	 * each buffer is queued at once and again after its action returns. On
	 * an OUT endpoint queueBulkOut copies a transfer into a free buffer and
	 * queues it, or returns kIOReturnNoResources if all are queued; a
	 * transfer that is a whole number of packets ends with a zero-length
	 * packet. Once stopBulkQueue returns, whatever it returns, the queue's
	 * action is not called again. It returns kIOReturnNotFound when the
	 * endpoint has no queue and the device is present: that is not a
	 * failure, and the endpoint takes a new queue unless an earlier
	 * stopBulkQueue on it failed. After a failed stopBulkQueue the endpoint
	 * takes no new queue until the device goes, and startBulkQueue returns
	 * kIOReturnBusy. kIOReturnTimeout from startBulkQueue means the
	 * controller takes no more commands: every call then returns
	 * kIOReturnNoDevice until a reboot. After any other failure
	 * startBulkQueue may be called again. A restart, after a halt or any
	 * other stop, is always stopBulkQueue, then clearHalt, then
	 * startBulkQueue: startBulkQueue starts the host's data toggle or
	 * sequence number at zero, and clearHalt resets the device's. One queue
	 * per endpoint, and not on an endpoint bulkTransfer uses. target is not
	 * retained: a driver stops its queues before it is freed, and its kext
	 * before it unloads.
	 */
	IOReturn startBulkQueue(uint8_t endpointAddress, uint32_t size, uint32_t count, OSObject *target,
	    PIEUSBQueueAction action);
	IOReturn queueBulkOut(uint8_t endpointAddress, const void *data, uint32_t length);
	IOReturn stopBulkQueue(uint8_t endpointAddress);
};

#endif /* _PIE_USB_H_ */
