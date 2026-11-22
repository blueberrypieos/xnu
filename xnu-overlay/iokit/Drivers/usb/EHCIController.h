/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_PIE_EHCI_CONTROLLER_H
#define _IOKIT_PIE_EHCI_CONTROLLER_H

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOWorkLoop.h>
#include <kern/thread.h>

#include "PIEUSB.h"

struct EHCIQH;
struct EHCIDevice;
struct EHCIPipe;

/*
 * An EHCI host controller with the asynchronous schedule only: control
 * and bulk transfers to high-speed devices, behind high-speed hubs that
 * are polled. Full- and low-speed devices go to the companion controller
 * or are left alone, and interrupt endpoints are not supported. A platform
 * subclass prepares the hardware in platformStart, which runs on the
 * controller's own thread before anything touches its registers, and
 * gives up once stopRequested returns true. stop cancels that thread and
 * waits for it, then halts the controller, and takes no requests from
 * then on. A queue head leaves the schedule, or has its overlay emptied,
 * while the asynchronous schedule is stopped. A controller that halts,
 * reports a host system error, or does not stop or start its asynchronous
 * schedule takes no more requests and keeps all its memory, as does one
 * that stop cannot halt. A device whose enumeration fails is left on a
 * disabled port. If that cannot be confirmed, nothing more is enumerated
 * until its hub goes, or its root port is disconnected.
 */
class EHCIController : public PIEUSBHost
{
	OSDeclareDefaultStructors(EHCIController);

	IOService *nub;
	IOMemoryMap *regsMap;
	uintptr_t cap, op;
	uint32_t nPorts;
	bool portPower;
	IOWorkLoop *workLoop;
	IOInterruptEventSource *intSource;

	IOBufferMemoryDescriptor *poolMem;
	IOMemoryMap *poolMap;
	uint8_t *pool;
	uint32_t poolPhys;
	size_t poolUsed;
	IOLock *dmaLock;
	struct {
		void *virt;
		uint32_t phys;
		size_t size;
	} dmaFree[128];
	uint32_t dmaFreeCount;

	struct EHCIQH *asyncHead;
	struct EHCIDevice *devices;
	uint32_t nextGeneration;
	struct EHCIPipe *activeQueues;
	bool dead, halted;

	IOLock *lock;
	IOLock *waitLock;
	uint32_t waitEvents;
	IOLock *portLock;
	bool portsChanged, stopping, threadRunning;
	uint32_t portDevice[16];
	uint32_t holdHub, holdPort;

	uint32_t r32(uintptr_t base, uint32_t off) { return *(volatile uint32_t *)(base + off); }
	void w32(uintptr_t base, uint32_t off, uint32_t v) { *(volatile uint32_t *)(base + off) = v; }

	void *allocDMA(size_t size, size_t align, uint32_t *phys);
	void freeDMA(void *virt, uint32_t phys, size_t size);
	struct EHCIQH *newQH(uint8_t address, uint8_t endpoint, uint16_t maxPacket, bool control);
	void freeQH(struct EHCIQH *qh);
	void linkQH(struct EHCIQH *qh);
	void unlinkEntry(struct EHCIQH *qh);
	bool unlinkQH(struct EHCIQH *qh);
	bool waitAsync(bool on);
	bool pauseAsync(void);
	bool resumeAsync(void);
	void freePipe(struct EHCIPipe *p);
	bool working(void);
	IOReturn waitTransfer(uint32_t slot, volatile void *tds, uint32_t count, bool shortEnds, uint32_t timeoutMS);
	IOReturn controlLocked(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
	    uint16_t wLength, void *data, uint32_t *actual);
	bool quiesce(struct EHCIQH *qh, uint32_t keep);
	void controllerDied(const char *why, uint32_t sts);
	void haltDied(uint32_t sts);
	bool haltController(void);

	void processQueue(struct EHCIPipe *p);
	void armIn(struct EHCIPipe *p, uint32_t b);
	static IOReturn gatedQueueAdd(OSObject *owner, void *pipe, void *arg1, void *arg2, void *arg3);
	static IOReturn gatedQueueTake(OSObject *owner, void *slot, void *index, void *generation, void *out);
	static IOReturn gatedQueueOut(OSObject *owner, void *args, void *arg1, void *arg2, void *arg3);
	static IOReturn gatedDeviceTake(OSObject *owner, void *slot, void *out, void *arg2, void *arg3);

	bool resetController(void);
	void interruptOccurred(IOInterruptEventSource *src, int count);
	static void bringUp(void *arg, wait_result_t wr);
	bool startController(void);
	void scanPorts(void);
	void attachPort(uint32_t port);
	uint32_t attachDevice(uint8_t rootPort, uint8_t depth);
	void detachDevice(uint32_t slot);
	IOReturn controlReadExact(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
	    uint16_t wIndex, uint16_t wLength, void *data);
	void setupHub(uint32_t slot);
	bool disableHubPort(uint32_t slot, uint8_t port);
	bool hubPortStatus(uint32_t slot, uint8_t port, uint16_t *status, uint16_t *change);
	void scanHub(uint32_t slot);

protected:
	virtual bool platformStart(IOService *provider);
	bool stopRequested(void);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	virtual bool current(uint32_t slot, uint32_t generation) APPLE_KEXT_OVERRIDE;
	virtual IOReturn controlTransfer(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
	    uint16_t wIndex, uint16_t wLength, void *data, uint32_t *actual, uint32_t generation = 0) APPLE_KEXT_OVERRIDE;
	virtual IOReturn startInterruptIn(uint32_t slot, const struct PIEUSBEndpoint *ep, OSObject *target,
	    PIEUSBInterruptAction action, uint32_t generation = 0) APPLE_KEXT_OVERRIDE;
	virtual IOReturn bulkTransfer(uint32_t slot, const struct PIEUSBEndpoint *ep, void *data, uint32_t length,
	    uint32_t *actual, uint32_t generation = 0) APPLE_KEXT_OVERRIDE;
	virtual IOReturn resetPipe(uint32_t slot, uint8_t endpointAddress, uint32_t generation = 0) APPLE_KEXT_OVERRIDE;
	virtual IOReturn startQueue(uint32_t slot, const struct PIEUSBEndpoint *ep, uint32_t size, uint32_t count,
	    OSObject *target, PIEUSBQueueAction action, uint32_t generation) APPLE_KEXT_OVERRIDE;
	virtual IOReturn queueOut(uint32_t slot, uint8_t endpointAddress, const void *data, uint32_t length,
	    uint32_t generation) APPLE_KEXT_OVERRIDE;
	virtual IOReturn stopQueue(uint32_t slot, uint8_t endpointAddress, uint32_t generation) APPLE_KEXT_OVERRIDE;
};

#endif
