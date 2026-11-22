/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/OSByteOrder.h>

#include "../rpi/RPiPlatform.h"
#include "PIEUSB.h"

enum {
	CAP_CAPLENGTH = 0x00, CAP_HCSPARAMS1 = 0x04, CAP_HCSPARAMS2 = 0x08,
	CAP_HCCPARAMS1 = 0x10, CAP_DBOFF = 0x14, CAP_RTSOFF = 0x18,

	OP_USBCMD = 0x00, OP_USBSTS = 0x04, OP_PAGESIZE = 0x08, OP_CRCR = 0x18,
	OP_DCBAAP = 0x30, OP_CONFIG = 0x38, OP_PORTSC = 0x400,

	RT_IMAN = 0x20, RT_IMOD = 0x24, RT_ERSTSZ = 0x28, RT_ERSTBA = 0x30, RT_ERDP = 0x38,
};

enum {
	USBCMD_RS = 1U << 0, USBCMD_HCRST = 1U << 1, USBCMD_INTE = 1U << 2,
	CRCR_CA = 1U << 2, CRCR_CRR = 1U << 3, CRCR_RSVD = 3U << 4,
	USBSTS_HCH = 1U << 0, USBSTS_EINT = 1U << 3, USBSTS_PCD = 1U << 4, USBSTS_CNR = 1U << 11,
	IMAN_IP = 1U << 0, IMAN_IE = 1U << 1,
	ERDP_EHB = 1U << 3,

	PORTSC_CCS = 1U << 0, PORTSC_PED = 1U << 1, PORTSC_PR = 1U << 4, PORTSC_PP = 1U << 9,
	PORTSC_SPEED_SHIFT = 10, PORTSC_CSC = 1U << 17, PORTSC_PEC = 1U << 18, PORTSC_PRC = 1U << 21,
	PORTSC_CHANGES = 0x00fe0000,
};

enum {
	TRB_CYCLE = 1U << 0, TRB_TOGGLE = 1U << 1, TRB_ISP = 1U << 2, TRB_IOC = 1U << 5,
	TRB_IDT = 1U << 6, TRB_BSR = 1U << 9, TRB_DIR_IN = 1U << 16,
	TRB_TYPE_SHIFT = 10,

	TRB_NORMAL = 1, TRB_SETUP = 2, TRB_DATA = 3, TRB_STATUS = 4, TRB_LINK = 6,
	TRB_ENABLE_SLOT = 9, TRB_ADDRESS_DEVICE = 11, TRB_CONFIGURE_EP = 12, TRB_EVALUATE_CONTEXT = 13,
	TRB_EV_TRANSFER = 32, TRB_EV_COMMAND = 33, TRB_EV_PORT = 34,

	TRB_DISABLE_SLOT = 10, TRB_RESET_EP = 14, TRB_STOP_EP = 15, TRB_SET_TR_DEQUEUE = 16, TRB_NOOP_CMD = 23,
	CC_SUCCESS = 1, CC_STALL = 6, CC_SHORT_PACKET = 13, CC_RING_STOPPED = 24, CC_COMMAND_ABORTED = 25,
	CC_GONE = 0x100,

	EP_STATE_RUNNING = 1, EP_STATE_HALTED = 2,

	EP_BULK_OUT = 2, EP_CONTROL = 4, EP_BULK_IN = 6, EP_INTERRUPT_IN = 7,
	SPEED_FULL = 1, SPEED_LOW = 2, SPEED_HIGH = 3, SPEED_SUPER = 4,
};

struct TRB {
	uint64_t param;
	uint32_t status;
	uint32_t control;
};

#define DWC3_GCTL               0xc110
#define DWC3_GCTL_PRTCAP_HOST   1U
#define DWC3_GSNPSID            0xc120

#define RING_TRBS       256
#define POOL_SIZE       (2 * 1024 * 1024)
#define MAX_SLOTS       32
#define MAX_DCI         32
#define CMD_TIMEOUT_MS  2000
#define ABORT_TIMEOUT_MS 5000
#define WAIT_RING_STOPPED 1
#define BULK_TIMEOUT_MS 10000
#define DMA_FREE_MAX    128

struct Ring {
	volatile struct TRB *trbs;
	uint64_t phys;
	uint32_t enqueue;
	uint32_t cycle;
};

struct InterruptPipe {
	struct Ring ring;
	uint8_t *buffer;
	uint64_t bufferPhys;
	uint16_t length;
	OSObject *target;
	PIEUSBInterruptAction action;
};

#define INTERRUPT_BUFFER(pipe)  ((pipe)->length ? (pipe)->length : 64U)

#define HUB_PORTS       15

#define BULK_MAX        65536
struct BulkPipe {
	struct Ring ring;
	uint8_t *buffer;
	uint64_t bufferPhys;
	struct PIEUSBEndpoint ep;
};

#define QUEUE_BUFFERS_MAX      32
#define QUEUE_BYTES_MAX        (256 * 1024)
#define QUEUE_NONE             0xff
#define QUEUE_NO_EVENT         0x80

struct BulkQueue {
	struct Ring ring;
	IOBufferMemoryDescriptor *mem;
	IOMemoryMap *map;
	uint8_t *buffers;
	uint64_t buffersPhys;
	uint32_t size, count, dci;
	uint32_t busy;
	bool in, halted;
	struct PIEUSBEndpoint ep;
	OSObject *target;
	PIEUSBQueueAction action;
	uint32_t lengths[QUEUE_BUFFERS_MAX];
	uint8_t owner[RING_TRBS];
	struct BulkQueue *next;
};

struct Device {
	bool used;
	bool gone;
	bool broken;
	bool timedOut;
	uint32_t generation;
	uint8_t port, speed;
	uint32_t route;
	uint8_t depth;
	uint8_t ttSlot, ttPort;
	bool isHub;
	uint8_t hubPorts;
	uint8_t hubChild[HUB_PORTS + 1];
	struct PIEUSBEndpoint hubStatus;
	uint8_t *outCtx;
	uint8_t *inCtx;
	uint64_t outPhys, inPhys;
	struct Ring ep0;
	uint8_t *ep0Buf;
	uint64_t ep0BufPhys;
	struct InterruptPipe *pipes[MAX_DCI];
	struct BulkPipe *bulk[MAX_DCI];
	struct BulkQueue *queues[MAX_DCI];
	struct BulkQueue *dead;
	uint8_t maxDCI;
};

static bool
kept_queue(const struct Device *d, uint32_t dci)
{
	for (const struct BulkQueue *s = d->dead; s != NULL; s = s->next) {
		if (s->dci == dci) {
			return true;
		}
	}
	return false;
}

class XHCIController : public PIEUSBHost
{
	OSDeclareDefaultStructors(XHCIController);

	IOService *nub;
	IOMemoryMap *regsMap;
	uintptr_t cap, op, rt, db;
	uint32_t maxSlots, maxPorts, ctxSize;
	IOWorkLoop *workLoop;
	IOInterruptEventSource *intSource;

	IOBufferMemoryDescriptor *poolMem;
	IOMemoryMap *poolMap;
	uint8_t *pool;
	uint64_t poolPhys;
	size_t poolUsed;
	IOLock *dmaLock;
	struct {
		void *virt;
		uint64_t phys;
		size_t size;
	} dmaFree[DMA_FREE_MAX];
	uint32_t dmaFreeCount;
	uint32_t nextGeneration;

	uint64_t *dcbaa;
	struct Ring cmdRing;
	volatile struct TRB *events;
	uint64_t eventsPhys;
	uint32_t eventDequeue, eventCycle;

	struct Device *devices;

	IOLock *waitLock;
	IOLock *cmdLock;
	uint64_t waitTRB, waitData;
	uint32_t waitOnSlot, waitOnDCI;
	bool waitDone;
	uint32_t waitCode, waitResidual, waitDataResidual;
	uint64_t cmdTRB;
	uint32_t cmdCode, cmdSlot, cmdConsumer;
	bool cmdDone;
	bool cmdDead;

	IOLock *portLock;
	bool portsChanged;
	bool stopping;
	uint32_t portSlot[256];

	uint32_t r32(uintptr_t base, uint32_t off) { return *(volatile uint32_t *)(base + off); }
	void w32(uintptr_t base, uint32_t off, uint32_t v) { *(volatile uint32_t *)(base + off) = v; }
	void w64(uintptr_t base, uint32_t off, uint64_t v)
	{
		w32(base, off, (uint32_t)v);
		w32(base, off + 4, (uint32_t)(v >> 32));
	}

	void *allocDMA(size_t size, size_t align, uint64_t *phys);
	void freeDMA(void *virt, uint64_t phys, size_t size);
	void freePipe(struct InterruptPipe *pipe);
	void freeBulk(struct BulkPipe *pipe);
	void freeQueue(struct BulkQueue *s);
	static IOReturn gatedQueueKeep(OSObject *owner, void *queue, void *slot, void *arg2, void *arg3);
	static IOReturn gatedPipeStart(OSObject *owner, void *pipe, void *slot, void *dci, void *arg3);
	void queueIn(struct BulkQueue *s, uint32_t b);
	void queueEvent(uint32_t slot, struct BulkQueue *s, uint64_t trb, uint32_t code, uint32_t residual);
	static IOReturn gatedQueueStart(OSObject *owner, void *queue, void *slot, void *arg2, void *arg3);
	static IOReturn gatedQueueTake(OSObject *owner, void *slot, void *dci, void *generation, void *out);
	static IOReturn gatedQueueOut(OSObject *owner, void *args, void *arg1, void *arg2, void *arg3);
	static IOReturn gatedEvents(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3);
	void initRing(struct Ring *ring);
	uint64_t ringPut(struct Ring *ring, uint64_t param, uint32_t status, uint32_t control);
	IOReturn command(uint64_t param, uint32_t status, uint32_t control, uint32_t *slotOut);
	IOReturn commandLocked(uint64_t param, uint32_t status, uint32_t control, uint32_t *slotOut);
	IOReturn addEndpointLocked(uint32_t slot, uint32_t dci, uint32_t type, const struct PIEUSBEndpoint *ep,
	    uint64_t dequeue, uint32_t interval, uint32_t avgTRB, bool replace);
	IOReturn waitFor(uint32_t timeoutMS, uint32_t *code, uint32_t *residual, uint32_t *dataResidual = NULL);
	IOReturn abortCommandLocked(uint64_t trb);
	void noOpCommand(uint32_t index);
	bool roomForCommandLocked(void);
	IOReturn stopEndpointLocked(uint32_t slot, uint32_t dci);
	void expect(uint64_t trb, uint32_t slot, uint32_t dci, uint64_t data = 0);
	uint32_t *slotContext(uint8_t *ctx, bool input) { return (uint32_t *)(void *)(ctx + (input ? ctxSize : 0)); }
	uint32_t *endpointContext(uint8_t *ctx, bool input, unsigned dci)
	{
		return (uint32_t *)(void *)(ctx + (input ? ctxSize : 0) + dci * ctxSize);
	}

	bool resetController(void);
	void interruptOccurred(IOInterruptEventSource *src, int count);
	static void enumerationThread(void *arg, wait_result_t wr);
	static void hubStatusChanged(OSObject *target, const uint8_t *data, uint32_t length);
	void scanPorts(void);
	void attachPort(uint32_t port);
	uint32_t attachDevice(uint8_t rootPort, uint32_t route, uint8_t depth, uint8_t speed, uint32_t parentSlot,
	    uint8_t parentPort);
	void detachDevice(uint32_t slot);
	IOReturn controlReadExact(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
	    uint16_t wIndex, uint16_t wLength, void *data);
	void setupHub(uint32_t slot, const uint8_t *cfg, uint32_t cfgLength);
	void scanHub(uint32_t slot);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;

	virtual bool current(uint32_t slot, uint32_t generation) APPLE_KEXT_OVERRIDE
	{
		return slot >= 1 && slot <= MAX_SLOTS && devices[slot].used && !devices[slot].gone &&
		       !devices[slot].broken && (generation == 0 || devices[slot].generation == generation);
	}

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

#define super PIEUSBHost
OSDefineMetaClassAndStructors(XHCIController, PIEUSBHost);

#pragma mark DMA pool and rings

void *
XHCIController::allocDMA(size_t size, size_t align, uint64_t *phys)
{
	void *virt = NULL;
	IOLockLock(dmaLock);
	for (uint32_t i = 0; i < dmaFreeCount; i++) {
		if (dmaFree[i].size == size && (dmaFree[i].phys & (align - 1)) == 0) {
			virt = dmaFree[i].virt;
			*phys = dmaFree[i].phys;
			dmaFree[i] = dmaFree[--dmaFreeCount];
			break;
		}
	}
	if (virt == NULL) {
		size_t start = (size_t)(((poolPhys + poolUsed + align - 1) & ~(uint64_t)(align - 1)) - poolPhys);
		if (start + size <= POOL_SIZE) {
			poolUsed = start + size;
			virt = pool + start;
			*phys = poolPhys + start;
		}
	}
	IOLockUnlock(dmaLock);
	if (virt != NULL) {
		bzero(virt, size);
	}
	return virt;
}

void
XHCIController::freeDMA(void *virt, uint64_t phys, size_t size)
{
	if (virt == NULL) {
		return;
	}
	IOLockLock(dmaLock);
	if (dmaFreeCount < DMA_FREE_MAX) {
		dmaFree[dmaFreeCount].virt = virt;
		dmaFree[dmaFreeCount].phys = phys;
		dmaFree[dmaFreeCount].size = size;
		dmaFreeCount++;
	}
	IOLockUnlock(dmaLock);
}

void
XHCIController::initRing(struct Ring *ring)
{
	ring->enqueue = 0;
	ring->cycle = 1;
	volatile struct TRB *link = &ring->trbs[RING_TRBS - 1];
	link->param = ring->phys;
	link->status = 0;
	link->control = TRB_LINK << TRB_TYPE_SHIFT | TRB_TOGGLE;
}

uint64_t
XHCIController::ringPut(struct Ring *ring, uint64_t param, uint32_t status, uint32_t control)
{
	volatile struct TRB *t = &ring->trbs[ring->enqueue];
	uint64_t phys = ring->phys + ring->enqueue * sizeof(struct TRB);
	t->param = param;
	t->status = status;
	__builtin_arm_dmb(0xf);
	t->control = (control & ~TRB_CYCLE) | ring->cycle;
	__builtin_arm_dmb(0xf);
	if (++ring->enqueue == RING_TRBS - 1) {
		volatile struct TRB *link = &ring->trbs[RING_TRBS - 1];
		link->control = (link->control & ~TRB_CYCLE) | ring->cycle;
		ring->cycle ^= 1;
		ring->enqueue = 0;
	}
	return phys;
}

#pragma mark Commands and completions

void
XHCIController::expect(uint64_t trb, uint32_t slot, uint32_t dci, uint64_t data)
{
	waitDone = false;
	waitTRB = trb;
	waitOnSlot = slot;
	waitOnDCI = dci;
	waitData = data;
	waitDataResidual = 0;
}

IOReturn
XHCIController::waitFor(uint32_t timeoutMS, uint32_t *code, uint32_t *residual, uint32_t *dataResidual)
{
	IOLockLock(waitLock);
	uint64_t deadline;
	clock_interval_to_deadline(timeoutMS, kMillisecondScale, &deadline);
	while (!waitDone) {
		if (IOLockSleepDeadline(waitLock, &waitDone, deadline, THREAD_UNINT) == THREAD_TIMED_OUT) {
			break;
		}
	}
	bool done = waitDone;
	*code = waitCode;
	if (residual != NULL) {
		*residual = waitResidual;
	}
	if (dataResidual != NULL) {
		*dataResidual = waitDataResidual;
	}
	waitTRB = 0;
	waitData = 0;
	waitDone = false;
	IOLockUnlock(waitLock);
	return done ? kIOReturnSuccess : kIOReturnTimeout;
}

IOReturn
XHCIController::command(uint64_t param, uint32_t status, uint32_t control, uint32_t *slotOut)
{
	IOLockLock(cmdLock);
	IOReturn ret = commandLocked(param, status, control, slotOut);
	IOLockUnlock(cmdLock);
	return ret;
}

IOReturn
XHCIController::commandLocked(uint64_t param, uint32_t status, uint32_t control, uint32_t *slotOut)
{
	if (cmdDead) {
		return kIOReturnNotResponding;
	}
	if (!roomForCommandLocked()) {
		IOLog("XHCI: the command ring is full\n");
		return kIOReturnNoResources;
	}
	IOLockLock(waitLock);
	uint64_t trb = ringPut(&cmdRing, param, status, control);
	cmdTRB = trb;
	cmdDone = false;
	expect(trb, 0, 0);
	IOLockUnlock(waitLock);
	w32(db, 0, 0);
	uint32_t code = 0;
	IOReturn ret = waitFor(CMD_TIMEOUT_MS, &code, NULL);
	if (ret != kIOReturnSuccess) {
		workLoop->runAction(&XHCIController::gatedEvents, this);
		IOLockLock(waitLock);
		if (cmdDone) {
			ret = kIOReturnSuccess;
		}
		IOLockUnlock(waitLock);
	}
	if (ret != kIOReturnSuccess) {
		uint32_t slot = control >> 24;
		IOLog("XHCI: command type %u timed out\n", (control >> TRB_TYPE_SHIFT) & 0x3f);
		ret = abortCommandLocked(trb);
		if (ret == kIOReturnNotResponding) {
			IOLog("XHCI: the command ring did not stop; the controller takes no more commands\n");
			cmdDead = true;
			for (uint32_t i = 1; i <= MAX_SLOTS; i++) {
				devices[i].broken = devices[i].broken || devices[i].used;
			}
			if (slot >= 1 && slot <= MAX_SLOTS && devices[slot].used) {
				devices[slot].timedOut = true;
			}
			return kIOReturnTimeout;
		}
		if (ret != kIOReturnSuccess) {
			return ret;
		}
	}
	IOLockLock(waitLock);
	code = cmdCode;
	if (slotOut != NULL) {
		*slotOut = cmdSlot;
	}
	IOLockUnlock(waitLock);
	if (code != CC_SUCCESS) {
		IOLog("XHCI: command type %u failed, completion code %u\n", (control >> TRB_TYPE_SHIFT) & 0x3f, code);
		return kIOReturnError;
	}
	return kIOReturnSuccess;
}

IOReturn
XHCIController::abortCommandLocked(uint64_t trb)
{
	if (r32(op, OP_USBSTS) & USBSTS_HCH) {
		return kIOReturnNotResponding;
	}
	uint32_t crcr = r32(op, OP_CRCR);
	if (crcr & CRCR_CRR) {
		IOLockLock(waitLock);
		expect(WAIT_RING_STOPPED, 0, 0);
		IOLockUnlock(waitLock);
		w64(op, OP_CRCR, (cmdRing.phys + (cmdRing.enqueue & ~3U) * sizeof(struct TRB)) | (crcr & CRCR_RSVD) |
		    cmdRing.cycle | CRCR_CA);
		for (uint32_t ms = 0; (r32(op, OP_CRCR) & CRCR_CRR) && ms < ABORT_TIMEOUT_MS; ms++) {
			IOSleep(1);
		}
		if ((r32(op, OP_CRCR) & CRCR_CRR) || (r32(op, OP_USBSTS) & USBSTS_HCH)) {
			IOLockLock(waitLock);
			waitTRB = 0;
			IOLockUnlock(waitLock);
			return kIOReturnNotResponding;
		}
		uint32_t code = 0;
		waitFor(CMD_TIMEOUT_MS, &code, NULL);
		if (r32(op, OP_USBSTS) & USBSTS_HCH) {
			return kIOReturnNotResponding;
		}
	}
	workLoop->runAction(&XHCIController::gatedEvents, this);
	IOLockLock(waitLock);
	bool done = cmdDone && cmdCode != CC_COMMAND_ABORTED;
	if (!cmdDone) {
		noOpCommand((uint32_t)((trb - cmdRing.phys) / sizeof(struct TRB)));
	}
	for (uint32_t i = cmdRing.enqueue & ~3U; i < cmdRing.enqueue; i++) {
		noOpCommand(i);
	}
	IOLockUnlock(waitLock);
	return done ? kIOReturnSuccess : kIOReturnAborted;
}

bool
XHCIController::roomForCommandLocked(void)
{
	for (int pass = 0; pass < 2; pass++) {
		IOLockLock(waitLock);
		bool room = (cmdRing.enqueue + 1) % (RING_TRBS - 1) != cmdConsumer;
		IOLockUnlock(waitLock);
		if (room) {
			return true;
		}
		if (pass == 0) {
			workLoop->runAction(&XHCIController::gatedEvents, this);
		}
	}
	uint32_t crcr = r32(op, OP_CRCR);
	if ((crcr & CRCR_CRR) || (r32(op, OP_USBSTS) & USBSTS_HCH)) {
		return false;
	}
	IOLockLock(waitLock);
	bzero((void *)(uintptr_t)cmdRing.trbs, RING_TRBS * sizeof(struct TRB));
	initRing(&cmdRing);
	cmdConsumer = 0;
	IOLockUnlock(waitLock);
	__builtin_arm_dmb(0xf);
	w64(op, OP_CRCR, cmdRing.phys | (crcr & CRCR_RSVD) | cmdRing.cycle);
	return true;
}

void
XHCIController::noOpCommand(uint32_t index)
{
	volatile struct TRB *t = &cmdRing.trbs[index];
	t->param = 0;
	t->status = 0;
	__builtin_arm_dmb(0xf);
	t->control = (t->control & TRB_CYCLE) | TRB_NOOP_CMD << TRB_TYPE_SHIFT;
	__builtin_arm_dmb(0xf);
}

IOReturn
XHCIController::stopEndpointLocked(uint32_t slot, uint32_t dci)
{
	struct Device *d = &devices[slot];
	IOReturn ret = kIOReturnNotResponding;
	for (int tries = 0; tries < 3; tries++) {
		uint32_t state = endpointContext(d->outCtx, false, dci)[0] & 7;
		if (state == EP_STATE_HALTED) {
			ret = commandLocked(0, 0, TRB_RESET_EP << TRB_TYPE_SHIFT | slot << 24 | dci << 16, NULL);
		} else if (state == EP_STATE_RUNNING) {
			ret = commandLocked(0, 0, TRB_STOP_EP << TRB_TYPE_SHIFT | slot << 24 | dci << 16, NULL);
		} else {
			return kIOReturnSuccess;
		}
		if (ret == kIOReturnSuccess) {
			return kIOReturnSuccess;
		}
	}
	return ret;
}

void
XHCIController::interruptOccurred(IOInterruptEventSource *src, int count)
{
	(void)src;
	(void)count;
	w32(op, OP_USBSTS, USBSTS_EINT | USBSTS_PCD);
	w32(rt, RT_IMAN, IMAN_IE | IMAN_IP);

	bool any = false;
	for (;;) {
		volatile struct TRB *e = &events[eventDequeue];
		uint32_t control = e->control;
		if ((control & TRB_CYCLE) != eventCycle) {
			break;
		}
		__builtin_arm_dmb(0xf);
		uint32_t type = (control >> TRB_TYPE_SHIFT) & 0x3f;
		uint32_t code = e->status >> 24;
		uint32_t slot = control >> 24;
		uint32_t dci = (control >> 16) & 0x1f;

		if (type == TRB_EV_COMMAND || type == TRB_EV_TRANSFER) {
			struct InterruptPipe *pipe = NULL;
			struct BulkQueue *queue = NULL;
			if (type == TRB_EV_TRANSFER && slot <= MAX_SLOTS && dci < MAX_DCI && dci > 1) {
				pipe = devices[slot].pipes[dci];
				queue = devices[slot].queues[dci];
			}
			if (queue != NULL) {
				queueEvent(slot, queue, e->param, code, e->status & 0xffffff);
			} else if (pipe != NULL && pipe->action != NULL) {
				uint32_t residual = e->status & 0xffffff;
				if ((code == CC_SUCCESS || code == CC_SHORT_PACKET) && residual <= pipe->length) {
					pipe->action(pipe->target, pipe->buffer, pipe->length - residual);
				}
				ringPut(&pipe->ring, pipe->bufferPhys, pipe->length, TRB_NORMAL << TRB_TYPE_SHIFT | TRB_IOC | TRB_ISP);
				w32(db, 4 * slot, dci);
			} else {
				IOLockLock(waitLock);
				bool match;
				if (type == TRB_EV_COMMAND) {
					uint64_t off = e->param - cmdRing.phys;
					if (code != CC_RING_STOPPED && off < (RING_TRBS - 1) * sizeof(struct TRB)) {
						cmdConsumer = (uint32_t)(off / sizeof(struct TRB) + 1) % (RING_TRBS - 1);
					}
					if (code != CC_RING_STOPPED && e->param == cmdTRB) {
						cmdDone = true;
						cmdCode = code;
						cmdSlot = slot;
					}
					match = waitOnDCI == 0 && (waitTRB == WAIT_RING_STOPPED ? code == CC_RING_STOPPED :
					    code != CC_RING_STOPPED && e->param == waitTRB);
				} else {
					bool waiter = waitOnDCI != 0 && slot == waitOnSlot && dci == waitOnDCI;
					if (waiter && waitData != 0 && e->param == waitData &&
					    (code == CC_SUCCESS || code == CC_SHORT_PACKET)) {
						waitDataResidual = e->status & 0xffffff;
					}
					match = waiter && (e->param == waitTRB || (code != CC_SUCCESS && code != CC_SHORT_PACKET));
				}
				if (waitTRB != 0 && match) {
					waitCode = code;
					waitResidual = e->status & 0xffffff;
					waitDone = true;
					IOLockWakeup(waitLock, &waitDone, true);
				}
				IOLockUnlock(waitLock);
			}
		} else if (type == TRB_EV_PORT) {
			IOLockLock(portLock);
			portsChanged = true;
			IOLockWakeup(portLock, &portsChanged, true);
			IOLockUnlock(portLock);
		}
		any = true;
		if (++eventDequeue == RING_TRBS) {
			eventDequeue = 0;
			eventCycle ^= 1;
		}
	}
	if (any) {
		w64(rt, RT_ERDP, (eventsPhys + eventDequeue * sizeof(struct TRB)) | ERDP_EHB);
	}
}

IOReturn
XHCIController::gatedEvents(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{
	(void)arg0;
	(void)arg1;
	(void)arg2;
	(void)arg3;
	((XHCIController *)owner)->interruptOccurred(NULL, 0);
	return kIOReturnSuccess;
}

#pragma mark Transfers

IOReturn
XHCIController::controlTransfer(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
    uint16_t wIndex, uint16_t wLength, void *data, uint32_t *actual, uint32_t generation)
{
	if (!current(slot, generation)) {
		return kIOReturnNoDevice;
	}
	if (wLength > 4096) {
		return kIOReturnBadArgument;
	}
	struct Device *d = &devices[slot];
	bool in = (bmRequestType & 0x80) != 0;
	uint64_t setup = (uint64_t)bmRequestType | (uint64_t)bRequest << 8 | (uint64_t)wValue << 16 |
	    (uint64_t)wIndex << 32 | (uint64_t)wLength << 48;
	uint32_t trt = wLength == 0 ? 0 : (in ? 3U : 2U);

	IOLockLock(cmdLock);
	if (!current(slot, generation)) {
		IOLockUnlock(cmdLock);
		return kIOReturnNoDevice;
	}
	if (!in && wLength > 0) {
		bcopy(data, d->ep0Buf, wLength);
	}
	IOLockLock(waitLock);
	ringPut(&d->ep0, setup, 8, TRB_SETUP << TRB_TYPE_SHIFT | TRB_IDT | trt << 16);
	uint64_t dataTRB = 0;
	if (wLength > 0) {
		dataTRB = ringPut(&d->ep0, d->ep0BufPhys, wLength,
		    TRB_DATA << TRB_TYPE_SHIFT | (in ? TRB_DIR_IN | TRB_ISP : 0));
	}
	uint64_t status = ringPut(&d->ep0, 0, 0,
	    TRB_STATUS << TRB_TYPE_SHIFT | TRB_IOC | ((in && wLength > 0) ? 0 : TRB_DIR_IN));
	expect(status, slot, 1, dataTRB);
	IOLockUnlock(waitLock);
	w32(db, 4 * slot, 1);

	uint32_t code = 0, residual = 0;
	IOReturn ret = waitFor(CMD_TIMEOUT_MS, &code, NULL, &residual);
	uint32_t moved = residual <= wLength ? wLength - residual : 0;
	if (current(slot, generation) && (ret != kIOReturnSuccess || code == CC_STALL ||
	    (endpointContext(d->outCtx, false, 1)[0] & 7) == EP_STATE_HALTED)) {
		uint64_t deq = d->ep0.phys + d->ep0.enqueue * sizeof(struct TRB);
		if (stopEndpointLocked(slot, 1) != kIOReturnSuccess ||
		    commandLocked(deq | d->ep0.cycle, 0, TRB_SET_TR_DEQUEUE << TRB_TYPE_SHIFT | slot << 24 | 1U << 16,
		    NULL) != kIOReturnSuccess) {
			d->broken = true;
			IOLog("XHCI: slot %u: ep0 did not stop; the device takes no more requests\n", slot);
		}
	} else if (ret == kIOReturnSuccess && (code == CC_SUCCESS || code == CC_SHORT_PACKET) && in && wLength > 0) {
		bcopy(d->ep0Buf, data, moved);
		bzero((uint8_t *)data + moved, wLength - moved);
	}
	IOLockUnlock(cmdLock);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	if (code == CC_GONE) {
		return kIOReturnNoDevice;
	}
	if (code == CC_STALL) {
		return kIOReturnNotResponding;
	}
	if (code != CC_SUCCESS && code != CC_SHORT_PACKET) {
		return kIOReturnError;
	}
	if (actual != NULL) {
		*actual = moved;
	}
	return kIOReturnSuccess;
}

static uint32_t
interrupt_interval(uint8_t speed, uint8_t bInterval)
{
	if (speed == SPEED_HIGH || speed == SPEED_SUPER) {
		return bInterval >= 1 && bInterval <= 16 ? bInterval - 1U : 3U;
	}
	uint32_t frames = bInterval ? bInterval : 1, exp = 3;
	while ((1U << (exp + 1)) <= frames * 8 && exp < 10) {
		exp++;
	}
	return exp;
}

IOReturn
XHCIController::startInterruptIn(uint32_t slot, const struct PIEUSBEndpoint *ep, OSObject *target,
    PIEUSBInterruptAction action, uint32_t generation)
{
	if (!current(slot, generation)) {
		return kIOReturnNoDevice;
	}
	if (!(ep->address & 0x80) || (ep->address & 0xf) == 0) {
		return kIOReturnBadArgument;
	}
	struct Device *d = &devices[slot];
	uint32_t dci = (ep->address & 0xf) * 2U + 1;
	if (dci >= MAX_DCI || d->pipes[dci] != NULL) {
		return kIOReturnBadArgument;
	}
	struct InterruptPipe *pipe = IOMallocType(struct InterruptPipe);
	pipe->ring.trbs = (volatile struct TRB *)allocDMA(RING_TRBS * sizeof(struct TRB), 4096, &pipe->ring.phys);
	pipe->length = ep->maxPacket & 0x7ff;
	size_t align = 64;
	while (align < INTERRUPT_BUFFER(pipe)) {
		align <<= 1;
	}
	pipe->buffer = (uint8_t *)allocDMA(INTERRUPT_BUFFER(pipe), align, &pipe->bufferPhys);
	if (pipe->ring.trbs == NULL || pipe->buffer == NULL || pipe->length == 0) {
		freePipe(pipe);
		return kIOReturnNoMemory;
	}
	initRing(&pipe->ring);
	pipe->target = target;
	pipe->action = action;

	IOReturn ret;
	IOLockLock(cmdLock);
	if (!current(slot, generation)) {
		ret = kIOReturnNoDevice;
	} else if (d->pipes[dci] != NULL) {
		ret = kIOReturnBadArgument;
	} else {
		ret = addEndpointLocked(slot, dci, EP_INTERRUPT_IN, ep, pipe->ring.phys | 1,
		    interrupt_interval(d->speed, ep->interval), pipe->length, false);
		if (ret == kIOReturnTimeout) {
			pipe->action = NULL;
		}
		if (ret == kIOReturnSuccess || ret == kIOReturnTimeout) {
			workLoop->runAction(&XHCIController::gatedPipeStart, this, pipe, (void *)(uintptr_t)slot,
			    (void *)(uintptr_t)dci);
		}
	}
	IOLockUnlock(cmdLock);
	if (ret != kIOReturnSuccess && ret != kIOReturnTimeout) {
		freePipe(pipe);
	}
	return ret;
}

IOReturn
XHCIController::gatedPipeStart(OSObject *owner, void *pipeArg, void *slotArg, void *dciArg, void *arg3)
{
	(void)arg3;
	XHCIController *self = (XHCIController *)owner;
	struct InterruptPipe *pipe = (struct InterruptPipe *)pipeArg;
	uint32_t slot = (uint32_t)(uintptr_t)slotArg, dci = (uint32_t)(uintptr_t)dciArg;
	self->devices[slot].pipes[dci] = pipe;
	if (pipe->action != NULL) {
		self->ringPut(&pipe->ring, pipe->bufferPhys, pipe->length, TRB_NORMAL << TRB_TYPE_SHIFT | TRB_IOC | TRB_ISP);
		self->w32(self->db, 4 * slot, dci);
	}
	return kIOReturnSuccess;
}

IOReturn
XHCIController::addEndpointLocked(uint32_t slot, uint32_t dci, uint32_t type, const struct PIEUSBEndpoint *ep,
    uint64_t dequeue, uint32_t interval, uint32_t avgTRB, bool replace)
{
	struct Device *d = &devices[slot];
	bzero(d->inCtx, ctxSize * 33);
	uint32_t *icc = (uint32_t *)(void *)d->inCtx;
	icc[0] = replace ? 1U << dci : 0;
	icc[1] = 1U << 0 | 1U << dci;
	if (dci > d->maxDCI) {
		d->maxDCI = (uint8_t)dci;
	}
	uint32_t *slotIn = slotContext(d->inCtx, true);
	uint32_t *slotOut = slotContext(d->outCtx, false);
	for (int i = 0; i < 8; i++) {
		slotIn[i] = slotOut[i];
	}
	slotIn[0] = (slotIn[0] & ~(0x1fU << 27)) | (uint32_t)d->maxDCI << 27;
	uint32_t *epc = endpointContext(d->inCtx, true, dci);
	epc[0] = interval << 16;
	epc[1] = 3U << 1 | type << 3 | (uint32_t)(ep->maxPacket & 0x7ff) << 16;
	epc[2] = (uint32_t)dequeue;
	epc[3] = (uint32_t)(dequeue >> 32);
	epc[4] = avgTRB | (type == EP_INTERRUPT_IN ? avgTRB << 16 : 0);
	return commandLocked(d->inPhys, 0, TRB_CONFIGURE_EP << TRB_TYPE_SHIFT | slot << 24, NULL);
}

IOReturn
XHCIController::bulkTransfer(uint32_t slot, const struct PIEUSBEndpoint *ep, void *data, uint32_t length,
    uint32_t *actual, uint32_t generation)
{
	if (!current(slot, generation)) {
		return kIOReturnNoDevice;
	}
	if (length > BULK_MAX) {
		return kIOReturnBadArgument;
	}
	struct Device *d = &devices[slot];
	bool in = (ep->address & 0x80) != 0;
	uint32_t dci = (ep->address & 0xf) * 2U + (in ? 1 : 0);
	if (dci < 2 || dci >= MAX_DCI) {
		return kIOReturnBadArgument;
	}
	IOLockLock(cmdLock);
	if (!current(slot, generation)) {
		IOLockUnlock(cmdLock);
		return kIOReturnNoDevice;
	}
	if (d->queues[dci] != NULL || kept_queue(d, dci)) {
		IOLockUnlock(cmdLock);
		return kIOReturnBusy;
	}
	if (d->bulk[dci] == NULL) {
		struct BulkPipe *pipe = IOMallocType(struct BulkPipe);
		pipe->ring.trbs = (volatile struct TRB *)allocDMA(RING_TRBS * sizeof(struct TRB), 4096, &pipe->ring.phys);
		pipe->buffer = (uint8_t *)allocDMA(BULK_MAX, BULK_MAX, &pipe->bufferPhys);
		if (pipe->ring.trbs == NULL || pipe->buffer == NULL) {
			IOLockUnlock(cmdLock);
			freeBulk(pipe);
			return kIOReturnNoMemory;
		}
		pipe->ep = *ep;
		initRing(&pipe->ring);
		IOReturn ret = addEndpointLocked(slot, dci, in ? EP_BULK_IN : EP_BULK_OUT, ep, pipe->ring.phys | 1, 0,
		    3072, false);
		if (ret != kIOReturnSuccess && ret != kIOReturnTimeout) {
			IOLockUnlock(cmdLock);
			freeBulk(pipe);
			return ret;
		}
		d->bulk[dci] = pipe;
		if (ret != kIOReturnSuccess) {
			IOLockUnlock(cmdLock);
			return ret;
		}
	}
	struct BulkPipe *pipe = d->bulk[dci];
	if (!in && length > 0) {
		bcopy(data, pipe->buffer, length);
	}
	IOLockLock(waitLock);
	expect(ringPut(&pipe->ring, pipe->bufferPhys, length,
	    TRB_NORMAL << TRB_TYPE_SHIFT | TRB_IOC | (in ? TRB_ISP : 0)), slot, dci);
	IOLockUnlock(waitLock);
	w32(db, 4 * slot, dci);
	uint32_t code = 0, residual = 0;
	IOReturn ret = waitFor(BULK_TIMEOUT_MS, &code, &residual);
	uint32_t moved = residual <= length ? length - residual : 0;
	if (ret == kIOReturnSuccess && (code == CC_SUCCESS || code == CC_SHORT_PACKET) && in && moved > 0) {
		bcopy(pipe->buffer, data, moved);
	}
	IOLockUnlock(cmdLock);
	if (ret != kIOReturnSuccess) {
		resetPipe(slot, ep->address, generation);
		return ret;
	}
	if (code == CC_GONE) {
		return kIOReturnNoDevice;
	}
	if (code == CC_STALL) {
		return kIOReturnNotResponding;
	}
	if (code != CC_SUCCESS && code != CC_SHORT_PACKET) {
		IOLog("XHCI: slot %u endpoint 0x%x: completion code %u\n", slot, ep->address, code);
		resetPipe(slot, ep->address, generation);
		return kIOReturnIOError;
	}
	if (actual != NULL) {
		*actual = moved;
	}
	return kIOReturnSuccess;
}

IOReturn
XHCIController::resetPipe(uint32_t slot, uint8_t endpointAddress, uint32_t generation)
{
	uint32_t dci = (endpointAddress & 0xf) * 2U + ((endpointAddress & 0x80) ? 1 : 0);
	if (!current(slot, generation)) {
		return kIOReturnNoDevice;
	}
	if (dci < 2 || dci >= MAX_DCI) {
		return kIOReturnBadArgument;
	}
	IOLockLock(cmdLock);
	if (!current(slot, generation)) {
		IOLockUnlock(cmdLock);
		return kIOReturnNoDevice;
	}
	struct Device *d = &devices[slot];
	struct BulkPipe *pipe = d->bulk[dci];
	IOReturn ret = kIOReturnSuccess;
	if (pipe != NULL) {
		ret = stopEndpointLocked(slot, dci);
		if (ret == kIOReturnSuccess) {
			uint64_t deq = pipe->ring.phys + pipe->ring.enqueue * sizeof(struct TRB);
			ret = addEndpointLocked(slot, dci, (endpointAddress & 0x80) ? EP_BULK_IN : EP_BULK_OUT, &pipe->ep,
			    deq | pipe->ring.cycle, 0, 3072, true);
		}
		if (ret != kIOReturnSuccess) {
			d->broken = true;
			IOLog("XHCI: slot %u endpoint 0x%x did not reset; the device takes no more requests\n", slot,
			    endpointAddress);
		}
	}
	IOLockUnlock(cmdLock);
	return ret;
}

void
XHCIController::freePipe(struct InterruptPipe *pipe)
{
	freeDMA((void *)(uintptr_t)pipe->ring.trbs, pipe->ring.phys, RING_TRBS * sizeof(struct TRB));
	freeDMA(pipe->buffer, pipe->bufferPhys, INTERRUPT_BUFFER(pipe));
	IOFreeType(pipe, struct InterruptPipe);
}

void
XHCIController::freeBulk(struct BulkPipe *pipe)
{
	freeDMA((void *)(uintptr_t)pipe->ring.trbs, pipe->ring.phys, RING_TRBS * sizeof(struct TRB));
	freeDMA(pipe->buffer, pipe->bufferPhys, BULK_MAX);
	IOFreeType(pipe, struct BulkPipe);
}

#pragma mark Bulk queues

void
XHCIController::freeQueue(struct BulkQueue *s)
{
	freeDMA((void *)(uintptr_t)s->ring.trbs, s->ring.phys, RING_TRBS * sizeof(struct TRB));
	OSSafeReleaseNULL(s->map);
	OSSafeReleaseNULL(s->mem);
	IOFreeType(s, struct BulkQueue);
}

IOReturn
XHCIController::gatedQueueKeep(OSObject *owner, void *queue, void *slotArg, void *arg2, void *arg3)
{
	(void)arg2;
	(void)arg3;
	XHCIController *self = (XHCIController *)owner;
	struct Device *d = &self->devices[(uint32_t)(uintptr_t)slotArg];
	struct BulkQueue *s = (struct BulkQueue *)queue;
	s->next = d->dead;
	d->dead = s;
	return kIOReturnSuccess;
}

void
XHCIController::queueIn(struct BulkQueue *s, uint32_t b)
{
	uint64_t trb = ringPut(&s->ring, s->buffersPhys + (uint64_t)b * s->size, s->size,
	    TRB_NORMAL << TRB_TYPE_SHIFT | TRB_IOC | TRB_ISP);
	s->owner[(trb - s->ring.phys) / sizeof(struct TRB)] = (uint8_t)b;
}

void
XHCIController::queueEvent(uint32_t slot, struct BulkQueue *s, uint64_t trb, uint32_t code, uint32_t residual)
{
	uint64_t off = trb - s->ring.phys;
	if (off >= (RING_TRBS - 1) * sizeof(struct TRB) || (off % sizeof(struct TRB)) != 0) {
		return;
	}
	uint32_t index = (uint32_t)(off / sizeof(struct TRB));
	uint8_t owner = s->owner[index];
	if (owner == QUEUE_NONE) {
		return;
	}
	s->owner[index] = QUEUE_NONE;
	uint32_t b = owner & (QUEUE_BUFFERS_MAX - 1);
	bool ok = (code == CC_SUCCESS || code == CC_SHORT_PACKET) && residual <= s->size;
	IOReturn status = ok ? kIOReturnSuccess : code == CC_STALL ? kIOReturnNotResponding : kIOReturnIOError;
	if (!ok && !s->halted) {
		s->halted = true;
		IOLog("XHCI: slot %u endpoint 0x%x: bulk queue halted, completion code %u\n", slot, s->ep.address, code);
	}
	if (s->in) {
		if (ok) {
			s->action(s->target, s->buffers + (size_t)b * s->size, s->size - residual, kIOReturnSuccess);
			if (!s->halted) {
				queueIn(s, b);
				w32(db, 4 * slot, s->dci);
			}
		} else {
			s->action(s->target, NULL, 0, status);
		}
		return;
	}
	if ((owner & QUEUE_NO_EVENT) && ok) {
		return;
	}
	if (!(s->busy & (1U << b))) {
		return;
	}
	s->busy &= ~(1U << b);
	s->action(s->target, NULL, ok ? s->lengths[b] : 0, status);
}

IOReturn
XHCIController::gatedQueueStart(OSObject *owner, void *queue, void *slotArg, void *arg2, void *arg3)
{
	(void)arg2;
	(void)arg3;
	XHCIController *self = (XHCIController *)owner;
	struct BulkQueue *s = (struct BulkQueue *)queue;
	uint32_t slot = (uint32_t)(uintptr_t)slotArg;
	struct Device *d = &self->devices[slot];
	d->queues[s->dci] = s;
	if (s->in) {
		for (uint32_t b = 0; b < s->count; b++) {
			self->queueIn(s, b);
		}
		self->w32(self->db, 4 * slot, s->dci);
	}
	return kIOReturnSuccess;
}

IOReturn
XHCIController::startQueue(uint32_t slot, const struct PIEUSBEndpoint *ep, uint32_t size, uint32_t count,
    OSObject *target, PIEUSBQueueAction action, uint32_t generation)
{
	if (!current(slot, generation)) {
		return kIOReturnNoDevice;
	}
	if (action == NULL || count < 1 || count > QUEUE_BUFFERS_MAX || size < 512 || size > 65536 ||
	    (size & (size - 1)) != 0 || (uint64_t)size * count > QUEUE_BYTES_MAX || (ep->attributes & 3) != 2 ||
	    (ep->maxPacket & 0x7ff) == 0) {
		return kIOReturnBadArgument;
	}
	bool in = (ep->address & 0x80) != 0;
	if (in && size % (ep->maxPacket & 0x7ffU) != 0) {
		return kIOReturnBadArgument;
	}
	uint32_t dci = (ep->address & 0xf) * 2U + (in ? 1 : 0);
	if (dci < 2 || dci >= MAX_DCI) {
		return kIOReturnBadArgument;
	}
	struct Device *d = &devices[slot];
	if (d->queues[dci] != NULL || d->bulk[dci] != NULL) {
		return kIOReturnBusy;
	}

	struct BulkQueue *s = IOMallocType(struct BulkQueue);
	s->ring.trbs = (volatile struct TRB *)allocDMA(RING_TRBS * sizeof(struct TRB), 4096, &s->ring.phys);
	s->mem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, (size_t)size * count,
	    0x7fff0000ULL);
	if (s->mem != NULL) {
		s->mem->performOperation(kIOMemoryIncoherentIOFlush, 0, (size_t)size * count);
		s->map = s->mem->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
	}
	uint64_t phys = s->mem != NULL ? s->mem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone) : 0;
	if (s->ring.trbs == NULL || s->map == NULL || !RPiDMAAddress(nub, phys, &s->buffersPhys)) {
		freeQueue(s);
		return kIOReturnNoMemory;
	}
	s->buffers = (uint8_t *)s->map->getVirtualAddress();
	s->size = size;
	s->count = count;
	s->dci = dci;
	s->in = in;
	s->ep = *ep;
	s->target = target;
	s->action = action;
	memset(s->owner, QUEUE_NONE, sizeof(s->owner));
	initRing(&s->ring);

	IOReturn ret;
	IOLockLock(cmdLock);
	if (!current(slot, generation)) {
		ret = kIOReturnNoDevice;
	} else if (d->queues[dci] != NULL || d->bulk[dci] != NULL || kept_queue(d, dci)) {
		ret = kIOReturnBusy;
	} else {
		bool configured = (endpointContext(d->outCtx, false, dci)[0] & 7) != 0;
		ret = addEndpointLocked(slot, dci, in ? EP_BULK_IN : EP_BULK_OUT, ep, s->ring.phys | 1, 0, 3072,
		    configured);
		if (ret == kIOReturnSuccess) {
			workLoop->runAction(&XHCIController::gatedQueueStart, this, s, (void *)(uintptr_t)slot);
		}
	}
	if (ret == kIOReturnTimeout) {
		IOLog("XHCI: slot %u endpoint 0x%x: no answer to configuring a bulk queue; its memory is kept\n", slot,
		    ep->address);
		workLoop->runAction(&XHCIController::gatedQueueKeep, this, s, (void *)(uintptr_t)slot);
	}
	IOLockUnlock(cmdLock);
	if (ret != kIOReturnSuccess && ret != kIOReturnTimeout) {
		freeQueue(s);
	}
	return ret;
}

IOReturn
XHCIController::gatedQueueTake(OSObject *owner, void *slotArg, void *dciArg, void *generationArg, void *out)
{
	XHCIController *self = (XHCIController *)owner;
	uint32_t slot = (uint32_t)(uintptr_t)slotArg, dci = (uint32_t)(uintptr_t)dciArg;
	if (slot == 0 || slot > MAX_SLOTS) {
		return kIOReturnNoDevice;
	}
	struct Device *d = &self->devices[slot];
	if (!d->used || d->generation != (uint32_t)(uintptr_t)generationArg) {
		return kIOReturnNoDevice;
	}
	*(struct BulkQueue **)out = d->queues[dci];
	d->queues[dci] = NULL;
	return kIOReturnSuccess;
}

IOReturn
XHCIController::stopQueue(uint32_t slot, uint8_t endpointAddress, uint32_t generation)
{
	uint32_t dci = (endpointAddress & 0xf) * 2U + ((endpointAddress & 0x80) ? 1 : 0);
	if (dci < 2 || dci >= MAX_DCI) {
		return current(slot, generation) ? kIOReturnBadArgument : kIOReturnNoDevice;
	}
	struct BulkQueue *s = NULL;
	IOLockLock(cmdLock);
	IOReturn ret = workLoop->runAction(&XHCIController::gatedQueueTake, this, (void *)(uintptr_t)slot,
	    (void *)(uintptr_t)dci, (void *)(uintptr_t)generation, &s);
	if (ret != kIOReturnSuccess || s == NULL) {
		IOLockUnlock(cmdLock);
		return ret != kIOReturnSuccess ? ret : current(slot, generation) ? kIOReturnNotFound : kIOReturnNoDevice;
	}
	ret = stopEndpointLocked(slot, dci);
	bool stopped = ret == kIOReturnSuccess;
	bool live = current(slot, generation);
	if (!stopped) {
		if (live) {
			IOLog("XHCI: slot %u endpoint 0x%x: the bulk queue did not stop; it is freed when the device goes\n",
			    slot, endpointAddress);
		}
		workLoop->runAction(&XHCIController::gatedQueueKeep, this, s, (void *)(uintptr_t)slot);
	}
	IOLockUnlock(cmdLock);
	if (stopped) {
		freeQueue(s);
	}
	return !live ? kIOReturnNoDevice : stopped ? kIOReturnSuccess : ret;
}

struct QueueOutArgs {
	uint32_t slot, generation, dci;
	const void *data;
	uint32_t length;
};

IOReturn
XHCIController::gatedQueueOut(OSObject *owner, void *args, void *arg1, void *arg2, void *arg3)
{
	(void)arg1;
	(void)arg2;
	(void)arg3;
	XHCIController *self = (XHCIController *)owner;
	struct QueueOutArgs *a = (struct QueueOutArgs *)args;
	if (!self->current(a->slot, a->generation)) {
		return kIOReturnNoDevice;
	}
	struct BulkQueue *s = self->devices[a->slot].queues[a->dci];
	if (s == NULL || s->in) {
		return kIOReturnNotFound;
	}
	if (s->halted) {
		return kIOReturnNotResponding;
	}
	if (a->length > s->size) {
		return kIOReturnBadArgument;
	}
	uint32_t all = s->count == 32 ? 0xffffffffU : (1U << s->count) - 1;
	uint32_t idle = ~s->busy & all;
	if (idle == 0) {
		return kIOReturnNoResources;
	}
	uint32_t b = (uint32_t)__builtin_ctz(idle);
	uint64_t buffer = s->buffersPhys + (uint64_t)b * s->size;
	bcopy(a->data, s->buffers + (size_t)b * s->size, a->length);
	uint32_t mps = s->ep.maxPacket & 0x7ff;
	uint64_t trb;
	if (a->length > 0 && a->length % mps == 0) {
		trb = self->ringPut(&s->ring, buffer, a->length, TRB_NORMAL << TRB_TYPE_SHIFT);
		s->owner[(trb - s->ring.phys) / sizeof(struct TRB)] = (uint8_t)(b | QUEUE_NO_EVENT);
		trb = self->ringPut(&s->ring, buffer, 0, TRB_NORMAL << TRB_TYPE_SHIFT | TRB_IOC);
	} else {
		trb = self->ringPut(&s->ring, buffer, a->length, TRB_NORMAL << TRB_TYPE_SHIFT | TRB_IOC);
	}
	s->owner[(trb - s->ring.phys) / sizeof(struct TRB)] = (uint8_t)b;
	s->lengths[b] = a->length;
	s->busy |= 1U << b;
	self->w32(self->db, 4 * a->slot, a->dci);
	return kIOReturnSuccess;
}

IOReturn
XHCIController::queueOut(uint32_t slot, uint8_t endpointAddress, const void *data, uint32_t length,
    uint32_t generation)
{
	if (!current(slot, generation)) {
		return kIOReturnNoDevice;
	}
	if (endpointAddress & 0x80) {
		return kIOReturnBadArgument;
	}
	struct QueueOutArgs a = { slot, generation, (endpointAddress & 0xfU) * 2U, data, length };
	if (a.dci < 2 || a.dci >= MAX_DCI) {
		return kIOReturnBadArgument;
	}
	return workLoop->runAction(&XHCIController::gatedQueueOut, this, &a);
}

#pragma mark Enumeration

IOReturn
XHCIController::controlReadExact(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
    uint16_t wIndex, uint16_t wLength, void *data)
{
	uint32_t got = 0;
	IOReturn ret = controlTransfer(slot, bmRequestType, bRequest, wValue, wIndex, wLength, data, &got);
	return ret == kIOReturnSuccess && got != wLength ? kIOReturnUnderrun : ret;
}

static uint32_t
valid_descriptors(const uint8_t *cfg, uint32_t length)
{
	uint32_t off = 0;
	while (off + 2 <= length && cfg[off] >= 2 && cfg[off] <= length - off) {
		off += cfg[off];
	}
	return off;
}

void
XHCIController::attachPort(uint32_t port)
{
	uintptr_t sc = op + OP_PORTSC + 0x10 * (port - 1);
	uint32_t portsc = r32(sc, 0);
	if (!(portsc & PORTSC_PED)) {
		w32(sc, 0, (portsc & ~(PORTSC_PED | PORTSC_CHANGES)) | PORTSC_PR);
		for (int i = 0; i < 100 && !(r32(sc, 0) & PORTSC_PRC); i++) {
			IOSleep(5);
		}
		portsc = r32(sc, 0);
		w32(sc, 0, (portsc & ~(PORTSC_PED | PORTSC_CHANGES)) | PORTSC_PRC);
		IOSleep(10);
		portsc = r32(sc, 0);
		if (!(portsc & PORTSC_PED)) {
			IOLog("XHCI: port %u: not enabled after reset (0x%08x)\n", port, portsc);
			return;
		}
	}
	uint8_t speed = (uint8_t)((portsc >> PORTSC_SPEED_SHIFT) & 0xf);
	portSlot[port] = attachDevice((uint8_t)port, 0, 0, speed, 0, 0);
	if (portSlot[port] == 0) {
		portSlot[port] = 0xffffffff;
	}
}

uint32_t
XHCIController::attachDevice(uint8_t rootPort, uint32_t route, uint8_t depth, uint8_t speed, uint32_t parentSlot,
    uint8_t parentPort)
{
	uint32_t slot = 0;
	if (command(0, 0, TRB_ENABLE_SLOT << TRB_TYPE_SHIFT, &slot) != kIOReturnSuccess || slot == 0 ||
	    slot > MAX_SLOTS) {
		return 0;
	}
	struct Device *d = &devices[slot];
	bzero(d, sizeof(*d));
	d->used = true;
	if (++nextGeneration == 0) {
		nextGeneration = 1;
	}
	d->generation = nextGeneration;
	d->port = rootPort;
	d->speed = speed;
	d->route = route;
	d->depth = depth;
	if (parentSlot != 0) {
		struct Device *parent = &devices[parentSlot];
		if ((speed == SPEED_LOW || speed == SPEED_FULL) && parent->speed == SPEED_HIGH) {
			d->ttSlot = (uint8_t)parentSlot;
			d->ttPort = parentPort;
		} else {
			d->ttSlot = parent->ttSlot;
			d->ttPort = parent->ttPort;
		}
	}
	d->outCtx = (uint8_t *)allocDMA(ctxSize * 32, ctxSize * 32, &d->outPhys);
	d->inCtx = (uint8_t *)allocDMA(ctxSize * 33, 4096, &d->inPhys);
	d->ep0.trbs = (volatile struct TRB *)allocDMA(RING_TRBS * sizeof(struct TRB), 4096, &d->ep0.phys);
	d->ep0Buf = (uint8_t *)allocDMA(4096, 4096, &d->ep0BufPhys);
	if (d->outCtx == NULL || d->inCtx == NULL || d->ep0.trbs == NULL || d->ep0Buf == NULL) {
		IOLog("XHCI: out of DMA memory\n");
		detachDevice(slot);
		return 0;
	}
	initRing(&d->ep0);
	d->maxDCI = 1;
	dcbaa[slot] = d->outPhys;

	uint16_t mps0 = speed == SPEED_SUPER ? 512 : speed == SPEED_HIGH ? 64 : 8;
	uint32_t *icc = (uint32_t *)(void *)d->inCtx;
	icc[1] = 1U << 0 | 1U << 1;
	uint32_t *slotIn = slotContext(d->inCtx, true);
	slotIn[0] = (route & 0xfffff) | (uint32_t)speed << 20 | 1U << 27;
	slotIn[1] = (uint32_t)rootPort << 16;
	slotIn[2] = (uint32_t)d->ttSlot | (uint32_t)d->ttPort << 8;
	uint32_t *ep0 = endpointContext(d->inCtx, true, 1);
	ep0[1] = 3U << 1 | (uint32_t)EP_CONTROL << 3 | (uint32_t)mps0 << 16;
	ep0[2] = (uint32_t)d->ep0.phys | 1;
	ep0[3] = (uint32_t)(d->ep0.phys >> 32);
	ep0[4] = 8;
	if (command(d->inPhys, 0, TRB_ADDRESS_DEVICE << TRB_TYPE_SHIFT | slot << 24, NULL) != kIOReturnSuccess) {
		detachDevice(slot);
		return 0;
	}

	uint8_t dev[18];
	if (controlReadExact(slot, 0x80, 6, 0x0100, 0, 8, dev) != kIOReturnSuccess || dev[0] < 18 || dev[1] != 1) {
		IOLog("XHCI: port %u route %x: no device descriptor\n", rootPort, route);
		detachDevice(slot);
		return 0;
	}
	uint16_t realMPS = dev[7];
	if (speed == SPEED_SUPER) {
		realMPS = dev[7] < 16 ? (uint16_t)(1U << dev[7]) : 0;
	}
	if (realMPS != mps0 && realMPS != 0) {
		bzero(d->inCtx, ctxSize * 33);
		icc[1] = 1U << 1;
		ep0 = endpointContext(d->inCtx, true, 1);
		ep0[1] = 3U << 1 | (uint32_t)EP_CONTROL << 3 | (uint32_t)realMPS << 16;
		command(d->inPhys, 0, TRB_EVALUATE_CONTEXT << TRB_TYPE_SHIFT | slot << 24, NULL);
	}
	uint8_t cfgHead[9];
	uint16_t total = 0;
	if (controlReadExact(slot, 0x80, 6, 0x0100, 0, 18, dev) == kIOReturnSuccess && dev[0] >= 18 && dev[1] == 1 &&
	    controlReadExact(slot, 0x80, 6, 0x0200, 0, 9, cfgHead) == kIOReturnSuccess && cfgHead[0] >= 9 &&
	    cfgHead[1] == 2) {
		total = OSReadLittleInt16(cfgHead, 2);
	}
	if (total > 4096) {
		IOLog("XHCI: port %u route %x: configuration of %u bytes is longer than 4096\n", rootPort, route, total);
		detachDevice(slot);
		return 0;
	}
	if (total < 9) {
		IOLog("XHCI: port %u route %x: descriptors unreadable\n", rootPort, route);
		detachDevice(slot);
		return 0;
	}
	uint8_t *cfg = (uint8_t *)IOMallocZeroData(total);
	if (cfg == NULL) {
		detachDevice(slot);
		return 0;
	}
	uint32_t length = 0;
	if (controlReadExact(slot, 0x80, 6, 0x0200, 0, total, cfg) == kIOReturnSuccess && cfg[0] >= 9 && cfg[1] == 2) {
		length = valid_descriptors(cfg, total);
	}
	bool ok = false;
	if (length < 9) {
		IOLog("XHCI: port %u route %x: configuration unreadable\n", rootPort, route);
	} else if (controlTransfer(slot, 0x00, 9, cfg[5], 0, 0, NULL, NULL) == kIOReturnSuccess) {
		static const char *const speeds[] = { "?", "full", "low", "high", "super" };
		char where[32];
		if (route != 0) {
			snprintf(where, sizeof(where), "port %u route %x", rootPort, route);
		} else {
			snprintf(where, sizeof(where), "port %u", rootPort);
		}
		IOLog("XHCI: %s: %04x:%04x, class %02x, %s speed, slot %u\n", where,
		    OSReadLittleInt16(dev, 8), OSReadLittleInt16(dev, 10), dev[4],
		    speeds[speed <= 4 ? speed : 0], slot);
		ok = true;
		if (dev[4] == 9) {
			setupHub(slot, cfg, length);
		}
		publishInterfaces(slot, d->generation, d->port, d->speed, dev, cfg, length);
	}
	IOFreeData(cfg, total);
	if (!ok) {
		detachDevice(slot);
	}
	return ok ? slot : 0;
}

static IOReturn
take_pipes(OSObject *owner, void *device, void *out, void *queuesOut, void *deadOut)
{
	(void)owner;
	struct Device *d = (struct Device *)device;
	struct InterruptPipe **pipes = (struct InterruptPipe **)out;
	struct BulkQueue **queues = (struct BulkQueue **)queuesOut;
	for (uint32_t dci = 0; dci < MAX_DCI; dci++) {
		pipes[dci] = d->pipes[dci];
		d->pipes[dci] = NULL;
		queues[dci] = d->queues[dci];
		d->queues[dci] = NULL;
	}
	*(struct BulkQueue **)deadOut = d->dead;
	d->dead = NULL;
	return kIOReturnSuccess;
}

static IOReturn
keep_pipes(OSObject *owner, void *device, void *in, void *queuesIn, void *deadIn)
{
	(void)owner;
	struct Device *d = (struct Device *)device;
	struct InterruptPipe **pipes = (struct InterruptPipe **)in;
	struct BulkQueue **queues = (struct BulkQueue **)queuesIn;
	struct BulkQueue *dead = (struct BulkQueue *)deadIn;
	for (uint32_t dci = 0; dci < MAX_DCI; dci++) {
		if (pipes[dci] != NULL) {
			pipes[dci]->action = NULL;
			d->pipes[dci] = pipes[dci];
		}
		if (queues[dci] != NULL) {
			queues[dci]->next = dead;
			dead = queues[dci];
		}
	}
	d->dead = dead;
	return kIOReturnSuccess;
}

void
XHCIController::detachDevice(uint32_t slot)
{
	if (slot == 0 || slot > MAX_SLOTS || !devices[slot].used) {
		return;
	}
	struct Device *d = &devices[slot];
	bool again = d->gone;
	if (d->isHub) {
		for (uint8_t p = 1; p <= d->hubPorts; p++) {
			uint8_t child = d->hubChild[p];
			d->hubChild[p] = 0;
			if (child != 0 && child != 0xff) {
				detachDevice(child);
			}
		}
	}

	IOLockLock(waitLock);
	d->gone = true;
	if (waitTRB != 0 && waitOnDCI != 0 && waitOnSlot == slot) {
		waitCode = CC_GONE;
		waitResidual = 0;
		waitDone = true;
		IOLockWakeup(waitLock, &waitDone, true);
	}
	IOLockUnlock(waitLock);
	struct InterruptPipe *pipes[MAX_DCI];
	struct BulkQueue *queues[MAX_DCI];
	struct BulkQueue *dead = NULL;
	IOLockLock(cmdLock);
	workLoop->runAction(&take_pipes, this, d, pipes, queues, &dead);
	bool quiet = !d->timedOut;
	for (uint32_t dci = 1; dci <= d->maxDCI && dci < MAX_DCI && d->outCtx != NULL; dci++) {
		uint32_t state = endpointContext(d->outCtx, false, dci)[0] & 7;
		if ((state == EP_STATE_RUNNING || state == EP_STATE_HALTED) &&
		    stopEndpointLocked(slot, dci) != kIOReturnSuccess) {
			quiet = false;
		}
	}
	IOReturn ret = kIOReturnNotReady;
	if (quiet) {
		workLoop->runAction(&XHCIController::gatedEvents, this);
		ret = commandLocked(0, 0, TRB_DISABLE_SLOT << TRB_TYPE_SHIFT | slot << 24, NULL);
		if (ret == kIOReturnAborted) {
			ret = commandLocked(0, 0, TRB_DISABLE_SLOT << TRB_TYPE_SHIFT | slot << 24, NULL);
		}
	}
	bool disabled = ret == kIOReturnSuccess;
	if (disabled) {
		dcbaa[slot] = 0;
	}
	bool clean = disabled && !d->timedOut;
	bool controllerDead = cmdDead;
	IOLockUnlock(cmdLock);
	if (!clean && !again) {
		IOLog("XHCI: slot %u did not stop cleanly; its memory is kept %s\n", slot,
		    controllerDead ? "because the controller takes no more commands" : "until it does");
	} else if (clean && again) {
		IOLog("XHCI: slot %u stopped; its memory is freed\n", slot);
	}
	while (clean && dead != NULL) {
		struct BulkQueue *next = dead->next;
		freeQueue(dead);
		dead = next;
	}

	terminateInterfaces(slot, d->generation);

	for (uint32_t dci = 0; dci < MAX_DCI && clean; dci++) {
		if (pipes[dci] != NULL) {
			freePipe(pipes[dci]);
		}
		if (queues[dci] != NULL) {
			freeQueue(queues[dci]);
		}
		if (d->bulk[dci] != NULL) {
			freeBulk(d->bulk[dci]);
			d->bulk[dci] = NULL;
		}
	}
	if (clean) {
		freeDMA(d->outCtx, d->outPhys, ctxSize * 32);
		freeDMA(d->inCtx, d->inPhys, ctxSize * 33);
		freeDMA((void *)(uintptr_t)d->ep0.trbs, d->ep0.phys, RING_TRBS * sizeof(struct TRB));
		freeDMA(d->ep0Buf, d->ep0BufPhys, 4096);
	}
	IOLockLock(cmdLock);
	if (clean || cmdDead) {
		bzero(d, sizeof(*d));
	} else {
		workLoop->runAction(&keep_pipes, this, d, pipes, queues, dead);
		d->isHub = false;
	}
	IOLockUnlock(cmdLock);
}

#pragma mark Hubs

enum {
	HUB_PORT_CONNECTION = 1U << 0, HUB_PORT_ENABLE = 1U << 1, HUB_PORT_LOW_SPEED = 1U << 9,
	HUB_PORT_HIGH_SPEED = 1U << 10,
	HUB_C_PORT_CONNECTION = 1U << 0, HUB_C_PORT_RESET = 1U << 4,
	FEAT_PORT_RESET = 4, FEAT_PORT_POWER = 8, FEAT_C_PORT_CONNECTION = 16, FEAT_C_PORT_ENABLE = 17,
	FEAT_C_PORT_RESET = 20,

	SS_HUB_DESCRIPTOR = 0x2a, SS_SET_HUB_DEPTH = 12,
	SS_C_BH_PORT_RESET = 1U << 5, SS_C_PORT_LINK_STATE = 1U << 6, SS_C_PORT_CONFIG_ERROR = 1U << 7,
	FEAT_C_PORT_LINK_STATE = 25, FEAT_C_PORT_CONFIG_ERROR = 26, FEAT_C_BH_PORT_RESET = 29,
};

void
XHCIController::hubStatusChanged(OSObject *target, const uint8_t *data, uint32_t length)
{
	(void)data;
	(void)length;
	XHCIController *self = OSDynamicCast(XHCIController, target);
	if (self == NULL) {
		return;
	}
	IOLockLock(self->portLock);
	self->portsChanged = true;
	IOLockWakeup(self->portLock, &self->portsChanged, true);
	IOLockUnlock(self->portLock);
}

void
XHCIController::setupHub(uint32_t slot, const uint8_t *cfg, uint32_t cfgLength)
{
	struct Device *d = &devices[slot];
	bool super = d->speed == SPEED_SUPER;
	uint8_t desc[12];
	uint16_t type = super ? SS_HUB_DESCRIPTOR : 0x29;
	uint16_t length = super ? 12 : 9;
	if (controlReadExact(slot, 0xa0, 6, (uint16_t)(type << 8), 0, length, desc) != kIOReturnSuccess ||
	    desc[0] < length || desc[1] != type) {
		IOLog("XHCI: slot %u: no hub descriptor\n", slot);
		return;
	}
	uint8_t ports = desc[2] < HUB_PORTS ? desc[2] : HUB_PORTS;
	uint16_t characteristics = OSReadLittleInt16(desc, 3);
	uint32_t powerDelay = desc[5] * 2U;
	if (super && controlTransfer(slot, 0x20, SS_SET_HUB_DEPTH, d->depth, 0, 0, NULL, NULL) != kIOReturnSuccess) {
		IOLog("XHCI: slot %u: USB 3 hub refused its depth\n", slot);
		return;
	}

	for (uint32_t off = 0; off + 2 <= cfgLength && cfg[off] >= 2; off += cfg[off]) {
		const uint8_t *e = cfg + off;
		if (e[1] == 5 && e[0] >= 7 && (e[2] & 0x80) && (e[2] & 0xf) != 0 && (e[3] & 3) == 3) {
			d->hubStatus.address = e[2];
			d->hubStatus.attributes = e[3];
			d->hubStatus.maxPacket = OSReadLittleInt16(e, 4);
			d->hubStatus.interval = e[6];
			break;
		}
	}

	bzero(d->inCtx, ctxSize * 33);
	uint32_t *icc = (uint32_t *)(void *)d->inCtx;
	icc[1] = 1U << 0;
	uint32_t *slotIn = slotContext(d->inCtx, true);
	uint32_t *slotOut = slotContext(d->outCtx, false);
	for (int i = 0; i < 8; i++) {
		slotIn[i] = slotOut[i];
	}
	slotIn[0] |= 1U << 26;
	slotIn[1] = (slotIn[1] & 0x00ffffffU) | (uint32_t)ports << 24;
	if (d->speed == SPEED_HIGH) {
		slotIn[2] = (slotIn[2] & ~(3U << 16)) | ((characteristics >> 5) & 3U) << 16;
	}
	if (command(d->inPhys, 0, TRB_CONFIGURE_EP << TRB_TYPE_SHIFT | slot << 24, NULL) != kIOReturnSuccess) {
		return;
	}
	for (uint8_t p = 1; p <= ports; p++) {
		controlTransfer(slot, 0x23, 3, FEAT_PORT_POWER, p, 0, NULL, NULL);
	}
	IOSleep(powerDelay > 100 ? powerDelay : 100);
	d->hubPorts = ports;
	d->isHub = true;
	IOLog("XHCI: slot %u: %shub, %u ports\n", slot, super ? "USB 3 " : "", ports);
	if (d->hubStatus.address != 0) {
		startInterruptIn(slot, &d->hubStatus, this, &XHCIController::hubStatusChanged);
	}
}

void
XHCIController::scanHub(uint32_t slot)
{
	struct Device *d = &devices[slot];
	bool super = d->speed == SPEED_SUPER;
	for (uint8_t p = 1; p <= d->hubPorts; p++) {
		uint8_t st[4];
		if (controlReadExact(slot, 0xa3, 0, 0, p, 4, st) != kIOReturnSuccess) {
			continue;
		}
		uint16_t status = OSReadLittleInt16(st, 0), change = OSReadLittleInt16(st, 2);
		if (change & HUB_C_PORT_CONNECTION) {
			controlTransfer(slot, 0x23, 1, FEAT_C_PORT_CONNECTION, p, 0, NULL, NULL);
		}
		if (super) {
			if (change & SS_C_PORT_LINK_STATE) {
				controlTransfer(slot, 0x23, 1, FEAT_C_PORT_LINK_STATE, p, 0, NULL, NULL);
			}
			if (change & SS_C_PORT_CONFIG_ERROR) {
				controlTransfer(slot, 0x23, 1, FEAT_C_PORT_CONFIG_ERROR, p, 0, NULL, NULL);
			}
			if (change & SS_C_BH_PORT_RESET) {
				controlTransfer(slot, 0x23, 1, FEAT_C_BH_PORT_RESET, p, 0, NULL, NULL);
			}
		}
		if (d->hubChild[p] != 0 && (!(status & HUB_PORT_CONNECTION) || (change & HUB_C_PORT_CONNECTION))) {
			uint8_t child = d->hubChild[p];
			d->hubChild[p] = 0;
			if (child != 0xff) {
				IOLog("XHCI: slot %u port %u: disconnected\n", slot, p);
				detachDevice(child);
			}
		}
		if (!(status & HUB_PORT_CONNECTION)) {
			continue;
		}
		if (d->hubChild[p] != 0 || d->depth >= 5) {
			continue;
		}
		controlTransfer(slot, 0x23, 3, FEAT_PORT_RESET, p, 0, NULL, NULL);
		for (int i = 0; i < 50; i++) {
			IOSleep(10);
			if (controlReadExact(slot, 0xa3, 0, 0, p, 4, st) == kIOReturnSuccess &&
			    (OSReadLittleInt16(st, 2) & HUB_C_PORT_RESET)) {
				break;
			}
		}
		controlTransfer(slot, 0x23, 1, FEAT_C_PORT_RESET, p, 0, NULL, NULL);
		if (super) {
			controlTransfer(slot, 0x23, 1, FEAT_C_BH_PORT_RESET, p, 0, NULL, NULL);
			controlTransfer(slot, 0x23, 1, FEAT_C_PORT_LINK_STATE, p, 0, NULL, NULL);
		} else {
			controlTransfer(slot, 0x23, 1, FEAT_C_PORT_ENABLE, p, 0, NULL, NULL);
		}
		IOSleep(10);
		if (controlReadExact(slot, 0xa3, 0, 0, p, 4, st) != kIOReturnSuccess ||
		    !(OSReadLittleInt16(st, 0) & HUB_PORT_ENABLE)) {
			IOLog("XHCI: slot %u port %u: not enabled after reset\n", slot, p);
			d->hubChild[p] = 0xff;
			continue;
		}
		status = OSReadLittleInt16(st, 0);
		uint8_t speed = super ? SPEED_SUPER : (status & HUB_PORT_LOW_SPEED) ? SPEED_LOW :
		    (status & HUB_PORT_HIGH_SPEED) ? SPEED_HIGH : SPEED_FULL;
		uint32_t route = d->route | (uint32_t)(p < 15 ? p : 15) << (4 * d->depth);
		uint32_t child = attachDevice(d->port, route, (uint8_t)(d->depth + 1), speed, slot, p);
		d->hubChild[p] = child != 0 ? (uint8_t)child : 0xff;
	}
}

void
XHCIController::scanPorts(void)
{
	for (uint32_t slot = 1; slot <= MAX_SLOTS; slot++) {
		if (devices[slot].used && devices[slot].gone) {
			detachDevice(slot);
		}
	}
	for (uint32_t port = 1; port <= maxPorts && port < 256; port++) {
		uintptr_t sc = op + OP_PORTSC + 0x10 * (port - 1);
		uint32_t portsc = r32(sc, 0);
		if (portsc & PORTSC_CSC) {
			w32(sc, 0, (portsc & ~(PORTSC_PED | PORTSC_CHANGES)) | PORTSC_CSC);
		}
		if (portSlot[port] != 0 && (!(portsc & PORTSC_CCS) || (portsc & PORTSC_CSC))) {
			if (portSlot[port] != 0xffffffff) {
				IOLog("XHCI: port %u: disconnected\n", port);
				detachDevice(portSlot[port]);
			}
			portSlot[port] = 0;
		}
		if ((portsc & PORTSC_CCS) && portSlot[port] == 0) {
			attachPort(port);
		}
	}
	for (uint32_t slot = 1; slot <= MAX_SLOTS; slot++) {
		if (devices[slot].used && devices[slot].isHub) {
			scanHub(slot);
		}
	}
}

void
XHCIController::enumerationThread(void *arg, wait_result_t wr)
{
	(void)wr;
	XHCIController *self = (XHCIController *)arg;
	IOSleep(50);
	for (;;) {
		self->scanPorts();
		IOLockLock(self->portLock);
		while (!self->portsChanged && !self->stopping) {
			IOLockSleep(self->portLock, &self->portsChanged, THREAD_UNINT);
		}
		self->portsChanged = false;
		bool stop = self->stopping;
		IOLockUnlock(self->portLock);
		if (stop) {
			break;
		}
		IOSleep(100);
	}
	self->release();
}

#pragma mark Controller

bool
XHCIController::resetController(void)
{
	w32(op, OP_USBCMD, r32(op, OP_USBCMD) & ~USBCMD_RS);
	for (int i = 0; i < 200 && !(r32(op, OP_USBSTS) & USBSTS_HCH); i++) {
		IODelay(100);
	}
	w32(op, OP_USBCMD, USBCMD_HCRST);
	for (int i = 0; i < 1000; i++) {
		if (!(r32(op, OP_USBCMD) & USBCMD_HCRST) && !(r32(op, OP_USBSTS) & USBSTS_CNR)) {
			return true;
		}
		IODelay(1000);
	}
	return false;
}

bool
XHCIController::start(IOService *provider)
{
	if (IODTMatchNubWithKeys(provider, "'rockchip,rk3568-dwc3'")) {
		IOLog("XHCI: the RK3566's DWC3 needs its PHY and clocks set up first; not started\n");
		return false;
	}
	if (!super::start(provider)) {
		return false;
	}
	nub = provider;
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		IOLog("XHCI: cannot map the registers\n");
		return false;
	}
	cap = (uintptr_t)regsMap->getVirtualAddress();

	uint32_t capWord = r32(cap, CAP_CAPLENGTH);
	uint32_t capLength = capWord & 0xff, hciVersion = capWord >> 16;
	if (capWord == 0xffffffff || capLength < 0x20 || (capLength & 3) || hciVersion < 0x90 || hciVersion > 0x200) {
		IOLog("XHCI: no xHCI at 0x%llx (capability register 0x%08x)\n", regsMap->getPhysicalAddress(), capWord);
		return false;
	}
	if (IODTMatchNubWithKeys(provider, "'snps,dwc3'") && regsMap->getLength() > DWC3_GCTL + 4) {
		uint32_t gctl = r32(cap, DWC3_GCTL);
		if (((gctl >> 12) & 3) != DWC3_GCTL_PRTCAP_HOST) {
			w32(cap, DWC3_GCTL, (gctl & ~(3U << 12)) | DWC3_GCTL_PRTCAP_HOST << 12);
			IODelay(1000);
		}
		IOLog("XHCI: DWC3 core 0x%08x in host mode\n", r32(cap, DWC3_GSNPSID));
	}
	op = cap + (r32(cap, CAP_CAPLENGTH) & 0xff);
	rt = cap + (r32(cap, CAP_RTSOFF) & ~0x1fU);
	db = cap + (r32(cap, CAP_DBOFF) & ~0x3U);
	uint32_t hcs1 = r32(cap, CAP_HCSPARAMS1), hcs2 = r32(cap, CAP_HCSPARAMS2), hcc1 = r32(cap, CAP_HCCPARAMS1);
	maxSlots = hcs1 & 0xff;
	if (maxSlots > MAX_SLOTS) {
		maxSlots = MAX_SLOTS;
	}
	maxPorts = hcs1 >> 24;
	ctxSize = (hcc1 & (1U << 2)) ? 64 : 32;
	uint32_t scratchpads = ((hcs2 >> 21) & 0x1f) << 5 | (hcs2 >> 27);

	waitLock = IOLockAlloc();
	cmdLock = IOLockAlloc();
	portLock = IOLockAlloc();
	dmaLock = IOLockAlloc();
	devices = IONewZero(struct Device, MAX_SLOTS + 1);
	if (waitLock == NULL || cmdLock == NULL || portLock == NULL || dmaLock == NULL || devices == NULL) {
		return false;
	}

	poolMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, POOL_SIZE,
	    0x7ffff000ULL);
	if (poolMem == NULL) {
		IOLog("XHCI: no DMA memory\n");
		return false;
	}
	poolMem->performOperation(kIOMemoryIncoherentIOFlush, 0, POOL_SIZE);
	poolMap = poolMem->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
	if (poolMap == NULL) {
		return false;
	}
	pool = (uint8_t *)poolMap->getVirtualAddress();
	if (!RPiDMAAddress(provider, poolMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &poolPhys)) {
		IOLog("XHCI: the controller cannot reach its DMA memory\n");
		return false;
	}
	if (poolPhys + POOL_SIZE > (1ULL << 32) && !(hcc1 & 1)) {
		IOLog("XHCI: DMA address 0x%llx needs 64-bit addressing, which the controller lacks\n", poolPhys);
		return false;
	}
	poolUsed = 0;

	if (!resetController()) {
		IOLog("XHCI: controller reset timed out\n");
		return false;
	}

	uint64_t phys;
	dcbaa = (uint64_t *)allocDMA((maxSlots + 1) * sizeof(uint64_t), 64, &phys);
	w64(op, OP_DCBAAP, phys);
	if (scratchpads > 0) {
		uint64_t arrayPhys;
		uint64_t *array = (uint64_t *)allocDMA(scratchpads * sizeof(uint64_t), 64, &arrayPhys);
		for (uint32_t i = 0; array != NULL && i < scratchpads; i++) {
			allocDMA(4096, 4096, &array[i]);
		}
		dcbaa[0] = arrayPhys;
	}
	w32(op, OP_CONFIG, maxSlots);

	cmdRing.trbs = (volatile struct TRB *)allocDMA(RING_TRBS * sizeof(struct TRB), 4096, &cmdRing.phys);
	initRing(&cmdRing);
	w64(op, OP_CRCR, cmdRing.phys | 1);

	events = (volatile struct TRB *)allocDMA(RING_TRBS * sizeof(struct TRB), 4096, &eventsPhys);
	uint64_t erstPhys;
	uint64_t *erst = (uint64_t *)allocDMA(16, 64, &erstPhys);
	if (dcbaa == NULL || cmdRing.trbs == NULL || events == NULL || erst == NULL) {
		return false;
	}
	erst[0] = eventsPhys;
	erst[1] = RING_TRBS;
	eventDequeue = 0;
	eventCycle = 1;
	w32(rt, RT_ERSTSZ, 1);
	w64(rt, RT_ERDP, eventsPhys);
	w64(rt, RT_ERSTBA, erstPhys);
	w32(rt, RT_IMOD, 4000);
	w32(rt, RT_IMAN, IMAN_IE | IMAN_IP);

	workLoop = IOWorkLoop::workLoop();
	intSource = IOInterruptEventSource::interruptEventSource(this,
	    OSMemberFunctionCast(IOInterruptEventAction, this, &XHCIController::interruptOccurred), provider, 0);
	if (workLoop == NULL || intSource == NULL || workLoop->addEventSource(intSource) != kIOReturnSuccess) {
		IOLog("XHCI: no interrupt\n");
		return false;
	}
	intSource->enable();

	w32(op, OP_USBCMD, USBCMD_RS | USBCMD_INTE);
	IOLog("XHCI: xHCI %x.%02x, %u slots, %u ports, %u-byte contexts, %u scratchpad pages\n",
	    r32(cap, CAP_CAPLENGTH) >> 24, (r32(cap, CAP_CAPLENGTH) >> 16) & 0xff, maxSlots, maxPorts,
	    ctxSize, scratchpads);

	for (uint32_t port = 1; port <= maxPorts; port++) {
		uintptr_t sc = op + OP_PORTSC + 0x10 * (port - 1);
		uint32_t portsc = r32(sc, 0);
		if (!(portsc & PORTSC_PP)) {
			w32(sc, 0, (portsc & ~(PORTSC_PED | PORTSC_CHANGES)) | PORTSC_PP);
		}
	}

	registerService();
	thread_t thread;
	retain();
	if (kernel_thread_start(&XHCIController::enumerationThread, this, &thread) != KERN_SUCCESS) {
		release();
		return false;
	}
	thread_deallocate(thread);
	return true;
}

void
XHCIController::stop(IOService *provider)
{
	IOLockLock(portLock);
	stopping = true;
	IOLockWakeup(portLock, &portsChanged, false);
	IOLockUnlock(portLock);
	w32(op, OP_USBCMD, r32(op, OP_USBCMD) & ~USBCMD_RS);
	if (intSource != NULL) {
		intSource->disable();
		workLoop->removeEventSource(intSource);
	}
	super::stop(provider);
}

#pragma mark PIEUSBHost

#undef super
#define super IOService
OSDefineMetaClassAndAbstractStructors(PIEUSBHost, IOService);

void
PIEUSBHost::publishInterfaces(uint32_t slot, uint32_t generation, uint8_t port, uint8_t speed, const uint8_t *dev,
    const uint8_t *cfg, uint32_t cfgLength)
{
	PIEUSBInterface *intf = NULL;
	for (uint32_t off = 0; off + 2 <= cfgLength && cfg[off] >= 2; off += cfg[off]) {
		const uint8_t *desc = cfg + off;
		if (off + desc[0] > cfgLength) {
			break;
		}
		if (desc[1] == 4 && desc[0] >= 9) {
			if (intf != NULL) {
				if (intf->attach(this)) {
					intf->registerService();
				}
				OSSafeReleaseNULL(intf);
			}
			if (desc[3] != 0) {
				continue;
			}
			intf = OSTypeAlloc(PIEUSBInterface);
			if (intf == NULL || !intf->init()) {
				OSSafeReleaseNULL(intf);
				continue;
			}
			intf->controller = this;
			intf->slot = slot;
			intf->generation = generation;
			intf->number = desc[2];
			intf->setProperty("idVendor", OSReadLittleInt16(dev, 8), 32);
			intf->setProperty("idProduct", OSReadLittleInt16(dev, 10), 32);
			intf->setProperty("bInterfaceNumber", desc[2], 32);
			intf->setProperty("bInterfaceClass", desc[5], 32);
			intf->setProperty("bInterfaceSubClass", desc[6], 32);
			intf->setProperty("bInterfaceProtocol", desc[7], 32);
			intf->setProperty("port", port, 32);
			intf->setProperty("speed", speed, 32);
			char loc[8];
			snprintf(loc, sizeof(loc), "%u", desc[2]);
			intf->setLocation(loc);
		} else if (desc[1] == 0x21 && desc[0] >= 9 && intf != NULL && desc[6] == 0x22) {
			intf->setProperty("HIDReportDescriptorLength", OSReadLittleInt16(desc, 7), 32);
		} else if (desc[1] == 5 && desc[0] >= 7 && (desc[2] & 0xf) != 0 && intf != NULL &&
		    intf->numEndpoints < sizeof(intf->endpoints) / sizeof(intf->endpoints[0])) {
			struct PIEUSBEndpoint *ep = &intf->endpoints[intf->numEndpoints++];
			ep->address = desc[2];
			ep->attributes = desc[3];
			ep->maxPacket = OSReadLittleInt16(desc, 4);
			ep->interval = desc[6];
		}
	}
	if (intf != NULL) {
		if (intf->attach(this)) {
			intf->registerService();
		}
		intf->release();
	}
}

void
PIEUSBHost::terminateInterfaces(uint32_t slot, uint32_t generation)
{
	OSIterator *children = getChildIterator(gIOServicePlane);
	OSArray *gone = OSArray::withCapacity(4);
	if (children != NULL && gone != NULL) {
		while (OSObject *child = children->getNextObject()) {
			PIEUSBInterface *intf = OSDynamicCast(PIEUSBInterface, child);
			if (intf != NULL && intf->slot == slot && intf->generation == generation) {
				gone->setObject(intf);
			}
		}
	}
	OSSafeReleaseNULL(children);
	if (gone != NULL) {
		for (unsigned i = 0; i < gone->getCount(); i++) {
			((IOService *)gone->getObject(i))->terminate();
		}
		gone->release();
	}
}

#pragma mark PIEUSBInterface

#undef super
#define super IOService
OSDefineMetaClassAndStructors(PIEUSBInterface, IOService);

IOReturn
PIEUSBInterface::controlRequest(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
    uint16_t wIndex, uint16_t wLength, void *data)
{
	return controller->controlTransfer(slot, bmRequestType, bRequest, wValue, wIndex, wLength, data, NULL,
	           generation);
}

IOReturn
PIEUSBInterface::bulkTransfer(uint8_t endpointAddress, void *data, uint32_t length, uint32_t *actual)
{
	for (unsigned i = 0; i < numEndpoints; i++) {
		if (endpoints[i].address == endpointAddress && (endpoints[i].attributes & 3) == 2) {
			return controller->bulkTransfer(slot, &endpoints[i], data, length, actual, generation);
		}
	}
	return controller->current(slot, generation) ? kIOReturnNotFound : kIOReturnNoDevice;
}

IOReturn
PIEUSBInterface::clearHalt(uint8_t endpointAddress)
{
	IOReturn ret = controlRequest(0x02, 1, 0, endpointAddress, 0, NULL);
	IOReturn reset = controller->resetPipe(slot, endpointAddress, generation);
	return ret != kIOReturnSuccess ? ret : reset;
}

IOReturn
PIEUSBInterface::startInterruptIn(uint8_t endpointAddress, OSObject *target, PIEUSBInterruptAction action)
{
	for (unsigned i = 0; i < numEndpoints; i++) {
		if (endpoints[i].address == endpointAddress && (endpoints[i].attributes & 3) == 3) {
			return controller->startInterruptIn(slot, &endpoints[i], target, action, generation);
		}
	}
	return controller->current(slot, generation) ? kIOReturnNotFound : kIOReturnNoDevice;
}

IOReturn
PIEUSBInterface::startBulkQueue(uint8_t endpointAddress, uint32_t size, uint32_t count, OSObject *target,
    PIEUSBQueueAction action)
{
	for (unsigned i = 0; i < numEndpoints; i++) {
		if (endpoints[i].address == endpointAddress && (endpoints[i].attributes & 3) == 2) {
			return controller->startQueue(slot, &endpoints[i], size, count, target, action, generation);
		}
	}
	return controller->current(slot, generation) ? kIOReturnNotFound : kIOReturnNoDevice;
}

IOReturn
PIEUSBInterface::queueBulkOut(uint8_t endpointAddress, const void *data, uint32_t length)
{
	return controller->queueOut(slot, endpointAddress, data, length, generation);
}

IOReturn
PIEUSBInterface::stopBulkQueue(uint8_t endpointAddress)
{
	return controller->stopQueue(slot, endpointAddress, generation);
}
