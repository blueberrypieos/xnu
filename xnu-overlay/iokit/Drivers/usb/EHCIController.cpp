/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <mach/mach_time.h>
#include <libkern/OSByteOrder.h>

#include "../rpi/RPiPlatform.h"
#include "EHCIController.h"

/* EHCI capability and operational registers. */
enum {
	CAP_HCSPARAMS = 0x04, CAP_HCCPARAMS = 0x08,
	OP_USBCMD = 0x00, OP_USBSTS = 0x04, OP_USBINTR = 0x08, OP_CTRLDSSEGMENT = 0x10,
	OP_ASYNCLISTADDR = 0x18, OP_CONFIGFLAG = 0x40, OP_PORTSC = 0x44,
};

enum {
	CMD_RS = 1U << 0, CMD_HCRESET = 1U << 1, CMD_ASE = 1U << 5, CMD_ITC_SHIFT = 16,
	STS_INT = 1U << 0, STS_ERR = 1U << 1, STS_PCD = 1U << 2, STS_HSE = 1U << 4,
	STS_HCH = 1U << 12, STS_ASS = 1U << 15,
	HCS_PPC = 1U << 4, HCC_64BIT = 1U << 0,

	PORT_CCS = 1U << 0, PORT_CSC = 1U << 1, PORT_PE = 1U << 2, PORT_PEC = 1U << 3, PORT_OCC = 1U << 5,
	PORT_RESET = 1U << 8, PORT_LINE_SHIFT = 10, PORT_PP = 1U << 12, PORT_OWNER = 1U << 13,
	PORT_W1C = PORT_CSC | PORT_PEC | PORT_OCC, PORT_LINE_K = 1,
};

/* Link pointers, queue heads and transfer descriptors. */
enum {
	LINK_T = 1U << 0, LINK_QH = 1U << 1,

	QH_EPS_HIGH = 2U << 12, QH_DTC = 1U << 14, QH_HEAD = 1U << 15, QH_RL_SHIFT = 28, QH_MULT1 = 1U << 30,

	TD_PING = 1U << 0, TD_SPLIT = 1U << 1, TD_MISSED = 1U << 2, TD_XACT = 1U << 3, TD_BABBLE = 1U << 4,
	TD_BUFFER = 1U << 5, TD_HALTED = 1U << 6, TD_ACTIVE = 1U << 7,
	TD_PID_OUT = 0U << 8, TD_PID_IN = 1U << 8, TD_PID_SETUP = 2U << 8,
	TD_CERR3 = 3U << 10, TD_IOC = 1U << 15, TD_BYTES_SHIFT = 16, TD_TOGGLE = 1U << 31,
};

#define TD_BYTES(tok)   (((tok) >> TD_BYTES_SHIFT) & 0x7fff)

struct QTD {
	uint32_t next, alt, token, buffer[5], bufferHigh[5];
	uint32_t pad[3];
};

struct QHW {
	uint32_t link, endpoint, caps, current;
	uint32_t next, alt, token, buffer[5], bufferHigh[5];
	uint32_t pad[7];
};

/* Queue heads are aligned on 128 bytes, so that none spans a 4 KiB page (EHCI 1.0 section 3). */
#define QH_ALIGN        128

#define MAX_DEVICES     16
#define HUB_PORTS       15
#define POOL_SIZE       (1024 * 1024)
#define CHUNK           16384
#define BULK_MAX        65536
#define BULK_TDS        (BULK_MAX / CHUNK)
#define QUEUE_BUFFERS_MAX      32
#define QUEUE_BYTES_MAX        (256 * 1024)
#define TDS_MAX         (2 * QUEUE_BUFFERS_MAX * 4 + 1)
#define OWNER_NONE      0xff
#define CTL_TIMEOUT_MS  2000
#define BULK_TIMEOUT_MS 10000
#define SPEED_HIGH      3

struct EHCIQH {
	volatile struct QHW *hw;
	uint32_t phys;
	struct EHCIQH *next;
	bool linked;
};

struct EHCIPipe {
	struct EHCIQH *qh;
	struct PIEUSBEndpoint ep;
	uint32_t slot, generation, index;
	bool in, queue, halted;
	IOBufferMemoryDescriptor *mem;
	IOMemoryMap *map;
	uint8_t *buffers;
	uint32_t buffersPhys;
	uint32_t size, count, chunk, chunks;
	volatile struct QTD *tds;
	uint32_t tdsPhys, ntds;
	uint32_t head, tail, busy;
	uint8_t owner[TDS_MAX];
	uint32_t lengths[QUEUE_BUFFERS_MAX];
	OSObject *target;
	PIEUSBQueueAction action;
	struct EHCIPipe *nextActive;
};

struct EHCIDevice {
	bool used, gone;
	uint32_t generation;
	uint8_t address, port, depth;
	bool isHub;
	uint8_t hubPorts;
	uint8_t hubChild[HUB_PORTS + 1];
	struct EHCIQH *ep0;
	volatile struct QTD *ctl;
	uint32_t ctlPhys;
	uint8_t *setup, *buf;
	uint32_t setupPhys, bufPhys;
	struct EHCIPipe *pipes[32];
};

static inline uint32_t
pipe_index(uint8_t endpointAddress)
{
	return (endpointAddress & 0xfU) | ((endpointAddress & 0x80) ? 16U : 0U);
}

static void
td_buffers(volatile struct QTD *td, uint32_t phys)
{
	td->buffer[0] = phys;
	for (int i = 1; i < 5; i++) {
		td->buffer[i] = (phys & ~0xfffU) + 0x1000U * (uint32_t)i;
	}
	for (int i = 0; i < 5; i++) {
		td->bufferHigh[i] = 0;
	}
}

static inline uint32_t
td_token(uint32_t pid, uint32_t bytes, uint32_t toggle)
{
	return TD_ACTIVE | pid | TD_CERR3 | TD_IOC | bytes << TD_BYTES_SHIFT | (toggle ? TD_TOGGLE : 0);
}

static IOReturn
td_error(uint32_t token)
{
	return (token & (TD_XACT | TD_BABBLE | TD_BUFFER)) ? kIOReturnIOError : kIOReturnNotResponding;
}

/*
 * A chain of transfer descriptors: kIOReturnSuccess once all of them are
 * done, or with shortEnds once one came back short, kIOReturnNotReady
 * while one is active, or the error of one that halted.
 */
static IOReturn
chain_state(volatile struct QTD *tds, uint32_t count, bool shortEnds)
{
	for (uint32_t i = 0; i < count; i++) {
		uint32_t tok = tds[i].token;
		if (tok & TD_HALTED) {
			return td_error(tok);
		}
		if (tok & TD_ACTIVE) {
			return kIOReturnNotReady;
		}
		if (shortEnds && TD_BYTES(tok) != 0) {
			break;
		}
	}
	return kIOReturnSuccess;
}

/*
 * Starts a chain on a queue head in the schedule that is neither active nor
 * halted, with one store to the pointer the controller takes: the
 * alternate pointer after a short packet, the next pointer otherwise
 * (EHCI 1.0 section 4.10.2).
 */
static void
qh_start(volatile struct QHW *hw, uint32_t first)
{
	__builtin_arm_dmb(0xf);
	if (TD_BYTES(hw->token) != 0 && !(hw->alt & LINK_T)) {
		hw->alt = first;
	} else {
		hw->next = first;
	}
	__builtin_arm_dmb(0xf);
}

#define super PIEUSBHost
OSDefineMetaClassAndStructors(EHCIController, PIEUSBHost);

#pragma mark DMA memory and queue heads

/* size bytes of the pool, zeroed, at a bus address that is a multiple of align. */
void *
EHCIController::allocDMA(size_t size, size_t align, uint32_t *phys)
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
		uint64_t at = ((uint64_t)poolPhys + poolUsed + align - 1) & ~(uint64_t)(align - 1);
		size_t start = (size_t)(at - poolPhys);
		if (start + size <= POOL_SIZE) {
			poolUsed = start + size;
			virt = pool + start;
			*phys = (uint32_t)at;
		}
	}
	IOLockUnlock(dmaLock);
	if (virt != NULL) {
		bzero(virt, size);
	}
	return virt;
}

void
EHCIController::freeDMA(void *virt, uint32_t phys, size_t size)
{
	if (virt == NULL) {
		return;
	}
	IOLockLock(dmaLock);
	if (dmaFreeCount < sizeof(dmaFree) / sizeof(dmaFree[0])) {
		dmaFree[dmaFreeCount].virt = virt;
		dmaFree[dmaFreeCount].phys = phys;
		dmaFree[dmaFreeCount].size = size;
		dmaFreeCount++;
	}
	IOLockUnlock(dmaLock);
}

struct EHCIQH *
EHCIController::newQH(uint8_t address, uint8_t endpoint, uint16_t maxPacket, bool control)
{
	struct EHCIQH *qh = IOMallocType(struct EHCIQH);
	qh->hw = (volatile struct QHW *)allocDMA(sizeof(struct QHW), QH_ALIGN, &qh->phys);
	if (qh->hw == NULL) {
		IOFreeType(qh, struct EHCIQH);
		return NULL;
	}
	qh->hw->link = LINK_T;
	qh->hw->endpoint = (uint32_t)(address & 0x7f) | (uint32_t)(endpoint & 0xf) << 8 | QH_EPS_HIGH |
	    (control ? QH_DTC : 0) | (uint32_t)(maxPacket & 0x7ff) << 16 | 4U << QH_RL_SHIFT;
	qh->hw->caps = QH_MULT1;
	qh->hw->next = LINK_T;
	qh->hw->alt = LINK_T;
	qh->hw->token = 0;
	return qh;
}

void
EHCIController::freeQH(struct EHCIQH *qh)
{
	if (qh != NULL) {
		freeDMA((void *)(uintptr_t)qh->hw, qh->phys, sizeof(struct QHW));
		IOFreeType(qh, struct EHCIQH);
	}
}

/* Into the asynchronous schedule, after its head. With lock held. */
void
EHCIController::linkQH(struct EHCIQH *qh)
{
	qh->hw->link = asyncHead->hw->link;
	__builtin_arm_dmb(0xf);
	asyncHead->hw->link = qh->phys | LINK_QH;
	__builtin_arm_dmb(0xf);
	qh->next = asyncHead->next;
	asyncHead->next = qh;
	qh->linked = true;
}

/* Out of the schedule's list, the link kept consistent. With lock held. */
void
EHCIController::unlinkEntry(struct EHCIQH *qh)
{
	if (qh == NULL || !qh->linked) {
		return;
	}
	struct EHCIQH *prev = asyncHead;
	while (prev->next != NULL && prev->next != qh) {
		prev = prev->next;
	}
	if (prev->next == qh) {
		prev->hw->link = qh->hw->link;
		__builtin_arm_dmb(0xf);
		prev->next = qh->next;
	}
	qh->next = NULL;
	qh->linked = false;
}

/*
 * Waits up to 20 ms for the asynchronous schedule's status to read on.
 * False, and the controller dead, if it does not, or if the controller has
 * halted or reports a host system error. dead is read after USBSTS: the
 * interrupt handler sets it before it acknowledges a host system error.
 */
bool
EHCIController::waitAsync(bool on)
{
	for (int i = 0; i < 400; i++) {
		uint32_t sts = r32(op, OP_USBSTS);
		__builtin_arm_dmb(0xf);
		if (dead) {
			return false;
		}
		if (sts & (STS_HCH | STS_HSE)) {
			haltDied(sts);
			return false;
		}
		if (((sts & STS_ASS) != 0) == on) {
			return true;
		}
		IODelay(50);
	}
	controllerDied(on ? "the asynchronous schedule did not start" : "the asynchronous schedule did not stop",
	    r32(op, OP_USBSTS));
	return false;
}

/*
 * The asynchronous schedule stopped: once its status reads 0, the
 * controller holds no pointer into it and reads none of its queue heads or
 * descriptors until resumeAsync (EHCI 1.0 sections 4.8 and 4.8.2). Its
 * enable bit changes only while the status equals it. False once the
 * controller has stopped working. With lock held.
 */
bool
EHCIController::pauseAsync(void)
{
	if (dead || !waitAsync(true)) {
		return false;
	}
	w32(op, OP_USBCMD, r32(op, OP_USBCMD) & ~CMD_ASE);
	return waitAsync(false);
}

/*
 * The schedule started again at its head, which never leaves it: the
 * controller would otherwise start at the queue head it stopped before,
 * which may be one that has left. False, and the controller dead, if the
 * schedule's status does not read 1 within 20 ms. With lock held.
 */
bool
EHCIController::resumeAsync(void)
{
	w32(op, OP_ASYNCLISTADDR, asyncHead->phys);
	w32(op, OP_USBCMD, r32(op, OP_USBCMD) | CMD_ASE);
	return waitAsync(true);
}

/*
 * Out of the asynchronous schedule, while the schedule is stopped, so that
 * the controller holds no pointer to the queue head or its descriptors
 * once this returns true, whether the queue head was active or not. False
 * once the controller has stopped working, or the schedule does not start
 * again: then the queue head's memory must be kept. With lock held.
 */
bool
EHCIController::unlinkQH(struct EHCIQH *qh)
{
	if (qh == NULL) {
		return true;
	}
	if (!qh->linked) {
		return !dead;
	}
	if (!pauseAsync()) {
		return false;
	}
	unlinkEntry(qh);
	return resumeAsync();
}

/*
 * A queue head stopped in the schedule, its overlay emptied but for the
 * token bits in keep, while the schedule is stopped. False if the
 * controller may still hold it.
 */
bool
EHCIController::quiesce(struct EHCIQH *qh, uint32_t keep)
{
	if (!pauseAsync()) {
		return false;
	}
	qh->hw->next = LINK_T;
	qh->hw->alt = LINK_T;
	qh->hw->token &= keep;
	__builtin_arm_dmb(0xf);
	return resumeAsync();
}

/*
 * The controller stopped working: every call then returns
 * kIOReturnNoDevice, waits end, and no memory it may hold is freed.
 */
void
EHCIController::controllerDied(const char *why, uint32_t sts)
{
	IOLockLock(waitLock);
	bool first = !dead;
	dead = true;
	waitEvents++;
	IOLockWakeup(waitLock, &waitEvents, false);
	IOLockUnlock(waitLock);
	if (first && !stopping) {
		IOLog("EHCI: %s, USBSTS 0x%08x; the controller takes no more requests, and its memory is kept\n", why, sts);
	}
}

void
EHCIController::haltDied(uint32_t sts)
{
	controllerDied((sts & STS_HSE) ? "host system error" : "the controller has halted", sts);
}

void
EHCIController::freePipe(struct EHCIPipe *p)
{
	if (p == NULL) {
		return;
	}
	freeQH(p->qh);
	freeDMA((void *)(uintptr_t)p->tds, p->tdsPhys, p->ntds * sizeof(struct QTD));
	OSSafeReleaseNULL(p->map);
	OSSafeReleaseNULL(p->mem);
	IOFreeType(p, struct EHCIPipe);
}

#pragma mark Transfers

/*
 * The controller still working, as a request is admitted: a halt or a host
 * system error makes it dead at once. None is admitted once stop has begun.
 */
bool
EHCIController::working(void)
{
	if (dead || op == 0 || stopping) {
		return false;
	}
	uint32_t sts = r32(op, OP_USBSTS);
	if (sts & (STS_HCH | STS_HSE)) {
		haltDied(sts);
		return false;
	}
	return true;
}

bool
EHCIController::current(uint32_t slot, uint32_t generation)
{
	return slot < MAX_DEVICES && devices[slot].used && !devices[slot].gone &&
	       (generation == 0 || devices[slot].generation == generation) && working();
}

/*
 * Waits until a chain of transfer descriptors has finished, as chain_state
 * says. A device that goes, or a controller that halts or stops working,
 * ends the wait.
 */
IOReturn
EHCIController::waitTransfer(uint32_t slot, volatile void *chain, uint32_t count, bool shortEnds, uint32_t timeoutMS)
{
	volatile struct QTD *tds = (volatile struct QTD *)chain;
	uint64_t deadline;
	clock_interval_to_deadline(timeoutMS, kMillisecondScale, &deadline);
	IOLockLock(waitLock);
	for (;;) {
		__builtin_arm_dmb(0xf);
		IOReturn ret = chain_state(tds, count, shortEnds);
		if (ret != kIOReturnNotReady) {
			IOLockUnlock(waitLock);
			return ret;
		}
		if (devices[slot].gone || dead || stopping) {
			IOLockUnlock(waitLock);
			return kIOReturnNoDevice;
		}
		uint32_t sts = r32(op, OP_USBSTS);
		if (sts & (STS_HCH | STS_HSE)) {
			IOLockUnlock(waitLock);
			haltDied(sts);
			return kIOReturnNoDevice;
		}
		if (mach_absolute_time() >= deadline) {
			IOLockUnlock(waitLock);
			return kIOReturnTimeout;
		}
		uint64_t wake;
		clock_interval_to_deadline(10, kMillisecondScale, &wake);
		IOLockSleepDeadline(waitLock, &waitEvents, wake < deadline ? wake : deadline, THREAD_UNINT);
	}
}

/* One control transfer on a device's endpoint 0, with lock held. */
IOReturn
EHCIController::controlLocked(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
    uint16_t wIndex, uint16_t wLength, void *data, uint32_t *actual)
{
	struct EHCIDevice *d = &devices[slot];
	if (wLength > 4096 || d->ep0 == NULL) {
		return kIOReturnBadArgument;
	}
	bool in = (bmRequestType & 0x80) != 0;
	d->setup[0] = bmRequestType;
	d->setup[1] = bRequest;
	OSWriteLittleInt16(d->setup, 2, wValue);
	OSWriteLittleInt16(d->setup, 4, wIndex);
	OSWriteLittleInt16(d->setup, 6, wLength);
	if (!in && wLength > 0) {
		bcopy(data, d->buf, wLength);
	}
	volatile struct QTD *setup = &d->ctl[0];
	volatile struct QTD *dataTD = wLength > 0 ? &d->ctl[1] : NULL;
	volatile struct QTD *status = &d->ctl[wLength > 0 ? 2 : 1];
	uint32_t count = wLength > 0 ? 3 : 2;
	uint32_t statusPhys = d->ctlPhys + (count - 1) * (uint32_t)sizeof(struct QTD);

	setup->token = 0;
	status->token = 0;
	setup->next = dataTD != NULL ? d->ctlPhys + (uint32_t)sizeof(struct QTD) : statusPhys;
	setup->alt = LINK_T;
	td_buffers(setup, d->setupPhys);
	if (dataTD != NULL) {
		dataTD->token = 0;
		dataTD->next = statusPhys;
		dataTD->alt = statusPhys;
		td_buffers(dataTD, d->bufPhys);
	}
	status->next = LINK_T;
	status->alt = LINK_T;
	td_buffers(status, d->bufPhys);
	__builtin_arm_dmb(0xf);
	status->token = td_token((in && wLength > 0) ? TD_PID_OUT : TD_PID_IN, 0, 1);
	if (dataTD != NULL) {
		dataTD->token = td_token(in ? TD_PID_IN : TD_PID_OUT, wLength, 1);
	}
	setup->token = td_token(TD_PID_SETUP, 8, 0);
	qh_start(d->ep0->hw, d->ctlPhys);

	IOReturn ret = waitTransfer(slot, d->ctl, count, false, CTL_TIMEOUT_MS);
	if (ret != kIOReturnSuccess && ret != kIOReturnNoDevice) {
		if (!quiesce(d->ep0, TD_TOGGLE)) {
			ret = kIOReturnNoDevice;
		} else if (ret == kIOReturnTimeout) {
			ret = chain_state(d->ctl, count, false);
		}
		if (ret == kIOReturnNotReady) {
			ret = kIOReturnTimeout;
		}
	}
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint32_t moved = dataTD != NULL ? wLength - TD_BYTES(dataTD->token) : 0;
	if (moved > wLength) {
		moved = 0;
	}
	if (in && wLength > 0) {
		bcopy(d->buf, data, moved);
		bzero((uint8_t *)data + moved, wLength - moved);
	}
	if (actual != NULL) {
		*actual = moved;
	}
	return kIOReturnSuccess;
}

IOReturn
EHCIController::controlTransfer(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
    uint16_t wIndex, uint16_t wLength, void *data, uint32_t *actual, uint32_t generation)
{
	IOLockLock(lock);
	IOReturn ret = current(slot, generation) ?
	    controlLocked(slot, bmRequestType, bRequest, wValue, wIndex, wLength, data, actual) : kIOReturnNoDevice;
	IOLockUnlock(lock);
	return ret;
}

IOReturn
EHCIController::startInterruptIn(uint32_t slot, const struct PIEUSBEndpoint *ep, OSObject *target,
    PIEUSBInterruptAction action, uint32_t generation)
{
	(void)ep;
	(void)target;
	(void)action;
	return current(slot, generation) ? kIOReturnUnsupported : kIOReturnNoDevice;
}

IOReturn
EHCIController::bulkTransfer(uint32_t slot, const struct PIEUSBEndpoint *ep, void *data, uint32_t length,
    uint32_t *actual, uint32_t generation)
{
	if (!current(slot, generation)) {
		return kIOReturnNoDevice;
	}
	if (length > BULK_MAX || (ep->address & 0xf) == 0 || (ep->attributes & 3) != 2 || (ep->maxPacket & 0x7ff) == 0) {
		return kIOReturnBadArgument;
	}
	bool in = (ep->address & 0x80) != 0;
	uint32_t index = pipe_index(ep->address);
	IOLockLock(lock);
	if (!current(slot, generation)) {
		IOLockUnlock(lock);
		return kIOReturnNoDevice;
	}
	struct EHCIDevice *d = &devices[slot];
	struct EHCIPipe *p = d->pipes[index];
	if (p != NULL && p->queue) {
		IOLockUnlock(lock);
		return kIOReturnBusy;
	}
	if (p == NULL) {
		p = IOMallocType(struct EHCIPipe);
		p->ep = *ep;
		p->slot = slot;
		p->index = index;
		p->in = in;
		p->ntds = BULK_TDS + 1;
		p->tds = (volatile struct QTD *)allocDMA(p->ntds * sizeof(struct QTD), 64, &p->tdsPhys);
		p->mem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, BULK_MAX, 0x7fff0000ULL);
		if (p->mem != NULL) {
			p->mem->performOperation(kIOMemoryIncoherentIOFlush, 0, BULK_MAX);
			p->map = p->mem->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
		}
		uint64_t bus = 0;
		p->qh = newQH(d->address, ep->address & 0xf, ep->maxPacket & 0x7ff, false);
		if (p->tds == NULL || p->map == NULL || p->qh == NULL || !RPiDMAAddress(nub,
		    p->mem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &bus) || bus + BULK_MAX > (1ULL << 32)) {
			IOLockUnlock(lock);
			freePipe(p);
			return kIOReturnNoMemory;
		}
		p->buffers = (uint8_t *)p->map->getVirtualAddress();
		p->buffersPhys = (uint32_t)bus;
		d->pipes[index] = p;
		linkQH(p->qh);
	}

	uint32_t n = length == 0 ? 1 : (length + CHUNK - 1) / CHUNK;
	uint32_t stopPhys = p->tdsPhys + BULK_TDS * (uint32_t)sizeof(struct QTD);
	volatile struct QTD *stopTD = &p->tds[BULK_TDS];
	stopTD->next = LINK_T;
	stopTD->alt = LINK_T;
	stopTD->token = 0;
	if (!in && length > 0) {
		bcopy(data, p->buffers, length);
	}
	for (uint32_t i = 0; i < n; i++) {
		volatile struct QTD *td = &p->tds[i];
		td->token = 0;
		td->next = i + 1 < n ? p->tdsPhys + (i + 1) * (uint32_t)sizeof(struct QTD) : stopPhys;
		td->alt = stopPhys;
		td_buffers(td, p->buffersPhys + i * CHUNK);
	}
	__builtin_arm_dmb(0xf);
	for (uint32_t i = n; i-- > 0;) {
		uint32_t len = length - i * CHUNK < CHUNK ? length - i * CHUNK : CHUNK;
		p->tds[i].token = td_token(in ? TD_PID_IN : TD_PID_OUT, length == 0 ? 0 : len, 0) & ~TD_TOGGLE;
		__builtin_arm_dmb(0xf);
	}
	qh_start(p->qh->hw, p->tdsPhys);

	IOReturn ret = waitTransfer(slot, p->tds, n, in, BULK_TIMEOUT_MS);
	if (ret != kIOReturnSuccess && ret != kIOReturnNoDevice) {
		if (!quiesce(p->qh, TD_TOGGLE)) {
			ret = kIOReturnNoDevice;
		} else if (ret == kIOReturnTimeout) {
			ret = chain_state(p->tds, n, in);
		}
		if (ret == kIOReturnNotReady) {
			ret = kIOReturnTimeout;
		}
	}
	uint32_t moved = 0;
	if (ret == kIOReturnSuccess) {
		for (uint32_t i = 0; i < n; i++) {
			uint32_t len = length - i * CHUNK < CHUNK ? length - i * CHUNK : CHUNK;
			uint32_t left = TD_BYTES(p->tds[i].token);
			moved += left <= len ? len - left : 0;
			if (left != 0) {
				break;
			}
		}
		if (in && moved > 0) {
			bcopy(p->buffers, data, moved);
		}
		if (actual != NULL) {
			*actual = moved;
		}
	}
	IOLockUnlock(lock);
	return ret;
}

IOReturn
EHCIController::resetPipe(uint32_t slot, uint8_t endpointAddress, uint32_t generation)
{
	IOLockLock(lock);
	if (!current(slot, generation)) {
		IOLockUnlock(lock);
		return kIOReturnNoDevice;
	}
	if ((endpointAddress & 0xf) == 0) {
		IOLockUnlock(lock);
		return kIOReturnBadArgument;
	}
	IOReturn ret = kIOReturnSuccess;
	struct EHCIPipe *p = devices[slot].pipes[pipe_index(endpointAddress)];
	if (p != NULL && !p->queue && !quiesce(p->qh, 0)) {
		ret = kIOReturnNoDevice;
	}
	IOLockUnlock(lock);
	return ret;
}

#pragma mark Bulk queues

/* Buffer b of an IN queue queued again: its descriptors, the first last. */
void
EHCIController::armIn(struct EHCIPipe *p, uint32_t b)
{
	for (uint32_t c = p->chunks; c-- > 0;) {
		volatile struct QTD *td = &p->tds[b * p->chunks + c];
		td_buffers(td, p->buffersPhys + b * p->size + c * p->chunk);
		__builtin_arm_dmb(0xf);
		td->token = td_token(TD_PID_IN, p->chunk, 0) & ~TD_TOGGLE;
	}
	__builtin_arm_dmb(0xf);
}

/*
 * Completions in order, on the work loop. An IN buffer has finished when
 * a descriptor came back short or its last one is done; the controller
 * has then moved on to the next buffer.
 */
void
EHCIController::processQueue(struct EHCIPipe *p)
{
	while (!p->halted && p->in) {
		uint32_t b = p->head, total = 0;
		bool complete = false;
		uint32_t bad = 0;
		for (uint32_t c = 0; c < p->chunks; c++) {
			volatile struct QTD *td = &p->tds[b * p->chunks + c];
			uint32_t tok = td->token;
			if (tok & TD_HALTED) {
				bad = tok;
				break;
			}
			if (tok & TD_ACTIVE) {
				break;
			}
			uint32_t left = TD_BYTES(tok);
			total += left <= p->chunk ? p->chunk - left : 0;
			if (left != 0 || c + 1 == p->chunks) {
				for (uint32_t rest = c + 1; rest < p->chunks; rest++) {
					p->tds[b * p->chunks + rest].token = 0;
				}
				complete = true;
				break;
			}
		}
		if (bad != 0) {
			p->halted = true;
			IOLog("EHCI: address %u endpoint 0x%x: bulk queue halted, status 0x%02x\n",
			    devices[p->slot].address, p->ep.address, bad & 0xff);
			p->action(p->target, NULL, 0, td_error(bad));
			return;
		}
		if (!complete) {
			return;
		}
		p->action(p->target, p->buffers + (size_t)b * p->size, total, kIOReturnSuccess);
		armIn(p, b);
		p->head = (b + 1) % p->count;
	}
	while (!p->halted && !p->in && p->head != p->tail) {
		uint32_t tok = p->tds[p->head].token;
		if (tok & TD_ACTIVE) {
			return;
		}
		uint8_t owner = p->owner[p->head];
		if (tok & TD_HALTED) {
			p->halted = true;
			IOLog("EHCI: address %u endpoint 0x%x: bulk queue halted, status 0x%02x\n",
			    devices[p->slot].address, p->ep.address, tok & 0xff);
			p->action(p->target, NULL, 0, td_error(tok));
			return;
		}
		p->owner[p->head] = OWNER_NONE;
		p->head = (p->head + 1) % p->ntds;
		if (owner != OWNER_NONE && (p->busy & (1U << owner))) {
			p->busy &= ~(1U << owner);
			p->action(p->target, NULL, p->lengths[owner], kIOReturnSuccess);
		}
	}
}

IOReturn
EHCIController::gatedQueueAdd(OSObject *owner, void *pipe, void *arg1, void *arg2, void *arg3)
{
	(void)arg1;
	(void)arg2;
	(void)arg3;
	EHCIController *self = (EHCIController *)owner;
	struct EHCIPipe *p = (struct EHCIPipe *)pipe;
	self->devices[p->slot].pipes[p->index] = p;
	p->nextActive = self->activeQueues;
	self->activeQueues = p;
	return kIOReturnSuccess;
}

IOReturn
EHCIController::gatedQueueTake(OSObject *owner, void *slotArg, void *indexArg, void *generationArg, void *out)
{
	EHCIController *self = (EHCIController *)owner;
	uint32_t slot = (uint32_t)(uintptr_t)slotArg, index = (uint32_t)(uintptr_t)indexArg;
	if (slot == 0 || slot >= MAX_DEVICES) {
		return kIOReturnNoDevice;
	}
	struct EHCIDevice *d = &self->devices[slot];
	if (!d->used || d->generation != (uint32_t)(uintptr_t)generationArg) {
		return kIOReturnNoDevice;
	}
	struct EHCIPipe *p = d->pipes[index];
	*(struct EHCIPipe **)out = NULL;
	if (p == NULL || !p->queue) {
		return kIOReturnSuccess;
	}
	d->pipes[index] = NULL;
	for (struct EHCIPipe **link = &self->activeQueues; *link != NULL; link = &(*link)->nextActive) {
		if (*link == p) {
			*link = p->nextActive;
			break;
		}
	}
	*(struct EHCIPipe **)out = p;
	return kIOReturnSuccess;
}

IOReturn
EHCIController::gatedDeviceTake(OSObject *owner, void *slotArg, void *out, void *arg2, void *arg3)
{
	(void)arg2;
	(void)arg3;
	EHCIController *self = (EHCIController *)owner;
	struct EHCIDevice *d = &self->devices[(uint32_t)(uintptr_t)slotArg];
	struct EHCIPipe **pipes = (struct EHCIPipe **)out;
	for (uint32_t i = 0; i < 32; i++) {
		pipes[i] = d->pipes[i];
		d->pipes[i] = NULL;
		if (pipes[i] == NULL || !pipes[i]->queue) {
			continue;
		}
		for (struct EHCIPipe **link = &self->activeQueues; *link != NULL; link = &(*link)->nextActive) {
			if (*link == pipes[i]) {
				*link = pipes[i]->nextActive;
				break;
			}
		}
	}
	return kIOReturnSuccess;
}

IOReturn
EHCIController::startQueue(uint32_t slot, const struct PIEUSBEndpoint *ep, uint32_t size, uint32_t count,
    OSObject *target, PIEUSBQueueAction action, uint32_t generation)
{
	if (!current(slot, generation)) {
		return kIOReturnNoDevice;
	}
	if (action == NULL || count < 1 || count > QUEUE_BUFFERS_MAX || size < 512 || size > 65536 ||
	    (size & (size - 1)) != 0 || (uint64_t)size * count > QUEUE_BYTES_MAX || (ep->address & 0xf) == 0 ||
	    (ep->attributes & 3) != 2 || (ep->maxPacket & 0x7ff) == 0) {
		return kIOReturnBadArgument;
	}
	bool in = (ep->address & 0x80) != 0;
	if (in && size % (ep->maxPacket & 0x7ffU) != 0) {
		return kIOReturnBadArgument;
	}
	uint32_t index = pipe_index(ep->address);
	IOLockLock(lock);
	if (!current(slot, generation)) {
		IOLockUnlock(lock);
		return kIOReturnNoDevice;
	}
	struct EHCIDevice *d = &devices[slot];
	if (d->pipes[index] != NULL) {
		IOLockUnlock(lock);
		return kIOReturnBusy;
	}
	struct EHCIPipe *p = IOMallocType(struct EHCIPipe);
	p->ep = *ep;
	p->slot = slot;
	p->generation = d->generation;
	p->index = index;
	p->in = in;
	p->queue = true;
	p->size = size;
	p->count = count;
	p->chunk = size < CHUNK ? size : CHUNK;
	p->chunks = size / p->chunk;
	p->ntds = in ? count * p->chunks : 2 * count * p->chunks + 1;
	p->target = target;
	p->action = action;
	memset(p->owner, OWNER_NONE, sizeof(p->owner));
	p->tds = (volatile struct QTD *)allocDMA(p->ntds * sizeof(struct QTD), 64, &p->tdsPhys);
	p->mem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, (size_t)size * count,
	    0x7fff0000ULL);
	if (p->mem != NULL) {
		p->mem->performOperation(kIOMemoryIncoherentIOFlush, 0, (size_t)size * count);
		p->map = p->mem->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
	}
	uint64_t bus = 0;
	p->qh = newQH(d->address, ep->address & 0xf, ep->maxPacket & 0x7ff, false);
	if (p->tds == NULL || p->map == NULL || p->qh == NULL || !RPiDMAAddress(nub,
	    p->mem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &bus) ||
	    bus + (uint64_t)size * count > (1ULL << 32)) {
		IOLockUnlock(lock);
		freePipe(p);
		return kIOReturnNoMemory;
	}
	p->buffers = (uint8_t *)p->map->getVirtualAddress();
	p->buffersPhys = (uint32_t)bus;

	for (uint32_t i = 0; i < p->ntds; i++) {
		volatile struct QTD *td = &p->tds[i];
		td->token = 0;
		if (in) {
			uint32_t b = i / p->chunks, c = i % p->chunks;
			uint32_t nextBuffer = p->tdsPhys + ((b + 1) % count) * p->chunks * (uint32_t)sizeof(struct QTD);
			td->next = c + 1 < p->chunks ? p->tdsPhys + (i + 1) * (uint32_t)sizeof(struct QTD) : nextBuffer;
			td->alt = nextBuffer;
		} else {
			td->next = p->tdsPhys + ((i + 1) % p->ntds) * (uint32_t)sizeof(struct QTD);
			td->alt = LINK_T;
		}
	}
	if (in) {
		for (uint32_t b = count; b-- > 0;) {
			armIn(p, b);
		}
	}
	p->qh->hw->next = p->tdsPhys;
	p->qh->hw->alt = LINK_T;
	p->qh->hw->token = 0;
	__builtin_arm_dmb(0xf);
	workLoop->runAction(&EHCIController::gatedQueueAdd, this, p);
	linkQH(p->qh);
	IOLockUnlock(lock);
	return kIOReturnSuccess;
}

struct EHCIQueueOutArgs {
	uint32_t slot, generation, index;
	const void *data;
	uint32_t length;
};

IOReturn
EHCIController::gatedQueueOut(OSObject *owner, void *args, void *arg1, void *arg2, void *arg3)
{
	(void)arg1;
	(void)arg2;
	(void)arg3;
	EHCIController *self = (EHCIController *)owner;
	struct EHCIQueueOutArgs *a = (struct EHCIQueueOutArgs *)args;
	if (!self->current(a->slot, a->generation)) {
		return kIOReturnNoDevice;
	}
	struct EHCIPipe *p = self->devices[a->slot].pipes[a->index];
	if (p == NULL || !p->queue || p->in) {
		return kIOReturnNotFound;
	}
	if (p->halted) {
		return kIOReturnNotResponding;
	}
	if (a->length > p->size) {
		return kIOReturnBadArgument;
	}
	uint32_t all = p->count == 32 ? 0xffffffffU : (1U << p->count) - 1;
	uint32_t idle = ~p->busy & all;
	if (idle == 0) {
		return kIOReturnNoResources;
	}
	uint32_t mps = p->ep.maxPacket & 0x7ff;
	uint32_t data = a->length == 0 ? 1 : (a->length + p->chunk - 1) / p->chunk;
	uint32_t k = data + ((a->length > 0 && a->length % mps == 0) ? 1 : 0);
	uint32_t used = (p->tail + p->ntds - p->head) % p->ntds;
	if (used + k >= p->ntds) {
		return kIOReturnNoResources;
	}
	uint32_t b = (uint32_t)__builtin_ctz(idle);
	uint32_t buffer = p->buffersPhys + b * p->size;
	bcopy(a->data, p->buffers + (size_t)b * p->size, a->length);
	uint32_t tokens[8];
	for (uint32_t i = 0; i < k; i++) {
		uint32_t slotTD = (p->tail + i) % p->ntds;
		volatile struct QTD *td = &p->tds[slotTD];
		uint32_t off = i < data ? i * p->chunk : a->length;
		uint32_t len = i < data ? (a->length - off < p->chunk ? a->length - off : p->chunk) : 0;
		td->token = 0;
		td_buffers(td, buffer + (off < p->size ? off : 0));
		tokens[i] = td_token(TD_PID_OUT, len, 0) & ~TD_TOGGLE;
		if (i + 1 < k) {
			tokens[i] &= ~TD_IOC;
		}
		p->owner[slotTD] = i + 1 == k ? (uint8_t)b : OWNER_NONE;
	}
	__builtin_arm_dmb(0xf);
	for (uint32_t i = k; i-- > 0;) {
		p->tds[(p->tail + i) % p->ntds].token = tokens[i];
		__builtin_arm_dmb(0xf);
	}
	p->tail = (p->tail + k) % p->ntds;
	p->lengths[b] = a->length;
	p->busy |= 1U << b;
	return kIOReturnSuccess;
}

IOReturn
EHCIController::queueOut(uint32_t slot, uint8_t endpointAddress, const void *data, uint32_t length,
    uint32_t generation)
{
	if (!current(slot, generation)) {
		return kIOReturnNoDevice;
	}
	if ((endpointAddress & 0x80) || (endpointAddress & 0xf) == 0) {
		return kIOReturnBadArgument;
	}
	struct EHCIQueueOutArgs a = { slot, generation, pipe_index(endpointAddress), data, length };
	return workLoop->runAction(&EHCIController::gatedQueueOut, this, &a);
}

/*
 * The queue taken off under the gate before anything else, so that its
 * action is not called once this returns, also for a device that has gone.
 * Once stop has begun, the schedule is left as it is and the queue's memory
 * kept, as on a controller that has stopped working.
 */
IOReturn
EHCIController::stopQueue(uint32_t slot, uint8_t endpointAddress, uint32_t generation)
{
	if ((endpointAddress & 0xf) == 0) {
		return current(slot, generation) ? kIOReturnBadArgument : kIOReturnNoDevice;
	}
	struct EHCIPipe *p = NULL;
	IOLockLock(lock);
	IOReturn ret = workLoop->runAction(&EHCIController::gatedQueueTake, this, (void *)(uintptr_t)slot,
	    (void *)(uintptr_t)pipe_index(endpointAddress), (void *)(uintptr_t)generation, &p);
	bool unlinked = ret == kIOReturnSuccess && p != NULL && !stopRequested() && unlinkQH(p->qh);
	bool live = current(slot, generation);
	IOLockUnlock(lock);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	if (p == NULL) {
		return live ? kIOReturnNotFound : kIOReturnNoDevice;
	}
	if (unlinked) {
		freePipe(p);
	}
	return !live ? kIOReturnNoDevice : unlinked ? kIOReturnSuccess : kIOReturnTimeout;
}

#pragma mark Interrupts

void
EHCIController::interruptOccurred(IOInterruptEventSource *src, int count)
{
	(void)src;
	(void)count;
	uint32_t sts = r32(op, OP_USBSTS);
	uint32_t ack = sts & (STS_INT | STS_ERR | STS_PCD | STS_HSE);
	if (ack == 0) {
		return;
	}
	if (sts & STS_HSE) {
		controllerDied("host system error", sts);
		__builtin_arm_dmb(0xf);
	}
	w32(op, OP_USBSTS, ack);
	if (sts & STS_PCD) {
		IOLockLock(portLock);
		portsChanged = true;
		IOLockWakeup(portLock, &portsChanged, true);
		IOLockUnlock(portLock);
	}
	if (sts & (STS_INT | STS_ERR)) {
		__builtin_arm_dmb(0xf);
		for (struct EHCIPipe *p = activeQueues; p != NULL; p = p->nextActive) {
			processQueue(p);
		}
		IOLockLock(waitLock);
		waitEvents++;
		IOLockWakeup(waitLock, &waitEvents, false);
		IOLockUnlock(waitLock);
	}
}

#pragma mark Enumeration

enum {
	HUB_PORT_CONNECTION = 1U << 0, HUB_PORT_ENABLE = 1U << 1, HUB_PORT_HIGH_SPEED = 1U << 10,
	HUB_C_PORT_CONNECTION = 1U << 0, HUB_C_PORT_RESET = 1U << 4,
	FEAT_PORT_ENABLE = 1, FEAT_PORT_RESET = 4, FEAT_PORT_POWER = 8, FEAT_C_PORT_CONNECTION = 16,
	FEAT_C_PORT_ENABLE = 17, FEAT_C_PORT_RESET = 20,
};

IOReturn
EHCIController::controlReadExact(uint32_t slot, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
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

/*
 * A new high-speed device at address 0: its address is its slot. The
 * first configuration is set without a look at its power budget. Each
 * descriptor is read in full, with its type and length checked, and the
 * configuration is used up to its first broken descriptor. 0 if the
 * device was not attached: its port is then disabled by the caller.
 */
uint32_t
EHCIController::attachDevice(uint8_t rootPort, uint8_t depth)
{
	uint32_t slot = 0;
	for (uint32_t s = 1; s < MAX_DEVICES; s++) {
		if (!devices[s].used) {
			slot = s;
			break;
		}
	}
	if (slot == 0) {
		IOLog("EHCI: no free device slot\n");
		return 0;
	}
	uint8_t dev[18];
	if (controlReadExact(0, 0x80, 6, 0x0100, 0, 8, dev) != kIOReturnSuccess || dev[0] < 18 || dev[1] != 1) {
		IOLog("EHCI: port %u depth %u: no device descriptor\n", rootPort, depth);
		return 0;
	}
	uint16_t mps0 = dev[7] == 8 || dev[7] == 16 || dev[7] == 32 || dev[7] == 64 ? dev[7] : 64;
	if (controlTransfer(0, 0x00, 5, (uint16_t)slot, 0, 0, NULL, NULL) != kIOReturnSuccess) {
		IOLog("EHCI: port %u depth %u: no address\n", rootPort, depth);
		return 0;
	}
	IOSleep(10);

	struct EHCIDevice *d = &devices[slot];
	IOLockLock(lock);
	bzero(d, sizeof(*d));
	if (++nextGeneration == 0) {
		nextGeneration = 1;
	}
	d->generation = nextGeneration;
	d->address = (uint8_t)slot;
	d->port = rootPort;
	d->depth = depth;
	d->ctl = (volatile struct QTD *)allocDMA(3 * sizeof(struct QTD), 64, &d->ctlPhys);
	d->setup = (uint8_t *)allocDMA(64, 64, &d->setupPhys);
	d->buf = (uint8_t *)allocDMA(4096, 4096, &d->bufPhys);
	d->ep0 = newQH(d->address, 0, mps0, true);
	bool ok = d->ctl != NULL && d->setup != NULL && d->buf != NULL && d->ep0 != NULL;
	if (ok) {
		linkQH(d->ep0);
		d->used = true;
	}
	IOLockUnlock(lock);
	if (!ok) {
		IOLog("EHCI: out of DMA memory\n");
		freeDMA((void *)(uintptr_t)d->ctl, d->ctlPhys, 3 * sizeof(struct QTD));
		freeDMA(d->setup, d->setupPhys, 64);
		freeDMA(d->buf, d->bufPhys, 4096);
		freeQH(d->ep0);
		bzero(d, sizeof(*d));
		return 0;
	}

	uint8_t cfgHead[9];
	uint16_t total = 0;
	if (controlReadExact(slot, 0x80, 6, 0x0100, 0, 18, dev) == kIOReturnSuccess && dev[0] >= 18 && dev[1] == 1 &&
	    controlReadExact(slot, 0x80, 6, 0x0200, 0, 9, cfgHead) == kIOReturnSuccess && cfgHead[0] >= 9 &&
	    cfgHead[1] == 2) {
		total = OSReadLittleInt16(cfgHead, 2);
	}
	if (total > 4096) {
		IOLog("EHCI: address %u: configuration of %u bytes is longer than 4096\n", slot, total);
		detachDevice(slot);
		return 0;
	}
	if (total < 9) {
		IOLog("EHCI: address %u: descriptors unreadable\n", slot);
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
	bool configured = false;
	if (length < 9) {
		IOLog("EHCI: address %u: configuration unreadable\n", slot);
	} else if (controlTransfer(slot, 0x00, 9, cfg[5], 0, 0, NULL, NULL) == kIOReturnSuccess) {
		configured = true;
		IOLog("EHCI: port %u depth %u: %04x:%04x, class %02x, high speed, address %u\n", rootPort, depth,
		    OSReadLittleInt16(dev, 8), OSReadLittleInt16(dev, 10), dev[4], slot);
		if (dev[4] == 9) {
			setupHub(slot);
		}
		publishInterfaces(slot, d->generation, rootPort, SPEED_HIGH, dev, cfg, length);
	} else {
		IOLog("EHCI: address %u: configuration not set\n", slot);
	}
	IOFreeData(cfg, total);
	if (!configured) {
		detachDevice(slot);
		return 0;
	}
	return slot;
}

void
EHCIController::detachDevice(uint32_t slot)
{
	if (slot == 0 || slot >= MAX_DEVICES || !devices[slot].used) {
		return;
	}
	struct EHCIDevice *d = &devices[slot];
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
	waitEvents++;
	IOLockWakeup(waitLock, &waitEvents, false);
	IOLockUnlock(waitLock);

	struct EHCIPipe *pipes[32];
	IOLockLock(lock);
	workLoop->runAction(&EHCIController::gatedDeviceTake, this, (void *)(uintptr_t)slot, pipes);
	bool clean = pauseAsync();
	if (clean) {
		for (uint32_t i = 0; i < 32; i++) {
			if (pipes[i] != NULL) {
				unlinkEntry(pipes[i]->qh);
			}
		}
		unlinkEntry(d->ep0);
		clean = resumeAsync();
	}
	IOLockUnlock(lock);
	terminateInterfaces(slot, d->generation);
	for (uint32_t i = 0; i < 32 && clean; i++) {
		freePipe(pipes[i]);
	}
	if (clean) {
		freeQH(d->ep0);
		freeDMA((void *)(uintptr_t)d->ctl, d->ctlPhys, 3 * sizeof(struct QTD));
		freeDMA(d->setup, d->setupPhys, 64);
		freeDMA(d->buf, d->bufPhys, 4096);
	}
	IOLockLock(lock);
	bzero(d, sizeof(*d));
	IOLockUnlock(lock);
	if (holdHub == slot) {
		holdHub = 0;
		IOLog("EHCI: address %u: gone; enumeration resumes\n", slot);
	}
}

void
EHCIController::setupHub(uint32_t slot)
{
	struct EHCIDevice *d = &devices[slot];
	uint8_t desc[9];
	if (controlReadExact(slot, 0xa0, 6, 0x2900, 0, sizeof(desc), desc) != kIOReturnSuccess || desc[0] < sizeof(desc) ||
	    desc[1] != 0x29) {
		IOLog("EHCI: address %u: no hub descriptor\n", slot);
		return;
	}
	uint8_t ports = desc[2] < HUB_PORTS ? desc[2] : HUB_PORTS;
	uint32_t powerDelay = desc[5] * 2U;
	for (uint8_t p = 1; p <= ports; p++) {
		controlTransfer(slot, 0x23, 3, FEAT_PORT_POWER, p, 0, NULL, NULL);
	}
	IOSleep(powerDelay > 100 ? powerDelay : 100);
	d->hubPorts = ports;
	d->isHub = true;
	IOLog("EHCI: address %u: hub, %u ports, polled\n", slot, ports);
}

/*
 * A hub port disabled, as a status read after the request shows, whatever
 * the request returned; three tries.
 */
bool
EHCIController::disableHubPort(uint32_t slot, uint8_t port)
{
	for (int i = 0; i < 3; i++) {
		uint16_t status, change;
		controlTransfer(slot, 0x23, 1, FEAT_PORT_ENABLE, port, 0, NULL, NULL);
		if (hubPortStatus(slot, port, &status, &change) && !(status & HUB_PORT_ENABLE)) {
			return true;
		}
	}
	return false;
}

/* A hub port's status and change bits, from a GET_STATUS that returned all four bytes. */
bool
EHCIController::hubPortStatus(uint32_t slot, uint8_t port, uint16_t *status, uint16_t *change)
{
	uint8_t st[4];
	if (controlReadExact(slot, 0xa3, 0, 0, port, 4, st) != kIOReturnSuccess) {
		return false;
	}
	*status = OSReadLittleInt16(st, 0);
	*change = OSReadLittleInt16(st, 2);
	return true;
}

void
EHCIController::scanHub(uint32_t slot)
{
	struct EHCIDevice *d = &devices[slot];
	for (uint8_t p = 1; p <= d->hubPorts; p++) {
		uint16_t status, change;
		if (!hubPortStatus(slot, p, &status, &change)) {
			continue;
		}
		if (change & HUB_C_PORT_CONNECTION) {
			controlTransfer(slot, 0x23, 1, FEAT_C_PORT_CONNECTION, p, 0, NULL, NULL);
		}
		if (d->hubChild[p] != 0 && (!(status & HUB_PORT_CONNECTION) || (change & HUB_C_PORT_CONNECTION))) {
			uint8_t child = d->hubChild[p];
			d->hubChild[p] = 0;
			if (child != 0xff) {
				IOLog("EHCI: address %u port %u: disconnected\n", slot, p);
				detachDevice(child);
			}
		}
		if (!(status & HUB_PORT_CONNECTION) || d->hubChild[p] != 0 || d->depth >= 5 || holdHub != 0 ||
		    holdPort != 0) {
			continue;
		}
		IOSleep(100);
		controlTransfer(slot, 0x23, 3, FEAT_PORT_RESET, p, 0, NULL, NULL);
		for (int i = 0; i < 50; i++) {
			IOSleep(10);
			if (hubPortStatus(slot, p, &status, &change) && (change & HUB_C_PORT_RESET)) {
				break;
			}
		}
		controlTransfer(slot, 0x23, 1, FEAT_C_PORT_RESET, p, 0, NULL, NULL);
		controlTransfer(slot, 0x23, 1, FEAT_C_PORT_ENABLE, p, 0, NULL, NULL);
		IOSleep(10);
		uint32_t child = 0;
		if (!hubPortStatus(slot, p, &status, &change) || !(status & HUB_PORT_ENABLE)) {
			IOLog("EHCI: address %u port %u: not enabled after reset\n", slot, p);
		} else if (!(status & HUB_PORT_HIGH_SPEED)) {
			IOLog("EHCI: address %u port %u: a full- or low-speed device; not supported\n", slot, p);
		} else {
			child = attachDevice(d->port, (uint8_t)(d->depth + 1));
		}
		if (child == 0 && !disableHubPort(slot, p)) {
			IOLog("EHCI: address %u port %u: still enabled; nothing is enumerated until the hub goes\n", slot, p);
			holdHub = slot;
		}
		d->hubChild[p] = child != 0 ? (uint8_t)child : 0xff;
	}
}

/*
 * A root port's device, 100 ms after it connected, and after its reset: a
 * high-speed one stays, a full- or low-speed one goes to the companion
 * controller. The port's change bits are cleared before the reset, so a
 * later one is a new change.
 */
void
EHCIController::attachPort(uint32_t port)
{
	uintptr_t sc = op + OP_PORTSC + 4 * (port - 1);
	if (holdHub != 0 || holdPort != 0) {
		return;
	}
	IOSleep(100);
	uint32_t ps = r32(sc, 0);
	if (!(ps & PORT_CCS)) {
		return;
	}
	if (ps & (PORT_CSC | PORT_PEC)) {
		w32(sc, 0, (ps & ~PORT_W1C) | (ps & (PORT_CSC | PORT_PEC)));
	}
	if (((ps >> PORT_LINE_SHIFT) & 3) == PORT_LINE_K) {
		IOLog("EHCI: port %u: a low-speed device; left to the companion\n", port);
		w32(sc, 0, (ps & ~PORT_W1C) | PORT_OWNER);
		portDevice[port] = 0xffffffff;
		return;
	}
	w32(sc, 0, (ps & ~(PORT_W1C | PORT_PE)) | PORT_RESET);
	IOSleep(50);
	w32(sc, 0, r32(sc, 0) & ~(PORT_W1C | PORT_RESET));
	for (int i = 0; i < 20 && (r32(sc, 0) & PORT_RESET); i++) {
		IOSleep(1);
	}
	IOSleep(10);
	ps = r32(sc, 0);
	if (!(ps & PORT_PE)) {
		IOLog("EHCI: port %u: a full-speed device; left to the companion\n", port);
		w32(sc, 0, (ps & ~PORT_W1C) | PORT_OWNER);
		portDevice[port] = 0xffffffff;
		return;
	}
	uint32_t slot = attachDevice((uint8_t)port, 0);
	if (slot == 0) {
		w32(sc, 0, r32(sc, 0) & ~(PORT_W1C | PORT_PE));
		for (int i = 0; i < 10 && (r32(sc, 0) & PORT_PE); i++) {
			IOSleep(1);
		}
		if (r32(sc, 0) & PORT_PE) {
			IOLog("EHCI: port %u: still enabled; nothing is enumerated until it is disconnected\n", port);
			holdPort = port;
		}
	}
	portDevice[port] = slot != 0 ? slot : 0xffffffff;
}

/*
 * The root ports: a disconnect, or a port the controller has disabled with
 * its device still connected, detaches the device, and a connected port
 * with none attaches one.
 */
void
EHCIController::scanPorts(void)
{
	for (uint32_t port = 1; port <= nPorts; port++) {
		uintptr_t sc = op + OP_PORTSC + 4 * (port - 1);
		uint32_t ps = r32(sc, 0);
		if (ps & (PORT_CSC | PORT_PEC)) {
			w32(sc, 0, (ps & ~PORT_W1C) | (ps & (PORT_CSC | PORT_PEC)));
		}
		bool attached = portDevice[port] != 0 && portDevice[port] != 0xffffffff;
		if (portDevice[port] != 0 && (!(ps & PORT_CCS) || (ps & PORT_CSC) || (attached && !(ps & PORT_PE)))) {
			if (attached) {
				IOLog("EHCI: port %u: %s\n", port, (ps & (PORT_CCS | PORT_CSC)) == PORT_CCS ? "disabled" : "disconnected");
				detachDevice(portDevice[port]);
			}
			portDevice[port] = 0;
			if (holdPort == port) {
				holdPort = 0;
				IOLog("EHCI: port %u: disconnected; enumeration resumes\n", port);
			}
		}
		if ((ps & PORT_CCS) && !(ps & PORT_OWNER) && portDevice[port] == 0) {
			attachPort(port);
		}
	}
	for (uint32_t slot = 1; slot < MAX_DEVICES; slot++) {
		if (devices[slot].used && !devices[slot].gone && devices[slot].isHub) {
			scanHub(slot);
		}
	}
}

#pragma mark Controller

bool
EHCIController::platformStart(IOService *provider)
{
	(void)provider;
	return true;
}

/* True once stop has begun: platformStart then gives up. */
bool
EHCIController::stopRequested(void)
{
	IOLockLock(portLock);
	bool s = stopping;
	IOLockUnlock(portLock);
	return s;
}

/*
 * Run/Stop cleared, and true once the controller has halted, which it must
 * do within 16 micro-frames (EHCI 1.0 section 2.3.1): waited for up to
 * 20 ms.
 */
bool
EHCIController::haltController(void)
{
	w32(op, OP_USBCMD, r32(op, OP_USBCMD) & ~CMD_RS);
	for (int i = 0; i < 200; i++) {
		if (r32(op, OP_USBSTS) & STS_HCH) {
			return true;
		}
		IODelay(100);
	}
	return (r32(op, OP_USBSTS) & STS_HCH) != 0;
}

bool
EHCIController::resetController(void)
{
	haltController();
	w32(op, OP_USBCMD, CMD_HCRESET);
	for (int i = 0; i < 250; i++) {
		if (!(r32(op, OP_USBCMD) & CMD_HCRESET)) {
			return true;
		}
		IOSleep(1);
	}
	return false;
}

bool
EHCIController::startController(void)
{
	uint32_t capWord = r32(cap, 0);
	uint32_t capLength = capWord & 0xff, version = capWord >> 16;
	if (capWord == 0xffffffff || capLength < 0x10 || version < 0x0095 || version > 0x0110) {
		IOLog("EHCI: no EHCI at 0x%llx (capability register 0x%08x)\n", regsMap->getPhysicalAddress(), capWord);
		return false;
	}
	op = cap + capLength;
	uint32_t hcs = r32(cap, CAP_HCSPARAMS), hcc = r32(cap, CAP_HCCPARAMS);
	nPorts = hcs & 0xf;
	if (nPorts >= sizeof(portDevice) / sizeof(portDevice[0])) {
		nPorts = sizeof(portDevice) / sizeof(portDevice[0]) - 1;
	}
	portPower = (hcs & HCS_PPC) != 0;
	if (!resetController()) {
		IOLog("EHCI: controller reset timed out\n");
		return false;
	}
	if (hcc & HCC_64BIT) {
		w32(op, OP_CTRLDSSEGMENT, 0);
	}
	w32(op, OP_USBINTR, 0);

	poolMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, POOL_SIZE, 0x7ffff000ULL);
	if (poolMem == NULL) {
		IOLog("EHCI: no DMA memory\n");
		return false;
	}
	poolMem->performOperation(kIOMemoryIncoherentIOFlush, 0, POOL_SIZE);
	poolMap = poolMem->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
	uint64_t bus = 0;
	if (poolMap == NULL || !RPiDMAAddress(nub, poolMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &bus) ||
	    bus + POOL_SIZE > (1ULL << 32)) {
		IOLog("EHCI: the controller cannot reach its DMA memory\n");
		return false;
	}
	pool = (uint8_t *)poolMap->getVirtualAddress();
	poolPhys = (uint32_t)bus;
	poolUsed = 0;

	asyncHead = IOMallocType(struct EHCIQH);
	asyncHead->hw = (volatile struct QHW *)allocDMA(sizeof(struct QHW), QH_ALIGN, &asyncHead->phys);
	struct EHCIDevice *d0 = &devices[0];
	d0->ctl = (volatile struct QTD *)allocDMA(3 * sizeof(struct QTD), 64, &d0->ctlPhys);
	d0->setup = (uint8_t *)allocDMA(64, 64, &d0->setupPhys);
	d0->buf = (uint8_t *)allocDMA(4096, 4096, &d0->bufPhys);
	d0->ep0 = newQH(0, 0, 64, true);
	if (asyncHead->hw == NULL || d0->ctl == NULL || d0->setup == NULL || d0->buf == NULL || d0->ep0 == NULL) {
		return false;
	}
	asyncHead->hw->link = asyncHead->phys | LINK_QH;
	asyncHead->hw->endpoint = QH_HEAD | QH_EPS_HIGH | 64U << 16;
	asyncHead->hw->caps = QH_MULT1;
	asyncHead->hw->next = LINK_T;
	asyncHead->hw->alt = LINK_T;
	asyncHead->hw->token = TD_HALTED;
	asyncHead->linked = true;
	__builtin_arm_dmb(0xf);
	linkQH(d0->ep0);
	d0->used = true;
	w32(op, OP_ASYNCLISTADDR, asyncHead->phys);

	workLoop = IOWorkLoop::workLoop();
	intSource = IOInterruptEventSource::interruptEventSource(this,
	    OSMemberFunctionCast(IOInterruptEventAction, this, &EHCIController::interruptOccurred), nub, 0);
	if (workLoop == NULL || intSource == NULL || workLoop->addEventSource(intSource) != kIOReturnSuccess) {
		IOLog("EHCI: no interrupt\n");
		return false;
	}
	intSource->enable();

	IOLockLock(lock);
	IOLockLock(portLock);
	bool cancelled = stopping;
	if (!cancelled) {
		w32(op, OP_USBCMD, 1U << CMD_ITC_SHIFT | CMD_ASE | CMD_RS);
	}
	IOLockUnlock(portLock);
	IOLockUnlock(lock);
	if (cancelled) {
		return false;
	}
	for (int i = 0; i < 100 && (r32(op, OP_USBSTS) & STS_HCH); i++) {
		IODelay(100);
	}
	for (int i = 0; i < 100 && !(r32(op, OP_USBSTS) & STS_ASS); i++) {
		IODelay(100);
	}
	w32(op, OP_CONFIGFLAG, 1);
	IOSleep(5);
	for (uint32_t port = 1; port <= nPorts && portPower; port++) {
		uintptr_t sc = op + OP_PORTSC + 4 * (port - 1);
		w32(sc, 0, (r32(sc, 0) & ~PORT_W1C) | PORT_PP);
	}
	IOSleep(20);
	w32(op, OP_USBINTR, STS_INT | STS_ERR | STS_PCD | STS_HSE);
	IOLog("EHCI: controller at 0x%llx, EHCI %x.%02x, %u ports, USBSTS 0x%08x\n", regsMap->getPhysicalAddress(),
	    version >> 8, version & 0xff, nPorts, r32(op, OP_USBSTS));
	return true;
}

void
EHCIController::bringUp(void *arg, wait_result_t wr)
{
	(void)wr;
	EHCIController *self = (EHCIController *)arg;
	if (self->platformStart(self->nub) && !self->stopRequested() && self->startController()) {
		IOSleep(50);
		for (;;) {
			self->scanPorts();
			IOLockLock(self->portLock);
			if (!self->portsChanged && !self->stopping) {
				uint64_t deadline;
				clock_interval_to_deadline(250, kMillisecondScale, &deadline);
				IOLockSleepDeadline(self->portLock, &self->portsChanged, deadline, THREAD_UNINT);
			}
			self->portsChanged = false;
			bool stop = self->stopping;
			IOLockUnlock(self->portLock);
			if (stop) {
				break;
			}
		}
	} else if (!self->stopRequested()) {
		IOLog("EHCI: not started\n");
	}
	IOLockLock(self->portLock);
	self->threadRunning = false;
	IOLockWakeup(self->portLock, &self->threadRunning, false);
	IOLockUnlock(self->portLock);
	self->release();
}

bool
EHCIController::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	nub = provider;
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		IOLog("EHCI: cannot map the registers\n");
		return false;
	}
	cap = (uintptr_t)regsMap->getVirtualAddress();
	lock = IOLockAlloc();
	waitLock = IOLockAlloc();
	portLock = IOLockAlloc();
	dmaLock = IOLockAlloc();
	devices = IONewZero(struct EHCIDevice, MAX_DEVICES);
	if (lock == NULL || waitLock == NULL || portLock == NULL || dmaLock == NULL || devices == NULL) {
		return false;
	}
	registerService();
	thread_t thread;
	retain();
	threadRunning = true;
	if (kernel_thread_start(&EHCIController::bringUp, this, &thread) != KERN_SUCCESS) {
		threadRunning = false;
		release();
		return false;
	}
	thread_deallocate(thread);
	return true;
}

/*
 * No more requests, the controller's thread cancelled and waited for, then
 * the controller halted. Its DMA memory is released in free only if it
 * halted here, had not stopped working before, and is still halted then.
 */
void
EHCIController::stop(IOService *provider)
{
	IOLockLock(portLock);
	stopping = true;
	IOLockWakeup(portLock, &portsChanged, false);
	while (threadRunning) {
		IOLockSleep(portLock, &threadRunning, THREAD_UNINT);
	}
	IOLockUnlock(portLock);
	if (op != 0) {
		IOLockLock(lock);
		bool wasDead = dead;
		halted = haltController() && !wasDead;
		IOLockUnlock(lock);
		if (!halted && poolMem != NULL) {
			IOLog("EHCI: the controller %s; its DMA memory is kept\n", wasDead ? "had stopped working" :
			    "did not halt");
		}
	}
	if (intSource != NULL) {
		intSource->disable();
		workLoop->removeEventSource(intSource);
	}
	super::stop(provider);
}

void
EHCIController::free(void)
{
	OSSafeReleaseNULL(intSource);
	OSSafeReleaseNULL(workLoop);
	if (halted && (r32(op, OP_USBSTS) & STS_HCH)) {
		OSSafeReleaseNULL(poolMap);
		OSSafeReleaseNULL(poolMem);
	}
	OSSafeReleaseNULL(regsMap);
	if (devices != NULL) {
		IODelete(devices, struct EHCIDevice, MAX_DEVICES);
		devices = NULL;
	}
	super::free();
}
