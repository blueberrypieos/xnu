/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * VirtioMMIONet: a virtio network device on the virtio-mmio transport
 * (virtio,mmio; version 2, the "modern" interface) as an
 * IOEthernetController. It serves QEMU's virt machine, where the network
 * stack is tested before the Raspberry Pi's own Ethernet; run QEMU with
 * -global virtio-mmio.force-legacy=false.
 *
 * One receive and one transmit split virtqueue. Packets are copied through
 * fixed 2 KB buffers (QEMU's DMA is cache coherent), with the 12-byte
 * virtio_net_hdr in front and no offloads.
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/OSByteOrder.h>

/*
 * The family is built with __PRIVATE_SPI__ (as its kext target is), which
 * changes its classes' vtables: drivers linked with it must see the same.
 */
#define __PRIVATE_SPI__ 1

/* IONetworkingFamily's headers (and kpi_mbuf.h) predate xnu's own warnings. */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-html"
#include <sys/kpi_mbuf.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IONetworkMedium.h>
#pragma clang diagnostic pop

/* virtio-mmio registers (Virtio 1.1, 4.2.2). */
enum {
	VIRTIO_MMIO_MAGIC           = 0x000,
	VIRTIO_MMIO_VERSION         = 0x004,
	VIRTIO_MMIO_DEVICE_ID       = 0x008,
	VIRTIO_MMIO_DEVICE_FEATURES = 0x010,
	VIRTIO_MMIO_DEVICE_FEATURES_SEL = 0x014,
	VIRTIO_MMIO_DRIVER_FEATURES = 0x020,
	VIRTIO_MMIO_DRIVER_FEATURES_SEL = 0x024,
	VIRTIO_MMIO_QUEUE_SEL       = 0x030,
	VIRTIO_MMIO_QUEUE_NUM_MAX   = 0x034,
	VIRTIO_MMIO_QUEUE_NUM       = 0x038,
	VIRTIO_MMIO_QUEUE_READY     = 0x044,
	VIRTIO_MMIO_QUEUE_NOTIFY    = 0x050,
	VIRTIO_MMIO_INTERRUPT_STATUS = 0x060,
	VIRTIO_MMIO_INTERRUPT_ACK   = 0x064,
	VIRTIO_MMIO_STATUS          = 0x070,
	VIRTIO_MMIO_QUEUE_DESC_LOW  = 0x080,
	VIRTIO_MMIO_QUEUE_DESC_HIGH = 0x084,
	VIRTIO_MMIO_QUEUE_DRIVER_LOW = 0x090,
	VIRTIO_MMIO_QUEUE_DRIVER_HIGH = 0x094,
	VIRTIO_MMIO_QUEUE_DEVICE_LOW = 0x0A0,
	VIRTIO_MMIO_QUEUE_DEVICE_HIGH = 0x0A4,
	VIRTIO_MMIO_CONFIG          = 0x100,
};

#define VIRTIO_MAGIC            0x74726976      /* "virt" */
#define VIRTIO_ID_NET           1

enum {
	STATUS_ACKNOWLEDGE = 1,
	STATUS_DRIVER      = 2,
	STATUS_DRIVER_OK   = 4,
	STATUS_FEATURES_OK = 8,
	STATUS_FAILED      = 128,
};

#define VIRTIO_NET_F_MAC        5               /* feature word 0 */
#define VIRTIO_F_VERSION_1      0               /* feature word 1 (bit 32) */

#define VRING_DESC_F_WRITE      2

#define RX_QUEUE        0
#define TX_QUEUE        1
#define QUEUE_SIZE      128
#define BUF_SIZE        2048
#define NET_HDR_SIZE    12              /* struct virtio_net_hdr_v1 */
#define MAX_FRAME       1514

struct vring_desc {
	uint64_t addr;
	uint32_t len;
	uint16_t flags;
	uint16_t next;
};

struct vring_avail {
	uint16_t flags;
	uint16_t idx;
	uint16_t ring[QUEUE_SIZE];
	uint16_t used_event;
};

struct vring_used_elem {
	uint32_t id;
	uint32_t len;
};

struct vring_used {
	uint16_t flags;
	uint16_t idx;
	struct vring_used_elem ring[QUEUE_SIZE];
	uint16_t avail_event;
};

/* One virtqueue and its packet buffers, each physically contiguous. */
struct VirtQueue {
	IOBufferMemoryDescriptor *ringMem;
	IOBufferMemoryDescriptor *bufMem;
	struct vring_desc *desc;
	struct vring_avail *avail;
	struct vring_used *used;
	uint8_t *bufs;
	uint64_t bufsPhys;
	uint16_t lastUsed;
	uint16_t freeCount;
	uint16_t freeList[QUEUE_SIZE];
};

class VirtioMMIONet : public IOEthernetController
{
	OSDeclareDefaultStructors(VirtioMMIONet);

	IOMemoryMap *regsMap;
	volatile uint32_t *regs;
	IOWorkLoop *workLoop;
	IOInterruptEventSource *intSource;
	IOEthernetInterface *netif;
	IONetworkStats *netStats;              /* the interface's counters */
	IOGatedOutputQueue *txQueue;
	IONetworkMedium *medium;
	struct VirtQueue rxq;
	struct VirtQueue txq;
	IOEthernetAddress mac;
	bool enabled;

	uint32_t rd(uint32_t off) { return OSSwapLittleToHostInt32(regs[off / 4]); }
	void wr(uint32_t off, uint32_t v) { regs[off / 4] = OSSwapHostToLittleInt32(v); }

	bool setupQueue(uint32_t index, struct VirtQueue *q);
	void freeQueue(struct VirtQueue *q);
	void postRxBuffer(uint16_t id);
	void interruptOccurred(IOInterruptEventSource *src, int count);
	void receivePackets(void);
	bool reclaimTx(void);

public:
	virtual IOService *probe(IOService *provider, SInt32 *score) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	virtual bool createWorkLoop(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;
	virtual IOOutputQueue *createOutputQueue(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn enable(IONetworkInterface *interface) APPLE_KEXT_OVERRIDE;
	virtual IOReturn disable(IONetworkInterface *interface) APPLE_KEXT_OVERRIDE;
	virtual UInt32 outputPacket(mbuf_t m, void *param) APPLE_KEXT_OVERRIDE;
	virtual IOReturn getHardwareAddress(IOEthernetAddress *addr) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setPromiscuousMode(bool active) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setMulticastMode(bool active) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setMulticastList(IOEthernetAddress *addrs, UInt32 count) APPLE_KEXT_OVERRIDE;
};

#define super IOEthernetController
OSDefineMetaClassAndStructors(VirtioMMIONet, IOEthernetController);

#pragma mark Transport

/* Only the transports that carry a network device. */
IOService *
VirtioMMIONet::probe(IOService *provider, SInt32 *score)
{
	IOMemoryMap *map = provider->mapDeviceMemoryWithIndex(0);
	if (map == NULL) {
		return NULL;
	}
	volatile uint32_t *r = (volatile uint32_t *)map->getVirtualAddress();
	bool net = OSSwapLittleToHostInt32(r[VIRTIO_MMIO_MAGIC / 4]) == VIRTIO_MAGIC &&
	    OSSwapLittleToHostInt32(r[VIRTIO_MMIO_DEVICE_ID / 4]) == VIRTIO_ID_NET;
	uint32_t version = OSSwapLittleToHostInt32(r[VIRTIO_MMIO_VERSION / 4]);
	map->release();
	if (!net) {
		return NULL;
	}
	if (version != 2) {
		IOLog("VirtioMMIONet: legacy virtio-mmio (version %u) is not supported; "
		    "run QEMU with -global virtio-mmio.force-legacy=false\n", version);
		return NULL;
	}
	return super::probe(provider, score);
}

bool
VirtioMMIONet::setupQueue(uint32_t index, struct VirtQueue *q)
{
	wr(VIRTIO_MMIO_QUEUE_SEL, index);
	if (rd(VIRTIO_MMIO_QUEUE_READY) != 0 || rd(VIRTIO_MMIO_QUEUE_NUM_MAX) < QUEUE_SIZE) {
		return false;
	}

	/* Descriptors, then the available ring, then (4-aligned) the used ring. */
	size_t descSize = sizeof(struct vring_desc) * QUEUE_SIZE;
	size_t usedOff = (descSize + sizeof(struct vring_avail) + 3) & ~(size_t)3;
	size_t ringSize = usedOff + sizeof(struct vring_used);
	q->ringMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, ringSize, ~(mach_vm_address_t)PAGE_MASK);
	q->bufMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, QUEUE_SIZE * BUF_SIZE,
	    ~(mach_vm_address_t)PAGE_MASK);
	if (q->ringMem == NULL || q->bufMem == NULL) {
		return false;
	}
	uintptr_t ring = (uintptr_t)q->ringMem->getBytesNoCopy();
	bzero((void *)ring, ringSize);
	uint64_t ringPhys = q->ringMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	q->desc = (struct vring_desc *)ring;
	q->avail = (struct vring_avail *)(ring + descSize);
	q->used = (struct vring_used *)(ring + usedOff);
	q->bufs = (uint8_t *)q->bufMem->getBytesNoCopy();
	q->bufsPhys = q->bufMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	q->lastUsed = 0;
	q->freeCount = 0;
	for (uint16_t i = 0; i < QUEUE_SIZE; i++) {
		q->freeList[q->freeCount++] = (uint16_t)(QUEUE_SIZE - 1 - i);
	}

	wr(VIRTIO_MMIO_QUEUE_NUM, QUEUE_SIZE);
	uint64_t availPhys = ringPhys + descSize, usedPhys = ringPhys + usedOff;
	wr(VIRTIO_MMIO_QUEUE_DESC_LOW, (uint32_t)ringPhys);
	wr(VIRTIO_MMIO_QUEUE_DESC_HIGH, (uint32_t)(ringPhys >> 32));
	wr(VIRTIO_MMIO_QUEUE_DRIVER_LOW, (uint32_t)availPhys);
	wr(VIRTIO_MMIO_QUEUE_DRIVER_HIGH, (uint32_t)(availPhys >> 32));
	wr(VIRTIO_MMIO_QUEUE_DEVICE_LOW, (uint32_t)usedPhys);
	wr(VIRTIO_MMIO_QUEUE_DEVICE_HIGH, (uint32_t)(usedPhys >> 32));
	wr(VIRTIO_MMIO_QUEUE_READY, 1);
	return true;
}

void
VirtioMMIONet::freeQueue(struct VirtQueue *q)
{
	OSSafeReleaseNULL(q->ringMem);
	OSSafeReleaseNULL(q->bufMem);
}

/* Hand receive buffer id (header + frame) to the device. */
void
VirtioMMIONet::postRxBuffer(uint16_t id)
{
	struct vring_desc *d = &rxq.desc[id];
	d->addr = OSSwapHostToLittleInt64(rxq.bufsPhys + (uint64_t)id * BUF_SIZE);
	d->len = OSSwapHostToLittleInt32(BUF_SIZE);
	d->flags = OSSwapHostToLittleInt16(VRING_DESC_F_WRITE);
	d->next = 0;
	uint16_t idx = OSSwapLittleToHostInt16(rxq.avail->idx);
	rxq.avail->ring[idx % QUEUE_SIZE] = OSSwapHostToLittleInt16(id);
	OSMemoryBarrier();
	rxq.avail->idx = OSSwapHostToLittleInt16((uint16_t)(idx + 1));
}

#pragma mark IOService

bool
VirtioMMIONet::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		return false;
	}
	regs = (volatile uint32_t *)regsMap->getVirtualAddress();

	/* Reset, then negotiate VIRTIO_F_VERSION_1 and the MAC address. */
	wr(VIRTIO_MMIO_STATUS, 0);
	while (rd(VIRTIO_MMIO_STATUS) != 0) {
	}
	wr(VIRTIO_MMIO_STATUS, STATUS_ACKNOWLEDGE);
	wr(VIRTIO_MMIO_STATUS, STATUS_ACKNOWLEDGE | STATUS_DRIVER);
	wr(VIRTIO_MMIO_DEVICE_FEATURES_SEL, 0);
	uint32_t features0 = rd(VIRTIO_MMIO_DEVICE_FEATURES);
	wr(VIRTIO_MMIO_DEVICE_FEATURES_SEL, 1);
	uint32_t features1 = rd(VIRTIO_MMIO_DEVICE_FEATURES);
	if (!(features1 & (1U << VIRTIO_F_VERSION_1))) {
		IOLog("VirtioMMIONet: device does not offer VIRTIO_F_VERSION_1\n");
		wr(VIRTIO_MMIO_STATUS, STATUS_FAILED);
		return false;
	}
	uint32_t want0 = features0 & (1U << VIRTIO_NET_F_MAC);
	wr(VIRTIO_MMIO_DRIVER_FEATURES_SEL, 0);
	wr(VIRTIO_MMIO_DRIVER_FEATURES, want0);
	wr(VIRTIO_MMIO_DRIVER_FEATURES_SEL, 1);
	wr(VIRTIO_MMIO_DRIVER_FEATURES, 1U << VIRTIO_F_VERSION_1);
	wr(VIRTIO_MMIO_STATUS, STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK);
	if (!(rd(VIRTIO_MMIO_STATUS) & STATUS_FEATURES_OK)) {
		IOLog("VirtioMMIONet: features not accepted\n");
		wr(VIRTIO_MMIO_STATUS, STATUS_FAILED);
		return false;
	}

	if (want0) {
		volatile uint8_t *config = (volatile uint8_t *)regs + VIRTIO_MMIO_CONFIG;
		for (int i = 0; i < 6; i++) {
			mac.bytes[i] = config[i];
		}
	} else {
		/* A locally administered address of our own. */
		static const uint8_t fallback[6] = { 0x02, 0x78, 0x6e, 0x70, 0x69, 0x01 };
		bcopy(fallback, mac.bytes, sizeof(fallback));
	}

	if (!setupQueue(RX_QUEUE, &rxq) || !setupQueue(TX_QUEUE, &txq)) {
		IOLog("VirtioMMIONet: cannot set up the virtqueues\n");
		wr(VIRTIO_MMIO_STATUS, STATUS_FAILED);
		return false;
	}
	/* Fill the receive queue: every buffer belongs to the device. */
	rxq.freeCount = 0;
	for (uint16_t id = 0; id < QUEUE_SIZE; id++) {
		postRxBuffer(id);
	}
	wr(VIRTIO_MMIO_STATUS, STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK | STATUS_DRIVER_OK);
	wr(VIRTIO_MMIO_QUEUE_NOTIFY, RX_QUEUE);

	intSource = IOInterruptEventSource::interruptEventSource(this,
	    OSMemberFunctionCast(IOInterruptEventAction, this, &VirtioMMIONet::interruptOccurred),
	    provider, 0);
	if (intSource == NULL || workLoop->addEventSource(intSource) != kIOReturnSuccess) {
		IOLog("VirtioMMIONet: no interrupt\n");
		return false;
	}

	OSDictionary *media = OSDictionary::withCapacity(1);
	medium = IONetworkMedium::medium(kIOMediumEthernetAuto, 1000 * 1000000ULL);
	if (media == NULL || medium == NULL || !IONetworkMedium::addMedium(media, medium) ||
	    !publishMediumDictionary(media)) {
		OSSafeReleaseNULL(media);
		return false;
	}
	media->release();
	setSelectedMedium(medium);

	if (!attachInterface((IONetworkInterface **)&netif, false)) {
		return false;
	}
	IOLog("VirtioMMIONet: %02x:%02x:%02x:%02x:%02x:%02x\n", mac.bytes[0], mac.bytes[1],
	    mac.bytes[2], mac.bytes[3], mac.bytes[4], mac.bytes[5]);
	/* Output packets and errors are counted here (IONetworkInterface counts the rest). */
	IONetworkData *stats = netif->getParameter(kIONetworkStatsKey);
	netStats = stats != NULL ? (IONetworkStats *)stats->getBuffer() : NULL;
	netif->registerService();
	return true;
}

void
VirtioMMIONet::free(void)
{
	if (regs != NULL) {
		wr(VIRTIO_MMIO_STATUS, 0);
	}
	if (intSource != NULL && workLoop != NULL) {
		workLoop->removeEventSource(intSource);
	}
	OSSafeReleaseNULL(intSource);
	OSSafeReleaseNULL(netif);
	OSSafeReleaseNULL(txQueue);
	OSSafeReleaseNULL(medium);
	OSSafeReleaseNULL(workLoop);
	freeQueue(&rxq);
	freeQueue(&txq);
	OSSafeReleaseNULL(regsMap);
	super::free();
}

#pragma mark IONetworkController

bool
VirtioMMIONet::createWorkLoop(void)
{
	workLoop = IOWorkLoop::workLoop();
	return workLoop != NULL;
}

IOWorkLoop *
VirtioMMIONet::getWorkLoop(void) const
{
	return workLoop;
}

IOOutputQueue *
VirtioMMIONet::createOutputQueue(void)
{
	txQueue = IOGatedOutputQueue::withTarget(this, getWorkLoop());
	if (txQueue != NULL) {
		txQueue->retain();
	}
	return txQueue;
}

IOReturn
VirtioMMIONet::enable(IONetworkInterface *interface)
{
	(void)interface;
	if (enabled) {
		return kIOReturnSuccess;
	}
	intSource->enable();
	txQueue->setCapacity(QUEUE_SIZE);
	txQueue->start();
	enabled = true;
	setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, medium, 1000 * 1000000ULL);
	return kIOReturnSuccess;
}

IOReturn
VirtioMMIONet::disable(IONetworkInterface *interface)
{
	(void)interface;
	if (!enabled) {
		return kIOReturnSuccess;
	}
	txQueue->stop();
	txQueue->flush();
	intSource->disable();
	enabled = false;
	setLinkStatus(kIONetworkLinkValid);
	return kIOReturnSuccess;
}

/* Called on the work loop (IOGatedOutputQueue). */
UInt32
VirtioMMIONet::outputPacket(mbuf_t m, void *param)
{
	(void)param;
	size_t len = mbuf_pkthdr_len(m);
	if (len > MAX_FRAME) {
		freePacket(m);
		if (netStats != NULL) {
			netStats->outputErrors++;
		}
		return kIOReturnOutputDropped;
	}
	if (txq.freeCount == 0) {
		reclaimTx();
		if (txq.freeCount == 0) {
			return kIOReturnOutputStall;
		}
	}
	uint16_t id = txq.freeList[--txq.freeCount];
	uint8_t *buf = txq.bufs + (size_t)id * BUF_SIZE;
	bzero(buf, NET_HDR_SIZE);
	mbuf_copydata(m, 0, len, buf + NET_HDR_SIZE);
	freePacket(m);

	struct vring_desc *d = &txq.desc[id];
	d->addr = OSSwapHostToLittleInt64(txq.bufsPhys + (uint64_t)id * BUF_SIZE);
	d->len = OSSwapHostToLittleInt32((uint32_t)(NET_HDR_SIZE + len));
	d->flags = 0;
	d->next = 0;
	uint16_t idx = OSSwapLittleToHostInt16(txq.avail->idx);
	txq.avail->ring[idx % QUEUE_SIZE] = OSSwapHostToLittleInt16(id);
	OSMemoryBarrier();
	txq.avail->idx = OSSwapHostToLittleInt16((uint16_t)(idx + 1));
	OSMemoryBarrier();
	wr(VIRTIO_MMIO_QUEUE_NOTIFY, TX_QUEUE);
	if (netStats != NULL) {
		netStats->outputPackets++;
	}
	return kIOReturnOutputSuccess;
}

/* Take back the transmit buffers the device is done with. */
bool
VirtioMMIONet::reclaimTx(void)
{
	bool any = false;
	OSMemoryBarrier();
	while (txq.lastUsed != OSSwapLittleToHostInt16(txq.used->idx)) {
		struct vring_used_elem *e = &txq.used->ring[txq.lastUsed % QUEUE_SIZE];
		uint32_t id = OSSwapLittleToHostInt32(e->id);
		if (id < QUEUE_SIZE && txq.freeCount < QUEUE_SIZE) {
			txq.freeList[txq.freeCount++] = (uint16_t)id;
		}
		txq.lastUsed++;
		any = true;
	}
	return any;
}

void
VirtioMMIONet::receivePackets(void)
{
	bool any = false;
	OSMemoryBarrier();
	while (rxq.lastUsed != OSSwapLittleToHostInt16(rxq.used->idx)) {
		struct vring_used_elem *e = &rxq.used->ring[rxq.lastUsed % QUEUE_SIZE];
		uint32_t id = OSSwapLittleToHostInt32(e->id);
		uint32_t len = OSSwapLittleToHostInt32(e->len);
		rxq.lastUsed++;
		if (id >= QUEUE_SIZE) {
			continue;
		}
		if (enabled && len > NET_HDR_SIZE && len <= BUF_SIZE) {
			uint32_t frame = len - NET_HDR_SIZE;
			mbuf_t m = allocatePacket(frame);
			if (m != NULL) {
				mbuf_copyback(m, 0, frame, rxq.bufs + (size_t)id * BUF_SIZE + NET_HDR_SIZE,
				    MBUF_DONTWAIT);
				netif->inputPacket(m, frame, IONetworkInterface::kInputOptionQueuePacket);
				any = true;
			}
		}
		postRxBuffer((uint16_t)id);
	}
	OSMemoryBarrier();
	wr(VIRTIO_MMIO_QUEUE_NOTIFY, RX_QUEUE);
	if (any) {
		netif->flushInputQueue();
	}
}

/* On the work loop: the device used buffers (bit 0) or changed config (bit 1). */
void
VirtioMMIONet::interruptOccurred(IOInterruptEventSource *src, int count)
{
	(void)src;
	(void)count;
	uint32_t status = rd(VIRTIO_MMIO_INTERRUPT_STATUS);
	wr(VIRTIO_MMIO_INTERRUPT_ACK, status);
	receivePackets();
	if (reclaimTx() && enabled) {
		txQueue->service(IOBasicOutputQueue::kServiceAsync);
	}
}

IOReturn
VirtioMMIONet::getHardwareAddress(IOEthernetAddress *addr)
{
	*addr = mac;
	return kIOReturnSuccess;
}

/* The device passes every frame up; filtering is the stack's. */
IOReturn VirtioMMIONet::setPromiscuousMode(bool active) { (void)active; return kIOReturnSuccess; }
IOReturn VirtioMMIONet::setMulticastMode(bool active) { (void)active; return kIOReturnSuccess; }

IOReturn
VirtioMMIONet::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
	(void)addrs;
	(void)count;
	return kIOReturnSuccess;
}
