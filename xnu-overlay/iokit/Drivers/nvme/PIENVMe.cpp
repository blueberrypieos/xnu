/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMessage.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <kern/clock.h>
#include <libkern/OSByteOrder.h>
#include <pexpert/pexpert.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#include <IOKit/storage/IOBlockStorageDevice.h>
#include <IOKit/storage/IOStorageDeviceCharacteristics.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>
#pragma clang diagnostic pop

#include "../rpi/RPiPlatform.h"

enum {
	NVME_CAP = 0x00, NVME_VS = 0x08, NVME_INTMS = 0x0c, NVME_INTMC = 0x10, NVME_CC = 0x14,
	NVME_CSTS = 0x1c, NVME_AQA = 0x24, NVME_ASQ = 0x28, NVME_ACQ = 0x30, NVME_DOORBELLS = 0x1000,
};

enum {
	CC_EN = 1U << 0,
	CC_SHN_NORMAL = 1U << 14,
	CC_SHN_MASK = 3U << 14,
	CC_IOSQES = 6U << 16,
	CC_IOCQES = 4U << 20,
	CSTS_RDY = 1U << 0,
	CSTS_CFS = 1U << 1,
	CSTS_SHST_MASK = 3U << 2,
	CSTS_SHST_DONE = 2U << 2,
};

enum {
	ADMIN_CREATE_SQ = 0x01, ADMIN_CREATE_CQ = 0x05, ADMIN_IDENTIFY = 0x06, ADMIN_SET_FEATURES = 0x09,
	NVM_FLUSH = 0x00, NVM_WRITE = 0x01, NVM_READ = 0x02,

	IDENTIFY_NAMESPACE = 0, IDENTIFY_CONTROLLER = 1,
	FEATURE_NUMBER_OF_QUEUES = 0x07,
};

#define PAGE            4096
#define MAX_QUEUE       64
#define MAX_TRANSFER    (128 * 1024)
#define MAX_NAMESPACES  4

enum {
	POOL_ADMIN_SQ = 0x0000, POOL_ADMIN_CQ = 0x1000, POOL_IO_SQ = 0x2000, POOL_IO_CQ = 0x3000,
	POOL_IDENTIFY = 0x4000, POOL_PRP_LIST = 0x5000, POOL_SIZE = 0x6000,
};

struct NVMeCommand {
	uint8_t opcode;
	uint8_t flags;
	uint16_t cid;
	uint32_t nsid;
	uint64_t reserved;
	uint64_t mptr;
	uint64_t prp1;
	uint64_t prp2;
	uint32_t cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
};
static_assert(sizeof(struct NVMeCommand) == 64, "NVMe submission entries are 64 bytes");

struct NVMeCompletion {
	uint32_t result;
	uint32_t reserved;
	uint16_t sqHead;
	uint16_t sqID;
	uint16_t cid;
	uint16_t status;
};
static_assert(sizeof(struct NVMeCompletion) == 16, "NVMe completion entries are 16 bytes");

class PIENVMeController;

class PIENVMeNamespace : public IOBlockStorageDevice
{
	OSDeclareDefaultStructors(PIENVMeNamespace);

public:
	PIENVMeController *controller;
	uint32_t nsid;
	uint32_t blockSize;
	uint64_t blocks;

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;

	virtual IOReturn doEjectMedia(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn doFormatMedia(UInt64 byteCapacity) APPLE_KEXT_OVERRIDE;
	virtual UInt32 doGetFormatCapacities(UInt64 *capacities, UInt32 capacitiesMaxCount) const APPLE_KEXT_OVERRIDE;
	virtual char *getVendorString(void) APPLE_KEXT_OVERRIDE;
	virtual char *getProductString(void) APPLE_KEXT_OVERRIDE;
	virtual char *getRevisionString(void) APPLE_KEXT_OVERRIDE;
	virtual char *getAdditionalDeviceInfoString(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportBlockSize(UInt64 *blockSize) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportEjectability(bool *isEjectable) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportMaxValidBlock(UInt64 *maxBlock) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportMediaState(bool *mediaPresent, bool *changedState) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportRemovability(bool *isRemovable) APPLE_KEXT_OVERRIDE;
	virtual IOReturn reportWriteProtection(bool *isWriteProtected) APPLE_KEXT_OVERRIDE;
	virtual IOReturn doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 nblks,
	    IOStorageAttributes *attributes, IOStorageCompletion *completion) APPLE_KEXT_OVERRIDE;
	virtual IOReturn doSynchronize(UInt64 block, UInt64 nblks, IOStorageSynchronizeOptions options) APPLE_KEXT_OVERRIDE;
};

class PIENVMeController : public IOService
{
	OSDeclareDefaultStructors(PIENVMeController);

	struct Queue {
		volatile struct NVMeCommand *sq;
		volatile struct NVMeCompletion *cq;
		uint64_t sqBus, cqBus;
		uint16_t id, size, sqTail, cqHead, nextCID;
		uint8_t phase;
	};

	RPiPCIDevice *pci;
	IOMemoryMap *regsMap;
	uintptr_t regs;
	uint32_t doorbellStride;
	uint32_t readyTimeoutMs;
	IOBufferMemoryDescriptor *poolMem;
	IOMemoryMap *poolMap;
	uint8_t *pool;
	uint64_t poolBus;
	IOBufferMemoryDescriptor *bounceMem;
	uint64_t bounceBus;
	struct Queue admin, io;
	IOLock *cmdLock;
	IOLock *lock;
	bool useInterrupt;
	IOWorkLoop *workLoop;
	IOInterruptEventSource *intSource;
	IONotifier *powerNotifier;

	uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(regs + off); }
	void wr32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(regs + off) = v; }
	uint64_t rd64(uint32_t off) { return (uint64_t)rd32(off + 4) << 32 | rd32(off); }
	void wr64(uint32_t off, uint64_t v) { wr32(off, (uint32_t)v); wr32(off + 4, (uint32_t)(v >> 32)); }

	bool waitReady(bool ready);
	void initQueue(struct Queue *q, uint16_t id, uint16_t size, uint32_t sqOff, uint32_t cqOff);
	IOReturn submit(struct Queue *q, struct NVMeCommand *cmd, uint32_t *result, uint32_t timeoutMs);
	IOReturn identify(uint32_t cns, uint32_t nsid);
	void interruptOccurred(IOInterruptEventSource *source, int count);

public:
	char model[41];
	char serial[21];
	char firmware[9];
	uint32_t maxTransfer;
	bool volatileCache;

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	void shutDown(void);

	IOReturn readWrite(uint32_t nsid, uint64_t block, uint32_t count, uint32_t blockSize,
	    IOMemoryDescriptor *buffer, IOByteCount offset, bool write);
	IOReturn flush(uint32_t nsid);
};

static IOReturn nvme_power_message(void *target, void *refCon, UInt32 messageType, IOService *provider,
    void *argument, vm_size_t size);

#define super IOService
OSDefineMetaClassAndStructors(PIENVMeController, IOService);

#pragma mark Queues and commands

bool
PIENVMeController::waitReady(bool ready)
{
	for (uint32_t ms = 0; ms <= readyTimeoutMs; ms++) {
		uint32_t csts = rd32(NVME_CSTS);
		if (csts == 0xffffffff || (csts & CSTS_CFS)) {
			return false;
		}
		if (((csts & CSTS_RDY) != 0) == ready) {
			return true;
		}
		IOSleep(1);
	}
	return false;
}

void
PIENVMeController::initQueue(struct Queue *q, uint16_t id, uint16_t size, uint32_t sqOff, uint32_t cqOff)
{
	q->sq = (volatile struct NVMeCommand *)(void *)(pool + sqOff);
	q->cq = (volatile struct NVMeCompletion *)(void *)(pool + cqOff);
	q->sqBus = poolBus + sqOff;
	q->cqBus = poolBus + cqOff;
	q->id = id;
	q->size = size;
	q->sqTail = 0;
	q->cqHead = 0;
	q->nextCID = 0;
	q->phase = 1;
	bzero((void *)(uintptr_t)q->sq, size * sizeof(struct NVMeCommand));
	bzero((void *)(uintptr_t)q->cq, size * sizeof(struct NVMeCompletion));
}

IOReturn
PIENVMeController::submit(struct Queue *q, struct NVMeCommand *cmd, uint32_t *result, uint32_t timeoutMs)
{
	cmd->cid = q->nextCID++;
	volatile struct NVMeCommand *slot = &q->sq[q->sqTail];
	const uint32_t *src = (const uint32_t *)cmd;
	volatile uint32_t *dst = (volatile uint32_t *)slot;
	for (unsigned i = 0; i < sizeof(*cmd) / 4; i++) {
		dst[i] = src[i];
	}
	q->sqTail = (uint16_t)((q->sqTail + 1) % q->size);
	__builtin_arm_dsb(0xf);
	wr32(NVME_DOORBELLS + (2 * q->id) * doorbellStride, q->sqTail);

	uint64_t deadline;
	clock_interval_to_deadline(timeoutMs, kMillisecondScale, &deadline);
	uint16_t status, cid;
	IOLockLock(lock);
	for (;;) {
		volatile struct NVMeCompletion *c = &q->cq[q->cqHead];
		if ((c->status & 1) == q->phase) {
			__builtin_arm_dmb(0xf);
			status = c->status;
			cid = c->cid;
			if (result != NULL) {
				*result = c->result;
			}
			q->cqHead = (uint16_t)((q->cqHead + 1) % q->size);
			if (q->cqHead == 0) {
				q->phase ^= 1;
			}
			wr32(NVME_DOORBELLS + (2 * q->id + 1) * doorbellStride, q->cqHead);
			if (cid == cmd->cid) {
				break;
			}
			IOLog("NVMe: late completion for cid %u (waiting for %u)\n", cid, cmd->cid);
			continue;
		}
		if (mach_absolute_time() >= deadline) {
			IOLockUnlock(lock);
			IOLog("NVMe: %s command 0x%02x (cid %u) timed out, CSTS 0x%08x\n", q->id ? "I/O" : "admin",
			    cmd->opcode, cmd->cid, rd32(NVME_CSTS));
			return kIOReturnTimeout;
		}
		if (useInterrupt) {
			wr32(NVME_INTMC, 1);
		}
		uint64_t wake;
		clock_interval_to_deadline(10, kMillisecondScale, &wake);
		IOLockSleepDeadline(lock, q, wake < deadline ? wake : deadline, THREAD_UNINT);
	}
	IOLockUnlock(lock);
	uint16_t sf = status >> 1;
	if (sf != 0) {
		IOLog("NVMe: %s command 0x%02x failed: status type %u, code 0x%02x%s\n", q->id ? "I/O" : "admin",
		    cmd->opcode, (sf >> 8) & 7, sf & 0xff, (sf & 0x4000) ? ", do not retry" : "");
		return kIOReturnIOError;
	}
	return kIOReturnSuccess;
}

void
PIENVMeController::interruptOccurred(IOInterruptEventSource *source, int count)
{
	(void)source;
	(void)count;
	wr32(NVME_INTMS, 1);
	IOLockLock(lock);
	IOLockWakeup(lock, &admin, false);
	IOLockWakeup(lock, &io, false);
	IOLockUnlock(lock);
}

IOReturn
PIENVMeController::identify(uint32_t cns, uint32_t nsid)
{
	struct NVMeCommand cmd = {};
	cmd.opcode = ADMIN_IDENTIFY;
	cmd.nsid = nsid;
	cmd.prp1 = poolBus + POOL_IDENTIFY;
	cmd.cdw10 = cns;
	bzero(pool + POOL_IDENTIFY, PAGE);
	return submit(&admin, &cmd, NULL, 5000);
}

static void
nvme_string(char *dst, const volatile uint8_t *src, unsigned length)
{
	unsigned n = length;
	while (n > 0 && (src[n - 1] == ' ' || src[n - 1] == 0)) {
		n--;
	}
	for (unsigned i = 0; i < n; i++) {
		dst[i] = (src[i] >= 0x20 && src[i] < 0x7f) ? (char)src[i] : '?';
	}
	dst[n] = 0;
}

#pragma mark Data

IOReturn
PIENVMeController::readWrite(uint32_t nsid, uint64_t block, uint32_t count, uint32_t blockSize,
    IOMemoryDescriptor *buffer, IOByteCount offset, bool write)
{
	IOByteCount bytes = (IOByteCount)count * blockSize;
	void *bounce = bounceMem->getBytesNoCopy();
	IOLockLock(cmdLock);
	if (write) {
		if (buffer->readBytes(offset, bounce, bytes) != bytes) {
			IOLockUnlock(cmdLock);
			return kIOReturnIOError;
		}
		bounceMem->performOperation(kIOMemoryIncoherentIOStore, 0, bytes);
	} else {
		bounceMem->performOperation(kIOMemoryIncoherentIOFlush, 0, bytes);
	}

	struct NVMeCommand cmd = {};
	cmd.opcode = write ? NVM_WRITE : NVM_READ;
	cmd.nsid = nsid;
	cmd.prp1 = bounceBus;
	if (bytes > 2 * PAGE) {
		cmd.prp2 = poolBus + POOL_PRP_LIST;
	} else if (bytes > PAGE) {
		cmd.prp2 = bounceBus + PAGE;
	}
	cmd.cdw10 = (uint32_t)block;
	cmd.cdw11 = (uint32_t)(block >> 32);
	cmd.cdw12 = count - 1;
	IOReturn ret = submit(&io, &cmd, NULL, 30000);
	if (ret == kIOReturnSuccess && !write) {
		bounceMem->performOperation(kIOMemoryIncoherentIOFlush, 0, bytes);
		if (buffer->writeBytes(offset, bounce, bytes) != bytes) {
			ret = kIOReturnIOError;
		}
	}
	IOLockUnlock(cmdLock);
	if (ret != kIOReturnSuccess) {
		IOLog("NVMe: %s of %u blocks at %llu (namespace %u) failed\n", write ? "write" : "read", count, block, nsid);
	}
	return ret;
}

IOReturn
PIENVMeController::flush(uint32_t nsid)
{
	if (!volatileCache) {
		return kIOReturnSuccess;
	}
	struct NVMeCommand cmd = {};
	cmd.opcode = NVM_FLUSH;
	cmd.nsid = nsid;
	IOLockLock(cmdLock);
	IOReturn ret = submit(&io, &cmd, NULL, 30000);
	IOLockUnlock(cmdLock);
	return ret;
}

#pragma mark Start and stop

bool
PIENVMeController::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	pci = OSDynamicCast(RPiPCIDevice, provider);
	if (pci == NULL) {
		return false;
	}
	int bar0 = pci->barIndex(0);
	regsMap = bar0 >= 0 ? provider->mapDeviceMemoryWithIndex((unsigned)bar0, kIOMapInhibitCache) : NULL;
	if (regsMap == NULL) {
		IOLog("NVMe: no BAR0\n");
		return false;
	}
	regs = (uintptr_t)regsMap->getVirtualAddress();

	uint64_t cap = rd64(NVME_CAP);
	uint32_t vs = rd32(NVME_VS);
	if (cap == ~0ULL || vs == 0xffffffff) {
		IOLog("NVMe: the controller does not answer (CAP 0x%llx)\n", cap);
		return false;
	}
	uint32_t mqes = (uint32_t)(cap & 0xffff) + 1;
	doorbellStride = 4U << ((cap >> 32) & 0xf);
	readyTimeoutMs = (uint32_t)((cap >> 24) & 0xff) * 500 + 500;
	uint32_t mpsMin = 12 + (uint32_t)((cap >> 48) & 0xf);
	if (mpsMin > 12 || !((cap >> 37) & 1)) {
		IOLog("NVMe: needs %u-byte pages or lacks the NVM command set (CAP 0x%llx)\n", 1U << mpsMin, cap);
		return false;
	}
	uint16_t qsize = (uint16_t)(mqes < MAX_QUEUE ? mqes : MAX_QUEUE);

	cmdLock = IOLockAlloc();
	lock = IOLockAlloc();
	if (cmdLock == NULL || lock == NULL) {
		return false;
	}
	poolMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, POOL_SIZE, 0x7ffff000ULL);
	bounceMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, MAX_TRANSFER, 0x7ffff000ULL);
	if (poolMem == NULL || bounceMem == NULL) {
		IOLog("NVMe: no DMA memory\n");
		return false;
	}
	poolMem->performOperation(kIOMemoryIncoherentIOFlush, 0, POOL_SIZE);
	poolMap = poolMem->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
	if (poolMap == NULL) {
		return false;
	}
	pool = (uint8_t *)poolMap->getVirtualAddress();
	if (!RPiDMAAddress(provider, poolMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &poolBus) ||
	    !RPiDMAAddress(provider, bounceMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &bounceBus)) {
		IOLog("NVMe: the controller cannot reach its DMA memory\n");
		return false;
	}
	volatile uint64_t *prpList = (volatile uint64_t *)(void *)(pool + POOL_PRP_LIST);
	for (unsigned i = 1; i < MAX_TRANSFER / PAGE; i++) {
		prpList[i - 1] = bounceBus + (uint64_t)i * PAGE;
	}

	pci->configWrite16(0x04, (uint16_t)(pci->configRead16(0x04) | 0x6));
	if (rd32(NVME_CC) & CC_EN) {
		wr32(NVME_CC, rd32(NVME_CC) & ~CC_EN);
	}
	if (!waitReady(false)) {
		IOLog("NVMe: the controller does not stop (CSTS 0x%08x)\n", rd32(NVME_CSTS));
		return false;
	}
	initQueue(&admin, 0, qsize, POOL_ADMIN_SQ, POOL_ADMIN_CQ);
	wr32(NVME_AQA, (uint32_t)(qsize - 1) << 16 | (uint32_t)(qsize - 1));
	wr64(NVME_ASQ, admin.sqBus);
	wr64(NVME_ACQ, admin.cqBus);
	wr32(NVME_INTMS, 1);
	wr32(NVME_CC, CC_IOCQES | CC_IOSQES | CC_EN);
	if (!waitReady(true)) {
		IOLog("NVMe: the controller does not become ready (CSTS 0x%08x)\n", rd32(NVME_CSTS));
		return false;
	}

	uint32_t irq = 1;
	PE_parse_boot_argn("nvme_irq", &irq, sizeof(irq));
	if (irq) {
		workLoop = IOWorkLoop::workLoop();
		intSource = IOInterruptEventSource::interruptEventSource(this,
		    OSMemberFunctionCast(IOInterruptEventAction, this, &PIENVMeController::interruptOccurred), provider, 0);
		if (workLoop != NULL && intSource != NULL && workLoop->addEventSource(intSource) == kIOReturnSuccess) {
			intSource->enable();
			useInterrupt = true;
			wr32(NVME_INTMC, 1);
		} else {
			IOLog("NVMe: no interrupt; completions are polled\n");
			OSSafeReleaseNULL(intSource);
		}
	}

	IOLockLock(cmdLock);
	IOReturn ret = identify(IDENTIFY_CONTROLLER, 0);
	uint32_t namespaces = 0, mdts = 0;
	if (ret == kIOReturnSuccess) {
		volatile uint8_t *id = pool + POOL_IDENTIFY;
		nvme_string(serial, id + 4, 20);
		nvme_string(model, id + 24, 40);
		nvme_string(firmware, id + 64, 8);
		mdts = id[77];
		volatileCache = (id[525] & 1) != 0;
		namespaces = OSReadLittleInt32((const void *)(uintptr_t)id, 516);
	}
	maxTransfer = MAX_TRANSFER;
	if (mdts != 0 && mdts < 20 && (PAGE << mdts) < maxTransfer) {
		maxTransfer = PAGE << mdts;
	}

	struct NVMeCommand cmd = {};
	if (ret == kIOReturnSuccess) {
		cmd.opcode = ADMIN_SET_FEATURES;
		cmd.cdw10 = FEATURE_NUMBER_OF_QUEUES;
		cmd.cdw11 = 0;
		ret = submit(&admin, &cmd, NULL, 5000);
	}
	if (ret == kIOReturnSuccess) {
		initQueue(&io, 1, qsize, POOL_IO_SQ, POOL_IO_CQ);
		bzero(&cmd, sizeof(cmd));
		cmd.opcode = ADMIN_CREATE_CQ;
		cmd.prp1 = io.cqBus;
		cmd.cdw10 = (uint32_t)(qsize - 1) << 16 | io.id;
		cmd.cdw11 = 1U << 1 | 1U;
		ret = submit(&admin, &cmd, NULL, 5000);
	}
	if (ret == kIOReturnSuccess) {
		bzero(&cmd, sizeof(cmd));
		cmd.opcode = ADMIN_CREATE_SQ;
		cmd.prp1 = io.sqBus;
		cmd.cdw10 = (uint32_t)(qsize - 1) << 16 | io.id;
		cmd.cdw11 = (uint32_t)io.id << 16 | 1U;
		ret = submit(&admin, &cmd, NULL, 5000);
	}
	IOLockUnlock(cmdLock);
	if (ret != kIOReturnSuccess) {
		IOLog("NVMe: controller set-up failed\n");
		return false;
	}
	IOLog("NVMe: %s (serial %s, firmware %s), NVMe %u.%u, %u namespace(s), %u-entry queues, %u KB per "
	    "command, %s, %s\n", model, serial, firmware, vs >> 16, (vs >> 8) & 0xff, namespaces, qsize,
	    maxTransfer / 1024, volatileCache ? "volatile write cache" : "no write cache",
	    useInterrupt ? "INTx" : "polled");
	setProperty("Model", model);
	setProperty("Serial Number", serial);
	setProperty("Firmware Revision", firmware);
	powerNotifier = registerPrioritySleepWakeInterest(&nvme_power_message, this, NULL);
	registerService();

	for (uint32_t nsid = 1; nsid <= namespaces && nsid <= MAX_NAMESPACES; nsid++) {
		IOLockLock(cmdLock);
		ret = identify(IDENTIFY_NAMESPACE, nsid);
		uint64_t size = 0;
		uint32_t lbads = 0;
		if (ret == kIOReturnSuccess) {
			volatile uint8_t *id = pool + POOL_IDENTIFY;
			size = OSReadLittleInt64((const void *)(uintptr_t)id, 0);
			uint32_t format = id[26] & 0xf;
			lbads = id[128 + 4 * format + 2];
		}
		IOLockUnlock(cmdLock);
		if (ret != kIOReturnSuccess || size == 0) {
			continue;
		}
		if (lbads < 9 || lbads > 12) {
			IOLog("NVMe: namespace %u: unsupported block size 2^%u\n", nsid, lbads);
			continue;
		}
		PIENVMeNamespace *ns = OSTypeAlloc(PIENVMeNamespace);
		if (ns == NULL || !ns->init(NULL)) {
			OSSafeReleaseNULL(ns);
			continue;
		}
		ns->controller = this;
		ns->nsid = nsid;
		ns->blockSize = 1U << lbads;
		ns->blocks = size;
		IOLog("NVMe: namespace %u: %llu blocks of %u bytes (%llu MB)\n", nsid, size, ns->blockSize,
		    size * ns->blockSize / (1024 * 1024));
		if (ns->attach(this)) {
			if (!ns->start(this)) {
				ns->detach(this);
			}
		}
		ns->release();
	}
	return true;
}

void
PIENVMeController::shutDown(void)
{
	if (regs == 0 || !(rd32(NVME_CSTS) & CSTS_RDY)) {
		return;
	}
	flush(0xffffffff);
	wr32(NVME_CC, (rd32(NVME_CC) & ~CC_SHN_MASK) | CC_SHN_NORMAL);
	uint32_t ms;
	for (ms = 0; ms < readyTimeoutMs && (rd32(NVME_CSTS) & CSTS_SHST_MASK) != CSTS_SHST_DONE; ms++) {
		IODelay(1000);
	}
	IOLog("NVMe: %s: shut down %s (%u ms)\n", model, ms < readyTimeoutMs ? "cleanly" : "late", ms);
}

static IOReturn
nvme_power_message(void *target, void *refCon, UInt32 messageType, IOService *provider, void *argument, vm_size_t size)
{
	(void)refCon;
	(void)provider;
	(void)argument;
	(void)size;
	if (messageType == kIOMessageSystemWillPowerOff || messageType == kIOMessageSystemWillRestart) {
		((PIENVMeController *)target)->shutDown();
	}
	return kIOReturnSuccess;
}

void
PIENVMeController::stop(IOService *provider)
{
	if (powerNotifier != NULL) {
		powerNotifier->remove();
		powerNotifier = NULL;
	}
	if (intSource != NULL) {
		intSource->disable();
		workLoop->removeEventSource(intSource);
	}
	if (regs != 0) {
		wr32(NVME_CC, rd32(NVME_CC) & ~CC_EN);
	}
	super::stop(provider);
}

void
PIENVMeController::free(void)
{
	OSSafeReleaseNULL(intSource);
	OSSafeReleaseNULL(workLoop);
	OSSafeReleaseNULL(poolMap);
	OSSafeReleaseNULL(poolMem);
	OSSafeReleaseNULL(bounceMem);
	OSSafeReleaseNULL(regsMap);
	if (lock != NULL) {
		IOLockFree(lock);
	}
	if (cmdLock != NULL) {
		IOLockFree(cmdLock);
	}
	super::free();
}

#pragma mark Namespaces

#undef super
#define super IOBlockStorageDevice
OSDefineMetaClassAndStructors(PIENVMeNamespace, IOBlockStorageDevice);

bool
PIENVMeNamespace::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	OSDictionary *proto = OSDictionary::withCapacity(2);
	if (proto != NULL) {
		OSString *s = OSString::withCString(kIOPropertyPhysicalInterconnectTypePCIExpress);
		proto->setObject(kIOPropertyPhysicalInterconnectTypeKey, s);
		s->release();
		s = OSString::withCString(kIOPropertyInternalKey);
		proto->setObject(kIOPropertyPhysicalInterconnectLocationKey, s);
		s->release();
		setProperty(kIOPropertyProtocolCharacteristicsKey, proto);
		proto->release();
	}
	OSDictionary *dev = OSDictionary::withCapacity(1);
	if (dev != NULL) {
		OSString *s = OSString::withCString(kIOPropertyMediumTypeSolidStateKey);
		dev->setObject(kIOPropertyMediumTypeKey, s);
		s->release();
		setProperty(kIOPropertyDeviceCharacteristicsKey, dev);
		dev->release();
	}
	setProperty(kIOMaximumBlockCountReadKey, controller->maxTransfer / blockSize, 64);
	setProperty(kIOMaximumBlockCountWriteKey, controller->maxTransfer / blockSize, 64);
	registerService();
	return true;
}

IOReturn
PIENVMeNamespace::doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 nblks,
    IOStorageAttributes *attributes, IOStorageCompletion *completion)
{
	(void)attributes;
	bool write = buffer->getDirection() == kIODirectionOut;
	uint32_t maxBlocks = controller->maxTransfer / blockSize;
	IOReturn ret = block + nblks > blocks ? kIOReturnBadArgument : kIOReturnSuccess;
	for (UInt64 done = 0; ret == kIOReturnSuccess && done < nblks;) {
		uint32_t n = (uint32_t)(nblks - done < maxBlocks ? nblks - done : maxBlocks);
		ret = controller->readWrite(nsid, block + done, n, blockSize, buffer, done * blockSize, write);
		done += n;
	}
	IOStorage::complete(completion, ret, ret == kIOReturnSuccess ? nblks * blockSize : 0);
	return kIOReturnSuccess;
}

IOReturn
PIENVMeNamespace::doSynchronize(UInt64 block, UInt64 nblks, IOStorageSynchronizeOptions options)
{
	(void)block;
	(void)nblks;
	(void)options;
	return controller->flush(nsid);
}

IOReturn PIENVMeNamespace::doEjectMedia(void) { return kIOReturnUnsupported; }
IOReturn PIENVMeNamespace::doFormatMedia(UInt64 byteCapacity) { (void)byteCapacity; return kIOReturnUnsupported; }

UInt32
PIENVMeNamespace::doGetFormatCapacities(UInt64 *capacities, UInt32 capacitiesMaxCount) const
{
	if (capacities != NULL && capacitiesMaxCount > 0) {
		capacities[0] = blocks * blockSize;
	}
	return 1;
}

char *PIENVMeNamespace::getVendorString(void) { return (char *)"NVMe"; }
char *PIENVMeNamespace::getProductString(void) { return controller->model; }
char *PIENVMeNamespace::getRevisionString(void) { return controller->firmware; }
char *PIENVMeNamespace::getAdditionalDeviceInfoString(void) { return controller->serial; }

IOReturn PIENVMeNamespace::reportBlockSize(UInt64 *size) { *size = blockSize; return kIOReturnSuccess; }
IOReturn PIENVMeNamespace::reportEjectability(bool *isEjectable) { *isEjectable = false; return kIOReturnSuccess; }
IOReturn PIENVMeNamespace::reportMaxValidBlock(UInt64 *maxBlock) { *maxBlock = blocks - 1; return kIOReturnSuccess; }

IOReturn
PIENVMeNamespace::reportMediaState(bool *mediaPresent, bool *changedState)
{
	*mediaPresent = true;
	if (changedState != NULL) {
		*changedState = false;
	}
	return kIOReturnSuccess;
}

IOReturn PIENVMeNamespace::reportRemovability(bool *isRemovable) { *isRemovable = false; return kIOReturnSuccess; }
IOReturn PIENVMeNamespace::reportWriteProtection(bool *isWriteProtected) { *isWriteProtected = false; return kIOReturnSuccess; }
