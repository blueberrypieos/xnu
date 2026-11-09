/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PIEDMAController: the channels, the platform functions, the
 * interrupts' plumbing and the self-test, for its subclasses' hardware.
 *
 * The sysctls, for every controller at once: debug.dma.channels (each
 * usable channel: its number, kind, whether it is taken, its transfers
 * and errors), debug.dma.selftest (root writes N: N memory-to-memory
 * copies on every free channel, of 1 byte to 1 MB at unaligned offsets,
 * each compared byte for byte with the bytes around it untouched; the
 * write fails with EIO if any did not; under QEMU, whose model takes whole
 * words only, the lengths and offsets are multiples of 4) and
 * debug.dma.selftest_result. The boot-arg pie_dma=0 leaves every
 * controller alone (the drivers that would use them go without).
 */

#include <IOKit/IOLib.h>
#include <libkern/OSByteOrder.h>
#include <pexpert/pexpert.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/sysctl.h>

#include <pie/rpi.h>

#include "DMAController.h"
#include "PIEDMAKernel.h"

OSDefineMetaClassAndAbstractStructors(PIEDMAController, IOService);
#define super IOService

enum {
	FW_GET_FIRMWARE_REVISION = 0x00000001,
	FW_VIDEOCORE_SINCE = 1546300800,        /* 2019-01-01 UTC: older than any BCM2711 or BCM2712 firmware */
};

#define kMaxControllers 8
#define kSelfTestBytes  (1024 * 1024 + 64)
#define kResultBytes    2048

/* The registry lock lives for the module, not for a racing last controller. */
static IOLock *sLock;
static PIEDMAController *sControllers[kMaxControllers];
static char sResult[kResultBytes];
static bool sSysctls;
static void registerSysctls(void);
static void unregisterSysctls(void);
struct DMARegistryLifetime {
	DMARegistryLifetime() { sLock = IOLockAlloc(); }
	~DMARegistryLifetime() {
		if (sSysctls) { unregisterSysctls(); }
		if (sLock != NULL) { IOLockFree(sLock); sLock = NULL; }
	}
};
static DMARegistryLifetime sRegistryLifetime;

#pragma mark start and stop

bool
PIEDMAController::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	snprintf(fName, sizeof(fName), "%s", provider->getName());
	uint32_t on = 1;
	if (PE_parse_boot_argn("pie_dma", &on, sizeof(on)) && on == 0) {
		IOLog("PIEDMA: %s: pie_dma=0: left alone\n", fName);
		return false;
	}
	if (sLock == NULL) { return false; }
	IOLockLock(sLock);
	bool clean = PIEDMAPrepareQuarantineFlag(provider, kPIEDMAControllerQuarantineKey);
	IOLockUnlock(sLock);
	if (!clean) { IOLog("PIEDMA: %s: quarantined until reboot\n", fName); return false; }
	fHardwareLock = IOSimpleLockAlloc();
	fLock = IOLockAlloc();
	fWorkLoop = IOWorkLoop::workLoop();
	fGate = fWorkLoop != NULL ? IOCommandGate::commandGate(this) : NULL;
	if (fLock == NULL || fHardwareLock == NULL || fGate == NULL || fWorkLoop->addEventSource(fGate) != kIOReturnSuccess) { return false; }
	fGateAdded = true;
	IODeviceMemory *mem = provider->getDeviceMemoryWithIndex(0);
	fMap = mem != NULL ? provider->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache) : NULL;
	if (fMap == NULL) {
		IOLog("PIEDMA: %s: cannot map its registers\n", fName);
		return false;
	}
	fRegs = (volatile uint8_t *)fMap->getVirtualAddress();
	fPhys = mem->getPhysicalAddress();
	if (!hwStart(provider)) {
		return false;
	}
	unsigned usable = 0;
	for (unsigned i = 0; i < fCount; i++) {
		if (fChannels[i].usable && !setUpCB(fChannels[i])) {
			IOLog("PIEDMA: %s: no memory for channel %u's control blocks\n", fName, fChannels[i].number);
			fChannels[i].usable = false;
		}
		usable += fChannels[i].usable;
	}
	if (usable == 0) {
		IOLog("PIEDMA: %s: no channel to use\n", fName);
		return false;
	}
	OSData *native = OSDynamicCast(OSData, provider->getProperty("AAPL,phandle"));
	OSData *raw = OSDynamicCast(OSData, provider->getProperty("phandle"));
	uint32_t handle = 0;
	if (native != NULL && native->getLength() == 4) { bcopy(native->getBytesNoCopy(), &handle, 4); }
	else if (raw != NULL && raw->getLength() == 4) { handle = OSReadBigInt32(raw->getBytesNoCopy(), 0); }
	if (handle == 0) { return false; }
	setProperty(kPIEDMAPHandleKey, handle, 32);
	for (unsigned i = 0; i < fInterruptCount; i++) {
		fInterrupts[i]->enable();
	}

	IOLockLock(sLock);
	for (unsigned i = 0; i < kMaxControllers && !fListed; i++) {
		if (sControllers[i] == NULL) { sControllers[i] = this; fListed = true; }
	}
	if (fListed && !sSysctls) { registerSysctls(); sSysctls = true; }
	IOLockUnlock(sLock);
	if (!fListed) { return false; }
	char line[256];
	describe(line, sizeof(line));
	IOLog("PIEDMA: %s", line);
	registerService();
	return true;
}

void
PIEDMAController::cleanup(void)
{
	if (fListed) {
		IOLockLock(sLock);
		for (unsigned i = 0; i < kMaxControllers; i++) {
			if (sControllers[i] == this) { sControllers[i] = NULL; }
		}
		fListed = false;
		IOLockUnlock(sLock);
	}
	for (unsigned i = 0; i < fInterruptCount; i++) { fInterrupts[i]->disable(); }
	if (fGateAdded) {
		fGate->runActionBlock(^IOReturn {
			fStopping = true;
			for (unsigned i = 0; i < fCount; i++) {
				Channel &c = fChannels[i];
				if (c.running) { complete(c, kIOReturnAborted); }
			}
			return kIOReturnSuccess;
		});
	}
	for (unsigned i = 0; i < fInterruptCount; i++) {
		fWorkLoop->removeEventSource(fInterrupts[i]);
		OSSafeReleaseNULL(fInterrupts[i]);
	}
	fInterruptCount = 0;
	if (fGateAdded) { fWorkLoop->removeEventSource(fGate); fGateAdded = false; }
}

void
PIEDMAController::stop(IOService *provider)
{
	cleanup();
	super::stop(provider);
}

void
PIEDMAController::free(void)
{
	cleanup();
	for (unsigned i = 0; i < kDMAMaxChannels; i++) {
		Channel &c = fChannels[i];
		OSSafeReleaseNULL(c.cbMap);
		OSSafeReleaseNULL(c.client);
		if (!c.quarantined) {
			OSSafeReleaseNULL(c.cbMem);
			OSSafeReleaseNULL(c.payload[0]);
			OSSafeReleaseNULL(c.payload[1]);
		}
		/* On unconfirmed shutdown these references intentionally pin physical
		 * memory until reboot, even across a kext unload. Never recycle it. */
	}
	OSSafeReleaseNULL(fMap);
	if (fHardwareLock != NULL) { IOSimpleLockFree(fHardwareLock); fHardwareLock = NULL; }
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fWorkLoop);
	if (fLock != NULL) { IOLockFree(fLock); fLock = NULL; }
	super::free();
}

/* The firmware's revision says QEMU (older than any Pi 4 or 5 firmware), as the other drivers tell. */
bool
PIEDMAController::isQEMU(void)
{
	uint32_t revision = 0;
	return RPiFirmwareProperty(FW_GET_FIRMWARE_REVISION, &revision, sizeof(revision)) != kIOReturnSuccess ||
	       revision < FW_VIDEOCORE_SINCE;
}

/*
 * A channel's control blocks: contiguous, page-aligned, below the
 * controller's reach. Select normal noncached memory at allocation as
 * well as mapping: a cached original alias can hide descriptors from DMA.
 * ARM coherent-I/O builds may deliberately make performOperation a no-op.
 */
bool
PIEDMAController::setUpCB(Channel &c)
{
	uint64_t mask = hwMaxAddress();
	if (mask > 0xFFFFFFFFULL) {
		mask = 0xFFFFFFFFULL;
	}
	mask &= ~0xFFFULL;
	uint32_t bytes = hwCBBytes();
	c.cbMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, bytes, mask);
	if (c.cbMem == NULL) {
		return false;
	}
	c.cbMem->performOperation(kIOMemoryIncoherentIOFlush, 0, bytes);
	c.cbMap = c.cbMem->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
	if (c.cbMap == NULL || !hwMemoryAddress(c.cbMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &c.cbBus)) {
		return false;
	}
	c.cb = (volatile uint8_t *)c.cbMap->getVirtualAddress();
	return true;
}

#pragma mark interrupts

bool
PIEDMAController::addInterrupt(IOService *provider, unsigned index, unsigned line)
{
	if (fInterruptCount >= kDMAMaxInterrupts) {
		return false;
	}
	IOFilterInterruptEventSource *s = IOFilterInterruptEventSource::filterInterruptEventSource(this,
	    interruptAction, interruptFilter, provider, (int)index);
	if (s == NULL || fWorkLoop->addEventSource(s) != kIOReturnSuccess) {
		OSSafeReleaseNULL(s);
		return false;
	}
	s->disable();
	fLines[fInterruptCount] = line;
	fInterrupts[fInterruptCount++] = s;
	return true;
}

bool
PIEDMAController::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
	PIEDMAController *me = (PIEDMAController *)owner;
	for (unsigned i = 0; i < me->fInterruptCount; i++) {
		if (me->fInterrupts[i] == source) {
			me->hwFilter(me->fLines[i]);
		}
	}
	return true;
}

void
PIEDMAController::interruptAction(OSObject *owner, IOInterruptEventSource *source, int)
{
	PIEDMAController *me = (PIEDMAController *)owner;
	for (unsigned i = 0; i < me->fInterruptCount; i++) {
		if (me->fInterrupts[i] == source) {
			me->hwService(me->fLines[i]);
		}
	}
}

/* On the work loop: the transfer has ended, one way or another; its waiter wakes. */
void
PIEDMAController::complete(Channel &c, IOReturn status)
{
	if (!c.running) {
		return;
	}
	if (!hwAbort(c)) {
		c.quarantined = true;
		c.usable = false;
		getProvider()->setProperty(kPIEDMAControllerQuarantineKey, kOSBooleanTrue);
		if (c.client != NULL) { c.client->setProperty(kPIEDMAPeripheralQuarantineKey, kOSBooleanTrue); }
		status = kIOReturnNotResponding;
		IOLog("PIEDMA: %s: channel %u did not quiesce; control/payload memory quarantined until reboot\n",
		    fName, c.number);
	} else {
		OSSafeReleaseNULL(c.payload[0]);
		OSSafeReleaseNULL(c.payload[1]);
	}
	c.running = false;
	c.transfers++;
	if (status != kIOReturnSuccess) {
		c.errors++;
	}
	IOLockLock(fLock);
	c.status = status;
	c.done = true;
	IOLockWakeup(fLock, &c, false);
	IOLockUnlock(fLock);
}

#pragma mark the platform functions

PIEDMAController::Channel *
PIEDMAController::channelFor(void *channel)
{
	for (unsigned i = 0; i < fCount; i++) {
		if (channel == &fChannels[i] && fChannels[i].allocated) {
			return &fChannels[i];
		}
	}
	return NULL;
}

/* The best free channel that fits; on the work loop. */
IOReturn
PIEDMAController::allocate(PIEDMARequest *r, bool internal)
{
	if (fStopping || PIEDMAIsQuarantined(getProvider(), kPIEDMAControllerQuarantineKey)) { return kIOReturnNotResponding; }
	if (!internal && (r->client == NULL || (r->width != 1 && r->width != 2 && r->width != 4))) {
		return kIOReturnBadArgument;
	}
	uint64_t device = 0;
	if (!internal && !hwDeviceAddress(r->client, r->registerOffset, &device)) {
		return kIOReturnUnsupported;
	}
	Channel *best = NULL;
	int bestFit = -1;
	for (unsigned i = 0; i < fCount; i++) {
		Channel &c = fChannels[i];
		int fit = internal ? 0 : hwFit(c, r);
		if (c.usable && !c.allocated && fit > bestFit) {
			best = &c;
			bestFit = fit;
		}
	}
	if (best == NULL) {
		return kIOReturnNoResources;
	}
	best->allocated = true;
	best->client = internal ? NULL : r->client;
	if (best->client != NULL) { best->client->retain(); }
	best->toDevice = r->toDevice;
	best->device = device;
	best->width = r->width;
	best->burst = r->burst != 0 ? r->burst : 1;
	best->cell = r->cell;
	r->channel = best;
	r->maxAddress = hwMaxAddress();
	r->maxLength = hwMaxLength(*best);
	return kIOReturnSuccess;
}

/* On the caller's thread: the transfer started on the work loop. */
IOReturn
PIEDMAController::startTransfer(Channel &c, int direction, uint64_t src, uint64_t dst, uint32_t length,
    IOBufferMemoryDescriptor *sourceMemory, IOBufferMemoryDescriptor *destinationMemory)
{
	/* What the CPU wrote through uncached mappings is in memory before the controller reads it. */
	__builtin_arm_dsb(0xf);
	IOReturn ret = fGate->runActionBlock(^IOReturn {
		if (fStopping || c.quarantined || PIEDMAIsQuarantined(getProvider(), kPIEDMAControllerQuarantineKey)) { return kIOReturnNotResponding; }
		/* A transfer still running keeps its state for its waiter. */
		if (c.running) {
			return kIOReturnBusy;
		}
		IOLockLock(fLock);
		c.done = false;
		c.status = kIOReturnSuccess;
		IOLockUnlock(fLock);
		c.payload[0] = sourceMemory;
		c.payload[1] = destinationMemory;
		if (sourceMemory != NULL) { sourceMemory->retain(); }
		if (destinationMemory != NULL) { destinationMemory->retain(); }
		c.running = true;
		IOReturn r = hwTransfer(c, direction, src, dst, length);
		if (r != kIOReturnSuccess) {
			complete(c, r);
			if (c.quarantined) { return kIOReturnNotResponding; }
		}
		return r;
	});
	if (ret != kIOReturnSuccess && ret != kIOReturnBusy) {
		IOLockLock(fLock);
		c.done = true;
		c.status = ret;
		IOLockUnlock(fLock);
	}
	return ret;
}

/* On the caller's thread, not the work loop's. */
IOReturn
PIEDMAController::wait(Channel &c, uint64_t timeoutNs)
{
	uint64_t deadline;
	clock_interval_to_deadline((uint32_t)(timeoutNs / 1000), kMicrosecondScale, &deadline);
	IOLockLock(fLock);
	while (!c.done) {
		if (IOLockSleepDeadline(fLock, &c, deadline, THREAD_UNINT) == THREAD_TIMED_OUT) {
			break;
		}
	}
	bool done = c.done;
	IOLockUnlock(fLock);
	if (!done) {
		fGate->runActionBlock(^IOReturn {
			if (c.running) {
				complete(c, kIOReturnTimeout);
			}
			return kIOReturnSuccess;
		});
	}
	IOLockLock(fLock);
	IOReturn status = c.done ? c.status : kIOReturnTimeout;
	IOLockUnlock(fLock);
	return status;
}

void
PIEDMAController::release(Channel &c)
{
	fGate->runActionBlock(^IOReturn {
		if (c.running) {
			complete(c, kIOReturnAborted);
		}
		if (!c.quarantined) { c.allocated = false; }
		OSSafeReleaseNULL(c.client);
		return kIOReturnSuccess;
	});
}

IOReturn
PIEDMAController::callPlatformFunction(const OSSymbol *functionName, bool waitForFunction, void *param1,
    void *param2, void *param3, void *param4)
{
	if (functionName->isEqualTo(kPIEDMAAllocate)) {
		PIEDMARequest *r = (PIEDMARequest *)param1;
		if (r == NULL) {
			return kIOReturnBadArgument;
		}
		IOLockLock(sLock);
		bool clean = PIEDMAPrepareQuarantineFlag(r->client, kPIEDMAPeripheralQuarantineKey) &&
		    !PIEDMAIsQuarantined(getProvider(), kPIEDMAControllerQuarantineKey);
		IOLockUnlock(sLock);
		if (!clean) { return kIOReturnNotResponding; }
		return fGate->runActionBlock(^IOReturn {
			return allocate(r, false);
		});
	}
	if (functionName->isEqualTo(kPIEDMAStart)) {
		Channel *c = channelFor(param1);
		if (c == NULL || param2 == NULL || param3 == NULL) {
			return kIOReturnBadArgument;
		}
		IOBufferMemoryDescriptor *memory = OSDynamicCast(IOBufferMemoryDescriptor, (OSObject *)param4);
		uint64_t pa = *(uint64_t *)param2, bus;
		uint32_t length = *(uint32_t *)param3;
		IOByteCount segment = 0;
		uint64_t base = memory != NULL ? memory->getPhysicalSegment(0, &segment, kIOMemoryMapperNone) : 0;
		if (memory == NULL || c->quarantined || length == 0 || length > memory->getLength() ||
		    pa < base || pa - base > segment || length > segment - (pa - base) ||
		    length > hwMaxLength(*c) || length % c->width != 0 || pa > hwMaxAddress() ||
		    length - 1 > hwMaxAddress() - pa || !hwMemoryAddress(pa, &bus)) {
			return kIOReturnBadArgument;
		}
		return c->toDevice ? startTransfer(*c, kDMAMemToDev, bus, c->device, length, memory, NULL) :
		       startTransfer(*c, kDMADevToMem, c->device, bus, length, NULL, memory);
	}
	if (functionName->isEqualTo(kPIEDMAWait)) {
		Channel *c = channelFor(param1);
		return c == NULL || param2 == NULL ? kIOReturnBadArgument : wait(*c, *(uint64_t *)param2);
	}
	if (functionName->isEqualTo(kPIEDMARelease)) {
		Channel *c = channelFor(param1);
		if (c == NULL) {
			return kIOReturnBadArgument;
		}
		release(*c);
		return c->quarantined ? kIOReturnNotResponding : kIOReturnSuccess;
	}
	return super::callPlatformFunction(functionName, waitForFunction, param1, param2, param3, param4);
}

#pragma mark the self-test

/* A line for each usable channel. */
void
PIEDMAController::describe(char *out, size_t outLen)
{
	size_t n = (size_t)snprintf(out, outLen, "%s:", fName);
	for (unsigned i = 0; i < fCount && n < outLen; i++) {
		Channel &c = fChannels[i];
		if (c.usable || c.quarantined) {
			n += (size_t)snprintf(out + n, outLen - n, " %u%s(%llu/%llu)", c.number, c.quarantined ? "!" : c.allocated ? "*" : "",
			    c.transfers, c.errors);
		}
	}
	if (n < outLen) {
		snprintf(out + n, outLen - n, "\n");
	}
}

static uint32_t
xorshift(uint32_t *s)
{
	uint32_t x = *s;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return *s = x;
}

/*
 * n copies on each free channel: lengths from 1 byte to 1 MB (as far as
 * the channel goes), at source and destination offsets 1-15 into their
 * buffers, which are filled with a pattern and with 0xa5; the copy must
 * match and the bytes just outside it must still be 0xa5. On the caller's
 * thread (a sysctl's), never the work loop's.
 */
bool
PIEDMAController::selfTest(unsigned n, char *out, size_t outLen)
{
	if (PIEDMAIsQuarantined(getProvider(), kPIEDMAControllerQuarantineKey)) {
		snprintf(out, outLen, "%s: quarantined until reboot\n", fName); return false;
	}
	static const uint32_t kLengths[] = {
		1, 2, 3, 4, 7, 31, 32, 33, 255, 4095, 4096, 4097, 65532, 65536, 65537, 262144, 1048576,
	};
	uint64_t mask = hwMaxAddress();
	if (mask > 0xFFFFFFFFULL) {
		mask = 0xFFFFFFFFULL;
	}
	mask &= ~0xFFFULL;
	IOBufferMemoryDescriptor *src = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, kSelfTestBytes, mask);
	IOBufferMemoryDescriptor *dst = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, kSelfTestBytes, mask);
	IOMemoryMap *srcMap = NULL, *dstMap = NULL;
	if (src != NULL && dst != NULL) {
		src->performOperation(kIOMemoryIncoherentIOFlush, 0, kSelfTestBytes);
		dst->performOperation(kIOMemoryIncoherentIOFlush, 0, kSelfTestBytes);
		srcMap = src->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
		dstMap = dst->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache);
	}
	uint64_t srcBus = 0, dstBus = 0;
	bool ok = srcMap != NULL && dstMap != NULL &&
	    hwMemoryAddress(src->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &srcBus) &&
	    hwMemoryAddress(dst->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &dstBus);
	size_t used = (size_t)snprintf(out, outLen, "%s:", fName);
	if (!ok) {
		snprintf(out + used, outLen - used, " no memory for the test\n");
		OSSafeReleaseNULL(srcMap);
		OSSafeReleaseNULL(dstMap);
		OSSafeReleaseNULL(src);
		OSSafeReleaseNULL(dst);
		return false;
	}
	volatile uint8_t *s = (volatile uint8_t *)srcMap->getVirtualAddress();
	volatile uint8_t *d = (volatile uint8_t *)dstMap->getVirtualAddress();
	bool all = true, words = isQEMU();
	for (unsigned ci = 0; ci < fCount; ci++) {
		/* Each free channel in turn, held for the test (taken channels are skipped). */
		Channel *c = &fChannels[ci];
		IOReturn taken = fGate->runActionBlock(^IOReturn {
			if (!c->usable || c->allocated) {
				return kIOReturnBusy;
			}
			c->allocated = true;
			c->width = 1;
			return kIOReturnSuccess;
		});
		if (taken != kIOReturnSuccess) {
			continue;
		}
		unsigned good = 0, bad = 0;
		uint32_t seed = 0x2545f491U ^ c->number, maxLen = hwMaxLength(*c);
		for (unsigned i = 0; i < n; i++) {
			uint32_t len = kLengths[i % (sizeof(kLengths) / sizeof(kLengths[0]))];
			uint32_t so = 1 + i % 15, doff = 1 + (i * 7) % 15;
			if (words) {
				len = (len + 3) & ~3U;
				so = 4 * (1 + i % 3);
				doff = 4 * (1 + (i * 7) % 3);
			}
			if (len > maxLen) {
				len = maxLen & ~3U;
			}
			if (len > kSelfTestBytes - 32) {
				len = kSelfTestBytes - 32;
			}
			for (uint32_t k = 0; k < len; k++) {
				s[so + k] = (uint8_t)xorshift(&seed);
			}
			for (uint32_t k = 0; k < len + 32; k++) {
				d[k] = 0xa5;
			}
			IOReturn ret = startTransfer(*c, kDMAMemToMem, srcBus + so, dstBus + doff, len, src, dst);
			if (ret == kIOReturnSuccess) {
				ret = wait(*c, 1000ULL * 1000 * 1000);
			}
			bool same = ret == kIOReturnSuccess && d[doff - 1] == 0xa5 && d[doff + len] == 0xa5;
			for (uint32_t k = 0; same && k < len; k++) {
				same = d[doff + k] == s[so + k];
			}
			if (same) {
				good++;
			} else {
				if (bad == 0) {
					IOLog("PIEDMA: %s: channel %u: a copy of %u bytes (%u to %u) failed: 0x%x\n", fName,
					    c->number, len, so, doff, ret);
				}
				bad++;
			}
			if (bad != 0 || c->quarantined) { break; }
		}
		fGate->runActionBlock(^IOReturn {
			if (!c->quarantined) { c->allocated = false; }
			return kIOReturnSuccess;
		});
		if (used < outLen) {
			used += (size_t)snprintf(out + used, outLen - used, " %u:%u/%u", c->number, good, good + bad);
		}
		all &= bad == 0;
		if (bad != 0 || c->quarantined) { break; }
	}
	if (used < outLen) {
		snprintf(out + used, outLen - used, " %s\n", all ? "ok" : "FAILED");
	}
	srcMap->release();
	dstMap->release();
	src->release();
	dst->release();
	return all;
}

#pragma mark sysctls

static int
sysctl_channels(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req)
{
	char *text = (char *)IOMallocZeroData(kResultBytes);
	if (text == NULL) {
		return ENOMEM;
	}
	size_t used = 0;
	IOLockLock(sLock);
	for (unsigned i = 0; i < kMaxControllers; i++) {
		if (sControllers[i] != NULL && used < kResultBytes) {
			sControllers[i]->describe(text + used, kResultBytes - used);
			used = strnlen(text, kResultBytes);
		}
	}
	IOLockUnlock(sLock);
	int err = sysctl_handle_string(oidp, text, kResultBytes, req);
	IOFreeData(text, kResultBytes);
	return err;
}

static int
sysctl_selftest(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req)
{
	int n = 0;
	int err = sysctl_handle_int(oidp, &n, 0, req);
	if (err != 0 || req->newptr == USER_ADDR_NULL) {
		return err;
	}
	if (n < 1 || n > 10000) {
		return EINVAL;
	}
	char *text = (char *)IOMallocZeroData(kResultBytes);
	if (text == NULL) {
		return ENOMEM;
	}
	bool all = true;
	size_t used = 0;
	IOLockLock(sLock);
	for (unsigned i = 0; i < kMaxControllers; i++) {
		if (sControllers[i] != NULL && used < kResultBytes) {
			all &= sControllers[i]->selfTest((unsigned)n, text + used, kResultBytes - used);
			used = strnlen(text, kResultBytes);
			if (!all) { break; }
		}
	}
	strlcpy(sResult, text, sizeof(sResult));
	IOLockUnlock(sLock);
	IOFreeData(text, kResultBytes);
	return all ? 0 : EIO;
}

static int
sysctl_result(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req)
{
	char text[kResultBytes];
	IOLockLock(sLock);
	strlcpy(text, sResult, sizeof(text));
	IOLockUnlock(sLock);
	return sysctl_handle_string(oidp, text, sizeof(text), req);
}

SYSCTL_DECL(_debug);
SYSCTL_NODE(_debug, OID_AUTO, dma, CTLFLAG_RW | CTLFLAG_LOCKED, NULL, "DMA controllers");
SYSCTL_PROC(_debug_dma, OID_AUTO, channels, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    sysctl_channels, "A", "each controller's usable channels: number, * if taken, (transfers/errors)");
SYSCTL_PROC(_debug_dma, OID_AUTO, selftest, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0,
    sysctl_selftest, "I", "write N: N memory-to-memory copies on every free channel");
SYSCTL_PROC(_debug_dma, OID_AUTO, selftest_result, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    sysctl_result, "A", "the last self-test's copies right of those made, by channel");

static struct sysctl_oid *const sOIDs[] = {
	&sysctl__debug_dma, &sysctl__debug_dma_channels, &sysctl__debug_dma_selftest, &sysctl__debug_dma_selftest_result,
};

static void
registerSysctls(void)
{
	for (unsigned i = 0; i < sizeof(sOIDs) / sizeof(sOIDs[0]); i++) {
		sysctl_register_oid(sOIDs[i]);
	}
}

/* Leaves first, then the node; sysctl_unregister_oid waits for handlers that are running. */
static void
unregisterSysctls(void)
{
	for (unsigned i = sizeof(sOIDs) / sizeof(sOIDs[0]); i-- > 0;) {
		sysctl_unregister_oid(sOIDs[i]);
	}
}
