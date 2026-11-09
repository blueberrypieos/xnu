/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>

#include "RPiPlatform.h"

extern "C" {
#include <pexpert/pexpert.h>
}

#pragma mark PSCI

static bool psci_use_hvc;
static bool psci_ready;

static bool
compatible_with_psci(IORegistryEntry *entry)
{
	OSData *compat = OSDynamicCast(OSData, entry->getProperty("compatible"));
	if (!compat) {
		return false;
	}
	const char *s = (const char *)compat->getBytesNoCopy();
	const char *end = s + compat->getLength();
	while (s < end) {
		if (strncmp(s, "arm,psci", 8) == 0) {
			return true;
		}
		s += strnlen(s, (size_t)(end - s)) + 1;
	}
	return false;
}

bool
rpi_psci_init(void)
{
	if (psci_ready) {
		return true;
	}

	IORegistryEntry *armio = IORegistryEntry::fromPath("/arm-io", gIODTPlane);
	if (!armio) {
		return false;
	}

	IORegistryEntry *psci = NULL;
	OSIterator *iter = armio->getChildIterator(gIODTPlane);
	if (iter) {
		IORegistryEntry *child;
		while ((child = OSDynamicCast(IORegistryEntry, iter->getNextObject()))) {
			if (compatible_with_psci(child)) {
				psci = child;
				psci->retain();
				break;
			}
		}
		iter->release();
	}
	armio->release();
	if (!psci) {
		return false;
	}

	OSData *method = OSDynamicCast(OSData, psci->getProperty("method"));
	psci_use_hvc = method && method->getLength() >= 3 &&
	    strncmp((const char *)method->getBytesNoCopy(), "hvc", 3) == 0;
	psci->release();
	psci_ready = true;
	return true;
}

int64_t
rpi_psci_call(uint64_t fn, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
	register uint64_t x0 asm("x0") = fn;
	register uint64_t x1 asm("x1") = arg0;
	register uint64_t x2 asm("x2") = arg1;
	register uint64_t x3 asm("x3") = arg2;

	if (psci_use_hvc) {
		asm volatile ("hvc #0"
                    : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
                    :
                    : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12",
                    "x13", "x14", "x15", "x16", "x17", "memory");
	} else {
		asm volatile ("smc #0"
                    : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
                    :
                    : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12",
                    "x13", "x14", "x15", "x16", "x17", "memory");
	}
	return (int64_t)x0;
}

#pragma mark RPiPlatformExpert

#undef super
#define super IODTPlatformExpert
OSDefineMetaClassAndStructors(RPiPlatformExpert, IODTPlatformExpert);

bool
RPiPlatformExpert::start(IOService *provider)
{
	if (!rpi_psci_init()) {
		IOLog("RPiPlatformExpert: no PSCI node, restart and SMP unavailable\n");
	} else {
		int64_t version = rpi_psci_call(PSCI_0_2_FN_PSCI_VERSION, 0, 0, 0);
		IOLog("RPiPlatformExpert: PSCI %lld.%lld\n", (version >> 16) & 0xffff, version & 0xffff);
	}

	if (!super::start(provider)) {
		return false;
	}

	OSArray *rtc = RPiDTFindCompatible("raspberrypi,rpi-rtc");
	hasRTC = rtc != NULL;
	OSSafeReleaseNULL(rtc);
	publishResource("IORTC", this);

	registerService();
	return true;
}

enum { FW_GET_RTC_REG = 0x00030087, FW_SET_RTC_REG = 0x00038087, RTC_TIME = 0 };

long
RPiPlatformExpert::getGMTTimeOfDay(void)
{
	if (!hasRTC) {
		return 0;
	}
	IOService *mbox = waitForMatchingService(nameMatching("brcm,bcm2835-mbox"), 5ULL * kSecondScale);
	OSSafeReleaseNULL(mbox);
	uint32_t data[2] = { RTC_TIME, 0 };
	if (RPiFirmwareProperty(FW_GET_RTC_REG, data, sizeof(data)) != kIOReturnSuccess) {
		IOLog("RPiPlatformExpert: the firmware RTC does not answer\n");
		return 0;
	}
	IOLog("RPiPlatformExpert: RTC time %u\n", data[1]);
	return (long)data[1];
}

void
RPiPlatformExpert::setGMTTimeOfDay(long secs)
{
	if (hasRTC && secs > 0) {
		uint32_t data[2] = { RTC_TIME, (uint32_t)secs };
		RPiFirmwareProperty(FW_SET_RTC_REG, data, sizeof(data));
	}
}

const char *
RPiPlatformExpert::deleteList(void)
{
	return "('packages')";
}

const char *
RPiPlatformExpert::excludeList(void)
{
	return "('chosen', 'memory', 'options', 'cpus', 'defaults', 'aliases')";
}

int
RPiPlatformExpert::haltRestart(unsigned int type)
{
	if (type == kPEHaltCPU || type == kPEUPSDelayHaltCPU) {
		rpi_watchdog_halt();
	} else if (type == kPEPanicBegin) {
		rpi_watchdog_panic();
	}
	if (!psci_ready) {
		return super::haltRestart(type);
	}

	switch (type) {
	case kPEHaltCPU:
	case kPEUPSDelayHaltCPU:
		rpi_psci_call(PSCI_0_2_FN_SYSTEM_OFF, 0, 0, 0);
		break;
	case kPERestartCPU:
	case kPEPanicRestartCPU:
	case kPEPanicRestartCPUNoCallouts:
		rpi_psci_call(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0);
		break;
	default:
		return super::haltRestart(type);
	}
	return -1;
}

#pragma mark RPiIO

#undef super
#define super IOService
OSDefineMetaClassAndStructors(RPiIO, IOService);

static bool
dt_node_disabled(IORegistryEntry *entry)
{
	OSData *status = OSDynamicCast(OSData, entry->getProperty("status"));
	if (status == NULL) {
		return false;
	}
	const char *s = (const char *)status->getBytesNoCopy();
	size_t len = strnlen(s, status->getLength());
	return !((len == 4 && !strncmp(s, "okay", 4)) || (len == 2 && !strncmp(s, "ok", 2)));
}

bool
RPiIO::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}

	RPiPublishDTChildren(this, provider);
	registerService();
	return true;
}

void
RPiPublishDTChildren(IOService *parent, IORegistryEntry *dtNode)
{
	OSIterator *iter = IODTFindMatchingEntries(dtNode, kIODTExclusive, NULL);
	if (iter) {
		IORegistryEntry *entry;
		while ((entry = OSDynamicCast(IORegistryEntry, iter->getNextObject()))) {
			if (dt_node_disabled(entry)) {
				continue;
			}
			RPiIODevice *nub = OSTypeAlloc(RPiIODevice);
			if (!nub) {
				continue;
			}
			if (nub->init(entry, gIODTPlane)) {
				nub->attach(parent);
				nub->registerService();
				if (IODTMatchNubWithKeys(nub, "'brcm,bcm2835-mbox'")) {
					RPiIO::logFirmware();
				}
				if (IODTMatchNubWithKeys(nub, "('brcm,bcm2835-pm-wdt', 'brcm,bcm2711-pm', 'brcm,bcm2712-pm')")) {
					rpi_watchdog_early_start(nub);
				}
			}
			nub->release();
		}
		iter->release();
	}
}

void
RPiIO::logFirmware(void)
{
	uint32_t fw = 0, board = 0;
	if (RPiFirmwareProperty(0x00000001, &fw, sizeof(fw)) == kIOReturnSuccess &&
	    RPiFirmwareProperty(0x00010002, &board, sizeof(board)) == kIOReturnSuccess) {
		IOLog("RPi: firmware revision %u, board revision 0x%x\n", fw, board);
	}
}

#pragma mark RPiIODevice

#undef super
#define super IOPlatformDevice
OSDefineMetaClassAndStructors(RPiIODevice, IOPlatformDevice);

bool
RPiIODevice::compareName(OSString *name, OSString **matched) const
{
	return IODTCompareNubName(this, name, matched) ||
	       IOService::compareName(name, matched);
}

IOReturn
RPiIODevice::getResources(void)
{
	if (!getDeviceMemory()) {
		IODTResolveAddressing(this, "reg", NULL);
	}
	return kIOReturnSuccess;
}

#pragma mark Device tree

OSArray *
RPiDTFindCompatible(const char *compat)
{
	OSString *key = OSString::withCString(compat);
	IORegistryIterator *iter = IORegistryIterator::iterateOver(IORegistryEntry::getRegistryRoot(), gIODTPlane,
	    kIORegistryIterateRecursively);
	OSArray *found = NULL;
	if (key != NULL && iter != NULL) {
		do {
			OSSafeReleaseNULL(found);
			found = OSArray::withCapacity(2);
			iter->reset();
			IORegistryEntry *entry;
			while (found != NULL && (entry = iter->getNextObject()) != NULL) {
				if (IODTCompareNubName(entry, key, NULL)) {
					found->setObject(entry);
				}
			}
		} while (found != NULL && !iter->isValid());
	}
	OSSafeReleaseNULL(iter);
	OSSafeReleaseNULL(key);
	if (found != NULL && found->getCount() == 0) {
		OSSafeReleaseNULL(found);
	}
	return found;
}

#pragma mark DMA addresses

bool
RPiDMAAddress(IORegistryEntry *device, uint64_t pa, uint64_t *bus)
{
	OSData *ranges = device != NULL ? OSDynamicCast(OSData, device->getProperty("pieboot,dma-ranges")) : NULL;
	if (ranges == NULL || ranges->getLength() < 3 * sizeof(uint64_t)) {
		*bus = pa;
		return true;
	}
	const uint64_t *r = (const uint64_t *)ranges->getBytesNoCopy();
	for (unsigned i = 0; i + 3 <= ranges->getLength() / sizeof(uint64_t); i += 3) {
		uint64_t cpu = r[i], base = r[i + 1], size = r[i + 2];
		if (pa >= cpu && pa - cpu < size) {
			*bus = base + (pa - cpu);
			return true;
		}
	}
	return false;
}

bool
RPiCPUAddress(IORegistryEntry *device, uint64_t bus, uint64_t *pa)
{
	OSData *ranges = device != NULL ? OSDynamicCast(OSData, device->getProperty("pieboot,dma-ranges")) : NULL;
	if (ranges == NULL || ranges->getLength() < 3 * sizeof(uint64_t)) {
		*pa = bus;
		return true;
	}
	const uint64_t *r = (const uint64_t *)ranges->getBytesNoCopy();
	for (unsigned i = 0; i + 3 <= ranges->getLength() / sizeof(uint64_t); i += 3) {
		uint64_t cpu = r[i], base = r[i + 1], size = r[i + 2];
		if (bus >= base && bus - base < size) {
			*pa = cpu + (bus - base);
			return true;
		}
	}
	return false;
}

#pragma mark Firmware mailbox

enum {
	MBOX_READ = 0x00, MBOX_STATUS0 = 0x18, MBOX_WRITE = 0x20, MBOX_STATUS1 = 0x38,
	MBOX_FULL = 0x80000000, MBOX_EMPTY = 0x40000000, MBOX_CH_PROPERTY = 8,
	MBOX_RESPONSE_OK = 0x80000000,
};

IOReturn
RPiFirmwareProperty(uint32_t tag, void *data, uint32_t length)
{
	static IOLock *lock;
	static IOMemoryMap *regsMap;
	static IOBufferMemoryDescriptor *msgMem;
	static uint32_t msgBus;

	if (lock == NULL) {
		IOLock *l = IOLockAlloc();
		if (l == NULL || !OSCompareAndSwapPtr(NULL, l, &lock)) {
			if (l != NULL) {
				IOLockFree(l);
			}
		}
	}
	if (length > 1024) {
		return kIOReturnBadArgument;
	}
	IOLockLock(lock);
	if (regsMap == NULL) {
		IOService *mbox = IOService::copyMatchingService(IOService::nameMatching("brcm,bcm2835-mbox"));
		msgMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryPhysicallyContiguous, 4096, 0x3ffff000ULL);
		uint64_t bus = 0;
		if (mbox != NULL && msgMem != NULL &&
		    RPiDMAAddress(mbox, msgMem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone), &bus) &&
		    bus < (1ULL << 32)) {
			mbox->getResources();
			regsMap = mbox->mapDeviceMemoryWithIndex(0);
			msgBus = (uint32_t)bus;
		}
		OSSafeReleaseNULL(mbox);
	}
	if (regsMap == NULL || msgMem == NULL) {
		IOLockUnlock(lock);
		return kIOReturnNotFound;
	}
	uintptr_t regs = (uintptr_t)regsMap->getVirtualAddress();
	uint32_t *msg = (uint32_t *)msgMem->getBytesNoCopy();
	uint32_t words = (length + 3) / 4;
	uint32_t total = 6 + words;
	bzero(msg, total * 4);
	msg[0] = total * 4;
	msg[1] = 0;
	msg[2] = tag;
	msg[3] = words * 4;
	msg[4] = length;
	bcopy(data, &msg[5], length);
	msg[5 + words] = 0;
	msgMem->performOperation(kIOMemoryIncoherentIOFlush, 0, total * 4);

	uint32_t word = msgBus | MBOX_CH_PROPERTY;
	IOReturn ret = kIOReturnTimeout;
	for (int i = 0; i < 100000 && (*(volatile uint32_t *)(regs + MBOX_STATUS1) & MBOX_FULL); i++) {
		IODelay(10);
	}
	*(volatile uint32_t *)(regs + MBOX_WRITE) = word;
	for (int i = 0; i < 100000; i++) {
		if (*(volatile uint32_t *)(regs + MBOX_STATUS0) & MBOX_EMPTY) {
			IODelay(10);
			continue;
		}
		if (*(volatile uint32_t *)(regs + MBOX_READ) == word) {
			msgMem->performOperation(kIOMemoryIncoherentIOFlush, 0, total * 4);
			ret = msg[1] == MBOX_RESPONSE_OK ? kIOReturnSuccess : kIOReturnError;
			if (ret == kIOReturnSuccess && !(msg[4] & 0x80000000U)) {
				IOLog("RPiFirmwareProperty: tag 0x%08x not answered (0x%08x)\n", tag, msg[4]);
			}
			bcopy(&msg[5], data, length);
			break;
		}
	}
	IOLockUnlock(lock);
	return ret;
}
