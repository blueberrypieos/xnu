/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOMessage.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <kern/clock.h>
#include <mach/mach_time.h>
#include <os/atomic_private.h>
#include <pexpert/pexpert.h>

#include "PIESDCard.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#include <IOKit/storage/IOStorageDeviceCharacteristics.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>
#pragma clang diagnostic pop

#define super IOBlockStorageDevice
OSDefineMetaClassAndAbstractStructors(PIESDCard, IOBlockStorageDevice);

/* Bits hi-lo of a 136-bit response, as sdCommand gives them. */
static uint32_t
r2_bits(const uint32_t resp[4], unsigned hi, unsigned lo)
{
	uint32_t v = 0;
	for (unsigned bit = hi; ; bit--) {
		v = (v << 1) | ((resp[bit / 32] >> (bit % 32)) & 1);
		if (bit == lo) {
			break;
		}
	}
	return v;
}

IOReturn
PIESDCard::sdAppCommand(uint32_t index, uint32_t arg, SDResponse type, uint32_t resp[4])
{
	uint32_t r[4];
	IOReturn ret = sdCommand(55, sdRCA << 16, kSDRespR1, r);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	return sdCommand(index, arg, type, resp);
}

IOReturn
PIESDCard::sdTransferBlocks(uint64_t block, uint32_t count, bool write)
{
	uint32_t arg = (uint32_t)(sdHighCapacity ? block : block * kSDBlockSize);
	uint32_t index = write ? (count > 1 ? 25 : 24) : (count > 1 ? 18 : 17);
	return sdTransfer(index, arg, count, write);
}

bool
PIESDCard::sdInitCard(void)
{
	uint32_t resp[4];

	if (!sdPowerUp() || !sdSetClock(kSDClockInitHz)) {
		return false;
	}
	sdCommand(0, 0, kSDRespNone, NULL);
	IODelay(1000);
	bool v2 = sdCommand(8, 0x1AA, kSDRespR1, resp) == kIOReturnSuccess && (resp[0] & 0xFFF) == 0x1AA;
	sdRCA = 0;
	uint32_t ocr = 0;
	for (int i = 0; i < 100; i++) {
		if (sdAppCommand(41, 0x00FF8000U | (v2 ? 0x40000000U : 0), kSDRespR3, resp) != kIOReturnSuccess) {
			IOLog("%s: no SD card answers\n", getName());
			return false;
		}
		ocr = resp[0];
		if (ocr & 0x80000000U) {
			break;
		}
		IOSleep(10);
	}
	if (!(ocr & 0x80000000U)) {
		IOLog("%s: card did not power up (OCR 0x%08x)\n", getName(), ocr);
		return false;
	}
	sdHighCapacity = (ocr & 0x40000000U) != 0;

	if (sdCommand(2, 0, kSDRespR2, resp) != kIOReturnSuccess) {
		return false;
	}
	sdVendor[0] = (char)r2_bits(resp, 119, 112);
	sdVendor[1] = (char)r2_bits(resp, 111, 104);
	sdVendor[2] = '\0';
	for (int i = 0; i < 5; i++) {
		sdProduct[i] = (char)r2_bits(resp, 103 - 8 * i, 96 - 8 * i);
	}
	sdProduct[5] = '\0';
	for (char *c = sdVendor; *c != '\0'; c++) {
		*c = *c >= ' ' && *c <= '~' ? *c : '?';
	}
	for (char *c = sdProduct; *c != '\0'; c++) {
		*c = *c >= ' ' && *c <= '~' ? *c : '?';
	}
	uint32_t prv = r2_bits(resp, 63, 56);
	snprintf(sdRevision, sizeof(sdRevision), "%u.%u", prv >> 4, prv & 0xF);

	if (sdCommand(3, 0, kSDRespR1, resp) != kIOReturnSuccess) {
		return false;
	}
	sdRCA = resp[0] >> 16;

	if (sdCommand(9, sdRCA << 16, kSDRespR2, resp) != kIOReturnSuccess) {
		return false;
	}
	if (r2_bits(resp, 127, 126) == 1) {
		sdBlocks = ((uint64_t)r2_bits(resp, 69, 48) + 1) * 1024;
	} else {
		uint32_t c_size = r2_bits(resp, 73, 62);
		uint32_t mult = r2_bits(resp, 49, 47);
		uint32_t bl_len = r2_bits(resp, 83, 80);
		sdBlocks = ((uint64_t)(c_size + 1) << (mult + 2)) << bl_len >> 9;
	}

	if (sdCommand(7, sdRCA << 16, kSDRespR1b, resp) != kIOReturnSuccess) {
		return false;
	}
	if (!sdSetClock(kSDClockRunHz)) {
		return false;
	}
	if (sdAppCommand(6, 2, kSDRespR1, resp) == kIOReturnSuccess) {
		sdSetBusWidth4();
	}
	if (!sdHighCapacity) {
		sdCommand(16, kSDBlockSize, kSDRespR1, resp);
	}
	return true;
}

bool
PIESDCard::sdReadReference(uint8_t *reference)
{
	if (sdStopping() || sdBlocks < kSDCheckBlocks || sdTransferBlocks(0, kSDCheckBlocks, false) != kIOReturnSuccess) {
		return false;
	}
	bcopy(sdBounce, reference, kSDCheckBlocks * kSDBlockSize);
	return true;
}

bool
PIESDCard::sdSameAs(const uint8_t *reference, const char *what)
{
	if (sdStopping()) {
		return false;
	}
	memset(sdBounce, 0xA5, kSDCheckBlocks * kSDBlockSize);
	IOReturn ret = sdTransferBlocks(0, kSDCheckBlocks, false);
	if (ret == kIOReturnSuccess && memcmp(sdBounce, reference, kSDCheckBlocks * kSDBlockSize) == 0) {
		return true;
	}
	if (!sdStopping()) {
		IOLog("%s: %s reads the card %s; left out\n", getName(), what,
		    ret == kIOReturnSuccess ? "wrongly" : "with errors");
	}
	return false;
}

/*
 * CMD6 to high speed, kept only if the card then reads as it did. Once the
 * stop is set, no further step reaches the card or changes the host's clock
 * or timing, nothing more is logged, and the host's setup ends.
 */
void
PIESDCard::sdSetUpHighSpeed(const uint8_t *reference, const char *bootArg)
{
	uint32_t on = 1;
	if (PE_parse_boot_argn(bootArg, &on, sizeof(on)) && on == 0) {
		return;
	}
	uint8_t status[64];
	if (sdStopping() || sdReadSmall(6, 0x00FFFFF1, status, sizeof(status)) != kIOReturnSuccess ||
	    !(status[13] & 0x02) || sdStopping()) {
		return;
	}
	if (sdReadSmall(6, 0x80FFFFF1, status, sizeof(status)) != kIOReturnSuccess || (status[16] & 0xF) != 1) {
		if (!sdStopping()) {
			IOLog("%s: the card did not switch to high speed\n", getName());
		}
		return;
	}
	bool fast = !sdStopping() && sdHighSpeedTiming(true) && !sdStopping() && sdSetClock(kSDClockHighSpeedHz) &&
	    sdSameAs(reference, "high speed");
	if (fast || sdStopping()) {
		return;
	}
	sdSetClock(kSDClockRunHz);
	if (sdStopping()) {
		return;
	}
	sdHighSpeedTiming(false);
	if (!sdStopping()) {
		sdReadSmall(6, 0x80FFFFF0, status, sizeof(status));
	}
}

bool
PIESDCard::sdStart(void)
{
	if (sdLock == NULL) {
		sdLock = IOLockAlloc();
	}
	if (sdLock == NULL || sdBounce == NULL) {
		return false;
	}
	IOLockLock(sdLock);
	sdPresent = !sdStopping() && sdInitCard() && !sdStopping();
	IOLockUnlock(sdLock);
	if (sdPresent) {
		IOLog("%s: %s %s rev %s, %llu blocks (%llu MB)%s\n", getName(), sdVendor, sdProduct, sdRevision,
		    sdBlocks, sdBlocks / 2048, sdHighCapacity ? ", high capacity" : "");
	}
	return sdPresent;
}

void
PIESDCard::sdPublish(const char *location)
{
	OSDictionary *proto = OSDictionary::withCapacity(2);
	if (proto != NULL) {
		OSString *s = OSString::withCString("SD");
		proto->setObject(kIOPropertyPhysicalInterconnectTypeKey, s);
		s->release();
		s = OSString::withCString(location);
		proto->setObject(kIOPropertyPhysicalInterconnectLocationKey, s);
		s->release();
		setProperty(kIOPropertyProtocolCharacteristicsKey, proto);
		proto->release();
	}
	setProperty(kIOMaximumBlockCountReadKey, kSDMaxBlocks, 64);
	setProperty(kIOMaximumBlockCountWriteKey, kSDMaxBlocks, 64);
	if (sdHaltNotifier == NULL) {
		sdHaltNotifier = registerPrioritySleepWakeInterest(&PIESDCard::sdHaltRestart, this);
		if (sdHaltNotifier == NULL) {
			IOLog("%s: no notice of halt or restart; I/O goes on through them\n", getName());
		}
	}
	registerService();
}

/*
 * Every card's stop: set at the first halt or restart notice, with the one
 * deadline that every card's transfer under way is waited for against.
 */
static bool sSDStopping;
static uint64_t sSDStopDeadline;

bool
PIESDCard::sdStopping(void)
{
	return os_atomic_load(&sSDStopping, relaxed);
}

/*
 * A card's setup steps that reach the card check the stop under sdLock
 * first; a transfer's own recovery does not. Before it publishes the card,
 * and where a step fails, the setup asks this: true once the stop is set,
 * and the card is then not published. The first such call logs it.
 */
bool
PIESDCard::sdSetupStopped(void)
{
	if (!sdStopping()) {
		return false;
	}
	if (!os_atomic_xchg(&sdSetupHalted, true, relaxed)) {
		IOLog("%s: not set up: the system is halting or restarting\n", getName());
	}
	return true;
}

/*
 * No new I/O on any card: the first notice sets the deadline and the stop,
 * which a transfer under way sees between its chunks, and every later
 * request at once. Then this card's transfer under way, which holds sdLock,
 * is waited for until the deadline, once per card.
 */
void
PIESDCard::sdStop(const char *why)
{
	uint64_t deadline;
	clock_interval_to_deadline(kSDStopWaitMs, kMillisecondScale, &deadline);
	os_atomic_cmpxchg(&sSDStopDeadline, 0, deadline, relaxed);
	os_atomic_store(&sSDStopping, true, relaxed);
	if (os_atomic_xchg(&sdStopDone, true, relaxed)) {
		return;
	}
	deadline = os_atomic_load(&sSDStopDeadline, relaxed);
	bool idle;
	while (!(idle = IOLockTryLock(sdLock)) && mach_absolute_time() < deadline) {
		IOSleep(10);
	}
	if (idle) {
		IOLockUnlock(sdLock);
	}
	IOLog("%s: no more I/O%s%s\n", getName(), why, idle ? "" : "; a transfer still runs at the 8 s deadline");
}

IOReturn
PIESDCard::sdHaltRestart(void *target, void *refCon, UInt32 messageType, IOService *provider, void *argument,
    vm_size_t size)
{
	(void)refCon;
	(void)provider;
	(void)argument;
	(void)size;
	if (messageType == kIOMessageSystemWillPowerOff) {
		((PIESDCard *)target)->sdStop(": the system is halting");
	} else if (messageType == kIOMessageSystemWillRestart) {
		((PIESDCard *)target)->sdStop(": the system is restarting");
	}
	return kIOReturnSuccess;
}

void
PIESDCard::free(void)
{
	if (sdHaltNotifier != NULL) {
		sdHaltNotifier->remove();
		sdHaltNotifier = NULL;
	}
	if (sdLock != NULL) {
		IOLockFree(sdLock);
	}
	super::free();
}

#pragma mark IOBlockStorageDevice

IOReturn
PIESDCard::doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 nblks,
    IOStorageAttributes *attributes, IOStorageCompletion *completion)
{
	(void)attributes;
	bool write = buffer->getDirection() == kIODirectionOut;
	IOReturn ret = kIOReturnSuccess;

	if (block + nblks > sdBlocks) {
		ret = kIOReturnBadArgument;
	} else if (write && sdReadOnly) {
		ret = kIOReturnNotWritable;
	}
	IOLockLock(sdLock);
	for (UInt64 done = 0; ret == kIOReturnSuccess && done < nblks;) {
		if (sdStopping()) {
			ret = kIOReturnOffline;
			break;
		}
		uint32_t n = (uint32_t)(nblks - done < kSDMaxBlocks ? nblks - done : kSDMaxBlocks);
		IOByteCount offset = done * kSDBlockSize, length = (IOByteCount)n * kSDBlockSize;
		if (write) {
			if (buffer->readBytes(offset, sdBounce, length) != length) {
				ret = kIOReturnIOError;
				break;
			}
			ret = sdTransferBlocks(block + done, n, true);
		} else {
			ret = sdTransferBlocks(block + done, n, false);
			if (ret == kIOReturnSuccess && buffer->writeBytes(offset, sdBounce, length) != length) {
				ret = kIOReturnIOError;
			}
		}
		done += n;
	}
	IOLockUnlock(sdLock);

	IOStorage::complete(completion, ret, ret == kIOReturnSuccess ? nblks * kSDBlockSize : 0);
	return kIOReturnSuccess;
}

IOReturn
PIESDCard::doSynchronize(UInt64 block, UInt64 nblks, IOStorageSynchronizeOptions options)
{
	(void)block;
	(void)nblks;
	(void)options;
	return kIOReturnSuccess;
}

IOReturn PIESDCard::doEjectMedia(void) { return kIOReturnUnsupported; }
IOReturn PIESDCard::doFormatMedia(UInt64 byteCapacity) { (void)byteCapacity; return kIOReturnUnsupported; }

UInt32
PIESDCard::doGetFormatCapacities(UInt64 *capacities, UInt32 capacitiesMaxCount) const
{
	if (capacities != NULL && capacitiesMaxCount > 0) {
		capacities[0] = sdBlocks * kSDBlockSize;
	}
	return 1;
}

char *PIESDCard::getVendorString(void) { return sdVendor; }
char *PIESDCard::getProductString(void) { return sdProduct; }
char *PIESDCard::getRevisionString(void) { return sdRevision; }
char *PIESDCard::getAdditionalDeviceInfoString(void) { return (char *)"SD card"; }

IOReturn PIESDCard::reportBlockSize(UInt64 *blockSize) { *blockSize = kSDBlockSize; return kIOReturnSuccess; }
IOReturn PIESDCard::reportEjectability(bool *isEjectable) { *isEjectable = false; return kIOReturnSuccess; }
IOReturn PIESDCard::reportMaxValidBlock(UInt64 *maxBlock) { *maxBlock = sdBlocks - 1; return kIOReturnSuccess; }

IOReturn
PIESDCard::reportMediaState(bool *mediaPresent, bool *changedState)
{
	*mediaPresent = sdPresent;
	if (changedState != NULL) {
		*changedState = false;
	}
	return kIOReturnSuccess;
}

IOReturn PIESDCard::reportRemovability(bool *isRemovable) { *isRemovable = false; return kIOReturnSuccess; }

IOReturn
PIESDCard::reportWriteProtection(bool *isWriteProtected)
{
	*isWriteProtected = sdReadOnly;
	return kIOReturnSuccess;
}
