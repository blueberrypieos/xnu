/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_SDMMC_PIESDCARD_H
#define _IOKIT_SDMMC_PIESDCARD_H

#include <IOKit/IOLib.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#include <IOKit/storage/IOBlockStorageDevice.h>
#pragma clang diagnostic pop

/*
 * An SD memory card behind a host controller: the card's protocol, the
 * block device and the checks of faster modes. A host driver subclasses
 * it and supplies the sd* operations. From the system's first halt or
 * restart notice on, no card takes new I/O: a transfer under way ends its
 * current chunk, and the rest of it, and every later request, fail with
 * kIOReturnOffline. Each card's notice waits for its transfer under way
 * until one deadline, kSDStopWaitMs after the first notice, for all cards
 * together. A transfer still running then is logged, and the halt or
 * restart goes on. A card still being set up makes no further setup step
 * that reaches it, and is not published.
 */
class PIESDCard : public IOBlockStorageDevice
{
	OSDeclareAbstractStructors(PIESDCard);

protected:
	enum SDResponse { kSDRespNone, kSDRespR1, kSDRespR1b, kSDRespR2, kSDRespR3 };
	enum {
		kSDBlockSize   = 512,
		kSDMaxBlocks   = 128,
		kSDBounceBytes = kSDMaxBlocks * kSDBlockSize,
		kSDCheckBlocks = 8,
	};
	enum {
		kSDClockInitHz      = 400000,
		kSDClockRunHz       = 25000000,
		kSDClockHighSpeedHz = 50000000,
	};
	enum { kSDStopWaitMs = 8000 };

	IOLock *sdLock;
	uint32_t *sdBounce;
	uint32_t sdClockHz;
	bool sdPresent;
	bool sdHighCapacity;
	bool sdReadOnly;
	uint32_t sdRCA;
	uint64_t sdBlocks;
	char sdVendor[16];
	char sdProduct[16];
	char sdRevision[8];
	IONotifier *sdHaltNotifier;
	bool sdStopDone;
	bool sdSetupHalted;

	/*
	 * The host's operations, called with sdLock held. A long response
	 * gives the register's bits 127-8 at the same bit positions of resp[]
	 * (resp[3] bit 31 is bit 127); a short one gives bits 39-8 in resp[0].
	 */
	virtual bool sdPowerUp(void) = 0;
	virtual bool sdSetClock(uint32_t hz) = 0;
	virtual void sdSetBusWidth4(void) = 0;
	virtual bool sdHighSpeedTiming(bool on) = 0;
	virtual IOReturn sdCommand(uint32_t index, uint32_t arg, SDResponse type, uint32_t resp[4]) = 0;
	virtual IOReturn sdReadSmall(uint32_t index, uint32_t arg, uint8_t *data, uint32_t bytes) = 0;
	virtual IOReturn sdTransfer(uint32_t index, uint32_t arg, uint32_t count, bool write) = 0;

	/* Sets the card up and logs it; a host may call it again. */
	bool sdStart(void);
	bool sdInitCard(void);
	IOReturn sdAppCommand(uint32_t index, uint32_t arg, SDResponse type, uint32_t resp[4]);
	IOReturn sdTransferBlocks(uint64_t block, uint32_t count, bool write);
	bool sdReadReference(uint8_t *reference);
	bool sdSameAs(const uint8_t *reference, const char *what);
	void sdSetUpHighSpeed(const uint8_t *reference, const char *bootArg);
	void sdPublish(const char *location);
	static bool sdStopping(void);
	bool sdSetupStopped(void);
	void sdStop(const char *why);
	static IOReturn sdHaltRestart(void *target, void *refCon, UInt32 messageType, IOService *provider, void *argument,
	    vm_size_t size);

public:
	virtual void free(void) APPLE_KEXT_OVERRIDE;

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

#endif
