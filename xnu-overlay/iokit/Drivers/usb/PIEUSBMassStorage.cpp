/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <libkern/OSByteOrder.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#include <IOKit/storage/IOBlockStorageDevice.h>
#include <IOKit/storage/IOStorageDeviceCharacteristics.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>
#pragma clang diagnostic pop

#include "PIEUSB.h"

#define CBW_SIGNATURE   0x43425355
#define CSW_SIGNATURE   0x53425355
#define CBW_LENGTH      31
#define CSW_LENGTH      13
#define BOUNCE_SIZE     65536

enum {
	SCSI_TEST_UNIT_READY = 0x00, SCSI_REQUEST_SENSE = 0x03, SCSI_INQUIRY = 0x12,
	SCSI_MODE_SENSE_6 = 0x1a, SCSI_START_STOP_UNIT = 0x1b, SCSI_READ_CAPACITY_10 = 0x25,
	SCSI_READ_10 = 0x28, SCSI_WRITE_10 = 0x2a, SCSI_SYNCHRONIZE_CACHE_10 = 0x35,
	SCSI_READ_16 = 0x88, SCSI_WRITE_16 = 0x8a, SCSI_SERVICE_ACTION_IN_16 = 0x9e,
	SCSI_READ_CAPACITY_16 = 0x10,

	SENSE_NOT_READY = 0x2, SENSE_ILLEGAL_REQUEST = 0x5, SENSE_UNIT_ATTENTION = 0x6,
	ASC_MEDIUM_NOT_PRESENT = 0x3a,

	BOT_RESET = 0xff, BOT_GET_MAX_LUN = 0xfe,
};

class PIEUSBMassStorage : public IOBlockStorageDevice
{
	OSDeclareDefaultStructors(PIEUSBMassStorage);

	PIEUSBInterface *intf;
	uint8_t bulkIn, bulkOut;
	IOLock *lock;
	uint8_t *bounce;
	uint32_t tag;
	uint8_t senseKey, senseASC;
	uint32_t blockSize;
	uint64_t blocks;
	bool writeProtected;
	char vendor[9];
	char product[17];
	char revision[5];

	void resetRecovery(void);
	IOReturn transport(const uint8_t *cdb, uint8_t cdbLength, bool in, uint32_t length, uint32_t *moved);
	IOReturn scsi(const uint8_t *cdb, uint8_t cdbLength, bool in, uint32_t length, uint32_t *moved = NULL);
	IOReturn waitReady(void);
	IOReturn readCapacity(void);
	IOReturn readWrite(uint64_t block, uint32_t count, bool write);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
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

#define super IOBlockStorageDevice
OSDefineMetaClassAndStructors(PIEUSBMassStorage, IOBlockStorageDevice);

#pragma mark Bulk-only transport

void
PIEUSBMassStorage::resetRecovery(void)
{
	intf->controlRequest(0x21, BOT_RESET, 0, intf->number, 0, NULL);
	intf->clearHalt(bulkIn);
	intf->clearHalt(bulkOut);
}

IOReturn
PIEUSBMassStorage::transport(const uint8_t *cdb, uint8_t cdbLength, bool in, uint32_t length, uint32_t *moved)
{
	uint8_t cbw[CBW_LENGTH] = { 0 };
	uint32_t thisTag = ++tag;
	OSWriteLittleInt32(cbw, 0, CBW_SIGNATURE);
	OSWriteLittleInt32(cbw, 4, thisTag);
	OSWriteLittleInt32(cbw, 8, length);
	cbw[12] = in ? 0x80 : 0;
	cbw[13] = 0;
	cbw[14] = cdbLength;
	bcopy(cdb, cbw + 15, cdbLength);
	uint32_t actual = 0;
	IOReturn ret = intf->bulkTransfer(bulkOut, cbw, CBW_LENGTH, &actual);
	if (ret != kIOReturnSuccess || actual != CBW_LENGTH) {
		resetRecovery();
		return ret != kIOReturnSuccess ? ret : kIOReturnIOError;
	}

	uint32_t got = 0;
	if (length > 0) {
		ret = intf->bulkTransfer(in ? bulkIn : bulkOut, bounce, length, &got);
		if (ret == kIOReturnNotResponding) {
			intf->clearHalt(in ? bulkIn : bulkOut);
		} else if (ret != kIOReturnSuccess) {
			resetRecovery();
			return ret;
		}
	}

	uint8_t csw[CSW_LENGTH];
	ret = intf->bulkTransfer(bulkIn, csw, CSW_LENGTH, &actual);
	if (ret == kIOReturnNotResponding) {
		intf->clearHalt(bulkIn);
		ret = intf->bulkTransfer(bulkIn, csw, CSW_LENGTH, &actual);
	}
	if (ret != kIOReturnSuccess || actual != CSW_LENGTH ||
	    OSReadLittleInt32(csw, 0) != CSW_SIGNATURE || OSReadLittleInt32(csw, 4) != thisTag || csw[12] > 1) {
		IOLog("USB storage: command 0x%02x: no valid status (0x%x), resetting\n", cdb[0], ret);
		resetRecovery();
		return ret != kIOReturnSuccess ? ret : kIOReturnIOError;
	}
	if (moved != NULL) {
		*moved = got;
	}
	return csw[12] == 0 ? kIOReturnSuccess : kIOReturnIOError;
}

IOReturn
PIEUSBMassStorage::scsi(const uint8_t *cdb, uint8_t cdbLength, bool in, uint32_t length, uint32_t *moved)
{
	IOReturn ret = transport(cdb, cdbLength, in, length, moved);
	if (ret != kIOReturnIOError) {
		return ret;
	}
	uint8_t rs[6] = { SCSI_REQUEST_SENSE, 0, 0, 0, 18, 0 };
	uint8_t sense[18];
	senseKey = senseASC = 0;
	if (transport(rs, sizeof(rs), true, sizeof(sense), NULL) == kIOReturnSuccess) {
		bcopy(bounce, sense, sizeof(sense));
		senseKey = sense[2] & 0xf;
		senseASC = sense[12];
	}
	return kIOReturnIOError;
}

#pragma mark The disk

IOReturn
PIEUSBMassStorage::waitReady(void)
{
	static const uint8_t tur[6] = { SCSI_TEST_UNIT_READY };
	IOReturn ret = kIOReturnNotReady;
	for (int tries = 0; tries < 50; tries++) {
		ret = scsi(tur, sizeof(tur), false, 0);
		if (ret == kIOReturnSuccess) {
			return ret;
		}
		if (ret == kIOReturnIOError && senseKey == SENSE_NOT_READY && senseASC == ASC_MEDIUM_NOT_PRESENT) {
			return kIOReturnNoMedia;
		}
		IOSleep(100);
	}
	return ret;
}

IOReturn
PIEUSBMassStorage::readCapacity(void)
{
	uint8_t rc10[10] = { SCSI_READ_CAPACITY_10 };
	IOReturn ret = scsi(rc10, sizeof(rc10), true, 8);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint64_t last = OSReadBigInt32(bounce, 0);
	blockSize = OSReadBigInt32(bounce, 4);
	if (last == 0xffffffff) {
		uint8_t rc16[16] = { SCSI_SERVICE_ACTION_IN_16, SCSI_READ_CAPACITY_16 };
		OSWriteBigInt32(rc16, 10, 32);
		ret = scsi(rc16, sizeof(rc16), true, 32);
		if (ret != kIOReturnSuccess) {
			return ret;
		}
		last = OSReadBigInt64(bounce, 0);
		blockSize = OSReadBigInt32(bounce, 8);
	}
	blocks = last + 1;
	return kIOReturnSuccess;
}

IOReturn
PIEUSBMassStorage::readWrite(uint64_t block, uint32_t count, bool write)
{
	uint8_t cdb[16] = { 0 };
	uint8_t length;
	if (block + count <= 0xffffffffULL) {
		cdb[0] = write ? SCSI_WRITE_10 : SCSI_READ_10;
		OSWriteBigInt32(cdb, 2, (uint32_t)block);
		OSWriteBigInt16(cdb, 7, (uint16_t)count);
		length = 10;
	} else {
		cdb[0] = write ? SCSI_WRITE_16 : SCSI_READ_16;
		OSWriteBigInt64(cdb, 2, block);
		OSWriteBigInt32(cdb, 10, count);
		length = 16;
	}
	uint32_t bytes = count * blockSize, moved = 0;
	IOReturn ret = scsi(cdb, length, !write, bytes, &moved);
	if (ret == kIOReturnIOError && senseKey == SENSE_UNIT_ATTENTION) {
		ret = scsi(cdb, length, !write, bytes, &moved);
	}
	if (ret == kIOReturnSuccess && moved != bytes) {
		ret = kIOReturnUnderrun;
	}
	return ret;
}

static void
scsi_string(char *out, const uint8_t *in, size_t length)
{
	size_t n = 0;
	for (size_t i = 0; i < length; i++) {
		out[n++] = (in[i] >= 0x20 && in[i] < 0x7f) ? (char)in[i] : ' ';
	}
	while (n > 0 && out[n - 1] == ' ') {
		n--;
	}
	out[n] = '\0';
}

bool
PIEUSBMassStorage::start(IOService *provider)
{
	intf = OSDynamicCast(PIEUSBInterface, provider);
	if (intf == NULL || !super::start(provider)) {
		return false;
	}
	for (unsigned i = 0; i < intf->numEndpoints; i++) {
		const struct PIEUSBEndpoint *ep = &intf->endpoints[i];
		if ((ep->attributes & 3) == 2) {
			if (ep->address & 0x80) {
				bulkIn = ep->address;
			} else {
				bulkOut = ep->address;
			}
		}
	}
	lock = IOLockAlloc();
	bounce = (uint8_t *)IOMallocData(BOUNCE_SIZE);
	if (bulkIn == 0 || bulkOut == 0 || lock == NULL || bounce == NULL) {
		IOLog("USB storage: no bulk endpoints\n");
		return false;
	}

	uint8_t maxLun = 0;
	if (intf->controlRequest(0xa1, BOT_GET_MAX_LUN, 0, intf->number, 1, &maxLun) != kIOReturnSuccess) {
		maxLun = 0;
	}

	uint8_t inquiry[6] = { SCSI_INQUIRY, 0, 0, 0, 36, 0 };
	if (scsi(inquiry, sizeof(inquiry), true, 36) != kIOReturnSuccess) {
		IOLog("USB storage: INQUIRY failed\n");
		return false;
	}
	uint8_t type = bounce[0] & 0x1f;
	scsi_string(vendor, bounce + 8, 8);
	scsi_string(product, bounce + 16, 16);
	scsi_string(revision, bounce + 32, 4);
	if (type != 0x00 && type != 0x0e) {
		IOLog("USB storage: %s %s: SCSI device type 0x%x, not a disk\n", vendor, product, type);
		return false;
	}
	IOReturn ret = waitReady();
	if (ret == kIOReturnSuccess) {
		ret = readCapacity();
	}
	if (ret != kIOReturnSuccess) {
		IOLog("USB storage: %s %s: %s\n", vendor, product, ret == kIOReturnNoMedia ? "no medium" : "not ready");
		return false;
	}
	if (blockSize < 512 || blockSize > 4096 || (blockSize & (blockSize - 1)) != 0) {
		IOLog("USB storage: %s %s: unsupported block size %u\n", vendor, product, blockSize);
		return false;
	}
	uint8_t modeSense[6] = { SCSI_MODE_SENSE_6, 0, 0x3f, 0, 192, 0 };
	uint32_t moved = 0;
	if (scsi(modeSense, sizeof(modeSense), true, 192, &moved) == kIOReturnSuccess && moved >= 4) {
		writeProtected = (bounce[2] & 0x80) != 0;
	}
	IOLog("USB storage: %s %s rev %s, %llu blocks of %u bytes (%llu MB)%s%s\n", vendor, product, revision,
	    blocks, blockSize, blocks * blockSize / (1024 * 1024), writeProtected ? ", write protected" : "",
	    maxLun > 0 ? ", LUN 0 only" : "");

	OSDictionary *proto = OSDictionary::withCapacity(2);
	if (proto != NULL) {
		OSString *s = OSString::withCString(kIOPropertyPhysicalInterconnectTypeUSB);
		proto->setObject(kIOPropertyPhysicalInterconnectTypeKey, s);
		s->release();
		s = OSString::withCString(kIOPropertyExternalKey);
		proto->setObject(kIOPropertyPhysicalInterconnectLocationKey, s);
		s->release();
		setProperty(kIOPropertyProtocolCharacteristicsKey, proto);
		proto->release();
	}
	setProperty(kIOMaximumBlockCountReadKey, BOUNCE_SIZE / blockSize, 64);
	setProperty(kIOMaximumBlockCountWriteKey, BOUNCE_SIZE / blockSize, 64);

	registerService();
	return true;
}

void
PIEUSBMassStorage::stop(IOService *provider)
{
	IOLog("USB storage: %s %s: gone\n", vendor, product);
	super::stop(provider);
}

void
PIEUSBMassStorage::free(void)
{
	if (bounce != NULL) {
		IOFreeData(bounce, BOUNCE_SIZE);
	}
	if (lock != NULL) {
		IOLockFree(lock);
	}
	super::free();
}

#pragma mark IOBlockStorageDevice

IOReturn
PIEUSBMassStorage::doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 nblks,
    IOStorageAttributes *attributes, IOStorageCompletion *completion)
{
	(void)attributes;
	bool write = buffer->getDirection() == kIODirectionOut;
	IOReturn ret = kIOReturnSuccess;
	uint32_t maxBlocks = BOUNCE_SIZE / blockSize;

	if (block + nblks > blocks) {
		ret = kIOReturnBadArgument;
	} else if (write && writeProtected) {
		ret = kIOReturnNotWritable;
	}
	IOLockLock(lock);
	for (UInt64 done = 0; ret == kIOReturnSuccess && done < nblks;) {
		uint32_t n = (uint32_t)(nblks - done < maxBlocks ? nblks - done : maxBlocks);
		IOByteCount offset = done * blockSize, length = (IOByteCount)n * blockSize;
		if (write) {
			if (buffer->readBytes(offset, bounce, length) != length) {
				ret = kIOReturnIOError;
				break;
			}
			ret = readWrite(block + done, n, true);
		} else {
			ret = readWrite(block + done, n, false);
			if (ret == kIOReturnSuccess && buffer->writeBytes(offset, bounce, length) != length) {
				ret = kIOReturnIOError;
			}
		}
		done += n;
	}
	IOLockUnlock(lock);

	IOStorage::complete(completion, ret, ret == kIOReturnSuccess ? nblks * blockSize : 0);
	return kIOReturnSuccess;
}

IOReturn
PIEUSBMassStorage::doSynchronize(UInt64 block, UInt64 nblks, IOStorageSynchronizeOptions options)
{
	(void)block;
	(void)nblks;
	(void)options;
	uint8_t cdb[10] = { SCSI_SYNCHRONIZE_CACHE_10 };
	IOLockLock(lock);
	IOReturn ret = scsi(cdb, sizeof(cdb), false, 0);
	if (ret == kIOReturnIOError && senseKey == SENSE_ILLEGAL_REQUEST) {
		ret = kIOReturnSuccess;
	}
	IOLockUnlock(lock);
	return ret;
}

IOReturn PIEUSBMassStorage::doEjectMedia(void) { return kIOReturnUnsupported; }
IOReturn PIEUSBMassStorage::doFormatMedia(UInt64 byteCapacity) { (void)byteCapacity; return kIOReturnUnsupported; }

UInt32
PIEUSBMassStorage::doGetFormatCapacities(UInt64 *capacities, UInt32 capacitiesMaxCount) const
{
	if (capacities != NULL && capacitiesMaxCount > 0) {
		capacities[0] = blocks * blockSize;
	}
	return 1;
}

char *PIEUSBMassStorage::getVendorString(void) { return vendor; }
char *PIEUSBMassStorage::getProductString(void) { return product; }
char *PIEUSBMassStorage::getRevisionString(void) { return revision; }
char *PIEUSBMassStorage::getAdditionalDeviceInfoString(void) { return (char *)"USB mass storage"; }

IOReturn PIEUSBMassStorage::reportBlockSize(UInt64 *size) { *size = blockSize; return kIOReturnSuccess; }
IOReturn PIEUSBMassStorage::reportEjectability(bool *isEjectable) { *isEjectable = false; return kIOReturnSuccess; }
IOReturn PIEUSBMassStorage::reportMaxValidBlock(UInt64 *maxBlock) { *maxBlock = blocks - 1; return kIOReturnSuccess; }

IOReturn
PIEUSBMassStorage::reportMediaState(bool *mediaPresent, bool *changedState)
{
	*mediaPresent = true;
	if (changedState != NULL) {
		*changedState = false;
	}
	return kIOReturnSuccess;
}

IOReturn PIEUSBMassStorage::reportRemovability(bool *isRemovable) { *isRemovable = false; return kIOReturnSuccess; }
IOReturn PIEUSBMassStorage::reportWriteProtection(bool *isWriteProtected) { *isWriteProtected = writeProtected; return kIOReturnSuccess; }
