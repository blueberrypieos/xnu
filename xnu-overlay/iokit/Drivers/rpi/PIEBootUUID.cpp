/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOBSD.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <libkern/OSByteOrder.h>
#include <libkern/crypto/md5.h>
#include <uuid/uuid.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#include <IOKit/storage/IOMedia.h>
#pragma clang diagnostic pop

#define HFS_UUID_NAMESPACE_ID   "\xB3\xE2\x0F\x39\xF2\x92\x11\xD6\x97\xA4\x00\x30\x65\x43\xEC\xAC"
#define VH_OFFSET               1024
#define VH_FINDER_INFO          80
#define PROBE_SIZE              4096

class PIEBootUUID : public IOService
{
	OSDeclareDefaultStructors(PIEBootUUID);

	OSString *want;
	uuid_t wanted;
	IOLock *lock;
	IOMedia *root;
	bool announced;
	IONotifier *notifier;
	IONotifier *clientNotifier;

	bool probe(IOMedia *media);
	void announceLocked(void);
	static bool published(void *target, void *refCon, IOService *service, IONotifier *notifier);
	static bool clientPublished(void *target, void *refCon, IOService *service, IONotifier *notifier);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
};

#define super IOService
OSDefineMetaClassAndStructors(PIEBootUUID, IOService);

bool
PIEBootUUID::probe(IOMedia *media)
{
	if (media->getSize() < PROBE_SIZE) {
		return false;
	}
	IOBufferMemoryDescriptor *buffer = IOBufferMemoryDescriptor::withCapacity(PROBE_SIZE, kIODirectionIn);
	if (buffer == NULL) {
		return false;
	}
	IOReturn ret = kIOReturnNotOpen;
	if (media->open(this, 0, kIOStorageAccessReader)) {
		ret = media->IOStorage::read(this, 0, buffer, NULL, (UInt64 *)NULL);
		media->close(this);
	}
	bool match = false;
	const uint8_t *vh = (const uint8_t *)buffer->getBytesNoCopy() + VH_OFFSET;
	uint16_t signature = OSReadBigInt16(vh, 0);
	if (ret == kIOReturnSuccess && (signature == 0x482B || signature == 0x4858)) {
		MD5_CTX md5;
		uuid_t uuid;
		MD5Init(&md5);
		MD5Update(&md5, HFS_UUID_NAMESPACE_ID, sizeof(uuid_t));
		MD5Update(&md5, vh + VH_FINDER_INFO + 6 * 4, 8);
		MD5Final(uuid, &md5);
		uuid[6] = 0x30 | (uuid[6] & 0x0F);
		uuid[8] = 0x80 | (uuid[8] & 0x3F);
		match = uuid_compare(uuid, wanted) == 0;
	}
	buffer->release();
	return match;
}

void
PIEBootUUID::announceLocked(void)
{
	if (root == NULL || announced || root->getProperty(kIOBSDMajorKey) == NULL ||
	    root->getProperty(kIOBSDMinorKey) == NULL) {
		return;
	}
	announced = true;
	OSString *name = OSDynamicCast(OSString, root->getProperty(kIOBSDNameKey));
	IOLog("PIEBootUUID: %s is the volume %s\n", name ? name->getCStringNoCopy() : root->getName(),
	    want->getCStringNoCopy());
	publishResource("boot-uuid-media", root);
}

bool
PIEBootUUID::published(void *target, void *refCon, IOService *service, IONotifier *notifier)
{
	(void)refCon;
	(void)notifier;
	PIEBootUUID *self = (PIEBootUUID *)target;
	IOMedia *media = OSDynamicCast(IOMedia, service);
	if (media == NULL) {
		return true;
	}
	IOLockLock(self->lock);
	if (self->root == NULL && self->probe(media)) {
		media->retain();
		self->root = media;
		self->announceLocked();
	}
	IOLockUnlock(self->lock);
	return true;
}

bool
PIEBootUUID::clientPublished(void *target, void *refCon, IOService *service, IONotifier *notifier)
{
	(void)refCon;
	(void)notifier;
	PIEBootUUID *self = (PIEBootUUID *)target;
	IOLockLock(self->lock);
	if (self->root != NULL && service->getProvider() == self->root) {
		self->announceLocked();
	}
	IOLockUnlock(self->lock);
	return true;
}

bool
PIEBootUUID::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	want = OSDynamicCast(OSString, getResourceService()->getProperty("boot-uuid"));
	if (want == NULL || uuid_parse(want->getCStringNoCopy(), wanted) != 0) {
		return false;
	}
	want->retain();
	lock = IOLockAlloc();
	if (lock == NULL) {
		return false;
	}
	clientNotifier = addMatchingNotification(gIOFirstPublishNotification, serviceMatching("IOMediaBSDClient"),
	    &PIEBootUUID::clientPublished, this, NULL, 0);
	notifier = addMatchingNotification(gIOFirstPublishNotification, serviceMatching("IOMedia"),
	    &PIEBootUUID::published, this, NULL, 0);
	return clientNotifier != NULL && notifier != NULL;
}
