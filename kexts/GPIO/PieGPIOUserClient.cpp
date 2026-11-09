/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PieGPIOController's user clients (<PieGPIO.h>): classic IOUserClients,
 * whose methods IOUserClient::externalMethod dispatches from a static
 * table (which checks the scalar and structure counts) and the controller
 * runs on its work loop. Each has an IOSharedDataQueue for its events,
 * which the process maps (clientMemoryForType) and is told of through the
 * port it gives (registerNotificationPort): info events on a chip
 * connection, edge events on a request.
 */

#include <IOKit/IOLib.h>

#include "PieGPIOController.h"

#define kInfoQueueEntries 32    /* as Linux's chip file's info event FIFO */

#pragma mark PieGPIOChipUserClient

OSDefineMetaClassAndStructors(PieGPIOChipUserClient, IOUserClient);

const IOExternalMethodDispatch PieGPIOChipUserClient::sMethods[kPieGPIOChipMethodCount] = {
	[kPieGPIOGetLineInfo]     = { &sGetLineInfo, 1, 0, 0, sizeof(PieGPIOLineInfo) },
	[kPieGPIOWatchLineInfo]   = { &sWatchLineInfo, 1, 0, 0, sizeof(PieGPIOLineInfo) },
	[kPieGPIOUnwatchLineInfo] = { &sUnwatchLineInfo, 1, 0, 0, 0 },
};

bool
PieGPIOChipUserClient::initWith(PieGPIOController *controller, task_t task, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(task, securityID, type)) {
		return false;
	}
	fQueue = IOSharedDataQueue::withEntries(kInfoQueueEntries, sizeof(PieGPIOInfoEvent));
	if (fQueue == NULL) {
		return false;
	}
	fController = controller;
	fController->retain();
	return true;
}

IOReturn
PieGPIOChipUserClient::clientClose(void)
{
	if (fController != NULL) {
		fController->removeChipClient(this);
	}
	terminate();
	return kIOReturnSuccess;
}

void
PieGPIOChipUserClient::free(void)
{
	if (fQueue != NULL) {
		fQueue->setNotificationPort(MACH_PORT_NULL);
	}
	if (fPort != MACH_PORT_NULL) {
		releaseNotificationPort(fPort);
	}
	OSSafeReleaseNULL(fQueue);
	OSSafeReleaseNULL(fController);
	IOUserClient::free();
}

IOReturn
PieGPIOChipUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    IOExternalMethodDispatch *, OSObject *, void *)
{
	if (selector >= kPieGPIOChipMethodCount) {
		return kIOReturnUnsupported;
	}
	return IOUserClient::externalMethod(selector, args, (IOExternalMethodDispatch *)&sMethods[selector], this,
	           NULL);
}

IOReturn
PieGPIOChipUserClient::sGetLineInfo(OSObject *target, void *, IOExternalMethodArguments *args)
{
	PieGPIOChipUserClient *self = (PieGPIOChipUserClient *)target;
	return self->fController->getLineInfo((uint32_t)args->scalarInput[0], (PieGPIOLineInfo *)args->structureOutput);
}

IOReturn
PieGPIOChipUserClient::sWatchLineInfo(OSObject *target, void *, IOExternalMethodArguments *args)
{
	PieGPIOChipUserClient *self = (PieGPIOChipUserClient *)target;
	return self->fController->watchLine(self, (uint32_t)args->scalarInput[0],
	           (PieGPIOLineInfo *)args->structureOutput);
}

IOReturn
PieGPIOChipUserClient::sUnwatchLineInfo(OSObject *target, void *, IOExternalMethodArguments *args)
{
	PieGPIOChipUserClient *self = (PieGPIOChipUserClient *)target;
	return self->fController->unwatchLine(self, (uint32_t)args->scalarInput[0]);
}

/* The info event queue, for IOConnectMapMemory64 (type kPieGPIOEventQueue). */
IOReturn
PieGPIOChipUserClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory)
{
	if (type != kPieGPIOEventQueue) {
		return kIOReturnBadArgument;
	}
	IOMemoryDescriptor *md = fQueue->getMemoryDescriptor();
	if (md == NULL) {
		return kIOReturnNoMemory;
	}
	*options = 0;
	*memory = md;
	return kIOReturnSuccess;
}

IOReturn
PieGPIOChipUserClient::registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t)
{
	if (type != kPieGPIOEventQueue) {
		return kIOReturnBadArgument;
	}
	mach_port_t old = MACH_PORT_NULL, *oldp = &old;
	IOReturn ret = fController->gated(^IOReturn {
		*oldp = fPort;
		fPort = port;
		fQueue->setNotificationPort(port);
		return kIOReturnSuccess;
	});
	if (old != MACH_PORT_NULL) {
		releaseNotificationPort(old);
	}
	return ret;
}

/* On the controller's work loop. */
void
PieGPIOChipUserClient::post(const PieGPIOInfoEvent *event)
{
	fQueue->enqueue((void *)event, sizeof(*event));
}

#pragma mark PieGPIORequestUserClient

OSDefineMetaClassAndStructors(PieGPIORequestUserClient, IOUserClient);

const IOExternalMethodDispatch PieGPIORequestUserClient::sMethods[kPieGPIORequestMethodCount] = {
	[kPieGPIORequestLines]     = { &sRequestLines, 0, sizeof(PieGPIORequest), 0, 0 },
	[kPieGPIOGetValues]        = { &sGetValues, 1, 0, 1, 0 },
	[kPieGPIOSetValues]        = { &sSetValues, 2, 0, 0, 0 },
	[kPieGPIOReconfigureLines] = { &sReconfigureLines, 0, sizeof(PieGPIORequest), 0, 0 },
};

bool
PieGPIORequestUserClient::initWith(PieGPIOController *controller, task_t task, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(task, securityID, type)) {
		return false;
	}
	fController = controller;
	fController->retain();
	return true;
}

IOReturn
PieGPIORequestUserClient::clientClose(void)
{
	if (fController != NULL) {
		fController->releaseLines(this);
	}
	terminate();
	return kIOReturnSuccess;
}

void
PieGPIORequestUserClient::free(void)
{
	if (fQueue != NULL) {
		fQueue->setNotificationPort(MACH_PORT_NULL);
	}
	if (fPort != MACH_PORT_NULL) {
		releaseNotificationPort(fPort);
	}
	OSSafeReleaseNULL(fQueue);
	OSSafeReleaseNULL(fController);
	IOUserClient::free();
}

IOReturn
PieGPIORequestUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    IOExternalMethodDispatch *, OSObject *, void *)
{
	if (selector >= kPieGPIORequestMethodCount) {
		return kIOReturnUnsupported;
	}
	return IOUserClient::externalMethod(selector, args, (IOExternalMethodDispatch *)&sMethods[selector], this,
	           NULL);
}

IOReturn
PieGPIORequestUserClient::sRequestLines(OSObject *target, void *, IOExternalMethodArguments *args)
{
	PieGPIORequestUserClient *self = (PieGPIORequestUserClient *)target;
	return self->fController->requestLines(self, (const PieGPIORequest *)args->structureInput);
}

IOReturn
PieGPIORequestUserClient::sGetValues(OSObject *target, void *, IOExternalMethodArguments *args)
{
	PieGPIORequestUserClient *self = (PieGPIORequestUserClient *)target;
	return self->fController->getValues(self, args->scalarInput[0], &args->scalarOutput[0]);
}

IOReturn
PieGPIORequestUserClient::sSetValues(OSObject *target, void *, IOExternalMethodArguments *args)
{
	PieGPIORequestUserClient *self = (PieGPIORequestUserClient *)target;
	return self->fController->setValues(self, args->scalarInput[0], args->scalarInput[1]);
}

IOReturn
PieGPIORequestUserClient::sReconfigureLines(OSObject *target, void *, IOExternalMethodArguments *args)
{
	PieGPIORequestUserClient *self = (PieGPIORequestUserClient *)target;
	return self->fController->reconfigureLines(self, (const PieGPIORequest *)args->structureInput);
}

/* On the controller's work loop, from requestLines. */
bool
PieGPIORequestUserClient::makeQueue(uint32_t entries)
{
	fQueue = IOSharedDataQueue::withEntries(entries, sizeof(PieGPIOEdgeEvent));
	if (fQueue == NULL) {
		return false;
	}
	if (fPort != MACH_PORT_NULL) {
		fQueue->setNotificationPort(fPort);
	}
	return true;
}

/*
 * The edge event queue, for IOConnectMapMemory64 (type
 * kPieGPIOEventQueue), once the lines are requested.
 */
IOReturn
PieGPIORequestUserClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory)
{
	if (type != kPieGPIOEventQueue) {
		return kIOReturnBadArgument;
	}
	IOMemoryDescriptor *md = NULL, **mdp = &md;     /* (no __block: kexts have no _Block_object_assign) */
	fController->gated(^IOReturn {
		*mdp = fQueue != NULL ? fQueue->getMemoryDescriptor() : NULL;
		return kIOReturnSuccess;
	});
	if (md == NULL) {
		return kIOReturnNotReady;
	}
	*options = 0;
	*memory = md;
	return kIOReturnSuccess;
}

IOReturn
PieGPIORequestUserClient::registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t)
{
	if (type != kPieGPIOEventQueue) {
		return kIOReturnBadArgument;
	}
	mach_port_t old = MACH_PORT_NULL, *oldp = &old;
	IOReturn ret = fController->gated(^IOReturn {
		*oldp = fPort;
		fPort = port;
		if (fQueue != NULL) {
			fQueue->setNotificationPort(port);
		}
		return kIOReturnSuccess;
	});
	if (old != MACH_PORT_NULL) {
		releaseNotificationPort(old);
	}
	return ret;
}

/* On the controller's work loop: numbered in the request, then queued (dropped, numbers kept, when full). */
void
PieGPIORequestUserClient::post(PieGPIOEdgeEvent *event)
{
	event->seqno = ++fSeqno;
	if (fQueue != NULL) {
		fQueue->enqueue(event, sizeof(*event));
	}
}
