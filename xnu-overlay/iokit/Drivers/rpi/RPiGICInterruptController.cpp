/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOKitDebug.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/OSAtomic.h>

#include "RPiPlatform.h"

extern "C" {
#include <pexpert/arm64/gicv2.h>
}

#undef super
#define super IOInterruptController
OSDefineMetaClassAndStructors(RPiGICInterruptController, IOInterruptController);

bool
RPiGICInterruptController::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}

	numVectors = pe_gicv2_num_irqs();
	if (numVectors == 0) {
		IOLog("RPiGIC: GIC not initialized by the kernel\n");
		return false;
	}
	vectors = IONewZero(IOInterruptVector, numVectors);
	if (!vectors) {
		return false;
	}
	for (uint32_t i = 0; i < numVectors; i++) {
		vectors[i].interruptLock = IOLockAlloc();
		if (!vectors[i].interruptLock) {
			return false;
		}
	}

	const OSSymbol *name = IODTInterruptControllerName(provider);
	if (!name) {
		return false;
	}
	setProperty("InterruptControllerName", (OSObject *)name);
	getPlatform()->registerInterruptController((OSSymbol *)name, this);
	name->release();

	getPlatform()->setCPUInterruptProperties(this);
	if (IOService::registerInterrupt(0, this, getInterruptHandlerAddress(), NULL) != kIOReturnSuccess) {
		IOLog("RPiGIC: cannot register with the CPU interrupt controller\n");
		return false;
	}
	IOService::enableInterrupt(0);

	IOLog("RPiGIC: %u interrupts\n", numVectors);
	registerService();
	return true;
}

IOInterruptAction
RPiGICInterruptController::getInterruptHandlerAddress(void)
{
	return OSMemberFunctionCast(IOInterruptAction, this, &RPiGICInterruptController::handleInterrupt);
}

IOReturn
RPiGICInterruptController::handleInterrupt(void *  , IOService *  , int  )
{
	uint32_t irq = pe_gicv2_pending_irq();

	if (irq >= numVectors) {
		return kIOReturnInvalid;
	}

	IOInterruptVector *vector = &vectors[irq];
	vector->interruptActive = 1;
	OSMemoryBarrier();

	if (!vector->interruptDisabledSoft && vector->interruptRegistered) {
		bool trace = (gIOKitTrace & kIOTraceInterrupts) != 0;
		if (trace) {
			timeStampInterruptHandlerStart(irq, vector);
		}
		vector->handler(vector->target, vector->refCon, vector->nub, vector->source);
		if (trace) {
			timeStampInterruptHandlerEnd(irq, vector);
		}
	} else {
		vector->interruptDisabledHard = 1;
		disableVectorHard(irq, vector);
		OSSynchronizeIO();
	}

	vector->interruptActive = 0;
	return kIOReturnSuccess;
}

bool
RPiGICInterruptController::vectorCanBeShared(IOInterruptVectorNumber  ,
    IOInterruptVector *  )
{
	return true;
}

int
RPiGICInterruptController::getVectorType(IOInterruptVectorNumber vectorNumber,
    IOInterruptVector *  )
{
	return pe_gicv2_is_edge((uint32_t)vectorNumber) ? kIOInterruptTypeEdge : kIOInterruptTypeLevel;
}

void
RPiGICInterruptController::disableVectorHard(IOInterruptVectorNumber vectorNumber,
    IOInterruptVector *  )
{
	pe_gicv2_enable_irq((uint32_t)vectorNumber, false);
}

void
RPiGICInterruptController::enableVector(IOInterruptVectorNumber vectorNumber,
    IOInterruptVector *  )
{
	pe_gicv2_enable_irq((uint32_t)vectorNumber, true);
}
