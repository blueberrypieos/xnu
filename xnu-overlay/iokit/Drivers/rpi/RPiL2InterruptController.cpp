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

enum {
	L2_VECTORS = 32,
	L2_CPU_STATUS = 0x00,
};

#undef super
#define super IOInterruptController
OSDefineMetaClassAndStructors(RPiL2InterruptController, IOInterruptController);

bool
RPiL2InterruptController::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}

	levelType = false;
	OSData *compatible = OSDynamicCast(OSData, provider->getProperty("compatible"));
	if (compatible != NULL) {
		const char *c = (const char *)compatible->getBytesNoCopy();
		unsigned length = compatible->getLength();
		for (unsigned i = 0; i < length; i += (unsigned)strnlen(c + i, length - i) + 1) {
			if (strncmp(c + i, "brcm,bcm7271-l2-intc", length - i) == 0) {
				levelType = true;
			}
		}
	}
	if (levelType) {
		maskStatus = 0x04;
		maskSet = 0x08;
		maskClear = 0x0c;
		clear = 0;
	} else {
		clear = 0x08;
		maskStatus = 0x0c;
		maskSet = 0x10;
		maskClear = 0x14;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL || regsMap->getLength() < (levelType ? 0x10U : 0x18U)) {
		IOLog("RPiL2: %s: cannot map the registers\n", provider->getName());
		return false;
	}
	regs = (volatile uint32_t *)regsMap->getVirtualAddress();

	regs[maskSet / 4] = ~0U;
	if (!levelType) {
		regs[clear / 4] = ~0U;
	}

	vectors = IONewZero(IOInterruptVector, L2_VECTORS);
	if (vectors == NULL) {
		return false;
	}
	for (uint32_t i = 0; i < L2_VECTORS; i++) {
		vectors[i].interruptLock = IOLockAlloc();
		if (vectors[i].interruptLock == NULL) {
			return false;
		}
	}

	const OSSymbol *name = IODTInterruptControllerName(provider);
	if (name == NULL) {
		return false;
	}
	setProperty("InterruptControllerName", (OSObject *)name);
	setProperty("InterruptControllerType", levelType ? "level" : "latched");
	getPlatform()->registerInterruptController((OSSymbol *)name, this);
	name->release();

	if (provider->registerInterrupt(0, this, getInterruptHandlerAddress(), NULL) != kIOReturnSuccess) {
		IOLog("RPiL2: %s: cannot take its parent interrupt\n", provider->getName());
		return false;
	}
	provider->enableInterrupt(0);

	IOLog("RPiL2: %s %s, %s, 32 interrupts\n", provider->getName(), provider->getLocation(),
	    levelType ? "level" : "latched");
	registerService();
	return true;
}

IOReturn
RPiL2InterruptController::registerInterrupt(IOService *nub, int source, void *target,
    IOInterruptHandler handler, void *refCon)
{
	OSData *data = nub->_interruptSources != NULL ? nub->_interruptSources[source].vectorData : NULL;
	if (data == NULL || data->getLength() < sizeof(uint32_t) ||
	    *(const uint32_t *)data->getBytesNoCopy() >= L2_VECTORS) {
		return kIOReturnBadArgument;
	}
	return super::registerInterrupt(nub, source, target, handler, refCon);
}

IOInterruptAction
RPiL2InterruptController::getInterruptHandlerAddress(void)
{
	return OSMemberFunctionCast(IOInterruptAction, this, &RPiL2InterruptController::handleInterrupt);
}

IOReturn
RPiL2InterruptController::handleInterrupt(void *  , IOService *  , int  )
{
	uint32_t pending = regs[L2_CPU_STATUS / 4] & ~regs[maskStatus / 4];

	if (pending != 0 && !levelType) {
		regs[clear / 4] = pending;
	}
	while (pending != 0) {
		uint32_t bit = (uint32_t)__builtin_ctz(pending);
		pending &= pending - 1;

		IOInterruptVector *vector = &vectors[bit];
		vector->interruptActive = 1;
		OSMemoryBarrier();
		if (!vector->interruptDisabledSoft && vector->interruptRegistered) {
			bool trace = (gIOKitTrace & kIOTraceInterrupts) != 0;
			if (trace) {
				timeStampInterruptHandlerStart(bit, vector);
			}
			vector->handler(vector->target, vector->refCon, vector->nub, vector->source);
			if (trace) {
				timeStampInterruptHandlerEnd(bit, vector);
			}
		} else {
			vector->interruptDisabledHard = 1;
			disableVectorHard(bit, vector);
			OSSynchronizeIO();
		}
		vector->interruptActive = 0;
	}
	return kIOReturnSuccess;
}

bool
RPiL2InterruptController::vectorCanBeShared(IOInterruptVectorNumber  ,
    IOInterruptVector *  )
{
	return true;
}

int
RPiL2InterruptController::getVectorType(IOInterruptVectorNumber  ,
    IOInterruptVector *  )
{
	return levelType ? kIOInterruptTypeLevel : kIOInterruptTypeEdge;
}

void
RPiL2InterruptController::disableVectorHard(IOInterruptVectorNumber vectorNumber,
    IOInterruptVector *  )
{
	if (vectorNumber < L2_VECTORS) {
		regs[maskSet / 4] = 1U << vectorNumber;
	}
}

void
RPiL2InterruptController::enableVector(IOInterruptVectorNumber vectorNumber,
    IOInterruptVector *  )
{
	if (vectorNumber < L2_VECTORS) {
		regs[maskClear / 4] = 1U << vectorNumber;
	}
}
