/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_RPI_PLATFORM_H
#define _IOKIT_RPI_PLATFORM_H

#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOInterruptController.h>
#include <IOKit/IOPMGR.h>

#include "../../pie/rpi.h"

class RPiPlatformExpert : public IODTPlatformExpert
{
	OSDeclareDefaultStructors(RPiPlatformExpert);

	bool hasRTC;

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual long getGMTTimeOfDay(void) APPLE_KEXT_OVERRIDE;
	virtual void setGMTTimeOfDay(long secs) APPLE_KEXT_OVERRIDE;
	virtual const char *deleteList(void) APPLE_KEXT_OVERRIDE;
	virtual const char *excludeList(void) APPLE_KEXT_OVERRIDE;
	virtual int haltRestart(unsigned int type) APPLE_KEXT_OVERRIDE;
};

class RPiIO : public IOService
{
	OSDeclareDefaultStructors(RPiIO);

public:
	static void logFirmware(void);

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
};

class RPiIODevice : public IOPlatformDevice
{
	OSDeclareDefaultStructors(RPiIODevice);

public:
	virtual bool compareName(OSString *name, OSString **matched = NULL) const APPLE_KEXT_OVERRIDE;
	virtual IOReturn getResources(void) APPLE_KEXT_OVERRIDE;
};

class RPiPCIe;
class RPiPCIDevice : public IOService
{
	OSDeclareDefaultStructors(RPiPCIDevice);

public:
	RPiPCIe *pcie;
	uint32_t location;

	uint32_t configRead32(uint16_t offset);
	void configWrite32(uint16_t offset, uint32_t value);
	uint16_t configRead16(uint16_t offset);
	void configWrite16(uint16_t offset, uint16_t value);
	uint8_t findCapability(uint8_t capID);
	int barIndex(unsigned bar);
};

class RPiMIP : public IOService
{
	OSDeclareDefaultStructors(RPiMIP);

	IOMemoryMap *regsMap;
	IOLock *lock;
	uint64_t doorbellAddress;
	uint32_t spiBase;
	uint32_t count;
	uint64_t used;
	const OSSymbol *gicName;
	const OSSymbol *owner[64];
	bool started;

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual bool serializeProperties(OSSerialize *s) const APPLE_KEXT_OVERRIDE;

	int allocate(unsigned n);
	uint64_t doorbell(void) const { return doorbellAddress; }
	uint32_t gicInterrupt(unsigned v) const { return 32 + spiBase + v; }
	const OSSymbol *gicControllerName(void) const { return gicName; }
};

void RPiPublishDTChildren(IOService *parent, IORegistryEntry *dtNode);

class RPiL2InterruptController : public IOInterruptController
{
	OSDeclareDefaultStructors(RPiL2InterruptController);

	IOMemoryMap *regsMap;
	volatile uint32_t *regs;
	bool levelType;
	uint32_t maskStatus, maskSet, maskClear, clear;

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;

	virtual IOReturn registerInterrupt(IOService *nub, int source, void *target,
	    IOInterruptHandler handler, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOInterruptAction getInterruptHandlerAddress(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn handleInterrupt(void *refCon, IOService *nub, int source) APPLE_KEXT_OVERRIDE;

	virtual bool vectorCanBeShared(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual int getVectorType(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void disableVectorHard(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void enableVector(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
};

class RPiGICInterruptController : public IOInterruptController
{
	OSDeclareDefaultStructors(RPiGICInterruptController);

	uint32_t numVectors;

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;

	virtual IOInterruptAction getInterruptHandlerAddress(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn handleInterrupt(void *refCon, IOService *nub, int source) APPLE_KEXT_OVERRIDE;

	virtual bool vectorCanBeShared(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual int getVectorType(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void disableVectorHard(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void enableVector(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
};

class RPiPSCIPMGR : public IOPMGR
{
	OSDeclareDefaultStructors(RPiPSCIPMGR);

	uint64_t secondaryEntry;

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;

	virtual void enableCPUCore(unsigned int cpu_id, uint64_t entry_pa) APPLE_KEXT_OVERRIDE;
	virtual void disableCPUCore(unsigned int cpu_id) APPLE_KEXT_OVERRIDE;
	virtual void enableCPUCluster(unsigned int cluster_id) APPLE_KEXT_OVERRIDE;
	virtual void disableCPUCluster(unsigned int cluster_id) APPLE_KEXT_OVERRIDE;
	virtual void initCPUIdle(ml_processor_info_t *info) APPLE_KEXT_OVERRIDE;
	virtual void enterCPUIdle(UInt64 *newIdleTimeoutTicks) APPLE_KEXT_OVERRIDE;
	virtual void exitCPUIdle(UInt64 *newIdleTimeoutTicks) APPLE_KEXT_OVERRIDE;
	virtual void updateCPUIdle(UInt64 *newIdleTimeoutTicks) APPLE_KEXT_OVERRIDE;
};

#define PSCI_0_2_FN_PSCI_VERSION        0x84000000u
#define PSCI_0_2_FN_CPU_OFF             0x84000002u
#define PSCI_0_2_FN64_CPU_ON            0xc4000003u
#define PSCI_0_2_FN_SYSTEM_OFF          0x84000008u
#define PSCI_0_2_FN_SYSTEM_RESET        0x84000009u

#define PSCI_RET_SUCCESS                0
#define PSCI_RET_ALREADY_ON             (-4)

bool rpi_psci_init(void);
int64_t rpi_psci_call(uint64_t fn, uint64_t arg0, uint64_t arg1, uint64_t arg2);

void rpi_watchdog_early_start(IOService *nub);
void rpi_watchdog_halt(void);
void rpi_watchdog_panic(void);

#endif
