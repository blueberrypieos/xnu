/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_ROCKCHIP_RKI2C_H
#define _IOKIT_ROCKCHIP_RKI2C_H

#include <IOKit/IOLocks.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>

/*
 * The RK3568 I2C controller as a bus master, polled. Its device tree
 * children are published as nubs below it; their drivers reach the bus
 * through their provider's provider, and find their address in the first
 * u64 of pieboot,bus-reg. readRegisters writes reg, restarts
 * and reads up to 32 bytes; writeRegisters writes reg and up to 30 bytes.
 * Both wait for the bus, so they are not for interrupt context. They spin
 * while they wait. A transfer fails if its start does not come within
 * 2 ms, its data does not end within four times its bus time of being
 * sent, or its stop does not end within as long as the two. The bus runs
 * at clock-frequency, from 10 kHz to 100 kHz, and the waits cover every
 * rate in that range.
 */
class RKI2C : public IOService
{
	OSDeclareDefaultStructors(RKI2C);

	IOMemoryMap *regsMap;
	IOLock *lock;
	uint32_t busHz;

	uint32_t rd(uint32_t off);
	void wr(uint32_t off, uint32_t v);
	uint32_t waitFor(uint32_t bits, uint64_t deadline);
	IOReturn transfer(uint8_t address, uint8_t reg, uint8_t *data, uint32_t length, bool read);
	uint64_t clockRate(IOMemoryMap *pmucru);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	IOReturn readRegisters(uint8_t address, uint8_t reg, uint8_t *data, uint32_t length);
	IOReturn writeRegisters(uint8_t address, uint8_t reg, const uint8_t *data, uint32_t length);
};

#endif
