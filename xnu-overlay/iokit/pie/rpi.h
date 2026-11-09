/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <pie/rpi.h>: the Raspberry Pi platform helpers that kernel modules may
 * call, from the com.blueberrypieos.kpi.platform symbol set
 * (config/PIE.exports). The kernel's own drivers get them through
 * iokit/Drivers/rpi/RPiPlatform.h; RPiPlatformExpert.cpp implements them.
 * C linkage, for C and C++ modules alike.
 */

#ifndef _PIE_RPI_H_
#define _PIE_RPI_H_

#include <sys/cdefs.h>
#include <stdint.h>
#include <stdbool.h>
#include <IOKit/IOReturn.h>

#ifdef __cplusplus
class IORegistryEntry;
class OSArray;
#else
typedef struct IORegistryEntry IORegistryEntry;
typedef struct OSArray OSArray;
#endif

__BEGIN_DECLS

/*
 * One property call to the VideoCore firmware (mailbox channel 8): tag,
 * with length bytes of request in data, answered in place. For drivers
 * that need the firmware after boot.
 */
IOReturn RPiFirmwareProperty(uint32_t tag, void *data, uint32_t length);

/*
 * The address at which a device reaches CPU physical address pa by DMA,
 * from its "pieboot,dma-ranges" (pieboot's translation of the device tree's
 * dma-ranges); pa itself when it has none. False if it cannot reach pa.
 */
bool RPiDMAAddress(IORegistryEntry *device, uint64_t pa, uint64_t *bus);

/*
 * The reverse: the CPU physical address that a device's DMA address bus
 * reaches. False if bus lies in none of its ranges.
 */
bool RPiCPUAddress(IORegistryEntry *device, uint64_t bus, uint64_t *pa);

/*
 * Every device tree node whose compatible (or name) is compat, in tree
 * order (the caller releases the array); NULL if there is none. Not
 * IODTFindMatchingEntries: a node that has not become a nub yet compares
 * only its name there.
 */
OSArray *RPiDTFindCompatible(const char *compat);

__END_DECLS

#endif /* _PIE_RPI_H_ */
