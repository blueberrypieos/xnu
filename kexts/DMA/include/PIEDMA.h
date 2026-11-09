/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <PIEDMA.h>: PIEDMA.kext's DMA controllers, for kernel drivers whose
 * device tree nodes name a channel (dmas = <&controller cell>, dma-names).
 *
 * Each controller is a PIEDMAController service whose
 * kPIEDMAPHandleKey property is its device tree node's phandle. A
 * driver calls its platform functions (IOService::callPlatformFunction;
 * <PIEDMAKernel.h> has inline helpers, so no link-time dependency):
 *
 * - kPIEDMAAllocate (PIEDMARequest *): a channel for one direction
 *   between memory and the device's register, paced by the device's
 *   request line (the dmas cell); r->channel is set. kIOReturnNoResources
 *   if every channel is taken.
 * - kPIEDMAStart (channel, uint64_t *pa, uint32_t *length, IOBufferMemoryDescriptor *memory): one
 *   transfer of length bytes from or to CPU physical address pa.
 *   memory is retained until confirmed idle; pa must lie in its physical segment.
 *   pa: physically contiguous, below r->maxAddress, not in the CPU's cache
 *   (an uncached or write-combined mapping, flushed once: the controllers
 *   are not coherent), length at most r->maxLength and a multiple of the
 *   register's width.
 * - kPIEDMAWait (channel, uint64_t *timeoutNs): until it ends:
 *   kIOReturnSuccess, kIOReturnIOError (the controller's error), or
 *   kIOReturnTimeout (it is aborted). Not from the controller's work loop.
 *   kIOReturnNotResponding means unconfirmed quiescence: channel, CBs and
 *   payload pages are quarantined until reboot; do not reuse the peripheral.
 * - kPIEDMARelease (channel): the channel free again (aborted if running).
 *
 * A transfer's channel is the caller's alone: Start and Wait in turn, from
 * one thread at a time.
 */

#ifndef _PIEDMA_H_
#define _PIEDMA_H_

#include <stdint.h>

#define kPIEDMAControllerClass        "PIEDMAController"
#define kPIEDMAPHandleKey             "IODMAPHandle"

#define kPIEDMAAllocate               "PIEDMAAllocate"
#define kPIEDMAControllerQuarantineKey "PIEDMAControllerQuarantined"
#define kPIEDMAPeripheralQuarantineKey "PIEDMAPeripheralQuarantined"
#define kPIEDMAStart                  "PIEDMAStart"
#define kPIEDMAWait                   "PIEDMAWait"
#define kPIEDMARelease                "PIEDMARelease"

#ifdef KERNEL
class IORegistryEntry;

typedef struct PIEDMARequest {
	/* In. */
	IORegistryEntry *client;        /* the device's node: its reg (its registers' first range) */
	uint32_t cell;                  /* its dmas cell for this direction (the request line) */
	uint32_t registerOffset;        /* the data register's offset in the device's registers */
	uint32_t width;                 /* the data register's width in bytes: 1, 2 or 4 */
	uint32_t burst;                 /* the device's burst, in registers' widths (0: 1) */
	bool toDevice;                  /* memory to the device; else the device to memory */
	/* Out. */
	void *channel;
	uint64_t maxAddress;            /* the highest CPU physical address the controller reaches */
	uint32_t maxLength;             /* the longest transfer */
} PIEDMARequest;
#endif

#endif /* _PIEDMA_H_ */
