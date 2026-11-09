/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PIEDMA.kext's classes: PIEDMAController, one per device tree DMA
 * node, keeps its channels (who has each, its control blocks, its
 * transfer's state) and serves <PIEDMA.h>'s platform functions; its
 * subclasses drive the hardware: PIEDMABCM2835 (the Broadcom DMA's
 * legacy, LITE and 40-bit DMA4 channels, BCM2711 and BCM2712) and
 * PIEDMADWAxi (RP1's Synopsys DesignWare AXI DMAC).
 *
 * A transfer is a chain of control blocks (a channel's own, in one
 * uncached page or so) the hardware follows by itself; its end or its
 * error raises the channel's interrupt, whose primary filter quenches it
 * (hwFilter) and whose action, on the work loop, ends the transfer
 * (hwService, complete), waking the thread in kPIEDMAWait.
 */

#ifndef _PIEDMA_CONTROLLER_H
#define _PIEDMA_CONTROLLER_H

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOService.h>
#include <IOKit/IOWorkLoop.h>

#include "PIEDMA.h"

#define kDMAMaxChannels         16
#define kDMAMaxInterrupts       16
#define kDMACBPages             2       /* each channel's control blocks: 8 KB */

enum { kDMAMemToDev, kDMADevToMem, kDMAMemToMem };

class PIEDMAController : public IOService
{
	OSDeclareAbstractStructors(PIEDMAController);

public:
	struct Channel {
		unsigned number;                /* the hardware's */
		bool usable;                    /* in the node's channel mask */
		bool allocated;
		uint32_t kind;                  /* the subclass's */
		uint32_t flags;                 /* the subclass's (e.g. RP1's chan-flags) */
		volatile uint8_t *regs;         /* the channel's registers */
		/* The client's request. */
		bool toDevice;
		uint64_t device;                /* the data register's bus address */
		uint32_t width, burst, cell;
		/* The control blocks. */
		IOBufferMemoryDescriptor *cbMem;
		IOMemoryMap *cbMap;
		volatile uint8_t *cb;
		uint64_t cbBus;
		/* The transfer: on the work loop; done and status also under fLock, for the waiter. */
		bool running, done, quarantined;
		IOBufferMemoryDescriptor *payload[2];
		IORegistryEntry *client;
		IOReturn status;
		uint64_t transfers, errors;
	};

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE { return fWorkLoop; }
	virtual IOReturn callPlatformFunction(const OSSymbol *functionName, bool waitForFunction, void *param1,
	    void *param2, void *param3, void *param4) APPLE_KEXT_OVERRIDE;

	/* debug.dma.selftest: n memory-to-memory copies on each free channel; a line for each into out. */
	bool selfTest(unsigned n, char *out, size_t outLen);
	/* debug.dma.channels: a line for each usable channel. */
	void describe(char *out, size_t outLen);

	char fName[48];

protected:
	/* Map the registers, set fChannels and fCount, add the interrupts; once, from start. */
	virtual bool hwStart(IOService *provider) = 0;
	/* The highest CPU physical address its memory side reaches. */
	virtual uint64_t hwMaxAddress(void) = 0;
	/* The longest transfer on a channel (its control blocks' worth). */
	virtual uint32_t hwMaxLength(Channel &c) = 0;
	/* Bus addresses: of memory (CPU physical pa), and of a device's register. */
	virtual bool hwMemoryAddress(uint64_t pa, uint64_t *bus) = 0;
	virtual bool hwDeviceAddress(IORegistryEntry *client, uint32_t offset, uint64_t *bus) = 0;
	/* Whether a channel can serve a request (width, burst), and how well (higher better; < 0 never). */
	virtual int hwFit(Channel &c, const PIEDMARequest *r) { (void)c; (void)r; return 0; }
	/* Build the control blocks for a transfer (bus addresses) and start it; on the work loop. */
	virtual IOReturn hwTransfer(Channel &c, int direction, uint64_t src, uint64_t dst, uint32_t length) = 0;
	/* Stop a channel's transfer at once; on the work loop. */
	virtual bool hwAbort(Channel &c) = 0;
	/* Primary interrupt context: quench interrupt line's sources (no locks). */
	virtual void hwFilter(unsigned line) = 0;
	/* On the work loop: the line's channels' ends and errors (complete). */
	virtual void hwService(unsigned line) = 0;
	/* How much memory each channel's control blocks take (page-aligned, so any CB alignment holds). */
	virtual uint32_t hwCBBytes(void) { return kDMACBPages * 4096; }

	/* For the subclasses: an interrupt (the provider's index) as line; a transfer ended. */
	bool addInterrupt(IOService *provider, unsigned index, unsigned line);
	void complete(Channel &c, IOReturn status);
	bool isQEMU(void);

	IOMemoryMap *fMap;
	volatile uint8_t *fRegs;
	uint64_t fPhys;                 /* the registers' CPU physical address */
	Channel fChannels[kDMAMaxChannels];
	unsigned fCount;
	IOWorkLoop *fWorkLoop;
	IOSimpleLock *fHardwareLock;

private:
	IOCommandGate *fGate;
	IOLock *fLock;
	IOFilterInterruptEventSource *fInterrupts[kDMAMaxInterrupts];
	unsigned fLines[kDMAMaxInterrupts];
	unsigned fInterruptCount;
	bool fListed, fGateAdded, fStopping;
	void cleanup(void);

	Channel *channelFor(void *channel);
	IOReturn allocate(PIEDMARequest *r, bool internal);
	IOReturn startTransfer(Channel &c, int direction, uint64_t src, uint64_t dst, uint32_t length,
	    IOBufferMemoryDescriptor *sourceMemory, IOBufferMemoryDescriptor *destinationMemory);
	IOReturn wait(Channel &c, uint64_t timeoutNs);
	void release(Channel &c);
	bool setUpCB(Channel &c);
	static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source);
	static void interruptAction(OSObject *owner, IOInterruptEventSource *source, int count);
};

#endif /* _PIEDMA_CONTROLLER_H */
