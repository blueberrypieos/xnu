/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_SDMMC_PIESDIO_H
#define _IOKIT_SDMMC_PIESDIO_H

#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOService.h>
#include <kern/thread.h>

class PIESDIOHost;

/* One function of an SDIO card, the nub its driver matches. <pie/sdio.h> takes it as struct pie_sdio_func. */
class PIESDIOFunction : public IOService
{
	OSDeclareDefaultStructors(PIESDIOFunction);

public:
	PIESDIOHost *host;
	uint8_t number;
	uint16_t vendor;
	uint16_t device;
	uint16_t maxBlockSize;
	uint16_t blockSize;
	void (*handler)(void *);
	void *handlerArg;
	void (*resetHandler)(void *);
	void *resetArg;
	thread_t resetThread;
};

/*
 * An SDIO card behind a host controller: power-up and enumeration, the
 * function nubs, CMD52 and CMD53 with their splitting, the bus claim and
 * the interrupt thread. A host driver subclasses it and supplies the
 * sdio* operations, which are called with the bus claimed. The host calls
 * sdioInterrupt() when the card interrupts, with the interrupt masked;
 * the bus layer unmasks it with sdioInterruptEnable() once the function
 * drivers' handlers have returned, and after it sets the card up again.
 */
class PIESDIOHost : public IOService
{
	OSDeclareAbstractStructors(PIESDIOHost);

public:
	enum SDIOResponse { kSDIORespNone, kSDIORespR1, kSDIORespR4, kSDIORespR5 };
	enum {
		kSDIOClockInitHz      = 400000,
		kSDIOClockRunHz       = 25000000,
		kSDIOClockHighSpeedHz = 50000000,
		kSDIOMaxFunctions     = 7,
	};

	int claimBus(void);
	void releaseBus(void);
	bool busClaimed(void);
	int io52(unsigned fn, uint32_t addr, bool write, uint8_t value, uint8_t *out);
	int io53(PIESDIOFunction *f, uint32_t addr, void *buf, size_t len, bool write, bool incr);
	int enableFunction(PIESDIOFunction *f, bool on);
	int setBlockSize(PIESDIOFunction *f, unsigned size);
	size_t alignSize(PIESDIOFunction *f, size_t len);
	size_t maxRequest(PIESDIOFunction *f);
	int setHandler(PIESDIOFunction *f, void (*handler)(void *), void *arg);
	void setResetHandler(PIESDIOFunction *f, void (*handler)(void *), void *arg);
	int resetCard(PIESDIOFunction *f);
	unsigned cardGeneration(void);
	uint32_t clockHz(void) { return sdioClock; }
	PIESDIOFunction *function(unsigned fn) { return fn >= 1 && fn <= functions && fn <= kSDIOMaxFunctions ? funcs[fn] : NULL; }

protected:
	virtual bool sdioPowerUp(void) = 0;
	virtual bool sdioSetClock(uint32_t hz) = 0;
	virtual void sdioSetBusWidth4(void) = 0;
	virtual bool sdioSetHighSpeed(bool on) = 0;
	virtual IOReturn sdioCommand(uint32_t index, uint32_t arg, SDIOResponse type, uint32_t resp[4]) = 0;
	virtual IOReturn sdioData(uint32_t arg, void *buf, uint32_t blksz, uint32_t nblk, bool write, uint32_t *r5) = 0;
	virtual void sdioInterruptEnable(bool on) = 0;
	virtual void sdioRecover(void) = 0;

	uint32_t sdioClock;

	bool sdioStartBus(void);
	void sdioStopBus(void);
	void sdioInterrupt(void);
	bool sdioDead(void) { return dead; }

public:
	virtual void free(void) APPLE_KEXT_OVERRIDE;

private:
	IOLock *busLock;
	thread_t busOwner;
	IOLock *irqLock;
	bool irqPending;
	bool resetPending;
	uint8_t resetAsked;
	bool stopping;
	bool polling;
	int threadsRunning;
	bool dead;
	int recoveries;
	unsigned generation;
	uint32_t rca;
	uint8_t cccrRevision;
	uint8_t cardCaps;
	uint8_t functions;
	uint8_t enabled;
	uint8_t interrupts;
	uint32_t commonCIS;
	uint16_t vendor;
	uint16_t device;
	uint16_t f0MaxBlock;
	PIESDIOFunction *funcs[kSDIOMaxFunctions + 1];

	bool initCard(void);
	bool readCIS(uint32_t addr, uint16_t *vendorOut, uint16_t *deviceOut, uint16_t *maxBlock, bool function);
	void publishFunctions(void);
	int restoreFunctions(void);
	void recover(unsigned fn);
	bool reinitCard(PIESDIOFunction *asker = NULL);
	int transfer(PIESDIOFunction *f, uint32_t addr, uint8_t *buf, uint32_t len, uint32_t blksz, bool write, bool incr);
	void dispatch(void);
	void announceReset(uint8_t skip);
	static void initThread(void *arg, wait_result_t wr);
	static void irqThread(void *arg, wait_result_t wr);
	void threadDone(void);
};

#endif
