/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * piegpio.kext's classes: the GPIO family (PieGPIOController, one per
 * Linux gpiochip, and its two user clients) and its controller drivers.
 *
 * PieGPIOController keeps the state of each line (who holds it, its
 * settings, its event counts) and does all of its work on its IOWorkLoop,
 * through its IOCommandGate: the user clients' methods, the kernel's
 * platform functions, the interrupts' actions and the debounce timer.
 * A driver subclass supplies the hardware: the hw* primitives, called on
 * that work loop, for at most 64 lines, by offset on the chip. Pin levels
 * there are the pins'; the family applies active-low, emulates open-drain
 * and open-source outputs where the hardware has neither, and debounces in
 * software.
 */

#ifndef _PIEGPIO_CONTROLLER_H
#define _PIEGPIO_CONTROLLER_H

#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOSharedDataQueue.h>

#include "PieGPIO.h"

#define kPieGPIOMaxChipLines 64

class PieGPIOChipUserClient;
class PieGPIORequestUserClient;

class PieGPIOController : public IOService
{
	OSDeclareAbstractStructors(PieGPIOController);

public:
	struct Line {
		PieGPIORequestUserClient *owner;        /* retained while it holds the line */
		char consumer[kPieGPIONameSize];
		bool reserved;                          /* the kernel's (IOGPIOReservedLines, a platform function) */
		bool peripheral;                        /* given a peripheral's function by kPieGPIOSetPinctrl */
		bool stable;                            /* a debounced line's last steady logical value */
		PieGPIOLineSettings settings;           /* while it is requested */
		uint64_t lineSeqno;
		uint64_t debounceDeadline;              /* mach time; 0: none */
		/* A kernel driver's edges (kPieGPIORequestEdges), on a reserved line. */
		PieGPIOEdgeFn kernelFn;
		void *kernelRefcon;
		bool kernelActiveLow, kernelRising, kernelFalling;
	};

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	IOReturn gated(IOReturn (^work)(void));
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;
	virtual IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
	    IOUserClient **handler) APPLE_KEXT_OVERRIDE;
	virtual IOReturn callPlatformFunction(const OSSymbol *functionName, bool waitForFunction, void *param1,
	    void *param2, void *param3, void *param4) APPLE_KEXT_OVERRIDE;

	/* For the user clients; each runs on the work loop. */
	IOReturn getLineInfo(uint32_t offset, PieGPIOLineInfo *info);
	IOReturn watchLine(PieGPIOChipUserClient *client, uint32_t offset, PieGPIOLineInfo *info);
	IOReturn unwatchLine(PieGPIOChipUserClient *client, uint32_t offset);
	void removeChipClient(PieGPIOChipUserClient *client);
	IOReturn requestLines(PieGPIORequestUserClient *client, const PieGPIORequest *request);
	IOReturn reconfigureLines(PieGPIORequestUserClient *client, const PieGPIORequest *request);
	IOReturn getValues(PieGPIORequestUserClient *client, uint64_t mask, uint64_t *values);
	IOReturn setValues(PieGPIORequestUserClient *client, uint64_t mask, uint64_t values);
	void releaseLines(PieGPIORequestUserClient *client);

	/*
	 * An interrupt's work, on the work loop: the edges the hardware
	 * latched (hwTakeEdges), stamped with when the interrupt came. A
	 * driver whose controllers share an interrupt (PieGPIOBrcmstb's banks)
	 * calls it for each.
	 */
	void handleInterrupt(uint64_t monotonicNs, uint64_t realtimeNs);

	uint32_t lineCount(void) const { return fLineCount; }

protected:
	/* Set up the hardware (map the registers, set fLineCount); once, from start. */
	virtual bool hwStart(IOService *provider) = 0;
	/* How many of the provider's interrupts are the controller's (registered by start). */
	virtual unsigned hwInterruptCount(void) { return 0; }
	/* Make the line a GPIO (from a peripheral's function), as it is requested or driven. */
	virtual void hwClaim(unsigned line) { (void)line; }
	virtual void hwSetOutput(unsigned line, bool output) = 0;
	virtual bool hwIsOutput(unsigned line) = 0;
	virtual bool hwRead(unsigned line) = 0;
	virtual void hwWrite(unsigned line, bool high) = 0;
	/* GPIOD_LINE_BIAS_*: false if the hardware has no such bias (left as it was). */
	virtual bool hwSetBias(unsigned line, uint8_t bias) { (void)line; (void)bias; return false; }
	virtual uint8_t hwGetBias(unsigned line) { (void)line; return 2; /* unknown */ }
	/* Open-drain in hardware: false if it has none (the family emulates it). */
	virtual bool hwSetOpenDrain(unsigned line, bool openDrain) { (void)line; (void)openDrain; return false; }
	/* Latch edges of the pin (levels: the pin's rising and falling). */
	virtual void hwSetEdges(unsigned line, bool rising, bool falling) = 0;
	/* Whether edges reach this controller (default: it has an interrupt). */
	virtual bool hwCanDetectEdges(void);
	/* The lines whose edges were latched since the last call, cleared. */
	virtual uint64_t hwTakeEdges(void) = 0;
	/*
	 * Of those, the latched edge, if the hardware says (true; *rising),
	 * for a line that detects both.
	 */
	virtual bool hwEdgeType(unsigned line, bool *rising) { (void)line; (void)rising; return false; }
	/*
	 * Pin functions, for kPieGPIOSetPinctrl: the function select value for
	 * a device tree function name (generic "function") or a Broadcom
	 * "brcm,function" number, false if the line has no such function; the
	 * line given that function (a peripheral's: not software I/O); its
	 * pad's drive strength in mA, false if the hardware sets none.
	 */
	virtual bool hwFunctionNumber(unsigned line, const char *name, unsigned *fsel)
	{ (void)line; (void)name; (void)fsel; return false; }
	virtual bool hwBrcmFunction(unsigned line, uint32_t function, unsigned *fsel)
	{ (void)line; (void)function; (void)fsel; return false; }
	virtual void hwSetFunction(unsigned line, unsigned fsel) { (void)line; (void)fsel; }
	virtual bool hwSupportsBias(void) { return false; }
	virtual bool hwSupportsDrive(uint32_t mA) { (void)mA; return false; }
	virtual bool hwSetDrive(unsigned line, uint32_t mA) { (void)line; (void)mA; return false; }

	/*
	 * For start: the device tree node whose properties are the chip's
	 * (gpio-line-names, phandle, IOGPIOReservedLines), its number, label,
	 * and its lines' first index in the node's.
	 */
	virtual IOService *dtNode(void) { return getProvider(); }
	virtual unsigned chipNumber(void);
	virtual const char *chipLabel(void) = 0;
	virtual unsigned lineNamesFirst(void) { return 0; }

	uint32_t fLineCount;
	IOWorkLoop *fWorkLoop;

private:
	IOCommandGate *fGate;
	IOTimerEventSource *fDebounceTimer;
	IOFilterInterruptEventSource *fInterrupts[4];
	unsigned fInterruptCount;
	OSSet *fChipClients;
	Line fLines[kPieGPIOMaxChipLines];
	bool fHwOpenDrain[kPieGPIOMaxChipLines];
	char fNames[kPieGPIOMaxChipLines][kPieGPIONameSize];
	uint32_t fGroupID;
	_Atomic uint64_t fStampMono, fStampReal;

	static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source);
	static void interruptAction(OSObject *owner, IOInterruptEventSource *source, int count);
	static void debounceAction(OSObject *owner, IOTimerEventSource *timer);

	bool checkSettings(const PieGPIOLineSettings *s);
	IOReturn checkRequest(PieGPIORequestUserClient *client, const PieGPIORequest *request, bool reconfigure);
	void applySettings(unsigned line, const PieGPIOLineSettings *s, bool first);
	void drive(unsigned line, bool logical);
	bool logicalValue(unsigned line);
	void fillInfo(unsigned line, PieGPIOLineInfo *info);
	void postInfo(unsigned line, uint32_t type);
	void edge(unsigned line, bool rising, uint64_t monotonicNs, uint64_t realtimeNs);
	void armDebounce(void);
	void readProperties(void);
	bool mayRequest(task_t task);
	IOReturn setPinctrl(IORegistryEntry *config, const char *consumer);
	IOReturn releaseConsumer(const char *consumer);
	IOReturn requestKernelEdges(const PieGPIOKernelEdges *e);
	void kernelEdge(unsigned line);

	friend class PieGPIOChipUserClient;
	friend class PieGPIORequestUserClient;
};

/* A process's view of the chip: line info, and its changes for the lines it watches. */
class PieGPIOChipUserClient : public IOUserClient
{
	OSDeclareDefaultStructors(PieGPIOChipUserClient);

public:
	bool initWith(PieGPIOController *controller, task_t task, void *securityID, UInt32 type);
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options,
	    IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
	virtual IOReturn registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon)
	APPLE_KEXT_OVERRIDE;

	void post(const PieGPIOInfoEvent *event);
	uint64_t fWatched;

private:
	PieGPIOController *fController;
	IOSharedDataQueue *fQueue;
	mach_port_t fPort;

	static const IOExternalMethodDispatch sMethods[kPieGPIOChipMethodCount];
	static IOReturn sGetLineInfo(OSObject *target, void *ref, IOExternalMethodArguments *args);
	static IOReturn sWatchLineInfo(OSObject *target, void *ref, IOExternalMethodArguments *args);
	static IOReturn sUnwatchLineInfo(OSObject *target, void *ref, IOExternalMethodArguments *args);
};

/* A process's request: the lines it holds, their values, and their edges. */
class PieGPIORequestUserClient : public IOUserClient
{
	OSDeclareDefaultStructors(PieGPIORequestUserClient);

public:
	bool initWith(PieGPIOController *controller, task_t task, void *securityID, UInt32 type);
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options,
	    IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
	virtual IOReturn registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon)
	APPLE_KEXT_OVERRIDE;

	/* On the controller's work loop. */
	bool makeQueue(uint32_t entries);
	void post(PieGPIOEdgeEvent *event);

	char fConsumer[kPieGPIONameSize];
	uint32_t fNumLines;
	uint32_t fOffsets[kPieGPIOMaxLines];
	uint64_t fSeqno;
	bool fRequested;
	bool fClosed;                   /* clientClose: no request, ever */

private:
	PieGPIOController *fController;
	IOSharedDataQueue *fQueue;
	mach_port_t fPort;

	static const IOExternalMethodDispatch sMethods[kPieGPIORequestMethodCount];
	static IOReturn sRequestLines(OSObject *target, void *ref, IOExternalMethodArguments *args);
	static IOReturn sGetValues(OSObject *target, void *ref, IOExternalMethodArguments *args);
	static IOReturn sSetValues(OSObject *target, void *ref, IOExternalMethodArguments *args);
	static IOReturn sReconfigureLines(OSObject *target, void *ref, IOExternalMethodArguments *args);
};

#endif /* _PIEGPIO_CONTROLLER_H */
