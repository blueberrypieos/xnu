/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PieGPIOController: the GPIO family (see PieGPIOController.h and
 * <PieGPIO.h>). What a line does follows Linux's GPIO character device, as
 * libgpiod sees it: a request holds its lines until it closes, which keep
 * their configuration after; edge detection needs an input, a drive mode
 * other than push-pull an output, a bias a direction; active-low turns
 * values and edges round; a debounced line reports an edge once its new
 * level has held for the period; each event is numbered in its request and
 * on its line.
 */

#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <sys/kauth.h>
#include <sys/proc.h>
#include <libkern/OSByteOrder.h>

#include "PieGPIOController.h"

OSDefineMetaClassAndAbstractStructors(PieGPIOController, IOService);
#define super IOService

enum { AS_IS = 1, INPUT = 2, OUTPUT = 3 };                       /* direction */
enum { EDGE_NONE = 1, EDGE_RISING = 2, EDGE_FALLING = 3, EDGE_BOTH = 4 };
enum { BIAS_AS_IS = 1, BIAS_UNKNOWN = 2, BIAS_DISABLED = 3, BIAS_PULL_UP = 4, BIAS_PULL_DOWN = 5 };
enum { PUSH_PULL = 1, OPEN_DRAIN = 2, OPEN_SOURCE = 3 };
enum { CLOCK_MONO = 1, CLOCK_REAL = 2, CLOCK_HTE = 3 };

static uint64_t
monotonic_ns(void)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_continuous_time(), &ns);
	return ns;
}

static uint64_t
realtime_ns(void)
{
	clock_sec_t s;
	clock_nsec_t ns;
	clock_get_calendar_nanotime(&s, &ns);
	return (uint64_t)s * NSEC_PER_SEC + ns;
}

IOReturn
PieGPIOController::gated(IOReturn (^work)(void))
{
	return fGate != NULL ? fGate->runActionBlock(work) : kIOReturnNotReady;
}

IOWorkLoop *
PieGPIOController::getWorkLoop(void) const
{
	return fWorkLoop;
}

bool
PieGPIOController::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	if (fWorkLoop == NULL && (fWorkLoop = IOWorkLoop::workLoop()) == NULL) {
		return false;
	}
	if (!hwStart(provider) || fLineCount == 0 || fLineCount > kPieGPIOMaxChipLines) {
		IOLog("%s: no lines it can drive (%u)\n", getName(), fLineCount);
		return false;
	}
	fGate = IOCommandGate::commandGate(this);
	fDebounceTimer = IOTimerEventSource::timerEventSource(this, debounceAction);
	fChipClients = OSSet::withCapacity(2);
	if (fGate == NULL || fDebounceTimer == NULL || fChipClients == NULL ||
	    fWorkLoop->addEventSource(fGate) != kIOReturnSuccess ||
	    fWorkLoop->addEventSource(fDebounceTimer) != kIOReturnSuccess) {
		return false;
	}
	unsigned want = hwInterruptCount();
	for (unsigned i = 0; i < want && i < sizeof(fInterrupts) / sizeof(fInterrupts[0]); i++) {
		IOFilterInterruptEventSource *src = IOFilterInterruptEventSource::filterInterruptEventSource(this,
		    interruptAction, interruptFilter, provider, (int)i);
		if (src == NULL || fWorkLoop->addEventSource(src) != kIOReturnSuccess) {
			OSSafeReleaseNULL(src);
			IOLog("%s: no interrupt %u: no edge events\n", getName(), i);
			break;
		}
		src->enable();
		fInterrupts[fInterruptCount++] = src;
	}
	readProperties();
	registerService();
	IOLog("%s: %s [%s], %u lines\n", getName(),
	    ((OSString *)getProperty(kPieGPIOChipNameKey))->getCStringNoCopy(), chipLabel(), fLineCount);
	return true;
}

void
PieGPIOController::stop(IOService *provider)
{
	for (unsigned i = 0; i < fInterruptCount; i++) {
		fInterrupts[i]->disable();
		fWorkLoop->removeEventSource(fInterrupts[i]);
		OSSafeReleaseNULL(fInterrupts[i]);
	}
	fInterruptCount = 0;
	if (fDebounceTimer != NULL) {
		fDebounceTimer->cancelTimeout();
		fWorkLoop->removeEventSource(fDebounceTimer);
	}
	if (fGate != NULL) {
		fWorkLoop->removeEventSource(fGate);
	}
	super::stop(provider);
}

void
PieGPIOController::free(void)
{
	OSSafeReleaseNULL(fDebounceTimer);
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fChipClients);
	OSSafeReleaseNULL(fWorkLoop);
	super::free();
}

#pragma mark properties

unsigned
PieGPIOController::chipNumber(void)
{
	OSNumber *n = OSDynamicCast(OSNumber, getProperty("IOGPIOChipBase"));
	return n != NULL ? n->unsigned32BitValue() : 0;
}

/*
 * What does not change, as properties (<PieGPIO.h>), and the lines the
 * kernel keeps for itself (IOGPIOReservedLines on the device tree node:
 * line number -> consumer).
 */
void
PieGPIOController::readProperties(void)
{
	IOService *node = dtNode();
	char name[16];
	snprintf(name, sizeof(name), "gpiochip%u", chipNumber());
	setProperty(kPieGPIOChipNameKey, name);
	setProperty(kPieGPIOChipLabelKey, chipLabel());
	setProperty(kPieGPIOLineCountKey, fLineCount, 32);

	OSData *names = OSDynamicCast(OSData, node->getProperty("gpio-line-names"));
	const char *p = names != NULL ? (const char *)names->getBytesNoCopy() : NULL;
	const char *end = p != NULL ? p + names->getLength() : NULL;
	OSArray *list = OSArray::withCapacity(fLineCount);
	for (unsigned i = 0, skip = lineNamesFirst(); i < fLineCount; ) {
		const char *n = "";
		if (p != NULL && p < end) {
			n = p;
			p += strnlen(p, (size_t)(end - p)) + 1;
			if (skip > 0) {
				skip--;
				continue;
			}
		}
		strlcpy(fNames[i], n, sizeof(fNames[i]));
		OSString *s = OSString::withCString(fNames[i]);
		if (list != NULL && s != NULL) {
			list->setObject(s);
		}
		OSSafeReleaseNULL(s);
		i++;
	}
	if (list != NULL) {
		setProperty(kPieGPIOLineNamesKey, list);
		list->release();
	}

	OSData *phandle = OSDynamicCast(OSData, node->getProperty("phandle"));
	bool appleHandle = false;
	if (phandle == NULL) {
		/* pieboot keeps FDT handles little-endian in AAPL,phandle. */
		phandle = OSDynamicCast(OSData, node->getProperty("AAPL,phandle"));
		appleHandle = true;
	}
	if (phandle != NULL && phandle->getLength() == sizeof(uint32_t) && lineNamesFirst() == 0) {
		uint32_t handle = appleHandle ? OSReadLittleInt32(phandle->getBytesNoCopy(), 0) :
		    OSReadBigInt32(phandle->getBytesNoCopy(), 0);
		setProperty(kPieGPIOPHandleKey, handle, 32);
	}

	OSDictionary *reserved = OSDynamicCast(OSDictionary, node->getProperty("IOGPIOReservedLines"));
	OSCollectionIterator *it = reserved != NULL ? OSCollectionIterator::withCollection(reserved) : NULL;
	OSSymbol *key;
	while (it != NULL && (key = OSDynamicCast(OSSymbol, it->getNextObject())) != NULL) {
		unsigned long line = strtoul(key->getCStringNoCopy(), NULL, 10);
		OSString *consumer = OSDynamicCast(OSString, reserved->getObject(key));
		if (line < lineNamesFirst() || line - lineNamesFirst() >= fLineCount) {
			continue;
		}
		Line &l = fLines[line - lineNamesFirst()];
		l.reserved = true;
		strlcpy(l.consumer, consumer != NULL ? consumer->getCStringNoCopy() : "kernel", sizeof(l.consumer));
	}
	OSSafeReleaseNULL(it);
	/* RPiGEM drives RP1's line 32, the PHY's reset, and publishes no reservation. */
	if (strcmp(chipLabel(), "pinctrl-rp1") == 0 && fLineCount > 32 && !fLines[32].reserved) {
		fLines[32].reserved = true;
		strlcpy(fLines[32].consumer, "RPiGEM", sizeof(fLines[32].consumer));
	}

	OSNumber *gid = OSDynamicCast(OSNumber, getProperty("GPIOGroupID"));
	fGroupID = gid != NULL ? gid->unsigned32BitValue() : 0;
}

#pragma mark connections

/* Requests: root, or a member of the group the personality names (gpio). */
bool
PieGPIOController::mayRequest(task_t task)
{
	if (IOUserClient::clientHasPrivilege(task, kIOClientPrivilegeAdministrator) == kIOReturnSuccess) {
		return true;
	}
	if (fGroupID == 0) {
		return false;
	}
	/* The caller's: IOServiceOpen runs on its thread. */
	proc_t p = proc_self();
	if (p == NULL) {
		return false;
	}
	kauth_cred_t cred = kauth_cred_proc_ref(p);
	int member = 0;
	bool ok = kauth_cred_ismember_gid(cred, fGroupID, &member) == 0 && member;
	kauth_cred_unref(&cred);
	proc_rele(p);
	return ok;
}

IOReturn
PieGPIOController::newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *,
    IOUserClient **handler)
{
	IOUserClient *client = NULL;
	if (type == kPieGPIOChipClient) {
		PieGPIOChipUserClient *c = OSTypeAlloc(PieGPIOChipUserClient);
		if (c != NULL && !c->initWith(this, owningTask, securityID, type)) {
			OSSafeReleaseNULL(c);
		}
		client = c;
	} else if (type == kPieGPIORequestClient) {
		if (!mayRequest(owningTask)) {
			return kIOReturnNotPrivileged;
		}
		PieGPIORequestUserClient *c = OSTypeAlloc(PieGPIORequestUserClient);
		if (c != NULL && !c->initWith(this, owningTask, securityID, type)) {
			OSSafeReleaseNULL(c);
		}
		client = c;
	} else {
		return kIOReturnBadArgument;
	}
	if (client == NULL) {
		return kIOReturnNoMemory;
	}
	if (!client->attach(this)) {
		client->release();
		return kIOReturnError;
	}
	if (!client->start(this)) {
		client->detach(this);
		client->release();
		return kIOReturnError;
	}
	if (type == kPieGPIOChipClient) {
		gated(^IOReturn {
			fChipClients->setObject(client);
			return kIOReturnSuccess;
		});
	}
	*handler = client;
	return kIOReturnSuccess;
}

void
PieGPIOController::removeChipClient(PieGPIOChipUserClient *client)
{
	gated(^IOReturn {
		fChipClients->removeObject(client);
		return kIOReturnSuccess;
	});
}

#pragma mark lines

bool
PieGPIOController::logicalValue(unsigned line)
{
	return hwRead(line) != (fLines[line].owner != NULL && fLines[line].settings.activeLow);
}

/* An output's value: open-drain and open-source by direction where the hardware has neither. */
void
PieGPIOController::drive(unsigned line, bool logical)
{
	const PieGPIOLineSettings &s = fLines[line].settings;
	bool pin = logical != (s.activeLow != 0);
	if (s.drive == OPEN_DRAIN && !fHwOpenDrain[line]) {
		if (pin) {
			hwSetOutput(line, false);
		} else {
			hwWrite(line, false);
			hwSetOutput(line, true);
		}
	} else if (s.drive == OPEN_SOURCE) {
		if (pin) {
			hwWrite(line, true);
			hwSetOutput(line, true);
		} else {
			hwSetOutput(line, false);
		}
	} else {
		hwWrite(line, pin);
		hwSetOutput(line, true);
	}
}

void
PieGPIOController::fillInfo(unsigned line, PieGPIOLineInfo *info)
{
	const Line &l = fLines[line];
	bzero(info, sizeof(*info));
	info->version = kPieGPIOABIVersion;
	info->size = sizeof(*info);
	info->offset = line;
	strlcpy(info->name, fNames[line], sizeof(info->name));
	if (l.owner != NULL) {
		info->flags = kPieGPIOLineUsed;
		strlcpy(info->consumer, l.owner->fConsumer, sizeof(info->consumer));
		info->settings = l.settings;
		info->settings.output = 0;
	} else {
		if (l.reserved) {
			info->flags = kPieGPIOLineUsed;
			strlcpy(info->consumer, l.consumer, sizeof(info->consumer));
		}
		info->settings.direction = AS_IS;
		info->settings.edge = EDGE_NONE;
		info->settings.bias = BIAS_AS_IS;
		info->settings.drive = PUSH_PULL;
		info->settings.clock = CLOCK_MONO;
	}
	/* What the hardware has where nothing was asked for. */
	if (info->settings.direction == AS_IS) {
		info->settings.direction = hwIsOutput(line) ? OUTPUT : INPUT;
	}
	if (info->settings.bias == BIAS_AS_IS) {
		info->settings.bias = hwGetBias(line);
	}
}

IOReturn
PieGPIOController::getLineInfo(uint32_t offset, PieGPIOLineInfo *info)
{
	return gated(^IOReturn {
		if (offset >= fLineCount) {
			return kIOReturnBadArgument;
		}
		fillInfo(offset, info);
		return kIOReturnSuccess;
	});
}

IOReturn
PieGPIOController::watchLine(PieGPIOChipUserClient *client, uint32_t offset, PieGPIOLineInfo *info)
{
	return gated(^IOReturn {
		if (offset >= fLineCount) {
			return kIOReturnBadArgument;
		}
		if (client->fWatched & (1ULL << offset)) {
			return kIOReturnBusy;
		}
		client->fWatched |= 1ULL << offset;
		fillInfo(offset, info);
		return kIOReturnSuccess;
	});
}

IOReturn
PieGPIOController::unwatchLine(PieGPIOChipUserClient *client, uint32_t offset)
{
	return gated(^IOReturn {
		if (offset >= fLineCount) {
			return kIOReturnBadArgument;
		}
		if (!(client->fWatched & (1ULL << offset))) {
			return kIOReturnBusy;
		}
		client->fWatched &= ~(1ULL << offset);
		return kIOReturnSuccess;
	});
}

void
PieGPIOController::postInfo(unsigned line, uint32_t type)
{
	PieGPIOInfoEvent ev;
	bzero(&ev, sizeof(ev));
	ev.timestampNs = monotonic_ns();
	ev.type = type;
	fillInfo(line, &ev.info);
	OSCollectionIterator *it = OSCollectionIterator::withCollection(fChipClients);
	PieGPIOChipUserClient *c;
	while (it != NULL && (c = OSDynamicCast(PieGPIOChipUserClient, it->getNextObject())) != NULL) {
		if (c->fWatched & (1ULL << line)) {
			c->post(&ev);
		}
	}
	OSSafeReleaseNULL(it);
}

bool
PieGPIOController::checkSettings(const PieGPIOLineSettings *s)
{
	if (s->direction < AS_IS || s->direction > OUTPUT || s->edge < EDGE_NONE || s->edge > EDGE_BOTH ||
	    s->bias < BIAS_AS_IS || s->bias > BIAS_PULL_DOWN || s->bias == BIAS_UNKNOWN ||
	    s->drive < PUSH_PULL || s->drive > OPEN_SOURCE || s->clock < CLOCK_MONO || s->clock > CLOCK_HTE) {
		return false;
	}
	/* As Linux's character device: edges on inputs, drive modes on outputs, a bias with a direction. */
	if (s->edge != EDGE_NONE && s->direction != INPUT) {
		return false;
	}
	if (s->drive != PUSH_PULL && s->direction != OUTPUT) {
		return false;
	}
	if (s->bias != BIAS_AS_IS && s->direction == AS_IS) {
		return false;
	}
	return true;
}

IOReturn
PieGPIOController::checkRequest(PieGPIORequestUserClient *client, const PieGPIORequest *r, bool reconfigure)
{
	if (r->version != kPieGPIOABIVersion || r->size != sizeof(*r) || r->numLines == 0 ||
	    r->numLines > kPieGPIOMaxLines) {
		return kIOReturnBadArgument;
	}
	if (reconfigure && (r->numLines != client->fNumLines ||
	    memcmp(r->offsets, client->fOffsets, r->numLines * sizeof(r->offsets[0])) != 0)) {
		return kIOReturnBadArgument;
	}
	uint64_t seen = 0;
	for (uint32_t i = 0; i < r->numLines; i++) {
		uint32_t line = r->offsets[i];
		if (line >= fLineCount || (seen & (1ULL << line)) || !checkSettings(&r->settings[i])) {
			return kIOReturnBadArgument;
		}
		seen |= 1ULL << line;
		if (r->settings[i].clock == CLOCK_HTE) {
			return kIOReturnUnsupported;
		}
		if (r->settings[i].edge != EDGE_NONE && !hwCanDetectEdges()) {
			return kIOReturnUnsupported;
		}
		if (!reconfigure && (fLines[line].owner != NULL || fLines[line].reserved)) {
			return kIOReturnBusy;
		}
	}
	return kIOReturnSuccess;
}

bool
PieGPIOController::hwCanDetectEdges(void)
{
	return fInterruptCount > 0;
}

/* A requested line's settings, into the hardware. */
void
PieGPIOController::applySettings(unsigned line, const PieGPIOLineSettings *s, bool first)
{
	Line &l = fLines[line];
	uint8_t direction = s->direction;
	if (direction == AS_IS) {
		direction = first ? (hwIsOutput(line) ? OUTPUT : INPUT) : l.settings.direction;
	}
	if (first) {
		hwClaim(line);
	}
	l.settings = *s;
	l.settings.direction = direction;
	if (direction != INPUT) {
		l.settings.debounceUs = 0;      /* Linux ignores it there */
	}
	if (s->direction == OUTPUT) {
		fHwOpenDrain[line] = hwSetOpenDrain(line, s->drive == OPEN_DRAIN);
		drive(line, s->output != 0);
	} else if (s->direction == INPUT) {
		fHwOpenDrain[line] = false;
		hwSetOpenDrain(line, false);
		hwSetOutput(line, false);
	}
	/* Logical edges on the pin: an active-low line's rising edge is the pin's falling one. */
	bool rising = s->edge == EDGE_RISING || s->edge == EDGE_BOTH;
	bool falling = s->edge == EDGE_FALLING || s->edge == EDGE_BOTH;
	if (s->edge != EDGE_NONE && l.settings.debounceUs != 0) {
		rising = falling = true;
		l.stable = logicalValue(line);
	}
	l.debounceDeadline = 0;
	if (s->activeLow) {
		bool t = rising;
		rising = falling;
		falling = t;
	}
	hwSetEdges(line, rising, falling);
	/* Arm before changing the pull: the pull itself may generate an edge. */
	if (s->bias != BIAS_AS_IS) {
		hwSetBias(line, s->bias);
	}
}

IOReturn
PieGPIOController::requestLines(PieGPIORequestUserClient *client, const PieGPIORequest *request)
{
	return gated(^IOReturn {
		if (client->fClosed) {
			return kIOReturnNotOpen;
		}
		if (client->fRequested) {
			return kIOReturnBusy;
		}
		IOReturn ret = checkRequest(client, request, false);
		if (ret != kIOReturnSuccess) {
			return ret;
		}
		uint32_t entries = request->eventBufferSize != 0 ? request->eventBufferSize : 16 * request->numLines;
		if (entries > kPieGPIOMaxEventBuffer) {
			entries = kPieGPIOMaxEventBuffer;
		}
		if (!client->makeQueue(entries)) {
			return kIOReturnNoMemory;
		}
		strlcpy(client->fConsumer, request->consumer[0] != '\0' ? request->consumer : "?", sizeof(client->fConsumer));
		client->fNumLines = request->numLines;
		memcpy(client->fOffsets, request->offsets, sizeof(client->fOffsets));
		client->fRequested = true;
		client->retain();
		for (uint32_t i = 0; i < request->numLines; i++) {
			unsigned line = request->offsets[i];
			fLines[line].owner = client;
			fLines[line].lineSeqno = 0;
			applySettings(line, &request->settings[i], true);
			postInfo(line, kPieGPIOLineRequested);
		}
		armDebounce();
		return kIOReturnSuccess;
	});
}

IOReturn
PieGPIOController::reconfigureLines(PieGPIORequestUserClient *client, const PieGPIORequest *request)
{
	return gated(^IOReturn {
		if (!client->fRequested) {
			return kIOReturnNotOpen;
		}
		IOReturn ret = checkRequest(client, request, true);
		if (ret != kIOReturnSuccess) {
			return ret;
		}
		for (uint32_t i = 0; i < request->numLines; i++) {
			applySettings(request->offsets[i], &request->settings[i], false);
			postInfo(request->offsets[i], kPieGPIOLineConfigChanged);
		}
		armDebounce();
		return kIOReturnSuccess;
	});
}

IOReturn
PieGPIOController::getValues(PieGPIORequestUserClient *client, uint64_t mask, uint64_t *values)
{
	return gated(^IOReturn {
		if (!client->fRequested) {
			return kIOReturnNotOpen;
		}
		if (client->fNumLines < 64 && (mask >> client->fNumLines) != 0) {
			return kIOReturnBadArgument;
		}
		uint64_t v = 0;
		for (uint32_t i = 0; i < client->fNumLines; i++) {
			if ((mask & (1ULL << i)) && logicalValue(client->fOffsets[i])) {
				v |= 1ULL << i;
			}
		}
		*values = v;
		return kIOReturnSuccess;
	});
}

IOReturn
PieGPIOController::setValues(PieGPIORequestUserClient *client, uint64_t mask, uint64_t values)
{
	return gated(^IOReturn {
		if (!client->fRequested) {
			return kIOReturnNotOpen;
		}
		if (client->fNumLines < 64 && (mask >> client->fNumLines) != 0) {
			return kIOReturnBadArgument;
		}
		for (uint32_t i = 0; i < client->fNumLines; i++) {
			if ((mask & (1ULL << i)) && fLines[client->fOffsets[i]].settings.direction != OUTPUT) {
				return kIOReturnNotPermitted;
			}
		}
		for (uint32_t i = 0; i < client->fNumLines; i++) {
			if (mask & (1ULL << i)) {
				drive(client->fOffsets[i], (values >> i) & 1);
			}
		}
		return kIOReturnSuccess;
	});
}

/* A request's end: its lines are free, as they are (Linux leaves their configuration too). */
void
PieGPIOController::releaseLines(PieGPIORequestUserClient *client)
{
	gated(^IOReturn {
		client->fClosed = true;
		if (!client->fRequested) {
			return kIOReturnSuccess;
		}
		for (uint32_t i = 0; i < client->fNumLines; i++) {
			unsigned line = client->fOffsets[i];
			Line &l = fLines[line];
			hwSetEdges(line, false, false);
			l.owner = NULL;
			l.debounceDeadline = 0;
			bzero(&l.settings, sizeof(l.settings));
			postInfo(line, kPieGPIOLineReleased);
		}
		client->fRequested = false;
		client->release();
		return kIOReturnSuccess;
	});
}

#pragma mark the kernel's drivers

/*
 * Platform functions, for drivers that drive a line themselves (RPiGEM,
 * the Pi 5's PHY reset): kPieGPIOSetOutput (line, pin level, consumer)
 * makes the line an output at that level and keeps it the kernel's (used,
 * by that consumer); kPieGPIOGetValue (line, uint32_t *) reads its pin;
 * kPieGPIOSetPinctrl (config node, consumer) gives lines a peripheral's
 * function (PIEThermal's fan PWM, piebus's I2C and SPI pins), kept the
 * same way; kPieGPIORequestEdges (PieGPIOKernelEdges *) makes the line an
 * input whose edges call the driver back (PIEBoard's power button),
 * kept the same way; kPieGPIORelease (consumer) gives back what a
 * consumer kept, edges and all.
 */
IOReturn
PieGPIOController::callPlatformFunction(const OSSymbol *functionName, bool waitForFunction, void *param1,
    void *param2, void *param3, void *param4)
{
	unsigned line = (unsigned)(uintptr_t)param1;
	if (functionName->isEqualTo(kPieGPIOSetOutput)) {
		bool high = (uintptr_t)param2 != 0;
		const char *consumer = param3 != NULL ? (const char *)param3 : "kernel";
		return gated(^IOReturn {
			if (line >= fLineCount) {
				return kIOReturnBadArgument;
			}
			Line &l = fLines[line];
			if (l.owner != NULL || (l.reserved && strncmp(l.consumer, consumer, sizeof(l.consumer)) != 0)) {
				return kIOReturnBusy;
			}
			bool first = !l.reserved;
			if (first) {
				l.reserved = true;
				strlcpy(l.consumer, consumer, sizeof(l.consumer));
			}
			/* A pre-reserved line may still have the firmware's pin mux. */
			l.peripheral = false;
			hwClaim(line);
			hwSetOpenDrain(line, false);
			hwWrite(line, high);
			hwSetOutput(line, true);
			if (first) {
				postInfo(line, kPieGPIOLineRequested);
			}
			return kIOReturnSuccess;
		});
	}
	if (functionName->isEqualTo(kPieGPIOGetValue)) {
		uint32_t *value = (uint32_t *)param2;
		return gated(^IOReturn {
			if (line >= fLineCount || value == NULL) {
				return kIOReturnBadArgument;
			}
			*value = hwRead(line);
			return kIOReturnSuccess;
		});
	}
	if (functionName->isEqualTo(kPieGPIOSetPinctrl)) {
		return setPinctrl((IORegistryEntry *)param1, param2 != NULL ? (const char *)param2 : "kernel");
	}
	if (functionName->isEqualTo(kPieGPIORelease)) {
		return param1 != NULL ? releaseConsumer((const char *)param1) : kIOReturnBadArgument;
	}
	if (functionName->isEqualTo(kPieGPIORequestEdges)) {
		return param1 != NULL ? requestKernelEdges((const PieGPIOKernelEdges *)param1) : kIOReturnBadArgument;
	}
	return super::callPlatformFunction(functionName, waitForFunction, param1, param2, param3, param4);
}

#pragma mark pin functions

namespace {
struct PinSetting {
	unsigned line;
	int fsel;               /* -1: as it is */
	uint8_t bias;           /* 0: as it is, else BIAS_* */
	uint32_t mA;            /* 0: as it is */
};

/* A u32 list property's cell i (big-endian), or its only cell; false if it has neither. */
bool
cell(OSData *d, unsigned i, uint32_t *v)
{
	unsigned n = d != NULL ? d->getLength() / 4 : 0;
	if (n == 0 || (n != 1 && i >= n)) {
		return false;
	}
	*v = OSReadBigInt32(d->getBytesNoCopy(), 4 * (n == 1 ? 0 : i));
	return true;
}
}

/*
 * One device tree pin configuration node, as Linux's pinctrl reads them:
 * the generic properties (pins = "gpioN"..., function, bias-disable,
 * bias-pull-up, bias-pull-down, drive-strength) or Broadcom's (brcm,pins,
 * brcm,function, brcm,pull: one cell for every pin, or one each), on the
 * node or on its children (a group of groups). Every line is checked
 * before any is changed.
 */
IOReturn
PieGPIOController::setPinctrl(IORegistryEntry *config, const char *consumer)
{
	if (config == NULL) {
		return kIOReturnBadArgument;
	}
	PinSetting pins[kPieGPIOMaxChipLines];
	unsigned count = 0;
	IOReturn ret = kIOReturnSuccess;
	OSArray *groups = OSArray::withCapacity(4);
	if (groups == NULL) {
		return kIOReturnNoMemory;
	}
	if (config->getProperty("pins") != NULL || config->getProperty("brcm,pins") != NULL) {
		groups->setObject(config);
	} else {
		OSIterator *children = config->getChildIterator(gIODTPlane);
		IORegistryEntry *child;
		while (children != NULL && (child = OSDynamicCast(IORegistryEntry, children->getNextObject())) != NULL) {
			if (child->getProperty("pins") != NULL || child->getProperty("brcm,pins") != NULL) {
				groups->setObject(child);
			}
		}
		OSSafeReleaseNULL(children);
	}
	for (unsigned g = 0; g < groups->getCount() && ret == kIOReturnSuccess; g++) {
		IORegistryEntry *node = (IORegistryEntry *)groups->getObject(g);
		OSData *names = OSDynamicCast(OSData, node->getProperty("pins"));
		OSData *brcmPins = OSDynamicCast(OSData, node->getProperty("brcm,pins"));
		OSData *function = OSDynamicCast(OSData, node->getProperty("function"));
		OSData *brcmFunction = OSDynamicCast(OSData, node->getProperty("brcm,function"));
		OSData *brcmPull = OSDynamicCast(OSData, node->getProperty("brcm,pull"));
		OSData *drive = OSDynamicCast(OSData, node->getProperty("drive-strength"));
		unsigned nPins = 0;
		if (names != NULL) {
			const char *bytes = (const char *)names->getBytesNoCopy();
			for (size_t off = 0; off < names->getLength(); ) {
				size_t len = strnlen(bytes + off, names->getLength() - off);
				if (len == 0 || len == names->getLength() - off) { ret = kIOReturnBadArgument; break; }
				nPins++;
				off += len + 1;
			}
		} else if (brcmPins != NULL && brcmPins->getLength() % 4 == 0) {
			nPins = brcmPins->getLength() / 4;
		}
		if (ret != kIOReturnSuccess || nPins == 0 || nPins > kPieGPIOMaxChipLines ||
		    (names != NULL && brcmPins != NULL)) { ret = kIOReturnBadArgument; break; }
		if ((brcmFunction != NULL && brcmFunction->getLength() != 4 && brcmFunction->getLength() != 4 * nPins) ||
		    (brcmPull != NULL && brcmPull->getLength() != 4 && brcmPull->getLength() != 4 * nPins) ||
		    (drive != NULL && drive->getLength() != 4)) { ret = kIOReturnBadArgument; break; }
		if (drive != NULL && !hwSupportsDrive(OSReadBigInt32(drive->getBytesNoCopy(), 0))) {
			ret = kIOReturnUnsupported; break;
		}
		uint8_t bias = node->getProperty("bias-disable") != NULL ? BIAS_DISABLED :
		    node->getProperty("bias-pull-up") != NULL ? BIAS_PULL_UP :
		    node->getProperty("bias-pull-down") != NULL ? BIAS_PULL_DOWN : 0;
		if ((bias != 0 || brcmPull != NULL) && !hwSupportsBias()) { ret = kIOReturnUnsupported; break; }
		char fname[32] = "";
		if (function != NULL) {
			/* Bounded by the property: its last string may lack its NUL. */
			size_t len = strnlen((const char *)function->getBytesNoCopy(), function->getLength());
			if (len == 0 || len >= function->getLength() || len >= sizeof(fname)) {
				ret = kIOReturnBadArgument; break;
			}
			memcpy(fname, function->getBytesNoCopy(), len);
			fname[len] = '\0';
		}
		const char *p = names != NULL ? (const char *)names->getBytesNoCopy() : NULL;
		const char *end = p != NULL ? p + names->getLength() : NULL;
		unsigned nBrcm = brcmPins != NULL ? brcmPins->getLength() / 4 : 0;
		for (unsigned i = 0; ; i++) {
			unsigned long line;
			if (p != NULL) {
				if (p >= end) {
					break;
				}
				size_t len = strnlen(p, (size_t)(end - p));
				char pin[16], *rest = NULL;
				memcpy(pin, p, len < sizeof(pin) - 1 ? len : sizeof(pin) - 1);
				pin[len < sizeof(pin) - 1 ? len : sizeof(pin) - 1] = '\0';
				line = strncmp(pin, "gpio", 4) == 0 ? strtoul(pin + 4, &rest, 10) : ~0UL;
				if (len >= sizeof(pin) || rest == NULL || rest == pin + 4 || *rest != '\0') {
					IOLog("%s: pin \"%.*s\" of %s: not a line\n", getName(), (int)len, p, node->getName());
					ret = kIOReturnUnsupported;
					break;
				}
				p += len + 1;
			} else {
				if (i >= nBrcm) {
					break;
				}
				line = OSReadBigInt32(brcmPins->getBytesNoCopy(), 4 * i);
			}
			if (line >= fLineCount || count == kPieGPIOMaxChipLines) {
				ret = kIOReturnBadArgument;
				break;
			}
			PinSetting &s = pins[count++];
			s.line = (unsigned)line;
			s.fsel = -1;
			s.bias = bias;
			s.mA = 0;
			uint32_t v;
			unsigned fsel;
			if (fname[0] != '\0') {
				if (!hwFunctionNumber(s.line, fname, &fsel)) {
					IOLog("%s: line %u has no function %s\n", getName(), s.line, fname);
					ret = kIOReturnUnsupported;
					break;
				}
				s.fsel = (int)fsel;
			} else if (cell(brcmFunction, i, &v)) {
				if (!hwBrcmFunction(s.line, v, &fsel)) {
					IOLog("%s: line %u has no brcm,function %u\n", getName(), s.line, v);
					ret = kIOReturnUnsupported;
					break;
				}
				s.fsel = (int)fsel;
			}
			if (cell(brcmPull, i, &v)) {
				if (v > 2) { ret = kIOReturnBadArgument; break; }
				s.bias = v == 1 ? BIAS_PULL_DOWN : v == 2 ? BIAS_PULL_UP : BIAS_DISABLED;
			}
			if (cell(drive, 0, &v)) {
				s.mA = v;
			}
		}
	}
	groups->release();
	if (ret == kIOReturnSuccess && count == 0) {
		ret = kIOReturnNotFound;
	}
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	PinSetting *list = pins;
	return gated(^IOReturn {
		for (unsigned i = 0; i < count; i++) {
			const Line &l = fLines[list[i].line];
			if (l.owner != NULL || (l.reserved && strncmp(l.consumer, consumer, sizeof(l.consumer)) != 0)) {
				return kIOReturnBusy;
			}
		}
		for (unsigned i = 0; i < count; i++) {
			const PinSetting &s = list[i];
			Line &l = fLines[s.line];
			bool first = !l.reserved;
			if (first) {
				l.reserved = true;
				strlcpy(l.consumer, consumer, sizeof(l.consumer));
			}
			if (s.fsel >= 0) {
				hwSetFunction(s.line, (unsigned)s.fsel);
				l.peripheral = true;
			}
			if (s.bias != 0) {
				hwSetBias(s.line, s.bias);
			}
			if (s.mA != 0) {
				hwSetDrive(s.line, s.mA);
			}
			if (first) {
				postInfo(s.line, kPieGPIOLineRequested);
			}
		}
		return kIOReturnSuccess;
	});
}

IOReturn
PieGPIOController::releaseConsumer(const char *consumer)
{
	return gated(^IOReturn {
		for (unsigned line = 0; line < fLineCount; line++) {
			Line &l = fLines[line];
			if (l.reserved && l.owner == NULL && strncmp(l.consumer, consumer, sizeof(l.consumer)) == 0) {
				if (l.kernelFn != NULL) {
					hwSetEdges(line, false, false);
					l.kernelFn = NULL;
					l.kernelRefcon = NULL;
				}
				l.reserved = false;
				l.peripheral = false;
				l.consumer[0] = '\0';
				postInfo(line, kPieGPIOLineReleased);
			}
		}
		return kIOReturnSuccess;
	});
}

/* The line an input of the kernel's, whose edges (logical) call e->fn back. */
IOReturn
PieGPIOController::requestKernelEdges(const PieGPIOKernelEdges *e)
{
	const char *consumer = e->consumer != NULL ? e->consumer : "kernel";
	return gated(^IOReturn {
		if (e->line >= fLineCount || e->fn == NULL || !(e->rising || e->falling)) {
			return kIOReturnBadArgument;
		}
		if (!hwCanDetectEdges()) {
			return kIOReturnUnsupported;
		}
		Line &l = fLines[e->line];
		if (l.owner != NULL || (l.reserved && strncmp(l.consumer, consumer, sizeof(l.consumer)) != 0)) {
			return kIOReturnBusy;
		}
		bool first = !l.reserved;
		if (first) {
			l.reserved = true;
			strlcpy(l.consumer, consumer, sizeof(l.consumer));
		}
		if (first || l.peripheral) {
			l.peripheral = false;
			hwClaim(e->line);
		}
		hwSetOutput(e->line, false);
		l.kernelFn = e->fn;
		l.kernelRefcon = e->refcon;
		l.kernelActiveLow = e->activeLow;
		l.kernelRising = e->rising;
		l.kernelFalling = e->falling;
		/* The pin's edges: an active-low line's logical rising edge is its pin's falling one. */
		hwSetEdges(e->line, e->activeLow ? e->falling : e->rising, e->activeLow ? e->rising : e->falling);
		if (first) {
			postInfo(e->line, kPieGPIOLineRequested);
		}
		return kIOReturnSuccess;
	});
}

/* On the work loop: a kernel driver's line's edge, which way it went (logical). */
void
PieGPIOController::kernelEdge(unsigned line)
{
	Line &l = fLines[line];
	bool rising;
	if (l.kernelRising != l.kernelFalling) {
		rising = l.kernelRising;
	} else if (hwEdgeType(line, &rising)) {
		rising = rising != l.kernelActiveLow;
	} else {
		rising = hwRead(line) != l.kernelActiveLow;
	}
	l.kernelFn(l.kernelRefcon, line, rising);
}

#pragma mark edges

/* In the primary interrupt handler: only when the edges came (the action reads them). */
bool
PieGPIOController::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *)
{
	PieGPIOController *self = (PieGPIOController *)owner;
	uint64_t none = 0;
	__c11_atomic_compare_exchange_strong(&self->fStampMono, &none, mach_continuous_time(),
	    __ATOMIC_RELAXED, __ATOMIC_RELAXED);
	return true;
}

void
PieGPIOController::interruptAction(OSObject *owner, IOInterruptEventSource *, int)
{
	PieGPIOController *self = (PieGPIOController *)owner;
	uint64_t at = __c11_atomic_exchange(&self->fStampMono, 0, __ATOMIC_RELAXED);
	uint64_t now = mach_continuous_time();
	if (at == 0 || at > now) {
		at = now;
	}
	uint64_t atNs, nowNs;
	absolutetime_to_nanoseconds(at, &atNs);
	absolutetime_to_nanoseconds(now, &nowNs);
	/* The calendar time then: now's, less what has passed since. */
	self->handleInterrupt(atNs, realtime_ns() - (nowNs - atNs));
}

void
PieGPIOController::handleInterrupt(uint64_t monotonicNs, uint64_t realtimeNs)
{
	uint64_t pending = hwTakeEdges();
	uint64_t now = mach_absolute_time();
	bool debounce = false;
	for (unsigned line = 0; pending != 0 && line < fLineCount; line++) {
		if (!(pending & (1ULL << line))) {
			continue;
		}
		pending &= ~(1ULL << line);
		Line &l = fLines[line];
		if (l.owner == NULL && l.kernelFn != NULL) {
			kernelEdge(line);
			continue;
		}
		if (l.owner == NULL || l.settings.edge == EDGE_NONE) {
			continue;
		}
		if (l.settings.debounceUs != 0) {
			uint64_t period;
			nanoseconds_to_absolutetime((uint64_t)l.settings.debounceUs * 1000, &period);
			l.debounceDeadline = now + period;
			debounce = true;
			continue;
		}
		bool rising;
		if (l.settings.edge == EDGE_RISING) {
			rising = true;
		} else if (l.settings.edge == EDGE_FALLING) {
			rising = false;
		} else if (hwEdgeType(line, &rising)) {
			rising = rising != (l.settings.activeLow != 0);
		} else {
			rising = logicalValue(line);
		}
		edge(line, rising, monotonicNs, realtimeNs);
	}
	if (debounce) {
		armDebounce();
	}
}

void
PieGPIOController::edge(unsigned line, bool rising, uint64_t monotonicNs, uint64_t realtimeNs)
{
	Line &l = fLines[line];
	PieGPIOEdgeEvent ev;
	bzero(&ev, sizeof(ev));
	ev.timestampNs = l.settings.clock == CLOCK_REAL ? realtimeNs : monotonicNs;
	ev.offset = line;
	ev.type = rising ? kPieGPIORisingEdge : kPieGPIOFallingEdge;
	ev.lineSeqno = ++l.lineSeqno;
	l.owner->post(&ev);
}

/* The debounce timer, at the first line's deadline. */
void
PieGPIOController::armDebounce(void)
{
	uint64_t first = 0;
	for (unsigned line = 0; line < fLineCount; line++) {
		uint64_t d = fLines[line].debounceDeadline;
		if (d != 0 && (first == 0 || d < first)) {
			first = d;
		}
	}
	if (first == 0) {
		fDebounceTimer->cancelTimeout();
		return;
	}
	uint64_t now = mach_absolute_time();
	fDebounceTimer->setTimeout(first > now ? first - now : 1);
}

/* A debounced line's level has held for its period: an edge if it is not the last steady one. */
void
PieGPIOController::debounceAction(OSObject *owner, IOTimerEventSource *)
{
	PieGPIOController *self = (PieGPIOController *)owner;
	uint64_t now = mach_absolute_time();
	uint64_t mono = monotonic_ns(), real = realtime_ns();
	for (unsigned line = 0; line < self->fLineCount; line++) {
		Line &l = self->fLines[line];
		if (l.debounceDeadline == 0 || l.debounceDeadline > now) {
			continue;
		}
		l.debounceDeadline = 0;
		if (l.owner == NULL) {
			continue;
		}
		bool v = self->logicalValue(line);
		if (v == l.stable) {
			continue;
		}
		l.stable = v;
		uint8_t e = l.settings.edge;
		if ((v && (e == EDGE_RISING || e == EDGE_BOTH)) || (!v && (e == EDGE_FALLING || e == EDGE_BOTH))) {
			self->edge(line, v, mono, real);
		}
	}
	self->armDebounce();
}
