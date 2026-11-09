/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <PieGPIO.h>: piegpio's interface between piegpio.kext and its clients,
 * by IOKit's conventions:
 *
 * - Each GPIO controller is a PieGPIOController service (one per Linux
 *   gpiochip), registered with what does not change as properties:
 *   kPieGPIOChipNameKey ("gpiochipN"), kPieGPIOChipLabelKey (Linux's label
 *   for it), kPieGPIOLineCountKey, kPieGPIOLineNamesKey (the device tree's
 *   gpio-line-names) and kPieGPIOPHandleKey (its device tree node's
 *   phandle, for drivers that name a line by phandle).
 *
 * - A process opens it (IOServiceOpen) with a connection type:
 *   kPieGPIOChipClient, for the lines' state and its changes (anyone), or
 *   kPieGPIORequestClient, which holds the lines it requests until it is
 *   closed (IOServiceClose, or the process ends): root, or a member of the
 *   group the controller's personality names (GPIOGroupID: gpio).
 *
 * - Methods (IOConnectCallMethod; selectors below) take scalars and the
 *   structures here, each starting with its version and size.
 *
 * - Events go through a shared data queue (IOSharedDataQueue): the
 *   connection's memory of type kPieGPIOEventQueue (IOConnectMapMemory64),
 *   read with IODataQueueDequeue, with a Mach message on the port given
 *   for that type (IOConnectSetNotificationPort) when it goes from empty
 *   to not. A chip connection's queue has PieGPIOInfoEvent entries, a
 *   request's PieGPIOEdgeEvent ones.
 *
 * - Kernel drivers drive lines through the controller's platform
 *   functions (IOService::callPlatformFunction): kPieGPIOSetOutput (line,
 *   level 0 or 1, consumer name), kPieGPIOGetValue (line, uint32_t *),
 *   kPieGPIOSetPinctrl (a device tree pin configuration node, as an
 *   IORegistryEntry *, and a consumer name: its lines get the function,
 *   bias and drive strength it names), kPieGPIORequestEdges (a
 *   PieGPIOKernelEdges *: the line an input whose edges call the driver
 *   back, on the controller's work loop) and kPieGPIORelease (consumer
 *   name: the lines it holds are free again, left as they are, and their
 *   edges no longer call it). A line set by any of the first, third and
 *   fourth is the consumer's until it releases it: used, with that
 *   consumer, to everyone else. <PieGPIOKernel.h> has
 *   helpers for drivers that find the controller from the device tree.
 *   Levels there are the pins'; values on a connection are logical (an
 *   active-low line reads 1 when its pin is low), as libgpiod's are.
 */

#ifndef _PIEGPIO_H_
#define _PIEGPIO_H_

#include <stdint.h>

#define kPieGPIOControllerClass         "PieGPIOController"
#define kPieGPIOChipNameKey             "IOGPIOChipName"
#define kPieGPIOChipLabelKey            "IOGPIOChipLabel"
#define kPieGPIOLineCountKey            "IOGPIOLineCount"
#define kPieGPIOLineNamesKey            "IOGPIOLineNames"
#define kPieGPIOPHandleKey              "IOGPIOPHandle"

#define kPieGPIOSetOutput               "PieGPIOSetOutput"
#define kPieGPIOGetValue                "PieGPIOGetValue"
#define kPieGPIOSetPinctrl              "PieGPIOSetPinctrl"
#define kPieGPIORelease                 "PieGPIORelease"
#define kPieGPIORequestEdges            "PieGPIORequestEdges"

#ifdef KERNEL
/*
 * kPieGPIORequestEdges's callback: an edge of the line, rising or falling
 * as the line's logical value went (active low applied). On the
 * controller's work loop, inside its gate: it must not wait for anything
 * that waits for the controller.
 */
typedef void (*PieGPIOEdgeFn)(void *refcon, uint32_t line, bool rising);

typedef struct PieGPIOKernelEdges {
	uint32_t line;
	bool activeLow;
	bool rising, falling;           /* the logical edges that call back */
	const char *consumer;
	PieGPIOEdgeFn fn;
	void *refcon;
} PieGPIOKernelEdges;
#endif

#define kPieGPIOABIVersion              1
#define kPieGPIOMaxLines                64      /* in one request, as Linux's GPIO_V2_LINES_MAX */
#define kPieGPIONameSize                32      /* names and consumers, NUL included */
#define kPieGPIOMaxEventBuffer          4096    /* edge events a request's queue holds at most */

enum {
	kPieGPIOChipClient              = 0,
	kPieGPIORequestClient           = 1,
};

enum {
	kPieGPIOEventQueue              = 0,    /* memory and notification type */
};

/* kPieGPIOChipClient's methods. */
enum {
	kPieGPIOGetLineInfo             = 0,    /* scalar offset -> PieGPIOLineInfo */
	kPieGPIOWatchLineInfo           = 1,    /* scalar offset -> PieGPIOLineInfo; info events for it */
	kPieGPIOUnwatchLineInfo         = 2,    /* scalar offset */
	kPieGPIOChipMethodCount
};

/* kPieGPIORequestClient's methods. */
enum {
	kPieGPIORequestLines            = 0,    /* PieGPIORequest; once */
	kPieGPIOGetValues               = 1,    /* scalar mask -> scalar values */
	kPieGPIOSetValues               = 2,    /* scalar mask, scalar values */
	kPieGPIOReconfigureLines        = 3,    /* PieGPIORequest (its consumer and buffer size unused) */
	kPieGPIORequestMethodCount
};

/*
 * A line's settings. The values are libgpiod's (enum gpiod_line_direction,
 * _edge, _bias, _drive and _clock), so that nothing translates them.
 */
typedef struct PieGPIOLineSettings {
	uint8_t direction;              /* 1 as is, 2 input, 3 output */
	uint8_t edge;                   /* 1 none, 2 rising, 3 falling, 4 both */
	uint8_t bias;                   /* 1 as is, 2 unknown, 3 disabled, 4 pull-up, 5 pull-down */
	uint8_t drive;                  /* 1 push-pull, 2 open-drain, 3 open-source */
	uint8_t clock;                  /* 1 monotonic, 2 realtime, 3 HTE (not supported) */
	uint8_t activeLow;
	uint8_t output;                 /* an output's first value, logical */
	uint8_t reserved;
	uint32_t debounceUs;
} PieGPIOLineSettings;

enum {
	kPieGPIOLineUsed                = 1U << 0,
};

typedef struct PieGPIOLineInfo {
	uint32_t version;
	uint32_t size;
	uint32_t offset;
	uint32_t flags;                 /* kPieGPIOLineUsed */
	char name[kPieGPIONameSize];
	char consumer[kPieGPIONameSize];
	PieGPIOLineSettings settings;   /* output: unused */
} PieGPIOLineInfo;

/*
 * A request: its lines (offsets on the chip) with their settings. Values
 * on the connection are bit masks in this order (bit i, line offsets[i]).
 */
typedef struct PieGPIORequest {
	uint32_t version;
	uint32_t size;
	char consumer[kPieGPIONameSize];
	uint32_t eventBufferSize;       /* 0: 16 for each line, as Linux */
	uint32_t numLines;
	uint32_t offsets[kPieGPIOMaxLines];
	PieGPIOLineSettings settings[kPieGPIOMaxLines];
} PieGPIORequest;

/* libgpiod's enum gpiod_info_event_type. */
enum {
	kPieGPIOLineRequested           = 1,
	kPieGPIOLineReleased            = 2,
	kPieGPIOLineConfigChanged       = 3,
};

typedef struct PieGPIOInfoEvent {
	uint64_t timestampNs;           /* monotonic */
	uint32_t type;
	uint32_t reserved;
	PieGPIOLineInfo info;
} PieGPIOInfoEvent;

/* libgpiod's enum gpiod_edge_event_type. */
enum {
	kPieGPIORisingEdge              = 1,
	kPieGPIOFallingEdge             = 2,
};

typedef struct PieGPIOEdgeEvent {
	uint64_t timestampNs;           /* the line's clock */
	uint32_t offset;
	uint32_t type;
	uint64_t seqno;                 /* in the request, from 1 */
	uint64_t lineSeqno;             /* on the line, from 1 */
} PieGPIOEdgeEvent;

#endif /* _PIEGPIO_H_ */
