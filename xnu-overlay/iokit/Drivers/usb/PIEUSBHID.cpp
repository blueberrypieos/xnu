/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <libkern/OSByteOrder.h>
#include <kern/thread_call.h>

#include "PIEUSB.h"
#include "../../../bsd/dev/evdev/evdev.h"

#ifndef MIN
#define MIN(a, b)               (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)               (((a) > (b)) ? (a) : (b))
#endif

#define HID_MAX_FIELDS          96
#define HID_MAX_RANGES          16
#define HID_MAX_ARRAY           32
#define HID_MAX_APPS            6
#define HID_MAX_REPORT_IDS      16
#define HID_MAX_DESCRIPTOR      4096

enum { HID_INPUT = 0, HID_OUTPUT = 1 };

enum { HID_CONSTANT = 0x01, HID_VARIABLE = 0x02, HID_RELATIVE = 0x04 };

enum {
	PAGE_GENERIC_DESKTOP = 0x01, PAGE_KEYBOARD = 0x07, PAGE_LED = 0x08,
	PAGE_BUTTON = 0x09, PAGE_CONSUMER = 0x0c,
};
#define USAGE(page, id)         ((uint32_t)(page) << 16 | (id))
enum {
	APP_POINTER = USAGE(PAGE_GENERIC_DESKTOP, 0x01),
	APP_MOUSE = USAGE(PAGE_GENERIC_DESKTOP, 0x02),
	APP_KEYBOARD = USAGE(PAGE_GENERIC_DESKTOP, 0x06),
	APP_KEYPAD = USAGE(PAGE_GENERIC_DESKTOP, 0x07),
	APP_SYSTEM_CONTROL = USAGE(PAGE_GENERIC_DESKTOP, 0x80),
	APP_CONSUMER_CONTROL = USAGE(PAGE_CONSUMER, 0x01),
};

struct HIDRange {
	uint32_t min, max;
};

struct HIDField {
	uint8_t kind;
	uint8_t reportID;
	uint8_t flags;
	uint8_t app;
	uint32_t bitPos;
	uint8_t size;
	uint8_t nranges;
	uint16_t count;
	int32_t logMin, logMax;
	struct HIDRange ranges[HID_MAX_RANGES];
	uint32_t last[HID_MAX_ARRAY];
};

struct HIDApp {
	uint32_t usage;
	struct evdev_dev *evdev;
	bool used;
};

class PIEUSBHID : public IOService
{
	OSDeclareDefaultStructors(PIEUSBHID);

	PIEUSBInterface *intf;
	IOLock *lock;
	bool stopping;
	bool reportIDs;
	bool boot;
	uint8_t napps;
	uint16_t nfields;
	struct HIDApp apps[HID_MAX_APPS];
	struct HIDField *fields;
	thread_call_t ledCall;
	volatile uint32_t ledState;

	bool parse(const uint8_t *d, uint32_t len);
	bool bootDescriptor(void);
	uint32_t usageAt(const struct HIDField *f, uint32_t index) const;
	void describe(struct HIDApp *app, struct evdev_dev *evdev);
	bool makeDevices(void);
	void freeDevices(void);
	void name(char *buf, size_t size, const char *suffix);
	void process(const uint8_t *data, uint32_t length);
	void pushField(struct HIDField *f, const uint8_t *data, uint32_t length);
	void sendLEDs(void);

	static void report(OSObject *target, const uint8_t *data, uint32_t length);
	static void ledThread(thread_call_param_t p0, thread_call_param_t p1);

	char product[64];
	char serial[64];
	uint16_t vendorID, productID, release;

public:
	static void evEvent(struct evdev_dev *evdev, uint16_t type, uint16_t code, int32_t value);

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
};

#define super IOService
OSDefineMetaClassAndStructors(PIEUSBHID, IOService);

static const struct evdev_methods pie_hid_methods = {
	.ev_open = NULL,
	.ev_close = NULL,
	.ev_event = PIEUSBHID::evEvent,
	.ev_get_keycode = NULL,
	.ev_set_keycode = NULL,
};

#pragma mark The report descriptor

static int32_t
sign_extend(uint32_t v, unsigned bits)
{
	if (bits == 0 || bits >= 32) {
		return (int32_t)v;
	}
	uint32_t m = 1U << (bits - 1);
	return (int32_t)((v ^ m) - m);
}

bool
PIEUSBHID::parse(const uint8_t *d, uint32_t len)
{
	struct Globals {
		uint32_t page;
		int32_t logMin, logMax;
		uint32_t size, count, reportID;
		bool logMinSigned;
	} g = {}, stack[4];
	unsigned sp = 0;
	struct HIDRange ranges[HID_MAX_RANGES];
	unsigned nranges = 0;
	uint32_t usageMin = 0;
	bool haveMin = false;
	int depth = 0;
	int app = -1;
	struct { uint8_t id; uint32_t bits[2]; } pos[HID_MAX_REPORT_IDS];
	unsigned npos = 0;

	nfields = 0;
	napps = 0;
	reportIDs = false;
	for (uint32_t i = 0; i < len;) {
		uint8_t prefix = d[i++];
		if (prefix == 0xfe) {
			if (i + 1 >= len) {
				break;
			}
			i += 2 + d[i];
			continue;
		}
		unsigned bytes = (prefix & 3) == 3 ? 4 : (prefix & 3);
		if (i + bytes > len) {
			break;
		}
		uint32_t u = 0;
		for (unsigned b = 0; b < bytes; b++) {
			u |= (uint32_t)d[i + b] << (8 * b);
		}
		int32_t s = sign_extend(u, bytes * 8);
		i += bytes;
		unsigned type = (prefix >> 2) & 3, tag = prefix >> 4;

		if (type == 1) {
			switch (tag) {
			case 0: g.page = u; break;
			case 1: g.logMin = s; g.logMinSigned = s < 0; break;
			case 2: g.logMax = g.logMinSigned ? s : (int32_t)u; break;
			case 7: g.size = u; break;
			case 8: g.reportID = u; reportIDs = true; break;
			case 9: g.count = u; break;
			case 10: if (sp < 4) { stack[sp++] = g; } break;
			case 11: if (sp > 0) { g = stack[--sp]; } break;
			}
			continue;
		}
		if (type == 2) {
			uint32_t usage = bytes == 4 ? u : USAGE(g.page, u);
			if (tag == 0 && nranges < HID_MAX_RANGES) {
				ranges[nranges].min = ranges[nranges].max = usage;
				nranges++;
			} else if (tag == 1) {
				usageMin = usage;
				haveMin = true;
			} else if (tag == 2 && haveMin && nranges < HID_MAX_RANGES) {
				ranges[nranges].min = usageMin;
				ranges[nranges].max = usage;
				nranges++;
				haveMin = false;
			}
			continue;
		}
		if (type != 0) {
			continue;
		}
		if (tag == 0xa) {
			if (depth == 0 && u == 1 && napps < HID_MAX_APPS) {
				app = napps++;
				apps[app].usage = nranges > 0 ? ranges[0].min : 0;
				apps[app].evdev = NULL;
				apps[app].used = false;
			}
			depth++;
		} else if (tag == 0xc) {
			if (depth > 0 && --depth == 0) {
				app = -1;
			}
		} else if (tag == 8 || tag == 9 || tag == 0xb) {
			unsigned kind = tag == 8 ? HID_INPUT : tag == 9 ? HID_OUTPUT : 2;
			unsigned p;
			for (p = 0; p < npos && pos[p].id != g.reportID; p++) {
			}
			if (p == npos && npos < HID_MAX_REPORT_IDS) {
				pos[npos].id = (uint8_t)g.reportID;
				pos[npos].bits[0] = pos[npos].bits[1] = 0;
				npos++;
			}
			if (p < npos && kind != 2) {
				if (!(u & HID_CONSTANT) && app >= 0 && nranges > 0 &&
				    g.size > 0 && g.size <= 32 && g.count > 0 && nfields < HID_MAX_FIELDS) {
					struct HIDField *f = &fields[nfields++];
					bzero(f, sizeof(*f));
					f->kind = (uint8_t)kind;
					f->reportID = (uint8_t)g.reportID;
					f->flags = (uint8_t)u;
					f->app = (uint8_t)app;
					f->bitPos = pos[p].bits[kind];
					f->size = (uint8_t)g.size;
					f->count = (uint16_t)MIN(g.count, 0xffffU);
					f->logMin = g.logMin;
					f->logMax = g.logMax;
					f->nranges = (uint8_t)nranges;
					bcopy(ranges, f->ranges, nranges * sizeof(ranges[0]));
				}
				pos[p].bits[kind] += g.size * g.count;
			}
		}
		nranges = 0;
		haveMin = false;
	}
	return nfields > 0;
}

uint32_t
PIEUSBHID::usageAt(const struct HIDField *f, uint32_t index) const
{
	uint32_t last = 0;
	for (unsigned r = 0; r < f->nranges; r++) {
		uint32_t n = f->ranges[r].max - f->ranges[r].min + 1;
		if (index < n) {
			return f->ranges[r].min + index;
		}
		index -= n;
		last = f->ranges[r].max;
	}
	return (f->flags & HID_VARIABLE) ? last : 0;
}

static const uint8_t boot_keyboard[] = {
	0x05, 0x01, 0x09, 0x06, 0xa1, 0x01,
	0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
	0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
	0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
	0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
	0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
	0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0xff, 0x05, 0x07,
	0x19, 0x00, 0x29, 0xff, 0x81, 0x00,
	0xc0,
};

static const uint8_t boot_mouse[] = {
	0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
	0x95, 0x03, 0x75, 0x01, 0x81, 0x02,
	0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
	0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7f,
	0x75, 0x08, 0x95, 0x02, 0x81, 0x06,
	0xc0, 0xc0,
};

bool
PIEUSBHID::bootDescriptor(void)
{
	OSNumber *sub = OSDynamicCast(OSNumber, intf->getProperty("bInterfaceSubClass"));
	OSNumber *proto = OSDynamicCast(OSNumber, intf->getProperty("bInterfaceProtocol"));
	if (sub == NULL || proto == NULL || sub->unsigned32BitValue() != 1) {
		return false;
	}
	bool ok = false;
	if (proto->unsigned32BitValue() == 1) {
		ok = parse(boot_keyboard, sizeof(boot_keyboard));
	} else if (proto->unsigned32BitValue() == 2) {
		ok = parse(boot_mouse, sizeof(boot_mouse));
	}
	if (ok) {
		intf->controlRequest(0x21, 0x0b, 0, intf->number, 0, NULL);
		boot = true;
	}
	return ok;
}

#pragma mark Event devices

static const struct { uint16_t usage, key; } consumer_keys[] = {
	{ 0x030, KEY_POWER }, { 0x032, KEY_SLEEP }, { 0x040, KEY_MENU },
	{ 0x06f, KEY_BRIGHTNESSUP }, { 0x070, KEY_BRIGHTNESSDOWN },
	{ 0x0b0, KEY_PLAY }, { 0x0b1, KEY_PAUSE }, { 0x0b2, KEY_RECORD },
	{ 0x0b3, KEY_FASTFORWARD }, { 0x0b4, KEY_REWIND },
	{ 0x0b5, KEY_NEXTSONG }, { 0x0b6, KEY_PREVIOUSSONG },
	{ 0x0b7, KEY_STOPCD }, { 0x0b8, KEY_EJECTCD }, { 0x0cd, KEY_PLAYPAUSE },
	{ 0x0e2, KEY_MUTE }, { 0x0e9, KEY_VOLUMEUP }, { 0x0ea, KEY_VOLUMEDOWN },
	{ 0x183, KEY_CONFIG }, { 0x18a, KEY_MAIL }, { 0x192, KEY_CALC },
	{ 0x194, KEY_FILE }, { 0x196, KEY_WWW }, { 0x19e, KEY_COFFEE },
	{ 0x1a7, KEY_DOCUMENTS }, { 0x221, KEY_SEARCH }, { 0x223, KEY_HOMEPAGE },
	{ 0x224, KEY_BACK }, { 0x225, KEY_FORWARD }, { 0x226, KEY_STOP },
	{ 0x227, KEY_REFRESH }, { 0x22a, KEY_BOOKMARKS }, { 0x29f, KEY_SCALE },
};

static uint16_t
hid_map(uint32_t app, uint32_t usage, uint8_t flags, uint16_t *type)
{
	uint16_t page = (uint16_t)(usage >> 16), id = (uint16_t)usage;
	bool keyApp = app == APP_KEYBOARD || app == APP_KEYPAD;
	bool pointerApp = app == APP_MOUSE || app == APP_POINTER;

	*type = EV_KEY;
	if (page == PAGE_KEYBOARD && keyApp) {
		return id < 256 ? evdev_hid2key(id) : 0;
	}
	if (page == PAGE_LED && keyApp) {
		*type = EV_LED;
		switch (id) {
		case 1: return LED_NUML + 1;
		case 2: return LED_CAPSL + 1;
		case 3: return LED_SCROLLL + 1;
		case 4: return LED_COMPOSE + 1;
		case 5: return LED_KANA + 1;
		}
		return 0;
	}
	if (page == PAGE_BUTTON && pointerApp && id >= 1 && id <= 16) {
		return id <= 8 ? (uint16_t)(BTN_MOUSE + id - 1) : (uint16_t)(BTN_MISC + id - 9);
	}
	if (page == PAGE_GENERIC_DESKTOP && pointerApp) {
		bool rel = (flags & HID_RELATIVE) != 0;
		*type = rel ? EV_REL : EV_ABS;
		switch (id) {
		case 0x30: return (uint16_t)((rel ? REL_X : ABS_X) + 1);
		case 0x31: return (uint16_t)((rel ? REL_Y : ABS_Y) + 1);
		case 0x32: return (uint16_t)((rel ? REL_Z : ABS_Z) + 1);
		case 0x38: *type = EV_REL; return REL_WHEEL + 1;
		}
		return 0;
	}
	if (page == PAGE_CONSUMER && id == 0x238 && pointerApp) {
		*type = EV_REL;
		return REL_HWHEEL + 1;
	}
	if (page == PAGE_CONSUMER && (app == APP_CONSUMER_CONTROL || keyApp)) {
		for (unsigned i = 0; i < sizeof(consumer_keys) / sizeof(consumer_keys[0]); i++) {
			if (consumer_keys[i].usage == id) {
				return consumer_keys[i].key;
			}
		}
		return 0;
	}
	if (page == PAGE_GENERIC_DESKTOP && (app == APP_SYSTEM_CONTROL || keyApp)) {
		switch (id) {
		case 0x81: return KEY_POWER;
		case 0x82: return KEY_SLEEP;
		case 0x83: return KEY_WAKEUP;
		}
	}
	return 0;
}

void
PIEUSBHID::describe(struct HIDApp *app, struct evdev_dev *evdev)
{
	unsigned a = (unsigned)(app - apps);
	for (unsigned i = 0; i < nfields; i++) {
		struct HIDField *f = &fields[i];
		if (f->app != a) {
			continue;
		}
		uint32_t n = 0;
		for (unsigned r = 0; r < f->nranges; r++) {
			n += f->ranges[r].max - f->ranges[r].min + 1;
		}
		if (n > 1024) {
			n = 1024;
		}
		for (uint32_t k = 0; k < n; k++) {
			uint16_t type, code = hid_map(app->usage, usageAt(f, k), f->flags, &type);
			if (code == 0) {
				continue;
			}
			if (type == EV_LED) {
				if (f->kind == HID_OUTPUT && (f->flags & HID_VARIABLE)) {
					app->used = true;
					if (evdev != NULL) {
						evdev_support_event(evdev, EV_LED);
						evdev_support_led(evdev, code - 1);
					}
				}
				continue;
			}
			if (f->kind != HID_INPUT) {
				continue;
			}
			app->used = true;
			if (evdev == NULL) {
				continue;
			}
			if (type == EV_KEY) {
				evdev_support_event(evdev, EV_KEY);
				evdev_support_key(evdev, code);
			} else if (type == EV_REL) {
				evdev_support_event(evdev, EV_REL);
				evdev_support_rel(evdev, code - 1);
			} else if (type == EV_ABS) {
				evdev_support_event(evdev, EV_ABS);
				evdev_support_abs(evdev, code - 1, f->logMin, f->logMax, 0, 0, 0);
			}
		}
	}
}

void
PIEUSBHID::name(char *buf, size_t size, const char *suffix)
{
	if (suffix != NULL) {
		snprintf(buf, size, "%s %s", product, suffix);
	} else {
		snprintf(buf, size, "%s", product);
	}
}

bool
PIEUSBHID::makeDevices(void)
{
	unsigned made = 0, used = 0;
	OSNumber *port = OSDynamicCast(OSNumber, intf->getProperty("port"));
	char phys[32];
	snprintf(phys, sizeof(phys), "usb-%u/input%u", port != NULL ? port->unsigned32BitValue() : 0,
	    intf->number);

	for (unsigned a = 0; a < napps; a++) {
		describe(&apps[a], NULL);
		used += apps[a].used;
	}
	for (unsigned a = 0; a < napps; a++) {
		struct HIDApp *app = &apps[a];
		if (!app->used) {
			continue;
		}
		struct evdev_dev *evdev = evdev_alloc();
		const char *suffix = NULL;
		if (used > 1) {
			switch (app->usage) {
			case APP_MOUSE: suffix = "Mouse"; break;
			case APP_POINTER: suffix = "Pointer"; break;
			case APP_KEYPAD: suffix = "Keypad"; break;
			case APP_SYSTEM_CONTROL: suffix = "System Control"; break;
			case APP_CONSUMER_CONTROL: suffix = "Consumer Control"; break;
			}
		}
		char buf[96];
		name(buf, sizeof(buf), suffix);
		evdev_set_name(evdev, buf);
		evdev_set_phys(evdev, phys);
		evdev_set_id(evdev, BUS_USB, vendorID, productID, release);
		if (serial[0] != '\0') {
			evdev_set_serial(evdev, serial);
		}
		evdev_set_methods(evdev, this, &pie_hid_methods);
		describe(app, evdev);
		if (app->usage == APP_KEYBOARD || app->usage == APP_KEYPAD ||
		    app->usage == APP_CONSUMER_CONTROL || app->usage == APP_SYSTEM_CONTROL) {
			evdev_support_event(evdev, EV_REP);
			evdev_set_flag(evdev, EVDEV_FLAG_SOFTREPEAT);
			evdev_set_flag(evdev, EVDEV_FLAG_CONSOLE_LEDS);
		}
		if (evdev_register(evdev) != 0) {
			IOLog("USB HID on interface %u: cannot register %s\n", intf->number, buf);
			evdev_free(evdev);
			continue;
		}
		IOLog("USB HID on interface %u: %s, /dev/input/event%d\n", intf->number, buf,
		    evdev_get_unit(evdev));
		IOLockLock(lock);
		app->evdev = evdev;
		IOLockUnlock(lock);
		made++;
	}
	return made > 0;
}

void
PIEUSBHID::freeDevices(void)
{
	struct evdev_dev *gone[HID_MAX_APPS] = {};
	IOLockLock(lock);
	stopping = true;
	for (unsigned a = 0; a < napps; a++) {
		gone[a] = apps[a].evdev;
		apps[a].evdev = NULL;
	}
	IOLockUnlock(lock);
	for (unsigned a = 0; a < HID_MAX_APPS; a++) {
		if (gone[a] != NULL) {
			evdev_free(gone[a]);
		}
	}
}

#pragma mark Reports

static uint32_t
hid_bits(const uint8_t *data, uint32_t length, uint32_t pos, unsigned size)
{
	uint32_t v = 0;
	for (unsigned b = 0; b < size; b++) {
		uint32_t bit = pos + b;
		if (bit / 8 < length && (data[bit / 8] >> (bit % 8)) & 1) {
			v |= 1U << b;
		}
	}
	return v;
}

void
PIEUSBHID::pushField(struct HIDField *f, const uint8_t *data, uint32_t length)
{
	struct HIDApp *app = &apps[f->app];
	struct evdev_dev *evdev = app->evdev;
	if (evdev == NULL) {
		return;
	}
	bool isSigned = f->logMin < 0;

	if (f->flags & HID_VARIABLE) {
		for (unsigned k = 0; k < f->count; k++) {
			uint32_t raw = hid_bits(data, length, f->bitPos + k * f->size, f->size);
			int32_t v = isSigned ? sign_extend(raw, f->size) : (int32_t)raw;
			uint16_t type, code = hid_map(app->usage, usageAt(f, k), f->flags, &type);
			if (code == 0) {
				continue;
			}
			switch (type) {
			case EV_KEY:
				evdev_push_key(evdev, code, v != 0 && v >= f->logMin && v <= f->logMax);
				break;
			case EV_REL:
				evdev_push_rel(evdev, code - 1, v);
				break;
			case EV_ABS:
				evdev_push_abs(evdev, code - 1, v);
				break;
			}
		}
		return;
	}

	uint32_t now[HID_MAX_ARRAY];
	unsigned n = MIN(f->count, (uint16_t)HID_MAX_ARRAY);
	for (unsigned k = 0; k < n; k++) {
		uint32_t raw = hid_bits(data, length, f->bitPos + k * f->size, f->size);
		int32_t v = isSigned ? sign_extend(raw, f->size) : (int32_t)raw;
		now[k] = 0;
		if (v >= f->logMin && v <= f->logMax) {
			uint32_t usage = usageAt(f, (uint32_t)(v - f->logMin));
			if (usage == USAGE(PAGE_KEYBOARD, 1)) {
				return;
			}
			if ((uint16_t)usage != 0) {
				now[k] = usage;
			}
		}
	}
	for (unsigned k = 0; k < n; k++) {
		uint32_t u = f->last[k];
		if (u == 0) {
			continue;
		}
		bool still = false;
		for (unsigned j = 0; j < n && !still; j++) {
			still = now[j] == u;
		}
		uint16_t type, code = hid_map(app->usage, u, f->flags, &type);
		if (!still && code != 0 && type == EV_KEY) {
			evdev_push_key(evdev, code, 0);
		}
	}
	for (unsigned k = 0; k < n; k++) {
		uint16_t type, code = hid_map(app->usage, now[k], f->flags, &type);
		if (now[k] != 0 && code != 0 && type == EV_KEY) {
			evdev_push_key(evdev, code, 1);
		}
		f->last[k] = now[k];
	}
}

void
PIEUSBHID::process(const uint8_t *data, uint32_t length)
{
	uint8_t id = 0;
	if (reportIDs) {
		if (length < 1) {
			return;
		}
		id = data[0];
		data++;
		length--;
	}
	bool touched[HID_MAX_APPS] = {};
	for (unsigned i = 0; i < nfields; i++) {
		struct HIDField *f = &fields[i];
		if (f->kind == HID_INPUT && f->reportID == id) {
			pushField(f, data, length);
			touched[f->app] = true;
		}
	}
	for (unsigned a = 0; a < napps; a++) {
		if (touched[a] && apps[a].evdev != NULL) {
			evdev_sync(apps[a].evdev);
		}
	}
}

void
PIEUSBHID::report(OSObject *target, const uint8_t *data, uint32_t length)
{
	PIEUSBHID *self = OSDynamicCast(PIEUSBHID, target);
	if (self == NULL) {
		return;
	}
	IOLockLock(self->lock);
	if (!self->stopping) {
		self->process(data, length);
	}
	IOLockUnlock(self->lock);
}

#pragma mark LEDs

void
PIEUSBHID::evEvent(struct evdev_dev *evdev, uint16_t type, uint16_t code, int32_t value)
{
	PIEUSBHID *self = (PIEUSBHID *)evdev_get_softc(evdev);
	if (type != EV_LED || code >= 32 || self == NULL) {
		return;
	}
	evdev_push_event(evdev, EV_LED, code, value != 0);
	if (value) {
		OSBitOrAtomic(1U << code, &self->ledState);
	} else {
		OSBitAndAtomic(~(1U << code), &self->ledState);
	}
	if (self->ledCall != NULL) {
		thread_call_enter(self->ledCall);
	}
}

void
PIEUSBHID::ledThread(thread_call_param_t p0, thread_call_param_t p1)
{
	(void)p1;
	((PIEUSBHID *)p0)->sendLEDs();
}

void
PIEUSBHID::sendLEDs(void)
{
	uint32_t leds = ledState;
	uint8_t ids[HID_MAX_REPORT_IDS];
	unsigned nids = 0;

	for (unsigned i = 0; i < nfields; i++) {
		struct HIDField *f = &fields[i];
		unsigned j;
		if (f->kind != HID_OUTPUT) {
			continue;
		}
		for (j = 0; j < nids && ids[j] != f->reportID; j++) {
		}
		if (j == nids && nids < HID_MAX_REPORT_IDS) {
			ids[nids++] = f->reportID;
		}
	}
	for (unsigned r = 0; r < nids; r++) {
		uint8_t buf[64];
		unsigned off = reportIDs ? 1 : 0;
		uint32_t bits = 0;
		bool any = false;
		bzero(buf, sizeof(buf));
		buf[0] = ids[r];
		for (unsigned i = 0; i < nfields; i++) {
			struct HIDField *f = &fields[i];
			if (f->kind != HID_OUTPUT || f->reportID != ids[r] || !(f->flags & HID_VARIABLE)) {
				continue;
			}
			bits = MAX(bits, f->bitPos + (uint32_t)f->size * f->count);
			for (unsigned k = 0; k < f->count; k++) {
				uint16_t type, code = hid_map(apps[f->app].usage, usageAt(f, k), f->flags, &type);
				if (type != EV_LED || code == 0) {
					continue;
				}
				any = true;
				uint32_t bit = f->bitPos + k * f->size;
				if ((leds & (1U << (code - 1))) && (off + bit / 8) < sizeof(buf)) {
					buf[off + bit / 8] |= (uint8_t)(1U << (bit % 8));
				}
			}
		}
		if (!any) {
			continue;
		}
		uint16_t len = (uint16_t)MIN(off + (bits + 7) / 8, sizeof(buf));
		intf->controlRequest(0x21, 0x09, (uint16_t)(2 << 8 | ids[r]), intf->number, len, buf);
	}
}

#pragma mark Start and stop

static void
usb_string(PIEUSBInterface *intf, uint8_t index, uint16_t lang, char *out, size_t size)
{
	uint8_t buf[256];
	out[0] = '\0';
	if (index == 0 || intf->controlRequest(0x80, 6, (uint16_t)(3 << 8 | index), lang,
	    sizeof(buf), buf) != kIOReturnSuccess) {
		return;
	}
	unsigned len = MIN((unsigned)buf[0], (unsigned)sizeof(buf)), o = 0;
	for (unsigned i = 2; i + 1 < len && o + 1 < size; i += 2) {
		uint16_t c = (uint16_t)(buf[i] | buf[i + 1] << 8);
		out[o++] = (c >= 0x20 && c < 0x7f) ? (char)c : '?';
	}
	out[o] = '\0';
	while (o > 0 && out[o - 1] == ' ') {
		out[--o] = '\0';
	}
}

bool
PIEUSBHID::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	intf = OSDynamicCast(PIEUSBInterface, provider);
	if (intf == NULL) {
		return false;
	}
	lock = IOLockAlloc();
	fields = IONewZero(struct HIDField, HID_MAX_FIELDS);
	ledCall = thread_call_allocate_with_options(&PIEUSBHID::ledThread, this,
	    THREAD_CALL_PRIORITY_KERNEL, 0);
	if (lock == NULL || fields == NULL || ledCall == NULL) {
		return false;
	}

	uint8_t dev[18];
	OSNumber *vid = OSDynamicCast(OSNumber, intf->getProperty("idVendor"));
	OSNumber *pid = OSDynamicCast(OSNumber, intf->getProperty("idProduct"));
	vendorID = vid != NULL ? (uint16_t)vid->unsigned32BitValue() : 0;
	productID = pid != NULL ? (uint16_t)pid->unsigned32BitValue() : 0;
	product[0] = serial[0] = '\0';
	if (intf->controlRequest(0x80, 6, 1 << 8, 0, sizeof(dev), dev) == kIOReturnSuccess && dev[1] == 1) {
		release = OSReadLittleInt16(dev, 12);
		uint8_t langs[4] = {};
		uint16_t lang = 0x0409;
		if (intf->controlRequest(0x80, 6, 3 << 8, 0, sizeof(langs), langs) == kIOReturnSuccess &&
		    langs[0] >= 4) {
			lang = (uint16_t)(langs[2] | langs[3] << 8);
		}
		char maker[64];
		usb_string(intf, dev[14], lang, maker, sizeof(maker));
		usb_string(intf, dev[15], lang, product, sizeof(product));
		usb_string(intf, dev[16], lang, serial, sizeof(serial));
		if (product[0] != '\0' && maker[0] != '\0' && strncmp(product, maker, strlen(maker)) != 0) {
			char both[64];
			snprintf(both, sizeof(both), "%s %s", maker, product);
			strlcpy(product, both, sizeof(product));
		}
	}
	if (product[0] == '\0') {
		snprintf(product, sizeof(product), "USB HID %04x:%04x", vendorID, productID);
	}

	bool ok = false;
	OSNumber *rlen = OSDynamicCast(OSNumber, intf->getProperty("HIDReportDescriptorLength"));
	uint32_t len = rlen != NULL ? rlen->unsigned32BitValue() : 0;
	if (len > 0 && len <= HID_MAX_DESCRIPTOR) {
		uint8_t *d = IONew(uint8_t, len);
		if (d != NULL && intf->controlRequest(0x81, 6, 0x22 << 8, intf->number, (uint16_t)len, d) ==
		    kIOReturnSuccess) {
			ok = parse(d, len);
			OSNumber *sub = OSDynamicCast(OSNumber, intf->getProperty("bInterfaceSubClass"));
			if (ok && sub != NULL && sub->unsigned32BitValue() == 1) {
				intf->controlRequest(0x21, 0x0b, 1, intf->number, 0, NULL);
			}
		}
		if (d != NULL) {
			IODelete(d, uint8_t, len);
		}
	}
	if (!ok && !bootDescriptor()) {
		IOLog("USB HID on interface %u: no report descriptor it can use\n", intf->number);
		return false;
	}
	intf->controlRequest(0x21, 0x0a, 0, intf->number, 0, NULL);

	if (!makeDevices()) {
		IOLog("USB HID on interface %u: nothing it knows (%u application(s))\n", intf->number, napps);
		return false;
	}

	for (unsigned i = 0; i < intf->numEndpoints; i++) {
		const struct PIEUSBEndpoint *ep = &intf->endpoints[i];
		if ((ep->address & 0x80) && (ep->attributes & 3) == 3) {
			IOReturn ret = intf->startInterruptIn(ep->address, this, &PIEUSBHID::report);
			if (ret != kIOReturnSuccess) {
				IOLog("USB HID on interface %u: cannot start its endpoint (0x%x)\n", intf->number, ret);
				freeDevices();
				return false;
			}
			IOLog("USB HID on interface %u: %s%s, %u field(s)\n", intf->number, product,
			    boot ? " (boot protocol)" : "", nfields);
			registerService();
			return true;
		}
	}
	IOLog("USB HID on interface %u: no interrupt endpoint\n", intf->number);
	freeDevices();
	return false;
}

void
PIEUSBHID::stop(IOService *provider)
{
	freeDevices();
	if (ledCall != NULL) {
		thread_call_cancel_wait(ledCall);
	}
	IOLog("USB HID on interface %u: gone\n", intf != NULL ? intf->number : 0);
	super::stop(provider);
}

void
PIEUSBHID::free(void)
{
	if (ledCall != NULL) {
		thread_call_cancel_wait(ledCall);
		thread_call_free(ledCall);
		ledCall = NULL;
	}
	for (unsigned a = 0; a < HID_MAX_APPS; a++) {
		if (apps[a].evdev != NULL) {
			evdev_free(apps[a].evdev);
			apps[a].evdev = NULL;
		}
	}
	if (fields != NULL) {
		IODelete(fields, struct HIDField, HID_MAX_FIELDS);
	}
	if (lock != NULL) {
		IOLockFree(lock);
	}
	super::free();
}
