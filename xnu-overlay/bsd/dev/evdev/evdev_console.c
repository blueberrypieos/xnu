/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <sys/param.h>

#include "evdev_compat.h"
#include "evdev.h"
#include "evdev_private.h"
#include "input.h"

#include "../../../iokit/pie/console.h"

extern void cons_cinput(char ch);

#define CONSOLE_MAX_KEYBOARDS   16

static LCK_MTX_DECLARE(evdev_console_mtx, &evdev_lck_grp);
static struct evdev_dev *evdev_console_leds[CONSOLE_MAX_KEYBOARDS];
static bool evdev_console_caps;
static thread_call_t evdev_console_led_call;

static const char console_plain[KEY_KPDOT + 1] = {
	[KEY_ESC] = 0x1b, [KEY_1] = '1', [KEY_2] = '2', [KEY_3] = '3',
	[KEY_4] = '4', [KEY_5] = '5', [KEY_6] = '6', [KEY_7] = '7',
	[KEY_8] = '8', [KEY_9] = '9', [KEY_0] = '0', [KEY_MINUS] = '-',
	[KEY_EQUAL] = '=', [KEY_BACKSPACE] = 0x7f, [KEY_TAB] = '\t',
	[KEY_Q] = 'q', [KEY_W] = 'w', [KEY_E] = 'e', [KEY_R] = 'r',
	[KEY_T] = 't', [KEY_Y] = 'y', [KEY_U] = 'u', [KEY_I] = 'i',
	[KEY_O] = 'o', [KEY_P] = 'p', [KEY_LEFTBRACE] = '[',
	[KEY_RIGHTBRACE] = ']', [KEY_ENTER] = '\r', [KEY_A] = 'a',
	[KEY_S] = 's', [KEY_D] = 'd', [KEY_F] = 'f', [KEY_G] = 'g',
	[KEY_H] = 'h', [KEY_J] = 'j', [KEY_K] = 'k', [KEY_L] = 'l',
	[KEY_SEMICOLON] = ';', [KEY_APOSTROPHE] = '\'', [KEY_GRAVE] = '`',
	[KEY_BACKSLASH] = '\\', [KEY_Z] = 'z', [KEY_X] = 'x', [KEY_C] = 'c',
	[KEY_V] = 'v', [KEY_B] = 'b', [KEY_N] = 'n', [KEY_M] = 'm',
	[KEY_COMMA] = ',', [KEY_DOT] = '.', [KEY_SLASH] = '/',
	[KEY_KPASTERISK] = '*', [KEY_SPACE] = ' ',
	[KEY_KP7] = '7', [KEY_KP8] = '8', [KEY_KP9] = '9', [KEY_KPMINUS] = '-',
	[KEY_KP4] = '4', [KEY_KP5] = '5', [KEY_KP6] = '6', [KEY_KPPLUS] = '+',
	[KEY_KP1] = '1', [KEY_KP2] = '2', [KEY_KP3] = '3', [KEY_KP0] = '0',
	[KEY_KPDOT] = '.',
};

static const char console_shifted[KEY_KPDOT + 1] = {
	[KEY_ESC] = 0x1b, [KEY_1] = '!', [KEY_2] = '@', [KEY_3] = '#',
	[KEY_4] = '$', [KEY_5] = '%', [KEY_6] = '^', [KEY_7] = '&',
	[KEY_8] = '*', [KEY_9] = '(', [KEY_0] = ')', [KEY_MINUS] = '_',
	[KEY_EQUAL] = '+', [KEY_BACKSPACE] = 0x7f, [KEY_TAB] = '\t',
	[KEY_Q] = 'Q', [KEY_W] = 'W', [KEY_E] = 'E', [KEY_R] = 'R',
	[KEY_T] = 'T', [KEY_Y] = 'Y', [KEY_U] = 'U', [KEY_I] = 'I',
	[KEY_O] = 'O', [KEY_P] = 'P', [KEY_LEFTBRACE] = '{',
	[KEY_RIGHTBRACE] = '}', [KEY_ENTER] = '\r', [KEY_A] = 'A',
	[KEY_S] = 'S', [KEY_D] = 'D', [KEY_F] = 'F', [KEY_G] = 'G',
	[KEY_H] = 'H', [KEY_J] = 'J', [KEY_K] = 'K', [KEY_L] = 'L',
	[KEY_SEMICOLON] = ':', [KEY_APOSTROPHE] = '"', [KEY_GRAVE] = '~',
	[KEY_BACKSLASH] = '|', [KEY_Z] = 'Z', [KEY_X] = 'X', [KEY_C] = 'C',
	[KEY_V] = 'V', [KEY_B] = 'B', [KEY_N] = 'N', [KEY_M] = 'M',
	[KEY_COMMA] = '<', [KEY_DOT] = '>', [KEY_SLASH] = '?',
	[KEY_KPASTERISK] = '*', [KEY_SPACE] = ' ',
	[KEY_KP7] = '7', [KEY_KP8] = '8', [KEY_KP9] = '9', [KEY_KPMINUS] = '-',
	[KEY_KP4] = '4', [KEY_KP5] = '5', [KEY_KP6] = '6', [KEY_KPPLUS] = '+',
	[KEY_KP1] = '1', [KEY_KP2] = '2', [KEY_KP3] = '3', [KEY_KP0] = '0',
	[KEY_KPDOT] = '.',
};

static const char *
console_sequence(uint16_t code)
{
	switch (code) {
	case KEY_UP: return "\x1b[A";
	case KEY_DOWN: return "\x1b[B";
	case KEY_RIGHT: return "\x1b[C";
	case KEY_LEFT: return "\x1b[D";
	case KEY_HOME: return "\x1b[H";
	case KEY_END: return "\x1b[F";
	case KEY_INSERT: return "\x1b[2~";
	case KEY_DELETE: return "\x1b[3~";
	case KEY_PAGEUP: return "\x1b[5~";
	case KEY_PAGEDOWN: return "\x1b[6~";
	case KEY_F1: return "\x1bOP";
	case KEY_F2: return "\x1bOQ";
	case KEY_F3: return "\x1bOR";
	case KEY_F4: return "\x1bOS";
	case KEY_F5: return "\x1b[15~";
	case KEY_F6: return "\x1b[17~";
	case KEY_F7: return "\x1b[18~";
	case KEY_F8: return "\x1b[19~";
	case KEY_F9: return "\x1b[20~";
	case KEY_F10: return "\x1b[21~";
	case KEY_F11: return "\x1b[23~";
	case KEY_F12: return "\x1b[24~";
	case KEY_KPENTER: return "\r";
	case KEY_KPSLASH: return "/";
	}
	return NULL;
}

static bool
console_key(struct evdev_dev *evdev, uint16_t code)
{
	return bit_test(evdev->ev_key_states, code) != 0;
}

static void
evdev_console_set_leds(thread_call_param_t p0, thread_call_param_t p1)
{
	(void)p0;
	(void)p1;
	lck_mtx_lock(&evdev_console_mtx);
	bool caps = evdev_console_caps;
	for (int i = 0; i < CONSOLE_MAX_KEYBOARDS; i++) {
		struct evdev_dev *evdev = evdev_console_leds[i];
		if (evdev != NULL) {
			evdev_inject_event(evdev, EV_LED, LED_CAPSL, caps);
			evdev_inject_event(evdev, EV_SYN, SYN_REPORT, 1);
		}
	}
	lck_mtx_unlock(&evdev_console_mtx);
}

void
evdev_console_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{
	if (type != EV_KEY || value == KEY_EVENT_UP || pie_console_graphics())
		return;

	if (code == KEY_CAPSLOCK) {
		if (value == KEY_EVENT_DOWN) {
			evdev_console_caps = !evdev_console_caps;
			if (evdev_console_led_call != NULL)
				thread_call_enter(evdev_console_led_call);
		}
		return;
	}

	bool shift = console_key(evdev, KEY_LEFTSHIFT) || console_key(evdev, KEY_RIGHTSHIFT);
	bool ctrl = console_key(evdev, KEY_LEFTCTRL) || console_key(evdev, KEY_RIGHTCTRL);
	bool alt = console_key(evdev, KEY_LEFTALT) || console_key(evdev, KEY_RIGHTALT);

	const char *seq = console_sequence(code);
	if (seq != NULL) {
		for (; *seq != '\0'; seq++)
			cons_cinput(*seq);
		return;
	}
	if (code >= nitems(console_plain) || console_plain[code] == 0)
		return;

	char c = console_plain[code];
	bool letter = c >= 'a' && c <= 'z';
	if (shift != (letter && evdev_console_caps))
		c = console_shifted[code];
	if (ctrl && c >= '@' && c <= '~')
		c = (char)(c & 0x1f);
	if (alt)
		cons_cinput(0x1b);
	cons_cinput(c);
}

bool
evdev_console_wanted(struct evdev_dev *evdev)
{
	return evdev->ev_console;
}

void
evdev_console_attach(struct evdev_dev *evdev)
{
	int first;

	if (!evdev_event_supported(evdev, EV_KEY))
		return;
	bit_ffs_at(evdev->ev_key_flags, KEY_ESC, KEY_KPDOT + 1, &first);
	if (first == -1)
		return;
	evdev->ev_console = true;

	if (!bit_test(evdev->ev_flags, EVDEV_FLAG_CONSOLE_LEDS) ||
	    !bit_test(evdev->ev_led_flags, LED_CAPSL))
		return;
	lck_mtx_lock(&evdev_console_mtx);
	if (evdev_console_led_call == NULL)
		evdev_console_led_call = thread_call_allocate_with_options(
			evdev_console_set_leds, NULL, THREAD_CALL_PRIORITY_KERNEL, 0);
	for (int i = 0; i < CONSOLE_MAX_KEYBOARDS; i++) {
		if (evdev_console_leds[i] == NULL) {
			evdev_console_leds[i] = evdev;
			break;
		}
	}
	bool caps = evdev_console_caps;
	lck_mtx_unlock(&evdev_console_mtx);
	if (caps && evdev_console_led_call != NULL)
		thread_call_enter(evdev_console_led_call);
}

void
evdev_console_detach(struct evdev_dev *evdev)
{
	lck_mtx_lock(&evdev_console_mtx);
	for (int i = 0; i < CONSOLE_MAX_KEYBOARDS; i++) {
		if (evdev_console_leds[i] == evdev)
			evdev_console_leds[i] = NULL;
	}
	lck_mtx_unlock(&evdev_console_mtx);
}
