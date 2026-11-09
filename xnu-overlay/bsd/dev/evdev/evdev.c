/*-
 * Copyright (c) 2014 Jakub Wojciech Klama <jceel@FreeBSD.org>
 * Copyright (c) 2015-2016 Vladimir Kondratyev <wulf@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/* Modified 2026-11-01: the xnu kernel's interfaces are used. */

#include <sys/param.h>
#include <sys/stat.h>
#include <miscfs/devfs/devfs.h>

#include "evdev_compat.h"
#include "evdev.h"
#include "evdev_private.h"
#include "input.h"

#ifdef EVDEV_DEBUG
#define	debugf(evdev, fmt, args...)	printf("evdev: " fmt "\n", ##args)
#else
#define	debugf(evdev, fmt, args...)
#endif

enum evdev_sparse_result
{
	EV_SKIP_EVENT,		/* Event value not changed */
	EV_REPORT_EVENT,	/* Event value changed */
	EV_REPORT_MT_SLOT,	/* Event value and MT slot number changed */
};

MALLOC_DEFINE(M_EVDEV, "evdev", "evdev memory");

/* adb keyboard driver used on powerpc does not support evdev yet */
#if defined(__powerpc__) && !defined(__powerpc64__)
int evdev_rcpt_mask = EVDEV_RCPT_KBDMUX | EVDEV_RCPT_HW_MOUSE;
#else
int evdev_rcpt_mask = EVDEV_RCPT_HW_MOUSE | EVDEV_RCPT_HW_KBD;
#endif
int evdev_sysmouse_t_axis = 0;


static void evdev_start_repeat(struct evdev_dev *, uint16_t);
static void evdev_stop_repeat(struct evdev_dev *);
static int evdev_check_event(struct evdev_dev *, uint16_t, uint16_t, int32_t);

struct evdev_dev *
evdev_alloc(void)
{
	struct evdev_dev *evdev;

	evdev = evdev_malloc(sizeof(struct evdev_dev), M_EVDEV, M_WAITOK | M_ZERO);
	evdev->ev_cdev_uid = UID_ROOT;
	evdev->ev_cdev_gid = EVDEV_GID_INPUT;
	evdev->ev_cdev_mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP;

	return (evdev);
}

void
evdev_free(struct evdev_dev *evdev)
{

	if (evdev != NULL && evdev->ev_registered)
		evdev_unregister(evdev);

	evdev_mfree(evdev, M_EVDEV);
}

static struct input_absinfo *
evdev_alloc_absinfo(void)
{

	return (evdev_malloc(sizeof(struct input_absinfo) * ABS_CNT, M_EVDEV,
	    M_WAITOK | M_ZERO));
}

static void
evdev_free_absinfo(struct input_absinfo *absinfo)
{

	evdev_mfree(absinfo, M_EVDEV);
}

int
evdev_set_report_size(struct evdev_dev *evdev, size_t report_size)
{
	if (report_size > KEY_CNT + REL_CNT + ABS_CNT + MAX_MT_SLOTS * MT_CNT +
	    MSC_CNT + LED_CNT + SND_CNT + SW_CNT + FF_CNT)
		return (EINVAL);

	evdev->ev_report_size = report_size;
	return (0);
}

static size_t
evdev_estimate_report_size(struct evdev_dev *evdev)
{
	size_t size = 0;
	int res;

	/*
	 * Keyboards generate one event per report but other devices with
	 * buttons like mouses can report events simultaneously
	 */
	bit_ffs_at(evdev->ev_key_flags, KEY_OK, KEY_CNT - KEY_OK, &res);
	if (res == -1)
		bit_ffs(evdev->ev_key_flags, BTN_MISC, &res);
	size += (res != -1);
	bit_count(evdev->ev_key_flags, BTN_MISC, KEY_OK - BTN_MISC, &res);
	size += res;

	/* All relative axes can be reported simultaneously */
	bit_count(evdev->ev_rel_flags, 0, REL_CNT, &res);
	size += res;

	/*
	 * All absolute axes can be reported simultaneously.
	 * Multitouch axes can be reported ABS_MT_SLOT times
	 */
	if (evdev->ev_absinfo != NULL) {
		bit_count(evdev->ev_abs_flags, 0, ABS_CNT, &res);
		size += res;
		bit_count(evdev->ev_abs_flags, ABS_MT_FIRST, MT_CNT, &res);
		if (res > 0) {
			res++;	/* ABS_MT_SLOT or SYN_MT_REPORT */
			if (bit_test(evdev->ev_abs_flags, ABS_MT_SLOT))
				/* MT type B */
				size += res * MAXIMAL_MT_SLOT(evdev);
			else
				/* MT type A */
				size += res * (MAX_MT_REPORTS - 1);
		}
	}

	/* All misc events can be reported simultaneously */
	bit_count(evdev->ev_msc_flags, 0, MSC_CNT, &res);
	size += res;

	/* All leds can be reported simultaneously */
	bit_count(evdev->ev_led_flags, 0, LED_CNT, &res);
	size += res;

	/* Assume other events are generated once per report */
	bit_ffs(evdev->ev_snd_flags, SND_CNT, &res);
	size += (res != -1);

	bit_ffs(evdev->ev_sw_flags, SW_CNT, &res);
	size += (res != -1);

	/* XXX: FF part is not implemented yet */

	size++;		/* SYN_REPORT */
	return (size);
}

static int
evdev_register_common(struct evdev_dev *evdev)
{
	int ret;

	debugf(evdev, "%s: registered evdev provider: %s <%s>\n",
	    evdev->ev_shortname, evdev->ev_name, evdev->ev_serial);

	/* Initialize internal structures */
	CK_SLIST_INIT(&evdev->ev_clients);
	sx_init(&evdev->ev_list_lock, "evsx");

	if (evdev_event_supported(evdev, EV_REP) &&
	    bit_test(evdev->ev_flags, EVDEV_FLAG_SOFTREPEAT)) {
		/* Initialize callout */
		callout_init_mtx(&evdev->ev_rep_callout,
		    evdev->ev_state_lock, 0);

		if (evdev->ev_rep[REP_DELAY] == 0 &&
		    evdev->ev_rep[REP_PERIOD] == 0) {
			/* Supply default values */
			evdev->ev_rep[REP_DELAY] = 250;
			evdev->ev_rep[REP_PERIOD] = 33;
		}
	}

	/* Initialize multitouch protocol type B states or A to B converter */
	if (bit_test(evdev->ev_abs_flags, ABS_MT_SLOT) ||
	    bit_test(evdev->ev_flags, EVDEV_FLAG_MT_TRACK))
		evdev_mt_init(evdev);

	/* Estimate maximum report size */
	if (evdev->ev_report_size == 0) {
		ret = evdev_set_report_size(evdev,
		    evdev_estimate_report_size(evdev));
		if (ret != 0)
			goto bail_out;
	}

	/* Create char device node */
	ret = evdev_cdev_create(evdev);
	if (ret != 0)
		goto bail_out;

	evdev_console_attach(evdev);

bail_out:
	if (ret != 0) {
		if (evdev->ev_rep_callout.c_call != NULL)
			callout_drain(&evdev->ev_rep_callout);
		sx_destroy(&evdev->ev_list_lock);
	}
	return (ret);
}

int
evdev_register(struct evdev_dev *evdev)
{
	int ret;

	if (bit_test(evdev->ev_flags, EVDEV_FLAG_EXT_EPOCH))
		evdev->ev_lock_type = EV_LOCK_EXT_EPOCH;
	else
		evdev->ev_lock_type = EV_LOCK_INTERNAL;
	evdev->ev_state_lock = &evdev->ev_mtx;
	mtx_init(&evdev->ev_mtx, "evmtx", NULL, MTX_DEF);

	ret = evdev_register_common(evdev);
	if (ret != 0)
		mtx_destroy(&evdev->ev_mtx);

	return (ret);
}

int
evdev_register_mtx(struct evdev_dev *evdev, struct mtx *mtx)
{

	evdev->ev_lock_type = EV_LOCK_MTX;
	evdev->ev_state_lock = mtx;
	return (evdev_register_common(evdev));
}

int
evdev_unregister(struct evdev_dev *evdev)
{
	struct evdev_client *client, *tmp;
	int ret;
	debugf(evdev, "%s: unregistered evdev provider: %s\n",
	    evdev->ev_shortname, evdev->ev_name);

	evdev_console_detach(evdev);

	ret = evdev_cdev_destroy(evdev);

	EVDEV_LIST_LOCK(evdev);
	/* Wake up sleepers */
	CK_SLIST_FOREACH_SAFE(client, &evdev->ev_clients, ec_link, tmp) {
		evdev_revoke_client(client);
		evdev_dispose_client(evdev, client);
		EVDEV_CLIENT_LOCKQ(client);
		evdev_notify_event(client);
		EVDEV_CLIENT_UNLOCKQ(client);
		evdev_cdev_release_client(client);
	}
	EVDEV_LIST_UNLOCK(evdev);

	if (evdev->ev_rep_callout.c_call != NULL)
		callout_drain(&evdev->ev_rep_callout);
	evdev->ev_cdev = NULL;
	sx_destroy(&evdev->ev_list_lock);
	if (ret == 0 && evdev->ev_lock_type != EV_LOCK_MTX)
		mtx_destroy(&evdev->ev_mtx);

	evdev_free_absinfo(evdev->ev_absinfo);
	evdev_mt_free(evdev);

	return (ret);
}

inline void
evdev_set_name(struct evdev_dev *evdev, const char *name)
{

	snprintf(evdev->ev_name, NAMELEN, "%s", name);
}

inline void
evdev_set_id(struct evdev_dev *evdev, uint16_t bustype, uint16_t vendor,
    uint16_t product, uint16_t version)
{

	evdev->ev_id = (struct input_id) {
		.bustype = bustype,
		.vendor = vendor,
		.product = product,
		.version = version
	};
}

inline void
evdev_set_phys(struct evdev_dev *evdev, const char *name)
{

	snprintf(evdev->ev_shortname, NAMELEN, "%s", name);
}

inline void
evdev_set_serial(struct evdev_dev *evdev, const char *serial)
{

	snprintf(evdev->ev_serial, NAMELEN, "%s", serial);
}

inline void
evdev_set_methods(struct evdev_dev *evdev, void *softc,
    const struct evdev_methods *methods)
{

	evdev->ev_methods = methods;
	evdev->ev_softc = softc;
}

inline void *
evdev_get_softc(struct evdev_dev *evdev)
{

	return (evdev->ev_softc);
}

inline void
evdev_support_prop(struct evdev_dev *evdev, uint16_t prop)
{

	KASSERT(prop < INPUT_PROP_CNT, ("invalid evdev input property"));
	bit_set(evdev->ev_prop_flags, prop);
}

inline void
evdev_support_event(struct evdev_dev *evdev, uint16_t type)
{

	KASSERT(type < EV_CNT, ("invalid evdev event property"));
	bit_set(evdev->ev_type_flags, type);
}

inline void
evdev_support_key(struct evdev_dev *evdev, uint16_t code)
{

	KASSERT(code < KEY_CNT, ("invalid evdev key property"));
	bit_set(evdev->ev_key_flags, code);
}

inline void
evdev_support_rel(struct evdev_dev *evdev, uint16_t code)
{

	KASSERT(code < REL_CNT, ("invalid evdev rel property"));
	bit_set(evdev->ev_rel_flags, code);
}

inline void
evdev_support_abs(struct evdev_dev *evdev, uint16_t code, int32_t minimum,
    int32_t maximum, int32_t fuzz, int32_t flat, int32_t resolution)
{
	struct input_absinfo absinfo;

	KASSERT(code < ABS_CNT, ("invalid evdev abs property"));

	absinfo = (struct input_absinfo) {
		.value = 0,
		.minimum = minimum,
		.maximum = maximum,
		.fuzz = fuzz,
		.flat = flat,
		.resolution = resolution,
	};
	evdev_set_abs_bit(evdev, code);
	evdev_set_absinfo(evdev, code, &absinfo);
}

inline void
evdev_set_abs_bit(struct evdev_dev *evdev, uint16_t code)
{

	KASSERT(code < ABS_CNT, ("invalid evdev abs property"));
	if (evdev->ev_absinfo == NULL)
		evdev->ev_absinfo = evdev_alloc_absinfo();
	bit_set(evdev->ev_abs_flags, code);
}

inline void
evdev_support_msc(struct evdev_dev *evdev, uint16_t code)
{

	KASSERT(code < MSC_CNT, ("invalid evdev msc property"));
	bit_set(evdev->ev_msc_flags, code);
}


inline void
evdev_support_led(struct evdev_dev *evdev, uint16_t code)
{

	KASSERT(code < LED_CNT, ("invalid evdev led property"));
	bit_set(evdev->ev_led_flags, code);
}

inline void
evdev_support_snd(struct evdev_dev *evdev, uint16_t code)
{

	KASSERT(code < SND_CNT, ("invalid evdev snd property"));
	bit_set(evdev->ev_snd_flags, code);
}

inline void
evdev_support_sw(struct evdev_dev *evdev, uint16_t code)
{

	KASSERT(code < SW_CNT, ("invalid evdev sw property"));
	bit_set(evdev->ev_sw_flags, code);
}

bool
evdev_event_supported(struct evdev_dev *evdev, uint16_t type)
{

	KASSERT(type < EV_CNT, ("invalid evdev event property"));
	return (bit_test(evdev->ev_type_flags, type));
}

inline void
evdev_set_absinfo(struct evdev_dev *evdev, uint16_t axis,
    struct input_absinfo *absinfo)
{

	KASSERT(axis < ABS_CNT, ("invalid evdev abs property"));

	if (axis == ABS_MT_SLOT &&
	    (absinfo->maximum < 1 || absinfo->maximum >= MAX_MT_SLOTS))
		return;

	if (evdev->ev_absinfo == NULL)
		evdev->ev_absinfo = evdev_alloc_absinfo();

	if (axis == ABS_MT_SLOT)
		evdev->ev_absinfo[ABS_MT_SLOT].maximum = absinfo->maximum;
	else
		memcpy(&evdev->ev_absinfo[axis], absinfo,
		    sizeof(struct input_absinfo));
}

inline void
evdev_set_repeat_params(struct evdev_dev *evdev, uint16_t property, int value)
{

	KASSERT(property < REP_CNT, ("invalid evdev repeat property"));
	evdev->ev_rep[property] = value;
}

inline void
evdev_set_flag(struct evdev_dev *evdev, uint16_t flag)
{

	KASSERT(flag < EVDEV_FLAG_CNT, ("invalid evdev flag property"));
	bit_set(evdev->ev_flags, flag);
}

void
evdev_set_cdev_mode(struct evdev_dev *evdev, uid_t uid, gid_t gid, int mode)
{
	evdev->ev_cdev_uid = uid;
	evdev->ev_cdev_gid = gid;
	evdev->ev_cdev_mode = mode;
}

static int
evdev_check_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{

	if (type >= EV_CNT)
		return (EINVAL);

	/* Allow SYN events implicitly */
	if (type != EV_SYN && !evdev_event_supported(evdev, type))
		return (EINVAL);

	switch (type) {
	case EV_SYN:
		if (code >= SYN_CNT)
			return (EINVAL);
		break;

	case EV_KEY:
		if (code >= KEY_CNT)
			return (EINVAL);
		if (!bit_test(evdev->ev_key_flags, code))
			return (EINVAL);
		break;

	case EV_REL:
		if (code >= REL_CNT)
			return (EINVAL);
		if (!bit_test(evdev->ev_rel_flags, code))
			return (EINVAL);
		break;

	case EV_ABS:
		if (code >= ABS_CNT)
			return (EINVAL);
		if (!bit_test(evdev->ev_abs_flags, code))
			return (EINVAL);
		if (code == ABS_MT_SLOT &&
		    (value < 0 || value > MAXIMAL_MT_SLOT(evdev)))
			return (EINVAL);
		if (ABS_IS_MT(code) && evdev->ev_mt == NULL &&
		    bit_test(evdev->ev_abs_flags, ABS_MT_SLOT))
			return (EINVAL);
		break;

	case EV_MSC:
		if (code >= MSC_CNT)
			return (EINVAL);
		if (!bit_test(evdev->ev_msc_flags, code))
			return (EINVAL);
		break;

	case EV_LED:
		if (code >= LED_CNT)
			return (EINVAL);
		if (!bit_test(evdev->ev_led_flags, code))
			return (EINVAL);
		break;

	case EV_SND:
		if (code >= SND_CNT)
			return (EINVAL);
		if (!bit_test(evdev->ev_snd_flags, code))
			return (EINVAL);
		break;

	case EV_SW:
		if (code >= SW_CNT)
			return (EINVAL);
		if (!bit_test(evdev->ev_sw_flags, code))
			return (EINVAL);
		break;

	case EV_REP:
		if (code >= REP_CNT)
			return (EINVAL);
		break;

	default:
		return (EINVAL);
	}

	return (0);
}

static void
evdev_modify_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t *value)
{
	int32_t fuzz, old_value, abs_change;

	EVDEV_LOCK_ASSERT(evdev);

	switch (type) {
	case EV_KEY:
		if (!evdev_event_supported(evdev, EV_REP))
			break;

		if (!bit_test(evdev->ev_flags, EVDEV_FLAG_SOFTREPEAT)) {
			/* Detect driver key repeats. */
			if (bit_test(evdev->ev_key_states, code) &&
			    *value == KEY_EVENT_DOWN)
				*value = KEY_EVENT_REPEAT;
		} else {
			/* Start/stop callout for evdev repeats */
			if (bit_test(evdev->ev_key_states, code) == !*value &&
			    (!CK_SLIST_EMPTY(&evdev->ev_clients) ||
			    evdev_console_wanted(evdev))) {
				if (*value == KEY_EVENT_DOWN)
					evdev_start_repeat(evdev, code);
				else
					evdev_stop_repeat(evdev);
			}
		}
		break;

	case EV_ABS:
		if (code == ABS_MT_SLOT)
			break;
		else if (!ABS_IS_MT(code))
			old_value = evdev->ev_absinfo[code].value;
		else if (!bit_test(evdev->ev_abs_flags, ABS_MT_SLOT))
			/* Pass MT protocol type A events as is */
			break;
		else if (code == ABS_MT_TRACKING_ID) {
			*value = evdev_mt_reassign_id(evdev,
			    evdev_mt_get_last_slot(evdev), *value);
			break;
		} else
			old_value = evdev_mt_get_value(evdev,
			    evdev_mt_get_last_slot(evdev), code);

		fuzz = evdev->ev_absinfo[code].fuzz;
		if (fuzz == 0)
			break;

		abs_change = abs(*value - old_value);
		if (abs_change < fuzz / 2)
			*value = old_value;
		else if (abs_change < fuzz)
			*value = (old_value * 3 + *value) / 4;
		else if (abs_change < fuzz * 2)
			*value = (old_value + *value) / 2;
		break;
	}
}

static enum evdev_sparse_result
evdev_sparse_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{
	int32_t last_mt_slot;

	EVDEV_LOCK_ASSERT(evdev);

	/*
	 * For certain event types, update device state bits
	 * and convert level reporting to edge reporting
	 */
	switch (type) {
	case EV_KEY:
		switch (value) {
		case KEY_EVENT_UP:
		case KEY_EVENT_DOWN:
			if (bit_test(evdev->ev_key_states, code) == value)
				return (EV_SKIP_EVENT);
			bit_change(evdev->ev_key_states, code, value);
			break;

		case KEY_EVENT_REPEAT:
			if (bit_test(evdev->ev_key_states, code) == 0 ||
			    !evdev_event_supported(evdev, EV_REP))
				return (EV_SKIP_EVENT);
			break;

		default:
			 return (EV_SKIP_EVENT);
		}
		break;

	case EV_LED:
		if (bit_test(evdev->ev_led_states, code) == value)
			return (EV_SKIP_EVENT);
		bit_change(evdev->ev_led_states, code, value);
		break;

	case EV_SND:
		bit_change(evdev->ev_snd_states, code, value);
		break;

	case EV_SW:
		if (bit_test(evdev->ev_sw_states, code) == value)
			return (EV_SKIP_EVENT);
		bit_change(evdev->ev_sw_states, code, value);
		break;

	case EV_REP:
		if (evdev->ev_rep[code] == value)
			return (EV_SKIP_EVENT);
		evdev_set_repeat_params(evdev, code, value);
		break;

	case EV_REL:
		if (value == 0)
			return (EV_SKIP_EVENT);
		break;

	/* For EV_ABS, save last value in absinfo and ev_mt_states */
	case EV_ABS:
		switch (code) {
		case ABS_MT_SLOT:
			/* Postpone ABS_MT_SLOT till next event */
			evdev_mt_set_last_slot(evdev, value);
			return (EV_SKIP_EVENT);

		case ABS_MT_FIRST ... ABS_MT_LAST:
			/* Pass MT protocol type A events as is */
			if (!bit_test(evdev->ev_abs_flags, ABS_MT_SLOT))
				break;
			/* Don`t repeat MT protocol type B events */
			last_mt_slot = evdev_mt_get_last_slot(evdev);
			if (evdev_mt_get_value(evdev, last_mt_slot, code)
			     == value)
				return (EV_SKIP_EVENT);
			evdev_mt_set_value(evdev, last_mt_slot, code, value);
			if (last_mt_slot != CURRENT_MT_SLOT(evdev)) {
				CURRENT_MT_SLOT(evdev) = last_mt_slot;
				evdev->ev_report_opened = true;
				return (EV_REPORT_MT_SLOT);
			}
			break;

		default:
			if (evdev->ev_absinfo[code].value == value)
				return (EV_SKIP_EVENT);
			evdev->ev_absinfo[code].value = value;
		}
		break;

	case EV_SYN:
		if (code == SYN_REPORT) {
			/* Count empty reports as well as non empty */
			evdev->ev_report_count++;
			/* Skip empty reports */
			if (!evdev->ev_report_opened)
				return (EV_SKIP_EVENT);
			evdev->ev_report_opened = false;
			return (EV_REPORT_EVENT);
		}
		break;
	}

	evdev->ev_report_opened = true;
	return (EV_REPORT_EVENT);
}

static void
evdev_propagate_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{
	struct evdev_client *client;

	debugf(evdev, "%s pushed event %d/%d/%d",
	    evdev->ev_shortname, type, code, value);

	EVDEV_LOCK_ASSERT(evdev);

	CK_SLIST_FOREACH(client, &evdev->ev_clients, ec_link) {
		if (evdev->ev_grabber != NULL && evdev->ev_grabber != client)
			continue;

		EVDEV_CLIENT_LOCKQ(client);
		evdev_client_push(client, type, code, value);
		if (type == EV_SYN && code == SYN_REPORT)
			evdev_notify_event(client);
		EVDEV_CLIENT_UNLOCKQ(client);
	}

	if (evdev->ev_console && evdev->ev_grabber == NULL)
		evdev_console_event(evdev, type, code, value);

	evdev->ev_event_count++;
}

void
evdev_send_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{
	enum evdev_sparse_result sparse;

	EVDEV_LOCK_ASSERT(evdev);

	evdev_modify_event(evdev, type, code, &value);
	sparse =  evdev_sparse_event(evdev, type, code, value);
	switch (sparse) {
	case EV_REPORT_MT_SLOT:
		/* report postponed ABS_MT_SLOT */
		evdev_propagate_event(evdev, EV_ABS, ABS_MT_SLOT,
		    CURRENT_MT_SLOT(evdev));
		/* FALLTHROUGH */
	case EV_REPORT_EVENT:
		evdev_propagate_event(evdev, type, code, value);
		/* FALLTHROUGH */
	case EV_SKIP_EVENT:
		break;
	}
}

void
evdev_restore_after_kdb(struct evdev_dev *evdev)
{
	int code;

	EVDEV_LOCK_ASSERT(evdev);

	/* Report postponed leds */
	bit_foreach(evdev->ev_kdb_led_states, LED_CNT, code)
		evdev_send_event(evdev, EV_LED, code,
		    !bit_test(evdev->ev_led_states, code));
	bit_nclear(evdev->ev_kdb_led_states, 0, LED_MAX);

	/* Release stuck keys (CTRL + ALT + ESC) */
	evdev_stop_repeat(evdev);
	bit_foreach(evdev->ev_key_states, KEY_CNT, code)
		evdev_send_event(evdev, EV_KEY, code, KEY_EVENT_UP);
	evdev_send_event(evdev, EV_SYN, SYN_REPORT, 1);
}

int
evdev_push_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{

	if (evdev_check_event(evdev, type, code, value) != 0)
		return (EINVAL);

	/*
	 * Discard all but LEDs kdb events as unrelated to userspace.
	 * Aggregate LED updates and postpone reporting until kdb deactivation.
	 */
	if (kdb_active || SCHEDULER_STOPPED()) {
		evdev->ev_kdb_active = true;
		if (type == EV_LED)
			bit_set(evdev->ev_kdb_led_states,
			    bit_test(evdev->ev_led_states, code) != value);
		return (0);
	}

	EVDEV_ENTER(evdev);

	/* Fix evdev state corrupted with discarding of kdb events */
	if (evdev->ev_kdb_active) {
		evdev->ev_kdb_active = false;
		evdev_restore_after_kdb(evdev);
	}

	if (type == EV_SYN && code == SYN_REPORT &&
	    bit_test(evdev->ev_abs_flags, ABS_MT_SLOT))
		evdev_mt_sync_frame(evdev);
	else
		if (bit_test(evdev->ev_flags, EVDEV_FLAG_MT_TRACK) &&
		    evdev_mt_record_event(evdev, type, code, value))
			goto exit;

	evdev_send_event(evdev, type, code, value);
exit:
	EVDEV_EXIT(evdev);

	return (0);
}

int
evdev_inject_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{
	int ret = 0;

	switch (type) {
	case EV_REP:
		/* evdev repeats should not be processed by hardware driver */
		if (bit_test(evdev->ev_flags, EVDEV_FLAG_SOFTREPEAT))
			goto push;
		/* FALLTHROUGH */
	case EV_LED:
	case EV_MSC:
	case EV_SND:
	case EV_FF:
		if (evdev->ev_methods != NULL &&
		    evdev->ev_methods->ev_event != NULL)
			evdev->ev_methods->ev_event(evdev, type, code, value);
		/*
		 * Leds and driver repeats should be reported in ev_event
		 * method body to interoperate with kbdmux states and rates
		 * propagation so both ways (ioctl and evdev) of changing it
		 * will produce only one evdev event report to client.
		 */
		if (type == EV_LED || type == EV_REP)
			break;
		/* FALLTHROUGH */
	case EV_SYN:
	case EV_KEY:
	case EV_REL:
	case EV_ABS:
	case EV_SW:
push:
		if (evdev->ev_lock_type == EV_LOCK_MTX)
			EVDEV_LOCK(evdev);
		ret = evdev_push_event(evdev, type,  code, value);
		if (evdev->ev_lock_type == EV_LOCK_MTX)
			EVDEV_UNLOCK(evdev);

		break;

	default:
		ret = EINVAL;
	}

	return (ret);
}

int
evdev_register_client(struct evdev_dev *evdev, struct evdev_client *client)
{
	int ret = 0;

	debugf(evdev, "adding new client for device %s", evdev->ev_shortname);

	EVDEV_LIST_LOCK_ASSERT(evdev);

	if (CK_SLIST_EMPTY(&evdev->ev_clients) && evdev->ev_methods != NULL &&
	    evdev->ev_methods->ev_open != NULL) {
		debugf(evdev, "calling ev_open() on device %s",
		    evdev->ev_shortname);
		ret = evdev->ev_methods->ev_open(evdev);
	}
	if (ret == 0) {
		if (evdev->ev_lock_type != EV_LOCK_MTX)
			EVDEV_LOCK(evdev);
		CK_SLIST_INSERT_HEAD(&evdev->ev_clients, client, ec_link);
		if (evdev->ev_lock_type != EV_LOCK_MTX)
			EVDEV_UNLOCK(evdev);
	}
	return (ret);
}

void
evdev_dispose_client(struct evdev_dev *evdev, struct evdev_client *client)
{
	debugf(evdev, "removing client for device %s", evdev->ev_shortname);

	EVDEV_LIST_LOCK_ASSERT(evdev);

	if (evdev->ev_lock_type != EV_LOCK_MTX)
		EVDEV_LOCK(evdev);
	CK_SLIST_REMOVE(&evdev->ev_clients, client, evdev_client, ec_link);
	if (evdev->ev_lock_type != EV_LOCK_MTX)
		EVDEV_UNLOCK(evdev);
	if (CK_SLIST_EMPTY(&evdev->ev_clients)) {
		if (evdev->ev_methods != NULL &&
		    evdev->ev_methods->ev_close != NULL)
			(void)evdev->ev_methods->ev_close(evdev);
		if (evdev_event_supported(evdev, EV_REP) &&
		    bit_test(evdev->ev_flags, EVDEV_FLAG_SOFTREPEAT) &&
		    !evdev_console_wanted(evdev)) {
			if (evdev->ev_lock_type != EV_LOCK_MTX)
				EVDEV_LOCK(evdev);
			evdev_stop_repeat(evdev);
			if (evdev->ev_lock_type != EV_LOCK_MTX)
				EVDEV_UNLOCK(evdev);
		}
	}
	if (evdev->ev_lock_type != EV_LOCK_MTX)
		EVDEV_LOCK(evdev);
	evdev_release_client(evdev, client);
	if (evdev->ev_lock_type != EV_LOCK_MTX)
		EVDEV_UNLOCK(evdev);
}

int
evdev_grab_client(struct evdev_dev *evdev, struct evdev_client *client)
{

	EVDEV_LOCK_ASSERT(evdev);

	if (evdev->ev_grabber != NULL)
		return (EBUSY);

	evdev->ev_grabber = client;

	return (0);
}

int
evdev_release_client(struct evdev_dev *evdev, struct evdev_client *client)
{

	EVDEV_LOCK_ASSERT(evdev);

	if (evdev->ev_grabber != client)
		return (EINVAL);

	evdev->ev_grabber = NULL;

	return (0);
}

int
evdev_get_unit(struct evdev_dev *evdev)
{

	return (evdev->ev_registered ? evdev->ev_unit : -1);
}

bool
evdev_is_grabbed(struct evdev_dev *evdev)
{
	if (kdb_active || SCHEDULER_STOPPED())
		return (false);
	/*
	 * The function is intended to be called from evdev-unrelated parts of
	 * code like syscons-compatible parts of mouse and keyboard drivers.
	 * That makes unlocked read-only access acceptable.
	 */
	return (evdev->ev_grabber != NULL);
}

static void
evdev_repeat_callout(void *arg)
{
	struct evdev_dev *evdev = (struct evdev_dev *)arg;

	evdev_send_event(evdev, EV_KEY, evdev->ev_rep_key, KEY_EVENT_REPEAT);
	evdev_send_event(evdev, EV_SYN, SYN_REPORT, 1);

	if (evdev->ev_rep[REP_PERIOD])
		callout_reset_ms(&evdev->ev_rep_callout,
		    evdev->ev_rep[REP_PERIOD],
		    evdev_repeat_callout, evdev);
	else
		evdev->ev_rep_key = KEY_RESERVED;
}

static void
evdev_start_repeat(struct evdev_dev *evdev, uint16_t key)
{

	EVDEV_LOCK_ASSERT(evdev);

	if (evdev->ev_rep[REP_DELAY]) {
		evdev->ev_rep_key = key;
		callout_reset_ms(&evdev->ev_rep_callout,
		    evdev->ev_rep[REP_DELAY],
		    evdev_repeat_callout, evdev);
	}
}

static void
evdev_stop_repeat(struct evdev_dev *evdev)
{

	EVDEV_LOCK_ASSERT(evdev);

	if (evdev->ev_rep_key != KEY_RESERVED) {
		callout_stop(&evdev->ev_rep_callout);
		evdev->ev_rep_key = KEY_RESERVED;
	}
}

static void
callout_thunk(thread_call_param_t p0, thread_call_param_t p1)
{
	struct callout *c = p0;

	lck_mtx_lock(c->c_mtx);
	if (c->c_pending && c->c_gen == (uintptr_t)p1) {
		c->c_pending = false;
		c->c_fn(c->c_arg);
	}
	lck_mtx_unlock(c->c_mtx);
}

void
callout_init_mtx(struct callout *c, struct mtx *mx, int flags)
{
	(void)flags;
	bzero(c, sizeof(*c));
	c->c_mtx = &mx->m;
	c->c_call = thread_call_allocate_with_options(callout_thunk, c,
	    THREAD_CALL_PRIORITY_KERNEL, 0);
}

void
callout_reset_ms(struct callout *c, int ms, void (*fn)(void *), void *arg)
{
	uint64_t deadline;

	LCK_MTX_ASSERT(c->c_mtx, LCK_MTX_ASSERT_OWNED);
	c->c_fn = fn;
	c->c_arg = arg;
	c->c_gen++;
	c->c_pending = true;
	clock_interval_to_deadline(ms > 0 ? ms : 1, NSEC_PER_MSEC, &deadline);
	thread_call_enter1_delayed(c->c_call, (thread_call_param_t)c->c_gen,
	    deadline);
}

void
callout_stop(struct callout *c)
{
	LCK_MTX_ASSERT(c->c_mtx, LCK_MTX_ASSERT_OWNED);
	c->c_pending = false;
	thread_call_cancel(c->c_call);
}

void
callout_drain(struct callout *c)
{
	if (c->c_call == NULL)
		return;
	lck_mtx_lock(c->c_mtx);
	c->c_pending = false;
	lck_mtx_unlock(c->c_mtx);
	thread_call_cancel_wait(c->c_call);
	thread_call_free(c->c_call);
	c->c_call = NULL;
}
