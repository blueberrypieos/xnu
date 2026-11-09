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
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/filio.h>
#include <sys/ttycom.h>
#include <sys/ioccom.h>
#include <sys/proc.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <sys/vnode.h>
#include <miscfs/devfs/devfs.h>

#include "evdev_compat.h"
#include "evdev.h"
#include "evdev_private.h"
#include "input.h"

#ifdef EVDEV_DEBUG
#define	debugf(client, fmt, args...)	printf("evdev cdev: "fmt"\n", ##args)
#else
#define	debugf(client, fmt, args...)
#endif

#define	DEF_RING_REPORTS	8


#define	EVDEV_MAX_UNITS		256
#define	EVDEV_MINOR(unit, inst)	((unit) << 16 | (inst))
#define	EVDEV_UNIT(dev)		(minor(dev) >> 16)
#define	EVDEV_INSTANCE(dev)	(minor(dev) & 0xffff)

#define	EVDEV_CLOCK_REALTIME		0
#define	EVDEV_CLOCK_MONOTONIC		6
#define	EVDEV_CLOCK_MONOTONIC_LINUX	1

LCK_GRP_DECLARE(evdev_lck_grp, "evdev");
static LCK_MTX_DECLARE(evdev_registry_mtx, &evdev_lck_grp);
static LCK_MTX_DECLARE(evdev_nodes_mtx, &evdev_lck_grp);

static struct evdev_dev *evdev_units[EVDEV_MAX_UNITS];
static LIST_HEAD(, evdev_client) evdev_clients =
    LIST_HEAD_INITIALIZER(evdev_clients);
static uint32_t evdev_next_instance;
static int evdev_major = -1;
static bool evdev_devfs_ready;

static open_close_fcn_t	evdev_open;
static open_close_fcn_t	evdev_close;
static read_write_fcn_t	evdev_read;
static read_write_fcn_t	evdev_write;
static ioctl_fcn_t	evdev_ioctl;
static select_fcn_t	evdev_select;

static int evdev_read_client(struct evdev_client *, struct uio *, int);
static int evdev_ioctl_client(struct evdev_dev *, struct evdev_client *,
    u_long, caddr_t);
static int evdev_ioctl_eviocgbit(struct evdev_dev *, int, int, caddr_t);
static void evdev_client_filter_queue(struct evdev_client *, uint16_t);

static const struct cdevsw evdev_cdevsw = {
	.d_open = evdev_open,
	.d_close = evdev_close,
	.d_read = evdev_read,
	.d_write = evdev_write,
	.d_ioctl = evdev_ioctl,
	.d_stop = eno_stop,
	.d_reset = eno_reset,
	.d_ttys = NULL,
	.d_select = evdev_select,
	.d_mmap = eno_mmap,
	.d_strategy = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type = 0,
};

static struct evdev_client *
evdev_client_find(dev_t dev)
{
	struct evdev_client *client;

	LIST_FOREACH(client, &evdev_clients, ec_dev_link) {
		if (client->ec_dev == dev)
			return (client);
	}
	return (NULL);
}

static struct evdev_client *
evdev_client_get(dev_t dev)
{
	struct evdev_client *client;

	lck_mtx_lock(&evdev_registry_mtx);
	client = evdev_client_find(dev);
	if (client != NULL)
		client->ec_refs++;
	lck_mtx_unlock(&evdev_registry_mtx);
	return (client);
}

static void
evdev_client_free(struct evdev_client *client)
{
	selthreadclear(&client->ec_selp);
	mtx_destroy(&client->ec_buffer_mtx);
	evdev_mfree(client, M_EVDEV);
}

void
evdev_cdev_release_client(struct evdev_client *client)
{
	uint32_t refs;

	lck_mtx_lock(&evdev_registry_mtx);
	refs = --client->ec_refs;
	lck_mtx_unlock(&evdev_registry_mtx);
	if (refs == 0)
		evdev_client_free(client);
}

static struct evdev_dev *
evdev_client_acquire_dev(struct evdev_client *client)
{
	struct evdev_dev *evdev = NULL;

	lck_mtx_lock(&evdev_registry_mtx);
	if (!client->ec_revoked &&
	    evdev_units[client->ec_unit] == client->ec_evdev) {
		evdev = client->ec_evdev;
		evdev->ev_cdev_active++;
	}
	lck_mtx_unlock(&evdev_registry_mtx);
	return (evdev);
}

static void
evdev_dev_release(struct evdev_dev *evdev)
{
	lck_mtx_lock(&evdev_registry_mtx);
	if (--evdev->ev_cdev_active == 0)
		wakeup(&evdev->ev_cdev_active);
	lck_mtx_unlock(&evdev_registry_mtx);
}

static int
evdev_clone(dev_t dev, int action)
{
	int unit = EVDEV_UNIT(dev);
	uint32_t inst;
	int tries = 0;

	if (action != DEVFS_CLONE_ALLOC)
		return (0);
	lck_mtx_lock(&evdev_registry_mtx);
	do {
		inst = ++evdev_next_instance & 0xffff;
		if (++tries > 0x10000) {
			lck_mtx_unlock(&evdev_registry_mtx);
			return (-1);
		}
	} while (inst == 0 || evdev_client_find(makedev(major(dev),
	    EVDEV_MINOR(unit, inst))) != NULL);
	lck_mtx_unlock(&evdev_registry_mtx);
	return (EVDEV_MINOR(unit, inst));
}

static int
evdev_open(dev_t dev, int oflags, int devtype, proc_t p)
{
	struct evdev_dev *evdev;
	struct evdev_client *client;
	size_t buffer_size;
	int ret, unit = EVDEV_UNIT(dev);

	(void)oflags;
	(void)devtype;
	(void)p;
	if (EVDEV_INSTANCE(dev) == 0 || unit >= EVDEV_MAX_UNITS)
		return (ENXIO);

	lck_mtx_lock(&evdev_registry_mtx);
	if (evdev_client_find(dev) != NULL) {
		lck_mtx_unlock(&evdev_registry_mtx);
		return (0);
	}
	evdev = evdev_units[unit];
	if (evdev == NULL) {
		lck_mtx_unlock(&evdev_registry_mtx);
		return (ENXIO);
	}
	evdev->ev_cdev_active++;
	lck_mtx_unlock(&evdev_registry_mtx);

	/* Initialize client structure */
	buffer_size = evdev->ev_report_size * DEF_RING_REPORTS;
	client = evdev_malloc(offsetof(struct evdev_client, ec_buffer) +
	    sizeof(struct input_event) * buffer_size,
	    M_EVDEV, M_WAITOK | M_ZERO);

	/* Initialize ring buffer */
	client->ec_buffer_size = buffer_size;
	client->ec_buffer_head = 0;
	client->ec_buffer_tail = 0;
	client->ec_buffer_ready = 0;

	client->ec_evdev = evdev;
	client->ec_dev = dev;
	client->ec_unit = unit;
	client->ec_refs = 2;
	mtx_init(&client->ec_buffer_mtx, "evclient", "evdev", MTX_DEF);

	ret = EVDEV_LIST_LOCK_SIG(evdev);
	if (ret == 0) {
		ret = evdev_register_client(evdev, client);
		EVDEV_LIST_UNLOCK(evdev);
	}

	if (ret == 0) {
		lck_mtx_lock(&evdev_registry_mtx);
		LIST_INSERT_HEAD(&evdev_clients, client, ec_dev_link);
		lck_mtx_unlock(&evdev_registry_mtx);
	} else {
		debugf(client, "cannot register evdev client");
		client->ec_revoked = true;
		evdev_client_free(client);
	}
	evdev_dev_release(evdev);

	return (ret);
}

static int
evdev_close(dev_t dev, int flags, int devtype, proc_t p)
{
	struct evdev_dev *evdev;
	struct evdev_client *client;
	bool disposed = false;

	(void)flags;
	(void)devtype;
	(void)p;
	lck_mtx_lock(&evdev_registry_mtx);
	client = evdev_client_find(dev);
	if (client != NULL)
		LIST_REMOVE(client, ec_dev_link);
	lck_mtx_unlock(&evdev_registry_mtx);
	if (client == NULL)
		return (0);

	evdev = evdev_client_acquire_dev(client);
	if (evdev != NULL) {
		EVDEV_LIST_LOCK(evdev);
		if (!client->ec_revoked) {
			evdev_dispose_client(evdev, client);
			evdev_revoke_client(client);
			disposed = true;
		}
		EVDEV_LIST_UNLOCK(evdev);
		evdev_dev_release(evdev);
	}
	if (disposed)
		evdev_cdev_release_client(client);
	evdev_cdev_release_client(client);
	return (0);
}

static int
evdev_select(dev_t dev, int which, void *wql, proc_t p)
{
	struct evdev_client *client;
	int revents = 0;

	client = evdev_client_get(dev);
	if (client == NULL)
		return (1);

	if (client->ec_revoked)
		revents = 1;
	else if (which == FREAD) {
		EVDEV_CLIENT_LOCKQ(client);
		if (!EVDEV_CLIENT_EMPTYQ(client))
			revents = (int)(EVDEV_CLIENT_SIZEQ(client) *
			    sizeof(struct input_event));
		else {
			client->ec_selected = true;
			selrecord(p, &client->ec_selp, wql);
		}
		EVDEV_CLIENT_UNLOCKQ(client);
	} else if (which == FWRITE)
		revents = 1;

	evdev_cdev_release_client(client);
	return (revents);
}

static int
evdev_read(dev_t dev, struct uio *uio, int ioflag)
{
	struct evdev_client *client;
	int ret;

	client = evdev_client_get(dev);
	if (client == NULL)
		return (ENXIO);
	ret = evdev_read_client(client, uio, ioflag);
	evdev_cdev_release_client(client);
	return (ret);
}

static int
evdev_read_client(struct evdev_client *client, struct uio *uio, int ioflag)
{
	union {
		struct input_event t;
	} event;
	struct input_event *head;
	size_t evsize;
	int ret = 0;
	int remaining;

	if (client->ec_revoked)
		return (ENODEV);

	evsize = sizeof(struct input_event);

	/* Zero-sized reads are allowed for error checking */
	if (uio_resid(uio) != 0 && (size_t)uio_resid(uio) < evsize)
		return (EINVAL);

	remaining = (int)(uio_resid(uio) / evsize);

	EVDEV_CLIENT_LOCKQ(client);

	if (EVDEV_CLIENT_EMPTYQ(client)) {
		if (ioflag & IO_NDELAY)
			ret = EWOULDBLOCK;
		else {
			if (remaining != 0) {
				client->ec_blocked = true;
				ret = mtx_sleep(client, &client->ec_buffer_mtx,
				    PCATCH, "evread", 0);
				if (ret == 0 && client->ec_revoked)
					ret = ENODEV;
			}
		}
	}

	while (ret == 0 && !EVDEV_CLIENT_EMPTYQ(client) && remaining > 0) {
		head = client->ec_buffer + client->ec_buffer_head;
		bcopy(head, &event.t, evsize);

		client->ec_buffer_head =
		    (client->ec_buffer_head + 1) % client->ec_buffer_size;
		remaining--;

		EVDEV_CLIENT_UNLOCKQ(client);
		ret = uiomove((char *)&event, (int)evsize, uio);
		EVDEV_CLIENT_LOCKQ(client);
	}

	EVDEV_CLIENT_UNLOCKQ(client);

	return (ret);
}

static int
evdev_write(dev_t dev, struct uio *uio, int ioflag)
{
	struct evdev_dev *evdev;
	struct evdev_client *client;
	union {
		struct input_event t;
	} event;
	size_t evsize;
	int ret = 0;

	(void)ioflag;
	client = evdev_client_get(dev);
	if (client == NULL)
		return (ENXIO);
	evdev = evdev_client_acquire_dev(client);
	if (evdev == NULL) {
		evdev_cdev_release_client(client);
		return (ENODEV);
	}

	evsize = sizeof(struct input_event);

	if (uio_resid(uio) % evsize != 0) {
		debugf(client, "write size not multiple of input_event size");
		ret = EINVAL;
	}

	while (uio_resid(uio) > 0 && ret == 0) {
		ret = uiomove((char *)&event, (int)evsize, uio);
		if (ret == 0) {
			ret = evdev_inject_event(evdev, event.t.type,
			    event.t.code, event.t.value);
		}
	}

	evdev_dev_release(evdev);
	evdev_cdev_release_client(client);
	return (ret);
}

static int
evdev_ioctl(dev_t dev, u_long cmd, caddr_t data, int fflag, proc_t p)
{
	struct evdev_dev *evdev;
	struct evdev_client *client;
	int ret;

	(void)fflag;
	(void)p;
	client = evdev_client_get(dev);
	if (client == NULL)
		return (ENXIO);
	evdev = evdev_client_acquire_dev(client);
	if (evdev == NULL) {
		evdev_cdev_release_client(client);
		return (ENODEV);
	}
	ret = evdev_ioctl_client(evdev, client, cmd, data);
	evdev_dev_release(evdev);
	evdev_cdev_release_client(client);
	return (ret);
}

static int
evdev_ioctl_client(struct evdev_dev *evdev, struct evdev_client *client,
    u_long cmd, caddr_t data)
{
	struct input_keymap_entry *ke;
	int ret, len, limit, type_num;
	uint32_t code;
	size_t nvalues;

	/*
	 * Fix evdev state corrupted with discarding of kdb events.
	 * EVIOCGKEY and EVIOCGLED ioctls can suffer from this.
	 */
	if (evdev->ev_kdb_active) {
		EVDEV_LOCK(evdev);
		if (evdev->ev_kdb_active) {
			evdev->ev_kdb_active = false;
			evdev_restore_after_kdb(evdev);
		}
		EVDEV_UNLOCK(evdev);
	}

	/* file I/O ioctl handling */
	switch (cmd) {
	case FIOSETOWN:
	case TIOCSPGRP:
		return (0);

	case FIOGETOWN:
	case TIOCGPGRP:
		*(int *)data = 0;
		return (0);

	case FIONBIO:
		return (0);

	case FIOASYNC:
		if (*(int *)data)
			client->ec_async = true;
		else
			client->ec_async = false;

		return (0);

	case FIONREAD:
		EVDEV_CLIENT_LOCKQ(client);
		*(int *)data =
		    EVDEV_CLIENT_SIZEQ(client) * sizeof(struct input_event);
		EVDEV_CLIENT_UNLOCKQ(client);
		return (0);
	}

	len = IOCPARM_LEN(cmd);
	debugf(client, "ioctl called: cmd=0x%08lx, data=%p", cmd, data);

	/* evdev fixed-length ioctls handling */
	switch (cmd) {
	case EVIOCGVERSION:
		*(int *)data = EV_VERSION;
		return (0);

	case EVIOCGID:
		debugf(client, "EVIOCGID: bus=%d vendor=0x%04x product=0x%04x",
		    evdev->ev_id.bustype, evdev->ev_id.vendor,
		    evdev->ev_id.product);
		memcpy(data, &evdev->ev_id, sizeof(struct input_id));
		return (0);

	case EVIOCGREP:
		if (!evdev_event_supported(evdev, EV_REP))
			return (ENOTSUP);

		memcpy(data, evdev->ev_rep, sizeof(evdev->ev_rep));
		return (0);

	case EVIOCSREP:
		if (!evdev_event_supported(evdev, EV_REP))
			return (ENOTSUP);

		evdev_inject_event(evdev, EV_REP, REP_DELAY, ((int *)data)[0]);
		evdev_inject_event(evdev, EV_REP, REP_PERIOD,
		    ((int *)data)[1]);
		return (0);

	case EVIOCGKEYCODE:
		/* Fake unsupported ioctl */
		return (0);

	case EVIOCGKEYCODE_V2:
		if (evdev->ev_methods == NULL ||
		    evdev->ev_methods->ev_get_keycode == NULL)
			return (ENOTSUP);

		ke = (struct input_keymap_entry *)data;
		evdev->ev_methods->ev_get_keycode(evdev, ke);
		return (0);

	case EVIOCSKEYCODE:
		/* Fake unsupported ioctl */
		return (0);

	case EVIOCSKEYCODE_V2:
		if (evdev->ev_methods == NULL ||
		    evdev->ev_methods->ev_set_keycode == NULL)
			return (ENOTSUP);

		ke = (struct input_keymap_entry *)data;
		evdev->ev_methods->ev_set_keycode(evdev, ke);
		return (0);

	case EVIOCGABS(0) ... EVIOCGABS(ABS_MAX):
		if (evdev->ev_absinfo == NULL)
			return (EINVAL);

		memcpy(data, &evdev->ev_absinfo[cmd - EVIOCGABS(0)],
		    sizeof(struct input_absinfo));
		return (0);

	case EVIOCSABS(0) ... EVIOCSABS(ABS_MAX):
		if (evdev->ev_absinfo == NULL)
			return (EINVAL);

		code = cmd - EVIOCSABS(0);
		/* mt-slot number can not be changed */
		if (code == ABS_MT_SLOT)
			return (EINVAL);

		EVDEV_LOCK(evdev);
		evdev_set_absinfo(evdev, code, (struct input_absinfo *)data);
		EVDEV_UNLOCK(evdev);
		return (0);

	case EVIOCSFF:
	case EVIOCRMFF:
	case EVIOCGEFFECTS:
		/* Fake unsupported ioctls */
		return (0);

	case EVIOCGRAB:
		EVDEV_LOCK(evdev);
		if (*(int *)data)
			ret = evdev_grab_client(evdev, client);
		else
			ret = evdev_release_client(evdev, client);
		EVDEV_UNLOCK(evdev);
		return (ret);

	case EVIOCREVOKE:
		if (*(int *)data != 0)
			return (EINVAL);

		EVDEV_LIST_LOCK(evdev);
		if (!client->ec_revoked) {
			evdev_dispose_client(evdev, client);
			evdev_revoke_client(client);
			ret = 1;
		} else
			ret = 0;
		EVDEV_LIST_UNLOCK(evdev);
		if (ret)
			evdev_cdev_release_client(client);
		return (0);

	case EVIOCSCLOCKID:
		switch (*(int *)data) {
		case EVDEV_CLOCK_REALTIME:
			client->ec_clock_id = EV_CLOCK_REALTIME;
			return (0);
		case EVDEV_CLOCK_MONOTONIC:
		case EVDEV_CLOCK_MONOTONIC_LINUX:
			client->ec_clock_id = EV_CLOCK_MONOTONIC;
			return (0);
		default:
			return (EINVAL);
		}
	}

	/* evdev variable-length ioctls handling */
	switch (IOCBASECMD(cmd)) {
	case EVIOCGNAME(0):
		/* Linux evdev does not terminate truncated strings with 0 */
		limit = MIN((int)strlen(evdev->ev_name) + 1, len);
		memcpy(data, evdev->ev_name, limit);
		return (0);

	case EVIOCGPHYS(0):
		if (evdev->ev_shortname[0] == 0)
			return (ENOENT);

		limit = MIN((int)strlen(evdev->ev_shortname) + 1, len);
		memcpy(data, evdev->ev_shortname, limit);
		return (0);

	case EVIOCGUNIQ(0):
		if (evdev->ev_serial[0] == 0)
			return (ENOENT);

		limit = MIN((int)strlen(evdev->ev_serial) + 1, len);
		memcpy(data, evdev->ev_serial, limit);
		return (0);

	case EVIOCGPROP(0):
		limit = MIN(len, (int)bitstr_size(INPUT_PROP_CNT));
		memcpy(data, evdev->ev_prop_flags, limit);
		return (0);

	case EVIOCGMTSLOTS(0):
		/* EVIOCGMTSLOTS always returns 0 on success */
		if (evdev->ev_mt == NULL)
			return (EINVAL);
		if ((size_t)len < sizeof(uint32_t))
			return (EINVAL);
		code = *(uint32_t *)data;
		if (!ABS_IS_MT(code))
			return (EINVAL);

		nvalues =
		    MIN(len / sizeof(int32_t) - 1, MAXIMAL_MT_SLOT(evdev) + 1);
		for (int i = 0; i < (int)nvalues; i++)
			((int32_t *)data)[i + 1] =
			    evdev_mt_get_value(evdev, i, code);
		return (0);

	case EVIOCGKEY(0):
		limit = MIN(len, (int)bitstr_size(KEY_CNT));
		EVDEV_LOCK(evdev);
		evdev_client_filter_queue(client, EV_KEY);
		memcpy(data, evdev->ev_key_states, limit);
		EVDEV_UNLOCK(evdev);
		return (0);

	case EVIOCGLED(0):
		limit = MIN(len, (int)bitstr_size(LED_CNT));
		EVDEV_LOCK(evdev);
		evdev_client_filter_queue(client, EV_LED);
		memcpy(data, evdev->ev_led_states, limit);
		EVDEV_UNLOCK(evdev);
		return (0);

	case EVIOCGSND(0):
		limit = MIN(len, (int)bitstr_size(SND_CNT));
		EVDEV_LOCK(evdev);
		evdev_client_filter_queue(client, EV_SND);
		memcpy(data, evdev->ev_snd_states, limit);
		EVDEV_UNLOCK(evdev);
		return (0);

	case EVIOCGSW(0):
		limit = MIN(len, (int)bitstr_size(SW_CNT));
		EVDEV_LOCK(evdev);
		evdev_client_filter_queue(client, EV_SW);
		memcpy(data, evdev->ev_sw_states, limit);
		EVDEV_UNLOCK(evdev);
		return (0);

	case EVIOCGBIT(0, 0) ... EVIOCGBIT(EV_MAX, 0):
		type_num = IOCBASECMD(cmd) - EVIOCGBIT(0, 0);
		debugf(client, "EVIOCGBIT(%d): data=%p, len=%d", type_num,
		    data, len);
		return (evdev_ioctl_eviocgbit(evdev, type_num, len, data));
	}

	return (EINVAL);
}

static int
evdev_ioctl_eviocgbit(struct evdev_dev *evdev, int type, int len, caddr_t data)
{
	unsigned long *bitmap;
	int limit;

	switch (type) {
	case 0:
		bitmap = evdev->ev_type_flags;
		limit = EV_CNT;
		break;
	case EV_KEY:
		bitmap = evdev->ev_key_flags;
		limit = KEY_CNT;
		break;
	case EV_REL:
		bitmap = evdev->ev_rel_flags;
		limit = REL_CNT;
		break;
	case EV_ABS:
		bitmap = evdev->ev_abs_flags;
		limit = ABS_CNT;
		break;
	case EV_MSC:
		bitmap = evdev->ev_msc_flags;
		limit = MSC_CNT;
		break;
	case EV_LED:
		bitmap = evdev->ev_led_flags;
		limit = LED_CNT;
		break;
	case EV_SND:
		bitmap = evdev->ev_snd_flags;
		limit = SND_CNT;
		break;
	case EV_SW:
		bitmap = evdev->ev_sw_flags;
		limit = SW_CNT;
		break;
	case EV_FF:
		/*
		 * We don't support EV_FF now, so let's
		 * just fake it returning only zeros.
		 */
		bzero(data, len);
		return (0);
	default:
		return (ENOTTY);
	}

	/*
	 * Clear ioctl data buffer in case it's bigger than
	 * bitmap size
	 */
	bzero(data, len);

	limit = (int)bitstr_size(limit);
	len = MIN(limit, len);
	memcpy(data, bitmap, len);
	return (0);
}

void
evdev_revoke_client(struct evdev_client *client)
{

	EVDEV_LIST_LOCK_ASSERT(client->ec_evdev);

	client->ec_revoked = true;
}

void
evdev_notify_event(struct evdev_client *client)
{

	EVDEV_CLIENT_LOCKQ_ASSERT(client);

	if (client->ec_blocked) {
		client->ec_blocked = false;
		wakeup(client);
	}
	if (client->ec_selected) {
		client->ec_selected = false;
		selwakeup(&client->ec_selp);
	}
}

static void
evdev_make_node(struct evdev_dev *evdev)
{
	evdev->ev_cdev = devfs_make_node_clone(
	    makedev(evdev_major, EVDEV_MINOR(evdev->ev_unit, 0)), DEVFS_CHAR,
	    evdev->ev_cdev_uid, evdev->ev_cdev_gid, evdev->ev_cdev_mode,
	    evdev_clone, "input/event%d", evdev->ev_unit);
	if (evdev->ev_cdev == NULL)
		printf("evdev: cannot make /dev/input/event%d\n",
		    evdev->ev_unit);
}

int
evdev_cdev_create(struct evdev_dev *evdev)
{
	int unit;

	lck_mtx_lock(&evdev_nodes_mtx);
	lck_mtx_lock(&evdev_registry_mtx);
	for (unit = 0; unit < EVDEV_MAX_UNITS; unit++) {
		if (evdev_units[unit] == NULL)
			break;
	}
	if (unit == EVDEV_MAX_UNITS) {
		lck_mtx_unlock(&evdev_registry_mtx);
		lck_mtx_unlock(&evdev_nodes_mtx);
		return (ENOSPC);
	}
	evdev->ev_unit = unit;
	evdev->ev_cdev_active = 0;
	evdev->ev_registered = true;
	evdev_units[unit] = evdev;
	lck_mtx_unlock(&evdev_registry_mtx);

	evdev->ev_cdev = NULL;
	if (evdev_devfs_ready) {
		evdev->ev_devnum = makedev(evdev_major, EVDEV_MINOR(unit, 0));
		evdev_make_node(evdev);
	}
	lck_mtx_unlock(&evdev_nodes_mtx);

	return (0);
}

int
evdev_cdev_destroy(struct evdev_dev *evdev)
{

	lck_mtx_lock(&evdev_nodes_mtx);
	lck_mtx_lock(&evdev_registry_mtx);
	evdev_units[evdev->ev_unit] = NULL;
	evdev->ev_registered = false;
	while (evdev->ev_cdev_active > 0)
		msleep(&evdev->ev_cdev_active, &evdev_registry_mtx, PRIBIO,
		    "evdestroy", NULL);
	lck_mtx_unlock(&evdev_registry_mtx);
	if (evdev->ev_cdev != NULL)
		devfs_remove(evdev->ev_cdev);
	evdev->ev_cdev = NULL;
	lck_mtx_unlock(&evdev_nodes_mtx);
	return (0);
}

void
evdev_init(void)
{
	int unit;

	evdev_major = cdevsw_add(-1, &evdev_cdevsw);
	if (evdev_major < 0) {
		printf("evdev: no major number\n");
		return;
	}
	if (cdevsw_setkqueueok(evdev_major, &evdev_cdevsw, 0) == -1)
		printf("evdev: no kqueue\n");

	lck_mtx_lock(&evdev_nodes_mtx);
	evdev_devfs_ready = true;
	for (unit = 0; unit < EVDEV_MAX_UNITS; unit++) {
		struct evdev_dev *evdev;

		evdev = evdev_units[unit];
		if (evdev != NULL && evdev->ev_cdev == NULL) {
			evdev->ev_devnum = makedev(evdev_major,
			    EVDEV_MINOR(unit, 0));
			evdev_make_node(evdev);
		}
	}
	lck_mtx_unlock(&evdev_nodes_mtx);

	uinput_init();
}

static void
evdev_client_gettime(struct evdev_client *client, struct timeval *tv)
{

	switch (client->ec_clock_id) {
	case EV_CLOCK_BOOTTIME:
		/*
		 * XXX: FreeBSD does not support true POSIX monotonic clock.
		 *      So aliase EV_CLOCK_BOOTTIME to EV_CLOCK_MONOTONIC.
		 */
	case EV_CLOCK_MONOTONIC:
		microuptime(tv);
		break;

	case EV_CLOCK_REALTIME:
	default:
		microtime(tv);
		break;
	}
}

void
evdev_client_push(struct evdev_client *client, uint16_t type, uint16_t code,
    int32_t value)
{
	struct timeval time;
	size_t count, head, tail, ready;

	EVDEV_CLIENT_LOCKQ_ASSERT(client);
	head = client->ec_buffer_head;
	tail = client->ec_buffer_tail;
	ready = client->ec_buffer_ready;
	count = client->ec_buffer_size;

	/* If queue is full drop its content and place SYN_DROPPED event */
	if ((tail + 1) % count == head) {
		debugf(client, "client %p: buffer overflow", client);

		head = (tail + count - 1) % count;
		client->ec_buffer[head] = (struct input_event) {
			.type = EV_SYN,
			.code = SYN_DROPPED,
			.value = 0
		};
		/*
		 * XXX: Here is a small race window from now till the end of
		 *      report. The queue is empty but client has been already
		 *      notified of data readyness. Can be fixed in two ways:
		 * 1. Implement bulk insert so queue lock would not be dropped
		 *    till the SYN_REPORT event.
		 * 2. Insert SYN_REPORT just now and skip remaining events
		 */
		client->ec_buffer_head = head;
		client->ec_buffer_ready = head;
	}

	client->ec_buffer[tail].type = type;
	client->ec_buffer[tail].code = code;
	client->ec_buffer[tail].value = value;
	client->ec_buffer_tail = (tail + 1) % count;

	/* Allow users to read events only after report has been completed */
	if (type == EV_SYN && code == SYN_REPORT) {
		evdev_client_gettime(client, &time);
		for (; ready != client->ec_buffer_tail;
		    ready = (ready + 1) % count)
			client->ec_buffer[ready].time = time;
		client->ec_buffer_ready = client->ec_buffer_tail;
	}
}

void
evdev_client_dumpqueue(struct evdev_client *client)
{
	struct input_event *event;
	size_t i, head, tail, ready, size;

	head = client->ec_buffer_head;
	tail = client->ec_buffer_tail;
	ready = client->ec_buffer_ready;
	size = client->ec_buffer_size;

	printf("evdev client: %p\n", client);
	printf("event queue: head=%zu ready=%zu tail=%zu size=%zu\n",
	    head, ready, tail, size);

	printf("queue contents:\n");

	for (i = 0; i < size; i++) {
		event = &client->ec_buffer[i];
		printf("%zu: ", i);

		if (i < head || i > tail)
			printf("unused\n");
		else
			printf("type=%d code=%d value=%d ", event->type,
			    event->code, event->value);

		if (i == head)
			printf("<- head\n");
		else if (i == tail)
			printf("<- tail\n");
		else if (i == ready)
			printf("<- ready\n");
		else
			printf("\n");
	}
}

static void
evdev_client_filter_queue(struct evdev_client *client, uint16_t type)
{
	struct input_event *event;
	size_t head, tail, count, i;
	bool last_was_syn = false;

	EVDEV_CLIENT_LOCKQ(client);

	i = head = client->ec_buffer_head;
	tail = client->ec_buffer_tail;
	count = client->ec_buffer_size;
	client->ec_buffer_ready = client->ec_buffer_tail;

	while (i != client->ec_buffer_tail) {
		event = &client->ec_buffer[i];
		i = (i + 1) % count;

		/* Skip event of given type */
		if (event->type == type)
			continue;

		/* Remove empty SYN_REPORT events */
		if (event->type == EV_SYN && event->code == SYN_REPORT) {
			if (last_was_syn)
				continue;
			else
				client->ec_buffer_ready = (tail + 1) % count;
		}

		/* Rewrite entry */
		memcpy(&client->ec_buffer[tail], event,
		    sizeof(struct input_event));

		last_was_syn = (event->type == EV_SYN &&
		    event->code == SYN_REPORT);

		tail = (tail + 1) % count;
	}

	client->ec_buffer_head = i;
	client->ec_buffer_tail = tail;

	EVDEV_CLIENT_UNLOCKQ(client);
}
