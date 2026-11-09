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
#include <sys/ioccom.h>
#include <sys/proc.h>
#include <sys/select.h>
#include <sys/uio.h>
#include <sys/vnode.h>
#include <miscfs/devfs/devfs.h>

#include "evdev_compat.h"
#include "evdev.h"
#include "evdev_private.h"
#include "input.h"
#include "uinput.h"

#ifdef UINPUT_DEBUG
#define	debugf(state, fmt, args...)	printf("uinput: " fmt "\n", ##args)
#else
#define	debugf(state, fmt, args...)
#endif

#define	UINPUT_BUFFER_SIZE	16

#define	UINPUT_LOCK(state)		sx_xlock(&(state)->ucs_lock)
#define	UINPUT_UNLOCK(state)		sx_unlock(&(state)->ucs_lock)
#define	UINPUT_LOCK_ASSERT(state)	sx_assert(&(state)->ucs_lock, SA_LOCKED)
#define UINPUT_EMPTYQ(state) \
    ((state)->ucs_buffer_head == (state)->ucs_buffer_tail)

enum uinput_state
{
	UINPUT_NEW = 0,
	UINPUT_CONFIGURED,
	UINPUT_RUNNING
};

static evdev_event_t	uinput_ev_event;

static open_close_fcn_t	uinput_open;
static open_close_fcn_t	uinput_close;
static read_write_fcn_t	uinput_read;
static read_write_fcn_t	uinput_write;
static ioctl_fcn_t	uinput_ioctl;
static select_fcn_t	uinput_select;

static const struct cdevsw uinput_cdevsw = {
	.d_open = uinput_open,
	.d_close = uinput_close,
	.d_read = uinput_read,
	.d_write = uinput_write,
	.d_ioctl = uinput_ioctl,
	.d_stop = eno_stop,
	.d_reset = eno_reset,
	.d_ttys = NULL,
	.d_select = uinput_select,
	.d_mmap = eno_mmap,
	.d_strategy = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type = 0,
};

static void *uinput_cdev;
static int uinput_major = -1;

static const struct evdev_methods uinput_ev_methods = {
	.ev_open = NULL,
	.ev_close = NULL,
	.ev_event = uinput_ev_event,
};

struct uinput_cdev_state
{
	enum uinput_state	ucs_state;
	struct evdev_dev *	ucs_evdev;
	struct sx		ucs_lock;
	size_t			ucs_buffer_head;
	size_t			ucs_buffer_tail;
	struct selinfo		ucs_selp;
	bool			ucs_blocked;
	bool			ucs_selected;
	dev_t			ucs_dev;
	uint32_t		ucs_refs;
	LIST_ENTRY(uinput_cdev_state) ucs_link;
	struct input_event      ucs_buffer[UINPUT_BUFFER_SIZE];
};

static void uinput_enqueue_event(struct uinput_cdev_state *, uint16_t,
    uint16_t, int32_t);
static int uinput_setup_provider(struct uinput_cdev_state *,
    struct uinput_user_dev *);
static int uinput_cdev_create(void);
static void uinput_notify(struct uinput_cdev_state *);

static LCK_MTX_DECLARE(uinput_mtx, &evdev_lck_grp);
static LIST_HEAD(, uinput_cdev_state) uinput_states =
    LIST_HEAD_INITIALIZER(uinput_states);
static uint32_t uinput_next_instance;

static struct uinput_cdev_state *
uinput_find(dev_t dev)
{
	struct uinput_cdev_state *state;

	LIST_FOREACH(state, &uinput_states, ucs_link) {
		if (state->ucs_dev == dev)
			return (state);
	}
	return (NULL);
}

static struct uinput_cdev_state *
uinput_get(dev_t dev)
{
	struct uinput_cdev_state *state;

	lck_mtx_lock(&uinput_mtx);
	state = uinput_find(dev);
	if (state != NULL)
		state->ucs_refs++;
	lck_mtx_unlock(&uinput_mtx);
	return (state);
}

static void uinput_dtor(void *);

static void
uinput_put(struct uinput_cdev_state *state)
{
	uint32_t refs;

	lck_mtx_lock(&uinput_mtx);
	refs = --state->ucs_refs;
	lck_mtx_unlock(&uinput_mtx);
	if (refs == 0)
		uinput_dtor(state);
}

static int
uinput_clone(dev_t dev, int action)
{
	uint32_t inst;
	int tries = 0;

	if (action != DEVFS_CLONE_ALLOC)
		return (0);
	lck_mtx_lock(&uinput_mtx);
	do {
		inst = ++uinput_next_instance & 0xffff;
		if (++tries > 0x10000) {
			lck_mtx_unlock(&uinput_mtx);
			return (-1);
		}
	} while (inst == 0 ||
	    uinput_find(makedev(major(dev), inst)) != NULL);
	lck_mtx_unlock(&uinput_mtx);
	return ((int)inst);
}

static void
uinput_ev_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{
	struct uinput_cdev_state *state = evdev_get_softc(evdev);

	if (type == EV_LED)
		evdev_push_event(evdev, type, code, value);

	UINPUT_LOCK(state);
	if (state->ucs_state == UINPUT_RUNNING) {
		uinput_enqueue_event(state, type, code, value);
		uinput_notify(state);
	}
	UINPUT_UNLOCK(state);
}

static void
uinput_enqueue_event(struct uinput_cdev_state *state, uint16_t type,
    uint16_t code, int32_t value)
{
	size_t head, tail;

	UINPUT_LOCK_ASSERT(state);

	head = state->ucs_buffer_head;
	tail = (state->ucs_buffer_tail + 1) % UINPUT_BUFFER_SIZE;

	microtime(&state->ucs_buffer[tail].time);
	state->ucs_buffer[tail].type = type;
	state->ucs_buffer[tail].code = code;
	state->ucs_buffer[tail].value = value;
	state->ucs_buffer_tail = tail;

	/* If queue is full remove oldest event */
	if (tail == head) {
		debugf(state, "state %p: buffer overflow", state);

		head = (head + 1) % UINPUT_BUFFER_SIZE;
		state->ucs_buffer_head = head;
	}
}

static int
uinput_open(dev_t dev, int oflags, int devtype, proc_t p)
{
	struct uinput_cdev_state *state;

	(void)oflags;
	(void)devtype;
	(void)p;
	if (minor(dev) == 0)
		return (ENXIO);
	lck_mtx_lock(&uinput_mtx);
	state = uinput_find(dev);
	lck_mtx_unlock(&uinput_mtx);
	if (state != NULL)
		return (0);

	state = evdev_malloc(sizeof(struct uinput_cdev_state), M_EVDEV,
	    M_WAITOK | M_ZERO);
	state->ucs_evdev = evdev_alloc();

	sx_init(&state->ucs_lock, "uinput");
	state->ucs_dev = dev;
	state->ucs_refs = 1;

	lck_mtx_lock(&uinput_mtx);
	if (uinput_find(dev) != NULL) {
		lck_mtx_unlock(&uinput_mtx);
		uinput_dtor(state);
		return (0);
	}
	LIST_INSERT_HEAD(&uinput_states, state, ucs_link);
	lck_mtx_unlock(&uinput_mtx);
	return (0);
}

static int
uinput_close(dev_t dev, int flags, int devtype, proc_t p)
{
	struct uinput_cdev_state *state;

	(void)flags;
	(void)devtype;
	(void)p;
	lck_mtx_lock(&uinput_mtx);
	state = uinput_find(dev);
	if (state != NULL)
		LIST_REMOVE(state, ucs_link);
	lck_mtx_unlock(&uinput_mtx);
	if (state != NULL)
		uinput_put(state);
	return (0);
}

static void
uinput_dtor(void *data)
{
	struct uinput_cdev_state *state = (struct uinput_cdev_state *)data;

	evdev_free(state->ucs_evdev);

	selthreadclear(&state->ucs_selp);
	sx_destroy(&state->ucs_lock);
	evdev_mfree(data, M_EVDEV);
}

static int uinput_read_state(struct uinput_cdev_state *, struct uio *, int);

static int
uinput_read(dev_t dev, struct uio *uio, int ioflag)
{
	struct uinput_cdev_state *state;
	int ret;

	state = uinput_get(dev);
	if (state == NULL)
		return (ENXIO);
	ret = uinput_read_state(state, uio, ioflag);
	uinput_put(state);
	return (ret);
}

static int
uinput_read_state(struct uinput_cdev_state *state, struct uio *uio,
    int ioflag)
{
	struct input_event *event;
	int remaining, ret = 0;

	/* Zero-sized reads are allowed for error checking */
	if (uio_resid(uio) != 0 &&
	    (size_t)uio_resid(uio) < sizeof(struct input_event))
		return (EINVAL);

	remaining = (int)(uio_resid(uio) / sizeof(struct input_event));

	UINPUT_LOCK(state);

	if (state->ucs_state != UINPUT_RUNNING)
		ret = EINVAL;

	if (ret == 0 && UINPUT_EMPTYQ(state)) {
		if (ioflag & IO_NDELAY)
			ret = EWOULDBLOCK;
		else {
			if (remaining != 0) {
				state->ucs_blocked = true;
				ret = sx_sleep(state, &state->ucs_lock,
				    PCATCH, "uiread", 0);
			}
		}
	}

	while (ret == 0 && !UINPUT_EMPTYQ(state) && remaining > 0) {
		event = &state->ucs_buffer[state->ucs_buffer_head];
		state->ucs_buffer_head = (state->ucs_buffer_head + 1) %
		    UINPUT_BUFFER_SIZE;
		remaining--;
		ret = uiomove((char *)event, sizeof(struct input_event), uio);
	}

	UINPUT_UNLOCK(state);

	return (ret);
}

static int
uinput_write(dev_t dev, struct uio *uio, int ioflag)
{
	struct uinput_cdev_state *state;
	struct uinput_user_dev *userdev;
	struct input_event event;
	int ret = 0;

	(void)ioflag;
	state = uinput_get(dev);
	if (state == NULL)
		return (ENXIO);

	UINPUT_LOCK(state);

	if (state->ucs_state != UINPUT_RUNNING) {
		/* Process written struct uinput_user_dev */
		if (uio_resid(uio) != sizeof(struct uinput_user_dev)) {
			debugf(state, "write size not multiple of "
			    "struct uinput_user_dev size");
			ret = EINVAL;
		} else {
			userdev = evdev_malloc(sizeof(*userdev), M_EVDEV,
			    M_WAITOK);
			ret = uiomove((char *)userdev,
			    sizeof(struct uinput_user_dev), uio);
			if (ret == 0)
				uinput_setup_provider(state, userdev);
			evdev_mfree(userdev, M_EVDEV);
		}
	} else {
		/* Process written event */
		if (uio_resid(uio) % sizeof(struct input_event) != 0) {
			debugf(state, "write size not multiple of "
			    "struct input_event size");
			ret = EINVAL;
		}

		while (ret == 0 && uio_resid(uio) > 0) {
			ret = uiomove((char *)&event,
			    sizeof(struct input_event), uio);
			if (ret == 0)
				ret = evdev_push_event(state->ucs_evdev,
				    event.type, event.code, event.value);
		}
	}

	UINPUT_UNLOCK(state);
	uinput_put(state);

	return (ret);
}

static int
uinput_setup_dev(struct uinput_cdev_state *state, struct input_id *id,
    char *name, uint32_t ff_effects_max)
{

	if (name[0] == 0)
		return (EINVAL);

	evdev_set_name(state->ucs_evdev, name);
	evdev_set_id(state->ucs_evdev, id->bustype, id->vendor, id->product,
	    id->version);
	state->ucs_state = UINPUT_CONFIGURED;

	return (0);
}

static int
uinput_setup_provider(struct uinput_cdev_state *state,
    struct uinput_user_dev *udev)
{
	struct input_absinfo absinfo;
	int i, ret;

	debugf(state, "setup_provider called, udev=%p", udev);

	ret = uinput_setup_dev(state, &udev->id, udev->name,
	    udev->ff_effects_max);
	if (ret)
		return (ret);

	bzero(&absinfo, sizeof(struct input_absinfo));
	for (i = 0; i < ABS_CNT; i++) {
		if (!bit_test(state->ucs_evdev->ev_abs_flags, i))
			continue;

		absinfo.minimum = udev->absmin[i];
		absinfo.maximum = udev->absmax[i];
		absinfo.fuzz = udev->absfuzz[i];
		absinfo.flat = udev->absflat[i];
		evdev_set_absinfo(state->ucs_evdev, i, &absinfo);
	}

	return (0);
}

static int
uinput_select(dev_t dev, int which, void *wql, proc_t p)
{
	struct uinput_cdev_state *state;
	int revents = 0;

	state = uinput_get(dev);
	if (state == NULL)
		return (1);

	/* Always allow write */
	if (which == FWRITE)
		revents = 1;

	if (which == FREAD) {
		UINPUT_LOCK(state);
		if (!UINPUT_EMPTYQ(state))
			revents = 1;
		else {
			state->ucs_selected = true;
			selrecord(p, &state->ucs_selp, wql);
		}
		UINPUT_UNLOCK(state);
	}

	uinput_put(state);
	return (revents);
}

static void
uinput_notify(struct uinput_cdev_state *state)
{

	UINPUT_LOCK_ASSERT(state);

	if (state->ucs_blocked) {
		state->ucs_blocked = false;
		wakeup(state);
	}
	if (state->ucs_selected) {
		state->ucs_selected = false;
		selwakeup(&state->ucs_selp);
	}
}

static int
uinput_ioctl_sub(struct uinput_cdev_state *state, u_long cmd, caddr_t data)
{
	struct uinput_setup *us;
	struct uinput_abs_setup *uabs;
	int ret, len, intdata = 0;
	char buf[NAMELEN];
	size_t done;

	UINPUT_LOCK_ASSERT(state);

	len = IOCPARM_LEN(cmd);
	if ((cmd & IOC_DIRMASK) == IOC_VOID && len == sizeof(int))
		intdata = *(int *)data;

	switch (IOCBASECMD(cmd)) {
	case UI_GET_SYSNAME(0):
		if (state->ucs_state != UINPUT_RUNNING)
			return (ENOENT);
		if (len == 0)
			return (EINVAL);
		snprintf(data, (size_t)len, "event%d", state->ucs_evdev->ev_unit);
		return (0);
	}

	switch (cmd) {
	case UI_DEV_CREATE:
		if (state->ucs_state != UINPUT_CONFIGURED)
			return (EINVAL);

		evdev_set_methods(state->ucs_evdev, state, &uinput_ev_methods);
		evdev_set_flag(state->ucs_evdev, EVDEV_FLAG_SOFTREPEAT);
		evdev_set_flag(state->ucs_evdev, EVDEV_FLAG_MT_KEEPID);
		ret = evdev_register(state->ucs_evdev);
		if (ret == 0)
			state->ucs_state = UINPUT_RUNNING;
		return (ret);

	case UI_DEV_DESTROY:
		if (state->ucs_state != UINPUT_RUNNING)
			return (0);

		evdev_unregister(state->ucs_evdev);
		bzero(state->ucs_evdev, sizeof(struct evdev_dev));
		state->ucs_state = UINPUT_NEW;
		return (0);

	case UI_DEV_SETUP:
		if (state->ucs_state == UINPUT_RUNNING)
			return (EINVAL);

		us = (struct uinput_setup *)data;
		return (uinput_setup_dev(state, &us->id, us->name,
		    us->ff_effects_max));

	case UI_ABS_SETUP:
		if (state->ucs_state == UINPUT_RUNNING)
			return (EINVAL);

		uabs = (struct uinput_abs_setup *)data;
		if (uabs->code > ABS_MAX)
			return (EINVAL);

		evdev_set_abs_bit(state->ucs_evdev, uabs->code);
		evdev_set_absinfo(state->ucs_evdev, uabs->code,
		    &uabs->absinfo);
		return (0);

	case UI_SET_EVBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > EV_MAX || intdata < 0)
			return (EINVAL);
		evdev_support_event(state->ucs_evdev, intdata);
		return (0);

	case UI_SET_KEYBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > KEY_MAX || intdata < 0)
			return (EINVAL);
		evdev_support_key(state->ucs_evdev, intdata);
		return (0);

	case UI_SET_RELBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > REL_MAX || intdata < 0)
			return (EINVAL);
		evdev_support_rel(state->ucs_evdev, intdata);
		return (0);

	case UI_SET_ABSBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > ABS_MAX || intdata < 0)
			return (EINVAL);
		evdev_set_abs_bit(state->ucs_evdev, intdata);
		return (0);

	case UI_SET_MSCBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > MSC_MAX || intdata < 0)
			return (EINVAL);
		evdev_support_msc(state->ucs_evdev, intdata);
		return (0);

	case UI_SET_LEDBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > LED_MAX || intdata < 0)
			return (EINVAL);
		evdev_support_led(state->ucs_evdev, intdata);
		return (0);

	case UI_SET_SNDBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > SND_MAX || intdata < 0)
			return (EINVAL);
		evdev_support_snd(state->ucs_evdev, intdata);
		return (0);

	case UI_SET_FFBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > FF_MAX || intdata < 0)
			return (EINVAL);
		/* Fake unsupported ioctl */
		return (0);

	case UI_SET_PHYS:
		if (state->ucs_state == UINPUT_RUNNING)
			return (EINVAL);
		ret = copyinstr(*(user_addr_t *)data, buf, sizeof(buf), &done);
		/* Linux returns EINVAL when string does not fit the buffer */
		if (ret == ENAMETOOLONG)
			ret = EINVAL;
		if (ret != 0)
			return (ret);
		evdev_set_phys(state->ucs_evdev, buf);
		return (0);

	case UI_SET_BSDUNIQ:
		if (state->ucs_state == UINPUT_RUNNING)
			return (EINVAL);
		ret = copyinstr(*(user_addr_t *)data, buf, sizeof(buf), &done);
		if (ret != 0)
			return (ret);
		evdev_set_serial(state->ucs_evdev, buf);
		return (0);

	case UI_SET_SWBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > SW_MAX || intdata < 0)
			return (EINVAL);
		evdev_support_sw(state->ucs_evdev, intdata);
		return (0);

	case UI_SET_PROPBIT:
		if (state->ucs_state == UINPUT_RUNNING ||
		    intdata > INPUT_PROP_MAX || intdata < 0)
			return (EINVAL);
		evdev_support_prop(state->ucs_evdev, intdata);
		return (0);

	case UI_BEGIN_FF_UPLOAD:
	case UI_END_FF_UPLOAD:
	case UI_BEGIN_FF_ERASE:
	case UI_END_FF_ERASE:
		if (state->ucs_state == UINPUT_RUNNING)
			return (EINVAL);
		/* Fake unsupported ioctl */
		return (0);

	case UI_GET_VERSION:
		*(unsigned int *)data = UINPUT_VERSION;
		return (0);
	}

	return (EINVAL);
}

static int
uinput_ioctl(dev_t dev, u_long cmd, caddr_t data, int fflag, proc_t p)
{
	struct uinput_cdev_state *state;
	int ret;

	(void)fflag;
	(void)p;
	if (cmd == FIONBIO || cmd == FIOASYNC)
		return (0);

	state = uinput_get(dev);
	if (state == NULL)
		return (ENXIO);

	debugf(state, "ioctl called: cmd=0x%08lx, data=%p", cmd, data);

	UINPUT_LOCK(state);
	ret = uinput_ioctl_sub(state, cmd, data);
	UINPUT_UNLOCK(state);

	uinput_put(state);
	return (ret);
}

static int
uinput_cdev_create(void)
{

	uinput_major = cdevsw_add(-1, &uinput_cdevsw);
	if (uinput_major < 0)
		return (ENXIO);
	if (cdevsw_setkqueueok(uinput_major, &uinput_cdevsw, 0) == -1)
		printf("uinput: no kqueue\n");
	uinput_cdev = devfs_make_node_clone(makedev(uinput_major, 0),
	    DEVFS_CHAR, UID_ROOT, EVDEV_GID_INPUT, 0660, uinput_clone,
	    "uinput");
	return (uinput_cdev != NULL ? 0 : ENXIO);
}

void
uinput_init(void)
{
	if (uinput_cdev_create() != 0)
		printf("uinput: cannot make /dev/uinput\n");
}
