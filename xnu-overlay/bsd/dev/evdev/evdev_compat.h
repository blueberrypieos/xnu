/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _DEV_EVDEV_EVDEV_COMPAT_H
#define _DEV_EVDEV_EVDEV_COMPAT_H

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/types.h>
#include <sys/errno.h>
#include <sys/queue.h>
#include <sys/select.h>
#include <sys/proc.h>
#include <sys/time.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/thread_call.h>
#include <kern/clock.h>
#include <libkern/libkern.h>
#include <stdbool.h>

#ifndef nitems
#define nitems(x)       (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef KASSERT
#define KASSERT(exp, msg) do {                                          \
	if (__improbable(!(exp)))                                       \
	        panic msg;                                              \
} while (0)
#endif

extern lck_grp_t evdev_lck_grp;

#include <sys/malloc.h>
#undef MALLOC_DECLARE
#undef MALLOC_DEFINE
#define MALLOC_DECLARE(type)
#define MALLOC_DEFINE(type, shortdesc, longdesc)

#ifndef abs
#define abs(x)          __builtin_abs(x)
#endif

static inline void *
evdev_compat_malloc(size_t size, int flags)
{
	size_t *p = (size_t *)kalloc_data(size + 16,
	    ((flags & M_NOWAIT) ? Z_NOWAIT : Z_WAITOK) | ((flags & M_ZERO) ? Z_ZERO : 0));
	if (p == NULL) {
		return NULL;
	}
	p[0] = size + 16;
	return (char *)p + 16;
}

static inline void
evdev_compat_free(void *ptr)
{
	if (ptr != NULL) {
		size_t *p = (size_t *)(void *)((char *)ptr - 16);
		kfree_data(p, p[0]);
	}
}

#define evdev_malloc(size, type, flags) evdev_compat_malloc((size), (flags))
#define evdev_mfree(ptr, type)          evdev_compat_free(ptr)

struct mtx {
	lck_mtx_t       m;
};
#define MTX_DEF         0
#define MA_OWNED        LCK_MTX_ASSERT_OWNED
#define mtx_init(mx, name, type, opts)  lck_mtx_init(&(mx)->m, &evdev_lck_grp, LCK_ATTR_NULL)
#define mtx_destroy(mx)                 lck_mtx_destroy(&(mx)->m, &evdev_lck_grp)
#define mtx_lock(mx)                    lck_mtx_lock(&(mx)->m)
#define mtx_unlock(mx)                  lck_mtx_unlock(&(mx)->m)
#define mtx_assert(mx, what)            LCK_MTX_ASSERT(&(mx)->m, (what))
#define mtx_sleep(chan, mx, pri, wmesg, timo) \
	msleep((chan), &(mx)->m, (pri), (wmesg), NULL)

#define sx              mtx
#define SA_LOCKED       MA_OWNED
#define SA_XLOCKED      MA_OWNED
#define sx_init(sx, name)               mtx_init((sx), (name), NULL, 0)
#define sx_destroy(sx)                  mtx_destroy(sx)
#define sx_xlock(sx)                    mtx_lock(sx)
#define sx_xlock_sig(sx)                (mtx_lock(sx), 0)
#define sx_unlock(sx)                   mtx_unlock(sx)
#define sx_assert(sx, what)             mtx_assert((sx), (what))
#define sx_sleep(chan, sx, pri, wmesg, timo) mtx_sleep((chan), (sx), (pri), (wmesg), (timo))

struct callout {
	thread_call_t   c_call;
	lck_mtx_t       *c_mtx;
	void            (*c_fn)(void *);
	void            *c_arg;
	uintptr_t       c_gen;
	bool            c_pending;
};

void callout_init_mtx(struct callout *c, struct mtx *mx, int flags);
void callout_reset_ms(struct callout *c, int ms, void (*fn)(void *), void *arg);
void callout_stop(struct callout *c);
void callout_drain(struct callout *c);

typedef unsigned long bitstr_t;
#define _BITSTR_BITS            (sizeof(bitstr_t) * 8)
#define _bit_idx(bit)           ((bit) / _BITSTR_BITS)
#define _bit_mask(bit)          (1UL << ((bit) % _BITSTR_BITS))
#define bitstr_size(nbits)      (howmany((nbits), _BITSTR_BITS) * sizeof(bitstr_t))
#define bit_decl(name, nbits)   name[howmany((nbits), _BITSTR_BITS)]

static inline int
bit_test(const bitstr_t *b, int bit)
{
	return (b[_bit_idx(bit)] & _bit_mask(bit)) != 0;
}

static inline void
bit_set(bitstr_t *b, int bit)
{
	b[_bit_idx(bit)] |= _bit_mask(bit);
}

static inline void
bit_clear(bitstr_t *b, int bit)
{
	b[_bit_idx(bit)] &= ~_bit_mask(bit);
}

static inline void
bit_nclear(bitstr_t *b, int start, int stop)
{
	for (int i = start; i <= stop; i++) {
		bit_clear(b, i);
	}
}

static inline void
bit_ffs_at(const bitstr_t *b, int start, int nbits, int *result)
{
	for (int i = start; i < nbits; i++) {
		if (bit_test(b, i)) {
			*result = i;
			return;
		}
	}
	*result = -1;
}

#define bit_ffs(b, nbits, result)       bit_ffs_at((b), 0, (nbits), (result))

static inline void
bit_count(const bitstr_t *b, int start, int nbits, int *result)
{
	int n = 0;
	for (int i = start; i < start + nbits; i++) {
		n += bit_test(b, i);
	}
	*result = n;
}

static inline int
bit_ffs_next(const bitstr_t *b, int start, int nbits)
{
	int r;
	bit_ffs_at(b, start, nbits, &r);
	return r;
}

#define bit_foreach_at(b, start, nbits, var)                            \
	for ((var) = bit_ffs_next((b), (start), (nbits)); (var) >= 0;   \
	    (var) = bit_ffs_next((b), (var) + 1, (nbits)))
#define bit_foreach(b, nbits, var)      bit_foreach_at((b), 0, (nbits), (var))

#define CK_SLIST_HEAD                   SLIST_HEAD
#define CK_SLIST_ENTRY                  SLIST_ENTRY
#define CK_SLIST_INIT                   SLIST_INIT
#define CK_SLIST_EMPTY                  SLIST_EMPTY
#define CK_SLIST_FOREACH                SLIST_FOREACH
#define CK_SLIST_FOREACH_SAFE           SLIST_FOREACH_SAFE
#define CK_SLIST_INSERT_HEAD            SLIST_INSERT_HEAD
#define CK_SLIST_REMOVE                 SLIST_REMOVE

#ifndef MPASS
#define MPASS(exp)              KASSERT((exp), ("Assertion %s failed", #exp))
#endif
#define bitcount(x)             __builtin_popcount(x)

#define kdb_active              0
#define SCHEDULER_STOPPED()     0

#endif
