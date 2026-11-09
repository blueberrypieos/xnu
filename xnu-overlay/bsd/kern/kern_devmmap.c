/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#if CONFIG_RPI

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/file_internal.h>
#include <sys/mman.h>
#include <sys/vnode.h>
#include <mach/mach_types.h>
#include <mach/vm_prot.h>
#include <mach/vm_types_unsafe.h>
#include <vm/vm_kern_xnu.h>
#include <vm/vm_map_xnu.h>
#include <kern/locks.h>
#include <os/atomic_private.h>

#include "../../iokit/pie/mmap.h"

#define nitems(a) (sizeof(a) / sizeof((a)[0]))

void cdevsw_mmap_forget(int maj);
int cdev_mmap(struct fileproc *fp, dev_t dev, vm_map_t map, vm_map_offset_t *addr,
    vm_map_size_t size, int prot, int flags, off_t pos);

static cdev_mmap_fn_t cdevsw_mmap_fns[256];

int
cdevsw_set_mmap(int maj, cdev_mmap_fn_t fn)
{
	if (maj < 0 || maj >= nchrdev || maj >= (int)nitems(cdevsw_mmap_fns)) {
		return EINVAL;
	}
	if (fn != NULL && cdevsw_isfree(maj) == maj) {
		return EINVAL;
	}
	os_atomic_store(&cdevsw_mmap_fns[maj], fn, release);
	return 0;
}

void
cdevsw_mmap_forget(int maj)
{
	if (maj >= 0 && maj < (int)nitems(cdevsw_mmap_fns)) {
		os_atomic_store(&cdevsw_mmap_fns[maj], NULL, release);
	}
}

static int
devmmap_prepare(struct fileproc *fp, vm_map_t map, vm_map_offset_t addr,
    vm_map_size_t size, int prot, int flags, off_t pos, int *maxprotp)
{
	int fflag = fp->fp_glob->fg_flag;
	int maxprot = 0;

	if (size == 0 || pos < 0 || (pos & vm_map_page_mask(map)) != 0) {
		return EINVAL;
	}
	if (flags & (MAP_JIT | MAP_RESILIENT_CODESIGN | MAP_RESILIENT_MEDIA)) {
		return EINVAL;
	}
	if (prot & PROT_EXEC) {
		return EACCES;
	}
	if (fflag & FREAD) {
		maxprot |= VM_PROT_READ;
	} else if (prot & PROT_READ) {
		return EACCES;
	}
	if ((flags & MAP_SHARED) == 0 || (fflag & FWRITE)) {
		maxprot |= VM_PROT_WRITE;
	} else if (prot & PROT_WRITE) {
		return EACCES;
	}
	if (flags & MAP_FIXED) {
		if (addr & vm_map_page_mask(map)) {
			return EINVAL;
		}
		(void)mach_vm_deallocate_kernel(map, addr, size);
	}
	*maxprotp = maxprot;
	return 0;
}

int
cdev_mmap(struct fileproc *fp, dev_t dev, vm_map_t map, vm_map_offset_t *addr,
    vm_map_size_t size, int prot, int flags, off_t pos)
{
	int maj = major(dev);
	cdev_mmap_fn_t fn;
	mach_vm_address_t a = *addr;
	int maxprot, error;

	if (maj < 0 || maj >= (int)nitems(cdevsw_mmap_fns) ||
	    (fn = os_atomic_load(&cdevsw_mmap_fns[maj], acquire)) == NULL) {
		return ENODEV;
	}
	error = devmmap_prepare(fp, map, a, size, prot, flags, pos, &maxprot);
	if (error) {
		return error;
	}
	error = fn(dev, map, &a, size, prot, maxprot, flags, pos);
	if (error == 0) {
		*addr = a;
	}
	return error;
}

#endif
