/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <pie/mmap.h>: mapping a driver's memory into the process that calls
 * mmap(2) on one of its character devices, and the minor numbers of clone
 * devices, for kernel modules (com.blueberrypieos.kpi.platform,
 * config/PIE.exports). bsd/kern/kern_devmmap.c and
 * iokit/Drivers/rpi/PIEServices.cpp implement it. C linkage.
 *
 * Device mmap
 * -----------
 * cdevsw's d_mmap slot is dead in xnu (mmap(2) refuses every character
 * device with ENODEV). A driver that registers a function with
 * cdevsw_set_mmap gets the mmap calls on its devices instead:
 *
 *     fn(dev, map, &addr, size, prot, maxprot, flags, offset)
 *
 * - dev is the opened node's dev_t (for a clone device, the instance's).
 * - map is the calling process's map (current_map(); current_task() is its
 *   task). The function maps size bytes there, at *addr if flags has
 *   MAP_FIXED, else anywhere (*addr is then a hint), and sets *addr to the
 *   address it chose. pie_mmap_memory does that for an
 *   IOMemoryDescriptor.
 * - size is a whole number of pages, offset a multiple of the page size
 *   (mmap fails with EINVAL before calling fn otherwise); prot is a subset
 *   of maxprot, which the file's open mode allows (PROT_READ needs FREAD,
 *   PROT_WRITE with MAP_SHARED needs FWRITE; PROT_EXEC is refused).
 * - MAP_FIXED: whatever was mapped at [*addr, *addr + size) has already
 *   been removed.
 * - It returns 0 or an errno value, which mmap returns.
 *
 * A driver registers after cdevsw_add and before making its nodes;
 * cdevsw_remove forgets the function.
 *
 * Clone devices
 * -------------
 * devfs_make_node_clone(dev, DEVFS_CHAR, uid, gid, perms, clone, fmt, ...)
 * makes a node every open of which gets its own vnode and dev_t: devfs
 * calls clone(dev, DEVFS_CLONE_ALLOC) for the new minor (-1: none left,
 * open fails with ENOMEM). Note that devfs makes that vnode at lookup, so
 * stat(2) of the node calls clone too, without an open or a close to
 * follow: keep per-open state in d_open (keyed by the minor) and free it in
 * d_close, which is called once, when the last descriptor of that open
 * goes. clone is called with DEVFS_CLONE_FREE only if devfs fails to make
 * the vnode.
 *
 * Blueberry Pie OS's drivers number their minors
 *
 *     type << 20 | index << 16 | instance
 *
 * (a 24-bit minor: 4 bits of node type, e.g. DRM's primary and render
 * nodes; 4 bits of device index, e.g. card0, card1; 16 bits of open
 * instance), which user space (libdrm) masks to find the node it opened.
 */

#ifndef _PIE_MMAP_H_
#define _PIE_MMAP_H_

#include <sys/cdefs.h>
#include <sys/types.h>
#include <mach/mach_types.h>
#include <mach/vm_types.h>
#include <IOKit/IOTypes.h>

#ifdef __cplusplus
class IOMemoryDescriptor;
#else
typedef struct IOMemoryDescriptor IOMemoryDescriptor;
#endif

__BEGIN_DECLS

typedef int (*cdev_mmap_fn_t)(dev_t dev, vm_map_t map, mach_vm_address_t *addr,
    mach_vm_size_t size, int prot, int maxprot, int flags, off_t offset);

/*
 * Register fn for character major maj, which must be in use (cdevsw_add);
 * NULL unregisters. 0, or EINVAL.
 */
int cdevsw_set_mmap(int maj, cdev_mmap_fn_t fn);

/*
 * Map size bytes of md, from offset, into map (the current process's) at
 * *addr (MAP_FIXED in flags) or anywhere, read-only unless prot has
 * PROT_WRITE, with cache mode cache (kIOMapDefaultCache,
 * kIOMapWriteCombineCache, kIOMapInhibitCache...); set *addr. 0 or an
 * errno value.
 *
 * The mapping belongs to the process from then on: munmap and exit remove
 * it, never md's release. It keeps md's pages as long as it lasts when md
 * is pageable (IOBufferMemoryDescriptor with kIOMemoryPageable, wired with
 * prepare()); a wired-kernel-memory md must outlive every mapping of it.
 */
int pie_mmap_memory(IOMemoryDescriptor *md, mach_vm_offset_t offset, vm_map_t map,
    mach_vm_address_t *addr, mach_vm_size_t size, int prot, int flags, IOOptionBits cache);

/*
 * Map the physical pages under size bytes of md, from offset, into map
 * (the current process's) as pie_mmap_memory does, and call
 * unmapped(arg) once the process no longer maps any of them (munmap, exit
 * or exec, or a MAP_FIXED mapping over them). The pages are the caller's
 * to keep until then: md must be physical (withPhysicalAddress) or wired
 * (prepare()), and the caller keeps what owns the pages (its buffer
 * object) alive until unmapped runs. unmapped runs on a kernel thread
 * (thread_call), may block, and is not called when this fails. The
 * mapping keeps a list of md's pages and faults them in from it, so md
 * itself may be released at any time; the pages may be RAM the VM manages
 * (a wired buffer) or not (device memory, a DRAM hole). 0 or an errno
 * value.
 */
typedef void (*pie_unmap_fn_t)(void *arg);
int pie_mmap_physical(IOMemoryDescriptor *md, mach_vm_offset_t offset, vm_map_t map,
    mach_vm_address_t *addr, mach_vm_size_t size, int prot, int flags, IOOptionBits cache,
    pie_unmap_fn_t unmapped, void *arg);

/* Clone device minors (see above). */
#define PIE_CLONE_MINOR(type, index, instance) \
	((((type) & 0xf) << 20) | (((index) & 0xf) << 16) | ((instance) & 0xffff))
#define PIE_CLONE_TYPE(minor)         (((minor) >> 20) & 0xf)
#define PIE_CLONE_INDEX(minor)        (((minor) >> 16) & 0xf)
#define PIE_CLONE_INSTANCE(minor)     ((minor) & 0xffff)

__END_DECLS

#endif /* _PIE_MMAP_H_ */
