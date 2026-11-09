/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <sys/errno.h>
#include <sys/mman.h>
#include <os/atomic_private.h>

extern "C" {
#include <kern/task.h>
#include <kern/thread_call.h>
#include <kern/locks.h>
#include <vm/vm_map_xnu.h>
#include <vm/vm_iokit.h>
}

#include "../../Kernel/IOKitKernelInternal.h"

#include "../../pie/mmap.h"

int
pie_mmap_memory(IOMemoryDescriptor *md, mach_vm_offset_t offset, vm_map_t map,
    mach_vm_address_t *addr, mach_vm_size_t size, int prot, int flags, IOOptionBits cache)
{
	task_t task = current_task();

	if (md == NULL || addr == NULL || size == 0 || offset > md->getLength() ||
	    size > md->getLength() - offset) {
		return EINVAL;
	}
	if (map != get_task_map(task)) {
		return EINVAL;
	}
	IOOptionBits options = (cache & kIOMapCacheMask) | ((flags & MAP_FIXED) ? 0 : kIOMapAnywhere);
	if ((prot & PROT_WRITE) == 0) {
		options |= kIOMapReadOnly;
	}
	IOMemoryMap *mapping = md->createMappingInTask(task,
	    (flags & MAP_FIXED) ? *addr : 0, options, offset, size);
	if (mapping == NULL) {
		return ENOMEM;
	}
	*addr = mapping->getAddress();
	mapping->taskDied();
	mapping->release();
	return 0;
}

struct pie_unmap_notify {
	struct pie_unmap_notify *next;
	pie_unmap_fn_t fn;
	void *arg;
	IOAddressRange *ranges;
	unsigned int count;
};

static LCK_GRP_DECLARE(pie_unmap_grp, "pie_unmap");
static LCK_SPIN_DECLARE(pie_unmap_lock, &pie_unmap_grp);
static struct pie_unmap_notify *pie_unmap_posted;
static thread_call_t pie_unmap_call;

static void
pie_unmap_notify_free(struct pie_unmap_notify *n)
{
	if (n->ranges) {
		IODeleteData(n->ranges, IOAddressRange, n->count);
	}
	IOFreeType(n, struct pie_unmap_notify);
}

extern "C" kern_return_t
pie_unmap_notify_fault(struct pie_unmap_notify *n, memory_object_t pager,
    vm_object_offset_t offset, vm_size_t size)
{
	if (n->count == 1) {
		return device_pager_populate_object(pager, 0, (ppnum_t)atop_64(n->ranges[0].address),
		           (vm_size_t)round_page_64(n->ranges[0].length));
	}
	for (vm_object_offset_t off = trunc_page_64(offset); off < offset + size; off += PAGE_SIZE) {
		vm_object_offset_t base = 0;
		unsigned int i = 0;
		for (; i < n->count && off >= base + n->ranges[i].length; i++) {
			base += n->ranges[i].length;
		}
		if (i == n->count) {
			return KERN_FAILURE;
		}
		kern_return_t kr = device_pager_populate_object(pager, off,
		    (ppnum_t)atop_64(n->ranges[i].address + (off - base)), PAGE_SIZE);
		if (kr != KERN_SUCCESS) {
			return kr;
		}
	}
	return KERN_SUCCESS;
}

static void
pie_unmap_deliver(thread_call_param_t p0 __unused, thread_call_param_t p1 __unused)
{
	for (;;) {
		lck_spin_lock(&pie_unmap_lock);
		struct pie_unmap_notify *n = pie_unmap_posted;
		pie_unmap_posted = NULL;
		lck_spin_unlock(&pie_unmap_lock);
		if (n == NULL) {
			return;
		}
		while (n != NULL) {
			struct pie_unmap_notify *next = n->next;
			n->fn(n->arg);
			pie_unmap_notify_free(n);
			n = next;
		}
	}
}

extern "C" void
pie_unmap_notify_post(struct pie_unmap_notify *n)
{
	lck_spin_lock(&pie_unmap_lock);
	n->next = pie_unmap_posted;
	pie_unmap_posted = n;
	lck_spin_unlock(&pie_unmap_lock);
	thread_call_enter(pie_unmap_call);
}

int
pie_mmap_physical(IOMemoryDescriptor *md, mach_vm_offset_t offset, vm_map_t map,
    mach_vm_address_t *addr, mach_vm_size_t size, int prot, int flags, IOOptionBits cache,
    pie_unmap_fn_t unmapped, void *arg)
{
	task_t task = current_task();

	if (md == NULL || addr == NULL || unmapped == NULL || size == 0 ||
	    (offset & PAGE_MASK) || (size & PAGE_MASK) || offset > md->getLength() ||
	    size > md->getLength() - offset) {
		return EINVAL;
	}
	if (map != get_task_map(task)) {
		return EINVAL;
	}
	if (pie_unmap_call == NULL) {
		thread_call_t call = thread_call_allocate_with_priority(pie_unmap_deliver, NULL,
		    THREAD_CALL_PRIORITY_KERNEL);
		if (call == NULL) {
			return ENOMEM;
		}
		if (!os_atomic_cmpxchg(&pie_unmap_call, NULL, call, release)) {
			thread_call_free(call);
		}
	}

	unsigned int count = 0, max = 0;
	IOAddressRange *ranges = NULL;
	for (mach_vm_size_t done = 0; done < size;) {
		IOByteCount len = 0;
		addr64_t pa = md->getPhysicalSegment(offset + done, &len, kIOMemoryMapperNone);
		if (pa == 0 || len == 0 || (pa & PAGE_MASK)) {
			if (ranges) {
				IODeleteData(ranges, IOAddressRange, max);
			}
			return EINVAL;
		}
		if (len > size - done) {
			len = (IOByteCount)(size - done);
		}
		if (count > 0 && ranges[count - 1].address + ranges[count - 1].length == pa) {
			ranges[count - 1].length += len;
		} else {
			if (count == max) {
				unsigned int nmax = max ? max * 2 : 8;
				IOAddressRange *n = IONewData(IOAddressRange, nmax);
				if (n == NULL) {
					if (ranges) {
						IODeleteData(ranges, IOAddressRange, max);
					}
					return ENOMEM;
				}
				if (ranges) {
					bcopy(ranges, n, count * sizeof(*n));
					IODeleteData(ranges, IOAddressRange, max);
				}
				ranges = n;
				max = nmax;
			}
			ranges[count].address = pa;
			ranges[count].length = len;
			count++;
		}
		done += len;
	}
	IOOptionBits mode = (cache & kIOMapCacheMask) >> kIOMapCacheShift;
	IOOptionBits mdOptions = kIODirectionInOut;
	if (mode != kIODefaultCache) {
		mdOptions |= (mode << kIOMemoryBufferCacheShift) & kIOMemoryBufferCacheMask;
	}
	IOMemoryDescriptor *phys = IOMemoryDescriptor::withAddressRanges(ranges, count,
	    mdOptions, TASK_NULL);
	if (phys == NULL) {
		IODeleteData(ranges, IOAddressRange, max);
		return ENOMEM;
	}
	IOMemoryDescriptorReserved *reserved = phys->getKernelReserved();
	struct pie_unmap_notify *n = IOMallocType(struct pie_unmap_notify);
	n->fn = unmapped;
	n->arg = arg;
	n->ranges = IONewData(IOAddressRange, count);
	if (n->ranges == NULL) {
		IOFreeType(n, struct pie_unmap_notify);
		IODeleteData(ranges, IOAddressRange, max);
		phys->release();
		return ENOMEM;
	}
	bcopy(ranges, n->ranges, count * sizeof(*ranges));
	n->count = count;
	IODeleteData(ranges, IOAddressRange, max);
	reserved->pieNotify = n;

	IOOptionBits options = (cache & kIOMapCacheMask) | ((flags & MAP_FIXED) ? 0 : kIOMapAnywhere);
	if ((prot & PROT_WRITE) == 0) {
		options |= kIOMapReadOnly;
	}
	IOMemoryMap *mapping = phys->createMappingInTask(task, (flags & MAP_FIXED) ? *addr : 0,
	    options, 0, size);
	if (mapping == NULL) {
		reserved->pieNotify = NULL;
		pie_unmap_notify_free(n);
		phys->release();
		return ENOMEM;
	}
	*addr = mapping->getAddress();
	mapping->taskDied();
	mapping->release();
	phys->release();
	return 0;
}
