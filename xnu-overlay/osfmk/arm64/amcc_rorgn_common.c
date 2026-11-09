/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <pexpert/arm64/board_config.h>
#include <arm64/amcc_rorgn.h>
#include <kern/assert.h>
#include <libkern/section_keywords.h>
#include <mach/vm_param.h>
#include <vm/pmap.h>

#if defined(KERNEL_INTEGRITY_KTRR) || defined(KERNEL_INTEGRITY_CTRR) || defined(KERNEL_INTEGRITY_PV_CTRR)

extern vm_offset_t segLOWESTRO, segHIGHESTRO;

SECURITY_READ_ONLY_LATE(vm_offset_t) ctrr_begin = 0;
SECURITY_READ_ONLY_LATE(vm_offset_t) ctrr_end = 0;

SECURITY_READ_ONLY_LATE(vm_offset_t) rorgn_begin = 0;
SECURITY_READ_ONLY_LATE(vm_offset_t) rorgn_end = 0;

void
rorgn_stash_range(void)
{
	assert(segLOWESTRO != 0 && segHIGHESTRO > segLOWESTRO);

	ctrr_begin = segLOWESTRO;
	ctrr_end = segHIGHESTRO;
	rorgn_begin = (vm_offset_t)kvtophys(segLOWESTRO);
	rorgn_end = (vm_offset_t)kvtophys(segHIGHESTRO - 1) + 1;
}

void
rorgn_lockdown(void)
{
}

bool
rorgn_contains(vm_offset_t addr, vm_size_t size, bool defval)
{
	vm_offset_t end;

	if (ctrr_begin == 0 || ctrr_end == 0) {
		return defval;
	}
	if (os_add_overflow(addr, size, &end)) {
		return false;
	}
	return addr >= ctrr_begin && end <= ctrr_end;
}

void
rorgn_validate_core(void)
{
}

#endif
