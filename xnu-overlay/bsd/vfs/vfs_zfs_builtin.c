/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * zfs_builtin_init sizes the ZFS ARC at boot: its maximum is a quarter of
 * the memory and its minimum 64 MiB. The boot-args zfs_arc_max= and
 * zfs_arc_min= set other sizes, in bytes, within the limits ZFS accepts.
 */

#include <sys/types.h>
#include <mach/vm_param.h>
#include <pexpert/pexpert.h>

extern uint64_t zfs_arc_max;
extern uint64_t zfs_arc_min;

void zfs_builtin_init(void);

void
zfs_builtin_init(void)
{
	uint64_t value;

	if (PE_parse_boot_argn("zfs_arc_max", &value, sizeof(value))) {
		zfs_arc_max = value;
	} else {
		zfs_arc_max = max_mem / 4;
	}
	if (PE_parse_boot_argn("zfs_arc_min", &value, sizeof(value))) {
		zfs_arc_min = value;
	} else {
		zfs_arc_min = 64ULL << 20;
	}
}
