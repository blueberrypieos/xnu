/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <kern/processor.h>
#include <kern/recount.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>

#include "../../iokit/pie/cpu.h"

unsigned int
pie_cpu_count(void)
{
	return (unsigned int)ml_get_max_cpu_number() + 1;
}

kern_return_t
pie_cpu_times(unsigned int cpu, uint64_t times[3])
{
	if (times == NULL || cpu > (unsigned int)ml_get_max_cpu_number()) {
		return KERN_INVALID_ARGUMENT;
	}
	processor_t processor = cpu_to_processor((int)cpu);
	if (processor == PROCESSOR_NULL) {
		return KERN_INVALID_ARGUMENT;
	}
	struct recount_usage usage = { 0 };
	uint64_t idle = 0;
	recount_processor_usage(&processor->pr_recount, &usage, &idle);
	absolutetime_to_nanoseconds(usage.ru_metrics[RCT_LVL_USER].rm_time_mach, &times[PIE_CPU_USER]);
	absolutetime_to_nanoseconds(recount_usage_system_time_mach(&usage), &times[PIE_CPU_SYSTEM]);
	absolutetime_to_nanoseconds(idle, &times[PIE_CPU_IDLE]);
	return KERN_SUCCESS;
}
