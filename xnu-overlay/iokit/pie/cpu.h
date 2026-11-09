/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <pie/cpu.h>: each CPU's time so far, busy and idle, for drivers that
 * follow the load (PIECPUFreq.kext's governor), from the
 * com.blueberrypieos.kpi.platform symbol set (config/PIE.exports);
 * osfmk/kern/pie_cpu.c implements it from the scheduler's own counts
 * (recount, as processor_info's PROCESSOR_CPU_LOAD_INFO reads them, but in
 * nanoseconds rather than 10 ms ticks). C linkage; any thread context.
 */

#ifndef _PIE_CPU_H_
#define _PIE_CPU_H_

#include <sys/cdefs.h>
#include <stdint.h>
#include <mach/kern_return.h>

__BEGIN_DECLS

#define PIE_CPU_USER   0
#define PIE_CPU_SYSTEM 1
#define PIE_CPU_IDLE   2

/* One more than the highest CPU number: the CPUs are numbered from 0, maybe with gaps. */
unsigned int pie_cpu_count(void);

/*
 * CPU cpu's time since boot in ns: in user space, in the kernel and idle
 * (to now, if it is idle now), into times[PIE_CPU_USER ... IDLE].
 * KERN_INVALID_ARGUMENT for a CPU there is not.
 */
kern_return_t pie_cpu_times(unsigned int cpu, uint64_t times[3]);

__END_DECLS

#endif /* _PIE_CPU_H_ */
