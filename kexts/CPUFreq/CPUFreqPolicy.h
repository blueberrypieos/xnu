/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PIECPUFreq.kext's governor, as plain C that the kext and
 * tests/cpufreq/unit.c share: the busiest CPU's load, and from it the ARM
 * clock's next rate. The rates it picks are the firmware's minimum, its
 * maximum, and the multiples of 100 MHz between them.
 *
 * ondemand: at a load of 80 % or more, the top rate at once; below 30 %,
 * one step down; between, the rate stays. performance: the top rate.
 * powersave: the minimum. The top rate is the maximum, or a limit below it
 * (rounded down to a step, never below the minimum).
 */

#ifndef _CPUFREQ_POLICY_H
#define _CPUFREQ_POLICY_H

#include <stdint.h>

enum { kCPUFreqOndemand, kCPUFreqPerformance, kCPUFreqPowersave, kCPUFreqGovernors };

#define kCPUFreqStepHz  100000000U
#define kCPUFreqUpPct   80U
#define kCPUFreqDownPct 30U

static const char *const kCPUFreqGovernorNames[kCPUFreqGovernors] = { "ondemand", "performance", "powersave" };

/* A CPU's load in percent over an interval: its busy and idle time then. */
static inline unsigned
CPUFreqLoadPct(uint64_t busy, uint64_t idle)
{
	if (busy + idle == 0) {
		return 0;
	}
	return (unsigned)(busy * 100 / (busy + idle));
}

/* The highest rate the governor may pick: max, or limit below it (0: none). */
static inline uint32_t
CPUFreqTop(uint32_t min, uint32_t max, uint32_t limit)
{
	if (limit == 0 || limit >= max) {
		return max;
	}
	uint32_t top = limit / kCPUFreqStepHz * kCPUFreqStepHz;
	return top < min ? min : top;
}

/* The next rate below cur: the step under it, or min. */
static inline uint32_t
CPUFreqStepDown(uint32_t cur, uint32_t min)
{
	if (cur <= min) {
		return min;
	}
	uint32_t r = (cur - 1) / kCPUFreqStepHz * kCPUFreqStepHz;
	return r < min ? min : r;
}

/* The rate to run at next, from the rate now and the busiest CPU's load. */
static inline uint32_t
CPUFreqNext(int governor, uint32_t cur, uint32_t min, uint32_t max, uint32_t limit, unsigned load)
{
	uint32_t top = CPUFreqTop(min, max, limit);
	uint32_t next;
	switch (governor) {
	case kCPUFreqPerformance:
		return top;
	case kCPUFreqPowersave:
		return min;
	default:
		if (load >= kCPUFreqUpPct) {
			next = top;
		} else if (load < kCPUFreqDownPct) {
			next = CPUFreqStepDown(cur, min);
		} else {
			next = cur;
		}
		break;
	}
	if (next > top) {
		next = top;
	}
	return next < min ? min : next;
}

#endif /* _CPUFREQ_POLICY_H */
