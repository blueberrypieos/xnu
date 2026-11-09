/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
#ifndef _RPI_RTC_ALARM_H
#define _RPI_RTC_ALARM_H

#include <stdint.h>

enum {
	RTC_TIME = 0, RTC_ALARM, RTC_ALARM_PENDING, RTC_ALARM_ENABLE,
	RTC_BBAT_CHG_VOLTS, RTC_BBAT_CHG_VOLTS_MIN, RTC_BBAT_CHG_VOLTS_MAX, RTC_BBAT_VOLTS,
};

/* Never enable an incomplete alarm. Attempt to disable it after any error. */
static inline int
RPiRTCProgramAlarm(uint32_t secs, int (*writeReg)(void *, uint32_t, uint32_t), void *context)
{
	int err = writeReg(context, RTC_ALARM_ENABLE, 0);
	if (err == 0) {
		err = writeReg(context, RTC_ALARM_PENDING, 1);
	}
	if (err == 0 && secs != 0) {
		err = writeReg(context, RTC_ALARM, secs);
	}
	if (err == 0 && secs != 0) {
		err = writeReg(context, RTC_ALARM_ENABLE, 1);
	}
	if (err != 0) {
		(void)writeReg(context, RTC_ALARM_ENABLE, 0);
	}
	return err;
}

#endif
