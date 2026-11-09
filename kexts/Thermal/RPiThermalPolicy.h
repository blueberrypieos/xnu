/* SPDX-License-Identifier: BSD-2-Clause */
/* Copyright (c) 2026 The Blueberry Pie OS Authors */
#ifndef PIE_THERMAL_POLICY_H
#define PIE_THERMAL_POLICY_H
#include <stdint.h>

/* Trips retain their hysteresis state across samples and manual overrides. */
template <class Trip>
static unsigned
thermalNextState(Trip *trips, unsigned count, bool sensorValid, int32_t temperature,
    unsigned maximum, int overrideState)
{
	if (!sensorValid) { return maximum; }
	unsigned state = 0;
	bool mapped = false;
	for (unsigned i = 0; i < count; i++) {
		Trip &trip = trips[i];
		if (!trip.on && temperature >= trip.temperature) {
			trip.on = true;
		} else if (trip.on && (int64_t)temperature < (int64_t)trip.temperature - trip.hysteresis) {
			trip.on = false;
		}
		if (trip.state > 0 && (unsigned)trip.state <= maximum) {
			mapped = true;
			if (trip.on && (unsigned)trip.state > state) { state = (unsigned)trip.state; }
		}
	}
	if (overrideState >= 0 && (unsigned)overrideState <= maximum) { return (unsigned)overrideState; }
	return mapped ? state : maximum;
}
#endif
