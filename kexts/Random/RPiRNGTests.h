/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * FIPS 140-2's power-up statistical tests (section 4.9.1, with Change
 * Notice 1's runs intervals, as rngtest has them; Change Notice 2 dropped
 * the tests) on 20,000 bits: the monobit, poker, runs and long run tests.
 * They catch a source that is stuck, biased or periodic, not one that is
 * merely predictable; a good source fails them about once in a thousand.
 * PIERandom.kext runs them on the RNG before feeding the kernel anything
 * from it; tests/random/unit.c runs them on the build host. Plain C,
 * nothing but the bytes.
 */

#ifndef _RPI_RNG_TESTS_H
#define _RPI_RNG_TESTS_H

#include <stdint.h>

#define kRNGTestBytes 2500      /* 20,000 bits */

/* NULL if the bits pass, else the name of the first test that failed. */
static inline const char *
RNGTestFIPS140(const uint8_t *bytes)
{
	/* Monobit: 9,725 < ones < 10,275. */
	unsigned ones = 0;
	for (unsigned i = 0; i < kRNGTestBytes; i++) {
		ones += (unsigned)__builtin_popcount(bytes[i]);
	}
	if (ones <= 9725 || ones >= 10275) {
		return "monobit";
	}

	/* Poker: the 5,000 4-bit nibbles' counts f[i]; 2.16 < 16/5000 * sum(f[i]^2) - 5000 < 46.17. */
	uint32_t f[16] = { 0 };
	for (unsigned i = 0; i < kRNGTestBytes; i++) {
		f[bytes[i] >> 4]++;
		f[bytes[i] & 15]++;
	}
	uint64_t sum = 0;
	for (unsigned i = 0; i < 16; i++) {
		sum += (uint64_t)f[i] * f[i];
	}
	/* In units of 1/5000: 10,800 < 16 * sum - 25,000,000 < 230,850. */
	int64_t x = (int64_t)(16 * sum) - 25000000;
	if (x <= 10800 || x >= 230850) {
		return "poker";
	}

	/*
	 * Runs: for each value, the runs of each length 1-5 and of 6 or more
	 * fall in these intervals; long run: no run of 26 or more.
	 */
	static const uint16_t lo[6] = { 2315, 1114, 527, 240, 103, 103 };
	static const uint16_t hi[6] = { 2685, 1386, 723, 384, 209, 209 };
	uint32_t runs[2][6] = { { 0 } };
	unsigned bit = bytes[0] >> 7, len = 0;
	for (unsigned i = 0; i < 8 * kRNGTestBytes; i++) {
		unsigned b = (bytes[i / 8] >> (7 - i % 8)) & 1;
		if (b == bit) {
			len++;
		} else {
			runs[bit][(len < 6 ? len : 6) - 1]++;
			bit = b;
			len = 1;
		}
		if (len >= 26) {
			return "long run";
		}
	}
	runs[bit][(len < 6 ? len : 6) - 1]++;
	for (unsigned v = 0; v < 2; v++) {
		for (unsigned l = 0; l < 6; l++) {
			if (runs[v][l] < lo[l] || runs[v][l] > hi[l]) {
				return "runs";
			}
		}
	}
	return (const char *)0;
}

#endif /* _RPI_RNG_TESTS_H */
