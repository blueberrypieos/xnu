/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _PEXPERT_ARM64_CORTEX_ARM64_COMMON_H
#define _PEXPERT_ARM64_CORTEX_ARM64_COMMON_H

#define __ARM_ARCH__                         8
#define __ARM_VMSA__                         8
#define __ARM_VFP__                          4
#define __ARM_DEBUG__                        7
#define __ARM_ENABLE_SWAP__                  1

#define ARM_ARCH_TIMER

#define CPU_PHYS_ID_AFF1_AFF0                1

#define CPU_VERSION_A0                       0x00
#define CPU_VERSION_A1                       0x01
#define CPU_VERSION_B0                       0x10
#define CPU_VERSION_B1                       0x11
#define CPU_VERSION_C0                       0x20
#define CPU_VERSION_UNKNOWN                  0xff

#endif
