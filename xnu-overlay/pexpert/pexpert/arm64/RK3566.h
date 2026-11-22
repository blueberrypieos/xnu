/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _PEXPERT_ARM64_RK3566_H
#define _PEXPERT_ARM64_RK3566_H

#define RK3566                    1

#define NO_MONITOR                1
#define NO_ECORE                  1
#define NO_CPU_OVRD               1

#define CORTEX_A55                1

#define __ARM_16K_PG__            1
#define __ARM_PAN_AVAILABLE__     1

#include <pexpert/arm64/cortex_arm64_common.h>

#define ARM_VM_PAGE_IS_HW_PAGE    1

#define HAS_PIE_GIC_V3            1
#define USE_APPLEARMSMP           1

#ifndef ASSEMBLER
#define PL011_UART
#define DW8250_UART
#endif

#endif
