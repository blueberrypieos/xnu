/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _PEXPERT_ARM64_PIE_GIC_H
#define _PEXPERT_ARM64_PIE_GIC_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include <pexpert/arm64/board_config.h>

__BEGIN_DECLS

#if HAS_GIC_V2
#define PIE_GIC_INTID_MASK      0x3ff
#else
#define PIE_GIC_INTID_MASK      0xffffff
#endif
#define PIE_GIC_SPURIOUS_MIN    1020

uint32_t pe_gic_ack(void);
void pe_gic_eoi(uint32_t iar);

bool pe_gic_is_ipi(uint32_t irq);
bool pe_gic_is_timer(uint32_t irq);

void pe_gic_set_pending_irq(uint32_t irq);
uint32_t pe_gic_pending_irq(void);

uint32_t pe_gic_num_irqs(void);

void pe_gic_enable_irq(uint32_t irq, bool enable);

void pe_gic_set_edge(uint32_t irq, bool edge);
bool pe_gic_is_edge(uint32_t irq);

void pe_gic_send_ipi(unsigned int cpu_mpidr);

__END_DECLS

#endif
