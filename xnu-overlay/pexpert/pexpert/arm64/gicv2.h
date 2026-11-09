/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _PEXPERT_ARM64_GICV2_H
#define _PEXPERT_ARM64_GICV2_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/cdefs.h>

__BEGIN_DECLS

#define GICC_IAR_INTID_MASK     0x3ff
#define GIC_SPURIOUS_IRQ_MIN    1020

uint32_t pe_gicv2_ack(void);
void pe_gicv2_eoi(uint32_t iar);

bool pe_gicv2_is_ipi(uint32_t irq);
bool pe_gicv2_is_timer(uint32_t irq);

void pe_gicv2_set_pending_irq(uint32_t irq);
uint32_t pe_gicv2_pending_irq(void);

uint32_t pe_gicv2_num_irqs(void);

void pe_gicv2_enable_irq(uint32_t irq, bool enable);

void pe_gicv2_set_edge(uint32_t irq, bool edge);
bool pe_gicv2_is_edge(uint32_t irq);

void pe_gicv2_send_ipi(unsigned int cpu_mpidr);

__END_DECLS

#endif
