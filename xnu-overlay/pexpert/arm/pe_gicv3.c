/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <stdint.h>
#include <arm64/proc_reg.h>
#include <kern/cpu_number.h>
#include <kern/simple_lock.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <pexpert/device_tree.h>
#include <pexpert/arm64/board_config.h>
#include <machine/atomic.h>

#if HAS_PIE_GIC_V3
#include <pexpert/arm64/pie_gic.h>

#define GICD_CTLR               0x0000
#define GICD_TYPER              0x0004
#define GICD_IGROUPR(n)         (0x0080 + 4 * (n))
#define GICD_ISENABLER(n)       (0x0100 + 4 * (n))
#define GICD_ICENABLER(n)       (0x0180 + 4 * (n))
#define GICD_ICPENDR(n)         (0x0280 + 4 * (n))
#define GICD_ICACTIVER(n)       (0x0380 + 4 * (n))
#define GICD_IPRIORITYR(n)      (0x0400 + 4 * (n))
#define GICD_ICFGR(n)           (0x0c00 + 4 * (n))
#define GICD_IGRPMODR(n)        (0x0d00 + 4 * (n))
#define GICD_IROUTER(n)         (0x6000 + 8 * (n))

#define GICD_CTLR_RWP           (1u << 31)
#define GICD_CTLR_ARE           (1u << 4)
#define GICD_CTLR_ENABLE_G1A    (1u << 1)
#define GICD_CTLR_ENABLE_G1     (1u << 0)
#define GICD_TYPER_ITLINES(t)   ((t) & 0x1f)

#define GICR_CTLR               0x0000
#define GICR_TYPER              0x0008
#define GICR_WAKER              0x0014
#define GICR_CTLR_RWP           (1u << 3)
#define GICR_TYPER_VLPIS        (1ull << 1)
#define GICR_TYPER_LAST         (1ull << 4)
#define GICR_WAKER_SLEEP        (1u << 1)
#define GICR_WAKER_ASLEEP       (1u << 2)
#define GICR_FRAME_SIZE         0x20000
#define GICR_FRAME_SIZE_VLPI    0x40000

#define GICR_SGI                0x10000
#define GICR_IGROUPR0           (GICR_SGI + 0x0080)
#define GICR_ISENABLER0         (GICR_SGI + 0x0100)
#define GICR_ICENABLER0         (GICR_SGI + 0x0180)
#define GICR_ICPENDR0           (GICR_SGI + 0x0280)
#define GICR_ICACTIVER0         (GICR_SGI + 0x0380)
#define GICR_IPRIORITYR(n)      (GICR_SGI + 0x0400 + 4 * (n))
#define GICR_IGRPMODR0          (GICR_SGI + 0x0d00)

#define ICC_SRE_SRE             (1ull << 0)
#define ICC_SRE_DFB             (1ull << 1)
#define ICC_SRE_DIB             (1ull << 2)
#define ICC_CTLR_EOIMODE        (1ull << 1)

#define GIC_PRIORITY_DEFAULT    0xa0
#define GIC_PRIORITY_MASK       0xf0
#define GIC_SGI_IPI             0
#define GIC_PPI_VTIMER          27
#define GIC_SPIN_LIMIT          10000000

static vm_offset_t gicd_base;
static vm_offset_t gicr_base;
static vm_size_t gicr_size;
static uint32_t gic_nirqs;
static vm_offset_t gicr_cpu[MAX_CPUS];
static uint32_t gic_pending_irq[MAX_CPUS];

static inline uint32_t
read32(vm_offset_t addr)
{
	return *(volatile uint32_t *)addr;
}

static inline void
write32(vm_offset_t addr, uint32_t v)
{
	*(volatile uint32_t *)addr = v;
}

static inline uint64_t
read64(vm_offset_t addr)
{
	return *(volatile uint64_t *)addr;
}

static inline void
write64(vm_offset_t addr, uint64_t v)
{
	*(volatile uint64_t *)addr = v;
}

static bool
compatible_with(DTEntry entry, const char *compat)
{
	void const *prop;
	unsigned int size;

	if (SecureDTGetProperty(entry, "compatible", &prop, &size) != kSuccess) {
		return false;
	}
	const char *s = prop, *end = s + size;
	while (s < end) {
		if (strcmp(s, compat) == 0) {
			return true;
		}
		s += strnlen(s, (size_t)(end - s)) + 1;
	}
	return false;
}

static bool
find_gic_below(DTEntry parent, DTEntry *out)
{
	OpaqueDTEntryIterator iter;
	DTEntry entry;

	if (SecureDTInitEntryIterator(parent, &iter) != kSuccess) {
		return false;
	}
	while (SecureDTIterateEntries(&iter, &entry) == kSuccess) {
		if (compatible_with(entry, "arm,gic-v3")) {
			*out = entry;
			return true;
		}
		if (find_gic_below(entry, out)) {
			return true;
		}
	}
	return false;
}

static void
gicd_wait(void)
{
	for (int i = 0; i < GIC_SPIN_LIMIT && (read32(gicd_base + GICD_CTLR) & GICD_CTLR_RWP); i++) {
	}
}

static void
gicr_wait(vm_offset_t rd)
{
	for (int i = 0; i < GIC_SPIN_LIMIT && (read32(rd + GICR_CTLR) & GICR_CTLR_RWP); i++) {
	}
}

static void
gic_dist_init(void)
{
	DTEntry armio, gic;
	void const *prop;
	unsigned int size;

	if (SecureDTLookupEntry(NULL, "/arm-io", &armio) != kSuccess) {
		panic("%s: no /arm-io in the device tree", __func__);
	}
	if (!find_gic_below(armio, &gic)) {
		panic("%s: no GICv3 node under /arm-io", __func__);
	}
	if (SecureDTGetProperty(gic, "reg", &prop, &size) != kSuccess ||
	    size < 4 * sizeof(uint64_t)) {
		panic("%s: GIC node needs GICD and GICR reg entries", __func__);
	}
	uint64_t const *reg = prop;
	vm_offset_t soc_base_phys = pe_arm_get_soc_base_phys();

	gicd_base = ml_io_map(soc_base_phys + reg[0], reg[1]);
	gicr_size = (vm_size_t)reg[3];
	gicr_base = ml_io_map(soc_base_phys + reg[2], gicr_size);
	if (!gicd_base || !gicr_base) {
		panic("%s: cannot map the GIC", __func__);
	}

	gic_nirqs = 32 * (GICD_TYPER_ITLINES(read32(gicd_base + GICD_TYPER)) + 1);
	if (gic_nirqs > 1020) {
		gic_nirqs = 1020;
	}

	write32(gicd_base + GICD_CTLR, 0);
	gicd_wait();

	for (uint32_t i = 32; i < gic_nirqs; i += 32) {
		write32(gicd_base + GICD_IGROUPR(i / 32), ~0u);
		write32(gicd_base + GICD_IGRPMODR(i / 32), 0);
		write32(gicd_base + GICD_ICENABLER(i / 32), ~0u);
		write32(gicd_base + GICD_ICPENDR(i / 32), ~0u);
		write32(gicd_base + GICD_ICACTIVER(i / 32), ~0u);
	}
	for (uint32_t i = 32; i < gic_nirqs; i += 16) {
		write32(gicd_base + GICD_ICFGR(i / 16), 0);
	}
	for (uint32_t i = 32; i < gic_nirqs; i += 4) {
		write32(gicd_base + GICD_IPRIORITYR(i / 4), GIC_PRIORITY_DEFAULT * 0x01010101u);
	}
	gicd_wait();

	write32(gicd_base + GICD_CTLR, GICD_CTLR_ARE);
	gicd_wait();

	uint64_t route = __builtin_arm_rsr64("MPIDR_EL1") & 0xff00ffffffull;
	for (uint32_t i = 32; i < gic_nirqs; i++) {
		write64(gicd_base + GICD_IROUTER(i), route);
	}

	write32(gicd_base + GICD_CTLR, GICD_CTLR_ARE | GICD_CTLR_ENABLE_G1A | GICD_CTLR_ENABLE_G1);
	gicd_wait();
}

static vm_offset_t
gic_find_redistributor(uint64_t mpidr)
{
	uint32_t aff = (uint32_t)(((mpidr >> 32) & 0xff) << 24) | (uint32_t)(mpidr & 0xffffff);

	for (vm_offset_t off = 0; off + GICR_FRAME_SIZE <= gicr_size;) {
		uint64_t typer = read64(gicr_base + off + GICR_TYPER);
		if ((uint32_t)(typer >> 32) == aff) {
			return gicr_base + off;
		}
		if (typer & GICR_TYPER_LAST) {
			break;
		}
		off += (typer & GICR_TYPER_VLPIS) ? GICR_FRAME_SIZE_VLPI : GICR_FRAME_SIZE;
	}
	return 0;
}

static void
gic_cpu_init(void)
{
	uint64_t mpidr = __builtin_arm_rsr64("MPIDR_EL1");
	vm_offset_t rd = gic_find_redistributor(mpidr);

	if (!rd) {
		panic("%s: no GIC redistributor for MPIDR 0x%llx", __func__, mpidr);
	}
	gicr_cpu[cpu_number()] = rd;

	write32(rd + GICR_WAKER, read32(rd + GICR_WAKER) & ~GICR_WAKER_SLEEP);
	for (int i = 0; i < GIC_SPIN_LIMIT && (read32(rd + GICR_WAKER) & GICR_WAKER_ASLEEP); i++) {
	}

	write32(rd + GICR_IGROUPR0, ~0u);
	write32(rd + GICR_IGRPMODR0, 0);
	write32(rd + GICR_ICENABLER0, ~0u);
	gicr_wait(rd);
	write32(rd + GICR_ICPENDR0, ~0u);
	write32(rd + GICR_ICACTIVER0, ~0u);
	for (uint32_t i = 0; i < 32; i += 4) {
		write32(rd + GICR_IPRIORITYR(i / 4), GIC_PRIORITY_DEFAULT * 0x01010101u);
	}
	write32(rd + GICR_ISENABLER0, (1u << GIC_SGI_IPI) | (1u << GIC_PPI_VTIMER));

	uint64_t sre = __builtin_arm_rsr64("ICC_SRE_EL1");
	__builtin_arm_wsr64("ICC_SRE_EL1", sre | ICC_SRE_SRE | ICC_SRE_DFB | ICC_SRE_DIB);
	__builtin_arm_isb(ISB_SY);
	if (!(__builtin_arm_rsr64("ICC_SRE_EL1") & ICC_SRE_SRE)) {
		panic("%s: the GICv3 system register interface is disabled at EL2", __func__);
	}
	__builtin_arm_wsr64("ICC_PMR_EL1", GIC_PRIORITY_MASK);
	__builtin_arm_wsr64("ICC_BPR1_EL1", 0);
	__builtin_arm_wsr64("ICC_CTLR_EL1", __builtin_arm_rsr64("ICC_CTLR_EL1") & ~ICC_CTLR_EOIMODE);
	__builtin_arm_wsr64("ICC_IGRPEN1_EL1", 1);
	__builtin_arm_dsb(DSB_SY);
	__builtin_arm_isb(ISB_SY);
}

void
pe_init_fiq(void)
{
	if (!gicd_base) {
		gic_dist_init();
	}
	gic_cpu_init();
}

uint32_t
pe_gic_ack(void)
{
	return (uint32_t)__builtin_arm_rsr64("ICC_IAR1_EL1");
}

void
pe_gic_eoi(uint32_t iar)
{
	__builtin_arm_wsr64("ICC_EOIR1_EL1", iar);
	__builtin_arm_isb(ISB_SY);
}

bool
pe_gic_is_ipi(uint32_t irq)
{
	return irq < 16;
}

bool
pe_gic_is_timer(uint32_t irq)
{
	return irq == GIC_PPI_VTIMER;
}

void
pe_gic_set_pending_irq(uint32_t irq)
{
	gic_pending_irq[cpu_number()] = irq;
}

uint32_t
pe_gic_pending_irq(void)
{
	return gic_pending_irq[cpu_number()];
}

void
pe_gic_send_ipi(unsigned int cpu_mpidr)
{
	uint64_t aff0 = cpu_mpidr & MPIDR_AFF0_MASK;
	uint64_t aff1 = (cpu_mpidr & MPIDR_AFF1_MASK) >> 8;
	uint64_t aff2 = (cpu_mpidr & MPIDR_AFF2_MASK) >> MPIDR_AFF2_SHIFT;
	uint64_t sgi = (aff2 << 32) | (aff1 << 16) | ((aff0 >> 4) << 44) | (1ull << (aff0 & 0xf)) |
	    ((uint64_t)GIC_SGI_IPI << 24);

	__builtin_arm_dsb(DSB_ISHST);
	__builtin_arm_wsr64("ICC_SGI1R_EL1", sgi);
	__builtin_arm_isb(ISB_SY);
}

uint32_t
pe_gic_num_irqs(void)
{
	return gic_nirqs;
}

void
pe_gic_enable_irq(uint32_t irq, bool enable)
{
	if (irq >= gic_nirqs) {
		return;
	}
	if (irq < 32) {
		vm_offset_t rd = gicr_cpu[cpu_number()];
		if (rd) {
			write32(rd + (enable ? GICR_ISENABLER0 : GICR_ICENABLER0), 1u << irq);
		}
		return;
	}
	write32(gicd_base + (enable ? GICD_ISENABLER(irq / 32) : GICD_ICENABLER(irq / 32)), 1u << (irq % 32));
}

SIMPLE_LOCK_DECLARE(gic_cfg_lock, 0);

void
pe_gic_set_edge(uint32_t irq, bool edge)
{
	if (irq < 32 || irq >= gic_nirqs) {
		return;
	}
	simple_lock(&gic_cfg_lock, LCK_GRP_NULL);
	uint32_t bit = 2u << (2 * (irq % 16));
	uint32_t v = read32(gicd_base + GICD_ICFGR(irq / 16));
	write32(gicd_base + GICD_ICFGR(irq / 16), edge ? (v | bit) : (v & ~bit));
	simple_unlock(&gic_cfg_lock);
}

bool
pe_gic_is_edge(uint32_t irq)
{
	if (irq < 16) {
		return true;
	}
	if (irq < 32 || irq >= gic_nirqs) {
		return false;
	}
	return (read32(gicd_base + GICD_ICFGR(irq / 16)) >> (2 * (irq % 16) + 1)) & 1;
}

#endif
