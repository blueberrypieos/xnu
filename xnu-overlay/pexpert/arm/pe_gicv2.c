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

#if HAS_GIC_V2
#include <pexpert/arm64/gicv2.h>

#define GICD_CTLR               0x000
#define GICD_TYPER              0x004
#define GICD_ISENABLER(n)       (0x100 + 4 * (n))
#define GICD_ICENABLER(n)       (0x180 + 4 * (n))
#define GICD_ICPENDR(n)         (0x280 + 4 * (n))
#define GICD_ICACTIVER(n)       (0x380 + 4 * (n))
#define GICD_IPRIORITYR(n)      (0x400 + 4 * (n))
#define GICD_ITARGETSR(n)       (0x800 + 4 * (n))
#define GICD_ICFGR(n)           (0xc00 + 4 * (n))
#define GICD_SGIR               0xf00

#define GICD_CTLR_ENABLE        0x3
#define GICD_TYPER_ITLINES(t)   ((t) & 0x1f)
#define GICD_SGIR_TARGET_SHIFT  16

#define GICC_CTLR               0x000
#define GICC_PMR                0x004
#define GICC_BPR                0x008
#define GICC_IAR                0x00c
#define GICC_EOIR               0x010

#define GICC_CTLR_ENABLE        0x3

#define GIC_PRIORITY_DEFAULT    0xa0
#define GIC_PRIORITY_MASK       0xf0
#define GIC_MAX_CPU_IF          8

#define GIC_SGI_IPI             0
#define GIC_PPI_VTIMER          27

static vm_offset_t gicd_base;
static vm_offset_t gicc_base;
static uint32_t gic_nirqs;

#define GIC_CPU_IF_VALID        0x80000000u
static struct {
	uint32_t phys_id;
	uint8_t mask;
} gic_cpu_if[GIC_MAX_CPU_IF];

static uint32_t gic_pending_irq[MAX_CPUS];

static inline uint32_t
gicd_read(uint32_t off)
{
	return *(volatile uint32_t *)(gicd_base + off);
}

static inline void
gicd_write(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(gicd_base + off) = v;
}

static inline uint32_t
gicc_read(uint32_t off)
{
	return *(volatile uint32_t *)(gicc_base + off);
}

static inline void
gicc_write(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(gicc_base + off) = v;
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

static const char *const gic_compats[] = {
	"arm,gic-400", "arm,cortex-a15-gic", "arm,cortex-a9-gic", "arm,gic-v2",
};

static bool
find_gic_below(DTEntry parent, DTEntry *out)
{
	OpaqueDTEntryIterator iter;
	DTEntry entry;

	if (SecureDTInitEntryIterator(parent, &iter) != kSuccess) {
		return false;
	}
	while (SecureDTIterateEntries(&iter, &entry) == kSuccess) {
		for (size_t i = 0; i < sizeof(gic_compats) / sizeof(gic_compats[0]); i++) {
			if (compatible_with(entry, gic_compats[i])) {
				*out = entry;
				return true;
			}
		}
		if (find_gic_below(entry, out)) {
			return true;
		}
	}
	return false;
}

static DTEntry
find_gic(void)
{
	DTEntry armio, gic;

	if (SecureDTLookupEntry(NULL, "/arm-io", &armio) != kSuccess) {
		panic("%s: no /arm-io in the device tree", __func__);
	}
	if (!find_gic_below(armio, &gic)) {
		panic("%s: no GICv2 node under /arm-io", __func__);
	}
	return gic;
}

static void
gic_dist_init(void)
{
	DTEntry gic = find_gic();
	void const *prop;
	unsigned int size;

	if (SecureDTGetProperty(gic, "reg", &prop, &size) != kSuccess ||
	    size < 4 * sizeof(uint64_t)) {
		panic("%s: GIC node needs GICD and GICC reg entries", __func__);
	}
	uint64_t const *reg = prop;
	vm_offset_t soc_base_phys = pe_arm_get_soc_base_phys();

	gicd_base = ml_io_map(soc_base_phys + reg[0], reg[1]);
	gicc_base = ml_io_map(soc_base_phys + reg[2], reg[3]);
	if (!gicd_base || !gicc_base) {
		panic("%s: cannot map the GIC", __func__);
	}

	gic_nirqs = 32 * (GICD_TYPER_ITLINES(gicd_read(GICD_TYPER)) + 1);
	if (gic_nirqs > 1020) {
		gic_nirqs = 1020;
	}

	gicd_write(GICD_CTLR, 0);

	for (uint32_t i = 32; i < gic_nirqs; i += 32) {
		gicd_write(GICD_ICENABLER(i / 32), ~0u);
		gicd_write(GICD_ICPENDR(i / 32), ~0u);
		gicd_write(GICD_ICACTIVER(i / 32), ~0u);
	}
	for (uint32_t i = 32; i < gic_nirqs; i += 16) {
		gicd_write(GICD_ICFGR(i / 16), 0);
	}
	uint32_t target = gicd_read(GICD_ITARGETSR(0)) & 0xff;
	if (target == 0) {
		target = 1;
	}
	for (uint32_t i = 32; i < gic_nirqs; i += 4) {
		gicd_write(GICD_IPRIORITYR(i / 4), GIC_PRIORITY_DEFAULT * 0x01010101u);
		gicd_write(GICD_ITARGETSR(i / 4), target * 0x01010101u);
	}

	gicd_write(GICD_CTLR, GICD_CTLR_ENABLE);
}

static void
gic_cpu_init(void)
{
	uint64_t mpidr = __builtin_arm_rsr64("MPIDR_EL1");
	uint32_t phys_id = (uint32_t)(mpidr & (MPIDR_AFF1_MASK | MPIDR_AFF0_MASK));

	uint8_t mask = gicd_read(GICD_ITARGETSR(0)) & 0xff;
	unsigned slot;
	for (slot = 0; slot < GIC_MAX_CPU_IF; slot++) {
		uint32_t id = os_atomic_load(&gic_cpu_if[slot].phys_id, acquire);
		if (id == (phys_id | GIC_CPU_IF_VALID) ||
		    (id == 0 && os_atomic_cmpxchg(&gic_cpu_if[slot].phys_id, 0, phys_id | GIC_CPU_IF_VALID, acq_rel))) {
			break;
		}
	}
	if (slot < GIC_MAX_CPU_IF) {
		os_atomic_store(&gic_cpu_if[slot].mask, mask ? mask : (uint8_t)(1u << slot), release);
	}

	gicd_write(GICD_ICENABLER(0), ~0u);
	gicd_write(GICD_ICPENDR(0), ~0u);
	gicd_write(GICD_ICACTIVER(0), ~0u);
	for (uint32_t i = 0; i < 32; i += 4) {
		gicd_write(GICD_IPRIORITYR(i / 4), GIC_PRIORITY_DEFAULT * 0x01010101u);
	}
	gicd_write(GICD_ISENABLER(0), (1u << GIC_SGI_IPI) | (1u << GIC_PPI_VTIMER));

	gicc_write(GICC_PMR, GIC_PRIORITY_MASK);
	gicc_write(GICC_BPR, 0);
	gicc_write(GICC_CTLR, GICC_CTLR_ENABLE);
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
pe_gicv2_ack(void)
{
	return gicc_read(GICC_IAR);
}

void
pe_gicv2_eoi(uint32_t iar)
{
	gicc_write(GICC_EOIR, iar);
}

bool
pe_gicv2_is_ipi(uint32_t irq)
{
	return irq < 16;
}

bool
pe_gicv2_is_timer(uint32_t irq)
{
	return irq == GIC_PPI_VTIMER;
}

void
pe_gicv2_set_pending_irq(uint32_t irq)
{
	gic_pending_irq[cpu_number()] = irq;
}

uint32_t
pe_gicv2_pending_irq(void)
{
	return gic_pending_irq[cpu_number()];
}

void
pe_gicv2_send_ipi(unsigned int cpu_mpidr)
{
	uint32_t phys_id = (cpu_mpidr & (MPIDR_AFF1_MASK | MPIDR_AFF0_MASK)) | GIC_CPU_IF_VALID;
	uint32_t mask = 0;

	for (unsigned i = 0; i < GIC_MAX_CPU_IF; i++) {
		if (os_atomic_load(&gic_cpu_if[i].phys_id, acquire) == phys_id) {
			mask = os_atomic_load(&gic_cpu_if[i].mask, acquire);
			break;
		}
	}
	if (mask == 0) {
		mask = 1u << (((cpu_mpidr & MPIDR_AFF0_MASK) | ((cpu_mpidr & MPIDR_AFF1_MASK) >> 8)) & 7);
	}
	__builtin_arm_dsb(DSB_ISHST);
	gicd_write(GICD_SGIR, (mask << GICD_SGIR_TARGET_SHIFT) | GIC_SGI_IPI);
}

uint32_t
pe_gicv2_num_irqs(void)
{
	return gic_nirqs;
}

void
pe_gicv2_enable_irq(uint32_t irq, bool enable)
{
	if (irq >= gic_nirqs) {
		return;
	}
	gicd_write(enable ? GICD_ISENABLER(irq / 32) : GICD_ICENABLER(irq / 32), 1u << (irq % 32));
}

SIMPLE_LOCK_DECLARE(gic_cfg_lock, 0);

void
pe_gicv2_set_edge(uint32_t irq, bool edge)
{
	if (irq < 32 || irq >= gic_nirqs) {
		return;
	}
	simple_lock(&gic_cfg_lock, LCK_GRP_NULL);
	uint32_t bit = 2u << (2 * (irq % 16));
	uint32_t v = gicd_read(GICD_ICFGR(irq / 16));
	gicd_write(GICD_ICFGR(irq / 16), edge ? (v | bit) : (v & ~bit));
	simple_unlock(&gic_cfg_lock);
}

bool
pe_gicv2_is_edge(uint32_t irq)
{
	if (irq >= gic_nirqs) {
		return false;
	}
	return (gicd_read(GICD_ICFGR(irq / 16)) >> (2 * (irq % 16) + 1)) & 1;
}

#endif
