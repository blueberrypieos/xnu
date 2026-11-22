/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_ROCKCHIP_RKSOC_H
#define _IOKIT_ROCKCHIP_RKSOC_H

#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <kern/thread.h>

/* RK3568 clock units: the main one and the PMU's. */
enum {
	RK_CRU_GPLL_CON0 = 0x040, RK_CRU_CPLL_CON0 = 0x060, RK_CRU_MODE_CON = 0x0c0,
	RK_CRU_GPLL_MODE_SHIFT = 6, RK_CRU_CPLL_MODE_SHIFT = 4,
	RK_PMUCRU_PPLL_CON0 = 0x000, RK_PMUCRU_MODE_CON = 0x080, RK_PMUCRU_PPLL_MODE_SHIFT = 0,
};
#define RK_CRU_CLKSEL(i)        (0x100 + (i) * 4)
#define RK_CRU_GATE(i)          (0x300 + (i) * 4)
#define RK_CRU_SOFTRST(i)       (0x400 + (i) * 4)
#define RK_PMUCRU_CLKSEL(i)     (0x100 + (i) * 4)
#define RK_PMUCRU_GATE(i)       (0x180 + (i) * 4)
#define RK_PMUCRU_SOFTRST(i)    (0x200 + (i) * 4)

IOMemoryMap *RKMapNode(IORegistryEntry *node);
IOMemoryMap *RKMapCompatible(const char *compat);
IOMemoryMap *RKMapPHandle(uint32_t phandle);
IORegistryEntry *RKFindPHandle(IORegistryEntry *root, uint32_t phandle);
void RKApplyPinctrl(IOService *provider, const char *driver);
IORegistryEntry *RKChildNamed(IORegistryEntry *parent, const char *name);
uint64_t RKPLLRate(IOMemoryMap *cru, uint32_t con, uint32_t mode, unsigned modeShift);
bool RKWaitForBootVolume(uint32_t seconds = 60);
bool RKStartThread(OSObject *owner, void (*fn)(void *, wait_result_t));

static inline uint32_t
RKRead(IOMemoryMap *map, uint32_t off)
{
	return *(volatile uint32_t *)(map->getVirtualAddress() + off);
}

/* A write of the bits in mask, through the register's upper half of write enables. */
static inline void
RKWriteMasked(IOMemoryMap *map, uint32_t off, uint32_t mask, uint32_t value)
{
	*(volatile uint32_t *)(map->getVirtualAddress() + off) = (mask << 16) | (value & mask);
}

#endif
