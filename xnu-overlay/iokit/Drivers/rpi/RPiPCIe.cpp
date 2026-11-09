/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceMemory.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/OSByteOrder.h>
#include <pexpert/pexpert.h>

#include "RPiPlatform.h"

enum {
	PCIE_RC_CFG_PCIE_CAP                 = 0x00ac,
	PCIE_RC_CFG_VENDOR_SPECIFIC_REG1     = 0x0188,
	PCIE_RC_CFG_PRIV1_ID_VAL3            = 0x043c,
	PCIE_RC_CFG_PRIV1_LINK_CAPABILITY    = 0x04dc,
	PCIE_RC_CFG_PRIV1_ROOT_CAP           = 0x04f8,
	PCIE_RC_DL_MDIO_ADDR                 = 0x1100,
	PCIE_RC_DL_MDIO_WR_DATA              = 0x1104,
	PCIE_RC_PL_PHY_CTL_15                = 0x184c,
	PCIE_RC_PL_LTSSM_HIST_0              = 0x1cec,
	PCIE_MISC_MISC_CTRL                  = 0x4008,
	PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO     = 0x400c,
	PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI     = 0x4010,
	PCIE_MISC_RC_BAR1_CONFIG_LO          = 0x402c,
	PCIE_MISC_RC_BAR2_CONFIG_LO          = 0x4034,
	PCIE_MISC_RC_BAR2_CONFIG_HI          = 0x4038,
	PCIE_MISC_RC_BAR3_CONFIG_LO          = 0x403c,
	PCIE_MISC_RC_CONFIG_RETRY_TIMEOUT    = 0x405c,
	PCIE_MISC_PCIE_CTRL                  = 0x4064,
	PCIE_MISC_PCIE_STATUS                = 0x4068,
	PCIE_MISC_REVISION                   = 0x406c,
	PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT = 0x4070,
	PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI    = 0x4080,
	PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI   = 0x4084,
	PCIE_MISC_RC_BAR1_CONFIG_HI          = 0x4030,
	PCIE_MISC_RC_BAR3_CONFIG_HI          = 0x4040,
	PCIE_MISC_CTRL_1                     = 0x40a0,
	PCIE_MISC_UBUS_CTRL                  = 0x40a4,
	PCIE_MISC_UBUS_TIMEOUT               = 0x40a8,
	PCIE_MISC_UBUS_BAR1_CONFIG_REMAP     = 0x40ac,
	PCIE_MISC_AXI_INTF_CTRL              = 0x416c,
	PCIE_MISC_AXI_READ_ERROR_DATA        = 0x4170,
	PCIE_MISC_HARD_PCIE_HARD_DEBUG       = 0x4204,
	PCIE_MISC_HARD_PCIE_HARD_DEBUG_2712  = 0x4304,
	PCIE_EXT_CFG_DATA                    = 0x8000,
	PCIE_EXT_CFG_INDEX                   = 0x9000,
	PCIE_RGR1_SW_INIT_1                  = 0x9210,
};

enum {
	MISC_CTRL_RCB_64B_MODE       = 1U << 7,
	MISC_CTRL_RCB_MPS_MODE       = 1U << 10,
	MISC_CTRL_SCB_ACCESS_EN      = 1U << 12,
	MISC_CTRL_CFG_READ_UR_MODE   = 1U << 13,
	MISC_CTRL_MAX_BURST_SHIFT    = 20,
	MISC_CTRL_MAX_BURST_MASK     = 3U << 20,
	MISC_CTRL_SCB0_SIZE_SHIFT    = 27,
	MISC_CTRL_SCB0_SIZE_MASK     = 0x1fU << 27,
	RC_BAR_SIZE_MASK             = 0x1f,
	STATUS_PHYLINKUP             = 1U << 4,
	STATUS_DL_ACTIVE             = 1U << 5,
	STATUS_PCIE_PORT             = 1U << 7,
	HARD_DEBUG_CLKREQ_DEBUG_EN   = 1U << 1,
	HARD_DEBUG_REFCLK_OVRD_EN    = 1U << 16,
	HARD_DEBUG_REFCLK_OVRD_OUT   = 1U << 20,
	HARD_DEBUG_L1SS_ENABLE       = 1U << 21,
	HARD_DEBUG_SERDES_IDDQ       = 1U << 27,
	HARD_DEBUG_CLKREQ_MASK       = HARD_DEBUG_CLKREQ_DEBUG_EN | HARD_DEBUG_REFCLK_OVRD_EN |
	    HARD_DEBUG_REFCLK_OVRD_OUT | HARD_DEBUG_L1SS_ENABLE,
	PCIE_CTRL_PERSTB             = 1U << 2,
	MDIO_DATA_DONE               = 1U << 31,
	UBUS_CTRL_REPLY_ERR_DIS      = 1U << 13,
	UBUS_CTRL_REPLY_DECERR_DIS   = 1U << 19,
	AXI_REQFIFO_EN_QOS_PROPAGATION = 1U << 7,
	AXI_DIS_QOS_GATING_IN_MASTER = 1U << 11,
	AXI_EN_QOS_UPDATE_TIMING_FIX = 1U << 12,
	AXI_EN_RCLK_QOS_ARRAY_FIX    = 1U << 13,
	AXI_MAX_OUTSTANDING_MASK     = 0x3f,
	CTRL_1_EN_VDM_QOS_CONTROL    = 1U << 5,
	RGR1_SW_INIT_1_PERST         = 1U << 0,
	RGR1_SW_INIT_1_INIT          = 1U << 1,
	VENDOR_REG1_ENDIAN_BAR2_MASK = 0xc,
	UBUS_REMAP_ACCESS_EN         = 1U << 0,
};

enum {
	PCI_VENDOR_ID = 0x00, PCI_COMMAND = 0x04, PCI_CLASS_REVISION = 0x08,
	PCI_HEADER_TYPE = 0x0e, PCI_BAR0 = 0x10, PCI_PRIMARY_BUS = 0x18,
	PCI_MEMORY_BASE = 0x20, PCI_PREF_MEMORY_BASE = 0x24, PCI_PREF_BASE_UPPER = 0x28,
	PCI_PREF_LIMIT_UPPER = 0x2c, PCI_SUBSYSTEM_VENDOR_ID = 0x2c, PCI_INTERRUPT_PIN = 0x3d,
	PCI_BRIDGE_CONTROL = 0x3e,

	PCI_COMMAND_MEMORY = 0x2, PCI_COMMAND_MASTER = 0x4, PCI_COMMAND_INTX_DISABLE = 0x400,
	PCI_BAR_IO = 0x1, PCI_BAR_TYPE_64 = 0x4,
};
#define PCI_BAR_MEM_MASK 0xfffffff0U

#define WIN_PCI_DEFAULT   0xc0000000ULL
#define WIN_CPU_DEFAULT   0x600000000ULL
#define WIN_SIZE_DEFAULT  0x40000000ULL
#define DMA_SIZE_DEFAULT  0xc0000000ULL
#define MAX_BARS          6

class RPiPCIe : public IOService
{
	OSDeclareDefaultStructors(RPiPCIe);

	IOMemoryMap *regsMap;
	uintptr_t regs;
	IOLock *cfgLock;
	uint64_t winPCI, winCPU, winSize;
	uint64_t memNext;
	bool adopt;
	struct {
		uint64_t pci, cpu, size;
	} windows[4];
	unsigned nwindows;
	uint8_t lastBus;
	uint64_t barLo, barHi;
	struct Pending {
		uint8_t bus, dev, fn, pin;
		IODeviceMemory *bars[MAX_BARS];
		uint8_t barNumbers[MAX_BARS];
		int nbars;
	} pending[8];
	unsigned functions;
	OSArray *intControllers;
	OSData *dmaRanges;
	uint32_t intxSPI[4];

	uint32_t rd(uint32_t off) { return *(volatile uint32_t *)(regs + off); }
	void wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(regs + off) = v; }
	uintptr_t cfgAddress(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off);

	bool bringUp(IOService *provider);
	bool bringUp2712(IOService *provider);
	void bridgeReset2712(IOService *provider, bool assert);
	bool mdioWrite(uint8_t reg, uint16_t value);
	void parseRanges(IOService *provider);
	void parseInterruptMap(IOService *provider);
	void scanBus(uint8_t bus, uint8_t swizzle);
	void scanAdopted(uint8_t bus, uint8_t swizzle);
	void adoptWindows(IOService *provider);
	void publish(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t pin, IODeviceMemory **bars, int nbars,
	    const uint8_t *barNumbers);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;

	uint32_t configRead32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off);
	void configWrite32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val);
	uint16_t configRead16(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off);
	void configWrite16(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint16_t val);
	uint8_t configRead8(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off);
};

#define super IOService
OSDefineMetaClassAndStructors(RPiPCIe, IOService);
OSDefineMetaClassAndStructors(RPiPCIDevice, IOService);

#pragma mark Config space

uintptr_t
RPiPCIe::cfgAddress(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	if (bus == 0) {
		return dev == 0 && fn == 0 ? regs + off : 0;
	}
	wr(PCIE_EXT_CFG_INDEX, (uint32_t)bus << 20 | (uint32_t)dev << 15 | (uint32_t)fn << 12);
	return regs + PCIE_EXT_CFG_DATA + off;
}

uint32_t
RPiPCIe::configRead32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	IOLockLock(cfgLock);
	uintptr_t p = cfgAddress(bus, dev, fn, off & ~3);
	uint32_t v = p ? *(volatile uint32_t *)p : 0xffffffff;
	IOLockUnlock(cfgLock);
	return v;
}

void
RPiPCIe::configWrite32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val)
{
	IOLockLock(cfgLock);
	uintptr_t p = cfgAddress(bus, dev, fn, off & ~3);
	if (p) {
		*(volatile uint32_t *)p = val;
	}
	IOLockUnlock(cfgLock);
}

uint16_t
RPiPCIe::configRead16(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	return (uint16_t)(configRead32(bus, dev, fn, off) >> ((off & 2) * 8));
}

void
RPiPCIe::configWrite16(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint16_t val)
{
	uint32_t v = configRead32(bus, dev, fn, off);
	uint32_t shift = (off & 2) * 8;
	v = (v & ~(0xffffU << shift)) | (uint32_t)val << shift;
	configWrite32(bus, dev, fn, off, v);
}

uint8_t
RPiPCIe::configRead8(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	return (uint8_t)(configRead32(bus, dev, fn, off) >> ((off & 3) * 8));
}

#pragma mark Device tree

static uint32_t
cell(const uint8_t *p)
{
	return OSReadBigInt32(p, 0);
}

void
RPiPCIe::parseRanges(IOService *provider)
{
	winPCI = WIN_PCI_DEFAULT;
	winCPU = WIN_CPU_DEFAULT;
	winSize = WIN_SIZE_DEFAULT;
	OSData *r = OSDynamicCast(OSData, provider->getProperty("fdt,ranges"));
	if (r == NULL) {
		r = OSDynamicCast(OSData, provider->getProperty("ranges"));
	}
	nwindows = 0;
	for (unsigned off = 0; r != NULL && off + 28 <= r->getLength(); off += 28) {
		const uint8_t *p = (const uint8_t *)r->getBytesNoCopy() + off;
		uint64_t pci = (uint64_t)cell(p + 4) << 32 | cell(p + 8);
		uint64_t cpu = (uint64_t)cell(p + 12) << 32 | cell(p + 16);
		uint64_t size = (uint64_t)cell(p + 20) << 32 | cell(p + 24);
		if ((cell(p) & 0x02000000) && nwindows < sizeof(windows) / sizeof(windows[0])) {
			windows[nwindows].pci = pci;
			windows[nwindows].cpu = cpu;
			windows[nwindows].size = size;
			nwindows++;
		}
		if (off == 0 && (cell(p) & 0x03000000) == 0x02000000) {
			winPCI = pci;
			winCPU = cpu;
			winSize = size;
		}
	}
}

void
RPiPCIe::parseInterruptMap(IOService *provider)
{
	for (int i = 0; i < 4; i++) {
		intxSPI[i] = 32 + 143 + i;
	}
	OSData *m = OSDynamicCast(OSData, provider->getProperty("interrupt-map"));
	if (m != NULL && m->getLength() % 32 == 0) {
		const uint32_t *c = (const uint32_t *)m->getBytesNoCopy();
		for (unsigned e = 0; e < m->getLength() / 32; e++, c += 8) {
			uint32_t pin = c[3];
			if (pin >= 1 && pin <= 4 && c[5] == 0) {
				intxSPI[pin - 1] = 32 + c[6];
			}
		}
	}
	OSArray *ctl = OSDynamicCast(OSArray, provider->getProperty(gIOInterruptControllersKey));
	if (ctl != NULL && ctl->getCount() > 0) {
		const OSObject *gic = ctl->getObject(0);
		intControllers = OSArray::withObjects(&gic, 1);
	}
}

#pragma mark Bring-up

static int
ilog2_64(uint64_t v)
{
	return 63 - __builtin_clzll(v);
}

static uint32_t
encode_ibar_size(uint64_t size)
{
	int log2 = ilog2_64(size);
	if (log2 >= 12 && log2 <= 15) {
		return (uint32_t)(log2 - 12) + 0x1c;
	}
	if (log2 >= 16 && log2 <= 36) {
		return (uint32_t)(log2 - 15);
	}
	return 0;
}

bool
RPiPCIe::bringUp(IOService *provider)
{
	wr(PCIE_RGR1_SW_INIT_1, rd(PCIE_RGR1_SW_INIT_1) | RGR1_SW_INIT_1_INIT | RGR1_SW_INIT_1_PERST);
	IODelay(150);
	wr(PCIE_RGR1_SW_INIT_1, rd(PCIE_RGR1_SW_INIT_1) & ~RGR1_SW_INIT_1_INIT);
	wr(PCIE_MISC_HARD_PCIE_HARD_DEBUG, rd(PCIE_MISC_HARD_PCIE_HARD_DEBUG) & ~HARD_DEBUG_SERDES_IDDQ);
	IODelay(150);

	uint32_t ctrl = rd(PCIE_MISC_MISC_CTRL);
	ctrl |= MISC_CTRL_SCB_ACCESS_EN | MISC_CTRL_CFG_READ_UR_MODE;
	ctrl &= ~MISC_CTRL_MAX_BURST_MASK;
	wr(PCIE_MISC_MISC_CTRL, ctrl);

	uint64_t dmaSize = DMA_SIZE_DEFAULT;
	OSData *dr = OSDynamicCast(OSData, provider->getProperty("dma-ranges"));
	if (dr != NULL && dr->getLength() >= 28) {
		const uint8_t *p = (const uint8_t *)dr->getBytesNoCopy();
		dmaSize = (uint64_t)cell(p + 20) << 32 | cell(p + 24);
	}
	uint64_t bar2Size = 1ULL << ilog2_64(dmaSize);
	if (bar2Size < dmaSize) {
		bar2Size <<= 1;
	}
	wr(PCIE_MISC_RC_BAR2_CONFIG_LO, encode_ibar_size(bar2Size));
	wr(PCIE_MISC_RC_BAR2_CONFIG_HI, 0);
	ctrl = rd(PCIE_MISC_MISC_CTRL) & ~MISC_CTRL_SCB0_SIZE_MASK;
	ctrl |= (uint32_t)(ilog2_64(bar2Size) - 15) << MISC_CTRL_SCB0_SIZE_SHIFT;
	wr(PCIE_MISC_MISC_CTRL, ctrl);
	wr(PCIE_MISC_RC_BAR1_CONFIG_LO, rd(PCIE_MISC_RC_BAR1_CONFIG_LO) & ~RC_BAR_SIZE_MASK);
	wr(PCIE_MISC_RC_BAR3_CONFIG_LO, rd(PCIE_MISC_RC_BAR3_CONFIG_LO) & ~RC_BAR_SIZE_MASK);

	wr(PCIE_RGR1_SW_INIT_1, rd(PCIE_RGR1_SW_INIT_1) & ~RGR1_SW_INIT_1_PERST);
	IOSleep(100);
	uint32_t status = 0;
	for (int i = 0; i <= 20; i++) {
		status = rd(PCIE_MISC_PCIE_STATUS);
		if ((status & (STATUS_PHYLINKUP | STATUS_DL_ACTIVE)) == (STATUS_PHYLINKUP | STATUS_DL_ACTIVE)) {
			break;
		}
		IOSleep(5);
	}
	if ((status & (STATUS_PHYLINKUP | STATUS_DL_ACTIVE)) != (STATUS_PHYLINKUP | STATUS_DL_ACTIVE)) {
		IOLog("RPiPCIe: link down (status 0x%08x)\n", status);
		return false;
	}
	if (!(status & STATUS_PCIE_PORT)) {
		IOLog("RPiPCIe: controller in endpoint mode\n");
		return false;
	}

	wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO, (uint32_t)winPCI);
	wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI, (uint32_t)(winPCI >> 32));
	uint64_t baseMB = winCPU >> 20, limitMB = (winCPU + winSize - 1) >> 20;
	uint32_t bl = rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT);
	bl = (bl & ~0xfff0U) | (uint32_t)(baseMB & 0xfff) << 4;
	bl = (bl & ~0xfff00000U) | (uint32_t)(limitMB & 0xfff) << 20;
	wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT, bl);
	wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI,
	    (rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI) & ~0xffU) | (uint32_t)((baseMB >> 12) & 0xff));
	wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI,
	    (rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI) & ~0xffU) | (uint32_t)((limitMB >> 12) & 0xff));

	wr(PCIE_RC_CFG_PRIV1_ID_VAL3, (rd(PCIE_RC_CFG_PRIV1_ID_VAL3) & ~0xffffffU) | 0x060400);
	wr(PCIE_RC_CFG_VENDOR_SPECIFIC_REG1, rd(PCIE_RC_CFG_VENDOR_SPECIFIC_REG1) & ~VENDOR_REG1_ENDIAN_BAR2_MASK);
	wr(PCIE_MISC_HARD_PCIE_HARD_DEBUG, rd(PCIE_MISC_HARD_PCIE_HARD_DEBUG) | HARD_DEBUG_CLKREQ_DEBUG_EN);
	return true;
}

static IOMemoryMap *
map_node(IORegistryEntry *node)
{
	OSArray *mem = OSDynamicCast(OSArray, node->getProperty(gIODeviceMemoryKey));
	if (mem == NULL) {
		mem = IODTResolveAddressing(node, "reg", NULL);
	}
	IOMemoryDescriptor *range = mem != NULL ? OSDynamicCast(IOMemoryDescriptor, mem->getObject(0)) : NULL;
	return range != NULL ? range->map(kIOMapInhibitCache) : NULL;
}

static IORegistryEntry *
find_compatible(const char *compat)
{
	OSArray *nodes = RPiDTFindCompatible(compat);
	IORegistryEntry *node = nodes != NULL ? OSDynamicCast(IORegistryEntry, nodes->getObject(0)) : NULL;
	if (node != NULL) {
		node->retain();
	}
	OSSafeReleaseNULL(nodes);
	return node;
}

static int
string_index(OSData *list, const char *want)
{
	const char *p = list != NULL ? (const char *)list->getBytesNoCopy() : NULL;
	const char *end = p != NULL ? p + list->getLength() : NULL;
	for (int i = 0; p != NULL && p < end; i++) {
		size_t n = strnlen(p, (size_t)(end - p));
		if (strlen(want) == n && strncmp(p, want, n) == 0) {
			return i;
		}
		p += n + 1;
	}
	return -1;
}

static void
power_m2_slot(void)
{
	OSArray *nodes = RPiDTFindCompatible("brcm,brcmstb-gpio");
	IORegistryEntry *node;
	for (unsigned n = 0; nodes != NULL && (node = OSDynamicCast(IORegistryEntry, nodes->getObject(n))) != NULL; n++) {
		int line = string_index(OSDynamicCast(OSData, node->getProperty("gpio-line-names")), "M2_PWR_EN");
		if (line < 0) {
			continue;
		}
		IOMemoryMap *map = map_node(node);
		if (map == NULL) {
			IOLog("RPiPCIe: M2_PWR_EN is GPIO %d of %s, whose registers are not there\n", line, node->getName());
			break;
		}
		volatile uint32_t *bank = (volatile uint32_t *)(map->getVirtualAddress() + 0x20 * (unsigned)(line / 32));
		uint32_t bit = 1U << (line % 32);
		bool output = !(bank[2] & bit), high = (bank[1] & bit) != 0;
		IOLog("RPiPCIe: M2_PWR_EN (GPIO %d of %s): %s, %s%s\n", line, node->getName(), output ? "output" : "input",
		    high ? "high" : "low", output && high ? "" : "; driven high");
		if (!(output && high)) {
			bank[1] |= bit;
			bank[2] &= ~bit;
			IOSleep(100);
		}
		char key[12];
		snprintf(key, sizeof(key), "%d", line);
		OSDictionary *reserved = OSDictionary::withCapacity(1);
		OSString *consumer = OSString::withCString("M2_PWR_EN (kernel)");
		if (reserved != NULL && consumer != NULL) {
			reserved->setObject(key, consumer);
			node->setProperty("IOGPIOReservedLines", reserved);
		}
		OSSafeReleaseNULL(consumer);
		OSSafeReleaseNULL(reserved);
		map->release();
		break;
	}
	OSSafeReleaseNULL(nodes);
}

static int
reset_id(IOService *provider, uint32_t controller, const char *want)
{
	OSData *resets = OSDynamicCast(OSData, provider->getProperty("resets"));
	int index = string_index(OSDynamicCast(OSData, provider->getProperty("reset-names")), want);
	if (resets == NULL || index < 0) {
		return -1;
	}
	const uint8_t *r = (const uint8_t *)resets->getBytesNoCopy();
	unsigned ncells = resets->getLength() / 4;
	for (unsigned i = 0, n = 0; i < ncells; n++) {
		bool ours = cell(r + 4 * i++) == controller;
		int id = ours && i < ncells ? (int)cell(r + 4 * i++) : -1;
		if ((int)n == index) {
			return id;
		}
	}
	return -1;
}

void
RPiPCIe::bridgeReset2712(IOService *provider, bool assert)
{
	OSArray *ctls = RPiDTFindCompatible("brcm,brcmstb-reset");
	IORegistryEntry *ctl = NULL;
	int id = -1;
	for (unsigned n = 0; ctls != NULL && id < 0 && (ctl = OSDynamicCast(IORegistryEntry, ctls->getObject(n))) != NULL; n++) {
		OSData *ph = OSDynamicCast(OSData, ctl->getProperty("AAPL,phandle"));
		id = ph != NULL && ph->getLength() >= 4 ? reset_id(provider, *(const uint32_t *)ph->getBytesNoCopy(), "bridge") : -1;
	}
	IOMemoryMap *map = id >= 0 ? map_node(ctl) : NULL;
	if (map != NULL) {
		volatile uint32_t *bank = (volatile uint32_t *)(map->getVirtualAddress() + 0x18 * (unsigned)(id / 32));
		if (assert) {
			IOLog("RPiPCIe: %s: bridge reset (line %d, status 0x%08x)\n", provider->getName(), id, bank[2]);
		}
		bank[assert ? 0 : 1] = 1U << (id % 32);
		map->release();
	} else {
		IOLog("RPiPCIe: %s: no bridge reset line; RGR1_SW_INIT_1 instead\n", provider->getName());
		uint32_t v = rd(PCIE_RGR1_SW_INIT_1);
		wr(PCIE_RGR1_SW_INIT_1, assert ? v | RGR1_SW_INIT_1_INIT : v & ~RGR1_SW_INIT_1_INIT);
	}
	OSSafeReleaseNULL(ctls);
	IODelay(200);
}

bool
RPiPCIe::mdioWrite(uint8_t reg, uint16_t value)
{
	wr(PCIE_RC_DL_MDIO_ADDR, reg);
	(void)rd(PCIE_RC_DL_MDIO_ADDR);
	wr(PCIE_RC_DL_MDIO_WR_DATA, MDIO_DATA_DONE | value);
	for (int i = 0; i < 20; i++) {
		if (!(rd(PCIE_RC_DL_MDIO_WR_DATA) & MDIO_DATA_DONE)) {
			return true;
		}
		IODelay(10);
	}
	return false;
}

bool
RPiPCIe::bringUp2712(IOService *provider)
{
	const char *name = provider->getName();
	power_m2_slot();
	wr(PCIE_MISC_PCIE_CTRL, rd(PCIE_MISC_PCIE_CTRL) & ~PCIE_CTRL_PERSTB);
	bridgeReset2712(provider, true);
	bridgeReset2712(provider, false);

	IORegistryEntry *rescal = find_compatible("brcm,bcm7216-pcie-sata-rescal");
	IOMemoryMap *rescalMap = rescal != NULL ? map_node(rescal) : NULL;
	uint32_t runRescal = 0;
	PE_parse_boot_argn("pcie_rescal", &runRescal, sizeof(runRescal));
	if (rescalMap != NULL) {
		volatile uint32_t *rc = (volatile uint32_t *)rescalMap->getVirtualAddress();
		if ((rc[2] & 1) || !runRescal) {
			IOLog("RPiPCIe: %s: rescal status 0x%x, start 0x%x; not run\n", name, rc[2], rc[0]);
		} else {
			rc[0] |= 1;
			int i;
			for (i = 0; i < 10 && !(rc[2] & 1); i++) {
				IODelay(100);
			}
			rc[0] &= ~1U;
			IOLog("RPiPCIe: %s: rescal %s\n", name, i < 10 ? "done" : "timed out");
		}
		rescalMap->release();
	} else {
		IOLog("RPiPCIe: %s: no rescal\n", name);
	}
	OSSafeReleaseNULL(rescal);

	wr(PCIE_MISC_HARD_PCIE_HARD_DEBUG_2712, rd(PCIE_MISC_HARD_PCIE_HARD_DEBUG_2712) & ~HARD_DEBUG_SERDES_IDDQ);
	IODelay(200);

	uint32_t ctrl = rd(PCIE_MISC_MISC_CTRL);
	ctrl |= MISC_CTRL_SCB_ACCESS_EN | MISC_CTRL_CFG_READ_UR_MODE | MISC_CTRL_RCB_MPS_MODE | MISC_CTRL_RCB_64B_MODE;
	ctrl = (ctrl & ~MISC_CTRL_MAX_BURST_MASK) | 2U << MISC_CTRL_MAX_BURST_SHIFT;
	wr(PCIE_MISC_MISC_CTRL, ctrl);
	if (!(rd(PCIE_MISC_PCIE_STATUS) & STATUS_PCIE_PORT)) {
		IOLog("RPiPCIe: %s: controller in endpoint mode\n", name);
		return false;
	}

	uint32_t aspm = provider->getProperty("aspm-no-l0s") != NULL ? 0x2 : 0x3;
	wr(PCIE_RC_CFG_PRIV1_LINK_CAPABILITY, (rd(PCIE_RC_CFG_PRIV1_LINK_CAPABILITY) & ~0xc00U) | aspm << 10);
	wr(PCIE_RC_CFG_PRIV1_ID_VAL3, (rd(PCIE_RC_CFG_PRIV1_ID_VAL3) & ~0xffffffU) | 0x060400);
	wr(PCIE_RC_CFG_VENDOR_SPECIFIC_REG1, rd(PCIE_RC_CFG_VENDOR_SPECIFIC_REG1) & ~VENDOR_REG1_ENDIAN_BAR2_MASK);

	static const uint8_t pllRegs[] = { 0x16, 0x17, 0x18, 0x19, 0x1b, 0x1c, 0x1e };
	static const uint16_t pllData[] = { 0x50b9, 0xbda1, 0x0094, 0x97b4, 0x5030, 0x5030, 0x0007 };
	bool mdio = mdioWrite(0x1f, 0x1600);
	for (unsigned i = 0; mdio && i < sizeof(pllRegs); i++) {
		mdio = mdioWrite(pllRegs[i], pllData[i]);
	}
	if (!mdio) {
		IOLog("RPiPCIe: %s: the PHY's MDIO does not answer\n", name);
	}
	IODelay(200);
	wr(PCIE_RC_PL_PHY_CTL_15, (rd(PCIE_RC_PL_PHY_CTL_15) & ~0xffU) | 0x12);
	wr(PCIE_MISC_UBUS_CTRL, rd(PCIE_MISC_UBUS_CTRL) | UBUS_CTRL_REPLY_ERR_DIS | UBUS_CTRL_REPLY_DECERR_DIS);
	wr(PCIE_MISC_AXI_READ_ERROR_DATA, 0xffffffff);
	wr(PCIE_MISC_UBUS_TIMEOUT, 0xB2D0000);
	wr(PCIE_MISC_RC_CONFIG_RETRY_TIMEOUT, 0xABA0000);
	uint32_t axi = rd(PCIE_MISC_AXI_INTF_CTRL) & ~AXI_REQFIFO_EN_QOS_PROPAGATION;
	axi |= AXI_EN_RCLK_QOS_ARRAY_FIX | AXI_EN_QOS_UPDATE_TIMING_FIX | AXI_DIS_QOS_GATING_IN_MASTER;
	wr(PCIE_MISC_AXI_INTF_CTRL, axi);
	axi = rd(PCIE_MISC_AXI_INTF_CTRL);
	if (!(axi & AXI_EN_QOS_UPDATE_TIMING_FIX)) {
		wr(PCIE_MISC_AXI_INTF_CTRL, (axi & ~AXI_MAX_OUTSTANDING_MASK) | 15);
	}
	wr(PCIE_MISC_CTRL_1, rd(PCIE_MISC_CTRL_1) & ~CTRL_1_EN_VDM_QOS_CONTROL);

	OSData *speed = OSDynamicCast(OSData, provider->getProperty("max-link-speed"));
	uint32_t gen = speed != NULL && speed->getLength() >= 4 ? cell((const uint8_t *)speed->getBytesNoCopy()) : 0;
	if (gen >= 1 && gen <= 3) {
		wr(PCIE_RC_CFG_PRIV1_LINK_CAPABILITY, (rd(PCIE_RC_CFG_PRIV1_LINK_CAPABILITY) & ~0xfU) | gen);
		uint32_t lnkctl2 = rd(PCIE_RC_CFG_PCIE_CAP + 0x30);
		wr(PCIE_RC_CFG_PCIE_CAP + 0x30, (lnkctl2 & ~0xfU) | gen);
	}

	wr(PCIE_MISC_HARD_PCIE_HARD_DEBUG_2712, rd(PCIE_MISC_HARD_PCIE_HARD_DEBUG_2712) & ~HARD_DEBUG_CLKREQ_MASK);
	wr(PCIE_MISC_PCIE_CTRL, rd(PCIE_MISC_PCIE_CTRL) | PCIE_CTRL_PERSTB);
	IOSleep(100);
	uint32_t status = 0;
	for (int i = 0; i <= 20; i++) {
		status = rd(PCIE_MISC_PCIE_STATUS);
		if ((status & (STATUS_PHYLINKUP | STATUS_DL_ACTIVE)) == (STATUS_PHYLINKUP | STATUS_DL_ACTIVE)) {
			break;
		}
		IOSleep(5);
	}
	if ((status & (STATUS_PHYLINKUP | STATUS_DL_ACTIVE)) != (STATUS_PHYLINKUP | STATUS_DL_ACTIVE)) {
		IOLog("RPiPCIe: %s: link still down (status 0x%08x, PCIE_CTRL 0x%08x, HARD_DEBUG 0x%08x, "
		    "LTSSM history %08x %08x %08x %08x)\n", name, status, rd(PCIE_MISC_PCIE_CTRL),
		    rd(PCIE_MISC_HARD_PCIE_HARD_DEBUG_2712), rd(PCIE_RC_PL_LTSSM_HIST_0), rd(PCIE_RC_PL_LTSSM_HIST_0 + 4),
		    rd(PCIE_RC_PL_LTSSM_HIST_0 + 8), rd(PCIE_RC_PL_LTSSM_HIST_0 + 12));
		return false;
	}

	OSData *mode = OSDynamicCast(OSData, provider->getProperty("brcm,clkreq-mode"));
	const char *m = mode != NULL ? (const char *)mode->getBytesNoCopy() : "default";
	bool safe = strncmp(m, "default", 8) != 0 && strncmp(m, "no-l1ss", 8) != 0;
	uint32_t hd = rd(PCIE_MISC_HARD_PCIE_HARD_DEBUG_2712) & ~HARD_DEBUG_CLKREQ_MASK;
	if (strncmp(m, "default", 8) == 0) {
		hd |= HARD_DEBUG_L1SS_ENABLE;
	} else {
		hd |= safe ? HARD_DEBUG_REFCLK_OVRD_OUT | HARD_DEBUG_REFCLK_OVRD_EN : HARD_DEBUG_CLKREQ_DEBUG_EN;
		wr(PCIE_RC_CFG_PRIV1_ROOT_CAP, (rd(PCIE_RC_CFG_PRIV1_ROOT_CAP) & ~0xf8U) | 2U << 3);
	}
	wr(PCIE_MISC_HARD_PCIE_HARD_DEBUG_2712, hd);

	uint16_t lnksta = (uint16_t)(rd(PCIE_RC_CFG_PCIE_CAP + 0x10) >> 16);
	IOLog("RPiPCIe: %s: link up, Gen %u x%u (status 0x%08x), clkreq-mode %s\n", name, lnksta & 0xf,
	    (lnksta >> 4) & 0x3f, status, safe ? "safe" : m);
	return true;
}

#pragma mark Enumeration

void
RPiPCIe::publish(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t pin, IODeviceMemory **bars, int nbars,
    const uint8_t *barNumbers)
{
	RPiPCIDevice *nub = OSTypeAlloc(RPiPCIDevice);
	if (nub == NULL || !nub->init()) {
		OSSafeReleaseNULL(nub);
		return;
	}
	uint32_t id = configRead32(bus, dev, fn, PCI_VENDOR_ID);
	uint32_t cls = configRead32(bus, dev, fn, PCI_CLASS_REVISION);
	uint32_t sub = configRead32(bus, dev, fn, PCI_SUBSYSTEM_VENDOR_ID);
	nub->setProperty("vendor-id", id & 0xffff, 32);
	nub->setProperty("device-id", id >> 16, 32);
	nub->setProperty("class-code", cls >> 8, 32);
	nub->setProperty("revision-id", cls & 0xff, 32);
	nub->setProperty("subsystem-vendor-id", sub & 0xffff, 32);
	nub->setProperty("subsystem-id", sub >> 16, 32);
	uint32_t loc = (uint32_t)bus << 16 | (uint32_t)dev << 8 | fn;
	nub->setProperty("pci-location", loc, 32);
	char name[32];
	snprintf(name, sizeof(name), "pci%x,%x", id & 0xffff, id >> 16);
	nub->setName(name);
	char where[16];
	snprintf(where, sizeof(where), "%x,%x,%x", bus, dev, fn);
	nub->setLocation(where);

	OSArray *mem = OSArray::withCapacity((unsigned)nbars);
	for (int i = 0; i < nbars && mem != NULL; i++) {
		mem->setObject(bars[i]);
	}
	if (mem != NULL) {
		nub->setDeviceMemory(mem);
		mem->release();
	}
	OSData *numbers = OSData::withBytes(barNumbers, (unsigned)nbars);
	if (numbers != NULL) {
		nub->setProperty("bar-numbers", numbers);
		numbers->release();
	}

	if (pin >= 1 && pin <= 4 && intControllers != NULL) {
		uint32_t intid = intxSPI[pin - 1];
		OSData *spec = OSData::withBytes(&intid, sizeof(intid));
		OSArray *specs = spec ? OSArray::withObjects((const OSObject **)&spec, 1) : NULL;
		if (specs != NULL) {
			nub->setProperty(gIOInterruptControllersKey, intControllers);
			nub->setProperty(gIOInterruptSpecifiersKey, specs);
		}
		OSSafeReleaseNULL(specs);
		OSSafeReleaseNULL(spec);
	}
	nub->pcie = this;
	nub->location = loc;
	if (dmaRanges != NULL) {
		nub->setProperty("pieboot,dma-ranges", dmaRanges);
	}

	if (id == 0x34831106) {
		uint32_t version = configRead32(bus, dev, fn, 0x50);
		if (version == 0) {
			uint32_t addr = (uint32_t)bus << 20 | (uint32_t)dev << 15 | (uint32_t)fn << 12;
			IOReturn fw = RPiFirmwareProperty(0x00030058, &addr, sizeof(addr));
			for (int ms = 0; ms < 100 && version == 0; ms++) {
				IODelay(1000);
				version = configRead32(bus, dev, fn, 0x50);
			}
			if (fw != kIOReturnSuccess) {
				IOLog("RPiPCIe: VL805 firmware load failed (0x%x)\n", fw);
			}
			IOLog("RPiPCIe: VL805 firmware notify: 0x%x, answer 0x%08x\n", fw, addr);
		}
		IOLog("RPiPCIe: VL805 firmware version %08x\n", version);
		if (version == 0) {
			IOLog("RPiPCIe: VL805 has no firmware; USB is off\n");
			nub->release();
			return;
		}
	}
	IOLog("RPiPCIe: %02x:%02x.%x %04x:%04x class %06x, %d BAR(s), INT%c\n", bus, dev, fn,
	    id & 0xffff, id >> 16, cls >> 8, nbars, pin ? 'A' + pin - 1 : '-');
	if (nub->attach(this)) {
		nub->registerService();
	}
	nub->release();
}

void
RPiPCIe::scanBus(uint8_t bus, uint8_t swizzle)
{
	for (uint8_t dev = 0; dev < 32; dev++) {
		if (bus <= 1 && dev > 0) {
			break;
		}
		for (uint8_t fn = 0; fn < 8; fn++) {
			uint32_t id = configRead32(bus, dev, fn, PCI_VENDOR_ID);
			if ((id & 0xffff) == 0xffff || (id & 0xffff) == 0) {
				if (fn == 0) {
					break;
				}
				continue;
			}
			uint8_t header = configRead8(bus, dev, fn, PCI_HEADER_TYPE);
			uint8_t pin = configRead8(bus, dev, fn, PCI_INTERRUPT_PIN);
			uint8_t rootPin = pin ? (uint8_t)((pin - 1 + swizzle + dev) % 4 + 1) : 0;

			if ((header & 0x7f) == 1) {
				uint8_t secondary = ++lastBus;
				configWrite32(bus, dev, fn, PCI_PRIMARY_BUS,
				    (configRead32(bus, dev, fn, PCI_PRIMARY_BUS) & 0xff000000U) |
				    0xffU << 16 | (uint32_t)secondary << 8 | bus);
				memNext = (memNext + 0xfffff) & ~0xfffffULL;
				uint64_t start = memNext;
				scanBus(secondary, bus == 0 ? 0 : (uint8_t)((swizzle + dev) % 4));
				configWrite32(bus, dev, fn, PCI_PRIMARY_BUS,
				    (configRead32(bus, dev, fn, PCI_PRIMARY_BUS) & 0xff00ffffU) | (uint32_t)lastBus << 16);
				uint64_t end = (memNext + 0xfffff) & ~0xfffffULL;
				memNext = end;
				if (end > start) {
					configWrite32(bus, dev, fn, PCI_MEMORY_BASE,
					    (uint32_t)((end - 1) >> 16 & 0xfff0) << 16 | (uint32_t)(start >> 16 & 0xfff0));
				} else {
					configWrite32(bus, dev, fn, PCI_MEMORY_BASE, 0x0000fff0);
				}
				configWrite32(bus, dev, fn, PCI_PREF_MEMORY_BASE, 0x0000fff0);
				configWrite32(bus, dev, fn, PCI_PREF_BASE_UPPER, 0);
				configWrite32(bus, dev, fn, PCI_PREF_LIMIT_UPPER, 0);
				configWrite16(bus, dev, fn, PCI_COMMAND,
				    (uint16_t)(configRead16(bus, dev, fn, PCI_COMMAND) | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER));
			} else if ((header & 0x7f) == 0) {
				IODeviceMemory *bars[MAX_BARS];
				uint8_t barNumbers[MAX_BARS];
				int nbars = 0;
				configWrite16(bus, dev, fn, PCI_COMMAND,
				    (uint16_t)(configRead16(bus, dev, fn, PCI_COMMAND) & ~(PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)));
				for (int b = 0; b < MAX_BARS; b++) {
					uint16_t off = (uint16_t)(PCI_BAR0 + 4 * b);
					uint32_t orig = configRead32(bus, dev, fn, off);
					configWrite32(bus, dev, fn, off, 0xffffffff);
					uint32_t mask = configRead32(bus, dev, fn, off);
					bool is64 = !(orig & PCI_BAR_IO) && (orig & 0x6) == PCI_BAR_TYPE_64;
					if (is64) {
						configWrite32(bus, dev, fn, off + 4, 0);
					}
					if (mask == 0 || mask == 0xffffffff || (orig & PCI_BAR_IO)) {
						configWrite32(bus, dev, fn, off, 0);
						if (is64) {
							b++;
						}
						continue;
					}
					uint32_t sizeMask = mask & PCI_BAR_MEM_MASK;
					uint64_t size = (uint64_t)(uint32_t)~sizeMask + 1;
					uint64_t addr = (memNext + size - 1) & ~(size - 1);
					if (size == 0 || addr + size > winPCI + winSize) {
						IOLog("RPiPCIe: %02x:%02x.%x BAR%d does not fit\n", bus, dev, fn, b);
						configWrite32(bus, dev, fn, off, 0);
						if (is64) {
							b++;
						}
						continue;
					}
					memNext = addr + size;
					configWrite32(bus, dev, fn, off, (uint32_t)addr | (orig & 0xf));
					bars[nbars] = IODeviceMemory::withRange(winCPU + (addr - winPCI), size);
					if (bars[nbars] != NULL) {
						barNumbers[nbars++] = (uint8_t)b;
					}
					if (is64) {
						b++;
					}
				}
				configWrite16(bus, dev, fn, PCI_COMMAND,
				    (uint16_t)((configRead16(bus, dev, fn, PCI_COMMAND) & ~PCI_COMMAND_INTX_DISABLE) |
				    PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER));
				publish(bus, dev, fn, rootPin, bars, nbars, barNumbers);
				for (int i = 0; i < nbars; i++) {
					bars[i]->release();
				}
			}
			if (fn == 0 && !(header & 0x80)) {
				break;
			}
		}
	}
}

void
RPiPCIe::adoptWindows(IOService *provider)
{
	const char *name = provider->getName();
	for (unsigned w = 0; w < nwindows && w < 4; w++) {
		uint64_t pci = (uint64_t)rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI + 8 * w) << 32 |
		    rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO + 8 * w);
		uint32_t baseLimit = rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT + 4 * w);
		uint64_t baseMB = (uint64_t)(rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI + 8 * w) & 0xff) << 12 |
		    (baseLimit >> 4 & 0xfff);
		uint64_t limitMB = (uint64_t)(rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI + 8 * w) & 0xff) << 12 |
		    (baseLimit >> 20 & 0xfff);
		uint64_t wantBase = windows[w].cpu >> 20, wantLimit = (windows[w].cpu + windows[w].size - 1) >> 20;
		bool ok = pci == windows[w].pci && baseMB == wantBase && limitMB == wantLimit;
		IOLog("RPiPCIe: %s: outbound window %u: CPU 0x%llx-0x%llx MB = PCI 0x%llx%s\n", name, w,
		    baseMB, limitMB, pci, ok ? "" : ", set to the device tree's");
		if (!ok) {
			wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO + 8 * w, (uint32_t)windows[w].pci);
			wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI + 8 * w, (uint32_t)(windows[w].pci >> 32));
			wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT + 4 * w,
			    (baseLimit & 0x000f000f) | (uint32_t)(wantBase & 0xfff) << 4 | (uint32_t)(wantLimit & 0xfff) << 20);
			uint32_t baseHi = rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI + 8 * w);
			wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI + 8 * w, (baseHi & ~0xffU) | (uint32_t)(wantBase >> 12 & 0xff));
			uint32_t limitHi = rd(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI + 8 * w);
			wr(PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI + 8 * w, (limitHi & ~0xffU) | (uint32_t)(wantLimit >> 12 & 0xff));
		}
	}
	IOLog("RPiPCIe: %s: MISC_CTRL 0x%08x, inbound RC_BAR1 0x%08x:%08x RC_BAR2 0x%08x:%08x RC_BAR3 0x%08x:%08x\n",
	    name, rd(PCIE_MISC_MISC_CTRL), rd(PCIE_MISC_RC_BAR1_CONFIG_HI), rd(PCIE_MISC_RC_BAR1_CONFIG_LO),
	    rd(PCIE_MISC_RC_BAR2_CONFIG_HI), rd(PCIE_MISC_RC_BAR2_CONFIG_LO),
	    rd(PCIE_MISC_RC_BAR3_CONFIG_HI), rd(PCIE_MISC_RC_BAR3_CONFIG_LO));
	IOLog("RPiPCIe: %s: inbound remap 1 0x%08x:%08x 2 0x%08x:%08x 3 0x%08x:%08x\n", name,
	    rd(PCIE_MISC_UBUS_BAR1_CONFIG_REMAP + 4), rd(PCIE_MISC_UBUS_BAR1_CONFIG_REMAP),
	    rd(PCIE_MISC_UBUS_BAR1_CONFIG_REMAP + 12), rd(PCIE_MISC_UBUS_BAR1_CONFIG_REMAP + 8),
	    rd(PCIE_MISC_UBUS_BAR1_CONFIG_REMAP + 20), rd(PCIE_MISC_UBUS_BAR1_CONFIG_REMAP + 16));

	OSData *dr = OSDynamicCast(OSData, provider->getProperty("dma-ranges"));
	unsigned n = 0;
	for (unsigned off = 0; dr != NULL && off + 28 <= dr->getLength() && n < 3; off += 28) {
		const uint8_t *p = (const uint8_t *)dr->getBytesNoCopy() + off;
		uint64_t pci = (uint64_t)cell(p + 4) << 32 | cell(p + 8);
		uint64_t cpu = (uint64_t)cell(p + 12) << 32 | cell(p + 16);
		uint64_t size = (uint64_t)cell(p + 20) << 32 | cell(p + 24);
		uint32_t encoded = (size & (size - 1)) == 0 ? encode_ibar_size(size) : 0;
		if (encoded == 0 || (pci & (size - 1)) != 0) {
			IOLog("RPiPCIe: %s: inbound range PCI 0x%llx, 0x%llx bytes: no window can hold it\n", name, pci, size);
			continue;
		}
		uint32_t bar = PCIE_MISC_RC_BAR1_CONFIG_LO + 8 * n, remap = PCIE_MISC_UBUS_BAR1_CONFIG_REMAP + 8 * n;
		n++;
		wr(bar, (uint32_t)pci | encoded);
		wr(bar + 4, (uint32_t)(pci >> 32));
		wr(remap, ((uint32_t)cpu & ~0xfffU) | UBUS_REMAP_ACCESS_EN);
		wr(remap + 4, (uint32_t)(cpu >> 32));
		IOLog("RPiPCIe: %s: inbound window %u: PCI 0x%llx = CPU 0x%llx, 0x%llx bytes\n", name, n, pci, cpu, size);
		if (pci != cpu) {
			uint64_t triple[3] = { cpu, pci, size };
			if (dmaRanges == NULL) {
				dmaRanges = OSData::withCapacity(3 * sizeof(triple));
			}
			if (dmaRanges != NULL) {
				dmaRanges->appendBytes(triple, sizeof(triple));
			}
		}
	}
}

void
RPiPCIe::scanAdopted(uint8_t bus, uint8_t swizzle)
{
	for (uint8_t dev = 0; dev < 32; dev++) {
		if (bus <= 1 && dev > 0) {
			break;
		}
		for (uint8_t fn = 0; fn < 8; fn++) {
			uint32_t id = configRead32(bus, dev, fn, PCI_VENDOR_ID);
			if ((id & 0xffff) == 0xffff || (id & 0xffff) == 0) {
				if (fn == 0) {
					break;
				}
				continue;
			}
			uint8_t header = configRead8(bus, dev, fn, PCI_HEADER_TYPE);
			uint8_t pin = configRead8(bus, dev, fn, PCI_INTERRUPT_PIN);
			uint8_t rootPin = pin ? (uint8_t)((pin - 1 + swizzle + dev) % 4 + 1) : 0;
			if ((header & 0x7f) == 1) {
				uint32_t busReg = configRead32(bus, dev, fn, PCI_PRIMARY_BUS);
				uint8_t secondary = (uint8_t)(busReg >> 8);
				uint8_t subordinate = (uint8_t)(busReg >> 16);
				bool numbered = secondary > bus && subordinate >= secondary;
				if (numbered) {
					if (subordinate > lastBus) {
						lastBus = subordinate;
					}
				} else {
					secondary = ++lastBus;
					configWrite32(bus, dev, fn, PCI_PRIMARY_BUS,
					    (busReg & 0xff000000U) | 0xffU << 16 | (uint32_t)secondary << 8 | bus);
				}
				uint32_t window = configRead32(bus, dev, fn, PCI_MEMORY_BASE);
				IOLog("RPiPCIe: %02x:%02x.%x %04x:%04x bridge to bus %u (%s), command 0x%x, window 0x%08x\n",
				    bus, dev, fn, id & 0xffff, id >> 16, secondary, numbered ? "the firmware's" : "numbered here",
				    configRead16(bus, dev, fn, PCI_COMMAND), window);

				uint64_t outerLo = barLo, outerHi = barHi;
				barLo = ~0ULL;
				barHi = 0;
				scanAdopted(secondary, bus == 0 ? 0 : (uint8_t)((swizzle + dev) % 4));
				uint64_t lo = barLo, hi = barHi;
				barLo = lo < outerLo ? lo : outerLo;
				barHi = hi > outerHi ? hi : outerHi;
				if (!numbered) {
					configWrite32(bus, dev, fn, PCI_PRIMARY_BUS,
					    (configRead32(bus, dev, fn, PCI_PRIMARY_BUS) & 0xff00ffffU) | (uint32_t)lastBus << 16);
				}
				if (hi > lo) {
					uint64_t base = (uint64_t)(window & 0xfff0) << 16;
					uint64_t limit = (uint64_t)(window >> 16 & 0xfff0) << 16 | 0xfffff;
					if (base > limit || lo < base || hi - 1 > limit) {
						lo &= ~0xfffffULL;
						window = (uint32_t)((hi - 1) >> 16 & 0xfff0) << 16 | (uint32_t)(lo >> 16 & 0xfff0);
						configWrite32(bus, dev, fn, PCI_MEMORY_BASE, window);
						IOLog("RPiPCIe: %02x:%02x.%x window set to 0x%08x\n", bus, dev, fn, window);
					}
				}
				configWrite16(bus, dev, fn, PCI_COMMAND,
				    (uint16_t)(configRead16(bus, dev, fn, PCI_COMMAND) | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER));
			} else if ((header & 0x7f) == 0) {
				struct {
					uint16_t off;
					uint8_t number;
					bool is64;
					uint32_t flags;
					uint64_t pci, size;
				} found[MAX_BARS];
				int nfound = 0;
				uint16_t cmd = configRead16(bus, dev, fn, PCI_COMMAND);
				configWrite16(bus, dev, fn, PCI_COMMAND, (uint16_t)(cmd & ~(PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)));
				bool assigned = false;
				for (int b = 0; b < MAX_BARS; b++) {
					uint16_t off = (uint16_t)(PCI_BAR0 + 4 * b);
					uint32_t orig = configRead32(bus, dev, fn, off);
					bool is64 = !(orig & PCI_BAR_IO) && (orig & 0x6) == PCI_BAR_TYPE_64;
					uint32_t origHi = is64 ? configRead32(bus, dev, fn, off + 4) : 0;
					configWrite32(bus, dev, fn, off, 0xffffffff);
					uint32_t mask = configRead32(bus, dev, fn, off);
					configWrite32(bus, dev, fn, off, orig);
					int number = b;
					if (is64) {
						b++;
					}
					if (mask == 0 || mask == 0xffffffff || (orig & PCI_BAR_IO)) {
						continue;
					}
					found[nfound].off = off;
					found[nfound].number = (uint8_t)number;
					found[nfound].is64 = is64;
					found[nfound].flags = orig & 0xf;
					found[nfound].pci = (uint64_t)origHi << 32 | (orig & PCI_BAR_MEM_MASK);
					found[nfound].size = (uint64_t)(uint32_t)~(mask & PCI_BAR_MEM_MASK) + 1;
					assigned |= found[nfound].pci != 0;
					nfound++;
				}
				if (nfound > 0 && nwindows > 0) {
					int shown = 0;
					for (int i = 0; i < nfound; i++) {
						if (found[i].number == 1) {
							shown = i;
						}
					}
					uint8_t wasNumber = found[shown].number;
					uint64_t was = found[shown].pci;
					for (int i = 0; i < nfound; i++) {
						found[i].pci = 0;
					}
					uint64_t next = windows[0].pci;
					for (int n = 0; n < nfound; n++) {
						int big = -1;
						for (int i = 0; i < nfound; i++) {
							if (found[i].pci == 0 && (big < 0 || found[i].size > found[big].size)) {
								big = i;
							}
						}
						if (big < 0) {
							break;
						}
						uint64_t addr = (next + found[big].size - 1) & ~(found[big].size - 1);
						found[big].pci = addr == 0 ? 1 : addr;
						next = addr + found[big].size;
						configWrite32(bus, dev, fn, found[big].off, (uint32_t)addr | found[big].flags);
						if (found[big].is64) {
							configWrite32(bus, dev, fn, found[big].off + 4, (uint32_t)(addr >> 32));
						}
					}
					for (int i = 0; i < nfound; i++) {
						found[i].pci &= ~1ULL;
					}
					IOLog("RPiPCIe: %02x:%02x.%x BARs placed from PCI 0x%llx (BAR%u was at 0x%llx%s)\n", bus, dev, fn,
					    windows[0].pci, wasNumber, was, assigned ? "" : ", none set");
				}
				IODeviceMemory *bars[MAX_BARS];
				uint8_t barNumbers[MAX_BARS];
				int nbars = 0;
				for (int i = 0; i < nfound; i++) {
					uint64_t pci = found[i].pci, size = found[i].size;
					uint64_t cpu = 0;
					for (unsigned w = 0; w < nwindows; w++) {
						if (pci >= windows[w].pci && pci + size <= windows[w].pci + windows[w].size) {
							cpu = windows[w].cpu + (pci - windows[w].pci);
						}
					}
					if (cpu == 0) {
						IOLog("RPiPCIe: %02x:%02x.%x BAR%d at PCI 0x%llx is in no window\n", bus, dev, fn,
						    found[i].number, pci);
						continue;
					}
					if (pci + size <= 0x100000000ULL) {
						barLo = pci < barLo ? pci : barLo;
						barHi = pci + size > barHi ? pci + size : barHi;
					}
					bars[nbars] = IODeviceMemory::withRange(cpu, size);
					if (bars[nbars] != NULL) {
						barNumbers[nbars++] = found[i].number;
					}
				}
				configWrite16(bus, dev, fn, PCI_COMMAND,
				    (uint16_t)(cmd | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER));
				if (functions < sizeof(pending) / sizeof(pending[0])) {
					Pending *f = &pending[functions++];
					f->bus = bus;
					f->dev = dev;
					f->fn = fn;
					f->pin = rootPin;
					f->nbars = nbars;
					for (int i = 0; i < nbars; i++) {
						f->bars[i] = bars[i];
						f->barNumbers[i] = barNumbers[i];
					}
				} else {
					for (int i = 0; i < nbars; i++) {
						bars[i]->release();
					}
				}
			}
			if (fn == 0 && !(header & 0x80)) {
				break;
			}
		}
	}
}

bool
RPiPCIe::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	regsMap = provider->mapDeviceMemoryWithIndex(0);
	if (regsMap == NULL) {
		IOLog("RPiPCIe: cannot map the registers\n");
		return false;
	}
	regs = (uintptr_t)regsMap->getVirtualAddress();
	cfgLock = IOLockAlloc();
	if (cfgLock == NULL) {
		return false;
	}
	parseRanges(provider);
	parseInterruptMap(provider);
	uint32_t rev = rd(PCIE_MISC_REVISION);
	adopt = IODTMatchNubWithKeys(provider, "'brcm,bcm2712-pcie'");
	if (adopt) {
		uint32_t status = rd(PCIE_MISC_PCIE_STATUS);
		bool firmware = (status & (STATUS_PHYLINKUP | STATUS_DL_ACTIVE)) == (STATUS_PHYLINKUP | STATUS_DL_ACTIVE);
		if (!firmware) {
			IOLog("RPiPCIe: %s: link down (status 0x%08x); bringing it up\n", provider->getName(), status);
			if (!bringUp2712(provider)) {
				return false;
			}
			uint32_t busReg = configRead32(0, 0, 0, PCI_PRIMARY_BUS);
			configWrite32(0, 0, 0, PCI_PRIMARY_BUS, (busReg & 0xff000000U) | 1U << 16 | 1U << 8);
			uint32_t id = 0xffffffff;
			int ms;
			for (ms = 0; ms < 1000; ms += 10) {
				id = configRead32(1, 0, 0, PCI_VENDOR_ID);
				if ((id & 0xffff) != 0xffff && (id & 0xffff) != 0x0001 && id != 0) {
					break;
				}
				IOSleep(10);
			}
			IOLog("RPiPCIe: %s: 01:00.0 reads %04x:%04x after %d ms\n", provider->getName(), id & 0xffff, id >> 16, ms);
		}
		IOLog("RPiPCIe: %s: revision 0x%x, link up (set up %s), %u windows\n",
		    provider->getName(), rev & 0xffff, firmware ? "by the firmware" : "here", nwindows);
		adoptWindows(provider);
		lastBus = 0;
		barLo = ~0ULL;
		barHi = 0;
		functions = 0;
		scanAdopted(0, 0);
		if (functions == 0) {
			IOLog("RPiPCIe: %s: no device behind the root port\n", provider->getName());
		}
		for (unsigned i = 0; i < functions; i++) {
			Pending *f = &pending[i];
			publish(f->bus, f->dev, f->fn, f->pin, f->bars, f->nbars, f->barNumbers);
			for (int b = 0; b < f->nbars; b++) {
				f->bars[b]->release();
			}
		}
		registerService();
		return true;
	}
	if (!bringUp(provider)) {
		return false;
	}
	IOLog("RPiPCIe: revision 0x%x, link up; window PCI 0x%llx = CPU 0x%llx, %llu MB\n",
	    rev & 0xffff, winPCI, winCPU, winSize >> 20);

	lastBus = 0;
	memNext = winPCI;
	scanBus(0, 0);
	registerService();
	return true;
}

#pragma mark RPiPCIDevice

uint32_t
RPiPCIDevice::configRead32(uint16_t offset)
{
	return pcie->configRead32((uint8_t)(location >> 16), (uint8_t)(location >> 8), (uint8_t)location, offset);
}

void
RPiPCIDevice::configWrite32(uint16_t offset, uint32_t value)
{
	pcie->configWrite32((uint8_t)(location >> 16), (uint8_t)(location >> 8), (uint8_t)location, offset, value);
}

uint8_t
RPiPCIDevice::findCapability(uint8_t capID)
{
	if (!(configRead16(0x06) & 0x10)) {
		return 0;
	}
	uint8_t off = (uint8_t)(configRead32(0x34) & 0xfc);
	for (int i = 0; off >= 0x40 && i < 48; i++) {
		uint32_t cap = configRead32(off);
		if ((cap & 0xff) == capID) {
			return off;
		}
		off = (uint8_t)((cap >> 8) & 0xfc);
	}
	return 0;
}

int
RPiPCIDevice::barIndex(unsigned bar)
{
	OSData *numbers = OSDynamicCast(OSData, getProperty("bar-numbers"));
	const uint8_t *n = numbers ? (const uint8_t *)numbers->getBytesNoCopy() : NULL;
	for (unsigned i = 0; n != NULL && i < numbers->getLength(); i++) {
		if (n[i] == bar) {
			return (int)i;
		}
	}
	return -1;
}

uint16_t
RPiPCIDevice::configRead16(uint16_t offset)
{
	return pcie->configRead16((uint8_t)(location >> 16), (uint8_t)(location >> 8), (uint8_t)location, offset);
}

void
RPiPCIDevice::configWrite16(uint16_t offset, uint16_t value)
{
	pcie->configWrite16((uint8_t)(location >> 16), (uint8_t)(location >> 8), (uint8_t)location, offset, value);
}
