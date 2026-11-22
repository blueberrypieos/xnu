/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IOLib.h>
#include <libkern/OSByteOrder.h>

#include "RKSoC.h"
#include "RKUSB2PHY.h"

/*
 * The RK3568's two USB 2 PHYs, each with two ports: port 0 is the
 * otg-port and port 1 the host-port. usb2phy0 at fe8a0000 feeds the two
 * DWC3 cores, usb2phy1 at fe8b0000 the two EHCI and OHCI hosts. The ports'
 * controls and status are in each PHY's GRF.
 */
enum {
	USB2PHY0_BASE = 0xfe8a0000, USB2PHY1_BASE = 0xfe8b0000,
	GRF_CON1 = 0x04, GRF_CON2 = 0x08, GRF_STATUS = 0xc0,
	CON_UTMI_GRF = 1U << 0, CON_NOT_SUSPENDED = 1U << 1,
	CON2_480M_OFF_IN_SUSPEND = 1U << 4,
	PORT_HOST = RK_USB2PHY_HOST_PORT, PORT_STATUS_SHIFT = 12,
	STATUS_VBUS_VALID = 1U << 2, STATUS_LINE_SHIFT = 4, STATUS_DISCONNECT = 1U << 7,
};

/*
 * Each PHY's clocks and resets: its 24 MHz reference gate in PMUCRU
 * GATE_CON02 and mux in CLKSEL_CON08, its GRF's clock in CRU GATE_CON34 and
 * reset in SOFTRST_CON28, and its power-on and port resets in SOFTRST_CON29.
 */
#define PMUGATE2_REF(i)         (1U << (1 + (i)))
#define PMUCLKSEL8_REF_OSC(i)   (1U << (i))
#define GATE34_GRF(i)           (1U << (7 + (i)))
#define SOFTRST28_GRF(i)        (1U << (10 + (i)))
#define SOFTRST29_PHY(i)        (0x7U << (3 * (i)))

#define super IOService
OSDefineMetaClassAndStructors(RKUSB2PHY, IOService);

/* A port's UTMI status: VBUS valid, the line state and host disconnect, as bits 2, 5-4 and 7. */
uint32_t
RKUSB2PHY::portStatus(unsigned port)
{
	return (RKRead(grfMap, GRF_STATUS) >> (port == PORT_HOST ? PORT_STATUS_SHIFT : 0)) &
	       (STATUS_VBUS_VALID | 3U << STATUS_LINE_SHIFT | STATUS_DISCONNECT);
}

/*
 * The PHY's reference clock, GRF clock and resets come up and its 480 MHz
 * clock stays on, the first time. The host-port then goes to its
 * controller's UTMI, out of suspend. otg-ports are not supported yet.
 */
IOReturn
RKUSB2PHY::enablePort(unsigned port)
{
	static const char *const lines[] = { "SE0", "D+ high", "D- high", "SE1" };
	if (port != PORT_HOST) {
		return kIOReturnUnsupported;
	}
	IOLockLock(lock);
	if (!up) {
		RKWriteMasked(pmucru, RK_PMUCRU_GATE(2), PMUGATE2_REF(index), 0);
		RKWriteMasked(pmucru, RK_PMUCRU_CLKSEL(8), PMUCLKSEL8_REF_OSC(index), PMUCLKSEL8_REF_OSC(index));
		RKWriteMasked(cru, RK_CRU_GATE(34), GATE34_GRF(index), 0);
		RKWriteMasked(cru, RK_CRU_SOFTRST(28), SOFTRST28_GRF(index), 0);
		RKWriteMasked(cru, RK_CRU_SOFTRST(29), SOFTRST29_PHY(index), 0);
		IODelay(2000);
		RKWriteMasked(grfMap, GRF_CON2, CON2_480M_OFF_IN_SUSPEND, 0);
		up = true;
	}
	if (!hostUp) {
		RKWriteMasked(grfMap, GRF_CON1, CON_UTMI_GRF | CON_NOT_SUSPENDED, CON_NOT_SUSPENDED);
		IODelay(2000);
		hostUp = true;
		uint32_t st = portStatus(PORT_HOST);
		IOLog("RKUSB2PHY: usb2phy%u host port enabled: VBUS %s, line state %s%s\n", index,
		    (st & STATUS_VBUS_VALID) ? "valid" : "not valid", lines[(st >> STATUS_LINE_SHIFT) & 3],
		    (st & STATUS_DISCONNECT) ? ", disconnected" : "");
	}
	IOLockUnlock(lock);
	return kIOReturnSuccess;
}

bool
RKUSB2PHY::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	IOMemoryMap *regs = provider->mapDeviceMemoryWithIndex(0);
	uint64_t base = regs != NULL ? regs->getPhysicalAddress() : 0;
	OSSafeReleaseNULL(regs);
	if (base != USB2PHY0_BASE && base != USB2PHY1_BASE) {
		IOLog("RKUSB2PHY: %s at 0x%llx: unknown PHY\n", provider->getName(), base);
		return false;
	}
	index = base == USB2PHY0_BASE ? 0 : 1;

	OSData *ph = OSDynamicCast(OSData, provider->getProperty("rockchip,usbgrf"));
	grfMap = ph != NULL && ph->getLength() == 4 ?
	    RKMapPHandle(OSSwapBigToHostInt32(*(const uint32_t *)ph->getBytesNoCopy())) : NULL;
	cru = RKMapCompatible("rockchip,rk3568-cru");
	pmucru = RKMapCompatible("rockchip,rk3568-pmucru");
	lock = IOLockAlloc();
	if (grfMap == NULL || cru == NULL || pmucru == NULL || lock == NULL) {
		IOLog("RKUSB2PHY: usb2phy%u: no GRF or clock unit\n", index);
		return false;
	}
	registerService();
	return true;
}

void
RKUSB2PHY::free(void)
{
	OSSafeReleaseNULL(grfMap);
	OSSafeReleaseNULL(cru);
	OSSafeReleaseNULL(pmucru);
	if (lock != NULL) {
		IOLockFree(lock);
		lock = NULL;
	}
	super::free();
}
