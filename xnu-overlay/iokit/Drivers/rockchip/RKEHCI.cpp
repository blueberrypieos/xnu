/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceMemory.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <libkern/OSByteOrder.h>

#include "../usb/EHCIController.h"
#include "RKSoC.h"
#include "RKUSB2PHY.h"

/*
 * The RK3568's two EHCI hosts at fd800000 and fd880000: their bus clocks in
 * CRU GATE_CON16, their resets in SOFTRST_CON14, and the USB 2 PHY port of
 * their phys property.
 */
enum {
	EHCI0_BASE = 0xfd800000, EHCI1_BASE = 0xfd880000,
	GATE16_USB_BUS = 0x7f, SOFTRST14_USB_BUS = 0xf,
};
#define GATE16_HOST(i)          (0x3U << (12 + 2 * (i)))
#define SOFTRST14_HOST(i)       (0x7U << (4 + 3 * (i)))

class RKEHCI : public EHCIController
{
	OSDeclareDefaultStructors(RKEHCI);

	RKUSB2PHY *findPHY(IOService *provider, unsigned *port);

protected:
	virtual bool platformStart(IOService *provider) APPLE_KEXT_OVERRIDE;
};

#define super EHCIController
OSDefineMetaClassAndStructors(RKEHCI, EHCIController);

/*
 * The PHY driver of the port that phys names, waited for up to ten
 * seconds, or until stop has begun. The port's parent is read again on
 * each try: the PHY's nub replaces its node while drivers start.
 */
RKUSB2PHY *
RKEHCI::findPHY(IOService *provider, unsigned *port)
{
	OSData *phys = OSDynamicCast(OSData, provider->getProperty("phys"));
	IORegistryEntry *portNode = phys != NULL && phys->getLength() >= 4 ?
	    RKFindPHandle(IORegistryEntry::getRegistryRoot(),
	    OSSwapBigToHostInt32(*(const uint32_t *)phys->getBytesNoCopy())) : NULL;
	if (portNode == NULL) {
		return NULL;
	}
	const char *name = portNode->getName(gIODTPlane);
	*port = name != NULL && strcmp(name, "host-port") == 0 ? RK_USB2PHY_HOST_PORT : RK_USB2PHY_OTG_PORT;
	RKUSB2PHY *phy = NULL;
	for (int tries = 0; phy == NULL && tries < 100 && !stopRequested(); tries++) {
		IORegistryEntry *phyNode = portNode->copyParentEntry(gIODTPlane);
		OSIterator *it = phyNode != NULL ? phyNode->getChildIterator(gIOServicePlane) : NULL;
		for (OSObject *o; it != NULL && (o = it->getNextObject()) != NULL;) {
			phy = OSDynamicCast(RKUSB2PHY, o);
			if (phy != NULL) {
				phy->retain();
				break;
			}
		}
		OSSafeReleaseNULL(it);
		OSSafeReleaseNULL(phyNode);
		if (phy == NULL) {
			IOSleep(100);
		}
	}
	portNode->release();
	return phy;
}

/*
 * After the boot volume has appeared: the PHY port, then the host's bus
 * clocks and resets. Waited for in steps of a second, so that stop can
 * cancel it.
 */
bool
RKEHCI::platformStart(IOService *provider)
{
	IODeviceMemory *mem = provider->getDeviceMemoryWithIndex(0);
	uint64_t base = mem != NULL ? mem->getPhysicalAddress() : 0;
	if (base != EHCI0_BASE && base != EHCI1_BASE) {
		IOLog("RKEHCI: %s at 0x%llx: unknown host\n", provider->getName(), base);
		return false;
	}
	unsigned host = base == EHCI0_BASE ? 0 : 1;
	bool found = false;
	for (int s = 0; s < 60 && !found && !stopRequested(); s++) {
		found = RKWaitForBootVolume(1);
	}
	if (stopRequested()) {
		return false;
	}
	if (!found) {
		IOLog("RKEHCI: no boot volume within a minute\n");
	}
	unsigned port = 0;
	RKUSB2PHY *phy = findPHY(provider, &port);
	IOReturn ret = phy != NULL ? phy->enablePort(port) : kIOReturnNotFound;
	OSSafeReleaseNULL(phy);
	if (ret != kIOReturnSuccess) {
		IOLog("RKEHCI: ehci%u: its PHY port is not available (0x%x)\n", host, ret);
		return false;
	}
	IOMemoryMap *cru = RKMapCompatible("rockchip,rk3568-cru");
	if (cru == NULL) {
		IOLog("RKEHCI: ehci%u: no clock unit\n", host);
		return false;
	}
	RKWriteMasked(cru, RK_CRU_GATE(16), GATE16_USB_BUS | GATE16_HOST(host), 0);
	RKWriteMasked(cru, RK_CRU_SOFTRST(14), SOFTRST14_USB_BUS | SOFTRST14_HOST(host), 0);
	IODelay(100);
	OSSafeReleaseNULL(cru);
	return true;
}
