/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_ROCKCHIP_RKUSB2PHY_H
#define _IOKIT_ROCKCHIP_RKUSB2PHY_H

#include <IOKit/IOLocks.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>

/*
 * An RK3568 USB 2 PHY. Its two ports are 0, the otg-port, and 1, the
 * host-port. The controller driver that uses a host-port enables it with
 * enablePort, which brings the PHY up the first time; portStatus gives the
 * port's VBUS valid, line state and host disconnect as bits 2, 5-4 and 7.
 */
enum { RK_USB2PHY_OTG_PORT = 0, RK_USB2PHY_HOST_PORT = 1 };

class RKUSB2PHY : public IOService
{
	OSDeclareDefaultStructors(RKUSB2PHY);

	IOMemoryMap *grfMap, *cru, *pmucru;
	IOLock *lock;
	unsigned index;
	bool up, hostUp;

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	IOReturn enablePort(unsigned port);
	uint32_t portStatus(unsigned port);
};

#endif
