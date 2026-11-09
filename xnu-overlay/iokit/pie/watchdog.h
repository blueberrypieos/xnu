/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <pie/watchdog.h>: the kernel's own use of the Raspberry Pi's PM
 * watchdog, for the driver that takes it over, from the
 * com.blueberrypieos.kpi.platform symbol set (config/PIE.exports);
 * iokit/Drivers/rpi/RPiWatchdogEarly.cpp implements it. C linkage.
 *
 * From the moment IOKit publishes the PM block (brcm,bcm2835-pm-wdt,
 * brcm,bcm2711-pm, brcm,bcm2712-pm), the kernel arms the watchdog for 15 s
 * and sets it again every second, unless the firmware is not the
 * VideoCore's (QEMU's, whose model resets at once when armed) or the
 * boot-arg pie_watchdog is 0: a hang before PIEWatchdog.kext loads
 * resets the board too. The PM block's nub says who has it in its
 * WatchdogOwner property: kernel, kext or none.
 *
 * Whoever has it, a halt disarms it (a halted board must stay halted), and
 * a panic sets it to a full 15 s once, as the panic begins: a panic that
 * never reboots resets the board then.
 */

#ifndef _PIE_WATCHDOG_H_
#define _PIE_WATCHDOG_H_

#include <sys/cdefs.h>
#include <stdbool.h>

__BEGIN_DECLS

/*
 * The driver has the watchdog from now on: the kernel stops setting it
 * (waiting for a set that is running) and leaves the hardware as it is.
 * True if the kernel had it armed. May block; not from an interrupt.
 */
bool pie_watchdog_takeover(void);

/*
 * The driver gives it back as it stops. With armed, the kernel arms it
 * again and sets it every second, and returns true; false if it will not
 * (QEMU's firmware, pie_watchdog=0, no PM block): the driver disarms it
 * itself. Without armed, the kernel leaves it alone (false).
 */
bool pie_watchdog_handback(bool armed);

__END_DECLS

#endif /* _PIE_WATCHDOG_H_ */
