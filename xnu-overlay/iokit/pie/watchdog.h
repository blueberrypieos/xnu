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
 *
 * With the boot-arg pie_watchdog_boot=N, N seconds from 60 to 86400, the
 * kernel panics unless user space checks in through IOWatchDogTimer
 * within N seconds of boot, armed or not. The driver takes the deadline
 * over as it starts, as a check-in due then. N=0 turns it off. Any
 * other value outside 60 to 86400, or not a number, gives 86400.
 */

#ifndef _PIE_WATCHDOG_H_
#define _PIE_WATCHDOG_H_

#include <sys/cdefs.h>
#include <stdbool.h>
#include <stdint.h>

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

/*
 * The boot deadline, in mach_absolute_time units, for the driver to take
 * over. *seconds gets its length. It returns 0 if there is none. The
 * kernel no longer checks it once this has returned it, so the driver
 * keeps the same deadline.
 */
uint64_t pie_watchdog_boot_deadline(uint32_t *seconds);

/*
 * The driver gives back a boot deadline still due, before
 * pie_watchdog_handback, as it stops. The kernel checks it again.
 */
void pie_watchdog_boot_restore(uint64_t deadline, uint32_t seconds);

__END_DECLS

#endif /* _PIE_WATCHDOG_H_ */
