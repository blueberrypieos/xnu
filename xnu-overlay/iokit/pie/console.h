/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <pie/console.h>: the kernel's video console (the text console on the
 * framebuffer) for a display driver that takes the screen over, from the
 * com.blueberrypieos.kpi.platform symbol set (config/PIE.exports);
 * osfmk/console/video_console.c and osfmk/arm/model_dep.c implement it.
 * C linkage.
 *
 * At boot the console draws on the framebuffer the firmware set up for
 * pieboot (PE_current_console describes it, and keeps describing it). A
 * DRM driver that scans out its own buffer moves the console there with
 * pie_console_takeover, stops it drawing while a program (the DRM
 * master) owns the screen with pie_console_set_graphics, and registers a
 * panic hook that puts the console's buffer back on the screen.
 */

#ifndef _PIE_CONSOLE_H_
#define _PIE_CONSOLE_H_

#include <sys/cdefs.h>
#include <stdbool.h>
#include <pexpert/pexpert.h>

__BEGIN_DECLS

/*
 * Draw the console on the framebuffer video describes from now on:
 * v_width x v_height pixels of v_depth bits (16: RGB565; 32: XRGB8888,
 * little-endian), v_rowBytes apart, at v_baseAddr + v_offset: a kernel
 * virtual address, or a physical one with bit 0 set (the console maps
 * v_length bytes, or v_height * v_rowBytes, write-combined, and unmaps
 * them at the next takeover). A virtual buffer must stay mapped until the
 * console has been moved off it.
 *
 * With copy, the text on the old screen is redrawn on the new one (the
 * rows nearest the cursor, if it has fewer); without, it starts empty.
 * 0, or EINVAL (no console yet, or a depth other than 16 or 32, or a
 * screen smaller than one character).
 */
int pie_console_takeover(const PE_Video *video, bool copy);

/*
 * graphics: stop drawing (a program owns the screen); the console keeps
 * its text, and redraws it all when graphics ends. The console keyboard
 * reads pie_console_graphics() to keep its keys from the console while
 * graphics lasts.
 */
void pie_console_set_graphics(bool graphics);
bool pie_console_graphics(void);

/*
 * fn is called first when the kernel panics, on the panicking CPU with the
 * others stopped: no locks, no allocation, no blocking (register writes
 * only: show the console's buffer). Then the console ends graphics and
 * draws the panic. NULL unregisters; a module unregisters before it
 * unloads.
 */
typedef void (*pie_panic_display_fn_t)(void);
void pie_panic_display_register(pie_panic_display_fn_t fn);

__END_DECLS

#endif /* _PIE_CONSOLE_H_ */
