/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */
/*
 * <pie/sdio.h>: the kernel's SDIO interface, for SDIO function drivers in
 * kexts, from the com.blueberrypieos.kpi.platform symbol set
 * (config/PIE.exports). iokit/Drivers/sdmmc/PIESDIO.cpp implements it.
 * C linkage.
 *
 * An SDIO host publishes a PIESDIOFunction nub for each function of the
 * card in its slot. A driver matches one with IOProviderClass
 * PIESDIOFunction and the properties SDIOVendorID, SDIODeviceID and
 * SDIOFunction; SDIOInterfaceClass is the function's standard interface
 * code. The nub, cast to struct pie_sdio_func *, is the handle every call
 * takes.
 *
 * A driver claims the bus before any other call and releases it after.
 * The claim is a mutex and not recursive; a call made without it returns
 * EPERM. The interrupt handler runs on the bus's interrupt thread, which
 * has claimed the bus for it. Once interrupt_unregister returns, with any
 * result but EPERM, the handler does not run again and its argument may
 * be freed. The calls return 0 or an errno, and ENXIO once the host has
 * given up on the card.
 *
 * read1 and write1 are CMD52 on function fn, 0 or the function's own.
 * The others are CMD53 on the function: read4 and write4 move 4 bytes at
 * an incrementing address, little-endian; read and write move any length,
 * as whole blocks and then the rest, at an incrementing or a fixed
 * address. enable waits for the function's I/O Ready. set_block_size 0
 * gives the smaller of the function's largest block and 512. align_size
 * rounds a length up to what one transfer moves: a multiple of 4, and
 * past one block a multiple of the block size. max_request is the most
 * one read or write call moves in one command.
 *
 * function gives the handle of the card's function fn, from the handle
 * of any of its functions, or NULL if the card has none. It stays valid
 * while the caller's own function is, because the host publishes and
 * removes a card's functions together. A driver of function 1 reaches
 * function 2 this way.
 *
 * reset_register sets the function's reset callback, and NULL removes
 * it. When the host has had to power the card up and set it up again
 * after an error, the callback runs on the bus's interrupt thread,
 * without the claim. The function's block size, I/O enable and interrupt
 * enable are as they were; everything else on the card is as at
 * power-up. reset_register is called without the claim, and returns
 * EDEADLK under it. It waits while the callback runs, unless the
 * callback makes the call itself. Once NULL is set, the old callback
 * does not run again and its argument may be freed.
 *
 * reset_card has the host power the card up and set it up again, as
 * after an error, and returns once it is done. It needs the claim, and
 * sleeps. Afterwards each function's block size, I/O enable and
 * interrupt enable are as they were, and everything else on the card is
 * as at power-up. The other functions' reset callbacks run as after an
 * error. The caller's own does not run for it, unless another new set-up
 * still waits for the callbacks, or comes before they run. If the card
 * does not come back, the host gives up on it and the call returns
 * ENXIO.
 *
 * generation is the card's set-up count. It is 1 after the first set-up.
 * It goes up by one each time the host sets the card up again, after an
 * error or for reset_card. It is 0 once the host has given up on the
 * card. A driver reads it with the claim held, and then only its own calls
 * change it. So it can tell whether the card was set up again since it
 * last set the card up itself. A reset callback may read it without the
 * claim. The callback then gets the count of its own set-up or a later
 * one, or 0 for a card given up.
 */

#ifndef _PIE_SDIO_H_
#define _PIE_SDIO_H_

#include <sys/cdefs.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

__BEGIN_DECLS

struct pie_sdio_func;

int pie_sdio_claim(struct pie_sdio_func *f);
void pie_sdio_release(struct pie_sdio_func *f);
int pie_sdio_read1(struct pie_sdio_func *f, unsigned int fn, uint32_t addr, uint8_t *val);
int pie_sdio_write1(struct pie_sdio_func *f, unsigned int fn, uint32_t addr, uint8_t val);
int pie_sdio_read4(struct pie_sdio_func *f, uint32_t addr, uint32_t *val);
int pie_sdio_write4(struct pie_sdio_func *f, uint32_t addr, uint32_t val);
int pie_sdio_read(struct pie_sdio_func *f, uint32_t addr, void *buf, size_t len, bool incr);
int pie_sdio_write(struct pie_sdio_func *f, uint32_t addr, const void *buf, size_t len, bool incr);
int pie_sdio_enable(struct pie_sdio_func *f);
int pie_sdio_disable(struct pie_sdio_func *f);
int pie_sdio_set_block_size(struct pie_sdio_func *f, unsigned int size);
size_t pie_sdio_align_size(struct pie_sdio_func *f, size_t len);
size_t pie_sdio_max_request(struct pie_sdio_func *f);
int pie_sdio_interrupt_register(struct pie_sdio_func *f, void (*handler)(void *), void *arg);
int pie_sdio_interrupt_unregister(struct pie_sdio_func *f);
int pie_sdio_reset_register(struct pie_sdio_func *f, void (*handler)(void *), void *arg);
int pie_sdio_reset_card(struct pie_sdio_func *f);
unsigned int pie_sdio_generation(struct pie_sdio_func *f);
bool pie_sdio_is_uhs(struct pie_sdio_func *f);
uint32_t pie_sdio_clock(struct pie_sdio_func *f);
unsigned int pie_sdio_number(struct pie_sdio_func *f);
uint16_t pie_sdio_vendor(struct pie_sdio_func *f);
uint16_t pie_sdio_device(struct pie_sdio_func *f);
unsigned int pie_sdio_block_size(struct pie_sdio_func *f);
struct pie_sdio_func *pie_sdio_function(struct pie_sdio_func *f, unsigned int fn);

__END_DECLS

#endif
