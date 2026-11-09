/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <kern/startup.h>
#include <mach/kmod.h>

extern kern_return_t pthread_start(kmod_info_t *ki, void *d);

__startup_func
static void
pthread_builtin_init(void)
{
	pthread_start(NULL, NULL);
}
STARTUP(SYSCTL, STARTUP_RANK_LAST, pthread_builtin_init);
