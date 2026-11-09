/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#if defined(__arm64__)

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/kauth.h>
#include <sys/sysctl.h>
#include <pexpert/pexpert.h>

static int
sysctl_previous_panic SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	const char *text = "";
	unsigned int len = 0;

	if (!kauth_cred_issuser(kauth_cred_get())) {
		return EPERM;
	}
	(void)PE_get_previous_panic(&text, &len);
	return SYSCTL_OUT(req, text, len + 1);
}

SYSCTL_PROC(_kern, OID_AUTO, previous_panic,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED | CTLFLAG_MASKED,
    0, 0, sysctl_previous_panic, "A", "the previous boot's panic log, if it panicked");

#endif
