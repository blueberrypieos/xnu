/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * RPiRTC: the Raspberry Pi 5's real-time clock (raspberrypi,rpi-rtc, in
 * its PMIC, kept running while the board is off by a battery on its BAT
 * connector), reached through the VideoCore firmware: mailbox tags
 * GET_RTC_REG 0x00030087 and SET_RTC_REG 0x00038087, each with a register
 * and its value, both 32-bit. The registers: TIME 0 (seconds since 1970),
 * ALARM 1, ALARM_PENDING 2, ALARM_ENABLE 3, BBAT_CHG_VOLTS 4 (the battery's
 * charge voltage in microvolts, 0: not charged), BBAT_CHG_VOLTS_MIN 5 and
 * _MAX 6, BBAT_VOLTS 7 (the battery's voltage).
 *
 * The time of day is the kernel's: RPiPlatformExpert reads TIME at boot,
 * before kexts load, and writes it whenever the clock is set (ntpdate,
 * date). This driver does the rest:
 * - At load, the charge voltage from the device tree's
 *   trickle-charge-microvolt (config.txt's dtparam=rtc_bbat_vchg, for the
 *   rechargeable battery; absent or 0, no charging), and the alarm that
 *   woke the board cleared, as Linux's driver does.
 * - The wake alarm: at ALARM's time a halted board powers on again. It
 *   raises no interrupt while the board runs; it is a wake-up only.
 * The hw.rtc sysctls: time, battery_uv, charge_uv (root: 0 or a voltage
 * within charge_min_uv-charge_max_uv; charging a battery that is not
 * rechargeable can damage it), charge_min_uv, charge_max_uv, wakealarm
 * (root: seconds since 1970, which sets and enables the alarm; 0 disables
 * it) and alarm_pending.
 *
 * The Pi 4 has no RTC: its device tree has no such node, and so has QEMU's
 * raspi4b's, so the driver matches only on the Pi 5 and 500.
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOService.h>
#include <libkern/OSByteOrder.h>
#include <sys/errno.h>
#include <sys/sysctl.h>

#include <pie/rpi.h>

#include "RPiRTCAlarm.h"

enum {
	FW_GET_RTC_REG = 0x00030087, FW_SET_RTC_REG = 0x00038087,
};

class RPiRTC : public IOService
{
	OSDeclareDefaultStructors(RPiRTC);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	int get(uint32_t reg, uint32_t *value);
	int set(uint32_t reg, uint32_t value);
	int setCharge(uint32_t uv);
	int setWakeAlarm(uint32_t secs);

private:
	IOLock *fLock;
	bool fSysctls, fOwnInstance;

	void registerSysctls(void);
	void unregisterSysctls(void);
};

OSDefineMetaClassAndStructors(RPiRTC, IOService);
#define super IOService

/* The instance the sysctls reach; set while they are registered. */
static RPiRTC *sRTC;

bool
RPiRTC::start(IOService *provider)
{
	if (!super::start(provider)) {
		return false;
	}
	/* Firmware registers are global: reject duplicate owners before writes. */
	if (!OSCompareAndSwapPtr(NULL, this, &sRTC)) {
		return false;
	}
	fOwnInstance = true;
	fLock = IOLockAlloc();
	uint32_t now = 0;
	if (fLock == NULL || get(RTC_TIME, &now) != 0) {
		IOLog("RPiRTC: the firmware's RTC does not answer\n");
		return false;
	}
	/* Capture boot status before any driver writes. Some EEPROM versions
	 * clear this flag before handing control to the OS. */
	uint32_t pending = 0;
	if (get(RTC_ALARM_PENDING, &pending) == 0 && pending != 0) {
		IOLog("RPiRTC: the board woke at its alarm\n");
	}
	uint32_t charge = 0;
	OSData *d = OSDynamicCast(OSData, provider->getProperty("trickle-charge-microvolt"));
	if (d != NULL && d->getLength() >= 4) {
		charge = OSReadBigInt32(d->getBytesNoCopy(), 0);
	}
	if (setCharge(charge) != 0) {
		IOLog("RPiRTC: the charge voltage (%u uV) was refused\n", charge);
	} else if (charge != 0) {
		IOLog("RPiRTC: charging the battery at %u uV\n", charge);
	}
	uint32_t bat = 0;
	set(RTC_ALARM_PENDING, 1);
	get(RTC_BBAT_VOLTS, &bat);
	IOLog("RPiRTC: time %u, battery %u uV\n", now, bat);
	registerSysctls();
	registerService();
	return true;
}

void
RPiRTC::stop(IOService *provider)
{
	unregisterSysctls();
	super::stop(provider);
}

void
RPiRTC::free(void)
{
	if (fOwnInstance) {
		OSCompareAndSwapPtr(this, NULL, &sRTC);
	}
	if (fLock != NULL) {
		IOLockFree(fLock);
		fLock = NULL;
	}
	super::free();
}

int
RPiRTC::get(uint32_t reg, uint32_t *value)
{
	uint32_t data[2] = { reg, 0 };
	if (RPiFirmwareProperty(FW_GET_RTC_REG, data, sizeof(data)) != kIOReturnSuccess) {
		return EIO;
	}
	*value = data[1];
	return 0;
}

int
RPiRTC::set(uint32_t reg, uint32_t value)
{
	uint32_t data[2] = { reg, value };
	return RPiFirmwareProperty(FW_SET_RTC_REG, data, sizeof(data)) == kIOReturnSuccess ? 0 : EIO;
}

int
RPiRTC::setCharge(uint32_t uv)
{
	IOLockLock(fLock);
	/* Turning charging off must not depend on optional limit registers. */
	if (uv == 0) {
		int err = set(RTC_BBAT_CHG_VOLTS, 0);
		IOLockUnlock(fLock);
		return err;
	}
	uint32_t lo = 0, hi = 0;
	int err = get(RTC_BBAT_CHG_VOLTS_MIN, &lo);
	if (err == 0) {
		err = get(RTC_BBAT_CHG_VOLTS_MAX, &hi);
	}
	if (err == 0 && uv != 0 && (uv < lo || uv > hi)) {
		err = EINVAL;
	}
	if (err == 0) {
		err = set(RTC_BBAT_CHG_VOLTS, uv);
	}
	IOLockUnlock(fLock);
	if (err == 0) {
		IOLog("RPiRTC: battery charging %s%u uV (hw.rtc.charge_uv)\n", uv != 0 ? "at " : "off, ", uv);
	}
	return err;
}

/* Disable, clear pending status, program, then enable; roll back on failure. */
int
RPiRTC::setWakeAlarm(uint32_t secs)
{
	IOLockLock(fLock);
	int err = RPiRTCProgramAlarm(secs, [](void *context, uint32_t reg, uint32_t value) -> int {
		return ((RPiRTC *)context)->set(reg, value);
	}, this);
	IOLockUnlock(fLock);
	if (err == 0 && secs != 0) {
		IOLog("RPiRTC: wake alarm at %u (hw.rtc.wakealarm)\n", secs);
	}
	return err;
}

#pragma mark sysctls

static int
sysctl_rtc(struct sysctl_oid *oidp, void *, int reg, struct sysctl_req *req)
{
	RPiRTC *r = sRTC;
	if (r == NULL) {
		return ENOENT;
	}
	uint32_t v = 0;
	int err = 0;
	/* Write-only safety controls must work even if firmware GET fails.
	 * Combined read/write requests still obtain their requested old value. */
	if (req->newptr == USER_ADDR_NULL || req->oldptr != USER_ADDR_NULL) {
		if (reg == RTC_ALARM) {
			uint32_t on = 0;
			err = r->get(RTC_ALARM_ENABLE, &on);
			if (err == 0 && on != 0) {
				err = r->get(RTC_ALARM, &v);
			}
		} else {
			err = r->get((uint32_t)reg, &v);
		}
		if (err != 0) {
			return err;
		}
	}
	/* 64-bit, so that a value too big for a register is refused, not cut to 32 bits. */
	int64_t out = v;
	err = sysctl_handle_quad(oidp, &out, 0, req);
	if (err != 0 || req->newptr == USER_ADDR_NULL) {
		return err;
	}
	if (out < 0 || out > UINT32_MAX) {
		return EINVAL;
	}
	switch (reg) {
	case RTC_BBAT_CHG_VOLTS: return r->setCharge((uint32_t)out);
	case RTC_ALARM: return r->setWakeAlarm((uint32_t)out);
	default: return EPERM;
	}
}

#define RTC_OID(name, flags, reg, descr) \
	SYSCTL_PROC(_hw_rtc, OID_AUTO, name, CTLTYPE_QUAD | (flags) | CTLFLAG_LOCKED, NULL, reg, \
	    sysctl_rtc, "Q", descr)

SYSCTL_NODE(_hw, OID_AUTO, rtc, CTLFLAG_RW | CTLFLAG_LOCKED, NULL, "real-time clock");
RTC_OID(time, CTLFLAG_RD, RTC_TIME, "the RTC's time, seconds since 1970");
RTC_OID(battery_uv, CTLFLAG_RD, RTC_BBAT_VOLTS, "the battery's voltage, microvolts");
RTC_OID(charge_uv, CTLFLAG_RW, RTC_BBAT_CHG_VOLTS, "the battery's charge voltage, microvolts; 0: not charged");
RTC_OID(charge_min_uv, CTLFLAG_RD, RTC_BBAT_CHG_VOLTS_MIN, "the lowest charge voltage");
RTC_OID(charge_max_uv, CTLFLAG_RD, RTC_BBAT_CHG_VOLTS_MAX, "the highest charge voltage");
RTC_OID(wakealarm, CTLFLAG_RW, RTC_ALARM, "when a halted board wakes, seconds since 1970; 0: never");
RTC_OID(alarm_pending, CTLFLAG_RD, RTC_ALARM_PENDING, "1: the alarm's time has come");

static struct sysctl_oid *const sOIDs[] = {
	&sysctl__hw_rtc, &sysctl__hw_rtc_time, &sysctl__hw_rtc_battery_uv, &sysctl__hw_rtc_charge_uv,
	&sysctl__hw_rtc_charge_min_uv, &sysctl__hw_rtc_charge_max_uv, &sysctl__hw_rtc_wakealarm,
	&sysctl__hw_rtc_alarm_pending,
};

void
RPiRTC::registerSysctls(void)
{
	if (!fOwnInstance) {
		return;
	}
	fSysctls = true;
	for (unsigned i = 0; i < sizeof(sOIDs) / sizeof(sOIDs[0]); i++) {
		sysctl_register_oid(sOIDs[i]);
	}
}

/* Leaves first, then the node; sysctl_unregister_oid waits for handlers that are running. */
void
RPiRTC::unregisterSysctls(void)
{
	if (!fSysctls) {
		return;
	}
	for (unsigned i = sizeof(sOIDs) / sizeof(sOIDs[0]); i-- > 0;) {
		sysctl_unregister_oid(sOIDs[i]);
	}
	fSysctls = false;
}
