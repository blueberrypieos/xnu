# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIERTC.kext: the Raspberry Pi 5's real-time clock (the firmware's
# raspberrypi,rpi-rtc) beyond the time of day, which the kernel keeps
# itself: the battery's charging and voltage, and the wake alarm, as
# hw.rtc sysctls. One binary for both kernels, checked against both
# kernels' symbol sets; on the Pi 4, which has no RTC, it matches nothing.
#
# The kernel package carries it (image/mkbootpkgs.sh), in
# /System/Library/Extensions; /etc/rc loads it early
# (/usr/lib/blueberry-pie-os/kexts.early). `make install` builds it only.
#
#   make -C kexts/RTC [KERNEL_BUILD=DIR] install

PROJECT := RTC
include ../common.mk

KEXT := PIERTC
KEXT_SOURCES := RPiRTC.cpp
KEXT_CXXFLAGS := -Wall -Werror
KEXT_OBJ := $(OBJ)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
