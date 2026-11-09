# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIEWatchdog.kext: the hardware watchdog both the BCM2711 and the
# BCM2712 have (the power-management block's), armed and set again every
# second by the kernel, with user space's check-ins through
# IOWatchDogTimer. One binary for both kernels, checked against both
# kernels' symbol sets.
#
# The kernel package carries it (image/mkbootpkgs.sh), in
# /System/Library/Extensions; /etc/rc loads it first
# (/usr/lib/blueberry-pie-os/kexts.early). `make install` builds it only.
#
#   make -C kexts/Watchdog [KERNEL_BUILD=DIR] install

PROJECT := Watchdog
include ../common.mk

KEXT := PIEWatchdog
KEXT_SOURCES := RPiWatchdog.cpp
KEXT_CXXFLAGS := -Wall -Werror
KEXT_OBJ := $(OBJ)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
