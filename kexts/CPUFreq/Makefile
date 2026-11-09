# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIECPUFreq.kext: the Raspberry Pi's CPU clock (the firmware's ARM
# clock), set by a governor that follows the busiest CPU's load, and the
# firmware's undervoltage and throttling reported, as hw.cpufreq sysctls;
# hw.cpufrequency, _min and _max filled in. One binary for both kernels,
# checked against both kernels' symbol sets; it needs a kernel with
# <pie/cpu.h> (this tree's kernel package).
#
# The kernel package carries it (image/mkbootpkgs.sh), in
# /System/Library/Extensions; /etc/rc loads it early
# (/usr/lib/blueberry-pie-os/kexts.early). `make install` builds it only.
#
#   make -C kexts/CPUFreq [KERNEL_BUILD=DIR] install

PROJECT := CPUFreq
include ../common.mk

KEXT := PIECPUFreq
KEXT_SOURCES := RPiCPUFreq.cpp
KEXT_CXXFLAGS := -Wall -Werror
KEXT_OBJ := $(OBJ)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
