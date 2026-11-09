# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIERandom.kext: the hardware random-number generator both the BCM2711
# and the BCM2712 have (the iproc RNG200), reseeding the kernel's PRNG
# (write_random(), in com.blueberrypieos.kpi.platform). One binary for
# both kernels, checked against both kernels' symbol sets.
#
# The kernel package carries it (image/mkbootpkgs.sh), in
# /System/Library/Extensions; /etc/rc loads it first
# (/usr/lib/blueberry-pie-os/kexts.early). `make install` builds it only.
#
#   make -C kexts/Random [KERNEL_BUILD=DIR] install

PROJECT := Random
include ../common.mk

KEXT := PIERandom
KEXT_SOURCES := RPiRNG200.cpp
KEXT_CXXFLAGS := -Wall -Werror
KEXT_OBJ := $(OBJ)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
