# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIEThermal.kext: the SoC's temperature (the AVS monitor both the
# BCM2711 and the BCM2712 have, else the firmware's) and the fan the device
# tree's thermal zone cools with (the Pi 5's fan connector, RP1's PWM; the
# Pi 4's gpio-fan overlay), as hw.thermal sysctls. One binary for both
# kernels, checked against both kernels' symbol sets. It sets pins through
# piegpio's GPIO controllers (callPlatformFunction, <PieGPIOKernel.h>),
# not a link-time dependency.
#
# The kernel package carries it (image/mkbootpkgs.sh), in
# /System/Library/Extensions; /etc/rc loads it early, after piegpio
# (/usr/lib/blueberry-pie-os/kexts.early), so that the fan never waits
# for an optional package. `make install` builds it only.
#
#   make -C kexts/Thermal [KERNEL_BUILD=DIR] install

PROJECT := Thermal
include ../common.mk

KEXT := PIEThermal
KEXT_SOURCES := RPiThermal.cpp
KEXT_CXXFLAGS := -Wall -Werror -I../GPIO/include
KEXT_OBJ := $(OBJ)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
