# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIEBoard.kext: the Raspberry Pi's LEDs (the device tree's gpio-leds:
# ACT and PWR, with triggers: disk activity, heartbeat; ACT lit at a
# panic) and the Pi 5's and 500's power button (gpio-keys: a press halts),
# as hw.led and hw.powerbutton sysctls. Through piegpio's controllers
# (<PieGPIOKernel.h>) and the firmware's GPIO expander. One binary for both
# kernels, checked against both kernels' symbol sets.
#
# The kernel package carries it (image/mkbootpkgs.sh), in
# /System/Library/Extensions; /etc/rc loads it early
# (/usr/lib/blueberry-pie-os/kexts.early), after piegpio.kext. `make
# install` builds it only.
#
#   make -C kexts/Board [KERNEL_BUILD=DIR] install

PROJECT := Board
include ../common.mk

KEXT := PIEBoard
KEXT_SOURCES := RPiBoard.cpp
KEXT_CXXFLAGS := -Wall -Werror -I../GPIO/include
KEXT_OBJ := $(OBJ)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
