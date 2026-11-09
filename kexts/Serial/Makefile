# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIESerial.kext: the Raspberry Pi 4's and 5's PL011 UARTs as serial
# ports (/dev/cu.serialN, /dev/tty.serialN), under IOSerialFamily
# (toolchain/build-ioserial-kext.sh, which must be built first: this kext
# links against it and is checked against its build). One binary for both
# kernels, checked against both kernels' symbol sets. It sets pins through
# piegpio (<PieGPIOKernel.h>: platform functions, no link-time
# dependency).
#
# The kernel package carries it (image/mkbootpkgs.sh), in
# /System/Library/Extensions; /etc/rc loads it early, after IOSerialFamily
# and piegpio (/usr/lib/blueberry-pie-os/kexts.early). `make install`
# builds it only.
#
#   make -C kexts/Serial [KERNEL_BUILD=DIR] install

PROJECT := Serial
include ../common.mk

KEXT := PIESerial
KEXT_SOURCES := RPiPL011Serial.cpp
KEXT_CXXFLAGS := -Wall -Werror -I../GPIO/include
KEXT_LIBDIRS := $(PIE)/build/kexts/IOSerialFamily
KEXT_OBJ := $(OBJ)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
