# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIEUSBEthernet.kext: USB Ethernet adapters, on the kernel's USB interfaces
# (<pie/usb.h>, com.blueberrypieos.kpi.platform) and IONetworkingFamily.
# RTLUSBEthernet drives the Realtek RTL8152, RTL8153 and RTL8153B. It is
# OpenBSD's if_ure. One binary for all the kernels, checked against each
# one's symbol set.
#
# The kernel package carries it in /System/Library/Extensions. It is not an
# early kext: kextd loads it when an adapter appears.
#
#   make -C kexts/USBEthernet [KERNEL_BUILD=DIR] [IONETWORKING_KEXT_DIR=DIR] install

PROJECT := USBEthernet
include ../common.mk

IONETWORKING_KEXT_DIR ?= $(PIE)/build/kexts/IONetworkingFamily

KEXT := PIEUSBEthernet
KEXT_SOURCES := RTLUSBEthernet.cpp
KEXT_CXXFLAGS := -Wall -Werror -Wno-inconsistent-missing-override
KEXT_OBJ := $(OBJ)
KEXT_LIBDIRS := $(IONETWORKING_KEXT_DIR)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
