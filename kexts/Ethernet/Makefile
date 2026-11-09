# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIEEthernet.kext: the Ethernet drivers, on IONetworkingFamily
# (IONetworkingFamily.kext, toolchain/build-ionetworking-kext.sh, its
# library): RPiGENET (the Raspberry Pi 4's GENET), RPiGEM (the Pi 5's RP1
# GEM), the BCM54213PE PHY both have (RPiPHY.cpp, private to this kext)
# and VirtioMMIONet (QEMU virt's virtio-net). One binary for both kernels,
# checked against both kernels' symbol sets; the platform helpers it
# calls are com.blueberrypieos.kpi.platform's (<pie/rpi.h>). RPiGEM
# pulses the PHY's reset through piegpio's GPIO controller
# (callPlatformFunction), not a link-time dependency.
#
# The kernel package carries it (image/mkbootpkgs.sh), in
# /System/Library/Extensions, with IONetworkingFamily.kext; /etc/rc loads
# it before the network starts (/usr/lib/blueberry-pie-os/kexts.early).
# `make install` builds it only: it has no package of its own.
#
#   make -C kexts/Ethernet [KERNEL_BUILD=DIR] [IONETWORKING_KEXT_DIR=DIR] install

PROJECT := Ethernet
include ../common.mk

IONETWORKING_KEXT_DIR ?= $(PIE)/build/kexts/IONetworkingFamily

KEXT := PIEEthernet
KEXT_SOURCES := RPiGENET.cpp RPiGEM.cpp RPiPHY.cpp VirtioMMIONet.cpp
KEXT_CXXFLAGS := -Wall -Werror -Wno-inconsistent-missing-override
KEXT_OBJ := $(OBJ)
KEXT_LIBDIRS := $(IONETWORKING_KEXT_DIR)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
