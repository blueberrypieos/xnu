# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# PIEDMA.kext: the Raspberry Pi's DMA controllers, for kernel drivers
# whose device tree nodes name a channel (<PIEDMA.h>, <PIEDMAKernel.h>
# in include/): the BCM2711's and BCM2712's Broadcom DMA (legacy, LITE and
# 40-bit channels) and RP1's DesignWare AXI DMAC; debug.dma sysctls with a
# self-test. One binary for both kernels, checked against both kernels'
# symbol sets.
#
# The kernel package carries it (image/mkbootpkgs.sh), in
# /System/Library/Extensions; /etc/rc loads it early
# (/usr/lib/blueberry-pie-os/kexts.early). `make install` builds it only.
#
#   make -C kexts/DMA [KERNEL_BUILD=DIR] install

PROJECT := DMA
include ../common.mk

KEXT := PIEDMA
KEXT_SOURCES := DMAController.cpp DMABCM2835.cpp DMADWAxi.cpp
KEXT_CXXFLAGS := -Wall -Werror -Iinclude
KEXT_OBJ := $(OBJ)
include $(TOOLCHAIN)/kext.mk

install: all

.PHONY: install
