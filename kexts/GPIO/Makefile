# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# piegpio.kext: the GPIO family (PieGPIOController and its user clients,
# include/PieGPIO.h) and its controller drivers: PieGPIOBCM2711 (the Pi 4's),
# PieGPIORP1 and PieGPIOBrcmstb (the Pi 5's). One binary for both kernels,
# checked against both kernels' symbol sets.
#
#   make -C kexts/GPIO KEXT_OBJ=DIR KEXT_SDK=SDK KEXT_SYSTEM_DIR=DIR [KEXT_SOCS=SOC]
#
# KEXT_SYSTEM_DIR holds the kernels' System.kexts, as
# blueberry-pie-os-kernel-dev installs them in /usr/lib/blueberry-pie-os/kernel.

PIETOOLCHAIN ?= /usr/lib/pietoolchain

KEXT := piegpio
KEXT_SOURCES := PieGPIOController.cpp PieGPIOUserClient.cpp PieGPIOBCM2711.cpp PieGPIORP1.cpp \
	PieGPIOBrcmstb.cpp
KEXT_CXXFLAGS := -Wall -Wextra -Werror -Wno-inconsistent-missing-override -Wno-unused-parameter \
	-I$(CURDIR)/include
include $(PIETOOLCHAIN)/toolchain/kernel/kext.mk
