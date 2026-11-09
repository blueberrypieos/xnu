/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PieGPIOBCM2711: the Raspberry Pi 4's GPIO block (brcm,bcm2711-gpio), 58
 * lines, as the BCM2711 peripherals datasheet describes it: a function
 * select field of 3 bits per line (GPFSELn: 0 input, 1 output, else a
 * peripheral's), set and clear registers for outputs, the pins' levels
 * (GPLEVn), rising and falling edge detect enables (GPRENn, GPFENn) whose
 * events latch in GPEDSn (written 1 to clear), and a 2-bit pull per line
 * (GPIO_PUP_PDN_CNTRL_REGn: 0 none, 1 up, 2 down). Its first three
 * interrupts are the banks' (lines 0-27, 28-45, 46-57); the edge detect
 * keeps an interrupt up until its event is cleared. No debounce in
 * hardware. The function select values: 0 input, 1 output, 4-7 alternate
 * functions 0-3, 3 alternate function 4, 2 alternate function 5 (the
 * device tree's brcm,function cells are these values). Linux's gpiochip0,
 * pinctrl-bcm2711.
 */

#include <IOKit/IOLib.h>

#include "PieGPIOController.h"

class PieGPIOBCM2711 : public PieGPIOController
{
	OSDeclareDefaultStructors(PieGPIOBCM2711);

	IOMemoryMap *fMap;
	volatile uint32_t *fRegs;

	uint32_t rd(unsigned off) { return fRegs[off / 4]; }
	void wr(unsigned off, uint32_t v) { fRegs[off / 4] = v; }
	void setBit(unsigned base, unsigned line, bool on)
	{
		unsigned off = base + 4 * (line / 32);
		uint32_t bit = 1U << (line % 32);
		wr(off, on ? rd(off) | bit : rd(off) & ~bit);
	}

protected:
	virtual bool hwStart(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual unsigned hwInterruptCount(void) APPLE_KEXT_OVERRIDE { return 3; }
	virtual void hwSetOutput(unsigned line, bool output) APPLE_KEXT_OVERRIDE;
	virtual bool hwIsOutput(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual bool hwRead(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual void hwWrite(unsigned line, bool high) APPLE_KEXT_OVERRIDE;
	virtual bool hwSupportsBias(void) APPLE_KEXT_OVERRIDE { return true; }
	virtual bool hwSetBias(unsigned line, uint8_t bias) APPLE_KEXT_OVERRIDE;
	virtual uint8_t hwGetBias(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual void hwSetEdges(unsigned line, bool rising, bool falling) APPLE_KEXT_OVERRIDE;
	virtual uint64_t hwTakeEdges(void) APPLE_KEXT_OVERRIDE;
	virtual bool hwFunctionNumber(unsigned line, const char *name, unsigned *fsel) APPLE_KEXT_OVERRIDE;
	virtual bool hwBrcmFunction(unsigned line, uint32_t function, unsigned *fsel) APPLE_KEXT_OVERRIDE;
	virtual void hwSetFunction(unsigned line, unsigned fsel) APPLE_KEXT_OVERRIDE;
	virtual const char *chipLabel(void) APPLE_KEXT_OVERRIDE { return "pinctrl-bcm2711"; }

public:
	virtual void free(void) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(PieGPIOBCM2711, PieGPIOController);

enum {
	GPFSEL0 = 0x00, GPSET0 = 0x1c, GPCLR0 = 0x28, GPLEV0 = 0x34, GPEDS0 = 0x40,
	GPREN0 = 0x4c, GPFEN0 = 0x58, PUP_PDN0 = 0xe4,
	LINES = 58,
};

bool
PieGPIOBCM2711::hwStart(IOService *provider)
{
	fMap = provider->mapDeviceMemoryWithIndex(0, kIOMapInhibitCache);
	if (fMap == NULL) {
		return false;
	}
	fRegs = (volatile uint32_t *)fMap->getVirtualAddress();
	fLineCount = LINES;
	/* Nothing latched from before (the firmware's settings). */
	wr(GPREN0, 0);
	wr(GPREN0 + 4, 0);
	wr(GPFEN0, 0);
	wr(GPFEN0 + 4, 0);
	wr(GPEDS0, ~0U);
	wr(GPEDS0 + 4, ~0U);
	return true;
}

void
PieGPIOBCM2711::free(void)
{
	OSSafeReleaseNULL(fMap);
	PieGPIOController::free();
}

void
PieGPIOBCM2711::hwSetFunction(unsigned line, unsigned fsel)
{
	unsigned off = GPFSEL0 + 4 * (line / 10), shift = 3 * (line % 10);
	wr(off, (rd(off) & ~(7U << shift)) | ((fsel & 7) << shift));
}

void
PieGPIOBCM2711::hwSetOutput(unsigned line, bool output)
{
	hwSetFunction(line, output ? 1 : 0);
}

/* Linux's names for them (pinctrl-bcm2835). */
bool
PieGPIOBCM2711::hwFunctionNumber(unsigned, const char *name, unsigned *fsel)
{
	static const char *const names[8] = { "gpio_in", "gpio_out", "alt5", "alt4", "alt0", "alt1", "alt2", "alt3" };
	for (unsigned i = 0; i < 8; i++) {
		if (strcmp(name, names[i]) == 0) {
			*fsel = i;
			return true;
		}
	}
	return false;
}

bool
PieGPIOBCM2711::hwBrcmFunction(unsigned, uint32_t function, unsigned *fsel)
{
	*fsel = function;
	return function < 8;
}

bool
PieGPIOBCM2711::hwIsOutput(unsigned line)
{
	return ((rd(GPFSEL0 + 4 * (line / 10)) >> (3 * (line % 10))) & 7) == 1;
}

bool
PieGPIOBCM2711::hwRead(unsigned line)
{
	return (rd(GPLEV0 + 4 * (line / 32)) >> (line % 32)) & 1;
}

void
PieGPIOBCM2711::hwWrite(unsigned line, bool high)
{
	wr((high ? GPSET0 : GPCLR0) + 4 * (line / 32), 1U << (line % 32));
}

bool
PieGPIOBCM2711::hwSetBias(unsigned line, uint8_t bias)
{
	uint32_t v = bias == 4 ? 1 : bias == 5 ? 2 : 0;         /* pull-up, pull-down, disabled */
	unsigned off = PUP_PDN0 + 4 * (line / 16), shift = 2 * (line % 16);
	wr(off, (rd(off) & ~(3U << shift)) | (v << shift));
	return true;
}

uint8_t
PieGPIOBCM2711::hwGetBias(unsigned line)
{
	switch ((rd(PUP_PDN0 + 4 * (line / 16)) >> (2 * (line % 16))) & 3) {
	case 1:
		return 4;
	case 2:
		return 5;
	case 0:
		return 3;
	default:
		return 2;
	}
}

void
PieGPIOBCM2711::hwSetEdges(unsigned line, bool rising, bool falling)
{
	setBit(GPREN0, line, rising);
	setBit(GPFEN0, line, falling);
	wr(GPEDS0 + 4 * (line / 32), 1U << (line % 32));
}

uint64_t
PieGPIOBCM2711::hwTakeEdges(void)
{
	uint32_t lo = rd(GPEDS0), hi = rd(GPEDS0 + 4) & ((1U << (LINES - 32)) - 1);
	wr(GPEDS0, lo);
	wr(GPEDS0 + 4, hi);
	return lo | ((uint64_t)hi << 32);
}
