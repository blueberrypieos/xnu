/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 *
 * PieGPIORP1: the Raspberry Pi 5's GPIOs, RP1's (raspberrypi,rp1-gpio), 54
 * lines in three banks (0-27, 28-33, 34-53), as the RP1 peripherals
 * datasheet describes them. The node's three register ranges are, per bank
 * 0x4000 apart:
 * - io_bank: a status and a control word per line (8 bytes); control's
 *   function select (5: software I/O, RIO), the overrides, and the edge
 *   interrupt masks (high, low); status's latched edges; per bank, the
 *   interrupts to the host (PCIE_INTE, PCIE_INTS);
 * - sys_rio: the software I/O's output, output enable and synchronised
 *   input words;
 * - pads: a word per line after the bank's voltage select: pull-up and
 *   pull-down enables, input enable, output disable.
 * Each has atomic set and clear aliases (+0x2000, +0x3000). The banks'
 * interrupts come through RP1's interrupt controller (RP1.cpp). A line is
 * switched to software I/O when it is requested (RP1 drives none of them
 * itself; the PHY's reset is driven through here, by RPiGEM). Each line
 * has nine functions (a0-a8); pads support 2, 4, 8 and 12 mA drive. Linux's
 * gpiochip0, pinctrl-rp1. Legacy brcm,function accepts only the known
 * PWM overlay pin/function pairs (hwBrcmFunction).
 */

#include <IOKit/IOLib.h>

#include "PieGPIOController.h"

class PieGPIORP1 : public PieGPIOController
{
	OSDeclareDefaultStructors(PieGPIORP1);

	IOMemoryMap *fMaps[3];
	uintptr_t fIO, fRIO, fPads;
	uint64_t fRisingSeen, fFallingSeen;

	static void bank(unsigned line, unsigned *b, unsigned *i);
	volatile uint32_t *ctrl(unsigned line, unsigned alias = 0);
	volatile uint32_t *status(unsigned line);
	volatile uint32_t *rio(unsigned line, unsigned reg, unsigned alias = 0);
	volatile uint32_t *pad(unsigned line, unsigned alias = 0);

protected:
	virtual bool hwStart(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual unsigned hwInterruptCount(void) APPLE_KEXT_OVERRIDE { return 3; }
	virtual void hwClaim(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual void hwSetOutput(unsigned line, bool output) APPLE_KEXT_OVERRIDE;
	virtual bool hwIsOutput(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual bool hwRead(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual void hwWrite(unsigned line, bool high) APPLE_KEXT_OVERRIDE;
	virtual bool hwSupportsBias(void) APPLE_KEXT_OVERRIDE { return true; }
	virtual bool hwSupportsDrive(uint32_t mA) APPLE_KEXT_OVERRIDE
	{ return mA == 2 || mA == 4 || mA == 8 || mA == 12; }
	virtual bool hwSetBias(unsigned line, uint8_t bias) APPLE_KEXT_OVERRIDE;
	virtual uint8_t hwGetBias(unsigned line) APPLE_KEXT_OVERRIDE;
	virtual void hwSetEdges(unsigned line, bool rising, bool falling) APPLE_KEXT_OVERRIDE;
	virtual uint64_t hwTakeEdges(void) APPLE_KEXT_OVERRIDE;
	virtual bool hwEdgeType(unsigned line, bool *rising) APPLE_KEXT_OVERRIDE;
	virtual bool hwFunctionNumber(unsigned line, const char *name, unsigned *fsel) APPLE_KEXT_OVERRIDE;
	virtual bool hwBrcmFunction(unsigned line, uint32_t function, unsigned *fsel) APPLE_KEXT_OVERRIDE;
	virtual void hwSetFunction(unsigned line, unsigned fsel) APPLE_KEXT_OVERRIDE;
	virtual bool hwSetDrive(unsigned line, uint32_t mA) APPLE_KEXT_OVERRIDE;
	virtual const char *chipLabel(void) APPLE_KEXT_OVERRIDE { return "pinctrl-rp1"; }

public:
	virtual void free(void) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(PieGPIORP1, PieGPIOController);

enum {
	BANK_STRIDE = 0x4000, ALIAS_SET = 0x2000, ALIAS_CLR = 0x3000,
	PCIE_INTE = 0x11c, PCIE_INTS = 0x124,
	RIO_OUT = 0x0, RIO_OE = 0x4, RIO_IN = 0x8,
	CTRL_FUNCSEL = 0x1f, CTRL_OVERRIDES = 0x3f000, FUNCSEL_RIO = 5,
	CTRL_IRQ_EDGE_LOW = 1U << 20, CTRL_IRQ_EDGE_HIGH = 1U << 21, CTRL_IRQRESET = 1U << 28,
	STATUS_OETOPAD = 1U << 13, STATUS_EDGE_LOW = 1U << 20, STATUS_EDGE_HIGH = 1U << 21,
	PAD_PDE = 1U << 2, PAD_PUE = 1U << 3, PAD_DRIVE = 3U << 4, PAD_IE = 1U << 6, PAD_OD = 1U << 7,
	FUNCTIONS = 9,
};
static const struct {
	unsigned first, count;
} sBanks[] = { { 0, 28 }, { 28, 6 }, { 34, 20 } };

/* Each line's functions a0-a8, by their device tree names (NULL: none). */
static const char *const sFunctions[54][FUNCTIONS] = {
	/*  0 */ { "spi0", "dpi", "uart1", "i2c0", NULL, "gpio", "proc_rio", "pio", "spi2" },
	/*  1 */ { "spi0", "dpi", "uart1", "i2c0", NULL, "gpio", "proc_rio", "pio", "spi2" },
	/*  2 */ { "spi0", "dpi", "uart1", "i2c1", "ir", "gpio", "proc_rio", "pio", "spi2" },
	/*  3 */ { "spi0", "dpi", "uart1", "i2c1", "ir", "gpio", "proc_rio", "pio", "spi2" },
	/*  4 */ { "gpclk0", "dpi", "uart2", "i2c2", "ri0", "gpio", "proc_rio", "pio", "spi3" },
	/*  5 */ { "gpclk1", "dpi", "uart2", "i2c2", "dtr0", "gpio", "proc_rio", "pio", "spi3" },
	/*  6 */ { "gpclk2", "dpi", "uart2", "i2c3", "dcd0", "gpio", "proc_rio", "pio", "spi3" },
	/*  7 */ { "spi0", "dpi", "uart2", "i2c3", "dsr0", "gpio", "proc_rio", "pio", "spi3" },
	/*  8 */ { "spi0", "dpi", "uart3", "i2c0", NULL, "gpio", "proc_rio", "pio", "spi4" },
	/*  9 */ { "spi0", "dpi", "uart3", "i2c0", NULL, "gpio", "proc_rio", "pio", "spi4" },
	/* 10 */ { "spi0", "dpi", "uart3", "i2c1", NULL, "gpio", "proc_rio", "pio", "spi4" },
	/* 11 */ { "spi0", "dpi", "uart3", "i2c1", NULL, "gpio", "proc_rio", "pio", "spi4" },
	/* 12 */ { "pwm0", "dpi", "uart4", "i2c2", "aaud", "gpio", "proc_rio", "pio", "spi5" },
	/* 13 */ { "pwm0", "dpi", "uart4", "i2c2", "aaud", "gpio", "proc_rio", "pio", "spi5" },
	/* 14 */ { "pwm0", "dpi", "uart4", "i2c3", "uart0", "gpio", "proc_rio", "pio", "spi5" },
	/* 15 */ { "pwm0", "dpi", "uart4", "i2c3", "uart0", "gpio", "proc_rio", "pio", "spi5" },
	/* 16 */ { "spi1", "dpi", "dsi0_te_ext", NULL, "uart0", "gpio", "proc_rio", "pio", NULL },
	/* 17 */ { "spi1", "dpi", "dsi1_te_ext", NULL, "uart0", "gpio", "proc_rio", "pio", NULL },
	/* 18 */ { "spi1", "dpi", "i2s0", "pwm0", "i2s1", "gpio", "proc_rio", "pio", "gpclk1" },
	/* 19 */ { "spi1", "dpi", "i2s0", "pwm0", "i2s1", "gpio", "proc_rio", "pio", NULL },
	/* 20 */ { "spi1", "dpi", "i2s0", "gpclk0", "i2s1", "gpio", "proc_rio", "pio", NULL },
	/* 21 */ { "spi1", "dpi", "i2s0", "gpclk1", "i2s1", "gpio", "proc_rio", "pio", NULL },
	/* 22 */ { "sd0", "dpi", "i2s0", "i2c3", "i2s1", "gpio", "proc_rio", "pio", NULL },
	/* 23 */ { "sd0", "dpi", "i2s0", "i2c3", "i2s1", "gpio", "proc_rio", "pio", NULL },
	/* 24 */ { "sd0", "dpi", "i2s0", NULL, "i2s1", "gpio", "proc_rio", "pio", "spi2" },
	/* 25 */ { "sd0", "dpi", "i2s0", "mic", "i2s1", "gpio", "proc_rio", "pio", "spi3" },
	/* 26 */ { "sd0", "dpi", "i2s0", "mic", "i2s1", "gpio", "proc_rio", "pio", "spi5" },
	/* 27 */ { "sd0", "dpi", "i2s0", "mic", "i2s1", "gpio", "proc_rio", "pio", "spi1" },
	/* 28 */ { "sd1", "i2c4", "i2s2", "spi6", "vbus0", "gpio", "proc_rio", NULL, NULL },
	/* 29 */ { "sd1", "i2c4", "i2s2", "spi6", "vbus0", "gpio", "proc_rio", NULL, NULL },
	/* 30 */ { "sd1", "i2c5", "i2s2", "spi6", "uart5", "gpio", "proc_rio", NULL, NULL },
	/* 31 */ { "sd1", "i2c5", "i2s2", "spi6", "uart5", "gpio", "proc_rio", NULL, NULL },
	/* 32 */ { "sd1", "gpclk3", "i2s2", "spi6", "uart5", "gpio", "proc_rio", NULL, NULL },
	/* 33 */ { "sd1", "gpclk4", "i2s2", "spi6", "uart5", "gpio", "proc_rio", NULL, NULL },
	/* 34 */ { "pwm1", "gpclk3", "vbus0", "i2c4", "mic", "gpio", "proc_rio", NULL, NULL },
	/* 35 */ { "spi8", "pwm1", "vbus0", "i2c4", "mic", "gpio", "proc_rio", NULL, NULL },
	/* 36 */ { "spi8", "uart5", "pcie_clkreq_n", "i2c5", "mic", "gpio", "proc_rio", NULL, NULL },
	/* 37 */ { "spi8", "uart5", "mic", "i2c5", "pcie_clkreq_n", "gpio", "proc_rio", NULL, NULL },
	/* 38 */ { "spi8", "uart5", "mic", "i2c6", "aaud", "gpio", "proc_rio", "dsi0_te_ext", NULL },
	/* 39 */ { "spi8", "uart5", "mic", "i2c6", "aaud", "gpio", "proc_rio", "dsi1_te_ext", NULL },
	/* 40 */ { "pwm1", "uart5", "i2c4", "spi6", "aaud", "gpio", "proc_rio", NULL, NULL },
	/* 41 */ { "pwm1", "uart5", "i2c4", "spi6", "aaud", "gpio", "proc_rio", NULL, NULL },
	/* 42 */ { "gpclk5", "uart5", "vbus1", "spi6", "i2s2", "gpio", "proc_rio", NULL, NULL },
	/* 43 */ { "gpclk4", "uart5", "vbus1", "spi6", "i2s2", "gpio", "proc_rio", NULL, NULL },
	/* 44 */ { "gpclk5", "i2c5", "pwm1", "spi6", "i2s2", "gpio", "proc_rio", NULL, NULL },
	/* 45 */ { "pwm1", "i2c5", "spi7", "spi6", "i2s2", "gpio", "proc_rio", NULL, NULL },
	/* 46 */ { "gpclk3", "i2c4", "spi7", "mic", "i2s2", "gpio", "proc_rio", "dsi0_te_ext", NULL },
	/* 47 */ { "gpclk5", "i2c4", "spi7", "mic", "i2s2", "gpio", "proc_rio", "dsi1_te_ext", NULL },
	/* 48 */ { "pwm1", "pcie_clkreq_n", "spi7", "mic", "uart5", "gpio", "proc_rio", NULL, NULL },
	/* 49 */ { "spi8", "spi7", "i2c5", "aaud", "uart5", "gpio", "proc_rio", NULL, NULL },
	/* 50 */ { "spi8", "spi7", "i2c5", "aaud", "vbus2", "gpio", "proc_rio", NULL, NULL },
	/* 51 */ { "spi8", "spi7", "i2c6", "aaud", "vbus2", "gpio", "proc_rio", NULL, NULL },
	/* 52 */ { "spi8", NULL, "i2c6", "aaud", "vbus3", "gpio", "proc_rio", NULL, NULL },
	/* 53 */ { "spi8", "spi7", NULL, "pcie_clkreq_n", "vbus3", "gpio", "proc_rio", NULL, NULL },
};

void
PieGPIORP1::bank(unsigned line, unsigned *b, unsigned *i)
{
	*b = line < 28 ? 0 : line < 34 ? 1 : 2;
	*i = line - sBanks[*b].first;
}

volatile uint32_t *
PieGPIORP1::ctrl(unsigned line, unsigned alias)
{
	unsigned b, i;
	bank(line, &b, &i);
	return (volatile uint32_t *)(fIO + BANK_STRIDE * b + alias + 8 * i + 4);
}

volatile uint32_t *
PieGPIORP1::status(unsigned line)
{
	unsigned b, i;
	bank(line, &b, &i);
	return (volatile uint32_t *)(fIO + BANK_STRIDE * b + 8 * i);
}

volatile uint32_t *
PieGPIORP1::rio(unsigned line, unsigned reg, unsigned alias)
{
	unsigned b, i;
	bank(line, &b, &i);
	return (volatile uint32_t *)(fRIO + BANK_STRIDE * b + alias + reg);
}

volatile uint32_t *
PieGPIORP1::pad(unsigned line, unsigned alias)
{
	unsigned b, i;
	bank(line, &b, &i);
	return (volatile uint32_t *)(fPads + BANK_STRIDE * b + alias + 4 + 4 * i);
}

static uint32_t
bit_of(unsigned line)
{
	return 1U << (line < 28 ? line : line < 34 ? line - 28 : line - 34);
}

bool
PieGPIORP1::hwStart(IOService *provider)
{
	for (unsigned m = 0; m < 3; m++) {
		if ((fMaps[m] = provider->mapDeviceMemoryWithIndex(m, kIOMapInhibitCache)) == NULL) {
			IOLog("PieGPIORP1: no register range %u\n", m);
			return false;
		}
	}
	fIO = fMaps[0]->getVirtualAddress();
	fRIO = fMaps[1]->getVirtualAddress();
	fPads = fMaps[2]->getVirtualAddress();
	fLineCount = 54;
	return true;
}

void
PieGPIORP1::free(void)
{
	for (unsigned m = 0; m < 3; m++) {
		OSSafeReleaseNULL(fMaps[m]);
	}
	PieGPIOController::free();
}

/* Software I/O. */
void
PieGPIORP1::hwClaim(unsigned line)
{
	hwSetFunction(line, FUNCSEL_RIO);
}

/* The function, the overrides off (the function drives the pin), the pad's input on and output not disabled. */
void
PieGPIORP1::hwSetFunction(unsigned line, unsigned fsel)
{
	*ctrl(line) = (*ctrl(line) & ~(uint32_t)(CTRL_FUNCSEL | CTRL_OVERRIDES)) | (fsel & CTRL_FUNCSEL);
	*pad(line) = (*pad(line) & ~(uint32_t)PAD_OD) | PAD_IE;
}

/* A function by name, or a0-a8 as "alt0"-"alt8". */
bool
PieGPIORP1::hwFunctionNumber(unsigned line, const char *name, unsigned *fsel)
{
	if (strncmp(name, "alt", 3) == 0 && name[3] >= '0' && name[3] < '0' + FUNCTIONS && name[4] == '\0') {
		*fsel = (unsigned)(name[3] - '0');
		return true;
	}
	for (unsigned f = 0; f < FUNCTIONS; f++) {
		if (sFunctions[line][f] != NULL && strcmp(sFunctions[line][f], name) == 0) {
			*fsel = f;
			return true;
		}
	}
	return false;
}

/*
 * A BCM2835 function number (brcm,function: 4 alt0, 2 alt5, as legacy
 * overlays give them) as RP1's function of the same peripheral on that
 * pin. Only the pwm and pwm-2chan overlays' pins: GPIO12 and 13's alt0 and
 * GPIO18 and 19's alt5, the BCM2835's PWM0, are RP1's pwm0 (a0 on 12 and
 * 13, a3 on 18 and 19). Any other is refused (logged by setPinctrl).
 */
bool
PieGPIORP1::hwBrcmFunction(unsigned line, uint32_t function, unsigned *fsel)
{
	static const struct {
		unsigned line;
		uint32_t function;
		const char *name;
	} sLegacy[] = { { 12, 4, "pwm0" }, { 13, 4, "pwm0" }, { 18, 2, "pwm0" }, { 19, 2, "pwm0" } };
	for (unsigned i = 0; i < sizeof(sLegacy) / sizeof(sLegacy[0]); i++) {
		if (sLegacy[i].line == line && sLegacy[i].function == function) {
			return hwFunctionNumber(line, sLegacy[i].name, fsel);
		}
	}
	return false;
}

bool
PieGPIORP1::hwSetDrive(unsigned line, uint32_t mA)
{
	if (!hwSupportsDrive(mA)) { return false; }
	uint32_t v = mA == 2 ? 0 : mA == 4 ? 1 : mA == 8 ? 2 : 3;
	*pad(line) = (*pad(line) & ~(uint32_t)PAD_DRIVE) | v << 4;
	return true;
}

void
PieGPIORP1::hwSetOutput(unsigned line, bool output)
{
	*rio(line, RIO_OE, output ? ALIAS_SET : ALIAS_CLR) = bit_of(line);
}

bool
PieGPIORP1::hwIsOutput(unsigned line)
{
	if ((*ctrl(line) & CTRL_FUNCSEL) == FUNCSEL_RIO) {
		return (*rio(line, RIO_OE) & bit_of(line)) != 0;
	}
	return (*status(line) & STATUS_OETOPAD) != 0;
}

bool
PieGPIORP1::hwRead(unsigned line)
{
	return (*rio(line, RIO_IN) & bit_of(line)) != 0;
}

void
PieGPIORP1::hwWrite(unsigned line, bool high)
{
	*rio(line, RIO_OUT, high ? ALIAS_SET : ALIAS_CLR) = bit_of(line);
}

bool
PieGPIORP1::hwSetBias(unsigned line, uint8_t bias)
{
	*pad(line, ALIAS_CLR) = PAD_PUE | PAD_PDE;
	if (bias == 4) {
		*pad(line, ALIAS_SET) = PAD_PUE;
	} else if (bias == 5) {
		*pad(line, ALIAS_SET) = PAD_PDE;
	}
	return true;
}

uint8_t
PieGPIORP1::hwGetBias(unsigned line)
{
	uint32_t p = *pad(line) & (PAD_PUE | PAD_PDE);
	return p == PAD_PUE ? 4 : p == PAD_PDE ? 5 : p == 0 ? 3 : 2;
}

void
PieGPIORP1::hwSetEdges(unsigned line, bool rising, bool falling)
{
	unsigned b, i;
	bank(line, &b, &i);
	*ctrl(line, ALIAS_CLR) = CTRL_IRQ_EDGE_HIGH | CTRL_IRQ_EDGE_LOW;
	*ctrl(line, ALIAS_SET) = CTRL_IRQRESET;
	*ctrl(line, ALIAS_SET) = (rising ? CTRL_IRQ_EDGE_HIGH : 0) | (falling ? CTRL_IRQ_EDGE_LOW : 0);
	volatile uint32_t *inte = (volatile uint32_t *)(fIO + BANK_STRIDE * b + (rising || falling ? ALIAS_SET : ALIAS_CLR) +
	    PCIE_INTE);
	*inte = 1U << i;
}

/* Each bank's lines with an interrupt, their edges noted, then reset. */
uint64_t
PieGPIORP1::hwTakeEdges(void)
{
	uint64_t pending = 0;
	fRisingSeen = fFallingSeen = 0;
	for (unsigned b = 0; b < 3; b++) {
		uint32_t ints = *(volatile uint32_t *)(fIO + BANK_STRIDE * b + PCIE_INTS);
		for (unsigned i = 0; i < sBanks[b].count; i++) {
			if (!(ints & (1U << i))) {
				continue;
			}
			unsigned line = sBanks[b].first + i;
			uint32_t st = *status(line);
			if (st & STATUS_EDGE_HIGH) {
				fRisingSeen |= 1ULL << line;
			}
			if (st & STATUS_EDGE_LOW) {
				fFallingSeen |= 1ULL << line;
			}
			*ctrl(line, ALIAS_SET) = CTRL_IRQRESET;
			pending |= 1ULL << line;
		}
	}
	return pending;
}

bool
PieGPIORP1::hwEdgeType(unsigned line, bool *rising)
{
	bool r = (fRisingSeen >> line) & 1, f = (fFallingSeen >> line) & 1;
	if (r == f) {
		return false;
	}
	*rising = r;
	return true;
}
