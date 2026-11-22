/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_RPI_RPISDHCIREGS_H
#define _IOKIT_RPI_RPISDHCIREGS_H

/*
 * The SD host controller registers and bits that RPiSDHCI, the SD card
 * driver, and RPiSDHCISDIO, the SDIO host, use. INT_CARD is the card
 * interrupt, which only the SDIO host enables.
 */
enum {
	SDHCI_ARGUMENT2       = 0x00,
	SDHCI_BLOCK           = 0x04,
	SDHCI_ARGUMENT        = 0x08,
	SDHCI_CMD_XFER        = 0x0C,
	SDHCI_RESPONSE        = 0x10,
	SDHCI_BUFFER          = 0x20,
	SDHCI_PRESENT_STATE   = 0x24,
	SDHCI_HOST_CONTROL    = 0x28,
	SDHCI_CLOCK_CONTROL   = 0x2C,
	SDHCI_INT_STATUS      = 0x30,
	SDHCI_INT_ENABLE      = 0x34,
	SDHCI_SIGNAL_ENABLE   = 0x38,
	SDHCI_HOST_CONTROL2   = 0x3C,
	SDHCI_CAPABILITIES    = 0x40,
	SDHCI_ADMA_ADDRESS    = 0x58,
	SDHCI_VERSION         = 0xFC,
	SDHCI_SDMA_ADDRESS    = SDHCI_ARGUMENT2,
};

enum {
	PRESENT_CMD_INHIBIT = 1U << 0,
	PRESENT_DAT_INHIBIT = 1U << 1,
	PRESENT_CARD        = 1U << 16,

	HC_DATA_4BIT        = 1U << 1,
	HC_HIGH_SPEED       = 1U << 2,
	HC_DMA_MASK         = 3U << 3,
	HC_DMA_SDMA         = 0U << 3,
	HC_DMA_ADMA2        = 2U << 3,
	POWER_ON_3V3        = (0x7U << 1 | 1U) << 8,
	HC2_UHS_MASK        = 7U << 16,
	HC2_UHS_SDR25       = 1U << 16,
	HC2_SIGNAL_1V8      = 1U << 19,

	CAP_ADMA2           = 1U << 19,
	CAP_HIGH_SPEED      = 1U << 21,
	CAP_SDMA            = 1U << 22,

	CLOCK_INTERNAL_EN   = 1U << 0,
	CLOCK_STABLE        = 1U << 1,
	CLOCK_CARD_EN       = 1U << 2,
	TIMEOUT_MAX         = 0xEU << 16,
	RESET_ALL           = 1U << 24,
	RESET_CMD           = 1U << 25,
	RESET_DATA          = 1U << 26,

	INT_CMD_DONE        = 1U << 0,
	INT_XFER_DONE       = 1U << 1,
	INT_DMA             = 1U << 3,
	INT_WRITE_READY     = 1U << 4,
	INT_READ_READY      = 1U << 5,
	INT_ERROR           = 1U << 15,
	INT_ERRORS          = 0xFFFFU << 16,
	INT_ADMA_ERROR      = 1U << 25,

	BLOCK_SDMA_512K     = 7U << 12,

	ADMA_VALID          = 1U << 0,
	ADMA_END            = 1U << 1,
	ADMA_TRAN           = 2U << 4,

	XFER_DMA_EN         = 1U << 0,
	XFER_BLOCK_COUNT_EN = 1U << 1,
	XFER_AUTO_CMD12     = 1U << 2,
	XFER_READ           = 1U << 4,
	XFER_MULTI          = 1U << 5,

	CMD_RESP_NONE       = 0,
	CMD_RESP_136        = 1,
	CMD_RESP_48         = 2,
	CMD_RESP_48_BUSY    = 3,
	CMD_CRC_CHECK       = 1U << 3,
	CMD_INDEX_CHECK     = 1U << 4,
	CMD_DATA            = 1U << 5,
};

enum {
	R_NONE = CMD_RESP_NONE,
	R1     = CMD_RESP_48 | CMD_CRC_CHECK | CMD_INDEX_CHECK,
	R1B    = CMD_RESP_48_BUSY | CMD_CRC_CHECK | CMD_INDEX_CHECK,
	R2     = CMD_RESP_136 | CMD_CRC_CHECK,
	R3     = CMD_RESP_48,
	R6     = R1,
	R7     = R1,
};

enum {
	INT_CARD            = 1U << 8,
	INT_CMD_TIMEOUT     = 1U << 16,
};

#endif
