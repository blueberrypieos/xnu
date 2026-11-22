/*	$OpenBSD: if_ure.c,v 1.39 2026/11/06 09:12:07 kirill Exp $	*/
/*-
 * Copyright (c) 2015, 2016, 2019 Kevin Lo <kevlo@openbsd.org>
 * Copyright (c) 2020 Jonathon Fletcher <jonathon.fletcher@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/* Modified 2026-11-18: an IOEthernetController on the kernel's USB interfaces, for the RTL8152, RTL8153 and RTL8153B. */

#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOWorkLoop.h>
#include <kern/clock.h>
#include <libkern/OSByteOrder.h>
#include <pie/usb.h>

#define __PRIVATE_SPI__ 1

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wshadow-field"
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-html"
#include <sys/kpi_mbuf.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IONetworkMedium.h>
#pragma clang diagnostic pop

#include "if_urereg.h"

#define USB_SPEED_FULL          1
#define USB_SPEED_HIGH          3
#define USB_SPEED_SUPER         4

#define MII_BMCR                0
#define MII_ANAR                4
#define MII_100T2CR             9
#define BMCR_STARTNEG           0x0200
#define BMCR_AUTOEN             0x1000
#define BMCR_RESET              0x8000
#define ANAR_CSMA               0x0001
#define ANAR_10                 0x0020
#define ANAR_10_FD              0x0040
#define ANAR_TX                 0x0080
#define ANAR_TX_FD              0x0100
#define ANAR_FC                 0x0400
#define ANAR_PAUSE_ASYM         0x0800
#define GTCR_ADV_1000THDX       0x0100
#define GTCR_ADV_1000TFDX       0x0200

#define ETHER_MTU_PIE           1500
#define FRAME_MAX               (ETHER_MTU_PIE + 14 + 4 + 4)
#define RX_BUFFERS              8
#define TX_BUFFER               2048
#define TX_BUFFERS              32
#define RX_HDR                  ((uint32_t)sizeof(struct ure_rxpkt))
#define TX_HDR                  ((uint32_t)sizeof(struct ure_txpkt))
#define RECOVER_TRIES           8
#define RECOVER_GOOD_MS         10000

class RTLUSBEthernet : public IOEthernetController
{
	OSDeclareDefaultStructors(RTLUSBEthernet);

	PIEUSBInterface *usb;
	uint8_t epIn, epOut;
	uint32_t usbSpeed;
	uint32_t flags, chip;
	uint32_t rxBufSize;
	bool gone, lostLogged, enabled, linkUp, queuesRunning, gaveUp;
	volatile bool rxHalted, txHalted, rxWorked, txWorked;
	IOWorkLoop *workLoop;
	IOTimerEventSource *linkTimer;
	IOInterruptEventSource *recoverSource;
	IOTimerEventSource *recoverTimer;
	uint32_t rxTries, txTries;
	uint64_t rxRestarted, txRestarted;
	volatile uint64_t rxHaltedAt, txHaltedAt;
	IOEthernetInterface *netif;
	IONetworkStats *netStats;
	IOGatedOutputQueue *txQueue;
	IONetworkMedium *mediumAuto;
	IONetworkMedium *mediums[6];
	IOEthernetAddress mac;
	bool promiscuous;
	uint8_t txFrame[TX_BUFFER];

	IOReturn ctl(bool write, uint16_t val, uint16_t index, void *buf, uint16_t len);
	uint8_t read1(uint16_t reg, uint16_t index);
	uint16_t read2(uint16_t reg, uint16_t index);
	uint32_t read4(uint16_t reg, uint16_t index);
	void write1(uint16_t reg, uint16_t index, uint32_t val);
	void write2(uint16_t reg, uint16_t index, uint32_t val);
	void write4(uint16_t reg, uint16_t index, uint32_t val);
	void set1(uint16_t reg, uint16_t index, uint32_t x) { write1(reg, index, read1(reg, index) | x); }
	void set2(uint16_t reg, uint16_t index, uint32_t x) { write2(reg, index, read2(reg, index) | x); }
	void clr1(uint16_t reg, uint16_t index, uint32_t x) { write1(reg, index, read1(reg, index) & ~x); }
	void clr2(uint16_t reg, uint16_t index, uint32_t x) { write2(reg, index, read2(reg, index) & ~x); }
	void clr4(uint16_t reg, uint16_t index, uint32_t x) { write4(reg, index, read4(reg, index) & ~x); }
	uint16_t phyRead(uint16_t addr);
	void phyWrite(uint16_t addr, uint16_t data);

	void rtl8152Init(void);
	bool rtl8153Init(void);
	bool rtl8153bInit(void);
	bool rtl8152NicReset(void);
	bool rtl8153NicReset(void);
	uint16_t rtl8153PhyStatus(int desired);
	void resetBMU(void);
	void disableTeredo(void);
	void reset(void);
	void mediaInit(void);
	void setRxFilter(void);
	void startAutoneg(void);
	void checkLink(void);
	bool startQueues(void);
	void stopQueues(void);

	static void rxDone(OSObject *target, void *data, uint32_t length, IOReturn status);
	static void txDone(OSObject *target, void *data, uint32_t length, IOReturn status);
	void timerFired(IOTimerEventSource *src);
	void recover(IOInterruptEventSource *src, int count);
	void giveUp(const char *why);
	void lostDevice(void);
	void recoverFired(IOTimerEventSource *src);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	virtual bool createWorkLoop(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;
	virtual IOOutputQueue *createOutputQueue(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn enable(IONetworkInterface *interface) APPLE_KEXT_OVERRIDE;
	virtual IOReturn disable(IONetworkInterface *interface) APPLE_KEXT_OVERRIDE;
	virtual UInt32 outputPacket(mbuf_t m, void *param) APPLE_KEXT_OVERRIDE;
	virtual IOReturn getHardwareAddress(IOEthernetAddress *addr) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setPromiscuousMode(bool active) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setMulticastMode(bool active) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setMulticastList(IOEthernetAddress *addrs, UInt32 count) APPLE_KEXT_OVERRIDE;
};

#define super IOEthernetController
OSDefineMetaClassAndStructors(RTLUSBEthernet, IOEthernetController);

#pragma mark Registers

IOReturn
RTLUSBEthernet::ctl(bool write, uint16_t val, uint16_t index, void *buf, uint16_t len)
{
	if (gone) {
		return kIOReturnNoDevice;
	}
	IOReturn ret = usb->controlRequest(write ? 0x40 : 0xc0, 0x05, val, index, len, buf);
	if (ret == kIOReturnNoDevice) {
		gone = true;
		if (recoverSource != NULL) {
			recoverSource->interruptOccurred(NULL, NULL, 0);
		}
	}
	return ret;
}

uint8_t
RTLUSBEthernet::read1(uint16_t reg, uint16_t index)
{
	uint8_t temp[4] = { 0 };
	ctl(false, reg & ~3, index, temp, 4);
	return (uint8_t)(OSReadLittleInt32(temp, 0) >> ((reg & 3) << 3));
}

uint16_t
RTLUSBEthernet::read2(uint16_t reg, uint16_t index)
{
	uint8_t temp[4] = { 0 };
	ctl(false, reg & ~3, index, temp, 4);
	return (uint16_t)(OSReadLittleInt32(temp, 0) >> ((reg & 2) << 3));
}

uint32_t
RTLUSBEthernet::read4(uint16_t reg, uint16_t index)
{
	uint8_t temp[4] = { 0 };
	ctl(false, reg, index, temp, 4);
	return OSReadLittleInt32(temp, 0);
}

void
RTLUSBEthernet::write1(uint16_t reg, uint16_t index, uint32_t val)
{
	uint16_t byen = URE_BYTE_EN_BYTE;
	uint8_t temp[4];
	uint8_t shift = reg & 3;
	val &= 0xff;
	if (reg & 3) {
		byen <<= shift;
		val <<= (shift << 3);
		reg &= ~3;
	}
	OSWriteLittleInt32(temp, 0, val);
	ctl(true, reg, index | byen, temp, 4);
}

void
RTLUSBEthernet::write2(uint16_t reg, uint16_t index, uint32_t val)
{
	uint16_t byen = URE_BYTE_EN_WORD;
	uint8_t temp[4];
	uint8_t shift = reg & 2;
	val &= 0xffff;
	if (reg & 2) {
		byen <<= shift;
		val <<= (shift << 3);
		reg &= ~3;
	}
	OSWriteLittleInt32(temp, 0, val);
	ctl(true, reg, index | byen, temp, 4);
}

void
RTLUSBEthernet::write4(uint16_t reg, uint16_t index, uint32_t val)
{
	uint8_t temp[4];
	OSWriteLittleInt32(temp, 0, val);
	ctl(true, reg, index | URE_BYTE_EN_DWORD, temp, 4);
}

uint16_t
RTLUSBEthernet::phyRead(uint16_t addr)
{
	write2(URE_PLA_OCP_GPHY_BASE, URE_MCU_TYPE_PLA, addr & 0xf000);
	return read2((addr & 0x0fff) | 0xb000, URE_MCU_TYPE_PLA);
}

void
RTLUSBEthernet::phyWrite(uint16_t addr, uint16_t data)
{
	write2(URE_PLA_OCP_GPHY_BASE, URE_MCU_TYPE_PLA, addr & 0xf000);
	write2((addr & 0x0fff) | 0xb000, URE_MCU_TYPE_PLA, data);
}

#pragma mark Chip set-up

void
RTLUSBEthernet::rtl8152Init(void)
{
	phyWrite(URE_OCP_ALDPS_CONFIG, URE_ENPDNPS | URE_LINKENA | URE_DIS_SDSAVE);
	IOSleep(20);

	if (chip & URE_CHIP_VER_4C00) {
		clr2(URE_PLA_LED_FEATURE, URE_MCU_TYPE_PLA, URE_LED_MODE_MASK);
	}

	clr2(URE_USB_UPS_CTRL, URE_MCU_TYPE_USB, URE_POWER_CUT);
	clr2(URE_USB_PM_CTRL_STATUS, URE_MCU_TYPE_USB, URE_RESUME_INDICATE);

	set2(URE_PLA_PHY_PWR, URE_MCU_TYPE_PLA, URE_TX_10M_IDLE_EN | URE_PFM_PWM_SWITCH);
	uint32_t pwrctrl = read4(URE_PLA_MAC_PWR_CTRL, URE_MCU_TYPE_PLA);
	pwrctrl &= ~URE_MCU_CLK_RATIO_MASK;
	pwrctrl |= URE_MCU_CLK_RATIO | URE_D3_CLK_GATED_EN;
	write4(URE_PLA_MAC_PWR_CTRL, URE_MCU_TYPE_PLA, pwrctrl);
	write2(URE_PLA_GPHY_INTR_IMR, URE_MCU_TYPE_PLA,
	    URE_GPHY_STS_MSK | URE_SPEED_DOWN_MSK | URE_SPDWN_RXDV_MSK | URE_SPDWN_LINKCHG_MSK);

	set2(URE_PLA_RSTTALLY, URE_MCU_TYPE_PLA, URE_TALLY_RESET);

	clr2(URE_USB_USB_CTRL, URE_MCU_TYPE_USB, URE_RX_AGG_DISABLE | URE_RX_ZERO_EN);
}

bool
RTLUSBEthernet::rtl8153Init(void)
{
	uint8_t u1u2[8];
	int i;

	memset(u1u2, 0x00, sizeof(u1u2));
	ctl(true, URE_USB_TOLERANCE, URE_BYTE_EN_SIX_BYTES, u1u2, sizeof(u1u2));

	for (i = 0; i < 500; i++) {
		if (read2(URE_PLA_BOOT_CTRL, URE_MCU_TYPE_PLA) & URE_AUTOLOAD_DONE) {
			break;
		}
		IOSleep(20);
	}
	if (i == 500) {
		IOLog("RTLUSBEthernet: timeout waiting for chip autoload\n");
		return false;
	}

	rtl8153PhyStatus(0);

	if (chip & (URE_CHIP_VER_5C00 | URE_CHIP_VER_5C10 | URE_CHIP_VER_5C20)) {
		phyWrite(URE_OCP_ADC_CFG, URE_CKADSEL_L | URE_ADC_EN | URE_EN_EMI_L);
	}

	rtl8153PhyStatus(URE_PHY_STAT_LAN_ON);

	clr2(URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB, URE_U2P3_ENABLE);

	if (chip & URE_CHIP_VER_5C10) {
		uint16_t reg = read2(URE_USB_SSPHYLINK2, URE_MCU_TYPE_USB);
		reg &= ~URE_PWD_DN_SCALE_MASK;
		reg |= URE_PWD_DN_SCALE(96);
		write2(URE_USB_SSPHYLINK2, URE_MCU_TYPE_USB, reg);

		set1(URE_USB_USB2PHY, URE_MCU_TYPE_USB, URE_USB2PHY_L1 | URE_USB2PHY_SUSPEND);
	} else if (chip & URE_CHIP_VER_5C20) {
		clr1(URE_PLA_DMY_REG0, URE_MCU_TYPE_PLA, URE_ECM_ALDPS);
	}
	if (chip & (URE_CHIP_VER_5C20 | URE_CHIP_VER_5C30)) {
		if (read2(URE_USB_BURST_SIZE, URE_MCU_TYPE_USB)) {
			set1(URE_USB_CSR_DUMMY1, URE_MCU_TYPE_USB, URE_DYNAMIC_BURST);
		} else {
			clr1(URE_USB_CSR_DUMMY1, URE_MCU_TYPE_USB, URE_DYNAMIC_BURST);
		}
	}

	set1(URE_USB_CSR_DUMMY2, URE_MCU_TYPE_USB, URE_EP4_FULL_FC);

	clr2(URE_USB_WDT11_CTRL, URE_MCU_TYPE_USB, URE_TIMER11_EN);

	clr2(URE_PLA_LED_FEATURE, URE_MCU_TYPE_PLA, URE_LED_MODE_MASK);

	uint32_t lpm;
	if ((chip & URE_CHIP_VER_5C10) && usbSpeed != USB_SPEED_SUPER) {
		lpm = URE_LPM_TIMER_500MS;
	} else {
		lpm = URE_LPM_TIMER_500US;
	}
	write1(URE_USB_LPM_CTRL, URE_MCU_TYPE_USB, URE_FIFO_EMPTY_1FB | URE_ROK_EXIT_LPM | lpm);

	uint16_t afe = read2(URE_USB_AFE_CTRL2, URE_MCU_TYPE_USB);
	afe &= ~URE_SEN_VAL_MASK;
	afe |= URE_SEN_VAL_NORMAL | URE_SEL_RXIDLE;
	write2(URE_USB_AFE_CTRL2, URE_MCU_TYPE_USB, afe);

	write2(URE_USB_CONNECT_TIMER, URE_MCU_TYPE_USB, 0x0001);

	clr2(URE_USB_POWER_CUT, URE_MCU_TYPE_USB, URE_PWR_EN | URE_PHASE2_EN);
	clr2(URE_USB_MISC_0, URE_MCU_TYPE_USB, URE_PCUT_STATUS);

	memset(u1u2, 0xff, sizeof(u1u2));
	ctl(true, URE_USB_TOLERANCE, URE_BYTE_EN_SIX_BYTES, u1u2, sizeof(u1u2));

	write2(URE_PLA_MAC_PWR_CTRL, URE_MCU_TYPE_PLA, 0);
	write2(URE_PLA_MAC_PWR_CTRL2, URE_MCU_TYPE_PLA, 0);
	write2(URE_PLA_MAC_PWR_CTRL3, URE_MCU_TYPE_PLA, 0);
	write2(URE_PLA_MAC_PWR_CTRL4, URE_MCU_TYPE_PLA, 0);

	clr2(URE_USB_USB_CTRL, URE_MCU_TYPE_USB, URE_RX_AGG_DISABLE | URE_RX_ZERO_EN);

	set2(URE_PLA_RSTTALLY, URE_MCU_TYPE_PLA, URE_TALLY_RESET);
	return true;
}

bool
RTLUSBEthernet::rtl8153bInit(void)
{
	int i;

	clr2(URE_USB_LPM_CONFIG, URE_MCU_TYPE_USB, LPM_U1U2_EN);

	for (i = 0; i < 500; i++) {
		if (read2(URE_PLA_BOOT_CTRL, URE_MCU_TYPE_PLA) & URE_AUTOLOAD_DONE) {
			break;
		}
		IOSleep(20);
	}
	if (i == 500) {
		IOLog("RTLUSBEthernet: timeout waiting for chip autoload\n");
		return false;
	}

	rtl8153PhyStatus(0);
	rtl8153PhyStatus(URE_PHY_STAT_LAN_ON);

	clr2(URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB, URE_U2P3_ENABLE);

	write2(URE_USB_MSC_TIMER, URE_MCU_TYPE_USB, 4095);

	clr2(URE_USB_POWER_CUT, URE_MCU_TYPE_USB, URE_PWR_EN);
	clr2(URE_USB_MISC_0, URE_MCU_TYPE_USB, URE_PCUT_STATUS);

	clr1(URE_USB_POWER_CUT, URE_MCU_TYPE_USB, URE_UPS_EN | URE_USP_PREWAKE);
	clr1(URE_USB_MISC_2, URE_MCU_TYPE_USB, URE_UPS_FORCE_PWR_DOWN | URE_UPS_NO_UPS);

	clr1(URE_PLA_INDICATE_FALG, URE_MCU_TYPE_PLA, URE_UPCOMING_RUNTIME_D3);
	clr1(URE_PLA_SUSPEND_FLAG, URE_MCU_TYPE_PLA, URE_LINK_CHG_EVENT);
	clr2(URE_PLA_EXTRA_STATUS, URE_MCU_TYPE_PLA, URE_LINK_CHANGE_FLAG);

	write1(URE_PLA_CRWECR, URE_MCU_TYPE_PLA, URE_CRWECR_CONFIG);
	clr2(URE_PLA_CONFIG34, URE_MCU_TYPE_PLA, URE_LINK_OFF_WAKE_EN);
	write1(URE_PLA_CRWECR, URE_MCU_TYPE_PLA, URE_CRWECR_NORAML);

	uint16_t reg = read2(URE_PLA_EXTRA_STATUS, URE_MCU_TYPE_PLA);
	if (read2(URE_PLA_PHYSTATUS, URE_MCU_TYPE_PLA) & URE_PHYSTATUS_LINK) {
		reg |= URE_CUR_LINK_OK;
	} else {
		reg &= ~URE_CUR_LINK_OK;
	}
	write2(URE_PLA_EXTRA_STATUS, URE_MCU_TYPE_PLA, reg | URE_POLL_LINK_CHG);

	if (usbSpeed == USB_SPEED_SUPER) {
		set2(URE_USB_LPM_CONFIG, URE_MCU_TYPE_USB, LPM_U1U2_EN);
	}

	set2(URE_PLA_MAC_PWR_CTRL2, URE_MCU_TYPE_PLA, URE_MAC_CLK_SPDWN_EN);

	clr2(URE_USB_USB_CTRL, URE_MCU_TYPE_USB, URE_RX_AGG_DISABLE | URE_RX_ZERO_EN);

	set2(URE_PLA_RSTTALLY, URE_MCU_TYPE_PLA, URE_TALLY_RESET);
	return true;
}

uint16_t
RTLUSBEthernet::rtl8153PhyStatus(int desired)
{
	uint16_t reg = 0;
	int i;

	for (i = 0; i < 500; i++) {
		reg = phyRead(URE_OCP_PHY_STATUS) & URE_PHY_STAT_MASK;
		if (desired) {
			if (reg == desired) {
				break;
			}
		} else if (reg == URE_PHY_STAT_LAN_ON || reg == URE_PHY_STAT_PWRDN || reg == URE_PHY_STAT_EXT_INIT) {
			break;
		}
		IOSleep(20);
	}
	if (i == 500) {
		IOLog("RTLUSBEthernet: timeout waiting for phy to stabilize\n");
	}
	return reg;
}

void
RTLUSBEthernet::resetBMU(void)
{
	uint8_t reg = read1(URE_USB_BMU_RESET, URE_MCU_TYPE_USB);
	reg &= ~(BMU_RESET_EP_IN | BMU_RESET_EP_OUT);
	write1(URE_USB_BMU_RESET, URE_MCU_TYPE_USB, reg);
	reg |= BMU_RESET_EP_IN | BMU_RESET_EP_OUT;
	write1(URE_USB_BMU_RESET, URE_MCU_TYPE_USB, reg);
}

void
RTLUSBEthernet::disableTeredo(void)
{
	if (flags & URE_FLAG_8153B) {
		write1(URE_PLA_TEREDO_CFG, URE_MCU_TYPE_PLA, 0xff);
	} else {
		clr2(URE_PLA_TEREDO_CFG, URE_MCU_TYPE_PLA, URE_TEREDO_SEL | URE_TEREDO_RS_EVENT_MASK | URE_OOB_TEREDO_EN);
	}
	write2(URE_PLA_WDT6_CTRL, URE_MCU_TYPE_PLA, URE_WDT6_SET_MODE);
	write2(URE_PLA_REALWOW_TIMER, URE_MCU_TYPE_PLA, 0);
	write4(URE_PLA_TEREDO_TIMER, URE_MCU_TYPE_PLA, 0);
}

void
RTLUSBEthernet::reset(void)
{
	int i;

	write1(URE_PLA_CR, URE_MCU_TYPE_PLA, URE_CR_RST);
	for (i = 0; i < URE_TIMEOUT; i++) {
		if (!(read1(URE_PLA_CR, URE_MCU_TYPE_PLA) & URE_CR_RST)) {
			break;
		}
		IODelay(100);
	}
	if (i == URE_TIMEOUT) {
		IOLog("RTLUSBEthernet: reset never completed\n");
	}
}

bool
RTLUSBEthernet::rtl8152NicReset(void)
{
	uint32_t rx_fifo1, rx_fifo2;
	int i;

	phyWrite(URE_OCP_ALDPS_CONFIG, URE_ENPDNPS | URE_LINKENA | URE_DIS_SDSAVE);
	IOSleep(20);

	clr4(URE_PLA_RCR, URE_MCU_TYPE_PLA, URE_RCR_ACPT_ALL);

	set2(URE_PLA_MISC_1, URE_MCU_TYPE_PLA, URE_RXDY_GATED_EN);
	disableTeredo();
	write1(URE_PLA_CRWECR, URE_MCU_TYPE_PLA, URE_CRWECR_NORAML);
	write1(URE_PLA_CR, URE_MCU_TYPE_PLA, 0);

	clr1(URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA, URE_NOW_IS_OOB);
	clr2(URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA, URE_MCU_BORW_EN);
	for (i = 0; i < URE_TIMEOUT; i++) {
		if (read1(URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA) & URE_LINK_LIST_READY) {
			break;
		}
		IOSleep(1);
	}
	if (i == URE_TIMEOUT) {
		IOLog("RTLUSBEthernet: timeout waiting for OOB control\n");
		return false;
	}
	set2(URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA, URE_RE_INIT_LL);
	for (i = 0; i < URE_TIMEOUT; i++) {
		if (read1(URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA) & URE_LINK_LIST_READY) {
			break;
		}
		IOSleep(1);
	}
	if (i == URE_TIMEOUT) {
		IOLog("RTLUSBEthernet: timeout waiting for OOB control\n");
		return false;
	}

	reset();

	write4(URE_PLA_RXFIFO_CTRL0, URE_MCU_TYPE_PLA, URE_RXFIFO_THR1_NORMAL);
	if (usbSpeed == USB_SPEED_FULL) {
		rx_fifo1 = URE_RXFIFO_THR2_FULL;
		rx_fifo2 = URE_RXFIFO_THR3_FULL;
	} else {
		rx_fifo1 = URE_RXFIFO_THR2_HIGH;
		rx_fifo2 = URE_RXFIFO_THR3_HIGH;
	}
	write4(URE_PLA_RXFIFO_CTRL1, URE_MCU_TYPE_PLA, rx_fifo1);
	write4(URE_PLA_RXFIFO_CTRL2, URE_MCU_TYPE_PLA, rx_fifo2);

	write4(URE_PLA_TXFIFO_CTRL, URE_MCU_TYPE_PLA, URE_TXFIFO_THR_NORMAL);

	write1(URE_USB_TX_AGG, URE_MCU_TYPE_USB, URE_TX_AGG_MAX_THRESHOLD);
	write4(URE_USB_RX_BUF_TH, URE_MCU_TYPE_USB, URE_RX_THR_HIGH);
	write4(URE_USB_TX_DMA, URE_MCU_TYPE_USB, URE_TEST_MODE_DISABLE | URE_TX_SIZE_ADJUST1);

	clr2(URE_PLA_CPCR, URE_MCU_TYPE_PLA, URE_CPCR_RX_VLAN);
	write2(URE_PLA_RMS, URE_MCU_TYPE_PLA, FRAME_MAX);
	set2(URE_PLA_TCR0, URE_MCU_TYPE_PLA, URE_TCR0_AUTO_FIFO);

	phyWrite(URE_OCP_ALDPS_CONFIG, URE_ENPWRSAVE | URE_ENPDNPS | URE_LINKENA | URE_DIS_SDSAVE);
	return true;
}

bool
RTLUSBEthernet::rtl8153NicReset(void)
{
	uint8_t u1u2[8] = { 0 };
	int i;

	if (flags & URE_FLAG_8153B) {
		clr2(URE_USB_LPM_CONFIG, URE_MCU_TYPE_USB, LPM_U1U2_EN);
	} else {
		memset(u1u2, 0x00, sizeof(u1u2));
		ctl(true, URE_USB_TOLERANCE, URE_BYTE_EN_SIX_BYTES, u1u2, sizeof(u1u2));
	}
	clr2(URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB, URE_U2P3_ENABLE);

	phyWrite(URE_OCP_POWER_CFG, phyRead(URE_OCP_POWER_CFG) & ~URE_EN_ALDPS);
	for (i = 0; i < 20; i++) {
		IOSleep(1);
		if (read2(0xe000, URE_MCU_TYPE_PLA) & 0x0100) {
			break;
		}
	}

	set2(URE_PLA_MISC_1, URE_MCU_TYPE_PLA, URE_RXDY_GATED_EN);
	disableTeredo();

	clr4(URE_PLA_RCR, URE_MCU_TYPE_PLA, URE_RCR_ACPT_ALL);

	reset();
	resetBMU();

	clr1(URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA, URE_NOW_IS_OOB);
	clr2(URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA, URE_MCU_BORW_EN);

	for (i = 0; i < URE_TIMEOUT; i++) {
		if (read1(URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA) & URE_LINK_LIST_READY) {
			break;
		}
		IOSleep(1);
	}
	if (i == URE_TIMEOUT) {
		IOLog("RTLUSBEthernet: timeout waiting for OOB control\n");
		return false;
	}
	set2(URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA, URE_RE_INIT_LL);
	for (i = 0; i < URE_TIMEOUT; i++) {
		if (read1(URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA) & URE_LINK_LIST_READY) {
			break;
		}
		IOSleep(1);
	}
	if (i == URE_TIMEOUT) {
		IOLog("RTLUSBEthernet: timeout waiting for OOB control\n");
		return false;
	}

	clr2(URE_PLA_CPCR, URE_MCU_TYPE_PLA, URE_CPCR_RX_VLAN);

	write2(URE_PLA_RMS, URE_MCU_TYPE_PLA, FRAME_MAX);
	write1(URE_PLA_MTPS, URE_MCU_TYPE_PLA, MTPS_JUMBO);

	set2(URE_PLA_TCR0, URE_MCU_TYPE_PLA, URE_TCR0_AUTO_FIFO);
	reset();

	write4(URE_PLA_RXFIFO_CTRL0, URE_MCU_TYPE_PLA, URE_RXFIFO_THR1_NORMAL);
	write2(URE_PLA_RXFIFO_CTRL1, URE_MCU_TYPE_PLA, URE_RXFIFO_THR2_NORMAL);
	write2(URE_PLA_RXFIFO_CTRL2, URE_MCU_TYPE_PLA, URE_RXFIFO_THR3_NORMAL);

	write4(URE_PLA_TXFIFO_CTRL, URE_MCU_TYPE_PLA, URE_TXFIFO_THR_NORMAL2);

	if (flags & URE_FLAG_8153B) {
		write4(URE_USB_RX_BUF_TH, URE_MCU_TYPE_USB, URE_RX_THR_B);
		clr2(URE_PLA_MAC_PWR_CTRL3, URE_MCU_TYPE_PLA, URE_PLA_MCU_SPDWN_EN);
	} else {
		set1(URE_PLA_CONFIG6, URE_MCU_TYPE_PLA, URE_LANWAKE_CLR_EN);
		clr1(URE_PLA_LWAKE_CTRL_REG, URE_MCU_TYPE_PLA, URE_LANWAKE_PIN);
		clr2(URE_USB_SSPHYLINK1, URE_MCU_TYPE_USB, URE_DELAY_PHY_PWR_CHG);
	}

	phyWrite(URE_OCP_POWER_CFG, phyRead(URE_OCP_POWER_CFG) | URE_EN_ALDPS);

	if (chip & (URE_CHIP_VER_5C20 | URE_CHIP_VER_5C30)) {
		set2(URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB, URE_U2P3_ENABLE);
	}

	if (flags & URE_FLAG_8153B) {
		if (usbSpeed == USB_SPEED_SUPER) {
			set2(URE_USB_LPM_CONFIG, URE_MCU_TYPE_USB, LPM_U1U2_EN);
		}
	} else {
		memset(u1u2, 0xff, sizeof(u1u2));
		ctl(true, URE_USB_TOLERANCE, URE_BYTE_EN_SIX_BYTES, u1u2, sizeof(u1u2));
	}
	return true;
}

void
RTLUSBEthernet::mediaInit(void)
{
	uint8_t addr[8] = { 0 };
	uint32_t reg;

	bcopy(mac.bytes, addr, 6);
	write1(URE_PLA_CRWECR, URE_MCU_TYPE_PLA, URE_CRWECR_CONFIG);
	ctl(true, URE_PLA_IDR, URE_MCU_TYPE_PLA | URE_BYTE_EN_SIX_BYTES, addr, sizeof(addr));
	write1(URE_PLA_CRWECR, URE_MCU_TYPE_PLA, URE_CRWECR_NORAML);

	if (!(flags & URE_FLAG_8152)) {
		reg = rxBufSize - FRAME_MAX - RX_HDR - URE_RX_BUF_ALIGN;
		if (flags & URE_FLAG_8153B) {
			write2(URE_USB_RX_EARLY_SIZE, URE_MCU_TYPE_USB, reg / URE_RX_BUF_ALIGN);
			write2(URE_USB_RX_EARLY_AGG, URE_MCU_TYPE_USB, 158);
			write2(URE_USB_PM_CTRL_STATUS, URE_MCU_TYPE_USB, 1875);
			if (usbSpeed == USB_SPEED_HIGH) {
				reg = read2(URE_USB_L1_CTRL, URE_MCU_TYPE_USB);
				reg &= ~0x0f;
				write2(URE_USB_L1_CTRL, URE_MCU_TYPE_USB, reg | 0x01);
			}
		} else {
			write2(URE_USB_RX_EARLY_SIZE, URE_MCU_TYPE_USB, reg / 4);
			switch (usbSpeed) {
			case USB_SPEED_SUPER:
				reg = URE_COALESCE_SUPER / 8;
				break;
			case USB_SPEED_HIGH:
				reg = URE_COALESCE_HIGH / 8;
				break;
			default:
				reg = URE_COALESCE_SLOW / 8;
				break;
			}
			write2(URE_USB_RX_EARLY_AGG, URE_MCU_TYPE_USB, reg);
		}
	}

	clr2(URE_PLA_FMC, URE_MCU_TYPE_PLA, URE_FMC_FCR_MCU_EN);
	set2(URE_PLA_FMC, URE_MCU_TYPE_PLA, URE_FMC_FCR_MCU_EN);

	set1(URE_PLA_CR, URE_MCU_TYPE_PLA, URE_CR_RE | URE_CR_TE);

	if (flags & URE_FLAG_8153B) {
		write1(URE_USB_UPT_RXDMA_OWN, URE_MCU_TYPE_USB, URE_OWN_UPDATE | URE_OWN_CLEAR);
	}

	clr2(URE_PLA_MISC_1, URE_MCU_TYPE_PLA, URE_RXDY_GATED_EN);
}

void
RTLUSBEthernet::setRxFilter(void)
{
	uint8_t hashes[8];
	uint32_t rxmode = read4(URE_PLA_RCR, URE_MCU_TYPE_PLA);
	rxmode &= ~URE_RCR_ACPT_ALL;
	rxmode |= URE_RCR_APM | URE_RCR_AB | URE_RCR_AM;
	if (promiscuous) {
		rxmode |= URE_RCR_AAP;
	}
	memset(hashes, 0xff, sizeof(hashes));
	ctl(true, URE_PLA_MAR, URE_MCU_TYPE_PLA | URE_BYTE_EN_DWORD, hashes, sizeof(hashes));
	write4(URE_PLA_RCR, URE_MCU_TYPE_PLA, rxmode);
}

void
RTLUSBEthernet::startAutoneg(void)
{
	phyWrite(URE_OCP_BASE_MII + MII_BMCR * 2, BMCR_RESET);
	for (int i = 0; i < URE_PHY_TIMEOUT; i++) {
		IOSleep(1);
		if (!(phyRead(URE_OCP_BASE_MII + MII_BMCR * 2) & BMCR_RESET)) {
			break;
		}
	}
	phyWrite(URE_OCP_BASE_MII + MII_ANAR * 2,
	    ANAR_CSMA | ANAR_10 | ANAR_10_FD | ANAR_TX | ANAR_TX_FD | ANAR_FC | ANAR_PAUSE_ASYM);
	if (!(flags & URE_FLAG_8152)) {
		phyWrite(URE_OCP_BASE_MII + MII_100T2CR * 2, GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX);
	}
	phyWrite(URE_OCP_BASE_MII + MII_BMCR * 2, BMCR_AUTOEN | BMCR_STARTNEG);
}

void
RTLUSBEthernet::checkLink(void)
{
	uint16_t status = read2(URE_PLA_PHYSTATUS, URE_MCU_TYPE_PLA);
	bool up = !gone && (status & URE_PHYSTATUS_LINK) != 0;
	if (up == linkUp) {
		return;
	}
	linkUp = up;
	if (!up) {
		IOLog("RTLUSBEthernet: link down\n");
		setLinkStatus(kIONetworkLinkValid);
		return;
	}
	mediaInit();
	if (gone) {
		linkUp = false;
		return;
	}
	uint32_t mbps = (status & URE_PHYSTATUS_1000MBPS) ? 1000 : (status & URE_PHYSTATUS_100MBPS) ? 100 : 10;
	bool full = (status & URE_PHYSTATUS_FDX) != 0;
	uint32_t index = (mbps == 1000 ? 4 : mbps == 100 ? 2 : 0) + (full ? 1 : 0);
	IOLog("RTLUSBEthernet: link up, %u Mb/s, %s duplex\n", mbps, full ? "full" : "half");
	setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, mediums[index], (UInt64)mbps * 1000000ULL);
}

#pragma mark Transfers

static uint64_t
uptime_ms(void)
{
	uint64_t abs, ns;
	clock_get_uptime(&abs);
	absolutetime_to_nanoseconds(abs, &ns);
	return ns / 1000000;
}

void
RTLUSBEthernet::rxDone(OSObject *target, void *data, uint32_t length, IOReturn status)
{
	RTLUSBEthernet *self = (RTLUSBEthernet *)target;
	if (status != kIOReturnSuccess) {
		if (!self->rxHalted) {
			self->rxHaltedAt = uptime_ms();
			self->rxHalted = true;
		}
		self->recoverSource->interruptOccurred(NULL, NULL, 0);
		return;
	}
	self->rxWorked = true;
	if (!self->enabled) {
		return;
	}
	const uint8_t *buf = (const uint8_t *)data;
	uint32_t off = 0, errors = 0;
	bool any = false;
	while (off + RX_HDR <= length) {
		uint8_t hdr[4];
		bcopy(buf + off, hdr, sizeof(hdr));
		uint32_t pktlen = OSReadLittleInt32(hdr, 0) & URE_RXPKT_LEN_MASK;
		off += RX_HDR;
		if (pktlen < 14 + 4 || pktlen > length - off) {
			errors++;
			break;
		}
		uint32_t frame = pktlen - 4;
		mbuf_t m = frame <= FRAME_MAX ? self->allocatePacketNoWait(frame) : NULL;
		if (m != NULL && mbuf_copyback(m, 0, frame, buf + off, MBUF_DONTWAIT) == 0) {
			self->netif->inputPacket(m, frame, IONetworkInterface::kInputOptionQueuePacket);
			any = true;
		} else {
			if (m != NULL) {
				self->freePacket(m);
			}
			errors++;
		}
		off += (pktlen + URE_RX_BUF_ALIGN - 1) & ~(uint32_t)(URE_RX_BUF_ALIGN - 1);
	}
	if (errors != 0 && self->netStats != NULL) {
		self->netStats->inputErrors += errors;
	}
	if (any) {
		self->netif->flushInputQueue();
	}
}

void
RTLUSBEthernet::txDone(OSObject *target, void *data, uint32_t length, IOReturn status)
{
	(void)data;
	(void)length;
	RTLUSBEthernet *self = (RTLUSBEthernet *)target;
	if (status != kIOReturnSuccess) {
		if (self->netStats != NULL) {
			self->netStats->outputErrors++;
		}
		if (!self->txHalted) {
			self->txHaltedAt = uptime_ms();
			self->txHalted = true;
		}
		self->recoverSource->interruptOccurred(NULL, NULL, 0);
		return;
	}
	self->txWorked = true;
	if (self->enabled) {
		self->txQueue->service(IOBasicOutputQueue::kServiceAsync);
	}
}

bool
RTLUSBEthernet::startQueues(void)
{
	rxHalted = txHalted = false;
	IOReturn ret = usb->startBulkQueue(epIn, rxBufSize, RX_BUFFERS, this, &RTLUSBEthernet::rxDone);
	if (ret != kIOReturnSuccess) {
		IOLog("RTLUSBEthernet: cannot start receiving (0x%x)\n", ret);
		if (ret == kIOReturnNoDevice) {
			lostDevice();
		}
		return false;
	}
	ret = usb->startBulkQueue(epOut, TX_BUFFER, TX_BUFFERS, this, &RTLUSBEthernet::txDone);
	if (ret != kIOReturnSuccess) {
		IOLog("RTLUSBEthernet: cannot start sending (0x%x)\n", ret);
		usb->stopBulkQueue(epIn);
		if (ret == kIOReturnNoDevice) {
			lostDevice();
		}
		return false;
	}
	queuesRunning = true;
	return true;
}

void
RTLUSBEthernet::stopQueues(void)
{
	if (!queuesRunning) {
		return;
	}
	queuesRunning = false;
	usb->stopBulkQueue(epIn);
	usb->stopBulkQueue(epOut);
}

void
RTLUSBEthernet::recover(IOInterruptEventSource *src, int count)
{
	(void)src;
	(void)count;
	if (gone) {
		lostDevice();
		return;
	}
	if (!enabled || !queuesRunning || gaveUp) {
		return;
	}
	if (rxHalted) {
		if (rxWorked && rxHaltedAt >= rxRestarted && rxHaltedAt - rxRestarted >= RECOVER_GOOD_MS) {
			rxTries = 0;
		}
		rxWorked = false;
	}
	if (txHalted) {
		if (txWorked && txHaltedAt >= txRestarted && txHaltedAt - txRestarted >= RECOVER_GOOD_MS) {
			txTries = 0;
		}
		txWorked = false;
	}
	uint32_t tries = rxHalted ? rxTries : 0;
	if (txHalted && txTries > tries) {
		tries = txTries;
	}
	if (tries >= RECOVER_TRIES) {
		giveUp("transfers halted too often; giving up until the interface is brought up again");
		return;
	}
	recoverTimer->setTimeoutMS(100U << tries);
}

void
RTLUSBEthernet::giveUp(const char *why)
{
	IOLog("RTLUSBEthernet: %s\n", why);
	gaveUp = true;
	recoverTimer->cancelTimeout();
	linkUp = false;
	setLinkStatus(kIONetworkLinkValid);
}

void
RTLUSBEthernet::lostDevice(void)
{
	if (!lostLogged) {
		lostLogged = true;
		IOLog("RTLUSBEthernet: the adapter does not answer\n");
	}
	gone = true;
	recoverTimer->cancelTimeout();
	linkTimer->cancelTimeout();
	linkUp = false;
	setLinkStatus(kIONetworkLinkValid);
}

void
RTLUSBEthernet::recoverFired(IOTimerEventSource *src)
{
	(void)src;
	if (!enabled || gone || !queuesRunning || gaveUp) {
		return;
	}
	bool failed = false;
	IOReturn ret;
	if (rxHalted) {
		IOLog("RTLUSBEthernet: receive halted; starting it again\n");
		ret = usb->stopBulkQueue(epIn);
		if (ret == kIOReturnNoDevice) {
			lostDevice();
			return;
		}
		if (ret != kIOReturnSuccess && ret != kIOReturnNotFound) {
			giveUp("receive did not stop; giving up until the adapter is plugged in again");
			return;
		}
		rxTries++;
		rxWorked = false;
		rxHalted = false;
		ret = usb->clearHalt(epIn);
		bool cleared = ret == kIOReturnSuccess;
		if (cleared) {
			ret = usb->startBulkQueue(epIn, rxBufSize, RX_BUFFERS, this, &RTLUSBEthernet::rxDone);
		}
		if (ret == kIOReturnNoDevice) {
			lostDevice();
			return;
		}
		if (cleared && ret == kIOReturnTimeout) {
			giveUp("receive did not start: the USB controller takes no more commands until a reboot");
			return;
		}
		if (cleared && ret == kIOReturnBusy) {
			giveUp("receive did not start; giving up until the adapter is plugged in again");
			return;
		}
		if (ret != kIOReturnSuccess) {
			IOLog("RTLUSBEthernet: cannot start receiving again\n");
			rxHalted = true;
			failed = true;
		} else {
			rxRestarted = uptime_ms();
		}
	}
	if (txHalted) {
		IOLog("RTLUSBEthernet: send halted; starting it again\n");
		ret = usb->stopBulkQueue(epOut);
		if (ret == kIOReturnNoDevice) {
			lostDevice();
			return;
		}
		if (ret != kIOReturnSuccess && ret != kIOReturnNotFound) {
			giveUp("send did not stop; giving up until the adapter is plugged in again");
			return;
		}
		txTries++;
		txWorked = false;
		txHalted = false;
		ret = usb->clearHalt(epOut);
		bool cleared = ret == kIOReturnSuccess;
		if (cleared) {
			ret = usb->startBulkQueue(epOut, TX_BUFFER, TX_BUFFERS, this, &RTLUSBEthernet::txDone);
		}
		if (ret == kIOReturnNoDevice) {
			lostDevice();
			return;
		}
		if (cleared && ret == kIOReturnTimeout) {
			giveUp("send did not start: the USB controller takes no more commands until a reboot");
			return;
		}
		if (cleared && ret == kIOReturnBusy) {
			giveUp("send did not start; giving up until the adapter is plugged in again");
			return;
		}
		if (ret != kIOReturnSuccess) {
			IOLog("RTLUSBEthernet: cannot start sending again\n");
			txHalted = true;
			failed = true;
		} else {
			txRestarted = uptime_ms();
		}
		txQueue->service(IOBasicOutputQueue::kServiceAsync);
	}
	if (failed) {
		recover(NULL, 0);
	}
}

void
RTLUSBEthernet::timerFired(IOTimerEventSource *src)
{
	if (enabled && !gone && !gaveUp) {
		checkLink();
		src->setTimeoutMS(1000);
	}
}

UInt32
RTLUSBEthernet::outputPacket(mbuf_t m, void *param)
{
	(void)param;
	size_t len = mbuf_pkthdr_len(m);
	if (!linkUp || gone || txHalted || len < 14 || len > FRAME_MAX - 4) {
		freePacket(m);
		if (netStats != NULL) {
			netStats->outputErrors++;
		}
		return kIOReturnOutputDropped;
	}
	OSWriteLittleInt32(txFrame, 0, (uint32_t)len | URE_TXPKT_TX_FS | URE_TXPKT_TX_LS);
	OSWriteLittleInt32(txFrame, 4, 0);
	mbuf_copydata(m, 0, len, txFrame + TX_HDR);
	IOReturn ret = usb->queueBulkOut(epOut, txFrame, (uint32_t)(TX_HDR + len));
	if (ret == kIOReturnNoResources) {
		return kIOReturnOutputStall;
	}
	freePacket(m);
	if (ret != kIOReturnSuccess) {
		if (ret == kIOReturnNoDevice && !gone) {
			gone = true;
			recoverSource->interruptOccurred(NULL, NULL, 0);
		}
		if (netStats != NULL) {
			netStats->outputErrors++;
		}
		return kIOReturnOutputDropped;
	}
	if (netStats != NULL) {
		netStats->outputPackets++;
	}
	return kIOReturnOutputSuccess;
}

#pragma mark IOService

bool
RTLUSBEthernet::start(IOService *provider)
{
	PIEUSBInterface *intf = OSDynamicCast(PIEUSBInterface, provider);
	if (intf == NULL || !super::start(provider)) {
		return false;
	}
	usb = intf;
	usb->retain();
	for (unsigned i = 0; i < usb->numEndpoints; i++) {
		const struct PIEUSBEndpoint *ep = &usb->endpoints[i];
		if ((ep->attributes & 3) != 2) {
			continue;
		}
		if (ep->address & 0x80) {
			epIn = ep->address;
		} else {
			epOut = ep->address;
		}
	}
	OSNumber *speed = OSDynamicCast(OSNumber, usb->getProperty("speed"));
	usbSpeed = speed != NULL ? speed->unsigned32BitValue() : 0;
	if (epIn == 0 || epOut == 0) {
		IOLog("RTLUSBEthernet: no bulk endpoints\n");
		return false;
	}

	rxBufSize = URE_8153_RX_BUFSZ;
	uint16_t ver = read2(URE_PLA_TCR1, URE_MCU_TYPE_PLA) & URE_VERSION_MASK;
	const char *name;
	switch (ver) {
	case 0x4c00:
		flags = URE_FLAG_8152;
		rxBufSize = URE_8152_RX_BUFSZ;
		chip |= URE_CHIP_VER_4C00;
		name = "RTL8152";
		break;
	case 0x4c10:
		flags = URE_FLAG_8152;
		rxBufSize = URE_8152_RX_BUFSZ;
		chip |= URE_CHIP_VER_4C10;
		name = "RTL8152";
		break;
	case 0x5c00:
		chip |= URE_CHIP_VER_5C00;
		name = "RTL8153";
		break;
	case 0x5c10:
		chip |= URE_CHIP_VER_5C10;
		name = "RTL8153";
		break;
	case 0x5c20:
		chip |= URE_CHIP_VER_5C20;
		name = "RTL8153";
		break;
	case 0x5c30:
		chip |= URE_CHIP_VER_5C30;
		name = "RTL8153";
		break;
	case 0x6000:
	case 0x6010:
		flags = URE_FLAG_8153B;
		name = "RTL8153B";
		break;
	default:
		IOLog("RTLUSBEthernet: chip version 0x%04x is not supported\n", ver);
		return false;
	}

	bool ok;
	if (flags & URE_FLAG_8152) {
		rtl8152Init();
		ok = true;
	} else if (flags & URE_FLAG_8153B) {
		ok = rtl8153bInit();
	} else {
		ok = rtl8153Init();
	}
	uint8_t eaddr[8] = { 0 };
	if (ok) {
		ctl(false, (chip & URE_CHIP_VER_4C00) ? URE_PLA_IDR : URE_PLA_BACKUP, URE_MCU_TYPE_PLA, eaddr, sizeof(eaddr));
	}
	if (!ok || gone) {
		return false;
	}
	bcopy(eaddr, mac.bytes, 6);

	linkTimer = IOTimerEventSource::timerEventSource(this,
	    OSMemberFunctionCast(IOTimerEventSource::Action, this, &RTLUSBEthernet::timerFired));
	recoverSource = IOInterruptEventSource::interruptEventSource(this,
	    OSMemberFunctionCast(IOInterruptEventAction, this, &RTLUSBEthernet::recover));
	recoverTimer = IOTimerEventSource::timerEventSource(this,
	    OSMemberFunctionCast(IOTimerEventSource::Action, this, &RTLUSBEthernet::recoverFired));
	if (linkTimer == NULL || recoverSource == NULL || recoverTimer == NULL ||
	    workLoop->addEventSource(linkTimer) != kIOReturnSuccess ||
	    workLoop->addEventSource(recoverSource) != kIOReturnSuccess ||
	    workLoop->addEventSource(recoverTimer) != kIOReturnSuccess) {
		return false;
	}
	recoverSource->enable();

	static const struct { IOMediumType type; UInt64 speed; } media[6] = {
		{ kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex, 10 },
		{ kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex, 10 },
		{ kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex, 100 },
		{ kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex, 100 },
		{ kIOMediumEthernet1000BaseT | kIOMediumOptionHalfDuplex, 1000 },
		{ kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex, 1000 },
	};
	OSDictionary *dict = OSDictionary::withCapacity(7);
	mediumAuto = IONetworkMedium::medium(kIOMediumEthernetAuto, 0);
	if (dict == NULL || mediumAuto == NULL || !IONetworkMedium::addMedium(dict, mediumAuto)) {
		OSSafeReleaseNULL(dict);
		return false;
	}
	for (int i = 0; i < 6; i++) {
		mediums[i] = IONetworkMedium::medium(media[i].type, media[i].speed * 1000000ULL);
		if (mediums[i] == NULL || !IONetworkMedium::addMedium(dict, mediums[i])) {
			dict->release();
			return false;
		}
	}
	bool published = publishMediumDictionary(dict);
	dict->release();
	if (!published) {
		return false;
	}
	setSelectedMedium(mediumAuto);

	if (!attachInterface((IONetworkInterface **)&netif, false)) {
		return false;
	}
	IOLog("RTLUSBEthernet: %s (0x%04x), %02x:%02x:%02x:%02x:%02x:%02x, USB %s speed\n", name, ver,
	    mac.bytes[0], mac.bytes[1], mac.bytes[2], mac.bytes[3], mac.bytes[4], mac.bytes[5],
	    usbSpeed == USB_SPEED_SUPER ? "super" : usbSpeed == USB_SPEED_HIGH ? "high" : "full");
	IONetworkData *stats = netif->getParameter(kIONetworkStatsKey);
	netStats = stats != NULL ? (IONetworkStats *)stats->getBuffer() : NULL;
	netif->registerService();
	return true;
}

void
RTLUSBEthernet::stop(IOService *provider)
{
	if (linkTimer != NULL) {
		linkTimer->cancelTimeout();
	}
	if (recoverTimer != NULL) {
		recoverTimer->cancelTimeout();
	}
	stopQueues();
	netStats = NULL;
	OSSafeReleaseNULL(netif);
	super::stop(provider);
}

void
RTLUSBEthernet::free(void)
{
	if (workLoop != NULL) {
		if (linkTimer != NULL) {
			workLoop->removeEventSource(linkTimer);
		}
		if (recoverSource != NULL) {
			workLoop->removeEventSource(recoverSource);
		}
		if (recoverTimer != NULL) {
			workLoop->removeEventSource(recoverTimer);
		}
	}
	OSSafeReleaseNULL(linkTimer);
	OSSafeReleaseNULL(recoverSource);
	OSSafeReleaseNULL(recoverTimer);
	OSSafeReleaseNULL(netif);
	OSSafeReleaseNULL(txQueue);
	OSSafeReleaseNULL(mediumAuto);
	for (int i = 0; i < 6; i++) {
		OSSafeReleaseNULL(mediums[i]);
	}
	OSSafeReleaseNULL(workLoop);
	OSSafeReleaseNULL(usb);
	super::free();
}

#pragma mark IONetworkController

bool
RTLUSBEthernet::createWorkLoop(void)
{
	workLoop = IOWorkLoop::workLoop();
	return workLoop != NULL;
}

IOWorkLoop *
RTLUSBEthernet::getWorkLoop(void) const
{
	return workLoop;
}

IOOutputQueue *
RTLUSBEthernet::createOutputQueue(void)
{
	txQueue = IOGatedOutputQueue::withTarget(this, getWorkLoop());
	if (txQueue != NULL) {
		txQueue->retain();
	}
	return txQueue;
}

IOReturn
RTLUSBEthernet::enable(IONetworkInterface *interface)
{
	(void)interface;
	if (enabled) {
		return kIOReturnSuccess;
	}
	if (gone) {
		return kIOReturnNoDevice;
	}
	reset();
	bool ok = (flags & URE_FLAG_8152) ? rtl8152NicReset() : rtl8153NicReset();
	if (!ok) {
		return kIOReturnIOError;
	}
	mediaInit();
	setRxFilter();
	linkUp = false;
	gaveUp = false;
	rxTries = txTries = 0;
	rxWorked = txWorked = false;
	IOReturn ret = usb->clearHalt(epIn);
	if (ret == kIOReturnSuccess) {
		ret = usb->clearHalt(epOut);
	}
	if (ret != kIOReturnSuccess) {
		IOLog("RTLUSBEthernet: cannot clear the endpoint halts (0x%x)\n", ret);
		if (ret == kIOReturnNoDevice) {
			lostDevice();
			return kIOReturnNoDevice;
		}
		return kIOReturnIOError;
	}
	enabled = true;
	if (!startQueues()) {
		enabled = false;
		return gone ? kIOReturnNoDevice : kIOReturnIOError;
	}
	rxRestarted = txRestarted = uptime_ms();
	startAutoneg();
	setLinkStatus(kIONetworkLinkValid);
	txQueue->setCapacity(TX_BUFFERS * 4);
	txQueue->start();
	linkTimer->setTimeoutMS(100);
	return kIOReturnSuccess;
}

IOReturn
RTLUSBEthernet::disable(IONetworkInterface *interface)
{
	(void)interface;
	if (!enabled) {
		return kIOReturnSuccess;
	}
	linkTimer->cancelTimeout();
	recoverTimer->cancelTimeout();
	txQueue->stop();
	txQueue->flush();
	stopQueues();
	enabled = false;
	reset();
	linkUp = false;
	setLinkStatus(kIONetworkLinkValid);
	return kIOReturnSuccess;
}

IOReturn
RTLUSBEthernet::getHardwareAddress(IOEthernetAddress *addr)
{
	*addr = mac;
	return kIOReturnSuccess;
}

IOReturn
RTLUSBEthernet::setPromiscuousMode(bool active)
{
	promiscuous = active;
	if (enabled) {
		setRxFilter();
	}
	return kIOReturnSuccess;
}

IOReturn
RTLUSBEthernet::setMulticastMode(bool active)
{
	(void)active;
	return kIOReturnSuccess;
}

IOReturn
RTLUSBEthernet::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
	(void)addrs;
	(void)count;
	return kIOReturnSuccess;
}
