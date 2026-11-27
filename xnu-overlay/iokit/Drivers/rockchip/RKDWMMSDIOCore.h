/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#ifndef _IOKIT_ROCKCHIP_RKDWMMSDIOCORE_H
#define _IOKIT_ROCKCHIP_RKDWMMSDIOCORE_H

#include <stdint.h>

struct RKDWSdioIO {
	void *cookie;
	uint32_t (*read)(void *, uint32_t);
	void (*write)(void *, uint32_t, uint32_t);
	uint64_t (*time)(void *);
	void (*delay)(void *, uint32_t);
};

class RKDWSdioCore
{
public:
	enum Result { Success, Timeout, Error, Invalid, Busy };
	enum Response { NoResponse, R1, R4, R5 };
	enum {
		Control = 0x00, Power = 0x04, Divider = 0x08, Source = 0x0c, ClockEnable = 0x10,
		TimeoutReg = 0x14, Width = 0x18, BlockSize = 0x1c, ByteCount = 0x20, Mask = 0x24,
		Argument = 0x28, Command = 0x2c, Response0 = 0x30, MaskedStatus = 0x40,
		InterruptStatus = 0x44, Status = 0x48, FifoThreshold = 0x4c, BusMode = 0x80,
		Fifo = 0x200,
		Reset = 7, FifoReset = 2, GlobalInterrupt = 1U << 4,
		Start = 1U << 31, Hold = 1U << 29, UpdateClock = 1U << 21, Initial = 1U << 15,
		WaitData = 1U << 13, Write = 1U << 10, Data = 1U << 9,
		ResponseCRC = 1U << 8, ResponseExpected = 1U << 6,
		ResponseError = 1U << 1, CommandDone = 1U << 2, DataOver = 1U << 3,
		TransmitRequest = 1U << 4, ReceiveRequest = 1U << 5, ResponseCRCError = 1U << 6,
		DataCRCError = 1U << 7, ResponseTimeout = 1U << 8, DataTimeout = 1U << 9,
		HostTimeout = 1U << 10, FifoError = 1U << 11, LockedError = 1U << 12,
		StartError = 1U << 13, EndError = 1U << 15, CardInterrupt = 1U << 24,
		DataBusy = 1U << 9, CommandEvents = 0xffff,
		CommandErrors = ResponseError | ResponseCRCError | ResponseTimeout | LockedError,
		DataErrors = DataCRCError | DataTimeout | HostTimeout | FifoError | StartError | EndError,
		R5Errors = 0xcb00,
	};

	void bind(const RKDWSdioIO &access, uint32_t depth, uint32_t sourceHz)
	{
		io = access;
		fifoDepth = depth;
		busHz = sourceHz;
		clockHz = 0;
		requestedHz = 0;
	}

	static bool matches(bool board, bool nonremovable, bool noSD, bool noMMC, uint64_t address)
	{
		return board && nonremovable && noSD && noMMC && address == 0xfe000000ULL;
	}

	uint32_t clock(void) const { return clockHz; }

	bool initialize(void)
	{
		wr(Mask, 0);
		wr(Control, Reset);
		if (!clear(Control, Reset, 100000)) {
			return false;
		}
		wr(Control, GlobalInterrupt);
		wr(Power, 1);
		wr(BusMode, 0);
		wr(TimeoutReg, ~0U);
		wr(FifoThreshold, ((fifoDepth / 2 - 1) << 16) | (fifoDepth / 2));
		wr(Width, 0);
		wr(InterruptStatus, CommandEvents | CardInterrupt);
		clockHz = 0;
		return true;
	}

	bool setClock(uint32_t hz)
	{
		if (hz == 0 || busHz == 0) {
			return false;
		}
		uint64_t denominator = (uint64_t)hz * 2;
		uint64_t divisor = busHz <= hz ? 0 : ((uint64_t)busHz + denominator - 1) / denominator;
		if (divisor > 255) {
			return false;
		}
		clockHz = 0;
		wr(ClockEnable, 0);
		if (!update()) {
			return false;
		}
		wr(Source, 0);
		wr(Divider, (uint32_t)divisor);
		if (!update()) {
			return false;
		}
		wr(ClockEnable, 1);
		if (!update() || rd(Divider) != divisor || rd(ClockEnable) != 1) {
			wr(ClockEnable, 0);
			(void)update();
			clockHz = 0;
			return false;
		}
		requestedHz = hz;
		clockHz = divisor == 0 ? busHz : busHz / (2 * (uint32_t)divisor);
		io.delay(io.cookie, 1000);
		return true;
	}

	bool clockOff(void)
	{
		clockHz = 0;
		wr(ClockEnable, 0);
		return update();
	}

	void width4(void) { wr(Width, 1); }

	Result command(uint32_t index, uint32_t argument, Response response, uint32_t values[4],
	    bool data = false, bool write = false)
	{
		if (index != 0 && index != 3 && index != 5 && index != 7 && index != 52 && index != 53) {
			return Invalid;
		}
		if (!clear(Command, Start, 100000)) {
			return Timeout;
		}
		if (data && !clear(Status, DataBusy, 500000)) {
			return Busy;
		}
		uint32_t flags = response == NoResponse ? 0U : (uint32_t)ResponseExpected;
		if (response == R1 || response == R5) {
			flags |= ResponseCRC;
		}
		if (data) {
			flags |= Data | WaitData | (write ? (uint32_t)Write : 0U);
		}
		if (index == 0) {
			flags |= Initial;
		}
		wr(InterruptStatus, CommandEvents);
		wr(Argument, argument);
		wr(Command, Start | Hold | flags | index);
		if (!clear(Command, Start, 100000)) {
			return Timeout;
		}
		uint32_t errors = CommandErrors;
		if (response == R4 || response == NoResponse) {
			errors &= ~ResponseCRCError;
		}
		uint32_t status = wait(CommandDone, errors, 500000);
		if ((status & ResponseTimeout) || !(status & (CommandDone | errors))) {
			return Timeout;
		}
		if ((status & errors) || !(status & CommandDone)) {
			return Error;
		}
		if (values != nullptr) {
			values[0] = response == NoResponse ? 0 : rd(Response0);
			for (unsigned i = 1; i < 4; i++) {
				values[i] = 0;
			}
		}
		wr(InterruptStatus, CommandDone | (response == R4 ? (uint32_t)ResponseCRCError : 0U));
		return Success;
	}

	Result transfer(uint32_t argument, void *buffer, uint32_t block, uint32_t count, bool write,
	    uint32_t *r5)
	{
		if (buffer == nullptr || r5 == nullptr || block == 0 || block > 512 || count == 0 || count > 511 ||
		    (((argument >> 31) != 0) != write) || (!(argument & (1U << 27)) && count != 1)) {
			return Invalid;
		}
		*r5 = 0;
		wr(Control, rd(Control) | FifoReset);
		if (!clear(Control, FifoReset, 100000)) {
			return Timeout;
		}
		uint32_t length = block * count;
		wr(BlockSize, block);
		wr(ByteCount, length);
		uint32_t response[4] = { 0, 0, 0, 0 };
		Result result = command(53, argument, R5, response, true, write);
		*r5 = response[0];
		if (result != Success || (*r5 & R5Errors)) {
			return result;
		}
		uint8_t *bytes = (uint8_t *)buffer;
		uint32_t done = 0;
		uint64_t started = now(), progress = started;
		while (done < length) {
			uint32_t events = rd(InterruptStatus);
			if (events & (DataErrors | CommandErrors)) {
				return events & (DataTimeout | HostTimeout | ResponseTimeout) ? Timeout : Error;
			}
			uint32_t used = (rd(Status) >> 17) & 0x1fff;
			if (used > fifoDepth) {
				return Error;
			}
			uint32_t available = write ? fifoDepth - used : used;
			while (available != 0 && done < length) {
				uint32_t n = length - done < 4 ? length - done : 4;
				uint32_t word = 0;
				if (write) {
					for (uint32_t i = 0; i < n; i++) {
						word |= (uint32_t)bytes[done + i] << (8 * i);
					}
					wr(Fifo, word);
				} else {
					word = rd(Fifo);
					for (uint32_t i = 0; i < n; i++) {
						bytes[done + i] = (uint8_t)(word >> (8 * i));
					}
				}
				done += n;
				available--;
				progress = now();
			}
			wr(InterruptStatus, write ? TransmitRequest : ReceiveRequest);
			if (done == length) {
				break;
			}
			if (now() - progress >= 2000000 || now() - started >= 10000000) {
				return Timeout;
			}
			io.delay(io.cookie, 5);
		}
		uint32_t status = wait(DataOver, DataErrors | CommandErrors, write ? 2000000 : 1000000);
		if (status & (DataTimeout | HostTimeout | ResponseTimeout)) {
			return Timeout;
		}
		if ((status & (DataErrors | CommandErrors)) || !(status & DataOver)) {
			return status & (DataErrors | CommandErrors) ? Error : Timeout;
		}
		wr(InterruptStatus, DataOver | TransmitRequest | ReceiveRequest);
		return Success;
	}

	void interruptEnable(bool on)
	{
		if (on) {
			wr(InterruptStatus, CardInterrupt);
		}
		wr(Mask, on ? (uint32_t)CardInterrupt : 0U);
	}

	bool interruptFilter(void)
	{
		if (!(rd(MaskedStatus) & CardInterrupt)) {
			return false;
		}
		wr(Mask, 0);
		return true;
	}

	bool recover(void)
	{
		uint32_t mask = rd(Mask), width = rd(Width), hz = requestedHz;
		wr(Mask, 0);
		wr(Control, Reset);
		if (!clear(Control, Reset, 100000)) {
			clockHz = 0;
			return false;
		}
		wr(Control, GlobalInterrupt);
		wr(BusMode, 0);
		wr(TimeoutReg, ~0U);
		wr(FifoThreshold, ((fifoDepth / 2 - 1) << 16) | (fifoDepth / 2));
		wr(Width, width);
		wr(InterruptStatus, CommandEvents);
		if (hz == 0 || !setClock(hz)) {
			return false;
		}
		wr(Mask, mask & CardInterrupt);
		return true;
	}

private:
	RKDWSdioIO io;
	uint32_t fifoDepth;
	uint32_t busHz;
	uint32_t clockHz;
	uint32_t requestedHz;

	uint32_t rd(uint32_t offset) { return io.read(io.cookie, offset); }
	void wr(uint32_t offset, uint32_t value) { io.write(io.cookie, offset, value); }
	uint64_t now(void) { return io.time(io.cookie); }

	bool clear(uint32_t offset, uint32_t bits, uint32_t usec)
	{
		uint64_t start = now();
		while (rd(offset) & bits) {
			if (now() - start >= usec) {
				return !(rd(offset) & bits);
			}
			io.delay(io.cookie, 5);
		}
		return true;
	}

	uint32_t wait(uint32_t wanted, uint32_t errors, uint32_t usec)
	{
		uint64_t start = now();
		for (;;) {
			uint32_t value = rd(InterruptStatus);
			if ((value & (wanted | errors)) || now() - start >= usec) {
				return value;
			}
			io.delay(io.cookie, 5);
		}
	}

	bool update(void)
	{
		if (!clear(Command, Start, 100000)) {
			return false;
		}
		wr(InterruptStatus, LockedError);
		wr(Argument, 0);
		wr(Command, Start | Hold | UpdateClock | WaitData);
		return clear(Command, Start, 100000) && !(rd(InterruptStatus) & LockedError);
	}
};

#endif
