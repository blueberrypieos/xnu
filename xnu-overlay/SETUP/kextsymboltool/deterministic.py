#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
import hashlib
import os
from pathlib import Path
import struct
import subprocess
import sys


def canonicalize(data):
    magics = {b'\xcf\xfa\xed\xfe': ('<', 32), b'\xce\xfa\xed\xfe': ('<', 28),
              b'\xfe\xed\xfa\xcf': ('>', 32), b'\xfe\xed\xfa\xce': ('>', 28)}
    if data[:4] not in magics:
        raise ValueError('symbol set must be a thin Mach-O file')
    endian, header = magics[data[:4]]
    if len(data) < header:
        raise ValueError('truncated Mach-O header')
    if struct.unpack_from(endian + 'I', data, 12)[0] != 11:
        raise ValueError('expected a symbol-set kext bundle')
    count, size = struct.unpack_from(endian + 'II', data, 16)
    end, cursor, identifiers = header + size, header, []
    if end > len(data):
        raise ValueError('truncated Mach-O commands')
    for _ in range(count):
        if cursor + 8 > end:
            raise ValueError('truncated Mach-O command')
        command, length = struct.unpack_from(endian + 'II', data, cursor)
        if length < 8 or cursor + length > end:
            raise ValueError('invalid Mach-O command size')
        if command == 0x1d:
            raise ValueError('refusing to rewrite a code-signed executable')
        if command == 0x1b:
            if length != 24:
                raise ValueError('invalid UUID command')
            identifiers.append(cursor + 8)
        cursor += length
    if cursor != end or len(identifiers) != 1:
        raise ValueError('expected exactly one UUID command')
    result = bytearray(data)
    offset = identifiers[0]
    result[offset:offset + 16] = bytes(16)
    identity = bytearray(hashlib.sha256(result).digest()[:16])
    identity[6] = (identity[6] & 0x0f) | 0x80
    identity[8] = (identity[8] & 0x3f) | 0x80
    result[offset:offset + 16] = identity
    return bytes(result)


def main():
    if len(sys.argv) < 2:
        raise ValueError('usage: deterministic.py RAW_TOOL [arguments]')
    epoch = os.environ.get('SOURCE_DATE_EPOCH')
    if epoch is not None and (not epoch.isascii() or not epoch.isdecimal()):
        raise ValueError('SOURCE_DATE_EPOCH must be a nonnegative decimal integer')
    result = subprocess.run(sys.argv[1:])
    if result.returncode:
        return result.returncode
    if epoch is None:
        return 0
    outputs = [sys.argv[i + 1] for i in range(2, len(sys.argv) - 1) if sys.argv[i] == '-output']
    if not outputs:
        raise ValueError('successful symbol-set command has no output')
    path = Path(outputs[-1])
    if path.resolve() == Path(os.devnull):
        return 0
    data = canonicalize(path.read_bytes())
    with path.open('r+b') as stream:
        stream.write(data)
        stream.truncate()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError) as error:
        raise SystemExit('kextsymboltool: ' + str(error))
