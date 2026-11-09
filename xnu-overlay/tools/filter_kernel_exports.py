#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
"""
usage: filter_kernel_exports.py NM KPI_IN KPI_OUT ALIAS_IN ALIAS_OUT INPUT...
  INPUT is an object, an archive or an ld64 -filelist file. A -filelist file
  ends in .filelist and names one path per line.

usage: filter_kernel_exports.py --exports EXPORTS_IN ALLSYMBOLS EXPORTS_OUT
  Filter a config/*.exports file for the kext symbol sets. ALLSYMBOLS is the
  linked kernel's global symbols, as nm -gj prints them.
"""
import subprocess
import sys


def inputs(args):
    for a in args:
        if a.endswith(".filelist"):
            with open(a) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        yield line
        else:
            yield a


def defined_symbols(nm, paths):
    syms = set()
    paths = list(paths)
    for i in range(0, len(paths), 500):
        out = subprocess.run([nm, "-gjU"] + paths[i:i + 500], check=True,
                             capture_output=True, text=True).stdout
        for line in out.splitlines():
            line = line.strip()
            if line and not line.endswith(":"):
                syms.add(line)
    return syms


def is_pattern(sym):
    return any(c in sym for c in "*?[")


def filter_exports_file(exports_in, allsymbols, exports_out):
    with open(allsymbols) as f:
        defined = {line.strip() for line in f if line.strip()}
    with open(exports_in) as f, open(exports_out, "w") as out:
        for line in f:
            text = line.strip()
            names = [n for n in text.split(":") if n]
            if (not text or text.startswith("#") or any(is_pattern(n) for n in names)
                    or any(n in defined for n in names)):
                out.write(line)
    return 0


def main():
    if len(sys.argv) == 5 and sys.argv[1] == "--exports":
        return filter_exports_file(*sys.argv[2:5])
    if len(sys.argv) < 7:
        sys.stderr.write(__doc__)
        return 1
    nm, kpi_in, kpi_out, alias_in, alias_out = sys.argv[1:6]
    defined = defined_symbols(nm, inputs(sys.argv[6:]))

    aliases = []
    with open(alias_in) as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 2:
                aliases.append((parts[0], parts[1]))
    kept_aliases = [(t, a) for t, a in aliases if t in defined]
    available = defined | {a for _, a in kept_aliases}

    dropped = []
    with open(kpi_in) as f, open(kpi_out, "w") as out:
        for line in f:
            sym = line.strip()
            if not sym or sym.startswith("#"):
                continue
            if sym in available or is_pattern(sym):
                out.write(sym + "\n")
            else:
                dropped.append(sym)
    with open(alias_out, "w") as out:
        for t, a in kept_aliases:
            out.write("%s %s\n" % (t, a))

    if dropped:
        sys.stderr.write("filter_kernel_exports: %d exported symbol(s) not defined, not exported:\n" % len(dropped))
        for sym in dropped:
            sys.stderr.write("    %s\n" % sym)
    return 0


if __name__ == "__main__":
    sys.exit(main())
