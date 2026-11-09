#!/bin/sh
# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# sdk-delta.sh SDK KERNEL-SDK ROOT: copy into ROOT, at the same paths, the
# kernel SDK's files that SDK lacks or has otherwise. usr/local/lib/kernel,
# usr/local/libexec and usr/local/include/dyld are left out.
set -e
cd "$2"
find . \( -type f -o -type l \) ! -path './usr/local/lib/kernel/*' ! -path './usr/local/libexec/*' ! -path './usr/local/include/dyld/*' | LC_ALL=C sort | while read -r f; do
	if [ -L "$f" ]; then
		[ -L "$1/$f" ] && [ "$(readlink "$f")" = "$(readlink "$1/$f")" ] && continue
	else
		[ -f "$1/$f" ] && [ ! -L "$1/$f" ] && cmp -s "$f" "$1/$f" && continue
	fi
	mkdir -p "$3/$(dirname "$f")"
	cp -P "$f" "$3/$f"
done
