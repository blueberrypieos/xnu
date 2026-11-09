#!/bin/sh
# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# interface.sh NM DIR: what kxld sees of DIR/System.kext, its Info.plists
# and its symbol sets' symbols.
set -e
(cd "$2" && find System.kext -type f | LC_ALL=C sort) | while read -r f; do
	echo "== $f"
	case "$f" in
	*/Info.plist) cat "$2/$f" ;;
	*) "$1" -m "$2/$f" 2>/dev/null || true ;;
	esac
done
