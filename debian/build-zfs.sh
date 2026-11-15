#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 The Blueberry Pie OS Authors
#
# debian/build-zfs.sh
#
# Builds libzfs_kext.a into the kernel SDK's usr/local/lib/kernel. Needs
# PIE_SDKROOT with xnu's headers, PIE_KERNEL_SOURCES/openzfs and
# PIE_KERNEL_BUILD, as pietoolchain's kernel library scripts do. JOBS sets
# the number of compile jobs.
set -euo pipefail
TOOLCHAIN=/usr/lib/pietoolchain/toolchain
KERNEL="$TOOLCHAIN/kernel"
CC="$TOOLCHAIN/bin/clang"
LIBTOOL="$TOOLCHAIN/prefix/bin/aarch64-apple-darwin-libtool"
BUILD=libzfs_kext
source "$KERNEL/kernel-build-common.sh"
pie_kernel_setup openzfs
KF="$SDK/System/Library/Frameworks/Kernel.framework"
GEN="$BUILD/include"
mkdir -p "$GEN"
meta() { sed -n "s/^$1: *//p" "$SRC/META"; }
VERSION=$(meta Version)
RELEASE=$(meta Release)
cat > "$GEN/zfs_config.h" <<EOF
#define ZFS_META_NAME "zfs"
#define ZFS_META_VERSION "$VERSION"
#define ZFS_META_RELEASE "$RELEASE"
#define ZFS_META_ALIAS "zfs-$VERSION-$RELEASE"
#define ZFS_META_AUTHOR "OpenZFS"
#define ZFS_META_LICENSE "CDDL"
#define ZFS_DEBUG_STR ""
#define SYSCONFDIR "/etc"
#define RUNSTATEDIR "/var/run"
#define KERNEL_MODPREFIX "/System/Library/Extensions"
#define HAVE_EFFICIENT_UNALIGNED_ACCESS 1
#define HAVE_IMPLICIT_FALLTHROUGH 1
#define HAVE_PAGEOUT_V2 1
EOF
printf '#define\tZFS_META_GITREV "zfs-%s-%s"\n' "$VERSION" "$RELEASE" > "$GEN/zfs_gitrev.h"

SOURCES=$("$PYTHON" -I - "$SRC/module/os/macos/Makefile.am" <<'EOF'
import os, re, sys
text = open(sys.argv[1]).read().replace("\\\n", " ")
for name in ("spl_SRCS", "zfs_common_SRCS", "zfs_os_SRCS", "zcommon_SRCS", "icp_SRCS", "lua_SRCS", "zstd_SRCS"):
    for f in re.search(r"^%s\s*=(.*)$" % name, text, re.M).group(1).split():
        f = os.path.normpath(f.replace("%D%", "module/os/macos"))
        if not f.endswith("_amd64.S"):
            print(f)
for f in ("aes/aesv8-armx.S", "aes/ghashv8-armx.S", "sha2/sha256-armv8.S", "sha2/sha512-armv8.S"):
    print("module/icp/asm-aarch64/" + f)
EOF
)

RENAMES=()
for s in cmn_err assfail crgetuid ddi_create_minor_node ddi_remove_minor_node vmem_create vmem_destroy \
	sbuf_new sbuf_len sbuf_finish sbuf_vprintf sbuf_printf sbuf_data sbuf_delete; do
	RENAMES+=("-D$s=spl_$s")
done
COMMON=(-arch arm64 -mcpu=generic -isysroot "$SDK" -mmacosx-version-min=26.0
	-Os -g -mkernel -nostdinc -fno-builtin-printf
	-fno-register-global-dtors-with-atexit -mllvm -disable-atexit-based-global-dtor-lowering
	-D__KERNEL__ -D_KERNEL -DKERNEL -DKERNEL_PRIVATE -DDRIVER_PRIVATE -DPRIVATE -DZFS_BUILTIN
	-DNDEBUG -UDEBUG -UHAVE_LARGE_STACKS -DNAMEDSTREAMS=1 -D__DARWIN_64_BIT_INO_T=1 -DAPPLE -DNeXT
	"${RENAMES[@]}"
	-include "$GEN/zfs_config.h" -I"$GEN"
	-I"$SRC/include/os/macos/spl" -I"$SRC/include/os/macos/zfs" -I"$SRC/module/icp/include"
	-I"$SRC/include" -I"$KF/Headers" -I"$KF/PrivateHeaders" -I"$SRC/module/zstd/include"
	-include "$SRC/module/zstd/include/zstd_compat_wrapper.h"
	-idirafter "$SDK/usr/local/include/kernel" -idirafter "$SDK/usr/include"
	-idirafter "$("$CC" -print-resource-dir)/include"
	-Wno-sign-conversion -Wno-shorten-64-to-32 -Wno-conditional-uninitialized -Wno-shadow
	-Wno-implicit-int-conversion -Wno-error=int-conversion -Wno-incompatible-function-pointer-types)
ASM=(-arch arm64 -mcpu=generic+crypto -mkernel -nostdinc -D_KERNEL -DKERNEL
	-I"$SRC/include" -I"$SRC/include/os/macos/spl" -I"$KF/Headers")

compile() {
	local src=$1 obj=$2
	case "$src" in
	*.cpp) "$CC" "${REPRO_FLAGS[@]}" "${COMMON[@]}" -x c++ -std=gnu++17 -fapple-kext -fno-rtti -fno-exceptions \
		-c "$SRC/$src" -o "$obj" ;;
	*.S) "$CC" "${REPRO_FLAGS[@]}" "${ASM[@]}" -c "$SRC/$src" -o "$obj" ;;
	*) "$CC" "${REPRO_FLAGS[@]}" "${COMMON[@]}" -std=gnu11 -c "$SRC/$src" -o "$obj" ;;
	esac
}

objs=()
pids=()
for src in $SOURCES; do
	obj="$BUILD/$(printf '%s' "${src%.*}" | tr / _).o"
	objs+=("$obj")
	compile "$src" "$obj" &
	pids+=($!)
	if [ "${#pids[@]}" -ge "${JOBS:-1}" ]; then
		wait "${pids[0]}"
		pids=("${pids[@]:1}")
	fi
done
for pid in "${pids[@]}"; do
	wait "$pid"
done
install -d "$SDK/usr/local/lib/kernel"
pie_kernel_archive "$SDK/usr/local/lib/kernel/libzfs_kext.a" "${objs[@]}"
