#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
sanitizer_flags=
if [ "${DFCOM_SANITIZE:-0}" = "1" ]; then
 sanitizer_flags="-fsanitize=address,undefined -fno-omit-frame-pointer"
fi
for variant in DFCom_Example DFCom_PatchOnly; do
 cc $sanitizer_flags -std=c99 -Wall -Wextra -Werror -Itests/host/stubs -I"$variant/HARDWARE" tests/host/test_queue.c "$variant/HARDWARE/DFCom_Tx.c" "$variant/HARDWARE/DFCom_Rx.c" -lm -o "$build/test"
 "$build/test"
done
