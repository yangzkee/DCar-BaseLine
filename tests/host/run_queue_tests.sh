#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
for variant in DFCom_Example DFCom_PatchOnly; do
 cc -std=c99 -Wall -Wextra -Werror -Itests/host/stubs -I"$variant/HARDWARE" tests/host/test_queue.c "$variant/HARDWARE/DFCom_Tx.c" "$variant/HARDWARE/DFCom_Rx.c" -lm -o "$build/test"
 "$build/test"
done
