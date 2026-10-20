#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT HUP INT TERM
for variant in DFCom_Example DFCom_PatchOnly; do
    cc -std=c99 -Wall -Wextra -Werror -Itests/host/stubs -I"$variant/HARDWARE" \
        -Dmain=demo_main -c "$variant/USER/main.c" -o "$build/main.o"
    cc -std=c99 -Wall -Wextra -Werror -Itests/host/stubs -I"$variant/HARDWARE" \
        tests/host/test_demo.c "$build/main.o" -o "$build/demo"
    cc -std=c99 -Wall -Wextra -Werror -Itests/host/telemetry_stubs \
        -Itests/host/stubs -I"$variant/HARDWARE" -Dprintf=test_printf \
        -c "$variant/HARDWARE/DFCom_Print.c" -o "$build/print.o"
    cc -std=c99 -Wall -Wextra -Werror -Itests/host/stubs -I"$variant/HARDWARE" \
        tests/host/test_telemetry_print.c "$build/print.o" -o "$build/telemetry"
    "$build/telemetry"
    for scenario in happy with-odom odom-only no-state no-telemetry uncalibrated begin-failure submit-failure wait-failure; do
        "$build/demo" "$scenario"
    done
done
