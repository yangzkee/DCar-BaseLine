#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT HUP INT TERM
for variant in DFCom_Example DFCom_PatchOnly; do
    for config in '0 1' '1 1' '1 2' '1 3'; do
        set -- $config
        enabled=$1
        route=$2
        cc -std=c99 -Wall -Wextra -Werror -Itests/host/stubs -I"$variant/HARDWARE" \
            -Dmain=demo_main -DDEMO_ENABLE_MOTION="$enabled" -DDEMO_ROUTE="$route" \
            -c "$variant/USER/main.c" -o "$build/main.o"
        cc -std=c99 -Wall -Wextra -Werror -Itests/host/stubs -I"$variant/HARDWARE" \
            -DTEST_ENABLED="$enabled" -DTEST_ROUTE="$route" \
            tests/host/test_demo.c "$build/main.o" -o "$build/demo"
        for scenario in happy no-state no-telemetry uncalibrated begin-failure submit-failure wait-failure; do
            "$build/demo" "$scenario"
        done
    done
done
