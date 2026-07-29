#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
route_header="$repo_dir/DFCom_Example/USER/nuedc_2026_routes.h"
route_source="$repo_dir/DFCom_Example/USER/nuedc_2026_routes.c"

if [ ! -f "$route_header" ] || [ ! -f "$route_source" ]; then
    echo "FAIL: 2026 H/D route module has not been implemented"
    exit 1
fi

build_dir=$(mktemp -d "${TMPDIR:-/tmp}/nuedc-2026-routes.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

cc -std=c99 -Wall -Wextra -Werror \
    -I"$repo_dir/tests/host/nuedc_stubs" \
    -I"$repo_dir/DFCom_Example/USER" \
    "$repo_dir/tests/host/test_nuedc_2026_routes.c" \
    "$route_source" \
    -lm \
    -o "$build_dir/test_nuedc_2026_routes"

"$build_dir/test_nuedc_2026_routes"

