#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
main_c="$repo_dir/DFCom_Example/USER/main.c"
project="$repo_dir/DFCom_Example/USER/Template.uvprojx"
readme="$repo_dir/README.md"

fail()
{
    echo "FAIL: $1"
    exit 1
}

rg -q '^#include "nuedc_2026_routes.h"' "$main_c" \
    || fail "main.c does not include the 2026 route API"
rg -q 'delay_ms\(NUEDC_2026_START_DELAY_MS\)' "$main_c" \
    || fail "main.c does not provide the configured start delay"
[ "$(rg -c 'Nuedc2026_RunSelectedRoute\(\)' "$main_c")" -eq 1 ] \
    || fail "main.c must execute the selected route exactly once"
! rg -q 'Cmd_Move_Linear\( 50|Cmd_Move_Rot\( 15|Cmd_Move_Rot\( -15' "$main_c" \
    || fail "old linear/rotation regression demo remains in main.c"

route_source_count=$(rg -c '<FileName>nuedc_2026_routes.c</FileName>' "$project" || true)
route_header_count=$(rg -c '<FileName>nuedc_2026_routes.h</FileName>' "$project" || true)
[ "${route_source_count:-0}" -eq 1 ] \
    || fail "Keil project must contain the NUEDC route source exactly once"
[ "${route_header_count:-0}" -eq 1 ] \
    || fail "Keil project must expose the NUEDC tuning header exactly once"
rg -q '<FilePath>\.\\nuedc_2026_routes.c</FilePath>' "$project" \
    || fail "Keil project route path is missing"

rg -q 'NUEDC_2026_ACTIVE_ROUTE' "$readme" \
    || fail "README does not explain H/D selection"
rg -q '不使用光电|不读取光电' "$readme" \
    || fail "README does not state the no-photoelectric contract"

python3 - "$project" <<'PY'
import sys
import xml.etree.ElementTree as ET

ET.parse(sys.argv[1])
print("Keil project XML parsed")
PY

echo "all NUEDC 2026 integration checks passed"
