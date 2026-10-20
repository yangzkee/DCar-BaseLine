#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT HUP INT TERM
for variant in DFCom_Example DFCom_PatchOnly; do
    includes="-Itests/host/driver_stubs -I$variant/SYSTEM/usart -I$variant/HARDWARE -Itests/host/stubs"
    # MMIO addresses are 32 bit on the MCU; mocks don't dereference their host casts.
    cc -std=c99 -Wall -Wextra -Werror -Wno-pointer-to-int-cast $includes         -Dfputc=driver_fputc -c "$variant/SYSTEM/usart/usart.c" -o "$build/usart.o"
    cc -std=c99 -Wall -Wextra -Werror $includes -DDFCom_RxParse=Actual_DFCom_RxParse         -c "$variant/HARDWARE/DFCom_Rx.c" -o "$build/rx.o"
    cc -std=c99 -Wall -Wextra -Werror $includes         -c "$variant/HARDWARE/DFCom_Tx.c" -o "$build/tx.o"
    cc -std=c99 -Wall -Wextra -Werror $includes tests/host/test_usart_dma.c         "$build/usart.o" "$build/rx.o" "$build/tx.o" -o "$build/usart"
    "$build/usart"
done
