#!/usr/bin/env bash
# Compile check: builds every example against the library for the RP2350 with Arduino-Pico
set -euo pipefail
cd "$(dirname "$0")/.."
for example in examples/*/; do
    echo "== $example"
    pio ci "$example" --lib=. --board=rpipico2 \
        --project-option="platform=https://github.com/maxgerhardt/platform-raspberrypi.git#372af498f885b827b2ddc10284a8bd0b353bdbd1" \
        --project-option="board_build.core=earlephilhower" \
        --project-option="build_flags=-Wall -Wextra -Werror"
done
