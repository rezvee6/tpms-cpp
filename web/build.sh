#!/usr/bin/env bash
# Builds the detector + simulator to WebAssembly (single-file JS with the wasm
# inlined, so index.html also works when opened straight from disk).
set -euo pipefail
cd "$(dirname "$0")/.."

emcc -std=c++17 -O3 \
  -Icore/include -Isim \
  core/src/detector.cpp sim/vehicle_sim.cpp sim/harness.cpp web/wasm_bindings.cpp \
  -lembind \
  -sMODULARIZE=1 -sEXPORT_NAME=createItpms \
  -sSINGLE_FILE=1 -sALLOW_MEMORY_GROWTH=1 -sENVIRONMENT=web \
  -o web/itpms.js

echo "built web/itpms.js ($(wc -c < web/itpms.js | tr -d ' ') bytes)"
