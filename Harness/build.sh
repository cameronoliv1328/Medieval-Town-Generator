#!/usr/bin/env bash
# Build + run the standalone layout harness and convert the output to PNG.
# Usage: Harness/build.sh [seed] [outprefix]
set -euo pipefail
cd "$(dirname "$0")/.."

SEED="${1:-1337}"
OUT="${2:-/tmp/town_${SEED}}"

g++ -std=c++17 -O2 -Wall -Wno-unused-function \
    -IHarness/Shim -I. \
    Harness/HarnessMain.cxx -o /tmp/town_harness

/tmp/town_harness "$SEED" "$OUT"

python3 - "$OUT.ppm" "$OUT.png" <<'EOF'
import sys
from PIL import Image
Image.open(sys.argv[1]).save(sys.argv[2])
print("wrote", sys.argv[2])
EOF
