#!/bin/bash
# Regenerates the files already in src/dsdl/ from pinned DSDL and dronecan_dsdlc commits
set -euo pipefail

DSDL_REV=b4653c7abc3c47cb31b16efa24ea755232774756
DSDLC_REV=de47a8972b7d8c59e50c3acfab5754db29798caa

ROOT=$(cd "$(dirname "$0")/.." && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

git clone -q https://github.com/dronecan/DSDL && git -C DSDL checkout -q $DSDL_REV
git clone -q https://github.com/dronecan/dronecan_dsdlc && git -C dronecan_dsdlc checkout -q $DSDLC_REV
python3 -m venv venv
venv/bin/pip install -q -r dronecan_dsdlc/requirements.txt
venv/bin/python dronecan_dsdlc/dronecan_dsdlc.py -O out DSDL/uavcan DSDL/dronecan

for f in "$ROOT"/src/dsdl/*; do
    src=$(find out -name "$(basename "$f")")
    # Local headers use quoted includes so they resolve from this library's folder
    sed -E 's/#include <((canard\.h|uavcan\.|dronecan\.)[^>]*)>/#include "\1"/' "$src" > "$f"
done
echo "Regenerated $(ls "$ROOT"/src/dsdl | wc -l | tr -d ' ') files"
