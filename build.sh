#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

./setup.sh

ARCHIVE="${MODULE_ARCHIVE:-module.tar.gz}"
rm -f "$ARCHIVE"

tar -czf "$ARCHIVE" \
  --exclude='__pycache__' \
  --exclude='*.pyc' \
  --exclude='.venv' \
  --exclude='.git' \
  --exclude='tests' \
  --exclude='examples' \
  --exclude='web' \
  --exclude='scripts' \
  --exclude='test_scan_rssi.py' \
  meta.json \
  run.sh \
  setup.sh \
  build.sh \
  requirements.txt \
  pyproject.toml \
  README.md \
  LICENSE \
  src \
  rssi_triangulation

echo "Created ${ARCHIVE}"
