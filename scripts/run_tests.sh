#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if [ ! -d .venv ]; then
  if ! python3 -m venv .venv; then
    echo "error: could not create .venv (try: sudo apt install python3-venv python3-full)" >&2
    exit 1
  fi
fi
# shellcheck disable=SC1091
source .venv/bin/activate
pip install -U pip
pip install -e ".[test]"
pytest "$@"
