#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

VENV_DIR=".venv"
PYTHON="${VENV_DIR}/bin/python"

if [ ! -x "$PYTHON" ]; then
  echo "Virtualenv not found; running setup.sh..." >&2
  ./setup.sh
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
pip install -r requirements.txt -q
pip install -e . -q

echo "Starting viam-labs:rssi-triangulation module..."
exec "$PYTHON" src/main.py "$@"
