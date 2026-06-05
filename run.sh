#!/usr/bin/env bash
set -euo pipefail

MODULE_ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$MODULE_ROOT"

VENV_DIR="${MODULE_ROOT}/.venv"
PYTHON="${VENV_DIR}/bin/python"
REQUIREMENTS="${MODULE_ROOT}/requirements.txt"

if [ ! -f "$REQUIREMENTS" ]; then
  echo "ERROR: requirements.txt not found in ${MODULE_ROOT}" >&2
  ls -la "$MODULE_ROOT" >&2
  exit 1
fi

if [ ! -x "$PYTHON" ]; then
  echo "Virtualenv not found; running setup.sh..." >&2
  ./setup.sh
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
pip install -r "$REQUIREMENTS" -q
pip install -e "$MODULE_ROOT" -q

echo "Starting viam-labs:rssi-triangulation module..."
exec "$PYTHON" "${MODULE_ROOT}/src/main.py" "$@"
