#!/bin/bash

set -euo pipefail

DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$DEMO_DIR/../../../.." && pwd)"
VENV_PATH="${PYBERRYPLC_VENV:-$PROJECT_DIR/.venv}"
PYTHON="$VENV_PATH/bin/python"

if [ ! -x "$PYTHON" ]; then
    echo "Python not found in virtual environment: $PYTHON"
    exit 1
fi

cd "$DEMO_DIR"
"$PYTHON" create_trajectory.py
echo "Starting CNC PLC demo. Use sudo if GPIO/UART access requires it."
sudo "$PYTHON" cnc_demo.py
