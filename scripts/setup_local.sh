#!/usr/bin/env bash
# One-time setup on the LOCAL machine (where the arm is plugged in). Linux or macOS.
# Creates a venv and installs the package with its local extras. Requires Python 3.10+.
#
#   ./scripts/setup_local.sh
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-python3}"
echo "Using $("$PY" --version)"
"$PY" -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[local]"

echo
echo "Done. Find your arm's serial port, then probe it:"
echo "  Linux:  ls /dev/ttyACM* /dev/ttyUSB*"
echo "  macOS:  ls /dev/cu.usbmodem*"
echo "  .venv/bin/soarm-local --port <PORT> probe"
