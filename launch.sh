#!/usr/bin/env bash
# Quick launcher for M9 Drone Flight Deck

set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$DIR"

python3 run_controller.py "$@"
