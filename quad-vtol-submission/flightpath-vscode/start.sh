#!/usr/bin/env bash
set -euo pipefail
flightpath_dir="$(cd "$(dirname "$0")" && pwd)"
flight_repo="$flightpath_dir/../hackathon_frfr_final_v2.5_v3"
cd "$flight_repo"
if [ ! -x .venv/bin/python ]; then
  echo 'Run bash simulation/setup.sh from the flight-software repository first.' >&2
  exit 1
fi
export PYTHONPATH="$flight_repo/flight-software"
exec .venv/bin/python -m simulation.flight_ops_bridge
