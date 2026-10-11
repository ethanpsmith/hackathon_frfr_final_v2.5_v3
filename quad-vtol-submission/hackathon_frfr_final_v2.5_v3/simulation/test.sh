#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"
export PYTHONPATH="$repo_root/flight-software"
exec .venv/bin/python -m pytest --sitl tests -q --junitxml=simulation/runs/junit.xml "$@"
