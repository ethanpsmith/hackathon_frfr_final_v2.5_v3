#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"
if command -v uv >/dev/null 2>&1; then
  uv_bin="$(command -v uv)"
elif [ -x "$HOME/.local/bin/uv" ]; then
  uv_bin="$HOME/.local/bin/uv"
else
  echo 'Install uv from https://docs.astral.sh/uv/getting-started/installation/ then rerun.' >&2
  exit 1
fi
command -v git >/dev/null
if ! command -v clang++ >/dev/null && ! command -v g++ >/dev/null; then
  echo 'A native C/C++ toolchain is required (Xcode command-line tools on macOS; build-essential on Ubuntu).' >&2
  exit 1
fi
"$uv_bin" python install 3.11.13
if [ ! -x .venv/bin/python ]; then
  "$uv_bin" venv --python 3.11.13 .venv
fi
"$uv_bin" pip sync --python .venv/bin/python simulation/requirements.lock
mkdir -p .tools
if [ ! -d .tools/ardupilot/.git ]; then
  git clone --depth 1 --branch Copter-4.6.3 https://github.com/ArduPilot/ardupilot.git .tools/ardupilot
fi
actual_revision="$(git -C .tools/ardupilot rev-parse HEAD)"
if [ "$actual_revision" != 92b0cd788ec29406f26c6f9c31d5ceedbd1cc538 ]; then
  echo 'Unexpected ArduPilot revision; use a clean .tools/ardupilot checkout.' >&2
  exit 1
fi
git -C .tools/ardupilot diff --exit-code
git -C .tools/ardupilot submodule update --init --recursive --depth 1
(
  cd .tools/ardupilot
  ../../.venv/bin/python waf configure --board sitl
  ../../.venv/bin/python waf copter -j 6
)
printf 'Ready. Run: bash simulation/test.sh\n'
