#!/usr/bin/env bash
# Build this checkout on the Linux box and boot manager until UI stays up.
# Usage: ./scripts/verify_host_boot.sh [--skip-build]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SKIP_BUILD=0
if [[ "${1:-}" == "--skip-build" ]]; then
  SKIP_BUILD=1
fi

export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
export NOBOARD=1
export FAKEUPLOAD=1
# Software GL so xvfb does not need a real GPU context.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  echo "==> uv sync --extra tools"
  uv sync --extra tools
  echo "==> scons --minimal"
  uv run --extra tools scons --minimal
fi

echo "==> boot manager until UI"
if [[ -z "${DISPLAY:-}" ]]; then
  exec xvfb-run -a -s "-screen 0 1920x1080x24" \
    uv run --extra tools python "$ROOT/scripts/verify_host_boot.py"
else
  exec uv run --extra tools python "$ROOT/scripts/verify_host_boot.py"
fi
