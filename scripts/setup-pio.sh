#!/usr/bin/env bash
set -euo pipefail

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but not installed. Install uv first: https://docs.astral.sh/uv/getting-started/installation/"
  return 1 2>/dev/null || exit 1
fi

uv tool install --force platformio
uv tool update-shell

uv_bin_dir="$(uv tool dir --bin)"
case ":$PATH:" in
  *":$uv_bin_dir:"*) ;;
  *) export PATH="$uv_bin_dir:$PATH" ;;
esac
hash -r 2>/dev/null || true

echo "PlatformIO installed and ready."
echo "Try: pio --version"
