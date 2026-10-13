#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
# Prepared assets are reused by development containers and baked into new images.
if [[ -f assets/robocasa/manifest.json \
   && -f assets/robocasa/fridges/Refrigerator040/prepared.xml \
   && -f assets/robocasa/stoves/Stove066/prepared.xml \
   && -f assets/robocasa/glass_cup/GlassCup008/prepared.xml \
   && -f assets/robocasa/honey_bottle/HoneyBottle002/prepared.xml ]]; then
    printf 'RoboCasa assets already prepared.\n'
    exit 0
fi
command -v uv >/dev/null || { printf 'Install uv to prepare world assets: https://docs.astral.sh/uv/\n' >&2; exit 1; }
exec uv run --no-project --python 3.12 --with mujoco==3.6.0 --with numpy==1.26.4 \
    python tools/prepare_world_assets.py
