"""Bootstrap the commit-pinned multiROS runner."""
import os
import platform
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
CORE = Path(os.environ.get('MULTIROS_ROOT', ROOT / 'tools/multiros')).resolve()
if not (CORE / 'multiros/cli.py').is_file():
    raise SystemExit('Initialize shared tools: git submodule update --init --recursive')
sys.path.insert(0, str(CORE))

from multiros.cli import load_env, main

load_env(ROOT)

# WSL uses Docker Desktop's GPU device and Mesa D3D12 for the host viewer.
# Explicit environment selections still take precedence.
if platform.system() == 'Linux' and 'microsoft' in platform.release().lower():
    os.environ.setdefault('MULTIROS_BACKEND', 'wsl')
    os.environ.setdefault('GALLIUM_DRIVER', 'd3d12')
    paths = os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep)
    if '/usr/lib/wsl/lib' not in paths:
        os.environ['LD_LIBRARY_PATH'] = os.pathsep.join(['/usr/lib/wsl/lib', *filter(None, paths)])

if __name__ == '__main__':
    raise SystemExit(main(['--config', str(ROOT / 'pixi.toml'), *sys.argv[1:]]))
