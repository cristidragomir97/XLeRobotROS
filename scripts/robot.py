"""Bootstrap the commit-pinned multiROS runner."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
CORE = Path(os.environ.get('MULTIROS_ROOT', ROOT / 'tools/multiros')).resolve()
if not (CORE / 'multiros/cli.py').is_file():
    raise SystemExit('Initialize shared tools: git submodule update --init --recursive')
sys.path.insert(0, str(CORE))

from multiros.cli import main

if __name__ == '__main__':
    raise SystemExit(main(['--config', str(ROOT / 'multiros.json'), *sys.argv[1:]]))
