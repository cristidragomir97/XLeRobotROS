"""Compile a scene once, then exit so compiler/cache memory is reclaimed."""
import argparse
import hashlib
import json
import os
from pathlib import Path

import mujoco
from .model import prepare_scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('upstream', 'runtime', 'world', 'world-assets', 'configuration', 'description'):
        parser.add_argument('--'+name, required=True)
    args = parser.parse_args()
    runtime = Path(args.runtime)
    scene = prepare_scene(args.upstream, runtime, args.world, args.world_assets or None,
                          args.configuration, args.description or None)
    model = mujoco.MjModel.from_xml_path(str(scene))
    temporary = runtime / 'scene.mjb.tmp'
    mujoco.mj_saveModel(model, str(temporary))
    with temporary.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    metadata = dict(schema=1, mujoco_version=mujoco.__version__, model_sha256=digest,
                    nq=model.nq, nv=model.nv, nu=model.nu, model_buffer_bytes=model.nbuffer)
    os.replace(temporary, runtime/'scene.mjb')
    temp_metadata = runtime/'model.json.tmp'
    temp_metadata.write_text(json.dumps(metadata, indent=2)+'\n')
    os.replace(temp_metadata, runtime/'model.json')


if __name__ == '__main__':
    main()
