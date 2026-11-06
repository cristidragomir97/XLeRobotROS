"""Bake visual-only LOD assets; run in a development environment, not at startup.

Requires trimesh==5.2.0 and fast-simplification==0.1.13. Source assets are never edited.
"""
import hashlib
import json
import os
import tempfile
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'src/xlerobot_description/visual_meshes'


def atomic_write(path, content):
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(content)
    os.replace(temporary, path)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    sources = [('cart_connector', ROOT/'src/xlerobot_description/cart_meshes/cart_connector.stl', 6000)]
    for ending in ('', '-1', '-2'):
        name = '4-Omni-Directional-Wheel_Single_Body-v1'+ending
        sources.append((name, ROOT/'upstream/simulation/mujoco/assets'/f'simplify_{name}.stl', 16000))
    records=[]
    for name, source, target in sources:
        mesh=trimesh.load_mesh(source, process=True)
        # Normalize units: connector STL uses metres, upstream wheels use mm.
        # Keep the quadric reduction numerically consistent across source units.
        center=mesh.bounds.mean(axis=0)
        extent=float(mesh.extents.max())
        vertices, faces=fast_simplification.simplify((mesh.vertices-center)/extent, mesh.faces,
                                                   target_count=target, agg=7.)
        vertices=vertices*extent+center
        result=trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        if not np.isfinite(vertices).all() or len(faces)>=len(mesh.faces):
            raise RuntimeError(f'Invalid reduction: {source}')
        filename=name+'.stl';atomic_write(OUTPUT/filename, result.export(file_type='stl'))
        record=dict(name=name, source=str(source.relative_to(ROOT)),
                    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                    file=filename, sha256=hashlib.sha256((OUTPUT/filename).read_bytes()).hexdigest(),
                    original_triangles=len(mesh.faces), target_triangles=target, triangles=len(faces),
                    original_bounds=mesh.bounds.tolist(), bounds=result.bounds.tolist())
        records.append(record)
        print(name,len(mesh.faces),'->',len(faces),flush=True)
    atomic_write(OUTPUT/'manifest.json', (json.dumps(dict(schema=1,
        generator=dict(trimesh=trimesh.__version__,fast_simplification=fast_simplification.__version__,aggressiveness=7.),
        meshes=records),indent=2)+'\n').encode())


if __name__=='__main__':main()
