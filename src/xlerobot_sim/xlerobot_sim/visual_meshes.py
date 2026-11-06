"""Select prebuilt visual-only meshes without changing physics geometry."""
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def apply_visual_meshes(path):
    preset = os.environ.get('XLEROBOT_VISUAL_MESHES', 'simplified')
    if preset == 'full':
        return
    if preset != 'simplified':
        raise ValueError('XLEROBOT_VISUAL_MESHES must be simplified or full')
    directory = Path(__file__).resolve().parents[2] / 'xlerobot_description/visual_meshes'
    if not directory.is_dir():
        from ament_index_python.packages import get_package_share_directory
        directory = Path(get_package_share_directory('xlerobot_description')) / 'visual_meshes'
    records = {r['name']: r for r in json.loads((directory/'manifest.json').read_text())['meshes']}
    tree = ET.parse(path)
    root = tree.getroot()
    assets = root.find('asset')
    for mesh in list(assets.findall('mesh')):
        name = mesh.get('name')
        key = 'cart_connector' if name.startswith('cart_connector_') else name
        if key not in records:
            continue
        # Clone only for explicitly massless visual geoms. Hidden originals may
        # contribute to inferred inertia and must keep their original mesh.
        geoms = [g for g in root.iter('geom') if g.get('mesh') == name
                 and g.get('group') == '2' and g.get('contype') == '0'
                 and g.get('conaffinity') == '0' and g.get('mass') == '0']
        if not geoms:
            continue
        record = records[key]
        source = (path.parent / mesh.get('file')).resolve()
        reduced = directory / record['file']
        for file, digest in [(source, record['source_sha256']), (reduced, record['sha256'])]:
            if hashlib.sha256(file.read_bytes()).hexdigest() != digest:
                raise ValueError(f'Visual mesh checksum mismatch: {file}; regenerate LOD assets')
        replacement = name + '_visual_lod'
        ET.SubElement(assets, 'mesh', **{**mesh.attrib, 'name': replacement, 'file': str(reduced.resolve())})
        for geom in geoms:
            geom.set('mesh', replacement)
        if not any(element.get('mesh') == name for element in root.iter()):
            assets.remove(mesh)
    tree.write(path)
