#!/usr/bin/env python3
"""Derive cart alignment/caster meshes from pinned upstream geometry.

Run through manage.sh exec. Output meshes preserve source triangles, partitioned
into four caster assemblies. Connector XY follows the original drive motor
mounts; its Z scale follows the upstream adjustable-height assembly guidance.
These fits are simulation assembly assumptions, not surveyed bolt locations.
"""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'upstream/simulation/Maniskill/assets/xlerobot'
OUT = ROOT / 'src/xlerobot_description/cart_meshes'
OUT.mkdir(exist_ok=True)
DTYPE = np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])


def read(path):
    with path.open('rb') as stream:
        stream.seek(80)
        count = struct.unpack('<I',stream.read(4))[0]
        return np.fromfile(stream,dtype=DTYPE,count=count)['vertices'].astype(float)


def write(path, triangles):
    data = np.zeros(len(triangles),dtype=DTYPE)
    normals = np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0])
    lengths = np.linalg.norm(normals,axis=1)
    normals /= np.maximum(lengths[:,None],1e-20)
    data['normal'],data['vertices'] = normals,triangles
    with path.open('wb') as stream:
        stream.write(b'Derived from pinned XLeRobot geometry'.ljust(80,b' '))
        stream.write(struct.pack('<I',len(data)))
        data.tofile(stream)


urdf = ET.parse(SOURCE/'xlerobot.urdf').getroot()
cart = urdf.find("link[@name='base_link']")
triangles = {}
for visual in cart.findall('visual'):
    mesh = visual.find('geometry/mesh')
    origin = visual.find('origin')
    vertices = read(SOURCE/mesh.get('filename')) * np.fromstring(mesh.get('scale'),sep=' ')
    rotation = Rotation.from_euler('xyz',np.fromstring(origin.get('rpy'),sep=' ')).as_matrix()
    triangles[Path(mesh.get('filename')).stem] = vertices@rotation.T + np.fromstring(origin.get('xyz'),sep=' ')
minimum = triangles['raskogwheel2'][:,:,2].min()
model = mujoco.MjModel.from_xml_path(str(ROOT/'upstream/simulation/mujoco/scene.xml'))
data = mujoco.MjData(model);mujoco.mj_forward(model,data)
base = model.body('chassis').id
rotation = data.xmat[base].reshape(3,3)
wheel_z = (rotation.T@(data.xpos[model.body('4-Omni-Directional-Wheel_Single_Body-v1-2').id]-data.xpos[base]))[2]
radius = yaml.safe_load((ROOT/'src/xlerobot_control/config/base.yaml').read_text())['wheel_radius']
shift = float(wheel_z-radius-minimum)
casters = []
for xpositive,ypositive in [(False,False),(False,True),(True,False),(True,True)]:
    pieces = {}
    for kind,name in [('fork','raskogwheel1'),('wheel','raskogwheel2')]:
        tri = triangles[name]
        c = tri.mean(axis=1)
        selected = tri[((c[:,0]>0)==xpositive)&((c[:,1]>0)==ypositive)]
        bounds = selected.reshape(-1,3)
        center = (bounds.min(0)+bounds.max(0))/2
        center[2] += shift
        pieces[kind] = (selected,center,bounds.ptp(0))
    pivot = pieces['fork'][1]
    wheel_center = pieces['wheel'][1]
    index = len(casters)
    for kind,(tri,center,extent) in pieces.items():
        tri = tri.copy();tri[:,:,2] += shift
        write(OUT/f'caster_{index}_{kind}.stl',tri-center)
    extent = pieces['wheel'][2]
    casters.append({'pivot':pivot.tolist(), 'wheel_offset':(wheel_center-pivot).tolist(),
                    'radius':float(extent[2]/2),'half_width':float(extent[0]/2),
                    'fork_size':pieces['fork'][2].tolist()})
connector = read(ROOT/'upstream/hardware/XLeRobot 0-3-0 new wheel connector.stl')*.001
v=connector.reshape(-1,3);lo,hi=v.min(0),v.max(0)
connector -= np.array([(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]])
write(OUT/'cart_connector.stl',connector)
# Use MuJoCo's exact mesh ray intersection for the cart underside at each mount.
with __import__('tempfile').TemporaryDirectory() as t:
    xml=ET.Element('mujoco');asset=ET.SubElement(xml,'asset')
    ET.SubElement(asset,'mesh',name='cart',file=str(SOURCE/'meshes/raskogbody.stl'),scale='0.0009 0.001 0.0009')
    world=ET.SubElement(xml,'worldbody')
    ET.SubElement(world,'geom',type='mesh',mesh='cart',pos=f'-0.01668 0.502 {0.895+shift}',euler='1.5708 0 0')
    ET.SubElement(xml,'compiler',angle='radian')
    path=Path(t)/'cart.xml';ET.ElementTree(xml).write(path)
    cm=mujoco.MjModel.from_xml_path(str(path));cd=mujoco.MjData(cm);mujoco.mj_forward(cm,cd)
    mounts=[]
    top=float((rotation.T@(data.xpos[model.body('base_plate_layer2-v1').id]-data.xpos[base]))[2])
    for name in ['drive_motor_mount-v4-3','drive_motor_mount-v4-2','drive_motor_mount-v4-1']:
        position=rotation.T@(data.xpos[model.body(name).id]-data.xpos[base]);position[2]=top
        height=mujoco.mj_rayMesh(cm,cd,0,position,np.array([0.,0.,1.]))
        assert height>0
        mounts.append({'position':position.tolist(),'scale':[1.,1.,float(height/(hi[2]-lo[2]))],'height':float(height)})
config={'cart_z_offset':shift,'caster_ground_before_offset':float(minimum),
        'casters':casters,'connectors':mounts}
(ROOT/'src/xlerobot_sim/config/cart_mount.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
print(yaml.safe_dump(config,sort_keys=False))
