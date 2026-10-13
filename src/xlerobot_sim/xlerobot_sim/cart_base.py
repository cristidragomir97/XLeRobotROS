"""Geometry-derived cart fit and four passive swivel/rolling caster assemblies."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


def cart_configuration():
    path = Path(__file__).resolve().parents[1]/'config/cart_mount.yaml'
    if not path.is_file():
        path=Path(get_package_share_directory('xlerobot_sim'))/'config/cart_mount.yaml'
    return yaml.safe_load(path.read_text())


def parallel(center):
    return np.dot(center,center)*np.eye(3)-np.outer(center,center)


def inertia_values(matrix):
    return ' '.join(map(str,[matrix[0,0],matrix[1,1],matrix[2,2],
                              matrix[0,1],matrix[0,2],matrix[1,2]]))


def add_cart_contacts(robot):
    config=cart_configuration()
    chassis=robot.find(".//body[@name='chassis']")
    assets=robot.find('asset')
    meshdir=Path(get_package_share_directory('xlerobot_description'))/'cart_meshes'
    for number in (1,2):
        chassis.remove(chassis.find(f"geom[@name='chassis_full_visual_{number}']"))
    root=chassis.find('inertial')
    mass=float(root.get('mass'))
    center=np.fromstring(root.get('pos'),sep=' ')
    xx,yy,zz,xy,xz,yz=np.fromstring(root.get('fullinertia'),sep=' ')
    inertia=np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])+mass*parallel(center)
    moment=mass*center
    for index,caster in enumerate(config['casters']):
        pivot=np.array(caster['pivot']);offset=np.array(caster['wheel_offset'])
        fork=ET.SubElement(chassis,'body',name=f'cart_caster_{index}_fork',pos=' '.join(map(str,pivot)))
        # Small passive vertical compliance represents caster tyre/mount flex,
        # allowing all seven wheels to share support without lifting the drive.
        ET.SubElement(fork,'joint',name=f'cart_caster_{index}_compliance',type='slide',axis='0 0 1',
                      limited='true',range='-0.002 0.002',stiffness='15000',damping='30',springref='0')
        ET.SubElement(fork,'joint',name=f'cart_caster_{index}_swivel',type='hinge',axis='0 0 1',
                      limited='false',damping='0.00002',armature='0.000001')
        wheel=ET.SubElement(fork,'body',name=f'cart_caster_{index}_wheel',pos=' '.join(map(str,offset)))
        ET.SubElement(wheel,'joint',name=f'cart_caster_{index}_spin',type='hinge',axis='1 0 0',
                      limited='false',damping='0.00002',armature='0.000001')
        radius,width=caster['radius'],2*caster['half_width']
        for kind,body,body_mass,position,diagonal,color in [
            ('fork',fork,.1,pivot,.1/12*(np.sum(np.square(caster['fork_size']))-np.square(caster['fork_size'])),'0.6 0.6 0.6 1'),
            ('wheel',wheel,.15,pivot+offset,np.array([.15*radius**2/2,*([.15*(3*radius**2+width**2)/12]*2)]),'0.02 0.02 0.02 1')]:
            mesh=f'cart_caster_{index}_{kind}'
            ET.SubElement(assets,'mesh',name=mesh,file=str(meshdir/f'caster_{index}_{kind}.stl'))
            ET.SubElement(body,'geom',name=mesh+'_visual',type='mesh',mesh=mesh,
                          group='2',rgba=color,contype='0',conaffinity='0',mass='0')
            ET.SubElement(body,'inertial',pos='0 0 0',mass=str(body_mass),diaginertia=' '.join(map(str,diagonal)))
            mass-=body_mass;moment-=body_mass*position
            inertia-=np.diag(diagonal)+body_mass*parallel(position)
        ET.SubElement(wheel,'geom',name=f'cart_caster_{index}_contact',type='cylinder',
                      size=f'{radius} {width/2}',quat='0.7071067811865476 0 0.7071067811865476 0',
                      group='3',rgba='0 0 0 0',contype='2',conaffinity='1',
                      friction='1 0.001 0.00001',mass='0')
    center=moment/mass
    inertia-=mass*parallel(center)
    if np.linalg.eigvalsh(inertia).min()<=0:
        raise ValueError('Caster mass allocation leaves nonphysical cart inertia')
    root.set('mass',str(mass));root.set('pos',' '.join(map(str,center)))
    root.set('fullinertia',inertia_values(inertia))
