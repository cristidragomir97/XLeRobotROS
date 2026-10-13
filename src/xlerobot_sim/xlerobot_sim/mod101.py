"""Translate the source Mod101 URDF subtrees to MJCF without changing their frames."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from .configuration import arm_joints


def origin(element):
    if element is None:
        return np.zeros(3),np.eye(3)
    p=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
    r=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return p,r


def pose_attributes(p,r):
    q=Rotation.from_matrix(r).as_quat()[[3,0,1,2]]
    return {'pos':' '.join(map(str,p)),'quat':' '.join(map(str,q))}


def resolve_mesh(filename):
    if filename.startswith('package://'):
        package,path=filename[10:].split('/',1)
        return Path(get_package_share_directory(package))/path
    if filename.startswith('file://'):
        return Path(filename[7:])
    raise ValueError(f'Unsupported Mod101 mesh URI: {filename}')


def replace_arms(robot, description):
    urdf=ET.parse(description).getroot()
    links={l.get('name'):l for l in urdf.findall('link')}
    materials={m.get('name'):m.find('color').get('rgba') for m in urdf.findall('material') if m.find('color') is not None}
    joints=urdf.findall('joint')
    chassis=robot.find(".//body[@name='chassis']")
    for name in ('Rotation_Pitch','Rotation_Pitch_R'):
        chassis.remove(chassis.find(f"body[@name='{name}']"))
    actuators=robot.find('actuator')
    old_names=set(sum(arm_joints('standard').values(),[]))
    for actuator in list(actuators):
        if actuator.get('joint') in old_names:
            actuators.remove(actuator)
    for name in ('arm_mount_L','arm_mount_R'):
        mount=chassis.find(f"body[@name='{name}']")
        for geom in list(mount.findall('geom')):
            mount.remove(geom)
    assets=robot.find('asset')
    meshes={}
    controlled=set(sum(arm_joints('mod101').values(),[]))
    equality=robot.find('equality')
    if equality is None:
        equality=ET.SubElement(robot,'equality')
    by_parent={}
    for joint in joints:
        by_parent.setdefault(joint.find('parent').get('link'),[]).append(joint)

    def geometry(body, visual, collision, number):
        p,r=origin(visual.find('origin'))
        attrs=pose_attributes(p,r)
        attrs.update(name=f"{body.get('name')}_{'collision' if collision else 'visual'}_{number}",
                     group='3' if collision else '2',contype='2' if collision else '0',
                     conaffinity='1' if collision else '0',mass='0')
        shape=list(visual.find('geometry'))[0]
        if shape.tag=='mesh':
            path=resolve_mesh(shape.get('filename'))
            scale=shape.get('scale','1 1 1')
            key=(str(path),scale)
            if key not in meshes:
                name=f'mod101_mesh_{len(meshes)}'
                ET.SubElement(assets,'mesh',name=name,file=str(path),scale=scale)
                meshes[key]=name
            attrs.update(type='mesh',mesh=meshes[key])
        elif shape.tag=='box':
            attrs.update(type='box',size=' '.join(map(str,np.fromstring(shape.get('size'),sep=' ')/2)))
        elif shape.tag=='cylinder':
            attrs.update(type='cylinder',size=f"{shape.get('radius')} {float(shape.get('length'))/2}")
        elif shape.tag=='sphere':
            attrs.update(type='sphere',size=shape.get('radius'))
        else:
            raise ValueError(f'Unsupported Mod101 shape: {shape.tag}')
        material=visual.find('material')
        rgba='0 0 0 0' if collision else '0.3 0.3 0.3 1'
        if material is not None and not collision:
            color=material.find('color')
            rgba=color.get('rgba') if color is not None else materials[material.get('name')]
        attrs['rgba']=rgba
        ET.SubElement(body,'geom',**attrs)

    def append_children(parent, parent_name):
        for joint in by_parent.get(parent_name,[]):
            name=joint.find('child').get('link')
            if not name.startswith('mod101_'):
                continue
            p,r=origin(joint.find('origin'))
            body=ET.SubElement(parent,'body',name=name,**pose_attributes(p,r))
            link=links[name]
            inertial=link.find('inertial')
            if inertial is not None:
                com,rotation=origin(inertial.find('origin'))
                raw=inertial.find('inertia')
                xx,yy,zz,xy,xz,yz=[float(raw.get(k)) for k in ('ixx','iyy','izz','ixy','ixz','iyz')]
                tensor=rotation@np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])@rotation.T
                ET.SubElement(body,'inertial',pos=' '.join(map(str,com)),mass=inertial.find('mass').get('value'),
                              fullinertia=' '.join(map(str,[tensor[0,0],tensor[1,1],tensor[2,2],tensor[0,1],tensor[0,2],tensor[1,2]])))
            if joint.get('type')!='fixed':
                joint_name=joint.get('name')
                axis=joint.find('axis').get('xyz')
                limits=joint.find('limit')
                # Engage simulated motor stops 2 mrad inside their source limits.
                # This keeps gravity deflection inside the admissible ROS range.
                stop = {'margin':'0.002', 'solreflimit':'0.004 1'} if joint_name in controlled else {}
                ET.SubElement(body,'joint',name=joint_name,type='slide' if joint.get('type')=='prismatic' else 'hinge',
                              axis=axis,range=f"{limits.get('lower')} {limits.get('upper')}",limited='true',damping='0.1',armature='0.001',**stop)
                mimic=joint.find('mimic')
                if mimic is not None:
                    ET.SubElement(equality,'joint',name=joint_name+'_mimic',joint1=joint_name,joint2=mimic.get('joint'),
                                  polycoef=f"{mimic.get('offset','0')} {mimic.get('multiplier','1')} 0 0 0",solref='0.004 1')
                if joint_name in controlled:
                    # Simulation gains and STS-family torque cap; not motor calibration.
                    torque=min(float(limits.get('effort')),2.94)
                    ET.SubElement(actuators,'position',name=joint_name,joint=joint_name,kp='75',kv='4',
                                  ctrlrange=f"{limits.get('lower')} {limits.get('upper')}",forcerange=f'-{torque} {torque}')
            for number,v in enumerate(link.findall('visual')):
                geometry(body,v,False,number)
            for number,v in enumerate(link.findall('collision')):
                geometry(body,v,True,number)
            append_children(body,name)
    for side in ('L','R'):
        append_children(chassis.find(f"body[@name='arm_mount_{side}']"),f'arm_mount_{side}')
