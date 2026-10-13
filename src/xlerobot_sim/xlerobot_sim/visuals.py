"""Restore full robot visuals from the pinned upstream ManiSkill URDF and its meshes."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from .cart_base import cart_configuration


def values(text, fallback):
    return np.fromstring(text if text is not None else fallback, sep=' ')


def pose(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, 3] = values(origin.get('xyz'), '0 0 0')
        t[:3, :3] = Rotation.from_euler('xyz', values(origin.get('rpy'), '0 0 0')).as_matrix()
    return t


def body_pose(body):
    t = np.eye(4)
    t[:3, 3] = values(body.get('pos'), '0 0 0')
    q = values(body.get('quat'), '1 0 0 0')
    t[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return t


def set_pose(body, transform):
    body.set('pos', ' '.join(f'{x:.15g}' for x in transform[:3, 3]))
    q = Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]]
    body.set('quat', ' '.join(f'{x:.15g}' for x in q))


def add_full_visuals(robot, source_dir):
    source_dir = Path(source_dir)
    source = ET.parse(source_dir / 'xlerobot.urdf').getroot()
    links = {link.get('name'): link for link in source.findall('link')}
    child_joints = {joint.find('child').get('link'): joint for joint in source.findall('joint')}
    colors = {material.get('name'): material.find('color').get('rgba')
              for material in source.findall('material') if material.find('color') is not None}
    assets = robot.find('asset')
    chassis = robot.find(".//body[@name='chassis']")
    cart_config = cart_configuration()
    cart_offset = cart_config['cart_z_offset']
    # Keep upstream's collision/mass proxy, but do not draw it over the detailed cart.
    for geom in chassis.findall('geom'):
        geom.set('group', '3')
        geom.set('rgba', '0 0 0 0')

    def visuals(target, source_link):
        for number, visual in enumerate(links[source_link].findall('visual')):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            mesh_name = 'full_' + Path(mesh.get('filename')).stem
            if assets.find(f"mesh[@name='{mesh_name}']") is None:
                ET.SubElement(assets, 'mesh', name=mesh_name,
                              file=str((source_dir / mesh.get('filename')).resolve()),
                              scale=mesh.get('scale', '1 1 1'))
            material = visual.find('material')
            color = material.find('color') if material is not None else None
            rgba = color.get('rgba') if color is not None else colors.get(
                material.get('name') if material is not None else '', '0.3 0.3 0.3 1')
            if source_link == 'base_link' and Path(mesh.get('filename')).stem == 'raskogbody':
                rgba = '0.95 0.95 0.95 1'
            geom = ET.SubElement(target, 'geom', name=f"{target.get('name')}_full_visual_{number}",
                                 type='mesh', mesh=mesh_name, rgba=rgba,
                                 group='2', contype='0', conaffinity='0', mass='0')
            transform = pose(visual.find('origin'))
            if source_link == 'base_link':
                transform[2,3] += cart_offset
            set_pose(geom, transform)

    visuals(chassis, 'base_link')  # cart body and both caster mesh sets
    # Original MuJoCo jaw *_part meshes are convex collision decompositions,
    # not the detailed chassis/motor visuals provided by ManiSkill.
    for name, source_link in [('Fixed_Jaw', 'Fixed_Jaw'), ('Moving_Jaw', 'Moving_Jaw'),
                              ('Fixed_Jaw_2', 'Fixed_Jaw_2'), ('Moving_Jaw_2', 'Moving_Jaw_2')]:
        target = robot.find(f".//body[@name='{name}']")
        for geom in target.findall('geom'):
            geom.set('group', '3')
        visuals(target, source_link)
    # Use the full assembly mounting poses, beside the tower, for physics and visuals.
    for side, source_base, source_shoulder, mj_shoulder, jaw, source_camera in [
            ('L', 'Base', 'Rotation_Pitch', 'Rotation_Pitch', 'Fixed_Jaw', 'Right_Arm_Camera'),
            ('R', 'Base_2', 'Rotation_Pitch_2', 'Rotation_Pitch_R', 'Fixed_Jaw_2', 'Left_Arm_Camera')]:
        shoulder = robot.find(f".//body[@name='{mj_shoulder}']")
        mount = ET.SubElement(chassis, 'body', name=f'arm_mount_{side}')
        mount_pose = pose(child_joints[source_base].find('origin'))
        mount_pose[2,3] += cart_offset
        set_pose(mount, mount_pose)
        set_pose(shoulder, mount_pose @ pose(child_joints[source_shoulder].find('origin')))
        visuals(mount, source_base)
        wrist = ET.SubElement(robot.find(f".//body[@name='{jaw}']"), 'body',
                              name=f'wrist_camera_{side}')
        set_pose(wrist, pose(child_joints[source_camera].find('origin')))
        visuals(wrist, source_camera)

    # Import the complete head support and articulated housing as one coherent assembly.
    support = ET.SubElement(chassis, 'body', name='top_base_link')
    support_pose = pose(child_joints['top_base_link'].find('origin'))
    support_pose[2,3] += cart_offset
    set_pose(support, support_pose)
    visuals(support, 'top_base_link')
    pan = robot.find(".//body[@name='head_pan_link']")
    chassis.remove(pan)
    support.append(pan)
    set_pose(pan, pose(child_joints['head_pan_link'].find('origin')))
    visuals(pan, 'head_pan_link')
    tilt = pan.find("body[@name='head_tilt_link']")
    set_pose(tilt, pose(child_joints['head_tilt_link'].find('origin')))
    visuals(tilt, 'head_tilt_link')

    # Source connector, fitted vertically as recommended by the assembly guide.
    from ament_index_python.packages import get_package_share_directory
    meshdir = Path(get_package_share_directory('xlerobot_description'))/'cart_meshes'
    for index,mount in enumerate(cart_config['connectors']):
        name=f'cart_connector_{index}'
        ET.SubElement(assets,'mesh',name=name,file=str(meshdir/'cart_connector.stl'),
                      scale=' '.join(map(str,mount['scale'])))
        body=ET.SubElement(chassis,'body',name=name,pos=' '.join(map(str,mount['position'])))
        ET.SubElement(body,'geom',name=name+'_visual',type='mesh',mesh=name,group='2',
                      rgba='0.1 0.1 0.1 1',contype='0',conaffinity='0',mass='0')
