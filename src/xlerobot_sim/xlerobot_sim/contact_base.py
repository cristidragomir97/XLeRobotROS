"""Adapt Aditya Kamath's passive-roller topology to Victor Wang's wheel frames.

Upstream sources stay pinned; only the runtime MJCF is changed. All added
geometry/mass values are simulation assumptions, not hardware measurements.
"""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from .cart_base import add_cart_contacts

WHEEL_BODIES = ['4-Omni-Directional-Wheel_Single_Body-v1-2',
                '4-Omni-Directional-Wheel_Single_Body-v1-1',
                '4-Omni-Directional-Wheel_Single_Body-v1']


def add_contact_base(robot, config):
    share = Path(get_package_share_directory('lekiwi_mujoco'))
    source = ET.parse(share / 'mjcf/base_subtree.xml').getroot()
    templates = source.find("body[@name='left_wheel_link']").findall('body')
    defaults = ET.parse(share / 'mjcf/base_shared.xml').getroot()
    parent = robot.find('default')
    if parent is None:
        parent = ET.SubElement(robot, 'default')
    for name in ('passive_roller', 'roller_geom'):
        element = deepcopy(defaults.find(f".//default[@class='{name}']"))
        element.set('class', 'xlerobot_' + name)
        if name == 'roller_geom':
            geom = element.find('geom')
            geom.attrib.pop('material', None)
            geom.set('rgba', '0 0 0 0')
            geom.set('size', ' '.join(map(str, np.array([.009, .018, .009]) * config['wheel_radius']/.051)))
        parent.append(element)
    chassis = robot.find(".//body[@name='chassis']")
    for joint in list(chassis.findall('joint')):
        chassis.remove(joint)
    ET.SubElement(chassis, 'freejoint', name='floating_base')
    # Robot-world contacts only; adjacent imported convex visual meshes must not
    # collide with each other or resist the rollers' passive lateral motion.
    for geom in chassis.iter('geom'):
        if geom.get('contype') != '0':
            geom.set('contype', '2')
            geom.set('conaffinity', '1')
            geom.set('group', '3')
            # Separate visible copy from its hidden collision approximation.
            if (geom.get('type') == 'mesh' and not
                    geom.get('mesh', '').startswith(('Fixed_Jaw_part', 'Moving_Jaw_part'))):
                visual = deepcopy(geom)
                owner = next(b for b in chassis.iter('body') if geom in list(b))
                visual.set('name', owner.get('name') + '_' + (geom.get('name') or geom.get('mesh')) + '_contact_visual')
                visual.set('contype', '0')
                visual.set('conaffinity', '0')
                visual.set('group', '2')
                visual.set('mass', '0')
                owner.append(visual)
    proxy = chassis.find('geom')
    description = Path(get_package_share_directory('xlerobot_description')) / 'urdf/xlerobot.urdf'
    cart = ET.parse(description).getroot().find("link[@name='base_link']/inertial")
    origin = cart.find('origin')
    center = np.fromstring(origin.get('xyz'), sep=' ')
    rotation = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
    raw = cart.find('inertia')
    xx, yy, zz, xy, xz, yz = [float(raw.get(k)) for k in ('ixx','iyy','izz','ixy','ixz','iyz')]
    inertia = rotation @ np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]]) @ rotation.T
    # The canonical description owns the cart estimate and its computed inertia.
    ET.SubElement(chassis, 'inertial', mass=cart.find('mass').get('value'),
                  pos=' '.join(map(str, center)),
                  fullinertia=' '.join(map(str, [inertia[0,0],inertia[1,1],inertia[2,2],
                                                 inertia[0,1],inertia[0,2],inertia[1,2]])))
    proxy.set('pos', ' '.join(map(str, center)))
    proxy.set('mass', '0')
    proxy.set('name', 'cart_collision')
    actuators = robot.find('actuator')
    for actuator in list(actuators):
        if actuator.get('name') in ('slider_actuator_x', 'slider_actuator_y', 'hinge_actuator_z'):
            actuators.remove(actuator)
    tendon = robot.find('tendon')
    if tendon is not None:
        robot.remove(tendon)
    for index, name in enumerate(WHEEL_BODIES):
        wheel = robot.find(f".//body[@name='{name}']")
        hub = next(b for b in chassis.iter('body') if wheel in list(b))
        joint = hub.find('joint')
        joint.set('damping', '0')
        joint.set('frictionloss', '0')
        joint.set('armature', '0.001')
        joint.set('limited', 'false')
        for geom in list(wheel.findall('geom')) + list(hub.findall('geom')):
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
        inertial = wheel.find('inertial')
        mass = float(inertial.get('mass'))
        raw = np.fromstring(inertial.get('fullinertia'), sep=' ')
        inertia = np.array([[raw[0],raw[3],raw[4]], [raw[3],raw[1],raw[5]], [raw[4],raw[5],raw[2]]])
        # Allocate 2% of the original wheel mass per roller, preserving aggregate
        # mass, COM and inertia rather than adding mass to imported CAD values.
        center = np.fromstring(inertial.get('pos'), sep=' ')
        inertia += mass * (np.dot(center, center)*np.eye(3)-np.outer(center, center))
        roller_mass = mass * .02
        radii = np.array([.009,.018,.009])*config['wheel_radius']/.051
        principal = roller_mass/5 * np.array([radii[1]**2+radii[2]**2, radii[0]**2+radii[2]**2, radii[0]**2+radii[1]**2])
        for template in templates:
            roller = deepcopy(template)
            number = template.get('name').rsplit('_',1)[1]
            roller.set('name', f'wheel{index+1}_roller_{number}')
            p = np.fromstring(template.get('pos'),sep=' ') * config['wheel_radius']/.051
            roller.set('pos', ' '.join(map(str,p)))
            joint = roller.find('joint')
            joint.set('name', f'wheel{index+1}_roller_joint_{number}')
            joint.set('class', 'xlerobot_passive_roller')
            geom = roller.find('geom')
            geom.set('name', f'wheel{index+1}_roller_collision_{number}')
            geom.set('class', 'xlerobot_roller_geom')
            geom.set('mass', str(roller_mass))
            wheel.append(roller)
            q = np.fromstring(roller.get('quat'),sep=' ')
            rotation = Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
            inertia -= rotation@np.diag(principal)@rotation.T + roller_mass*(np.dot(p,p)*np.eye(3)-np.outer(p,p))
        remaining = mass - len(templates)*roller_mass
        remaining_center = mass * center/remaining
        inertia -= remaining*(np.dot(remaining_center,remaining_center)*np.eye(3)-np.outer(remaining_center,remaining_center))
        if np.linalg.eigvalsh(inertia).min() <= 0:
            raise ValueError(f'Roller allocation leaves invalid inertia in {name}')
        inertial.set('mass', str(remaining))
        inertial.set('pos', ' '.join(map(str,remaining_center)))
        inertial.set('fullinertia',' '.join(map(str,[inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]])))
        actuator = actuators.find(f"velocity[@name='wheel{index+1}']")
        actuator.set('kv', str(config['wheel_gain']))
        actuator.set('forcerange', f"-{config['wheel_torque']} {config['wheel_torque']}")
        actuator.set('ctrlrange', f"-{config['wheel_velocity_max']} {config['wheel_velocity_max']}")

    add_cart_contacts(robot)
