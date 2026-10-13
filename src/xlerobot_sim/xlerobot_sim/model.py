"""Owned overlays and controls; upstream MJCF and meshes remain untouched."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml
from .visuals import add_full_visuals
from .contact_base import add_contact_base
from lekiwi_mujoco.simulation import RobotControl, WHEELS

from .configuration import arm_joints, control_joints, HEAD_JOINTS, description_path
ARM_JOINTS = arm_joints()
CAMERA_POS = (0.055, 0.02, 0.0225)  # upstream ManiSkill head camera + RGB offset
IMU_POS = (0.0, 0.0, -0.015)  # provisional cube beneath printed lower plate
IMU_SIZE = 0.02


def base_configuration():
    path = Path(__file__).resolve().parents[2] / 'xlerobot_control/config/base.yaml'
    if not path.is_file():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('xlerobot_control')) / 'config/base.yaml'
    config = yaml.safe_load(path.read_text())
    for key in ('wheel_radius', 'robot_radius', 'linear_max', 'angular_max', 'wheel_velocity_max', 'wheel_gain', 'wheel_torque'):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f'Invalid base configuration: {key}')
    if not math.isfinite(config['wheel_offset']):
        raise ValueError('Invalid wheel offset')
    return config


def prepare_scene(upstream, destination, world='room', world_assets=None, configuration='standard', description=None):
    """Copy XML into runtime storage, reference pinned assets, add owned sensors/world."""
    if world not in ('room', 'empty', 'robocasa_home'):
        raise ValueError(f'Unknown world: {world}')
    upstream, destination = Path(upstream), Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    robot = ET.parse(upstream / 'xlerobot.xml').getroot()
    robot.find('compiler').set('meshdir', str((upstream / 'assets').resolve()))
    add_full_visuals(robot, upstream.parent / 'Maniskill/assets/xlerobot')
    chassis = robot.find(".//body[@name='chassis']")
    head = robot.find(".//body[@name='head_tilt_link']")
    # Use the upstream camera frame and existing head meshes; no extra housing.
    cam_body = ET.SubElement(head, 'body', name='head_camera_link', pos='0.055 0 0.0225')
    # MuJoCo looks along camera -Z with +Y up. Forward +X, up +Z in head frame.
    ET.SubElement(cam_body, 'camera', name='rgbd', pos='0 0.02 0',
                  xyaxes='0 -1 0 0 0 1', fovy='60')
    imu = ET.SubElement(chassis, 'body', name='imu_link', pos=' '.join(map(str, IMU_POS)))
    ET.SubElement(imu, 'geom', type='box', size='0.01 0.01 0.01',
                  rgba='0.1 0.8 0.4 1', contype='0', conaffinity='0', mass='0.005')
    ET.SubElement(imu, 'site', name='imu_site', size='0.002')
    sensors = ET.SubElement(robot, 'sensor')
    ET.SubElement(sensors, 'accelerometer', name='imu_accel', site='imu_site')
    ET.SubElement(sensors, 'gyro', name='imu_gyro', site='imu_site')
    ET.SubElement(sensors, 'framequat', name='imu_orientation', objtype='site', objname='imu_site')
    actuators = robot.find('actuator')
    add_contact_base(robot, base_configuration())
    if configuration == 'mod101':
        from .mod101 import replace_arms
        replace_arms(robot, description or description_path(configuration, destination))
    elif configuration != 'standard':
        raise ValueError(f'Unknown configuration: {configuration}')
    for name in HEAD_JOINTS:
        joint = robot.find(f".//joint[@name='{name}']")
        joint.attrib.pop('actuatorfrcrange', None)
        joint.set('damping', '1')
        ET.SubElement(actuators, 'position', name=name, joint=name,
                      kp='100', kv='20', forcerange='-20 20', ctrlrange=joint.get('range'))
    # Upstream mirrored arm joints inherit actuator ranges for the opposite sign.
    # The owned overlay makes each position actuator's target range match its own joint.
    for actuator in actuators.findall('position'):
        joint = robot.find(f".//joint[@name='{actuator.get('joint')}']")
        actuator.set('ctrlrange', joint.get('range'))
    ET.indent(robot)
    ET.ElementTree(robot).write(destination / 'robot.xml')
    scene = ET.parse(upstream / 'scene.xml').getroot()
    scene.find('include').set('file', 'robot.xml')
    scene.find('visual/global').set('offwidth', '640')
    scene.find('visual/global').set('offheight', '480')
    if world == 'room':
        assets = scene.find('asset')
        ET.SubElement(assets, 'texture', name='wall_pattern', type='2d', builtin='checker',
                      rgb1='0.65 0.7 0.75', rgb2='0.25 0.35 0.45', width='256', height='256')
        ET.SubElement(assets, 'material', name='wall_material', texture='wall_pattern',
                      texrepeat='8 4', texuniform='true')
        wb = scene.find('worldbody')
        # Asymmetric, textured geometry provides depth returns and visual features.
        for name, pos, size in [('north', '0 4 1.5', '4 0.1 1.5'),
                                ('south', '0 -4 1.5', '4 0.1 1.5'),
                                ('east', '4 0 1.5', '0.1 4 1.5'),
                                ('west', '-4 0 1.5', '0.1 4 1.5')]:
            ET.SubElement(wb, 'geom', name=name, type='box', pos=pos, size=size,
                          material='wall_material')
        rng = np.random.default_rng(42)
        for i in range(45):
            # Feature panels on inner north/south walls. Source world is rotated vs odom.
            y = 3.88 if i % 2 else -3.88
            x, z = rng.uniform(-3.7, 3.7), rng.uniform(0.25, 2.7)
            rgba = ' '.join(map(str, [*rng.uniform(0.05, 0.95, 3), 1]))
            ET.SubElement(wb, 'geom', name=f'panel_{i}', type='box',
                          pos=f'{x} {y} {z}', size='0.08 0.025 0.12', rgba=rgba)
        for i, (x, y, z, sx, sy, sz) in enumerate([
                (1.8, 2.2, 0.6, .45, .5, .6), (-1.8, 2.8, .8, .55, .3, .8),
                (2.4, -2.0, .45, .4, .6, .45), (-2.5, -1.8, .6, .5, .4, .6)]):
            ET.SubElement(wb, 'geom', name=f'furniture_{i}', type='box',
                          pos=f'{x} {y} {z}', size=f'{sx} {sy} {sz}',
                          material='wall_material')
    ET.indent(scene)
    path = destination / 'scene.xml'
    ET.ElementTree(scene).write(path)
    if world == 'robocasa_home':
        from .world import add_home, default_assets
        add_home(path, world_assets or default_assets())
    return path


class Robot:
    def __init__(self, path):
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.base = self.model.body('chassis').id
        self.robot_geom_mask = self.model.body_rootid[self.model.geom_bodyid] == self.base
        self.configuration = 'mod101' if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'mod101_L_joint_base') >= 0 else 'standard'
        self.control_names = control_joints(self.configuration)
        commanded = set(self.control_names)
        self.joint_ids = [i for i in range(self.model.njnt)
                          if self.model.joint(i).name in commanded]
        self.r0 = self.data.xmat[self.base].reshape(3, 3).copy()
        self.p0 = self.data.xpos[self.base].copy()
        self.positions = {}
        for i in range(self.model.nu):
            if self.model.actuator_biastype[i] == mujoco.mjtBias.mjBIAS_AFFINE and self.model.actuator_biasprm[i, 1] != 0:
                jid = int(self.model.actuator_trnid[i, 0])
                name = self.model.joint(jid).name
                self.positions[name] = (i, int(self.model.jnt_qposadr[jid]))
                self.data.ctrl[i] = self.data.qpos[self.model.jnt_qposadr[jid]]
        # Reuse LeKiwi's wheel kinematics/limits through a binding for XLeRobot's
        # existing actuator names and contact-driven wheels.
        base_config = base_configuration()
        class WheelBindings:
            wheels = [self.model.actuator(n).id for n in ('wheel1', 'wheel2', 'wheel3')]
            payload = ()
            def numeric(bindings, name):
                if name == 'wheel_kinematics':
                    return np.array([base_config[k] for k in ('wheel_radius', 'robot_radius', 'wheel_offset')])
                if name == 'base_velocity_limits':
                    return np.array([base_config['linear_max'], base_config['linear_max'], base_config['angular_max']])
                if name.startswith('velocity_limit_'):
                    return np.array([base_config['wheel_velocity_max']])
                raise ValueError(name)
        self.base_control = RobotControl(self.model, WheelBindings())
        self.jp = np.zeros((3, self.model.nv))
        self.jr = np.zeros((3, self.model.nv))

    def pose(self):
        r = self.r0.T @ self.data.xmat[self.base].reshape(3, 3)
        p = self.r0.T @ (self.data.xpos[self.base] - self.p0)
        p += np.array([0., 0., self.p0[2]]) - r @ np.array([0., 0., self.p0[2]])
        return p, math.atan2(r[1, 0], r[0, 0])

    def orientation(self):
        quaternion = np.empty(4)
        rotation = self.r0.T @ self.data.xmat[self.base].reshape(3, 3)
        mujoco.mju_mat2Quat(quaternion, rotation.ravel())
        return quaternion  # MuJoCo order: w, x, y, z

    def twist(self):
        mujoco.mj_jacBody(self.model, self.data, self.jp, self.jr, self.base)
        r = self.data.xmat[self.base].reshape(3, 3)
        angular = r.T @ (self.jr @ self.data.qvel)
        linear = r.T @ (self.jp @ self.data.qvel)
        # Express twist at the same rigid reference point as pose()/base_footprint.
        linear += np.cross(angular, [0., 0., -self.p0[2]])
        return linear, angular

    def drive(self, command):
        wheel_vel = self.base_control.wheel_speeds(command)
        self.data.ctrl[self.base_control.wheels] = wheel_vel

    def set_positions(self, names, positions):
        values = np.asarray(positions, dtype=float)
        if len(names) != len(values) or len(set(names)) != len(names) or not np.isfinite(values).all():
            raise ValueError('Joint commands require unique names and matching finite positions')
        for name, value in zip(names, values):
            if name not in self.positions:
                raise ValueError(f'Unknown position-controlled joint: {name}')
            aid, _ = self.positions[name]
            jid = int(self.model.actuator_trnid[aid, 0])
            low, high = self.model.jnt_range[jid]
            alo, ahi = self.model.actuator_ctrlrange[aid]
            if not max(low, alo) <= value <= min(high, ahi):
                raise ValueError(f'{name} target {value} exceeds joint/actuator limits')
        for name, value in zip(names, values):
            self.data.ctrl[self.positions[name][0]] = value

    def step(self, command, count=1):
        for _ in range(count):
            self.drive(command)
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
