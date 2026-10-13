#!/usr/bin/env python3
"""Geometry/frame, motion, stopping, inertial and RGB-D contracts without running ROS."""
import argparse
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/xlerobot_sim'))
from xlerobot_sim.model import Robot, prepare_scene
from xlerobot_sim.control_io import ControlExchange
from xlerobot_sim.configuration import description_path, control_joints
import struct
import time


def transform(xyz, rpy):
    t = np.eye(4)
    t[:3, 3] = xyz
    t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    return t


def verify_frames(robot, description):
    model, data = robot.model, robot.data
    urdf = ET.parse(description).getroot()
    joints = urdf.findall('joint')
    rng = np.random.default_rng(23)
    for _ in range(6):
        for name, (_, address) in robot.positions.items():
            jid = model.joint(name).id
            lo, hi = model.jnt_range[jid]
            data.qpos[address] = rng.uniform(max(lo, -.5), min(hi, .5))
        for joint in joints:
            mimic = joint.find('mimic')
            if mimic is not None:
                master = model.joint(mimic.get('joint')).qposadr[0]
                slave = model.joint(joint.get('name')).qposadr[0]
                data.qpos[slave] = data.qpos[master]*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))
        # Sweep a genuine floating base, including roll/pitch/height.
        data.qpos[:3] = model.qpos0[:3] + rng.uniform(-.1, .1, 3)
        initial = Rotation.from_quat(model.qpos0[3:7][[1,2,3,0]])
        rotation = initial * Rotation.from_euler('xyz', rng.uniform(-.3,.3,3))
        data.qpos[3:7] = rotation.as_quat()[[3,0,1,2]]
        mujoco.mj_forward(model, data)
        p, yaw = robot.pose()
        frames = {'base_footprint': transform(p, Rotation.from_quat(robot.orientation()[[1,2,3,0]]).as_euler('xyz'))}
        pending = list(joints)
        while pending:
            ready = [j for j in pending if j.find('parent').get('link') in frames]
            assert ready, 'Disconnected URDF frame tree'
            for joint in ready:
                parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
                origin = joint.find('origin')
                attrs = {} if origin is None else origin.attrib
                t = transform(np.fromstring(attrs.get('xyz','0 0 0'), sep=' '),
                              np.fromstring(attrs.get('rpy','0 0 0'), sep=' '))
                if joint.get('type') != 'fixed':
                    q = data.qpos[model.joint(joint.get('name')).qposadr[0]]
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    motion = np.eye(4)
                    if joint.get('type') == 'prismatic':
                        motion[:3, 3] = axis*q
                    else:
                        motion[:3, :3] = Rotation.from_rotvec(axis*q).as_matrix()
                    t = t @ motion
                frames[child] = frames[parent] @ t
                pending.remove(joint)
        for b in range(1, model.nbody):
            name = model.body(b).name
            if '_roller_' in name or name.startswith('cart_caster_'):
                continue  # Internal passive physics joints do not enter the ROS tree.
            frame = frames['base_link' if name == 'chassis' else name]
            r = data.xmat[b].reshape(3, 3)
            offset = model.jnt_pos[model.body_jntadr[b]] if model.body_jntnum[b] == 1 else np.zeros(3)
            expected = np.eye(4)
            expected[:3, :3] = robot.r0.T @ r
            expected[:3, 3] = robot.r0.T @ (data.xpos[b]+r@offset-robot.p0) + [0., 0., robot.p0[2]]
            assert np.allclose(frame, expected, atol=2e-8), f'URDF/MJCF frame mismatch: {name}'
        cam = model.camera('rgbd').id
        optical = frames['head_camera_rgb_optical_frame']
        expected_rotation = robot.r0.T @ data.cam_xmat[cam].reshape(3, 3) @ np.diag([1., -1., -1.])
        assert np.allclose(optical[:3, :3], expected_rotation, atol=2e-8), 'Optical axes mismatch'
        assert np.allclose(optical[:3, 3], robot.r0.T @ (data.cam_xpos[cam]-robot.p0)+[0., 0., robot.p0[2]], atol=2e-8)
    print('PASS: full URDF/MuJoCo body and camera transforms at six nonzero poses')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--configuration', choices=['standard','mod101'], default='standard')
    configuration = parser.parse_args().configuration
    expected_count = len(control_joints(configuration))
    with tempfile.TemporaryDirectory(prefix='xlerobot-check-') as directory:
        description = description_path(configuration, directory)
        path = prepare_scene(ROOT / 'upstream/simulation/mujoco', directory, configuration=configuration, description=description)
        robot = Robot(path)
        if configuration == 'mod101':
            robot.step([0.,0.,0.],1500)
            for name in robot.control_names[3:-2]:
                joint = robot.model.joint(name)
                low, high = robot.model.jnt_range[joint.id]
                assert low <= robot.data.qpos[joint.qposadr[0]] <= high, name
                source = ET.parse(description).getroot()
                limit = source.find(f"joint[@name='{name}']/limit")
                hardware = source.find(f"ros2_control/joint[@name='{name}']/command_interface")
                assert np.allclose([low,high], [float(limit.get('lower')),float(limit.get('upper'))])
                assert np.allclose([low,high], [float(hardware.find("param[@name='min']").text),float(hardware.find("param[@name='max']").text)])
            print('PASS: settled Mod101 joints stay inside matching URDF/MuJoCo/control limits')
            from check_tower_clearance import Clearance
            clearance = Clearance(robot)
            assert not any(clearance.pairs(side) for side in ('L','R')), 'Settled arm meshes intersect the tower'
            print('PASS: settled Mod101 arm/tower triangle meshes do not intersect')
        verify_frames(robot, description)
        # Gripper visuals must use detailed source meshes, not convex collision
        # decompositions accidentally displayed as jaws.
        if configuration == 'standard':
            for mesh in ('full_Fixed_Jaw', 'full_Fixed_Jaw_Motor', 'full_Moving_Jaw'):
                assert robot.model.mesh_facenum[robot.model.mesh(mesh).id] > 1000
            for name in ('Fixed_Jaw', 'Moving_Jaw', 'Fixed_Jaw_2', 'Moving_Jaw_2'):
                body = robot.model.body(name).id
                geoms = range(robot.model.body_geomadr[body],
                              robot.model.body_geomadr[body]+robot.model.body_geomnum[body])
                visible = [g for g in geoms if robot.model.geom_group[g] == 2]
                assert len(visible) == (2 if name.startswith('Fixed') else 1)
                assert all(robot.model.geom_contype[g] == 0 and robot.model.geom_conaffinity[g] == 0 for g in visible)
            print('PASS: detailed gripper chassis/motor meshes on both arms; collision hulls hidden')
        else:
            source = ET.parse(description).getroot()
            for side in ('L','R'):
                arm = [j for j in source.findall('joint') if j.get('name').startswith(f'mod101_{side}_')]
                rails = [b for j in arm for b in source.find(f"link[@name='{j.find('child').get('link')}']").findall('visual/geometry/box')]
                assert sum(np.allclose(np.fromstring(b.get('size'),sep=' '), [.14,.02,.02]) for b in rails) == 2
            assert len(source.findall('joint/mimic')) == 4
            # Physical equality constraints must carry the rack sliders with both motors.
            robot.set_positions(['mod101_L_6','mod101_R_6'], [1.,1.])
            robot.step([0.,0.,0.],1000)
            for joint in source.findall('joint'):
                mimic = joint.find('mimic')
                if mimic is not None:
                    actual = robot.data.qpos[robot.model.joint(joint.get('name')).qposadr[0]]
                    master = robot.data.qpos[robot.model.joint(mimic.get('joint')).qposadr[0]]
                    assert abs(actual-master*float(mimic.get('multiplier'))) < .0002
            print('PASS: two 140/140 mm Mod101 arms, six arm axes plus PG motor and four physical rack mimics')
        # Removing ground support must produce gravity-driven falling, not an
        # imposed chassis height. Wheels are the only base actuators.
        assert robot.model.joint('floating_base').type == mujoco.mjtJoint.mjJNT_FREE
        assert robot.model.nu == expected_count
        assert sum('_roller_joint_' in (robot.model.joint(i).name or '') for i in range(robot.model.njnt)) == 48
        mujoco.mj_resetData(robot.model, robot.data)
        robot.data.qpos[2] += .3
        mujoco.mj_forward(robot.model, robot.data)
        height = robot.data.xpos[robot.base,2]
        robot.step([0.,0.,0.],50)
        assert height-robot.data.xpos[robot.base,2] > .04
        robot.step([0.,0.,0.],1450)
        assert .02 < robot.data.xpos[robot.base,2] < .05
        assert robot.data.ncon > 0
        assert np.isfinite(robot.data.qpos).all()
        print('PASS: floating base falls under gravity and lands on passive roller contacts')
        for label, command in [('forward', [.2, 0., 0.]), ('strafe', [0., .2, 0.]),
                               ('rotation', [0., 0., .3])]:
            mujoco.mj_resetData(robot.model, robot.data)
            mujoco.mj_forward(robot.model, robot.data)
            robot.step([0.,0.,0.],500)
            robot.step(command, 1000)
            p, yaw = robot.pose()
            assert np.isfinite(robot.data.qpos).all()
            if label == 'forward':
                assert .3 < p[0] < .5 and abs(p[1]) < .03, (label, p)
            elif label == 'strafe':
                assert .3 < p[1] < .5 and abs(p[0]) < .03, (label, p)
            else:
                assert .5 < yaw < .7, (label, yaw)
            robot.step([0., 0., 0.], 500)
            linear, angular = robot.twist()
            assert np.linalg.norm(linear[:2]) < .01 and abs(angular[2]) < .01
            print(f'PASS: {label} in ROS base axes and stopping')
        mujoco.mj_resetData(robot.model, robot.data)
        mujoco.mj_forward(robot.model, robot.data)
        robot.step([0., 0., 0.], 500)
        assert np.allclose(robot.data.sensor('imu_accel').data, [0., 0., 9.80665], atol=.05)
        assert np.linalg.norm(robot.data.sensor('imu_gyro').data) < .01
        imu_id = robot.model.body('imu_link').id
        local = robot.data.xmat[robot.base].reshape(3,3).T @ (robot.data.xpos[imu_id]-robot.data.xpos[robot.base])
        assert np.allclose(local, [0.,0.,-.015])
        caster_contacts = {robot.model.geom(int(g)).name for contact in robot.data.contact
                           for g in contact.geom if 'cart_caster' in (robot.model.geom(int(g)).name or '')}
        assert caster_contacts == {f'cart_caster_{i}_contact' for i in range(4)}, caster_contacts
        for i in range(4):
            geom = robot.model.geom(f'cart_caster_{i}_contact').id
            lowest = robot.data.geom_xpos[geom,2]-robot.model.geom_size[geom,0]
            assert abs(lowest)<.001, (i,lowest)
        print('PASS: all four cart tyres contact the ground with passive swivel/rolling motion')
        print('PASS: bottom-base IMU location and stationary specific force')
        scene_option = mujoco.MjvOption()
        scene_option.geomgroup[3] = 0
        with mujoco.Renderer(robot.model, height=480, width=640) as renderer:
            renderer.update_scene(robot.data, camera='rgbd', scene_option=scene_option)
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            depth = renderer.render()
            assert rgb.shape == (480, 640, 3) and rgb.std() > 15
            assert np.isfinite(depth).all() and np.count_nonzero((depth > .15) & (depth < 8.)) > depth.size * .5
            # Metric depth must agree with a central optical ray's geometry intersection.
            cam = robot.model.camera('rgbd').id
            direction = -robot.data.cam_xmat[cam].reshape(3, 3)[:, 2].copy()
            geom = np.array([-1], dtype=np.int32)
            distance = mujoco.mj_ray(robot.model, robot.data, robot.data.cam_xpos[cam],
                                     direction, None, 1, robot.model.body('head_camera_link').id, geom)
            assert abs(float(depth[240, 320])-distance) < .03, (depth[240, 320], distance)
            print(f'PASS: textured RGB and metric depth (central wall {distance:.3f} m)')
            renderer.disable_depth_rendering()
            renderer.enable_segmentation_rendering()
            segmentation = renderer.render()
            ids = segmentation[:, :, 0]
            geometry = (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & (ids >= 0)
            robot_pixels = geometry & robot.robot_geom_mask[np.clip(ids, 0, robot.model.ngeom-1)]
            assert not robot_pixels[240, 320], 'Self-filter must preserve central room depth'
            # The corrected mounts can place both arms outside the head camera view.
            # Exercise robot segmentation from an external view instead.
            camera = mujoco.MjvCamera()
            camera.lookat[:] = robot.data.xpos[robot.base] + [0., 0., .6]
            camera.distance, camera.azimuth, camera.elevation = 2.5, 120., -20.
            renderer.update_scene(robot.data, camera=camera, scene_option=scene_option)
            segmentation = renderer.render()
            ids = segmentation[:, :, 0]
            geometry = (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & (ids >= 0)
            robot_pixels = geometry & robot.robot_geom_mask[np.clip(ids, 0, robot.model.ngeom-1)]
            assert robot_pixels.sum() > 100, 'Self-filter must identify the full robot'
            print(f'PASS: self-filter identifies {robot_pixels.sum()} robot pixels without masking the wall')
        home_path = prepare_scene(ROOT / 'upstream/simulation/mujoco', Path(directory)/'home', 'robocasa_home', configuration=configuration, description=description)
        home = Robot(home_path)
        assert len(home.joint_ids) == expected_count, 'Furniture joints must not leak into robot joint states'
        fridge = home.model.body('kitchen_fridge').id
        assert not np.any(home.robot_geom_mask[home.model.body_rootid[home.model.geom_bodyid] == fridge]), 'Self-filter masks world furniture'
        expected = {'full_raskogbody', 'full_raskogwheel1', 'full_raskogwheel2',
                    'full_topbase1', 'full_topbase2', 'full_tophead3', 'full_tophead4',
                    'full_tophead5', 'full_tophead6', 'full_XLeRobot_camera1', 'full_XLeRobot_camera2'}
        assert expected <= {home.model.mesh(i).name for i in range(home.model.nmesh)}
        assert all(home.model.geom_group[i] == 3 for i in range(home.model.ngeom)
                   if home.model.geom_bodyid[i] == home.base and home.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX)
        home.step([0., 0., 0.], 1000)
        assert np.isfinite(home.data.qpos).all()
        print('PASS: complete cart/head/wrist meshes, shared home, robot-only state and self-filter')
        # A controller process writes the documented ABI; only physics owns state.
        mujoco.mj_resetData(robot.model, robot.data)
        mujoco.mj_forward(robot.model, robot.data)
        exchange = ControlExchange(Path(directory) / 'control.bin', robot)
        interfaces = ET.parse(description).getroot().findall('ros2_control/joint')
        assert exchange.names == [j.get('name') for j in interfaces]
        command = np.frombuffer(exchange.memory, dtype='<f8', count=expected_count, offset=1080)
        command[:3] = robot.base_control.wheel_speeds([0., .15, 0.])
        command[3:] = [np.clip(robot.data.qpos[robot.positions[n][1]], *robot.model.actuator_ctrlrange[robot.positions[n][0]]) for n in exchange.names[3:]]
        struct.pack_into('<Q', exchange.memory, 48, 1)
        struct.pack_into('<d', exchange.memory, 40, time.monotonic())
        struct.pack_into('<Q', exchange.memory, 24, 2)
        assert np.allclose(exchange.apply(), [0., .15, 0.])
        struct.pack_into('<d', exchange.memory, 40, time.monotonic()-2.)
        assert np.allclose(exchange.apply(), np.zeros(3)), 'Stale controller must stop the base'
        struct.pack_into('<d', exchange.memory, 40, time.monotonic())
        command[0] = np.nan
        assert np.allclose(exchange.apply(), np.zeros(3)), 'Nonfinite commands must stop the base'
        del command
        exchange.close()
        print('PASS: ROS control joint ABI, LeKiwi wheel kinematics and stale/nonfinite command stop')
        print('All model checks passed.')


if __name__ == '__main__':
    main()
