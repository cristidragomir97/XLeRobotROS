#!/usr/bin/env python3
"""Acceptance checks against an active mapping simulation; briefly moves the simulated base."""
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo, Imu, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class Check(Node):
    def __init__(self):
        super().__init__('xlerobot_acceptance')
        self.messages = {}
        self.counts = {}
        self.stamps = {key: set() for key in ('rgb', 'depth', 'info')}
        subscriptions = [(Image, '/camera/color/image_raw', 'rgb'),
                         (Image, '/camera/aligned_depth_to_color/image_raw', 'depth'),
                         (Image, '/camera/depth_filtered/image_raw', 'filtered'),
                         (CameraInfo, '/camera/color/camera_info', 'info'),
                         (Imu, '/imu/data', 'imu'), (JointState, '/joint_states', 'joints'),
                         (Odometry, '/odom', 'odom'), (Odometry, '/ground_truth/odom', 'truth'),
                         (PointCloud2, '/camera/points', 'cloud'), (OccupancyGrid, '/map', 'map')]
        self.subscriptions_keepalive = []
        for kind, topic, key in subscriptions:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if key == 'map' else qos_profile_sensor_data
            self.subscriptions_keepalive.append(self.create_subscription(
                kind, topic, lambda msg, k=key: self.receive(k, msg), qos))
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def receive(self, key, msg):
        self.messages[key] = msg
        self.counts[key] = self.counts.get(key, 0)+1
        if key in self.stamps:
            self.stamps[key].add((msg.header.stamp.sec, msg.header.stamp.nanosec))

    def wait(self, predicate, seconds=60):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if predicate():
                return
        raise AssertionError(f'Timed out; received {self.counts}')

    def future(self, future, seconds=60):
        self.wait(future.done, seconds)
        return future.result()

    def drive(self, y, seconds):
        command = Twist()
        command.linear.y = y
        start = self.messages['truth'].header.stamp
        end_sim = start.sec+start.nanosec*1e-9+seconds
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=.02)
            now = self.messages['truth'].header.stamp
            if now.sec+now.nanosec*1e-9 >= end_sim:
                return
        raise AssertionError('Simulation failed to advance during drive')


def main():
    rclpy.init()
    node = Check()
    try:
        node.wait(lambda: set(node.messages) >= {'rgb', 'depth', 'filtered', 'info', 'imu', 'joints', 'odom', 'truth', 'cloud', 'map'})
        node.wait(lambda: len(node.stamps['rgb'] & node.stamps['depth'] & node.stamps['info']) >= 3)
        rgb, depth, info = (node.messages[k] for k in ('rgb', 'depth', 'info'))
        assert rgb.encoding == 'rgb8' and depth.encoding == '32FC1'
        assert len(rgb.data) == rgb.width*rgb.height*3
        pixels = np.frombuffer(bytes(depth.data), dtype='<f4')
        assert np.isfinite(pixels).all() and np.count_nonzero(pixels > 0) > pixels.size*.5
        filtered = np.frombuffer(bytes(node.messages['filtered'].data), dtype='<f4')
        assert np.isfinite(filtered).all() and np.count_nonzero(filtered > 0) <= np.count_nonzero(pixels > 0)
        assert info.k[0] > 0 and info.header.frame_id == rgb.header.frame_id == depth.header.frame_id
        imu = node.messages['imu']
        assert imu.header.frame_id == 'imu_link'
        assert abs(imu.linear_acceleration.z-9.80665) < .2
        mod101 = 'mod101_L_joint_base' in node.messages['joints'].name
        target_joint = 'mod101_L_joint_base' if mod101 else 'Rotation_L'
        assert len(node.messages['joints'].name) == (19 if mod101 else 17)
        assert node.messages['cloud'].width*node.messages['cloud'].height > 100
        cells = np.array(node.messages['map'].data)
        assert np.count_nonzero(cells >= 0) > 100 and np.count_nonzero(cells > 50) > 20
        for parent, child in [('base_link', 'imu_link'), ('base_link', 'head_camera_rgb_optical_frame'),
                              ('odom', 'base_footprint'), ('map', 'odom')]:
            node.wait(lambda p=parent, c=child: node.buffer.can_transform(p, c, Time()), 30)
        assert '/scan' not in dict(node.get_topic_names_and_types())
        assert not any('slam_toolbox' in name for name in node.get_node_names())
        print('PASS: synchronized metric RGB-D, IMU, TF, depth cloud and RTAB-Map occupancy map')
        client = ActionClient(node, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')
        assert client.wait_for_server(timeout_sec=5)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [target_joint]
        point = JointTrajectoryPoint(positions=[.1])
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        handle = node.future(client.send_goal_async(goal))
        assert handle.accepted
        outcome = node.future(handle.get_result_async())
        assert outcome.status == 4 and outcome.result.error_code == 0, outcome
        actual = node.messages['joints']
        assert abs(actual.position[actual.name.index(target_joint)]-.1) < .08
        invalid = FollowJointTrajectory.Goal()
        invalid.trajectory.joint_names = ['unknown_joint']
        invalid.trajectory.points = [JointTrajectoryPoint(positions=[20.], time_from_start=point.time_from_start)]
        assert not node.future(client.send_goal_async(invalid)).accepted
        print('PASS: standard trajectory action, named partial goal and unknown-joint rejection')
        if mod101:
            from xlerobot_sim.configuration import arm_joints
            import xml.etree.ElementTree as ET
            description = ET.parse('/workspace/xlerobot/.runtime/xlerobot_mod101.urdf').getroot()
            for side, controller in [('L','left_arm_controller'), ('R','right_arm_controller')]:
                names = arm_joints('mod101')[side]
                state = node.messages['joints']
                initial = [state.position[state.name.index(n)] for n in names]
                for index, name in enumerate(names):
                    limit = description.find(f"joint[@name='{name}']/limit")
                    initial[index] = float(np.clip(initial[index], float(limit.get('lower')), float(limit.get('upper'))))
                arm_client = ActionClient(node, FollowJointTrajectory, f'/{controller}/follow_joint_trajectory')
                assert arm_client.wait_for_server(timeout_sec=5)
                for values in ([.15,.2,.3,.2,.15,.25,1.], initial):
                    action = FollowJointTrajectory.Goal()
                    action.trajectory.joint_names = names
                    action.trajectory.points = [JointTrajectoryPoint(positions=values, time_from_start=Duration(seconds=3).to_msg())]
                    accepted = node.future(arm_client.send_goal_async(action))
                    assert accepted.accepted
                    result = node.future(accepted.get_result_async())
                    assert result.status == 4 and result.result.error_code == 0, result
                    measured = node.messages['joints']
                    assert all(abs(measured.position[measured.name.index(n)]-v)<.08 for n,v in zip(names, values))
                arm_client.destroy()
            print('PASS: both Mod101 arms move and restore all six axes and PG gripper motors')
        truth0 = node.messages['truth'].pose.pose.position.y
        estimated0 = node.messages['odom'].pose.pose.position.y
        node.drive(.15, 1.5)
        truth1 = node.messages['truth'].pose.pose.position.y
        node.wait(lambda: node.messages['odom'].pose.pose.position.y > estimated0+.12, 30)
        estimated1 = node.messages['odom'].pose.pose.position.y
        assert .15 < truth1-truth0 < .3
        assert abs((estimated1-estimated0)-(truth1-truth0)) < .1
        # Stop publishing commands: verify the simulator's watchdog, not an explicit zero.
        node.wait(lambda: abs(node.messages['truth'].twist.twist.linear.y) < .01, 30)
        node.drive(-.15, 1.5)
        node.publisher.publish(Twist())
        print(f'PASS: lateral RGB-D odometry tracks motion (truth {truth1-truth0:.3f} m, estimate {estimated1-estimated0:.3f} m), command watchdog stops base')
        print('All live checks passed.')
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
