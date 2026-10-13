#!/usr/bin/env python3
"""Exercise real visual tracking loss with a ceiling-facing simulated camera."""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from control_msgs.action import FollowJointTrajectory
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener


class Check(Node):
    def __init__(self):
        super().__init__('xlerobot_tracking_acceptance')
        self.data = {}
        self.subs = [self.create_subscription(kind, topic, lambda m,k=key:self.data.update({k:m}), qos_profile_sensor_data)
                     for kind,topic,key in [(OdomInfo,'/odom_info_lite','info'),(JointState,'/joint_states','joints'),(Odometry,'/odom','odom')]]
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

    def wait(self, condition, timeout=20):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self, timeout_sec=.05)
            if condition(): return
        raise AssertionError('Tracking condition timed out')

    def tilt(self, value):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['head_tilt_joint']
        point = JointTrajectoryPoint(positions=[value])
        point.time_from_start.sec=3
        goal.trajectory.points=[point]
        future=self.client.send_goal_async(goal)
        self.wait(future.done)
        handle=future.result()
        assert handle.accepted
        result=handle.get_result_async()
        self.wait(result.done)
        assert result.result().result.error_code==0


def main():
    rclpy.init()
    node=Check()
    initial=None
    try:
        assert node.client.wait_for_server(timeout_sec=10)
        node.wait(lambda: {'joints','info','odom'} <= node.data.keys() and not node.data['info'].lost)
        initial=node.data['joints'].position[node.data['joints'].name.index('head_tilt_joint')]
        before=node.data['odom'].pose.pose.position
        node.tilt(-.76)  # Optical axis points up into the featureless ceiling.
        node.wait(lambda: node.data['info'].lost)
        print('PASS: real RGB-D tracking loss detected',flush=True)
        end=time.monotonic()+3
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
        t=lambda s:s.sec+s.nanosec*1e-9
        assert node.data['odom'].pose.pose.orientation.w!=0
        print('PASS: invalid null odometry suppressed during tracking loss',flush=True)
        node.tilt(initial)
        node.wait(lambda: not node.data['info'].lost,30)
        node.wait(lambda: t(node.data['odom'].header.stamp)>=t(node.data['info'].header.stamp)-.2)
        node.wait(lambda: t(node.data['info'].header.stamp)-t(node.buffer.lookup_transform('odom','base_footprint',Time()).header.stamp)<.5)
        after=node.data['odom'].pose.pose.position
        jump=math.hypot(after.x-before.x,after.y-before.y)
        assert jump<.15, f'Pose discontinuity {jump:.3f} m'
        print(f'PASS: automatic visual recovery, stationary pose difference {jump:.3f} m',flush=True)
    finally:
        if initial is not None:
            node.tilt(initial)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__': main()
