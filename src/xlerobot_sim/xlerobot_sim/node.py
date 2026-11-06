"""One physics owner, ROS commands/actions, synchronized RGB-D and an ideal base IMU."""
import math
import os
import subprocess
import sys
from pathlib import Path
import threading
import time
import mujoco
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from .model import Robot
from .control_io import ControlExchange


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def duration(value):
    return value.sec + value.nanosec * 1e-9


class Simulator(Node):
    def __init__(self):
        super().__init__('xlerobot_sim')
        self.declare_parameter('upstream', '/workspace/xlerobot/upstream/simulation/mujoco')
        self.declare_parameter('runtime', '/workspace/xlerobot/.runtime')
        self.declare_parameter('world', 'robocasa_home')
        self.declare_parameter('configuration', 'standard')
        self.declare_parameter('description', '')
        self.declare_parameter('world_assets', '')
        self.declare_parameter('headless', False)
        self.declare_parameter('camera_rate', 15.0)
        self.declare_parameter('viewer_rate', 50.0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('ground_truth_tf', False)
        self.runtime = Path(self.get_parameter('runtime').value)
        # The compiler retains large mesh/texture caches. Isolate preparation
        # from the long-lived physics owner and reuse the exact compiled model.
        command = [sys.executable, '-m', 'xlerobot_sim.prepare_model']
        for parameter in ('upstream', 'runtime', 'world', 'world_assets', 'configuration', 'description'):
            command.extend(['--'+parameter.replace('_', '-'), str(self.get_parameter(parameter).value)])
        subprocess.run(command, check=True)
        self.path = self.runtime / 'scene.mjb'
        self.robot = Robot(self.path)
        self.lock = threading.RLock()
        self.width = int(self.get_parameter('camera_width').value)
        self.height = int(self.get_parameter('camera_height').value)
        self.camera_rate = float(self.get_parameter('camera_rate').value)
        if self.camera_rate <= 0:
            raise ValueError('camera_rate must be positive')
        self.camera_period = 1 / self.camera_rate
        self.viewer_rate = float(self.get_parameter('viewer_rate').value)
        if self.viewer_rate <= 0:
            raise ValueError('viewer_rate must be positive')
        self.viewer_period = 1 / self.viewer_rate
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        # Reliable camera transport inside the container lets rgbd_odometry use reliable
        # /odom publishers required by Nav2; sensor-data subscribers remain compatible.
        self.tf = TransformBroadcaster(self)
        self.exchange = ControlExchange(self.runtime / 'control.bin', self.robot)
        self.get_logger().info('MuJoCo physics/sensors ready; ros2_control owns joint and wheel commands.')

    def publish_state(self):
        model, data = self.robot.model, self.robot.data
        now = stamp(data.time)
        self.clock_pub.publish(Clock(clock=now))
        self.exchange.update()
        p, yaw = self.robot.pose()
        lin, ang = self.robot.twist()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(p[0]), float(p[1])
        odom.pose.pose.position.z = float(p[2])
        q = self.robot.orientation()
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = map(float, q)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, lin)
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, ang)
        self.odom_pub.publish(odom)
        if self.get_parameter('ground_truth_tf').value:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = 'base_footprint'
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, p)
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = now, 'imu_link'
        # Ideal simulated orientation is expressed in initial odom frame, not MuJoCo world.
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = map(float, q)
        for field, sensor_name in [(imu.angular_velocity, 'imu_gyro'),
                                   (imu.linear_acceleration, 'imu_accel')]:
            values = data.sensor(sensor_name).data
            field.x, field.y, field.z = map(float, values)
        imu.orientation_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        imu.angular_velocity_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        imu.linear_acceleration_covariance = [1e-4, 0., 0., 0., 1e-4, 0., 0., 0., 1e-4]
        self.imu_pub.publish(imu)

    def snapshot(self, name):
        temp = self.runtime / f'{name}.tmp'
        data = self.robot.data
        with temp.open('wb') as stream:
            np.savez(stream, time=data.time, qpos=data.qpos, qvel=data.qvel, ctrl=data.ctrl)
        os.replace(temp, self.runtime / f'{name}.npz')


def main(args=None):
    rclpy.init(args=args)
    node = Simulator()
    node.get_logger().info(
        'Physics and RGB-D sensors run in Docker; pixi run viewer-native opens the host frontend.')
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    next_camera = .2
    next_viewer = 0.
    deadline = time.monotonic()
    ticks = 5
    period = ticks * node.robot.model.opt.timestep
    try:
        while rclpy.ok():
            with node.lock:
                command = node.exchange.apply()
                node.robot.step(command, ticks)
                if not np.isfinite(node.robot.data.qpos).all():
                    raise RuntimeError('Non-finite simulation state')
                node.publish_state()
                sim_time = node.robot.data.time
                if sim_time >= next_camera:
                    node.snapshot('physics')
                    next_camera = sim_time + node.camera_period
                if sim_time >= next_viewer:
                    node.snapshot('viewer')
                    next_viewer = sim_time + node.viewer_period
            deadline += period
            delay = deadline-time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -.5:
                deadline = time.monotonic()  # slow host: reduce real-time factor, never skip physics
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        node.exchange.close()
        node.destroy_node()
