"""Render simulated RGB-D sensors and publish ROS images inside Docker."""
import math
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .renderer import Renderer


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('xlerobot_camera')
        self.declare_parameter('runtime', '/workspace/xlerobot/.runtime')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('rate', 15.0)
        self.runtime = Path(self.get_parameter('runtime').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.filtered_pub = self.create_publisher(Image, '/camera/depth_filtered/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        rate = float(self.get_parameter('rate').value)
        self.sensor = Renderer(self.runtime, rate=rate, width=self.width, height=self.height)
        self.get_logger().info('RGB-D rendering and ROS publication run in the container.')
        self.create_timer(1. / rate, self.publish_frames)

    def publish_frames(self):
        if not self.sensor.step():
            return
        timestamp, rgb, depth, filtered = self.sensor.frame
        header = stamp(timestamp)
        for pub, pixels, encoding, bpp in [(self.rgb_pub, rgb, 'rgb8', 3),
                                           (self.depth_pub, depth, '32FC1', 4),
                                           (self.filtered_pub, filtered, '32FC1', 4)]:
            image = Image()
            image.header.stamp, image.header.frame_id = header, 'head_camera_rgb_optical_frame'
            image.height, image.width = self.height, self.width
            image.encoding, image.is_bigendian, image.step = encoding, 0, self.width * bpp
            image.data = pixels.tobytes()
            pub.publish(image)
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = header, 'head_camera_rgb_optical_frame'
        info.height, info.width = self.height, self.width
        f = self.height / (2 * math.tan(math.radians(60) / 2))
        cx, cy = (self.width - 1) / 2, (self.height - 1) / 2
        info.distortion_model, info.d = 'plumb_bob', [0.] * 5
        info.k = [f, 0., cx, 0., f, cy, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [f, 0., cx, 0., 0., f, cy, 0., 0., 0., 1., 0.]
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sensor.renderer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
