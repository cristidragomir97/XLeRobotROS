"""Read-only RGB-D renderer; never steps physics or writes controller commands."""
import math
import os
import time
from pathlib import Path
import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image
from .model import Robot


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('xlerobot_camera')
        self.declare_parameter('runtime', '/workspace/xlerobot/.runtime')
        self.runtime = Path(self.get_parameter('runtime').value)
        deadline = time.monotonic() + 30.
        while not (self.runtime / 'physics.npz').is_file():
            if time.monotonic() > deadline:
                raise RuntimeError('Physics owner did not publish a fresh startup snapshot')
            time.sleep(.05)
        self.robot = Robot(self.runtime / 'scene.xml')
        self.height, self.width = 480, 640
        self.scene_option = mujoco.MjvOption()
        self.scene_option.geomgroup[3] = 0
        self.renderer = mujoco.Renderer(self.robot.model, height=self.height, width=self.width)
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.filtered_depth_pub = self.create_publisher(Image, '/camera/depth_filtered/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.last_time = -1.
        self.create_timer(.02, self.render)

    def render(self):
        path = self.runtime / 'physics.npz'
        if not path.is_file():
            return
        with np.load(path) as state:
            timestamp = float(state['time'])
            if timestamp <= self.last_time:
                return
            data = self.robot.data
            data.qpos[:] = state['qpos']
            data.qvel[:] = state['qvel']
            data.ctrl[:] = state['ctrl']
            data.time = timestamp
        mujoco.mj_forward(self.robot.model, data)
        self.publish_camera(data)
        self.snapshot(data)
        self.last_time = timestamp

    def publish_camera(self, data):
        self.renderer.update_scene(data, camera='rgbd', scene_option=self.scene_option)
        self.renderer.disable_depth_rendering()
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().astype('<f4')
        depth[(depth < .15) | (depth > 8.) | ~np.isfinite(depth)] = 0.
        self.latest_rgb, self.latest_depth = rgb, depth
        self.renderer.disable_depth_rendering()
        # Keep raw RGB-D intact, but mask robot geometry from mapping/obstacle inputs.
        # This is an ideal simulation self-filter, analogous to a robot-body depth filter.
        self.renderer.enable_segmentation_rendering()
        segmentation = self.renderer.render()
        self.renderer.disable_segmentation_rendering()
        ids = segmentation[:, :, 0]
        geometry = (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & (ids >= 0)
        robot_pixels = geometry & self.robot.robot_geom_mask[np.clip(ids, 0, self.robot.model.ngeom-1)]
        filtered_depth = depth.copy()
        filtered_depth[robot_pixels] = 0.
        now = stamp(data.time)
        for pub, pixels, encoding, bpp in [(self.rgb_pub, rgb, 'rgb8', 3),
                                           (self.depth_pub, depth, '32FC1', 4),
                                           (self.filtered_depth_pub, filtered_depth, '32FC1', 4)]:
            image = Image()
            image.header.stamp, image.header.frame_id = now, 'head_camera_rgb_optical_frame'
            image.height, image.width = self.height, self.width
            image.encoding, image.is_bigendian, image.step = encoding, 0, self.width*bpp
            image.data = pixels.tobytes()
            pub.publish(image)
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = now, 'head_camera_rgb_optical_frame'
        info.height, info.width = self.height, self.width
        f = self.height/(2*math.tan(math.radians(60)/2))
        cx, cy = (self.width-1)/2, (self.height-1)/2
        info.distortion_model, info.d = 'plumb_bob', [0.]*5
        info.k = [f, 0., cx, 0., f, cy, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [f, 0., cx, 0., 0., f, cy, 0., 0., 0., 1., 0.]
        self.info_pub.publish(info)

    def snapshot(self, data):
        temp = self.runtime / 'state.tmp'
        with temp.open('wb') as stream:
            np.savez(stream, time=data.time, qpos=data.qpos,
                     qvel=data.qvel, ctrl=data.ctrl,
                     rgb=self.latest_rgb, depth=self.latest_depth)
        os.replace(temp, self.runtime / 'state.npz')



def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.renderer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
