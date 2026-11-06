"""Container RGB-D rendering from read-only physics snapshots; never steps physics."""
import argparse
import math
import os
import time
from pathlib import Path

import mujoco
import numpy as np

from .model import Robot
from .camera_rendering import CameraRenderer


def wait_for(path, name, timeout=30.):
    deadline = time.monotonic() + timeout
    while not path.is_file():
        if time.monotonic() > deadline:
            raise RuntimeError(f'Physics owner did not publish {name}')
        time.sleep(.05)


class Renderer:
    def __init__(self, runtime, rate=15., width=640, height=480):
        self.runtime = Path(runtime)
        wait_for(self.runtime / 'scene.mjb', 'runtime/scene.mjb')
        wait_for(self.runtime / 'physics.npz', 'physics.npz')
        self.robot = Robot(self.runtime / 'scene.mjb')
        self.fast = os.environ.get('XLEROBOT_RENDER_QUALITY', 'fast') == 'fast'
        if self.fast:
            self.robot.model.vis.quality.offsamples = 0
            self.robot.model.vis.quality.shadowsize = 1
        self.width, self.height = width, height
        if rate <= 0 or width <= 0 or height <= 0:
            raise ValueError('Camera rate and dimensions must be positive')
        self.period = 1. / rate
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0
        self.renderer = CameraRenderer(self.robot.model, height=height, width=width)
        self.last_time = -1.
        self.frame = None
        self.f_info = self.height / (2 * math.tan(math.radians(60) / 2))
        self.cx, self.cy = (self.width - 1) / 2, (self.height - 1) / 2

    def render(self, data):
        return self.renderer.render(data, camera='rgbd', option=self.option,
                                    robot_geom_mask=self.robot.robot_geom_mask, fast=self.fast)

    def step(self):
        with np.load(self.runtime / 'physics.npz') as state:
            timestamp = float(state['time'])
            if timestamp <= self.last_time:
                return False
            data = self.robot.data
            data.qpos[:] = state['qpos']
            data.qvel[:] = state['qvel']
            data.ctrl[:] = state['ctrl']
            data.time = timestamp
        mujoco.mj_forward(self.robot.model, data)
        rgb, depth, filtered = self.render(data)
        self.last_time = timestamp
        self.frame = timestamp, rgb, depth, filtered
        return True

    def run(self):
        try:
            while True:
                started = time.monotonic()
                if not self.step():
                    time.sleep(.01)
                    continue
                time.sleep(max(0., self.period - (time.monotonic() - started)))
        finally:
            self.renderer.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime', default='/workspace/xlerobot/.runtime')
    parser.add_argument('--rate', type=float, default=15.)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    args = parser.parse_args()
    Renderer(args.runtime, args.rate, args.width, args.height).run()


if __name__ == '__main__':
    main()
