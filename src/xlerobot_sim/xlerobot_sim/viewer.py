"""Browser mirror of the single ROS simulator; never advances the physics."""
import argparse
from pathlib import Path
import signal
import time

import mujoco
import numpy as np
import viser
from mjviser import ViserMujocoScene
from rclpy.utilities import remove_ros_args


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime', default='/workspace/xlerobot/.runtime')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args(remove_ros_args()[1:])
    runtime = Path(args.runtime)
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    deadline = time.monotonic() + 30.
    while running and not (runtime / 'physics.npz').exists():
        if time.monotonic() > deadline:
            raise RuntimeError('Physics owner did not publish a fresh startup snapshot')
        time.sleep(.1)
    if not running:
        return
    model = mujoco.MjModel.from_xml_path(str(runtime / 'scene.xml'))
    data = mujoco.MjData(model)
    server = viser.ViserServer(host='0.0.0.0', port=args.port)
    scene = ViserMujocoScene(server, model, num_envs=1)
    scene.create_visualization_gui(camera_distance=5.)
    status = server.gui.add_markdown('Waiting for ROS simulation…')
    clock = server.gui.add_number('Simulation time', initial_value=0., disabled=True)
    server.gui.add_markdown('RGB-D camera on head; green IMU cube under base. Drive through ROS.')
    with server.gui.add_folder('Head RGB-D camera'):
        rgb_panel = server.gui.add_image(np.zeros((160, 214, 3), dtype=np.uint8), label='RGB')
        depth_panel = server.gui.add_image(np.zeros((160, 214, 3), dtype=np.uint8), label='Depth (0.15–8 m)')
        server.gui.add_markdown('Depth preview: brighter = farther; black = invalid. ROS depth is in metres.')
    last_update = -1
    last_camera = 0.
    try:
        while running:
            try:
                modified = (runtime / 'state.npz').stat().st_mtime_ns
                if modified != last_update:
                    with np.load(runtime / 'state.npz') as state:
                        data.time = float(state['time'])
                        for name in ('qpos', 'qvel', 'ctrl'):
                            getattr(data, name)[:] = state[name]
                        if time.monotonic()-last_camera > .2:
                            rgb_panel.image = state['rgb'][::3, ::3]
                            gray = np.clip(state['depth'][::3, ::3]*255/8, 0, 255).astype(np.uint8)
                            depth_panel.image = np.repeat(gray[:, :, None], 3, axis=2)
                            last_camera = time.monotonic()
                    mujoco.mj_forward(model, data)
                    scene.update_from_mjdata(data)
                    clock.value = data.time
                    last_update = modified
                age = time.time()-(runtime / 'state.npz').stat().st_mtime
                status.content = '**Live**' if age < 1 else '**Simulation stopped**'
            except FileNotFoundError:
                pass
            time.sleep(.05)
    finally:
        server.stop()
