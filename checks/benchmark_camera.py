"""Compare reference three-pass and combined two-pass rendering on one frozen state.

Run inside the robot container with other GPU workloads paused for useful timings.
"""
import argparse
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from xlerobot_sim.renderer import Renderer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, default=Path('/workspace/xlerobot/.runtime'))
    parser.add_argument('--frames', type=int, default=12)
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error('frames must be positive')
    renderer = Renderer(args.runtime)
    model, data = renderer.robot.model, renderer.robot.data
    with np.load(args.runtime / 'physics.npz', allow_pickle=False) as state:
        for name in ('qpos', 'qvel', 'ctrl'):
            getattr(data, name)[:] = state[name]
        data.time = float(state['time'])
    mujoco.mj_forward(model, data)
    reference = mujoco.Renderer(model, height=480, width=640)
    try:
        def three_pass():
            reference.update_scene(data, camera='rgbd', scene_option=renderer.option)
            for flag in (mujoco.mjtRndFlag.mjRND_SHADOW, mujoco.mjtRndFlag.mjRND_REFLECTION):
                reference.scene.flags[flag] = not renderer.fast
            reference.disable_segmentation_rendering()
            rgb = reference.render().copy()
            reference.enable_depth_rendering()
            depth = reference.render().copy()
            depth[(depth < .15) | (depth > 8.) | ~np.isfinite(depth)] = 0.
            reference.disable_depth_rendering()
            reference.enable_segmentation_rendering()
            segments = reference.render()
            reference.disable_segmentation_rendering()
            ids = segments[:, :, 0]
            robot = ((segments[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & (ids >= 0)
                     & renderer.robot.robot_geom_mask[np.clip(ids, 0, model.ngeom - 1)])
            filtered = depth.copy()
            filtered[robot] = 0.
            return rgb, depth, filtered

        for actual, expected in zip(renderer.render(data), three_pass()):
            np.testing.assert_allclose(actual, expected, rtol=1e-6, atol=1e-6)
        timings = {'three_pass': [], 'two_pass': []}
        for index in range(args.frames):
            methods = [('three_pass', three_pass), ('two_pass', lambda: renderer.render(data))]
            # Alternate ordering to avoid consistently favoring the second path.
            for label, function in methods[::1 if index % 2 else -1]:
                started = time.perf_counter()
                function()
                timings[label].append((time.perf_counter() - started) * 1000)
        results = {label: {'median_ms': round(float(np.median(values)), 3),
                           'p95_ms': round(float(np.percentile(values, 95)), 3)}
                   for label, values in timings.items()}
        print(json.dumps(dict(reference_equivalence='passed', frames=args.frames,
                              mujoco=mujoco.__version__, effects=not renderer.fast,
                              viewport=[640, 480], **results), indent=2), flush=True)
    finally:
        reference.close()
        renderer.renderer.close()


if __name__ == '__main__':
    main()
