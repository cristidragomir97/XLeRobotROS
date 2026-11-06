"""Check combined readback against MuJoCo's independent RGB/depth/ID passes."""
import unittest

import mujoco
import numpy as np

from xlerobot_sim.camera_rendering import CameraRenderer


class CameraRenderingTests(unittest.TestCase):
    def test_reference_pixels_depth_and_self_filter_at_multiple_poses(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco>
          <visual><global offwidth="160" offheight="120"/><quality offsamples="0"/></visual>
          <worldbody>
            <light pos="1 1 4" dir="-1 -1 -4"/>
            <camera name="rgbd" pos="0 0 3"/>
            <geom type="plane" size="3 3 .1" rgba=".2 .3 .4 1"/>
            <body pos=".2 .1 .3"><freejoint/>
              <geom type="box" size=".2 .1 .3" rgba=".8 .2 .1 1"/>
            </body>
          </worldbody></mujoco>''')
        data = mujoco.MjData(model)
        option = mujoco.MjvOption()
        mask = np.array([False, True])
        combined = CameraRenderer(model, width=160, height=120)
        self.addCleanup(combined.close)
        with mujoco.Renderer(model, width=160, height=120) as reference:
            for fast, x in [(True, .2), (False, -.35)]:
                with self.subTest(fast=fast, x=x):
                    data.qpos[0] = x
                    mujoco.mj_forward(model, data)
                    reference.update_scene(data, camera='rgbd', scene_option=option)
                    for flag in (mujoco.mjtRndFlag.mjRND_SHADOW, mujoco.mjtRndFlag.mjRND_REFLECTION):
                        reference.scene.flags[flag] = not fast
                    reference.disable_segmentation_rendering()
                    rgb = reference.render().copy()
                    reference.enable_depth_rendering()
                    depth = reference.render().copy()
                    depth[(depth < .15) | (depth > 8.) | ~np.isfinite(depth)] = 0.
                    reference.disable_depth_rendering()
                    reference.enable_segmentation_rendering()
                    segments = reference.render()
                    ids = segments[:, :, 0]
                    robot_pixels = ((segments[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
                                    & (ids >= 0) & mask[np.clip(ids, 0, model.ngeom - 1)])
                    self.assertGreater(np.count_nonzero(robot_pixels), 0)
                    filtered = depth.copy()
                    filtered[robot_pixels] = 0.
                    actual = combined.render(data, camera='rgbd', option=option,
                                             robot_geom_mask=mask, fast=fast)
                    np.testing.assert_array_equal(actual[0], rgb)
                    np.testing.assert_allclose(actual[1], depth, rtol=1e-6, atol=1e-6)
                    np.testing.assert_allclose(actual[2], filtered, rtol=1e-6, atol=1e-6)
                    self.assertAlmostEqual(float(actual[1][60, 80]), 3., places=4)
        combined.close()  # Explicit close is idempotent.


if __name__ == '__main__':
    unittest.main()
