"""RGB plus metric depth/segmentation in two draws, using MuJoCo's public GL API."""
import mujoco
from mujoco.gl_context import GLContext
import numpy as np


class CameraRenderer:
    def __init__(self, model, *, width, height):
        if min(width, height) <= 0:
            raise ValueError('Camera dimensions must be positive')
        if width > model.vis.global_.offwidth or height > model.vis.global_.offheight:
            raise ValueError('Camera dimensions exceed the model offscreen framebuffer')
        self.model = model
        self.scene = mujoco.MjvScene(model, maxgeom=10000)
        self.viewport = mujoco.MjrRect(0, 0, width, height)
        self.gl = GLContext(width, height)
        self.context = None
        try:
            self.gl.make_current()
            self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)
            self.context.readDepthMap = mujoco.mjtDepthMap.mjDEPTH_ZEROFAR
        except Exception:
            self.close()
            raise
        self.color = np.empty((height, width, 3), dtype=np.uint8)
        self.ids = np.empty_like(self.color)
        self.depth = np.empty((height, width), dtype=np.float32)

    def render(self, data, *, camera, option, robot_geom_mask, fast=True):
        if self.context is None:
            raise RuntimeError('Camera renderer is closed')
        self.gl.make_current()
        view = mujoco.MjvCamera()
        view.type = mujoco.mjtCamera.mjCAMERA_FIXED
        view.fixedcamid = self.model.camera(camera).id
        mujoco.mjv_updateScene(self.model, data, option, None, view,
                              mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        for flag in (mujoco.mjtRndFlag.mjRND_SHADOW, mujoco.mjtRndFlag.mjRND_REFLECTION):
            self.scene.flags[flag] = not fast
        mujoco.mjr_render(self.viewport, self.scene, self.context)
        mujoco.mjr_readPixels(self.color, None, self.viewport, self.context)

        # MuJoCo's reference depth path uses the same ID-color pass as segmentation.
        # Read both buffers from that pass instead of submitting the scene twice.
        original_flags = self.scene.flags.copy()
        try:
            self.scene.flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = 1
            self.scene.flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = 1
            mujoco.mjr_render(self.viewport, self.scene, self.context)
            mujoco.mjr_readPixels(self.ids, self.depth, self.viewport, self.context)
        finally:
            self.scene.flags[:] = original_flags

        # Invert the float32 projection coefficients used by MuJoCo's reverse-Z
        # framebuffer. Float64 arithmetic preserves metric accuracy at long range.
        near = np.float32(self.model.vis.map.znear * self.model.stat.extent)
        far = np.float32(self.model.vis.map.zfar * self.model.stat.extent)
        c = np.float32(-.5) * (-(far + near) / (far - near)) - np.float32(.5)
        d = np.float32(-.5) * (-(np.float32(2) * far * near) / (far - near))
        depth = (d / (self.depth[::-1].astype(np.float64) + c)).astype('<f4')
        depth[(depth < .15) | (depth > 8.) | ~np.isfinite(depth)] = 0.
        pixels = self.ids[::-1].astype(np.uint32)
        ids = pixels[:, :, 0] + (pixels[:, :, 1] << 8) + (pixels[:, :, 2] << 16)
        # Only the robot/background classification is needed; avoid allocating
        # a full (height, width, 2) object-ID/type segmentation image.
        robot_ids = np.zeros(self.scene.ngeom + 1, dtype=bool)
        for geom in self.scene.geoms[:self.scene.ngeom]:
            if (geom.segid >= 0 and geom.objtype == mujoco.mjtObj.mjOBJ_GEOM
                    and 0 <= geom.objid < len(robot_geom_mask)):
                robot_ids[geom.segid + 1] = robot_geom_mask[geom.objid]
        filtered = depth.copy()
        filtered[robot_ids[ids]] = 0.
        return self.color[::-1].copy(), depth, filtered

    def close(self):
        if self.gl is not None:
            self.gl.make_current()
            if self.context is not None:
                self.context.free()
                self.context = None
            self.gl.free()
            self.gl = None
