# Container rendering

Physics, RGB-D rendering and ROS publication run in the Linux container. The
native frontend only renders its interactive window. Closing it does not stop
camera images or mapping.

The camera process follows `physics.npz`, calls `mj_forward`, renders RGB/depth/
segmentation and publishes the arrays directly to ROS. It does not step physics.
`state.npz` remains available for checks/previews; `viewer.npz` feeds the native
frontend. These atomic files are still prototype transport on the bind mount.

## Backends

macOS/lighter uses the shared patched Mesa Zink 25.1.9 image, system Venus and
MoltenVK/Metal. It requests `lighter.sh/gpu=all`. The shared core carries the
texture shader-interface and Metal resource-name fixes with attribution.

The lighter camera profile defaults to `fast`: no MSAA, shadows or reflections.
Set `XLEROBOT_RENDER_QUALITY=standard` for those effects. Dimensions, calibration,
metric depth and segmentation are unchanged.

Linux uses the runtime EGL libraries. Select `MULTIROS_BACKEND=dri` for DRM
nodes; NVIDIA needs an appropriate local Compose override and container toolkit.
Those GPU paths still need integration testing.

Windows uses WSLC with GPU access and an initial EGL/D3D12 configuration.
It has not been tested on Windows yet; see [the test procedure](windows.md).

## Validate

```bash
pixi run check-rendering --require-accelerated
pixi run test standard
pixi run test mod101
```

The first command reports the actual GL renderer and checks textured RGB,
a known metric depth and segmentation. Full-model checks cover the robot scene.
A listed GPU or successful CUDA check is not sufficient.

For explicit CPU fallback, select `MULTIROS_BACKEND=cpu` before creating the
container. It uses OSMesa/llvmpipe. Record the result as software rendering.
Existing WSLC containers must be recreated to apply changed image/environment
settings; named build/install/log volumes can be retained.
