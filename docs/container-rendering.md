# Container rendering

Physics, RGB-D rendering and ROS publication run in the Linux container.
A short-lived `xlerobot_sim.prepare_model` process generates MJCF and compiles
`.runtime/scene.mjb` once per launch, then exits. Physics and camera processes
load that same binary; the WSL native viewer does too. This avoids retaining
XML compiler/spec/asset-cache memory in each long-lived process. The original
XML and generated model metadata stay available for inspection. The
native frontend only renders its interactive window. Closing it does not stop
camera images or mapping.

The camera process follows `physics.npz`, calls `mj_forward`, renders RGB/depth/
segmentation and publishes the arrays directly to ROS. It does not step physics.
Rendered arrays go directly to ROS; no compatibility image archive is written.
`viewer.npz` feeds the native frontend. Atomic physics/viewer files remain
prototype transport on the bind mount.

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

WSL/Docker Desktop uses the shared multiROS WSL profile: `/dev/dxg`, the WSL driver
libraries and Mesa D3D12. The host bootstrap also selects D3D12 for the passive
viewer. Explicit `MULTIROS_BACKEND` and `GALLIUM_DRIVER` settings take precedence.
This path has passed live checks on Intel UHD 770; see [performance](wsl-performance.md).

Native Windows WSLC remains unvalidated; see [the test procedure](windows.md).

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

The default WSL native window uses the shared capped GLFW frontend. It consumes
small state snapshots and renders its own viewpoint; it does not receive camera
pixels. The dashboard instead consumes compressed RGB and existing JPEG depth
previews. Raw metric inputs remain inside the ROS mapping pipeline. Image
publishers retain one frame and skip message construction for unused planes.
