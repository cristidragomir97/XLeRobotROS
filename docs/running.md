# Running

Run commands from the repository root. `pixi run up` initializes pinned
submodules, builds the images and starts the runtime. `pixi run sim` launches
the standard robot in the furnished home with RTAB-Map and Nav2 enabled.

```bash
pixi run sim
# Separate terminal:
pixi run view
```

On WSL, the viewer uses a lightweight MuJoCo/GLFW window at up to 15 FPS.
Left drag orbits, right drag pans, the middle button or scroll wheel zooms, and
Escape closes it. `pixi run view --rate 20` changes the GPU budget. The full UI
is available with `pixi run view --frontend passive --full-forward`; its own
render thread is not frame-capped by `--rate`.

The viewer mirrors physics state and renders its own window. Cameras render
inside the container. Close the viewer freely; stop the simulation using Ctrl+C
or `pixi run sim-stop`. `pixi run down` and `restart` first stop the simulator
so RTAB-Map can save its database.

The dashboard runs at http://localhost:8889. Its RGB panel consumes the standard
JPEG topic `/camera/color/image_raw/compressed`; the bridge passes the encoded
bytes through. Depth previews are JPEG over WebSocket. Mapping still receives
raw RGB and metric float depth inside the container. Use `pixi run view` for the native
MuJoCo window.

## Driving

```bash
pixi run teleop
```

Use `i`/`,` for forward/backward and `j`/`l` for rotation. Shift+`J`/`L` strafes.
Space or `k` stops. Base commands are clamped to 0.4 m/s and 0.8 rad/s and expire
after 0.5 seconds. The simulation uses one controller/physics owner.

## Worlds and maps

`world:=robocasa_home` is the default furnished home. `world:=room` selects a
smaller textured diagnostic room; `world:=empty` selects the empty plane.

RTAB-Map stores databases in `.runtime`, separately by world, arm configuration
and odometry mode. Wheel sessions use the `_wheel.db` suffix, preserving existing
visual-odometry maps. Stop mapping
before reopening its database in localization mode:

```bash
pixi run sim-stop
pixi run sim mode:=localization
```

Use `database:=/workspace/xlerobot/.runtime/new-room.db` for a new database.
`nav:=false` disables Nav2. The camera has a forward field of view; explore
surrounding free space before asking the planner to traverse it.

Mapping uses measured wheel velocities for `/odom` and `odom -> base_footprint`.
RTAB-Map uses synchronized RGB-D images and interpolates wheel pose from TF;
it owns `map -> odom`. No visual odometry node runs by default. The IMU remains
available on `/imu/data`, but is not fused into wheel odometry by an EKF.
Ground-truth odometry is diagnostic only and is not fed into mapping.

`odometry:=rgbd` explicitly restores visual odometry for comparisons. For physics
diagnostics use `mode:=none nav:=false odometry:=ground_truth`. RTAB-Map rejects
ground-truth odometry in mapping/localization modes.

Useful launch arguments include `camera_rate:=15.0`, `depth_cloud:=false`,
`world:=room`, `mode:=mapping|localization|none`.
Pass them after the configuration, e.g. `pixi run sim mod101 world:=room`.

## Configuration

Copy `.env.example` to `.env` to change ports, ROS domain, default configuration
or camera quality. The shared runner reads it as data. Explicit environment
variables override that file.

WSL selects the Docker Desktop GPU override automatically. Other hosts follow
the selected runtime/context. Linux DRM users can
set `MULTIROS_BACKEND=dri`; CPU fallback uses `cpu`. See
[graphics](container-rendering.md) and [Windows](windows.md).

The host has no ROS installation. Native RViz is not part of this recipe;
use the dashboard or a separately configured ROS visualization workstation.
