# Running the simulation

[← Back to the README](../README.md)

Run commands from the repository root. `start` initializes the required submodules and prepares the world assets.

From the repository root, with Docker Engine and Compose available:

```bash
git submodule update --init
./scripts/manage.sh start
./scripts/manage.sh sim
```

`sim` starts with: **native MuJoCo window on,
RoboCasa home, RGB-D mapping and navigation on, browser viewer on, RViz off**.
The native viewer shows the full upstream robot meshes, including the cart and
casters, printed drive base, arm mounts, head support/housing, and wrist cameras.
Visual geometry is enabled and collision proxies are hidden. Closing the native
window shuts down the launch and saves the map.

Open **http://localhost:8082** for the live browser simulation. The viewer mirrors
the ROS simulator, including the upstream camera meshes and green IMU cube; it never
steps another physics instance. Expand its head RGB-D panel to see RGB and depth
previews. The terminal running `sim` owns the launch;
Ctrl+C shuts down the stack and lets RTAB-Map save its database.
Use `./scripts/manage.sh sim-stop` to stop a launch from another terminal while
keeping the development container running. `stop`, `restart`, `start` and
`recreate` also shut down any existing simulation gracefully before changing
the container, so the map database can be saved.

In another terminal:

```bash
./scripts/manage.sh teleop
```

Use `i`/`,` for forward/backward, `j`/`l` for rotation. Hold Shift for holonomic
movement, including `J`/`L` for left/right strafe. Space or `k` stops. Base commands
are clamped to 0.4 m/s and 0.8 rad/s, and stop after 0.5 seconds without a command.

Optional desktop visualization:

```bash
./scripts/manage.sh rviz
```

The native MuJoCo window and RViz use your desktop's X display and authorization.
For a headless run with the browser viewer, use:

```bash
./scripts/manage.sh sim headless:=true
```

`XLEROBOT_HEADLESS=true` in `.env` also changes that default. Camera rendering uses
EGL in both modes. Compose adds graphics devices when
available; Mesa software rendering also works. On NVIDIA, configure the NVIDIA
Container Toolkit and a suitable Compose device reservation for your host.

Robocore Viz starts with the sim at [http://localhost:8889](http://localhost:8889),
with camera, map, drive and arm controls. See the [dashboard profile guide](../src/robocore_viz/README.md).

## Worlds, maps and navigation

The default launch uses **RGB-D visual odometry and RTAB-Map**, with base IMU
orientation supporting odometry and feedback-based wheel motion as its prior.
No laser scanner or SLAM Toolbox is launched.
The default `robocasa_home` is the same three-room world as Mr Handy: an open
test room, kitchen/dining area, and furnished living area, with attributed
RoboCasa/Lightwheel objects. `world:=room` selects the smaller textured diagnostic
room; `world:=empty` selects the original empty plane.

RTAB-Map stores a separate database per world, outside the container: the home
uses `.runtime/robocasa_home.db`, and the diagnostic room uses `.runtime/room.db`.
An explicit `database:=` overrides that selection. Stop the
mapping run before reopening that database in localization mode:

```bash
./scripts/manage.sh sim mode:=localization
```

Start a separate map without deleting the previous database:

```bash
./scripts/manage.sh sim database:=/workspace/xlerobot/.runtime/new-room.db
```

Nav2 starts by default on the depth-built occupancy map. To disable it:

```bash
./scripts/manage.sh sim nav:=false
```

The global map comes from RTAB-Map; obstacle observations come from the RGB-D
point cloud. The local planner and velocity smoother permit lateral motion.
Use RViz's Nav2 goal tool or the `/navigate_to_pose` action after mapping enough
free space. The camera has a forward field of view, so explore the surrounding
area before asking the planner to traverse it.

Launch options include `headless:=true`, `mjviser:=false`, `mjviser_port:=8080`,
`camera_rate:=15.0`, `world:=robocasa_home|room|empty`,
`mode:=mapping|localization|none`, and `rviz:=true`.
The earlier `viewer:=false` spelling remains an alias for `mjviser:=false`.
For physics diagnostics only, use `mode:=none nav:=false odometry:=ground_truth`.
Mapping/localization require estimated RGB-D odometry and reject that diagnostic
option. The simulator always publishes `/ground_truth/odom` for comparisons;
it does not feed that pose to SLAM.
