# Development

The shared runner lives in the pinned `tools/multiros` submodule. Pixi tasks
call it through `scripts/robot.py`. `[tool.multiros]` in `pixi.toml` owns this robot's source
paths, colcon package list, runtime configuration and checks.

```bash
pixi run setup                 # initialize pinned source modules
pixi run images                # rebuild shared and robot images
pixi run up                    # build/start the runtime
pixi run build                 # rebuild the ROS workspace
pixi run build xlerobot_sim    # selected packages
pixi run exec                  # sourced ROS shell
pixi run status
pixi run logs
pixi run down
```

Source is bind-mounted. Build/install/log use project-specific volumes. Host
Pixi only needs Python; `pixi run view` enters the optional MuJoCo environment.
Use `MULTIROS_ROOT` only when deliberately testing another shared-core checkout.

## Checks

```bash
pixi run test standard
pixi run test mod101
pixi run check-rendering --require-accelerated
pixi run check-unit
```

Model checks exercise transforms, contacts, motion/stopping, IMU, camera depth
and controller ABI in an isolated model. Live checks require an active simulation:

```bash
pixi run check-control
pixi run check-runtime
pixi run check-dashboard
pixi run check-tracking   # optional odometry:=rgbd mode only
pixi run check-nav
pixi run check-clearance --sweep
```

These live checks can move the simulated base, head or arms. Run them against
simulation only. See [validation](validation.md) for their scope.

The home assets are checked in with attribution and checksums. `world-assets`
regenerates prepared models inside the runtime; it is not required at startup.

For shared changes, test multiROS first, commit it, then update this repository's
submodule pin. Push multiROS before pushing a robot commit that references it.
Do not commit `.pixi`, runtime snapshots, maps, build outputs or local `.env`.

## Repository layout

- `src/`: owned ROS packages and robot overlays.
- `dependencies/` and `upstream/`: pinned third-party source; keep their history
  and licenses rather than editing or pruning them as local leftovers.
- `scripts/`: host bootstrap and simulation lifecycle.
- `tools/`: asset preparation and the shared multiROS submodule.
- `docker/`: the robot runtime Dockerfile, entrypoint and macOS image selection.
  Generic GPU profiles live in multiROS; the root Compose file owns robot
  mounts, ports and service settings.
- `checks/`: model/runtime acceptance checks, camera regression tests and the
  repeatable camera benchmark.
- `docs/`: setup, architecture, validation and performance evidence.
- `.runtime/`: generated scenes, snapshots and persistent maps; ignored by Git.
- `.pixi/`, `build/`, `install/`, `log/`: generated environments/build outputs;
  ignored by Git. Container build outputs live in named Docker volumes.

The one-off GLX probe, topic-rate shell loop and five overlapping render probes
were removed after the WSL investigation. Their maintained replacements are
`pixi run check-rendering`, `checks/benchmark_camera.py`, the camera regression
tests and the shared viewer benchmark documented in [performance](wsl-performance.md).
Keep future experiments out of the normal runtime tools; promote reusable
checks with explicit inputs and document the evidence they establish.

## Visual meshes

Reduced visual assets and their checksum manifest live in
[`xlerobot_description/visual_meshes`](../src/xlerobot_description/visual_meshes/README.md).
`tools/simplify_visual_meshes.py` regenerates them without modifying upstream.
`checks/check_visual_meshes.py --reference FULL.xml --candidate REDUCED.xml
--snapshot STATE.npz` checks collision geometry, inertia, calibration and identical
physics. Rebuild `xlerobot_description` after regenerating installed assets.

## Measure memory

From Linux/WSL with the stack and native viewer running:

```bash
pixi run measure-memory --output /tmp/xlerobot-memory.json
```

This reports proportional resident memory (PSS) by process and groups physics /
camera rendering, ROS services, native viewing and runtime overhead. Simulation
processes include their ROS bindings. Run after startup/build/tests settle;
compilation is now a separate transient process. Windows GPU allocations and
WSL VM filesystem cache are outside this application total. See
[measurements](wsl-performance.md#memory-use).
