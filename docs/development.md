# Development

The shared runner lives in the pinned `tools/multiros` submodule. Pixi tasks
call it through `scripts/robot.py`. `multiros.json` owns this robot's source
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
pixi run check-tracking
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
