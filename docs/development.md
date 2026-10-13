# Development and checks

[← Back to the README](../README.md)

```bash
./scripts/manage.sh build
./scripts/manage.sh world-assets # prepare/reuse the shared home assets
./scripts/manage.sh test standard
./scripts/manage.sh test mod101
./scripts/manage.sh test --sim  # active mapping run; briefly moves the simulated base and one arm
./scripts/manage.sh test --nav  # active nav:=true run; sends one short navigation goal
./scripts/manage.sh test --dashboard # active run; checks sensors, head controls and strafe
./scripts/manage.sh test --tracking # active run; points the head up, checks loss/recovery, restores it
./scripts/manage.sh exec       # sourced ROS shell
./scripts/manage.sh stop
```

The headless checks verify full URDF/MJCF transforms at nonzero poses, optical
axes, forward/lateral/yaw motion, stopping, IMU gravity, and metric rendered
depth. Live checks cover synchronized camera topics, sensor TF, RTAB-Map odometry
and map, standard controllers, arm actions, and the base command watchdog.
Recorded results and current simulation limits are in [validation notes](validation.md).

Source is bind-mounted; build/install/log use project-specific Docker volumes.
After code changes, run `build` and restart `sim`. After Docker dependency
changes, run `recreate`. `.env.example` defines the ROS domain (185) and browser
port (8082), separate from the neighboring robot projects. ROS commands are
run inside this container; host ROS tools need matching discovery/network setup.
