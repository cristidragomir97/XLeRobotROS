# XLeRobot ROS 2

ROS 2 Jazzy control, MuJoCo simulation and RGB-D mapping for
[Victor Wang's XLeRobot](https://github.com/Vector-Wangel/XLeRobot), built on
[Aditya Kamath's LeKiwi ROS 2](https://github.com/adityakamath/lekiwi_ros2).

![XLeRobot in MuJoCo](docs/media/full-robot.png)

The Linux container runs physics, controllers, camera rendering, RTAB-Map,
Nav2 and the dashboard. The native MuJoCo viewer displays the same state.
Closing its window leaves simulation and cameras running.

## Run

Install [Pixi](https://pixi.sh), Git, and a container runtime: Docker with Compose
on macOS/Linux, or Microsoft's `wslc.exe` on Windows. macOS acceleration uses
lighter. Windows is ready for its first runtime test; see the procedure below.

```bash
git clone --recurse-submodules https://github.com/cristidragomir97/XLeRobotROS.git
cd XLeRobotROS
pixi install --locked
pixi run doctor
pixi run up
pixi run sim
```

In another terminal:

```bash
pixi run view
```

The dashboard is at http://localhost:8889. `up` initializes the pinned source
modules and builds the shared rendering image and robot image. The first build
can take a while. The normal host environment contains no ROS packages.

Stop the simulation with Ctrl+C or `pixi run sim-stop`. Stop the container with
`pixi run down`; it saves the map and preserves volumes. Use `pixi run build`
after editing ROS source, or `pixi run images` after changing image dependencies.

The shared runner is pinned at `tools/multiros`. Host tasks and runtime setup
are described in `multiros.json`.

## Arm configurations

```bash
pixi run sim standard
pixi run sim-stop
pixi run sim mod101
```

The Mod101 configuration has two 7DOF arms with PG grippers and the same base,
head, cameras and home world. Both configurations use the same controllers and
topic contract. See [Mod101 details](docs/mod101.md).

## Windows test

Use PowerShell on Windows with WSL 2.9.3 or newer:

```powershell
wsl --update
wslc version
wslc run --rm hello-world
```

After cloning as above:

```powershell
pixi install --locked
pixi run doctor
pixi run up
pixi run check-rendering --require-accelerated
pixi run test
pixi run sim
# In another PowerShell window:
pixi run view
```

Windows automatically selects WSLC. Start with the rendering check: GPU device
access alone does not prove EGL works. [Windows troubleshooting](docs/windows.md)
explains logs, software fallback and container recreation. These commands have
been planned and tested on macOS; the actual WSLC/GPU run is still unvalidated.

## Validation

Apple Silicon/lighter has passed textured RGB-D rendering, model/control checks,
dashboard checks and a short navigation goal. Long-run mapping/navigation,
Linux GPU backends and Windows runtime behavior need further testing. See
[validation](docs/validation.md) for evidence and limits.

## Documentation

- [Running, maps and launch options](docs/running.md)
- [Builds and checks](docs/development.md)
- [Container graphics](docs/container-rendering.md)
- [Windows](docs/windows.md)
- [ROS packages, controllers and topics](docs/ros-stack.md)
- [Mod101 arms](docs/mod101.md)
- [Robocore Viz profiles](src/robocore_viz/README.md)

## Credits

Victor Wang and the XLeRobot contributors supplied the robot design and assets.
Aditya Kamath supplied the LeKiwi ROS packages and STS hardware integration.
Their repositories remain pinned submodules with their history and licenses.
See [credits](docs/credits.md), [NOTICE](NOTICE) and [Apache-2.0](LICENSE).
