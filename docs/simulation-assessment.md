# Existing simulation and ROS integration assessment

Inspected 2026-10-13. Upstream: https://github.com/Vector-Wangel/XLeRobot,
commit `b017b5e6354bd9f61f4247a920c72622ca0aade0`. Upstream working tree is clean.

This is the initial exploration record. The subsequent implementation uses
RGB-D RTAB-Map and a bottom-base IMU per the user's revised requirements, rather
than the laser/SLAM Toolbox proposal below. The wrapper now has its own Git
repository and upstream is registered as a submodule. See the root README for
the implemented architecture and current checks.

## Existing MuJoCo model

Entry point: `upstream/simulation/mujoco/xlerobot_mujoco.py` loads `scene.xml`
relative to the working directory. The scene includes `xlerobot.xml` and a flat
ground plane. Mesh assets are already present. A separate ManiSkill simulation
also exists, but is not needed for the initial MuJoCo ROS stack.

The compiled model has 20 scalar joints, 18 actuators, 34 bodies, no cameras,
and no sensors. It includes two six-joint arms (including jaws), three wheels,
two unactuated head joints, and three planar chassis joints.

The chassis is constrained to X/Y translation and yaw. Velocity actuators drive
these joints directly; wheel-floor forces do not provide the locomotion mechanism.
Wheel meshes use small isotropic friction, with no anisotropic contact pairs.
This is a simplified holonomic platform, not the Ekumen omniwheel physics model.

The keyboard controller relies on array offsets. Its comments label actuator
indices 3–8 as the left arm, but the model assigns them to the right arm.
Joint positions and actuators have different orders (wheel positions precede
arm positions). A ROS adapter must resolve joints and actuators by name.
The chassis has a fixed approximately 90-degree rotation: coordinate conversion
must use actual body transforms and joint axes, not assume qpos[:3] directly
represents a ROS base pose. The two head joints have inertias but no visual
geometry or actuators.

## Executed checks

Loaded the unchanged upstream scene with locally installed MuJoCo 3.5.0
(upstream requirements pin 3.3.0). Stepped four independent two-second runs:
rest, X velocity, Y velocity, and yaw velocity. All states remained finite;
all runs ended with zero active contacts. Direct 0.3 velocity targets reached
about 0.041 m/s on the translational joints after two seconds and 0.296 rad/s
on yaw. These are actuator targets, not verified ROS-frame velocities.
The upstream feedback controller adds gain to translational targets.
No GUI or ROS runtime has been tested in this exploration.

## Neighboring stack conventions

`../mr_handy` provides the closest compact starting point: ROS 2 Jazzy on
Ubuntu Noble, Docker Compose, isolated project build/install/log volumes,
`scripts/manage.sh`, optional graphics overlays, and a sourced entrypoint.
Its ROS control and MuJoCo packages are specific to that assembly and should
not be copied wholesale. `../grove-g1` has a larger development stack with
the same operator entry point convention.

Grove's active 2D navigation path uses SLAM Toolbox online asynchronous mapping,
AMCL localization, and Nav2. Its LiDAR point cloud is converted to `/scan`.
FAST-LIO sources also exist, but are not the SLAM backend described by
`g1_navigation`. Grove uses wall time because its simulation track does not
publish `/clock`; an XLeRobot bridge should publish `/clock` and consistently
enable simulation time instead. Reuse SLAM Toolbox, not G1 gait constraints,
sensor height bands, footprint, or odometry assumptions.

## Proposed wrapper structure

Keep upstream unchanged and pinned as `upstream/`. Own Docker files,
`scripts/manage.sh`, ROS packages, generated scene overlays, configs, and
checks in the wrapper repository. Suggested ROS packages:

- `xlerobot_description`: consistent ROS frames and upstream mesh references.
- `xlerobot_mujoco`: simulator ownership, named actuator mapping, `/clock`,
  joint states, simulated odometry, and sensor publication.
- `xlerobot_bringup`: launch orchestration, teleoperation, command timeout,
  and arm/head control interfaces.
- `xlerobot_navigation`: SLAM Toolbox, AMCL, and holonomic Nav2 configuration.

First establish ROS base commands, state/TF, and arm control on the existing
planar model. Add a bounded mapping world and a clearly identified simulated
laser scanner; the upstream empty plane has nothing useful to map. Simulation
ground-truth odometry must be labeled as such. Add controller/action interfaces
and validate lateral motion, stopping, limits, and TF before navigation.

For wheel-driven physics, generate an overlay using a free chassis, capsule
wheel collision geometry, anisotropic floor contact pairs, and wheel velocity
actuators. Validate wheel signs, frame alignment, traction, and chassis stability
independently before replacing the planar backend. Ekumen explains why capsule
contact frames are useful here:
https://ekumenlabs.com/blog/posts/modeling-lekiwis-omni-base-in-mujoco/

## Git layout issue

The wrapper directory currently has an empty, read-only `.git` directory.
Git therefore resolves the repository root to `/home/cdr/Work`, whose origin is
an unrelated `ros2_feetech_manager` repository. `upstream/` is a clean standalone
Git checkout, not a registered submodule. Establishing a real wrapper repository
requires resolving the protected `.git` directory first; submodule registration
must not alter the unrelated parent repository.
