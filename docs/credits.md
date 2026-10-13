# Credits and licenses

[← Back to the README](../README.md)

## Victor Wang — XLeRobot

[Victor Wang / Vector-Wangel](https://github.com/Vector-Wangel) and the
[XLeRobot contributors](https://github.com/Vector-Wangel/XLeRobot) are credited
for the original robot, its mechanical design, meshes, kinematic descriptions
and simulation. The cart, arm mounts, head assembly and wrist-camera visuals
in this stack come from their upstream assets.

The original repository is retained in [`upstream/`](../upstream), pinned at
`b017b5e6354bd9f61f4247a920c72622ca0aade0`, with its
[Apache-2.0 license](../upstream/LICENSE). For research using XLeRobot, please use
[the authors' upstream citation](../upstream/README.md#citation), preserving its
full author list.

## Aditya Kamath — LeKiwi ROS 2 and STS Hardware Interface

[Aditya Kamath / Kamath Robotics](https://github.com/adityakamath) is credited
for [LeKiwi ROS 2](https://github.com/adityakamath/lekiwi_ros2) and
[STS Hardware Interface](https://github.com/adityakamath/sts_hardware_interface).
His packages supply the omni-controller defaults, Twist command switch,
MuJoCo wheel kinematics, passive omniwheel contact topology and Feetech STS
driver used or built here. The
[XLeRobot adaptation](ros-stack.md#packages-and-control) documents each package's
role; the simulator uses a local hardware adapter and the physical STS driver
is available for later hardware configuration.

LeKiwi ROS 2 is retained in [`dependencies/lekiwi_ros2/`](../dependencies/lekiwi_ros2),
pinned at `ba338c74def07b3e76c5095451d59876eee8d57f`, with its
[Apache-2.0 license](../dependencies/lekiwi_ros2/LICENSE). Its nested STS driver
is pinned at `2d0eccd931262ab9d4469206ae0b2d05947785a4`, with its own
[Apache-2.0 license](../dependencies/lekiwi_ros2/modules/sts_hardware_interface/LICENSE).
Upstream copyright notices and repository history are retained.

## Other projects

- The Mod101 PG gripper adapts NormaCore's Parallel Jaw Gripper; its original license and attribution remain in the gripper package.
- [Ekumen's LeKiwi omni-base article](https://ekumenlabs.com/blog/posts/modeling-lekiwis-omni-base-in-mujoco/), by Franco Cipollone, informed the contact model.
- [RTAB-Map](https://github.com/introlab/rtabmap_ros) and [Nav2](https://github.com/ros-navigation/navigation2) supply RGB-D SLAM and navigation.
- Robocore Viz comes from Mr Handy, originally based on Dheera Venkatraman's ROSboard. See its [license](../src/robocore_viz/LICENSE) and [provenance](../src/robocore_viz/PROVENANCE.md).
- The home contains NVIDIA / Lightwheel assets under CC-BY-4.0. See [asset attribution](../assets/robocasa/README.md).

This integration is [Apache-2.0](../LICENSE). Upstream files retain their own licenses and copyright notices; [NOTICE](../NOTICE) records attribution and local adaptations.

