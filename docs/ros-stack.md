# ROS stack

[← Back to the README](../README.md)

## Local packages

| Package | Responsibility |
|---|---|
| `xlerobot_description` | Complete robot URDF, meshes and the Mod101 variant |
| `xlerobot_control` | `ros2_control` hardware adapter for MuJoCo |
| `xlerobot_sim` | Simulation, RGB-D/IMU sensors, world, launch files and SLAM/Nav2 setup |
| `robocore_viz` | Configurable browser dashboard, shared with Mr Handy |

Upstream repositories stay as pinned submodules; the integration lives in `src/`.

## Topics and TF

| Interface | Type / purpose |
|---|---|
| `/cmd_vel` | `geometry_msgs/Twist`, body-frame X/Y and yaw |
| `/joint_states` | 17 (`standard`) or 19 (`mod101`): arms/tools, three wheels, head pan/tilt |
| `/left_arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` |
| `/right_arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` |
| `/head_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` |
| `/camera/color/image_raw` | Rectified simulated `rgb8` |
| `/camera/aligned_depth_to_color/image_raw` | Registered `32FC1` depth in metres, zero for invalid/out-of-range pixels |
| `/camera/depth_filtered/image_raw` | Same depth with robot pixels masked; used for SLAM and obstacle clouds |
| `/camera/color/camera_info` | Shared RGB/depth pinhole calibration |
| `/camera/points` | Registered RGB-D point cloud |
| `/imu/data` | Ideal base IMU orientation, angular velocity and specific force |
| `/odom`, `/odom_info` | RTAB-Map RGB-D odometry and diagnostics |
| `/base_controller/odom` | Feedback-based wheel odometry and visual motion prior |
| `/map` | RTAB-Map depth-built occupancy grid |
| `/ground_truth/odom` | MuJoCo pose diagnostic, never the default TF owner |
| `/clock` | Simulation clock; all ROS consumers use simulation time |

TF ownership is `map -> odom` from RTAB-Map, `odom -> wheel_odom` from
RGB-D odometry, `wheel_odom -> base_footprint` from the omni wheel controller,
and the robot/sensor tree from `robot_state_publisher`.
The IMU supplies orientation, angular velocity and specific force; wheel odometry
uses measured wheel velocities with `open_loop: false`. Wheel odometry is
used as the RGB-D motion prior, with no EKF fusing wheel, IMU and visual
odometry. After ten failed visual updates, odometry resets its reference using
the accumulated wheel guess. Invalid null poses are suppressed. Tracking can
still fail against featureless walls; the wheel prior helps recovery, while
visual mapping pauses until tracking recovers. The cart body uses a white finish in ROS and MuJoCo.

The dashboard derives its map marker from current TF instead of sparse
localization messages. It displays the map update age separately from pose
freshness and reports `/odom_info_lite` tracking loss explicitly. An unchanged
map can have an older timestamp while the robot pose remains live.

Original arm joint names are `Rotation_L`, `Pitch_L`, `Elbow_L`, `Wrist_Pitch_L`,
`Wrist_Roll_L`, `Jaw_L`, with `_R` for the other arm. Head joint names are
`head_pan_joint`, `head_tilt_joint`. Standard ROS 2
`joint_trajectory_controller` instances own these actions. Named partial goals,
feedback, cancellation, and the standard trajectory/tolerance semantics are
available. Default final position tolerance is 0.08 rad. Commands are bounded
at the hardware interface; an unreachable out-of-range goal can be accepted by
the standard controller and then abort on goal tolerance. The dashboard validates
target limits before sending and derives duration from URDF velocity limits.

Example:

```bash
./scripts/manage.sh exec ros2 action send_goal \
  /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [Rotation_L], points: [{positions: [0.2], time_from_start: {sec: 2}}]}}'
```

## Packages and control

`dependencies/lekiwi_ros2` is a second pinned submodule, at
`ba338c74def07b3e76c5095451d59876eee8d57f`. Its STS driver submodule is pinned by
that repository at `2d0eccd931262ab9d4469206ae0b2d05947785a4`. Victor Wang's
`upstream` remains independent and unchanged. Build scripts initialize only the
required STS nested submodule and select these upstream packages:

| Package | How XLeRobot uses it |
|---|---|
| `lekiwi_control` | Loads its omni-controller defaults with XLeRobot overrides; runs its Twist-to-TwistStamped command switch. |
| `lekiwi_mujoco` | Reuses wheel kinematics/limits and the 16-roller contact topology, adapted to the original XLeRobot wheel frames. |
| `lekiwi_description` | Installed dependency/assets for LeKiwi packages; XLeRobot retains its own complete canonical URDF. |
| `sts_hardware_interface` | Builds the upstream physical Feetech driver and its mock support; physical hardware is not started by the simulator. |
| `xlerobot_control` | Owned `hardware_interface::SystemInterface` plugin exposing the simulator's measured joints and bounded commands. |

The live controller manager runs at 50 Hz, with `joint_state_broadcaster`,
`base_controller` (`OmniWheelDriveController`), and standard trajectory controllers
for left arm, right arm, and head. It owns 17 hardware command interfaces and
34 position/velocity state interfaces with original arms, or 19/38 with Mod101. The standard controllers own trajectory actions and joint-state publication.

```bash
./scripts/manage.sh exec ros2 control list_controllers
./scripts/manage.sh exec ros2 control list_hardware_interfaces
```

`/cmd_vel` enters LeKiwi's `twist_switch_node`; its default channel converts to
`/base_controller/cmd_vel` (`TwistStamped`). Both keyboard/dashboard commands and
Nav2's smoothed output retain `/cmd_vel`. The optional switched input is
`/cmd_vel_external`, selected via `/twist_switch`. Wheel odometry is available at
`/base_controller/odom`; RTAB-Map owns `/odom` and the correction to wheel TF.
LeKiwi's LiDAR, SLAM Toolbox, camera payload and autonomy launch files are not
started. Our existing RGB-D mapping and Nav2 configuration stay in use.

One Python process owns MuJoCo physics, clock, ideal IMU and diagnostic odometry.
`SimulatorSystem` exchanges joints through `.runtime/control.bin`, with sequence
checks and a wall-time command watchdog. A separate read-only RGB-D process
renders atomic physics snapshots; it never advances physics. This keeps GPU
rendering out of the controller feedback loop. The browser viewer reads the
same rendered snapshots. MuJoCo is pinned to 3.13.0 to match the selected LeKiwi
simulation package.

The base now moves through wheel/ground contacts, using the physical adaptation
described in the [validation notes](validation.md). The STS driver
is available, but a physical configuration still needs actual bus ports, motor
IDs, direction/zero calibration and limits. No physical servo bus is opened by
`manage.sh sim`.
