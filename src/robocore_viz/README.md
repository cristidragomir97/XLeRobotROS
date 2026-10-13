# Robocore Viz

Reusable ROS 2 browser dashboard and websocket bridge, vendored from Mr Handy.
The package contains no XLeRobot imports or dependencies. Copy this directory
into another colcon workspace to reuse it; retain LICENSE and PROVENANCE.md.

Run with a robot profile:

```bash
ros2 run robocore_viz robocore_viz_node --ros-args \
  -p port:=8888 \
  -p config_file:=/absolute/path/robot-dashboard.yaml \
  -p robot_description_file:=/absolute/path/robot.urdf
```

Expose the HTTP port from Docker. The same port serves HTML, configuration,
graph/parameter APIs and the websocket at `/robocore_viz/v1`.
The dashboard includes camera/depth and map previews, drive controls, joint
sliders, telemetry, topic inspection, ROS graph, parameters, TF, diagnostics,
logs and system statistics. It discovers additional image and map topics.
Metric `32FC1` depth is rendered as a grayscale preview; the displayed range is
in metres and the original ROS image is unchanged.

Profiles are YAML. See `examples/mr_handy.yaml` and the consumer's
`xlerobot_sim/config/dashboard.yaml`. Set `name`, `id` (isolates saved browser
preferences), `tagline`, camera/map topic-label lists, `topics` bindings,
`drive` and `telemetry`. Drive supports Twist and TwistStamped, maximum linear
and angular speeds, and `omnidirectional: true` for the strafe toggle.
Telemetry kinds are localization, navigation, plan, odometry, imu, lidar,
joints and compute; sections are navigation or hardware.

`joint_groups` declare title, topic, type and joints. Types are
`std_msgs/msg/Float64MultiArray`, `trajectory_msgs/msg/JointTrajectory` and
`control_msgs/action/FollowJointTrajectory`. For actions, `topic` is the action
server name, without a `/_action` suffix. A list of joint names derives limits,
units and speed from `robot_description_file`. Alternatively supply explicit
`{name, min, max, velocity, unit}` objects. Optional `label` sets the slider name.
The browser waits for fresh joint feedback before enabling sliders and derives
trajectory duration from joint speed limits. Action controls finish one motion
before dispatching the latest queued edit.

Only configured drive and joint interfaces accept browser commands. A wall-time
watchdog publishes zero velocity when drive messages stop. Joint commands hold
their positions. Graph, parameter inspection and topic visualization do not
require a joint controller. Parameters are read-only.

HTTP `/robocore_viz/config` returns the resolved profile;
`/robocore_viz/joint-controls` returns groups with resolved URDF limits.
The package retains the upstream ROS 1 compatibility layer, but the profile/action
integration is validated on ROS 2 Jazzy.

Map markers use a 10 Hz `_robot_pose` websocket stream derived from TF. Set
`frames.fixed` (default `map`) and optionally `frames.robot`; when omitted, the
robot frame comes from the configured odometry's child frame. This stream is
internal to the bridge, not a published ROS topic. Run with `use_sim_time:=true`
for simulations so header ages are measured against the same ROS clock.

An optional `topics.tracking` binding accepts RTAB-Map OdomInfo (`lost`). The
dashboard distinguishes tracking loss, stale pose and map update age, rejects
invalid null odometry, and hides expired robot markers. Without that binding,
pose validity and timestamp checks still apply.
