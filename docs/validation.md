# Validation

Executed 2026-10-13 with Dockerized ROS 2 Jazzy, MuJoCo 3.13.0,
RTAB-Map, and both pinned upstream submodules. Both upstream working trees stayed clean.

- Docker image and all eight selected workspace packages built successfully,
  including the upstream STS hardware plugin.
- Canonical URDF validation passed: 45 links, 44 joints, 17 movable joints,
  60 resolved mesh references and approximately 22.36 kg total mass, including
  the documented 8 kg cart estimate.
- Full-robot URDF/MuJoCo transforms agreed at six nonzero joint and floating-base
  poses, including roll, pitch and height.
- Floating chassis, 48 passive rollers and exactly 17 wheel/arm/head actuators
  were verified. A 0.3 m drop fell under gravity and settled onto ground contacts.
- Forward, lateral and yaw motion and stopping passed with four passive casters.
- All four cart tyres contacted the floor after settling, within 1 mm of the
  contact plane. Cart assembly Z was corrected by -35.93 mm; three source
  mounting connectors span the measured 42 mm gap.
- Caster/mount meshes were derived from pinned upstream sources; allocation of
  caster mass preserves the original aggregate 8 kg cart estimate.
- Bottom-base IMU measured approximately +9.80665 m/s² stationary specific force.
- RGB and metric depth rendering passed; diagnostic-room central wall distance
  was 3.993 m. Segmentation identified 20,159 robot pixels without masking the wall.
- Both grippers use detailed ManiSkill fixed jaw, gripper motor and moving jaw
  meshes; coarse convex hulls remain hidden collision geometry.
- Full cart/head/wrist meshes and the shared RoboCasa home compiled and stepped;
  only robot joints entered control/state publication.
- Shared control ABI, explicit joint ordering, upstream wheel kinematics,
  stale command stopping and nonfinite command stopping passed.
- Live controller manager reported all five controllers active, 17 claimed
  hardware command interfaces and 34 position/velocity state interfaces.
- Standard partial-joint arm trajectory succeeded; an unknown joint was rejected.
  Position commands are clamped to URDF limits by the simulator hardware adapter.
- Live RGB/depth/calibration timestamps matched. Camera, IMU, robot and SLAM TF
  resolved; RGB-D clouds and RTAB-Map occupancy maps arrived.
- Lateral motion measured 0.207 m in simulation and 0.206 m in visual odometry.
  The base stopped after command publication ceased.
- Dashboard checks received all 17 joints, RGB/metric-depth previews, map and IMU.
  A second browser client received the retained map while the first remained
  subscribed. Standard head actions moved and restored the head; dashboard
  strafe and command watchdog checks passed.
- Nav2 completed a 0.6 m RGB-D map goal, with 0.438 m measured simulated motion,
  within the configured 0.2 m estimated-pose goal tolerance.
- Chromium browser checks passed branding, 14 enabled joint sliders, actual
  head movement/restoration, RGB/depth previews, maps and ROS inspectors, with
  no JavaScript errors. The dashboard screenshot was refreshed.
- Generic visualization source copies in XLeRobot and Mr Handy are identical.

The launch retains sibling defaults: RoboCasa home, native MuJoCo window on,
mapping/navigation and mjviser on, RViz off. Current integration checks run
with `headless:=true`: this desktop's X server reports a 0×0 screen, preventing
GLFW from creating the native window. Native visualization was observed during
previous full-mesh verification before this display condition. Browser viewers
remain available on localhost:8889 and localhost:8082.

These checks cover short simulated motions, not long-route robustness,
physical servo configuration/calibration, hardware wheel traction or realistic sensor
noise. The base is contact-driven and floats freely. Contact friction and
servo dynamics and caster compliance remain estimates. Robot self-collisions
are disabled; ROS caster visuals retain their nominal unmeasured joint pose. The physical STS driver builds
but no servo bus is opened or tested.

CAD Viewer startup was attempted for the canonical URDF but failed because
its Python backend lacks compatible OCP/build123d dependencies. Structural
validation and direct forward-kinematics checks passed; browser visualization
and ROS RViz remain available.

## Mod101 arm configuration — 2026-10-13

The original-arm launch and the new `sim_mod101.launch.py` share the same
RoboCasa home, native-window, mapping/navigation and browser defaults.
`manage.sh sim standard|mod101` selects the launch; `sim-stop` handles either
and saves its database. A second configuration launch was rejected by the
shared runtime lock. The Mod101 mapping database uses a separate filename.

Verified with Mod101 pinned at `e0b450111f3fad7a514446c07a642c384dd5c332`:

- Docker built all 15 selected ROS packages, including `mod101_description`
  and six tool-description packages. A fresh image, with no source mount or
  workspace build volumes, expanded its installed xacro, resolved all arm
  assets, settled within limits and simulated lateral wheel motion.
- The shared platform URDF passed the skill validator: 31 links, 30 joints,
  5 movable, 30 resolved mesh references. The expanded Mod101 description
  passed: 87 links, 86 joints, 23 movable, 86 resolved mesh references,
  23.08 kg aggregate estimated mass.
- Both arms use the source small shoulder/elbow modules, 140/140 mm rails,
  `arm_dof=7`, PG grippers, base housings and wrist-camera meshes. In Mod101
  terminology that is six arm axes plus the gripper motor per arm.
- Full URDF/MuJoCo transforms, including all four prismatic mimic sliders and
  head optical axes, matched at six nonzero floating-base/arm poses.
- Source URDF, MuJoCo and ROS-control limits matched. With provisional stops
  engaging 2 mrad inside the ranges and a 4 ms limit time constant, settled
  motor joints remained within those ranges. Initial hardware hold commands
  and watchdog holds are bounded even if soft physics limits deflect.
- Both PG motors drove their two rack sliders through physical equality
  constraints, agreeing with source mimic ratios within 0.2 mm.
- Floating-base drop/landing, forward/strafe/yaw, stopping, four cart-tyre
  contacts, stationary IMU, RGB-D rendering, metric depth, self-filtering,
  furnished-home compilation and the variable-count shared control ABI passed.
- Live Mod101 controller manager reported all five controllers active,
  19 claimed commands and 38 state interfaces. Both arms moved and restored
  all six axes and their PG motors through the standard trajectory actions.
  Partial goals and unknown-joint rejection passed.
- Synchronized RGB-D, IMU, sensor/map TF, depth cloud and RTAB-Map map passed.
  Final lateral motion measured 0.208 m in physics and 0.211 m in RGB-D odometry;
  command silence stopped the base.
- Dashboard HTTP/WebSocket checks received the 19 joints, RGB/depth, map and
  IMU; durable map delivery, head movement/restoration, strafe and watchdog
  passed. Chromium showed 16 enabled joint sliders and functioning inspectors
  with no JavaScript errors. See [dashboard](media/dashboard-mod101.png),
  [full robot](media/mod101.png) and [arm mounting preview](media/mod101-arms.png).
- Nav2 completed a 0.6 m RGB-D map goal, with 0.439 m measured simulated motion,
  within the configured goal tolerance.
- The original configuration passed the model checks and its new explicit
  launch passed live controller/sensor/SLAM/action checks: 17 commands,
  34 state interfaces, lateral motion 0.209 m truth / 0.192 m estimate.
- All three submodule worktrees remained unchanged. The development container
  was recreated from the new image and the final Mod101 launch checked again.

Checks used `headless:=true` because the desktop screen remains unavailable.
CAD Viewer startup was attempted again for the platform and expanded Mod101
URDF paths; it still fails on the missing Python `OCP` backend. MuJoCo/browser
review and direct frame validation succeeded. The live preview is available
through the normal browser ports.

The Mod101-to-cart hardware adapter/bolt pattern is unverified. Source arm
mass properties, servo dynamics, joint stops and contact settings remain
simulation estimates. Robot self-collisions are disabled. No physical servo
bus, Mod101 MoveIt planning or wrist-camera image stream was started.

### Forward-facing arm mounts

Both Mod101 attachment joints now apply −π/2 yaw relative to their platform
mount datums, as requested. Their translations and source arm joint zeros stay
unchanged. The expanded URDF passed structural validation; all Mod101 frame,
gripper, base-contact, motion, IMU and rendering checks passed again. The live
Mod101 launch was restarted with this orientation, and the full/close-up model
previews were refreshed. CAD Viewer still cannot start because OCP is missing.

### Tower mesh clearance and mount height

Actual visual triangles were checked with FCL 0.7.0.11, using MuJoCo convex-hull
distances only as a conservative broad phase. The old neutral pose had a real
3 mm overlap between each fixed Mod101 base housing and the tower's mounting
plate. Increasing each attachment Z by 3.1 mm clears that overlap, with 0.1 mm
mesh clearance above the 3 mm plate. XY and −90° yaw remain unchanged.
The neutral moving shoulder clears the tower by approximately 10 mm.

Shoulder-length comparison, retaining a 140 mm forearm:

| Upper-arm rail | Left intersecting poses | Right intersecting poses |
|---|---|---|
| 140 mm | 129 / 545 | 123 / 545 |
| 100 mm | 130 / 545 | 133 / 545 |
| 82 mm | 134 / 545 | 133 / 545 |

Each arm was checked on the same 245-pose base/shoulder/elbow grid plus 300
seeded whole-arm/head configurations, placing rack sliders at their source
mimic relation. This is a finite geometry study, not continuous path checking.
Neither shorter rail eliminated inward-yaw/shoulder/wrist collisions; 140/140 mm
is retained. The neutral poses are clear on both sides. Full joint travel is
not certified safe; standard trajectory controllers do not avoid collisions,
and simulation self-collisions remain disabled. Arm/arm and cart collisions
are outside this check. The reusable command is `manage.sh test --clearance`;
add `--sweep` for the range study. Detailed poses, parts and source pins are
recorded in [mod101-tower-clearance.json](mod101-tower-clearance.json).

After the height correction, URDF validation and the complete Mod101 model
checks passed, including the new settled arm/tower triangle-mesh regression.
The Docker image includes pinned FCL; `manage.sh test --clearance` passes on
both neutral arms. A separate 21-point interpolation of each live acceptance
arm trajectory showed no tower intersections. The controller manager remains
active with 19 claimed command interfaces; previews and the live demo were
restarted with the corrected mounts. CAD Viewer startup was retried and still
fails because OCP is unavailable; MuJoCo rendering remains functional.

### Forward mount translation

Both Mod101 mounts moved 135 mm along ROS base_link +X, from X=−0.135 m to
X=0. Y remains ±0.133 m and mount height/yaw remain unchanged. The wrapper
transforms the forward offset through the inherited 1.5708 rad datum yaw.
This is a provisional mounting placement; the physical bracket is unverified.

Actual mesh checks found no neutral tower or cart intersections on either arm.
Closest housing/tower clearance is about 4.0 mm and moving-shoulder clearance
about 43.9 mm. Repeating the same 545-pose study found 30 intersecting poses
per arm, compared with 129 left / 123 right at the earlier rearward position.
The earlier shoulder-length comparison applies to that previous placement.
Inward configurations still exist that intersect the tower; this sampling
continues to be a geometry study, not collision-aware motion planning.

The forward-mounted profile passed structural URDF validation and all model
checks, including settled tower mesh clearance, six-pose frame agreement,
wheel/caster contact and motion, IMU and RGB-D rendering. The Docker image
was rebuilt and the live Mod101 launch restarted. CAD Viewer remains unavailable
because OCP is missing; the rendered previews and browser viewer work.
