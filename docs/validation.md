# Validation and limits

## Current architecture — 5 November 2026

Tested on Apple Silicon/M4 with lighter, ROS Jazzy and MuJoCo 3.13.0. Physics,
RGB-D rendering and ROS run in the container; the native viewer mirrors state.
Native Windows/WSLC remains unvalidated. WSL/Docker Desktop results follow.

- The shared patched Mesa image passed textured RGB, known metric depth and
  segmentation. The diagnostic reported Zink/Venus/MoltenVK on Apple M4.
- Standard and Mod101 isolated model checks passed: nonzero-pose frame agreement,
  floating-base contacts, wheel-driven forward/strafe/yaw motion, stopping,
  IMU gravity, camera depth, self-filtering and the controller joint ABI.
- Standard live controller, RGB-D, TF, dashboard and watchdog checks passed.
  Camera publication was roughly 13–14 Hz at 640×480 with the fast rendering
  profile, mapping and native viewing active; 15 Hz is the configured target.
- A short Nav2 goal passed after increasing the action acknowledgement timeout
  from 20 to 200 ms. Physical motion was 0.426 m for a 0.6 m requested goal,
  within the configured 0.2 m goal tolerance. The watchdog was unchanged.
- Brief visual-odometry loss recovered automatically. Loop-closure rejections
  occurred on the reused map. These tests do not certify long-run navigation.

The workflow cleanup is verified separately below; it does not replace those
sensor/control checks. Full GPU live Mod101 validation is still outstanding.

## Earlier robot and arm checks

The October 2026 model/control work established full robot mesh/frame agreement,
passive roller/caster contacts, variable joint counts and both arm profiles.
Mod101 control and partial-joint actions passed, including rejection of unknown
joints and bounded commands. No physical servo bus was opened.

The current Mod101 mounts use X=0, Y=±0.133 m and the documented forward-facing
yaw. A 545-pose geometry study found 30 inward configurations intersecting the
tower per arm. Neutral tower/cart meshes were clear. The earlier 140/100/82 mm
rail comparison applied to the previous rearward mount position; shortening the
arm did not eliminate inward collisions. See
[the geometry results](mod101-tower-clearance.json) and [mount details](mod101.md).

This is finite sampling, not collision-aware planning. Robot self-collisions are
disabled. Arm masses, servo dynamics, friction, caster compliance and cart mass
remain estimates. Physical brackets, calibration and hardware traction are
unverified. Wrist camera meshes are present, but wrist image streams were not
validated.

## Workflow checks

The shared core has tests for argument handling, project-scoped shutdown,
WSLC command planning, environment parsing and snapshot validation. The native
viewer was attached to the existing live scene and to an independent model.
Closing those probe windows left the existing simulator running.

The replacement `pixi run build` completed all 15 selected ROS packages.
Both model configurations, the three dashboard configuration tests and the GPU
RGB-D diagnostic passed through the new tasks. Locked host installation passed,
and `doctor` resolves the clean, commit-pinned core under `tools/multiros`.

See the pinned [multiROS validation](../tools/multiros/VALIDATION.md) for the
cleanup check results. The Windows procedure in [windows.md](windows.md) is a
manual handoff, not a claim that WSLC rendering has passed.

## WSL / Docker Desktop — 6 November 2026

The standard robot runs with the fast rendering preset on Intel UHD 770 via
Mesa D3D12, MuJoCo 3.13.0 and the native WSL viewer. The shared runner reads
`[tool.multiros]` from `pixi.toml`; the JSON recipe has been removed.

Live checks passed synchronized RGB/metric depth/self-filtering, IMU, TF, depth
cloud and RTAB-Map occupancy; arm trajectory execution and unknown-joint rejection;
lateral visual odometry and command watchdog; and all 17 claimed commands / 34
state interfaces under the expected controllers.

The existing image contained an older C++ control bridge using uptime timestamps
while current Python used wall-clock timestamps. This rejected fresh commands
as stale. Rebuilding `xlerobot_control` from current source fixed the failure.
The entrypoint now loads the development overlay even when only the C++ package
has been rebuilt. New image builds include the current source directly.

The reusable camera benchmark/tests remain under `checks/`; eight unused graphics
and rate probes were removed. See [development](development.md) for the layout
and [WSL performance](wsl-performance.md) for earlier rendering measurements.

Dashboard HTTP/WebSocket configuration, current map-frame pose, visual tracking,
RGB/depth previews, head actions, base commands and watchdog checks also passed.
The maintained rendering diagnostic reported Intel UHD 770 through D3D12 and
passed textured RGB, 1.99 m reference depth and segmentation. All four robot
unit/regression tests passed. The shared core passed 23 tests with viewer
dependencies installed. Current live rates are recorded in
[WSL performance](wsl-performance.md).

The browser 3D integration, launch switches, dependency declarations and port
were removed. Installed browser-renderer packages were uninstalled from the
active container, and the simulation package was rebuilt. Only dashboard port
8889 is exposed. Camera frames now publish directly without the unused image
archive. After restarting, all four regression tests and live RGB-D/mapping,
arm action, base odometry and watchdog checks passed again. Native viewer
overhead measurements are in [WSL performance](wsl-performance.md).

A subsequent dashboard recheck exposed an RTAB-Map `Memory::addLink` assertion
while reusing the accumulated default database. The original database was
preserved; this session was restarted with
`.runtime/robocasa_home_camera_cleanup_20261106.db` for a fresh mapping check.
This does not resolve the saved-database reuse failure.

With the fresh database, dashboard map/pose/tracking streams, RGB/depth, head
controls, strafe commands and watchdog checks passed.

## Wheel odometry default — 6 November 2026

RTAB-Map remains the mapping/localization backend. The omni controller publishes
`/odom` and `odom -> base_footprint` from measured wheel velocities, with
`open_loop=false`. RTAB-Map owns `map -> odom` and interpolates odometry through
TF at synchronized RGB-D timestamps. It does not subscribe to visual odometry
diagnostics. No `rgbd_odometry` node runs in the default mode. The IMU remains
published but is not fused through an EKF. Ground truth is diagnostic only.

All four regression tests and live RGB-D/mapping, arm action, base/watchdog,
dashboard and controller-ownership checks passed. The short lateral check
measured 0.208 m of physical travel and 0.214 m from wheel odometry. With the
camera process briefly paused, wheel odometry continued at 50 Hz and no camera
frames arrived; the camera was resumed immediately afterward. This establishes
independence from image processing, not long-run slip/drift accuracy.

The current test session uses `.runtime/robocasa_home_wheel_20261106.db`. Future
default wheel sessions use a separate `_wheel.db` database, preserving older
visual-odometry maps. `odometry:=rgbd` remains an explicit comparison mode.

Nav2 also reached the 0.6 m map goal successfully; ground truth confirmed
0.409 m of base movement within the configured goal tolerance.

## Reduced visual meshes — 6 November 2026

The default model now uses generated visual-only meshes for the three drive
wheels and three cart connectors. Visible triangle instances drop from 999,513
to 552,087. Full and reduced compiled models have identical collision meshes,
inertia, joint/actuator settings and camera parameters. From the same state,
300 physics steps produce exactly identical positions and velocities.

Standard and Mod101 isolated model checks, all four regression tests, live
RGB-D/IMU/TF/mapping, arm actions, base odometry/watchdog and dashboard checks
passed with the reduced visuals. Full-body and wheel close-ups were inspected.
The native viewer was restored and the camera resumed after isolated timings.
See [performance](wsl-performance.md#visual-mesh-simplification--6-november-2026)
for paired timings, raw results and the limited size of the measured gain.

## Compressed display and capped viewer — 6 November 2026

The dashboard now consumes standard JPEG RGB, with byte-for-byte pass-through
verified by a regression test. Raw RGB/depth calibration and synchronization,
metric depth/self-filtering, IMU, TF, mapping, arm actions, lateral wheel odometry,
watchdog and dashboard controls all passed again. Seven robot regression tests
pass. The shared core passes 28 tests, including visibility-aware pose caching
and public camera controls. The three new viewer CPU tests also pass against
Mr Handy's MuJoCo 3.12.0 environment without starting that robot.

Hidden-window GPU smoke checks verify actual rendering, unchanged-pose reuse,
visible motion redraw, zoom/orbit and no physics-state mutation. The WSL recipe
selects a 1280×720 capped GLFW window; other platform/robot defaults are retained.
The final frontend comparison and its limitations are in
[performance](wsl-performance.md#viewer-implementation-and-compressed-rgb--6-november-2026).

## Compiled model memory reduction — 6 November 2026

Preparation runs in a short-lived worker and emits the exact compiled model.
Physics, RGB-D and the WSL viewer load `.runtime/scene.mjb`; XML remains available
for inspection. Original collision/inertia/actuation/calibration and 300-step
dynamics match. RGB, metric depth and filtered depth are byte-identical with
unused fast-mode buffers reduced, and the native window renders textured geometry.
Live RGB-D/IMU/TF/mapping, arm trajectories, lateral wheel odometry/watchdog and
dashboard controls pass. Seven regression tests, 28 shared tests and a separate
Mod101 worker/compiled control ABI check pass. Live camera delivery was 8.9 Hz,
median image age 130 ms and physics real-time factor 1.001 with the viewer open.

Application PSS fell from about 4.10 GiB to 2.14 GiB; see
[memory measurements](wsl-performance.md#memory-use) for the sim/ROS/viewer split,
map/warmup conditions and excluded WSL/GPU allocations. Original saved map files
are preserved; this test uses `.runtime/ram-optimized-20261106.db`. The previously
observed saved-database reuse assertion remains unresolved.
