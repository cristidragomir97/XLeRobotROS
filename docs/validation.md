# Validation and limits

## Current architecture — 5 November 2026

Tested on Apple Silicon/M4 with lighter, ROS Jazzy and MuJoCo 3.13.0. Physics,
RGB-D rendering and ROS run in the container; the native viewer mirrors state.
Windows/WSLC and Linux GPU runtime testing are still pending.

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
