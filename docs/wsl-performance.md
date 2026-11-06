# WSL / Docker Desktop performance investigation

Measured 2026-11-05 in Ubuntu WSL2 with Docker Desktop, MuJoCo 3.13.0,
Intel UHD Graphics 770, and the standard robot in `robocasa_home`.
The shared runner/viewer came from the sibling `../multiROS` checkout with
local changes, using `MULTIROS_ROOT`; the pinned submodule was not changed.

## Native viewer comparison

After closing a competing game, run the same benchmark sequentially on WSL,
Windows, Windows, then WSL. Pause the container camera during these comparisons.
Use a hidden 1280×720 GLFW window, the same camera settings, no shadows or planar
reflections, five warmup frames and 90 measured frames. `glFinish` includes GPU
completion in the frame time. These are rendering costs, not measured passive
viewer presentation rates.

| Host | Median frame time, runs 1 / 2 | p95, runs 1 / 2 | Median snapshot read |
| --- | --- | --- | --- |
| WSL, Mesa D3D12 | 46.54 / 43.27 ms | 54.77 / 50.07 ms | 0.17 ms |
| Windows, native Intel GL | 29.26 / 29.10 ms | 33.28 / 32.81 ms | 4.1 ms over WSL UNC |

Windows has a measurable rendering advantage. WSL is selected by user preference
because approximately 21–23 rendered frames/s is acceptable. Tests with Fortnite
running were inconsistent and are excluded from these results.

The reproducible benchmark is `checks/benchmark_viewer.py` in multiROS. Run it in
each platform's matching MuJoCo environment with `--runtime PATH_TO_RUNTIME`.
On WSL select `GALLIUM_DRIVER=d3d12` and add `/usr/lib/wsl/lib` to
`LD_LIBRARY_PATH`; the default host configuration otherwise selected llvmpipe.
Raw results are in multiROS at
`investigations/rendering/wsl-desktop-2026-11-05.json`.

## Camera pipeline

The old Python path drew the same scene three times for RGB, depth and
segmentation. The new `camera_rendering.py` uses public MuJoCo GL APIs and two
draws: normal RGB, then ID-color rendering with **both color and depth buffers
read back from that draw**. Segmentation identifies robot pixels for depth
filtering. Reverse-Z metric-depth conversion matches the pinned MuJoCo reference.

With other rendering workloads paused, 12 paired samples on a frozen full robot
state measured a median **121.25 ms for three passes versus 82.50 ms for two**.
The compatibility archive cost 1.61 ms and physics snapshot reads 0.36 ms.
Full-scene RGB, depth and filtered depth matched the reference within 1e-6.
`checks/test_camera_rendering.py` also compares moved geometry and both quality
presets against independent MuJoCo reference renders.

For a repeatable camera comparison inside the container, pause other GPU work
and run:

```bash
python3 checks/benchmark_camera.py --frames 12
```

After restarting the optimized simulation and opening the WSL viewer, an 8-second
WebSocket sample measured 7.6 Hz camera/visual odometry delivery, median camera
source age 150 ms, odometry age 170 ms, and bridge delivery below 1 ms. The initial
effects-enabled camera measurement was 0.6 Hz and 1.42 s old. Those live samples
have different GPU contention conditions and are not an isolated speedup ratio.

## ROS integrations reviewed

- [ros-controls mujoco_ros2_control](https://control.ros.org/jazzy/doc/mujoco_ros2_control/doc/index.html)
  provides a ROS Jazzy system interface and optional camera plugins. Its
  [camera implementation](https://github.com/ros-controls/mujoco_ros2_control/blob/main/mujoco_ros2_control_plugins/src/camera_plugin.cpp)
  snapshots state for its render thread and reads RGB/depth together after one
  draw. It does not provide the robot segmentation used by this pipeline. The
  two-pass change adopts combined readback while preserving the self-filter.
- [MuJoCo ROS RenderCore](https://ubi-agni.github.io/mujoco_ros_pkgs/2.0.0/architecture/render_core/render_core.html)
  separates bounded snapshot/frame storage from rendering and consumers.
  [Camera configuration](https://ubi-agni.github.io/mujoco_ros_pkgs/2.0.0/concepts/configuration/configuration.html)
  supports RGB/depth/segmentation streams and renders lazily for subscribers.
  Demand-driven rendering and bounded latest-state transport are useful future
  directions if more cameras or consumers are added.

Neither integration was installed or benchmarked here. Replacing the physics and
control bridge requires validating XLeRobot's joint mapping, Mod101 variant,
wheel contacts, IMU/TF and controller behavior. Current measurements establish a
redundant draw cost; they do not establish that changing ROS plugins would improve
the remaining GPU-bound render cost.

## Current runtime check — 6 November 2026

With the native WSL viewer open, a 10-second ROS sample received 74 RGB frames:
**7.36 Hz**, median source age **140 ms**, p95 **203.5 ms**, and physics real-time
factor **1.000**. Age is latest received simulation clock minus image stamp.
This is a live freshness measurement, not an isolated GPU benchmark or a new
measurement of native-window presentation FPS. Other foreground GPU workload
was not independently verified during this sample.

The configured camera target remains 15 Hz; the current full-scene pipeline
does not achieve that target under this workload.

## Native viewer overhead — 6 November 2026

Repeated 10-second ROS samples on the same running simulation in open, closed,
closed, open order gave:

| Native viewer | Camera delivery | Median image source age | Physics real-time factor |
| --- | --- | --- | --- |
| Open, run 1 | 7.21 Hz | 160 ms | 1.000 |
| Closed, run 1 | 10.04 Hz | 120 ms | 1.000 |
| Closed, run 2 | 8.92 Hz | 130 ms | 1.000 |
| Open, run 2 | 7.23 Hz | 160 ms | 1.000 |

The mean of the open rates is about 24% below the mean of the closed rates.
This is a throughput comparison, not a GPU utilization measurement. The native
window creates its own context and draws the scene alongside container camera
rendering; it does not advance physics. Camera source age grew by 30–40 ms,
while physics stayed at real time. Dashboard/mapping remained enabled in every
sample, so this does not measure their separate costs. Other foreground GPU
load was not independently verified.

[Raw samples](wsl-viewer-overhead-2026-11-06.json). Samples precede removal of the
unused compatibility image archive; do not attribute that later change to this
comparison. Earlier isolated native render costs were 43–47 ms/frame; those are
not a new measurement of live presentation FPS.

## Visual mesh simplification — 6 November 2026

Reduced the three wheel visuals from 78,560 to 16,000 triangles each, and each
of the three cart connector visuals from 146,866 to 60,284. Visible mesh-instance
triangles fall from **999,513 to 552,087 (44.8%)**. Unmodified hidden proxies stay
available for inertia; unused original visual assets are removed from generated
MJCF. The scene geometry and camera view were fixed for the comparisons below.

| Frozen-state rendering | Original | Simplified |
| --- | --- | --- |
| Native 1280×720, median | 41.77 ms | 37.91 ms |
| Native p95 | 45.51 ms | 42.46 ms |
| RGB-D/self-filter 640×480, median | 82.27 ms | 76.38 ms |
| RGB-D/self-filter p95 | 84.72 ms | 80.52 ms |

Native results use 40 paired samples after 10 warmup pairs, alternating order,
one hidden GLFW window and `glFinish`. RGB-D results use 12 paired samples after
three warmup pairs, alternating order and combined readback. Active cameras and
the normal native viewer were paused during isolated comparisons; physics kept
running. Both use fast effects and the same frozen state. These are render
costs, not measured live window FPS or a guarantee about other GPU workloads.
Separate-process native comparisons varied substantially; the paired comparison
controls ordering better. The measured cost reduction is about 9% native and
7% RGB-D, substantially smaller than the triangle reduction.

Collision geometry, inertia, joint/actuator settings and camera parameters match
exactly. From the same snapshot, 300 physics steps produce identical positions
and velocities. Full-body and wheel close-up images were inspected; a rougher
8,000-triangle wheel version was discarded. The connectors stop above the
requested budget, so their actual count is recorded rather than claiming 6,000.

[Raw measurements](wsl-mesh-simplification-2026-11-06.json).
[Asset provenance and regeneration](../src/xlerobot_description/visual_meshes/README.md).
Set `XLEROBOT_VISUAL_MESHES=full` before recreating the container for the originals;
`simplified` is the default. No simplification library is required at startup.

Live samples after the restart, with the restored native window open, measured
4.74 and 4.66 Hz with median image ages 230/220 ms and real-time physics. The
visible WSL window frame was 1782×978. These live rates are worse than earlier
sessions despite the paired render-cost reduction; they are not evidence of an
end-to-end speedup. Host GPU/display contention and the live viewer remain
unresolved. Windows process inspection found no Fortnite/Unreal process during
this check.

Closing the native window afterward gave 9.91 Hz, 120 ms median / 150 ms p95
image age and real-time factor 1.000. The viewer was reopened after measurement.

## Viewer implementation and compressed RGB — 6 November 2026

The old passive viewer synchronizes complete visual model/data arrays repeatedly
and renders independently of the Python update loop. Measured sync calls cost
about 8.5–8.8 ms, while snapshot loads cost about 0.4 ms. Sleeping in that loop
did not cap the render thread. The shared GLFW frontend avoids these copies,
uses kinematics-only transforms, and budgets complete frames at 15 FPS. It
skips minimized/unchanged rigid scenes and invalidates for visible motion,
lights, camera input and window events. The current live samples still drew
every capped frame; no idle-cache saving is claimed for them.

Four ten-second ROS samples in full UI, GLFW, GLFW, full UI order measured:

| | Full MuJoCo UI | Capped GLFW window |
| --- | --- | --- |
| Viewer CPU, percent of one core | 56.8–58.6% | 14.5–15.0% |
| Camera delivery | 7.91–7.94 Hz | 8.87–9.01 Hz |
| Median image age | 140 ms | 130 ms |
| Physics real-time factor | 1.000–1.001 | 1.000 |

The full UI's default window and the 1280×720 lightweight window differ, so this
is a comparison of complete frontend configurations, not one isolated change.
GLFW submitted approximately 14.9 frames/s. Full-UI sync counts are not its
presentation FPS. New frontend validation is WSL-only; macOS/native Windows
defaults and Mr Handy's recipe are unchanged.

Every sample subscribed to compressed RGB. Median JPEG payload was about
15.3–15.4 KB versus 921.6 KB raw (about 60 times smaller for this view). The
dashboard passes these JPEG bytes through without another encoding step. Depth
previews already used JPEG over WebSocket; metric depth and raw RGB for mapping
are unchanged. Compression improves image transport, not the independent native
3D renderer. These samples do not isolate a CPU benefit from JPEG.

[Raw profiles](wsl-viewer-frontends-2026-11-06.json) include per-phase timings,
draw counts and camera freshness. `pixi run view --rate 20` raises the WSL drawing
cap. `--frontend passive --full-forward` restores full diagnostics at higher
resource cost. These short tests do not establish long-run performance.

## Memory use

Measured on 6 November 2026 using proportional set size (PSS), after the mesh,
compressed-RGB and lightweight-viewer changes. The same standard robot, furnished
home, wheel odometry, RTAB-Map, Nav2 and dashboard are enabled. Separate fresh
map databases preserve prior user data; the known saved-database reuse assertion
is not resolved by this change.

| Process group | Before | After validation |
| --- | --- | --- |
| Physics + RGB-D rendering | 2,189 MiB | 718 MiB |
| Native viewer | 1,113 MiB | 394 MiB |
| ROS mapping/navigation/controllers/dashboard/launch | 889 MiB | 1,077 MiB |
| Container init/shell overhead | 3 MiB | 3 MiB |
| **Application total** | **4.10 GiB** | **2.14 GiB** |

The final split is approximately **297 MiB physics**, **421 MiB camera rendering**
and **394 MiB native viewing**. RTAB-Map accounts for about **585 MiB** of the ROS
group. ROS caches warmed and motion/dashboard checks ran before the final sample;
the before/after snapshots are not at identical uptimes. No ROS memory limits,
map resolution, texture resolution or model fidelity were reduced. These are
short fresh-map samples, not a peak-memory measurement or a fixed RAM budget.

The large savings come from model preparation. The compiled model buffer is
206.7 MiB, but an isolated XML load used roughly 998 MiB RSS and retained an
asset cache of 231 MiB. Loading the identical MJB in a fresh process used about
267 MiB RSS. Clearing the cache alone did not recover all compilation memory.
The runtime now compiles once in a short-lived worker, exits that worker, and
loads the compiled MJB into physics and camera processes. The WSL viewer uses
the same binary and hashes it incrementally without a full-file Python bytes
allocation. Fast rendering also avoids large unused shadow buffers; the GLFW
window uses a minimal offscreen allocation because it draws into the window.

XML/MJB compiled physics match exactly over 300 steps. RGB, metric depth and
self-filtered depth are bit-for-bit identical with the reduced unused buffers.
The native MJB window passes a textured/nonblank framebuffer check. Standard live
control, camera/IMU/TF/mapping, dashboard and seven regression tests pass. Mod101
preparation/compiled control ABI and 28 shared-core tests also pass.

`pixi run measure-memory` reproduces the process split. PSS apportions shared
library pages rather than adding duplicate RSS. ROS bindings within physics and
camera processes are counted with simulation. The total excludes Windows GPU
allocations and the WSL VM's retained filesystem cache, so Task Manager can show
more memory than this application total.

[Raw process measurements](wsl-memory-2026-11-06.json). MuJoCo documents its
[compiler asset cache](https://mujoco.readthedocs.io/en/3.13.0/APIreference/APIfunctions.html#asset-cache)
and [compiled model loading](https://mujoco.readthedocs.io/en/3.13.0/APIreference/APIfunctions.html#mj-loadmodel).
