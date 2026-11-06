# Windows and WSL

The tested setup uses a WSL shell, Docker Desktop and the native WSL MuJoCo
viewer. Run the [README workflow](../README.md) from WSL; the bootstrap selects
the shared multiROS WSL profile and Mesa D3D12 automatically. Performance and remaining
limits are in [WSL performance](wsl-performance.md).

## Native PowerShell / WSLC (unvalidated)

Use native PowerShell, Git and Pixi on 64-bit Windows. WSL 2.9.3 or newer must
provide `wslc.exe`; Docker Desktop is not required. See
[Microsoft's documentation](https://learn.microsoft.com/en-us/windows/wsl/wsl-container).

```powershell
wsl --update
wslc version
wslc run --rm hello-world
git clone --recurse-submodules https://github.com/cristidragomir97/XLeRobotROS.git
cd XLeRobotROS
pixi install --locked
pixi run doctor
```

`doctor` should report `runtime: wslc`. It should not call Docker. The host
runtime selects WSLC automatically; `MULTIROS_RUNTIME=wslc` can force it.

## Build and check

```powershell
pixi run up
pixi run check-rendering --require-accelerated
pixi run test
pixi run test mod101
```

The first command builds the shared rendering base and ROS image. Source is
mounted at `/workspace/xlerobot`; build/install/log use named volumes.
The persistent container is `xlerobot-ros-dev`.

The GPU check must report an accelerated GL renderer and pass textured RGB,
metric depth and segmentation. A successful `--gpus all` request is not enough.
This is the first actual Windows test of the EGL/D3D12 path.

## Run and inspect

```powershell
pixi run sim
# Another terminal:
pixi run view
```

Open http://localhost:8889. Confirm the robot scene and meshes appear, joints
follow simulation, RGB/depth change, and closing the viewer leaves sensors
running. Then run the live checks from [development](development.md).

```powershell
pixi run sim-stop
pixi run down
```

Keep the `doctor`, rendering and model-check output with the Windows/WSL/GPU
versions. Dependency locks and command-planning tests do not establish that
these runtime checks will pass.

## Troubleshooting

Use `pixi run logs`, `pixi run status` or `pixi run exec` to inspect the runtime.
Check `wslc run --help` and `wslc build --help` if the installed release rejects
a flag. The adapter was checked against Microsoft's current CLI source.

To isolate GPU problems, stop the project, remove its stopped container and
select software rendering:

```powershell
pixi run down
wslc container rm xlerobot-ros-dev
$env:MULTIROS_BACKEND = "cpu"
pixi run up
pixi run check-rendering
```

This preserves named volumes and uses OSMesa/llvmpipe. It is a CPU run, not a
GPU success. To retry acceleration, stop/remove the container again and remove
the override with `Remove-Item Env:MULTIROS_BACKEND` before `up`.

Recreate the stopped container after image, mount or environment changes;
`up` starts an existing WSLC container without silently discarding it.

`wslc settings` configures session CPU/memory/storage. Its settings file is
`%LOCALAPPDATA%\wslc\settings.yaml`; `.wslconfig` does not control this session.
