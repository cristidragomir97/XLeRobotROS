# Derived upstream cart geometry

These meshes retain the source triangles from Victor Wang and contributors'
pinned `Vector-Wangel/XLeRobot` repository, under its Apache-2.0 license.
`caster_N_fork.stl` and `caster_N_wheel.stl` partition the original ManiSkill
`raskogwheel1.stl`/`raskogwheel2.stl` into four caster assemblies and express
vertices in each caster's local frame. `cart_connector.stl` is the original
`hardware/XLeRobot 0-3-0 new wheel connector.stl`, converted from millimetres
to metres and centered for instancing. No replacement CAD shape is synthesized.

Reproduce the meshes and computed configuration with:

```bash
./scripts/manage.sh exec python3 tools/prepare_cart_mount.py
```

The helper writes `src/xlerobot_sim/config/cart_mount.yaml`: cart height
correction, bounds-derived caster pivots/radii, and connector fit transforms.
Connector XY follows the original motor mounts as a simulation fit; Z scales
to the measured gap, as the upstream assembly guide permits. Actual bolt
alignment, tyre stiffness and caster masses need hardware measurements.
