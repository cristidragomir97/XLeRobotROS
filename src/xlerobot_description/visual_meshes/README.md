# Reduced visual meshes

Derived from the pinned XLeRobot wheel meshes and the existing cart connector.
Original sources, author credits and Apache-2.0 attribution remain in
[NOTICE](../../../NOTICE) and [cart mesh provenance](../cart_meshes/README.md).
Source checksums, output checksums, tool versions, requested budgets and actual
triangle counts are recorded in `manifest.json`.

These assets apply only to explicitly massless, non-colliding visual geoms.
Collision meshes, hidden inertia proxies and source files stay intact. Unused
original visual assets are removed from the generated MJCF. The final wheels
use 16,000 triangles each; a more aggressive 8,000-triangle version showed rough
edges in close-up and was not retained. The connector reaches about 60,000
triangles; the simplifier cannot reach the requested 6,000 without further work.

The runtime defaults to these assets. Set `XLEROBOT_VISUAL_MESHES=full` in `.env`
and recreate the container for the originals. This setting is independent of
`XLEROBOT_RENDER_QUALITY` (shadows, reflections and MSAA).

To regenerate in a development environment:

```bash
python -m pip install trimesh==5.2.0 fast-simplification==0.1.13
python tools/simplify_visual_meshes.py
```

The simplifier is not a runtime dependency. Rebuild `xlerobot_description` after
regenerating installed assets. Use `checks/check_visual_meshes.py` against a full
and reduced generated scene to verify calibration, collision geometry, inertia,
actuators and identical dynamics from the same snapshot. Rendering measurements
and limits are in [WSL performance](../../../docs/wsl-performance.md).
