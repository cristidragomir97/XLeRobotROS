# RoboCasa-compatible Lightwheel assets

Source: https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF
Authors/attribution: NVIDIA / Lightwheel. License: CC-BY-4.0 (https://creativecommons.org/licenses/by/4.0/).

Selected: Refrigerator040, Stove066, GlassCup008 and HoneyBottle002. Source models
are unchanged; prepared XML scales cup/bottle geometry to 45%, inserts computed
inertials, and the scene namespaces identifiers and adapts collision masks.
`manifest.json` records the pinned revision and SHA256 provenance. Downloads and
selected source/prepared files are cached locally by scripts/setup-world-assets.sh.

The custom room/furniture layout is adapted from Mr Handy's home world and lives
in src/xlerobot_sim/worlds/home.xml. Prepared assets are retained with pinned
revision/checksum metadata and baked into the Docker image.
