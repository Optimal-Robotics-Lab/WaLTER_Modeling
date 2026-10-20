# WaLTER_Modeling
Model generation scripts and descriptor files for the WaLTER robot morphology.

The repo contains two independent MuJoCo model pipelines:

- **parametric**: a fully parameterized model built from YAML configs. Comes in WS and FS sizes.
- **hardware**: a high-fidelity full-scale (FS) model of the real hardware, using CAD meshes and measured values.

## Layout

```
WaLTER_Modeling/
  cli.py                 # generate-walter command
  naming.py              # default naming convention (constants + patterns)
  parametric/
    generator.py         # ParametricModel
    configs/WS, configs/FS
  hardware/
    generator.py         # HardwareModel
    configs/hardware_config.yaml
    assets/meshes/visual, assets/meshes/collision
docs/
  naming_convention.md
scripts/
  view_parametric.py
  view_hardware.py
```

## Setup

```bash
uv sync     # installs the package (editable) plus the dev tools: ruff, pyright
```

## Generating models

```bash
uv run generate-walter parametric -o WaLTER_WS.xml        # parametric, WS
uv run generate-walter parametric --fs -o WaLTER_FS.xml   # parametric, FS
uv run generate-walter hardware -o WaLTER_Hardware.xml    # hardware, FS
```

From Python:

```python
from WaLTER_Modeling import HardwareModel, parametric

hw = HardwareModel()                       # hw.spec, hw.mj_model, hw.model_xml
ws = parametric.build(full_scale=False)    # same attributes
```

## Viewing

```bash
uv run python scripts/view_parametric.py [--fs]
uv run python scripts/view_hardware.py
```

Where the interactive viewer isn't available (for example in WSL), render images instead. They're written to `renders/`, which git ignores:

```bash
MUJOCO_GL=glfw uv run python scripts/render_hardware.py                     # whole robot
MUJOCO_GL=glfw uv run python scripts/render_hardware.py --collision --frames # overlay contact shapes and body frames
MUJOCO_GL=glfw uv run python scripts/render_hardware.py --focus fl_shin --distance 0.7 -o renders/fl_leg.png
```

## Naming convention

All new models follow [docs/naming_convention.md](docs/naming_convention.md). The hardware model uses it already. The parametric model still uses its original names and will be migrated.

## Code standards

Python code follows the Optimal Robotics Lab style guide, and changes go through feature branches and Pull Requests into `main`. Both are summarized for the Cursor agent in `.cursor/rules/`. Formatting, linting, and type checking are configured in `pyproject.toml`:

```bash
uv run ruff format <files>
uv run ruff check <files>
uv run pyright <files>
```
