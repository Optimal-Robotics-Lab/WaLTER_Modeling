"""Opens the hardware model in the MuJoCo viewer, standing on a flat floor.

Requires the package to be installed (``uv sync``); run with
``uv run python scripts/view_hardware.py``.
"""

import time

import mujoco
import mujoco.viewer

from WaLTER_Modeling import HardwareModel

FLOOR_RGBA = [0.5, 0.8, 0.9, 1]
# Zero x/y half-sizes render the plane as infinite; the third entry is the grid spacing.
FLOOR_SIZE = [0, 0, 0.05]
LIGHT_POSITION = [0, 0, 5]


def main() -> None:
    walter = HardwareModel(inertia_from_visual=True, roofrack=True)

    # The hardware model has no environment of its own; add one before compiling.
    walter.spec.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=FLOOR_SIZE,
        rgba=FLOOR_RGBA,
    )
    walter.spec.worldbody.add_light(pos=LIGHT_POSITION, dir=[0, 0, -1])

    model = walter.spec.compile()
    data = mujoco.MjData(model)

    # Pace the simulation against wall-clock time so it plays back in real time.
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
