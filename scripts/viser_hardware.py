"""Opens the hardware model in a browser via Viser, standing on a flat floor.

The native MuJoCo viewer opens an OpenGL window, which crashes under WSL.
Viser runs a web server instead: the browser renders the meshes, so this
process never needs a display. Run from the repo root, then open
http://localhost:8080:

    uv run python scripts/viser_hardware.py
"""

import time

import mujoco
import viser
from mjviser import ViserMujocoScene

from WaLTER_Modeling import HardwareModel

FLOOR_RGBA = [0.5, 0.8, 0.9, 1]
# Zero x/y half-sizes render the plane as infinite; the third entry is the grid spacing.
FLOOR_SIZE = [0, 0, 0.05]
LIGHT_POSITION = [0, 0, 5]


def main() -> None:
    walter = HardwareModel()

    # The hardware model has no environment of its own; add one before compiling.
    walter.spec.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=FLOOR_SIZE,
        rgba=FLOOR_RGBA,
    )
    walter.spec.worldbody.add_light(pos=LIGHT_POSITION, dir=[0, 0, -1])

    model = walter.spec.compile()
    data = mujoco.MjData(model)

    # Replaces mujoco.viewer.launch_passive: a web server streams the scene.
    server = viser.ViserServer()
    scene = ViserMujocoScene(server, model, num_envs=1)
    # Side panel: contacts, forces, frames, and geom groups
    # (group 2 = visual meshes, group 3 = collision shapes).
    scene.create_visualization_gui()

    print("Open http://localhost:8080 in a browser. Press Ctrl+C here to stop.")
    try:
        while True:
            step_start = time.perf_counter()

            mujoco.mj_step(model, data)
            # Replaces viewer.sync(): pushes the new body poses to the browser.
            scene.update_from_mjdata(data)

            # Pace against wall-clock time so playback stays in real time.
            elapsed = time.perf_counter() - step_start
            time_until_next_step = model.opt.timestep - elapsed
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    main()
