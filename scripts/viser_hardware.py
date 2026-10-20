"""Opens the hardware model in a browser via Viser, standing on a flat floor.

The native MuJoCo viewer opens an OpenGL window, which crashes under WSL.
Viser runs a web server instead: the browser renders the meshes, so this
process never needs a display. Run from the repo root, then open
http://localhost:8080.

Pause playback, then drag a knee slider. Each knee rotates only its own shin.
While the simulation is playing the knees stay near zero: the two wheels sit
on opposite sides of the knee, so gravity applies no flexing torque.
"""

import mjviser.viewer as mjviser_viewer
import mujoco

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

    print("Open http://localhost:8080 in a browser. Press Ctrl+C here to stop.")
    print("Pause, then move a knee slider. That shin rotates on its own.")
    mjviser_viewer.Viewer(model, data).run()


if __name__ == "__main__":
    main()
