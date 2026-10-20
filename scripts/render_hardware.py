"""Renders the hardware model to a PNG grid of views, without opening a window.

Used to check mesh placement where the interactive viewer is unavailable (for
example under WSL). The model is shown in its zero pose, floating in space.

Examples (from the repo root):
    MUJOCO_GL=glfw uv run python scripts/render_hardware.py --collision --frames
    MUJOCO_GL=glfw uv run python scripts/render_hardware.py \\
        --focus fl_shin --distance 0.7 -o renders/fl_leg.png
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from WaLTER_Modeling import HardwareModel
from WaLTER_Modeling.hardware.generator import COLLISION_GROUP

TILE_WIDTH = 640
TILE_HEIGHT = 480
DEFAULT_DISTANCE = 1.4
# Frame axis size relative to the model's mean body size; MuJoCo's default of 1.0
# draws axes that hide the leg meshes.
FRAME_LENGTH_SCALE = 0.35
FRAME_WIDTH_SCALE = 0.25
# Translucent red so collision primitives can be compared against the CAD meshes.
COLLISION_RGBA = [0.9, 0.2, 0.2, 0.35]

# (azimuth, elevation) in degrees. MuJoCo's camera looks along +x at azimuth 0,
# so azimuth 180 views the robot's front and -90 views its left (+y) side.
# The iso view is from the front-left and above, like the reference CAD render.
VIEWS = {
    "front": (180.0, 0.0),
    "left": (-90.0, 0.0),
    "top": (180.0, -89.0),
    "iso": (-135.0, -20.0),
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Render the hardware model to a PNG grid of views."
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("renders/hardware.png"),
        help="PNG file to write",
    )
    parser.add_argument(
        "--collision",
        action="store_true",
        help="Overlay collision geoms in translucent red",
    )
    parser.add_argument(
        "--frames",
        action="store_true",
        help="Draw body frames (red x, green y, blue z)",
    )
    parser.add_argument(
        "--focus",
        default=None,
        help="Body to center the camera on, for example fl_shin",
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=DEFAULT_DISTANCE,
        help="Camera distance from the focus point in meters",
    )
    return parser


def render_views(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    lookat: np.ndarray,
    distance: float,
    scene_option: mujoco.MjvOption,
) -> np.ndarray:
    """Renders every entry in VIEWS and tiles them into a 2-column grid image."""
    renderer = mujoco.Renderer(model, TILE_HEIGHT, TILE_WIDTH)
    tiles = []
    for azimuth, elevation in VIEWS.values():
        camera = mujoco.MjvCamera()
        camera.lookat[:] = lookat
        camera.distance = distance
        camera.azimuth = azimuth
        camera.elevation = elevation
        renderer.update_scene(data, camera, scene_option)
        tiles.append(renderer.render())
    renderer.close()

    rows = [np.concatenate(tiles[i : i + 2], axis=1) for i in range(0, len(tiles), 2)]
    return np.concatenate(rows, axis=0)


def main() -> None:
    args = build_parser().parse_args()

    walter = HardwareModel()
    walter.spec.visual.global_.offwidth = TILE_WIDTH
    walter.spec.visual.global_.offheight = TILE_HEIGHT
    model = walter.spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    scene_option = mujoco.MjvOption()
    if args.collision:
        scene_option.geomgroup[COLLISION_GROUP] = 1
        is_collision = model.geom_group == COLLISION_GROUP
        model.geom_rgba[is_collision] = COLLISION_RGBA
    if args.frames:
        scene_option.frame = mujoco.mjtFrame.mjFRAME_BODY
        model.vis.scale.framelength = FRAME_LENGTH_SCALE
        model.vis.scale.framewidth = FRAME_WIDTH_SCALE

    # Default focus is the whole robot's center of mass (subtree of the root body).
    if args.focus is None:
        lookat = data.subtree_com[1].copy()
    else:
        lookat = data.body(args.focus).xpos.copy()

    image = render_views(model, data, lookat, args.distance, scene_option)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(args.output)
    print(f"Wrote {args.output} (views: {', '.join(VIEWS)})")


if __name__ == "__main__":
    main()
