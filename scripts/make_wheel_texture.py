"""Builds the wheel texture and bakes UVs into the visual mesh.

The wheel is one mesh, so one atlas has to carry both looks. The rounded
outside, out to where the tire face turns flat, samples a semi-offroad tread.
Only the shallow hub recess on the outboard face samples the gray metal disk.
MuJoCo uses the OBJ texture coordinates as-is (v up, matching the image).
The CAD file's original coordinates do not follow the tire, so this rewrites
them. Re-run from the repo root after a CAD re-export:

    uv run python scripts/make_wheel_texture.py
"""

from pathlib import Path

import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parents[1]
MESH_PATH = (
    REPO_ROOT
    / "WaLTER_Modeling"
    / "hardware"
    / "assets"
    / "meshes"
    / "visual"
    / "wheel_vis_mesh.obj"
)
TEXTURE_PATH = (
    REPO_ROOT / "WaLTER_Modeling" / "hardware" / "assets" / "textures" / "wheel.png"
)

TEX_W = 2048
TEX_H = 1024
# Top-left square. The image is 2:1, so this square is 0.25 in u and 0.50 in v.
FACE_SIZE = 512
# The recess lip is at about 0.073 m. A slightly larger radius keeps that lip
# on metal instead of the rubber corners of the square.
FACE_RADIUS = 0.078
# Faces inside this cylinder, on the outboard side, are the modeled hub pocket.
HUB_MAX_RADIUS = 0.0742
HUB_MIN_Z = 0.075
# Flat side faces share one rubber texel, off the hub square and the tread strip.
RUBBER_COL = 1500
RUBBER_ROW = 400
# One revolution of tread. Tall enough that shoulder blocks stay near square
# when the strip covers the whole rounded outside, not just the crown.
TREAD_H = 256
# Outward radial normal above this is still the round. The flat side is near 0.
TREAD_MIN_RADIAL = 0.45

RUBBER = np.array([18, 17, 16], dtype=np.float64)
GROOVE = np.array([6, 6, 7], dtype=np.float64)
BLOCK = np.array([62, 58, 52], dtype=np.float64)

# Mesh z of the rounded outside, inboard shoulder to outboard shoulder.
TREAD_Z = (0.0, 0.078)


def _v_of_row(row: float) -> float:
    """OBJ v (0 at the bottom of the image) for a pixel row measured from the top."""
    return 1.0 - row / TEX_H


def make_wheel_texture() -> np.ndarray:
    """Returns an RGB atlas: metal hub disk and a periodic tread strip."""
    image = np.empty((TEX_H, TEX_W, 3), dtype=np.float64)
    image[:] = RUBBER
    _draw_tread(image)
    _draw_hub(image)
    return np.clip(image, 0, 255).astype(np.uint8)


def _draw_tread(image: np.ndarray) -> None:
    """Staggered all-terrain blocks. The pattern repeats exactly around the tire."""
    height = TREAD_H
    row0 = TEX_H - height
    ys, xs = np.mgrid[0:height, 0:TEX_W]
    color = np.empty((height, TEX_W, 3), dtype=np.float64)
    color[:] = GROOVE

    n_rows = 4
    n_lugs = 14
    coverage = np.zeros((height, TEX_W), dtype=np.float64)
    highlight = np.zeros((height, TEX_W), dtype=np.float64)
    sipe = np.zeros((height, TEX_W), dtype=np.float64)

    for row in range(n_rows):
        shoulder = row in (0, n_rows - 1)
        duty = 0.70 if shoulder else 0.56
        stagger = 0.0 if row % 2 == 0 else 0.5
        pitch = TEX_W / n_lugs
        block_w = duty * pitch
        row_h = height / n_rows
        margin = 0.10 * row_h if shoulder else 0.16 * row_h
        y0 = row * row_h + margin
        y1 = (row + 1) * row_h - margin
        radius = 0.14 * (y1 - y0)
        for lug in range(n_lugs):
            x0 = (lug + stagger) * pitch + 0.08 * pitch
            x1 = x0 + block_w
            mask = _periodic_round_rect(xs, ys, x0, x1, y0, y1, radius)
            coverage = np.maximum(coverage, mask)
            # Short sipes across the block, kept off the rounded corners.
            local_x = np.mod(xs - x0, TEX_W)
            local_y = (ys - y0) / max(y1 - y0, 1.0)
            on_block = mask > 0.5
            # One sipe on the center rows. Shoulder lugs stay solid.
            if not shoulder:
                sipe = np.maximum(
                    sipe,
                    on_block
                    * (np.abs(local_x / block_w - 0.55) < 0.02)
                    * (local_y > 0.25)
                    * (local_y < 0.75),
                )
            highlight = np.maximum(
                highlight,
                on_block * np.clip(1.0 - local_x / (0.10 * block_w), 0.0, 1.0),
            )

    color[:] = GROOVE * (1.0 - coverage[..., None]) + BLOCK * coverage[..., None]
    color += highlight[..., None] * np.array([16.0, 15.0, 14.0])
    color[sipe > 0.5] = GROOVE + 2.0

    # A few pixels of rubber at the strip edge, so the flat side face does not
    # pick up a cut-off lug through texture filtering.
    fade = 6
    ramp = np.linspace(0.0, 1.0, fade)[:, None, None]
    color[:fade] = color[:fade] * ramp + RUBBER * (1.0 - ramp)
    color[-fade:] = color[-fade:] * ramp[::-1] + RUBBER * (1.0 - ramp[::-1])
    image[row0:, :, :] = color


def _periodic_round_rect(
    xs: np.ndarray,
    ys: np.ndarray,
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    radius: float,
) -> np.ndarray:
    """Antialiased rounded rectangle that wraps across x = 0 and x = width."""
    dx = np.abs(np.mod(xs - 0.5 * (x0 + x1) + TEX_W / 2.0, TEX_W) - TEX_W / 2.0)
    half_w = 0.5 * (x1 - x0)
    half_h = 0.5 * (y1 - y0)
    # Inset the box by the corner radius, then measure distance to that box.
    inset_x = max(half_w - radius, 0.0)
    inset_y = max(half_h - radius, 0.0)
    outside_x = np.maximum(dx - inset_x, 0.0)
    outside_y = np.maximum(np.abs(ys - 0.5 * (y0 + y1)) - inset_y, 0.0)
    dist = np.hypot(outside_x, outside_y)
    return np.clip(radius - dist + 0.75, 0.0, 1.0)


def _draw_hub(image: np.ndarray) -> None:
    """Brushed steel disk with a bolt circle. Corners of the square stay rubber."""
    size = FACE_SIZE
    ys, xs = np.mgrid[0:size, 0:size]
    center = (size - 1) / 2.0
    dx = xs - center
    dy = ys - center
    radius = np.hypot(dx, dy)
    angle = np.arctan2(dy, dx)
    limit = size / 2.0 - 1.0
    disk = radius <= limit

    # A few broad machining rings, not a radial grating.
    rings = 2.5 * np.sin(radius * 0.12)
    key = 18.0 * np.cos(angle - 2.2) * np.clip(radius / limit, 0.0, 1.0)
    color = np.array([158.0, 162.0, 168.0]) + rings[..., None] + key[..., None]
    hub = radius < limit * 0.30
    color[hub] = np.array([176.0, 180.0, 186.0]) + 0.35 * rings[hub, None]
    cap = radius < limit * 0.10
    color[cap] = np.array([96.0, 98.0, 104.0])
    # Darker steel at the lip, still gray, so it meets the black tire cleanly.
    rim = radius > limit * 0.92
    color[rim] = np.array([110.0, 114.0, 120.0])

    bolt_ring = limit * 0.58
    bolt_radius = limit * 0.075
    for index in range(6):
        bolt_angle = index * np.pi / 3.0 - np.pi / 2.0
        bx = center + np.cos(bolt_angle) * bolt_ring
        by = center + np.sin(bolt_angle) * bolt_ring
        _paint_bolt(color, xs, ys, bx, by, bolt_radius)

    face = image[:size, :size, :]
    face[:] = np.where(disk[..., None], color, RUBBER)


def _paint_bolt(
    color: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    cx: float,
    cy: float,
    radius: float,
) -> None:
    dx = xs - cx
    dy = ys - cy
    dist = np.hypot(dx, dy)
    angle = np.arctan2(dy, dx)
    sector = np.pi / 3.0
    # Distance to a hexagon's flat, used so the head isn't a plain circle.
    angled = np.mod(angle, sector) - sector / 2.0
    hex_limit = radius * np.cos(np.pi / 6.0) / np.cos(angled)
    head = dist <= hex_limit
    color[head] = np.array([64.0, 66.0, 70.0])
    rim = head & (dist > hex_limit * 0.78)
    color[rim] = np.array([92.0, 96.0, 102.0])
    shine = head & (dx < -0.15 * radius) & (dist < hex_limit * 0.55)
    color[shine] = np.array([168.0, 172.0, 178.0])


def _face_uv(points: np.ndarray) -> np.ndarray:
    """Maps hub-plane x, y into the metal square. +y is toward the top of the image."""
    u_center = 0.5 * FACE_SIZE / TEX_W
    v_center = 1.0 - 0.5 * FACE_SIZE / TEX_H
    half_u = u_center
    half_v = 0.5 * FACE_SIZE / TEX_H
    u = u_center + half_u * (points[:, 0] / FACE_RADIUS)
    v = v_center + half_v * (points[:, 1] / FACE_RADIUS)
    return np.column_stack((u, v))


def _tread_uv(points: np.ndarray) -> np.ndarray:
    z0, z1 = TREAD_Z
    z = np.clip(points[:, 2], z0, z1)
    v = (z - z0) / (z1 - z0) * (TREAD_H / TEX_H)
    return np.column_stack((_circumferential_u(points), v))


def _rubber_uv(count: int) -> np.ndarray:
    """One texel. The flat tire face should not pick up tread or the hub."""
    uv = np.array(
        [RUBBER_COL / TEX_W, _v_of_row(RUBBER_ROW)],
        dtype=np.float64,
    )
    return np.repeat(uv[None, :], count, axis=0)


def _circumferential_u(points: np.ndarray) -> np.ndarray:
    """Angle around mesh z, in 0-1. Faces that cross the seam stay continuous."""
    angle = np.arctan2(points[:, 1], points[:, 0])
    u = (angle + np.pi) / (2.0 * np.pi)
    if u.max() - u.min() > 0.5:
        u = np.where(u < 0.5, u + 1.0, u)
    return u


def _region(points: np.ndarray, normal: np.ndarray) -> str:
    """Splits the mesh into the hub pocket, the rounded tread, and flat rubber.

    The hub is the shallow outboard recess, not the whole side of the wheel.
    Tread is every face whose normal is still outward and radial, which is the
    round outside up to where the side face becomes flat.
    """
    vertex_radius = np.hypot(points[:, 0], points[:, 1])
    in_pocket = float(points[:, 2].min()) > HUB_MIN_Z
    if in_pocket and float(vertex_radius.max()) <= HUB_MAX_RADIUS:
        return "hub"
    center = points.mean(axis=0)
    radius = float(np.hypot(center[0], center[1]))
    radial = center[:2] / max(radius, 1e-12)
    normal_radial = float(np.dot(normal[:2], radial))
    if normal_radial > TREAD_MIN_RADIAL:
        return "tread"
    return "rubber"


def _corner_uvs(points: np.ndarray, region: str) -> np.ndarray:
    if region == "hub":
        return _face_uv(points)
    if region == "tread":
        return _tread_uv(points)
    return _rubber_uv(len(points))


def bake_wheel_uvs(vertices: np.ndarray, mesh_path: Path) -> dict[str, int]:
    """Rewrites texture coordinates in ``mesh_path``. Geometry and normals stay put."""
    v_lines: list[str] = []
    vn_lines: list[str] = []
    # usemtl lines stay in front of the faces they apply to.
    stream: list[tuple[str, object]] = []
    with mesh_path.open() as mesh_file:
        for line in mesh_file:
            if line.startswith("v "):
                v_lines.append(line if line.endswith("\n") else line + "\n")
            elif line.startswith("vn "):
                vn_lines.append(line if line.endswith("\n") else line + "\n")
            elif line.startswith("usemtl "):
                stream.append(("usemtl", line if line.endswith("\n") else line + "\n"))
            elif line.startswith("f "):
                corners: list[tuple[str, str]] = []
                for token in line.split()[1:]:
                    parts = token.split("/")
                    if len(parts) != 3 or not parts[0] or not parts[2]:
                        raise ValueError(
                            f"Expected v/vt/vn corners, received '{token}'."
                        )
                    corners.append((parts[0], parts[2]))
                stream.append(("f", corners))

    if len(v_lines) != len(vertices):
        raise ValueError(
            f"Parsed {len(v_lines)} vertex lines but {len(vertices)} positions."
        )

    counts = {"hub": 0, "tread": 0, "rubber": 0}
    uv_lines: list[str] = []
    face_lines: list[str] = []
    for kind, payload in stream:
        if kind == "usemtl":
            face_lines.append(str(payload))
            continue
        corners = payload
        assert isinstance(corners, list)
        indices = [int(vertex_index) - 1 for vertex_index, _ in corners]
        points = vertices[indices]
        edge_a = points[1] - points[0]
        edge_b = points[2] - points[0]
        normal = np.cross(edge_a, edge_b)
        length = float(np.linalg.norm(normal))
        if length < 1e-12:
            normal = np.array([0.0, 0.0, 1.0])
        else:
            normal = normal / length
        region = _region(points, normal)
        counts[region] += 1
        uvs = _corner_uvs(points, region)
        tokens: list[str] = []
        for (vertex_index, normal_index), uv in zip(corners, uvs, strict=True):
            uv_lines.append(f"vt {uv[0]:.6f} {uv[1]:.6f}\n")
            tokens.append(f"{vertex_index}/{len(uv_lines)}/{normal_index}")
        face_lines.append("f " + " ".join(tokens) + "\n")

    header = (
        "# WaveFront *.obj file (generated by Autodesk ATF)\n"
        "# Texture coordinates written by scripts/make_wheel_texture.py\n"
        "# for assets/textures/wheel.png. Vertex positions are unchanged.\n"
        "\n"
        "mtllib Wheel.mtl\n"
        "\n"
        "g Body1\n"
        "\n"
    )
    footer = (
        f"# {len(v_lines)} vertices\n"
        f"# {len(uv_lines)} texture params\n"
        f"# {len(vn_lines)} normals\n"
        f"# {sum(counts.values())} facets\n"
        "\n"
        "# 1 groups\n"
    )
    body = (
        header
        + "".join(v_lines)
        + "".join(uv_lines)
        + "".join(vn_lines)
        + "".join(face_lines)
        + footer
    )
    mesh_path.write_text(body)
    return counts


def _load_vertices(mesh_path: Path) -> np.ndarray:
    vertices: list[list[float]] = []
    with mesh_path.open() as mesh_file:
        for line in mesh_file:
            if line.startswith("v "):
                vertices.append([float(value) for value in line.split()[1:4]])
    return np.asarray(vertices, dtype=np.float64)


def main() -> None:
    image = make_wheel_texture()
    TEXTURE_PATH.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(TEXTURE_PATH)
    counts = bake_wheel_uvs(_load_vertices(MESH_PATH), MESH_PATH)
    print(f"Wrote {TEXTURE_PATH.relative_to(REPO_ROOT)}")
    print(f"Updated UVs in {MESH_PATH.relative_to(REPO_ROOT)}: {counts}")


if __name__ == "__main__":
    main()
