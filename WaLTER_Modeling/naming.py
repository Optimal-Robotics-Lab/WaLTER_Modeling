"""Default WaLTER naming convention for MuJoCo model elements.

Generators build every element name with these constants and helpers, never by hand,
so a convention change only happens here. The regex ``PATTERNS`` let tests and
tools check that a compiled model conforms. See ``docs/naming_convention.md``.
"""

import re

# Legs: front/back by robot +x, right/left by robot -y/+y.
LEGS = ("fr", "fl", "br", "bl")
FRONT_LEGS = ("fr", "fl")
BACK_LEGS = ("br", "bl")
RIGHT_LEGS = ("fr", "br")
LEFT_LEGS = ("fl", "bl")

# Bodies and joints deliberately use different words (wheel vs. axle, thigh vs. hip),
# so an element's kind is clear from its name alone.
BACK_TORSO = "back_torso"
FRONT_TORSO = "front_torso"
LEG_BODIES = ("thigh", "shin", "wheel1", "wheel2")

ROOT = "root"
WAIST = "waist"
LEG_JOINTS = ("hip", "knee", "axle1", "axle2")

# Geom kinds: contact geometry and visual-only geometry are always separate geoms.
COL = "col"
VIS = "vis"


def leg_body(leg: str, part: str) -> str:
    return f"{leg}_{part}"


def leg_joint(leg: str, joint: str) -> str:
    return f"{leg}_{joint}"


def geom(body: str, kind: str, index: int | str | None = None) -> str:
    """Geom name for ``kind`` (``COL`` or ``VIS``).

    ``index`` distinguishes multiple geoms of the same kind on one body.
    """
    return f"{body}_{kind}" if index is None else f"{body}_{kind}_{index}"


def motor(joint: str) -> str:
    return f"{joint}_motor"


def site(descriptor: str) -> str:
    return f"{descriptor}_site"


def mesh(part: str, kind: str) -> str:
    return f"{part}_{kind}_mesh"


def right_mesh(left_mesh: str) -> str:
    """Name of the right-side (mirrored) copy of a left-side mesh asset.

    For example ``thigh_vis_mesh`` becomes ``thigh_right_vis_mesh``.
    """
    if not PATTERNS["mesh"].match(left_mesh):
        raise ValueError(
            f"Mesh name '{left_mesh}' does not match the "
            "'{part}_{vis|col}_mesh' convention."
        )
    part, kind, _ = left_mesh.rsplit("_", 2)
    return mesh(f"{part}_right", kind)


# Validation patterns, built from the constants above so they can't drift from them.
_LEG_PATTERN = "(?:" + "|".join(LEGS) + ")"
_BODY_PATTERN = (
    rf"(?:{BACK_TORSO}|{FRONT_TORSO}|{_LEG_PATTERN}_(?:{'|'.join(LEG_BODIES)}))"
)
_JOINT_PATTERN = rf"(?:{ROOT}|{WAIST}|{_LEG_PATTERN}_(?:{'|'.join(LEG_JOINTS)}))"

PATTERNS = {
    "body": re.compile(rf"^{_BODY_PATTERN}$"),
    "joint": re.compile(rf"^{_JOINT_PATTERN}$"),
    "geom": re.compile(rf"^{_BODY_PATTERN}_(?:{COL}|{VIS})(?:_[a-z0-9]+)?$"),
    "actuator": re.compile(rf"^{_JOINT_PATTERN}_motor$"),
    "site": re.compile(r"^[a-z0-9_]+_site$"),
    "mesh": re.compile(rf"^[a-z0-9_]+_(?:{COL}|{VIS})_mesh$"),
}
