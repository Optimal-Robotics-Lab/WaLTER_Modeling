"""Builds the high-fidelity full-scale WaLTER model with MjSpec.

Every value comes from ``configs/hardware_config.yaml``, and every element name
comes from ``WaLTER_Modeling.naming``. Each body frame sits at the joint that
connects it to its parent, which matches how CAD meshes are exported.
"""

from pathlib import Path

import mujoco
import numpy as np
import yaml
from numpy.typing import ArrayLike

from .. import naming

PACKAGE_DIR = Path(__file__).parent
DEFAULT_CONFIG = PACKAGE_DIR / "configs" / "hardware_config.yaml"
MESH_DIR = PACKAGE_DIR / "assets" / "meshes"
MESH_SUBDIRS = {naming.VIS: "visual", naming.COL: "collision"}
TEXTURE_DIR = PACKAGE_DIR / "assets" / "textures"
# One atlas for every wheel. UVs in wheel_vis_mesh.obj put the tread on the
# rolling surface and the gray hub on the faces. See scripts/make_wheel_texture.py.
WHEEL_VIS_MESH = "wheel_vis_mesh"
WHEEL_TEXTURE = "wheel.png"
WHEEL_MATERIAL = "wheel"

# MuJoCo viewer groups 0-2 are visible by default, so contact geometry in group 3
# stays hidden unless the user toggles it on.
VISUAL_GROUP = 2
COLLISION_GROUP = 3
DEFAULT_RGBA = [177 / 255, 166 / 255, 136 / 255, 1]

GEOM_TYPES = {
    "box": mujoco.mjtGeom.mjGEOM_BOX,
    "capsule": mujoco.mjtGeom.mjGEOM_CAPSULE,
    "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER,
    "sphere": mujoco.mjtGeom.mjGEOM_SPHERE,
    "ellipsoid": mujoco.mjtGeom.mjGEOM_ELLIPSOID,
    "mesh": mujoco.mjtGeom.mjGEOM_MESH,
}
MAX_GEOM_SIZE_ENTRIES = 3

# Right legs are the left legs reflected across the robot's xz-plane.
POSITION_MIRROR_Y = np.array([1.0, -1.0, 1.0])
# Reflecting a rotation across the xz-plane maps quaternion (w, x, y, z)
# to (w, -x, y, -z).
QUATERNION_MIRROR_Y = np.array([1.0, -1.0, 1.0, -1.0])
# Mirroring a placed mesh needs the mirrored pose above *and* the mesh itself
# reflected in its own frame: M (p + R v) = M p + (M R M)(M v), with M = diag(1, -1, 1).
MESH_MIRROR_SCALE = [1.0, -1.0, 1.0]


def mirror_position(position: ArrayLike, mirrored: bool) -> np.ndarray:
    position = np.asarray(position, dtype=float)
    return position * POSITION_MIRROR_Y if mirrored else position


def mirror_quaternion(quaternion: ArrayLike, mirrored: bool) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=float)
    return quaternion * QUATERNION_MIRROR_Y if mirrored else quaternion


class HardwareModel:
    """High-fidelity full-scale WaLTER model built from the hardware config.

    Exposes the same attributes as ``ParametricModel``: ``spec`` (editable
    ``MjSpec``, for example to add a floor before recompiling), ``mj_model``
    (compiled model), and ``model_xml`` (MJCF string).
    """

    def __init__(self, config_path: str | Path = DEFAULT_CONFIG, waist_locked: bool = False) -> None:
        self.config = yaml.safe_load(Path(config_path).read_text())

        self.spec = mujoco.MjSpec()
        # Absolute asset directories let the exported XML load from any folder,
        # at the cost of tying it to this machine's checkout location.
        self.spec.meshdir = str(MESH_DIR)
        self.spec.texturedir = str(TEXTURE_DIR)
        self._added_meshes: set[str] = set()
        self._add_wheel_material()

        # Visual and contact geometry get separate default classes so visual meshes
        # never collide and no geom ever contributes mass; bodies set mass explicitly.
        self.visual_class = self.spec.add_default("visual", self.spec.default)
        self.visual_class.geom.group = VISUAL_GROUP
        self.visual_class.geom.contype = 0
        self.visual_class.geom.conaffinity = 0
        self.visual_class.geom.density = 0
        self.visual_class.geom.rgba = DEFAULT_RGBA

        self.collision_class = self.spec.add_default("collision", self.spec.default)
        self.collision_class.geom.group = COLLISION_GROUP
        self.collision_class.geom.contype = 1
        self.collision_class.geom.conaffinity = 1
        self.collision_class.geom.density = 0

        self.waist_locked = waist_locked

        self._build_robot()

        self.mj_model = self.spec.compile()
        self.model_xml = self.spec.to_xml()

    def _add_wheel_material(self) -> None:
        """RGB texture shared by every wheel visual. Nothing here affects contact."""
        texture_path = TEXTURE_DIR / WHEEL_TEXTURE
        if not texture_path.is_file():
            raise FileNotFoundError(f"Wheel texture not found at {texture_path}.")
        self.spec.add_texture(
            name="wheel_texture",
            type=mujoco.mjtTexture.mjTEXTURE_2D,
            file=WHEEL_TEXTURE,
        )
        material = self.spec.add_material(
            name=WHEEL_MATERIAL,
            rgba=[1.0, 1.0, 1.0, 1.0],
            # Low specular so the rubber stays matte. The hub reads as metal
            # from the texture itself, not from a shiny material on the tire.
            specular=0.12,
            shininess=0.35,
            reflectance=0.0,
        )
        material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "wheel_texture"

    def _build_robot(self) -> None:
        config = self.config
        waist_config = config["joints"]["waist"]
        

        # Root: the back torso carries the free joint, so it defines the base frame.
        back_torso = self.spec.worldbody.add_body(
            name=naming.BACK_TORSO,
            pos=config["start_pos"],
        )
        back_torso.add_freejoint(name=naming.ROOT)
        self._add_body_contents(back_torso, config["back_torso"], mirrored=False)

        # Waist: a passive spring-damper hinge about the robot's long (x) axis.
        front_torso = back_torso.add_body(
            name=naming.FRONT_TORSO,
            pos=config["front_torso"]["pos"],
        )
        if not self.waist_locked:
            front_torso.add_joint(
                type=mujoco.mjtJoint.mjJNT_HINGE,
                name=naming.WAIST,
                axis=waist_config["axis"],
                range=waist_config["range"],
                stiffness=waist_config.get("stiffness", 0),
                damping=waist_config.get("damping", 0),
            )

        self._add_body_contents(front_torso, config["front_torso"], mirrored=False)

        # Legs: front legs hang off the front torso and back legs off the back torso,
        # so the waist joint couples the two leg pairs.
        for leg in naming.LEGS:
            torso = front_torso if leg in naming.FRONT_LEGS else back_torso
            self._add_leg(torso, leg)

    def _add_leg(self, torso: mujoco.MjsBody, leg: str) -> None:
        """Adds the thigh, shin, and two wheels of one leg as a serial chain."""
        config = self.config
        mirrored = leg in naming.RIGHT_LEGS
        hip_key = "pos_front" if leg in naming.FRONT_LEGS else "pos_back"

        thigh = self._add_actuated_link(
            parent=torso,
            leg=leg,
            part="thigh",
            joint="hip",
            joint_type="hip",
            joint_position=config["thigh"][hip_key],
            body_config=config["thigh"],
            mirrored=mirrored,
        )
        shin = self._add_actuated_link(
            parent=thigh,
            leg=leg,
            part="shin",
            joint="knee",
            joint_type="knee",
            joint_position=config["shin"]["pos"],
            body_config=config["shin"],
            mirrored=mirrored,
        )
        # Both wheels share one wheel definition and the "axle" joint and motor
        # parameters; only their mounting points on the shin differ.
        for wheel_index in (1, 2):
            self._add_actuated_link(
                parent=shin,
                leg=leg,
                part=f"wheel{wheel_index}",
                joint=f"axle{wheel_index}",
                joint_type="axle",
                joint_position=config["wheel"][f"pos_axle{wheel_index}"],
                body_config=config["wheel"],
                mirrored=mirrored,
            )

    def _add_actuated_link(
        self,
        parent: mujoco.MjsBody,
        leg: str,
        part: str,
        joint: str,
        joint_type: str,
        joint_position: ArrayLike,
        body_config: dict,
        mirrored: bool,
    ) -> mujoco.MjsBody:
        """Adds a body, its motor-driven hinge, and the matching actuator.

        ``joint_type`` selects the shared entries under ``joints`` and ``motors``
        in the config (hip, knee, or axle).
        """
        joint_config = self.config["joints"][joint_type]
        motor_config = self.config["motors"][joint_type]
        joint_name = naming.leg_joint(leg, joint)
        gear_ratio = motor_config["gear_ratio"]

        # Body frame at the joint, so meshes exported in the part frame need no offset.
        body = parent.add_body(
            name=naming.leg_body(leg, part),
            pos=mirror_position(joint_position, mirrored),
        )

        # Reflected rotor inertia: the rotor spins gear_ratio times faster than the
        # joint, so its inertia appears at the joint scaled by gear_ratio^2.
        hinge = body.add_joint(
            type=mujoco.mjtJoint.mjJNT_HINGE,
            name=joint_name,
            axis=joint_config["axis"],
            damping=joint_config.get("damping", 0),
            armature=motor_config["rotor_inertia"] * gear_ratio**2,
        )
        if "range" in joint_config:
            hinge.range = joint_config["range"]

        self._add_body_contents(body, body_config, mirrored)

        # The control signal is motor-side torque; the gear ratio maps it to joint
        # torque, so the joint torque limit is stall_torque * gear_ratio.
        stall_torque = motor_config["stall_torque"]
        actuator = self.spec.add_actuator(
            name=naming.motor(joint_name),
            target=joint_name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
        )
        actuator.gear = [gear_ratio, 0, 0, 0, 0, 0]
        actuator.ctrlrange = [-stall_torque, stall_torque]
        return body

    def _add_body_contents(
        self,
        body: mujoco.MjsBody,
        body_config: dict,
        mirrored: bool,
    ) -> None:
        """Adds the explicit inertial, collision geoms, and visual geoms to a body."""
        # Inertial: set explicitly from CAD or measurements, never inferred from geoms.
        inertial = body_config["inertial"]
        body.explicitinertial = True
        body.mass = inertial["mass"]
        body.ipos = mirror_position(inertial["pos"], mirrored)
        body.inertia = inertial["diaginertia"]

        # Collision geoms: indexed only when a body has more than one.
        collision_configs = body_config["col"]
        has_multiple = len(collision_configs) > 1
        for index, geom_config in enumerate(collision_configs):
            name = naming.geom(body.name, naming.COL, index if has_multiple else None)
            self._add_geom(body, self.collision_class, name, geom_config, mirrored)

        # Visual geoms: use the CAD mesh when one exists; otherwise draw copies of
        # the collision primitives so the model is still visible in the viewer.
        visual_config = body_config.get("vis")
        if visual_config:
            mesh_name = self._add_mesh_asset(
                left_mesh=visual_config["mesh"],
                kind=naming.VIS,
                mirrored=mirrored,
                right_mesh=visual_config.get("mesh_right"),
            )
            geom = body.add_geom(
                self.visual_class,
                name=naming.geom(body.name, naming.VIS),
                type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname=mesh_name,
                pos=mirror_position(visual_config.get("pos", [0, 0, 0]), mirrored),
                quat=mirror_quaternion(
                    visual_config.get("quat", [1, 0, 0, 0]), mirrored
                ),
            )
            # The visual class color would tint the texture. White leaves the
            # atlas unchanged on the wheel and does not touch the other meshes.
            if visual_config["mesh"] == WHEEL_VIS_MESH:
                geom.material = WHEEL_MATERIAL
                geom.rgba = [1.0, 1.0, 1.0, 1.0]
            return

        for index, geom_config in enumerate(collision_configs):
            name = naming.geom(body.name, naming.VIS, index if has_multiple else None)
            self._add_geom(body, self.visual_class, name, geom_config, mirrored)

    def _add_geom(
        self,
        body: mujoco.MjsBody,
        default_class: mujoco.MjsDefault,
        name: str,
        geom_config: dict,
        mirrored: bool,
    ) -> mujoco.MjsGeom:
        geom_type = geom_config["type"]
        if geom_type not in GEOM_TYPES:
            raise ValueError(
                f"Geom '{name}' has type '{geom_type}'; "
                f"expected one of {sorted(GEOM_TYPES)}."
            )

        geom = body.add_geom(
            default_class,
            name=name,
            type=GEOM_TYPES[geom_type],
            pos=mirror_position(geom_config.get("pos", [0, 0, 0]), mirrored),
            quat=mirror_quaternion(geom_config.get("quat", [1, 0, 0, 0]), mirrored),
        )

        if geom_type == "mesh":
            geom.meshname = self._add_mesh_asset(
                left_mesh=geom_config["mesh"],
                kind=naming.COL,
                mirrored=mirrored,
                right_mesh=geom_config.get("mesh_right"),
            )
        else:
            # Primitive sizes vary in length (sphere 1, capsule/cylinder 2, box 3),
            # but MuJoCo always stores three entries.
            size = list(geom_config["size"])
            if not 1 <= len(size) <= MAX_GEOM_SIZE_ENTRIES:
                raise ValueError(
                    f"Geom '{name}' size must have 1 to 3 entries, received {size}."
                )
            geom.size = size + [0] * (MAX_GEOM_SIZE_ENTRIES - len(size))

        # Contact parameters only mean something on geoms that collide.
        if default_class is self.collision_class:
            for contact_key in ("friction", "solref"):
                if contact_key in geom_config:
                    setattr(geom, contact_key, geom_config[contact_key])
        return geom

    def _add_mesh_asset(
        self,
        left_mesh: str,
        kind: str,
        mirrored: bool,
        right_mesh: str | None = None,
    ) -> str:
        """Registers a mesh asset once per model and returns its asset name.

        Meshes are modeled for the left legs. Right-side bodies use ``right_mesh``
        when the right part is modeled separately; otherwise they get a mirrored
        copy of the left mesh named ``{part}_right_{kind}_mesh``. Several geoms
        may share one asset (for example all four thighs), but MuJoCo rejects
        duplicate asset names.
        """
        mirror_scale = None
        if not mirrored:
            file_stem = asset_name = left_mesh
        elif right_mesh is not None:
            file_stem = asset_name = right_mesh
        else:
            file_stem = left_mesh
            asset_name = naming.right_mesh(left_mesh)
            mirror_scale = MESH_MIRROR_SCALE

        if asset_name in self._added_meshes:
            return asset_name

        relative_file = Path(MESH_SUBDIRS[kind]) / f"{file_stem}.obj"
        if not (MESH_DIR / relative_file).is_file():
            raise FileNotFoundError(
                f"Mesh '{file_stem}' not found at {MESH_DIR / relative_file}."
            )
        mesh_asset = self.spec.add_mesh(name=asset_name, file=relative_file.as_posix())
        if mirror_scale is not None:
            mesh_asset.scale = mirror_scale
        self._added_meshes.add(asset_name)
        return asset_name
