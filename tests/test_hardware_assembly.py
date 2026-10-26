"""Checks that the hardware kinematic tree matches the naming convention.

Front legs hang off the front torso and back legs off the back torso. A knee
hinge rotates its own shin and the wheels on that shin, and leaves the other
leg where it is. Torso meshes keep the waist pilots and hip pads on the joints.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import yaml

from WaLTER_Modeling import HardwareModel
from WaLTER_Modeling.hardware.generator import DEFAULT_CONFIG, MESH_DIR


def _body_id(model: mujoco.MjModel, name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert body_id >= 0, name
    return body_id


def _parent_name(model: mujoco.MjModel, name: str) -> str | None:
    parent_id = int(model.body_parentid[_body_id(model, name)])
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id)


def test_legs_attach_to_the_torso_half_they_belong_to() -> None:
    model = HardwareModel().mj_model

    assert _parent_name(model, "front_torso") == "back_torso"
    assert _parent_name(model, "fr_thigh") == "front_torso"
    assert _parent_name(model, "fl_thigh") == "front_torso"
    assert _parent_name(model, "br_thigh") == "back_torso"
    assert _parent_name(model, "bl_thigh") == "back_torso"

    for leg in ("fr", "fl", "br", "bl"):
        assert _parent_name(model, f"{leg}_shin") == f"{leg}_thigh"
        assert _parent_name(model, f"{leg}_wheel1") == f"{leg}_shin"
        assert _parent_name(model, f"{leg}_wheel2") == f"{leg}_shin"
        knee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_knee")
        assert model.jnt_bodyid[knee_id] == _body_id(model, f"{leg}_shin")


def test_visual_mass_is_optional_and_uses_exact_mesh_volume() -> None:
    """Authored inertials remain available. The default uses each link mass."""
    explicit = HardwareModel(inertia_from_visual=False)
    visual = HardwareModel()

    assert explicit.convex_mesh_files == []
    assert visual.convex_mesh_files == []
    legacy = mujoco.mjtMeshInertia.mjMESH_INERTIA_LEGACY
    exact = mujoco.mjtMeshInertia.mjMESH_INERTIA_EXACT
    assert all(int(mesh.inertia) == int(legacy) for mesh in explicit.spec.meshes)
    assert all(int(mesh.inertia) == int(exact) for mesh in visual.spec.meshes)

    # Authored inertials stay in place until the switch is turned on.
    explicit_model = explicit.mj_model
    assert explicit_model.body_mass[_body_id(explicit_model, "fl_thigh")] == 3.375
    assert explicit_model.body_mass[_body_id(explicit_model, "back_torso")] == 18.0

    expected_mass = {
        "back_torso": 18.79,
        "front_torso": 25.91,
        "fl_thigh": 5.25,
        "fr_thigh": 5.25,
        "fl_shin": 5.76,
        "fl_wheel1": 1.59,
        "fr_wheel2": 1.59,
    }
    for name, mass in expected_mass.items():
        body_id = _body_id(visual.mj_model, name)
        assert abs(float(visual.mj_model.body_mass[body_id]) - mass) < 1e-9

    # The mesh distribution is not the placeholder diagonal inertia.
    fl_thigh = _body_id(visual.mj_model, "fl_thigh")
    assert not np.allclose(
        visual.mj_model.body_inertia[fl_thigh], [0.01046, 0.01046, 0.0027]
    )


def _visual_vertices(mesh_name: str) -> np.ndarray:
    path = Path(MESH_DIR) / "visual" / f"{mesh_name}.obj"
    vertices = []
    with path.open() as mesh_file:
        for line in mesh_file:
            if line.startswith("v "):
                vertices.append([float(value) for value in line.split()[1:4]])
    return np.asarray(vertices, dtype=float)


def _quat_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Rotation matrix for a MuJoCo quaternion (w, x, y, z)."""
    w, x, y, z = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _hip_pad_centers(vertices: np.ndarray) -> np.ndarray:
    """Centers of the circular pads on the outer side faces, in the mesh frame.

    The pad axis is the mesh x axis. The outer face is the embossed surface
    the hip joint sits on.
    """
    on_outer_face = np.abs(np.abs(vertices[:, 0]) - vertices[:, 0].max()) < 1e-6
    centers = []
    for sign in (1.0, -1.0):
        side = vertices[on_outer_face & (np.sign(vertices[:, 0]) == sign)]
        center_yz = side[:, 1:].mean(axis=0)
        radius = np.linalg.norm(side[:, 1:] - center_yz, axis=1)
        assert radius.std() < 1e-5
        centers.append([side[:, 0].mean(), center_yz[0], center_yz[1]])
    return np.asarray(centers, dtype=float)


def test_torso_meshes_put_the_waist_and_hips_on_the_embossed_pads() -> None:
    """Each body-mesh origin is the waist axis, and each hip is a side-pad center."""
    robot = HardwareModel()
    config = robot.config
    # Mesh +z -> body +x, mesh +x -> body +y, mesh +y -> body +z.
    rotation = _quat_matrix(np.array([0.5, 0.5, 0.5, 0.5]))

    waist_in_back = []
    for body_name, mesh_name in (
        ("back_torso", "rearbody_vis_mesh"),
        ("front_torso", "frontbody_vis_mesh"),
    ):
        visual = config[body_name]["vis"]
        assert np.allclose(visual["quat"], [0.5, 0.5, 0.5, 0.5])
        mesh_origin_in_body = np.asarray(visual["pos"], dtype=float)
        if body_name == "front_torso":
            mesh_origin_in_body = mesh_origin_in_body + np.asarray(
                config["front_torso"]["pos"], dtype=float
            )
        waist_in_back.append(mesh_origin_in_body)

        vertices = _visual_vertices(mesh_name)
        # The waist pilot is the small circle centered on the mesh origin.
        radial = np.linalg.norm(vertices[:, :2], axis=1)
        pilot = vertices[(radial < 0.05) & (np.abs(vertices[:, 2]) < 1e-6)]
        assert len(pilot) > 10
        assert np.linalg.norm(pilot.mean(axis=0)) < 1e-6

        pad_centers = _hip_pad_centers(vertices)
        pads_in_body = mesh_origin_in_body + (rotation @ pad_centers.T).T
        if body_name == "front_torso":
            # pad positions above were shifted into the back frame; hips are
            # stored in the front frame, whose origin is the waist.
            pads_in_body = pads_in_body - np.asarray(
                config["front_torso"]["pos"], dtype=float
            )
            expected = np.asarray(config["thigh"]["pos_front"], dtype=float)
        else:
            expected = np.asarray(config["thigh"]["pos_back"], dtype=float)
        left_pad = pads_in_body[np.argmax(pads_in_body[:, 1])]
        assert np.allclose(left_pad, expected, atol=5e-5)
        thigh_name = "fl_thigh" if body_name == "front_torso" else "bl_thigh"
        thigh_id = _body_id(robot.mj_model, thigh_name)
        assert np.allclose(robot.mj_model.body_pos[thigh_id], left_pad, atol=5e-5)

    assert np.allclose(waist_in_back[0], waist_in_back[1], atol=1e-9)


def test_back_torso_has_its_own_geoms() -> None:
    model = HardwareModel().mj_model
    back_torso = _body_id(model, "back_torso")
    for geom_name in ("back_torso_col", "back_torso_vis"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        assert geom_id >= 0
        assert int(model.geom_bodyid[geom_id]) == back_torso


def test_assembled_pose_does_not_embed_the_wheels_in_the_thighs() -> None:
    """Thigh collision must clear the wheels, or the contact solver locks the knee."""
    model = HardwareModel().mj_model
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    for i in range(data.ncon):
        contact = data.contact[i]
        names = tuple(
            sorted(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
                for geom_id in contact.geom
            )
        )
        assert contact.dist >= -1e-6, names
        assert not ("thigh" in names[0] and "wheel" in names[1]), names


def _joint_frictionloss(model: mujoco.MjModel, name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    assert joint_id >= 0, name
    # Hinge joints have one degree of freedom. frictionloss lives on that dof.
    dof = int(model.jnt_dofadr[joint_id])
    return float(model.dof_frictionloss[dof])


def test_joint_frictionloss_comes_from_the_hardware_config(tmp_path: Path) -> None:
    """Each joint type takes its Coulomb torque, in N*m, from the hardware config."""
    robot = HardwareModel()
    _assert_configured_frictionloss(robot)

    config = yaml.safe_load(Path(DEFAULT_CONFIG).read_text())
    config["joints"]["waist"]["frictionloss"] = 0.4
    config["joints"]["hip"]["frictionloss"] = 1.5
    config["joints"]["knee"]["frictionloss"] = 2.5
    config["joints"]["axle"]["frictionloss"] = 0.05
    config_path = tmp_path / "hardware_config.yaml"
    config_path.write_text(yaml.safe_dump(config))

    updated = HardwareModel(config_path)
    _assert_configured_frictionloss(updated)
    # A config change must not leak onto joints that have no frictionloss entry.
    root_id = mujoco.mj_name2id(updated.mj_model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    root_dof = int(updated.mj_model.jnt_dofadr[root_id])
    assert np.allclose(updated.mj_model.dof_frictionloss[root_dof : root_dof + 6], 0.0)


def _assert_configured_frictionloss(robot: HardwareModel) -> None:
    model = robot.mj_model
    joints = robot.config["joints"]
    expected = {"waist": joints["waist"]["frictionloss"]}
    for leg in ("fr", "fl", "br", "bl"):
        expected[f"{leg}_hip"] = joints["hip"]["frictionloss"]
        expected[f"{leg}_knee"] = joints["knee"]["frictionloss"]
        expected[f"{leg}_axle1"] = joints["axle"]["frictionloss"]
        expected[f"{leg}_axle2"] = joints["axle"]["frictionloss"]
    for name, frictionloss in expected.items():
        assert abs(_joint_frictionloss(model, name) - frictionloss) < 1e-9


def test_knee_motor_flexes_the_shin() -> None:
    model = HardwareModel().mj_model
    data = mujoco.MjData(model)
    actuator_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "fl_knee_motor"
    )
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "fl_knee")
    qpos_adr = int(model.jnt_qposadr[joint_id])
    data.ctrl[actuator_id] = model.actuator_ctrlrange[actuator_id, 1]

    for _ in range(250):
        mujoco.mj_step(model, data)

    # Full stall torque should flex the shin well past a small twitch. A thigh
    # capsule embedded in the wheels holds this near zero.
    assert data.qpos[qpos_adr] > 0.5


def test_knee_rotates_only_its_own_shin() -> None:
    model = HardwareModel().mj_model
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    fl_shin = _body_id(model, "fl_shin")
    bl_shin = _body_id(model, "bl_shin")
    fl_wheel = _body_id(model, "fl_wheel1")
    shin_quat = data.xquat[fl_shin].copy()
    other_shin_quat = data.xquat[bl_shin].copy()
    wheel_pos = data.xpos[fl_wheel].copy()

    knee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "fl_knee")
    data.qpos[model.jnt_qposadr[knee_id]] = 0.8
    mujoco.mj_forward(model, data)

    assert not np.allclose(data.xquat[fl_shin], shin_quat)
    assert not np.allclose(data.xpos[fl_wheel], wheel_pos)
    assert np.allclose(data.xquat[bl_shin], other_shin_quat)
