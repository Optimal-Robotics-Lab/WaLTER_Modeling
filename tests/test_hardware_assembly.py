"""Checks that the hardware kinematic tree matches the naming convention.

Front legs hang off the front torso and back legs off the back torso. A knee
hinge rotates its own shin and the wheels on that shin, and leaves the other
leg where it is.
"""

from __future__ import annotations

import mujoco
import numpy as np
import viser

from WaLTER_Modeling import HardwareModel
from WaLTER_Modeling.hardware.viser_scene import PerBodyMujocoScene


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
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id))
                or ""
                for geom_id in contact.geom
            )
        )
        assert contact.dist >= -1e-6, names
        assert not (
            "thigh" in names[0] and "wheel" in names[1]
        ), names


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


def test_each_body_is_its_own_viser_node() -> None:
    model = HardwareModel().mj_model
    server = viser.ViserServer(port=0)
    try:
        scene = PerBodyMujocoScene(server, model, num_envs=1)
        node_names = {group.handle.name for group in scene._mesh_groups}
    finally:
        server.stop()

    for body_name in (
        "back_torso",
        "br_thigh",
        "br_shin",
        "bl_shin",
        "bl_wheel1",
        "bl_wheel2",
        "fr_wheel2",
        "fl_shin",
    ):
        assert any(name.startswith(f"/bodies/{body_name}/") for name in node_names)

    fl_shin_nodes = {name for name in node_names if "/fl_shin/" in name}
    bl_shin_nodes = {name for name in node_names if "/bl_shin/" in name}
    assert fl_shin_nodes
    assert bl_shin_nodes
    assert fl_shin_nodes.isdisjoint(bl_shin_nodes)
