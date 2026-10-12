"""Viser scene that keeps a separate node for every MuJoCo body.

mjviser draws bodies that share identical local geometry with one batched
node, and names that node after the first body. The hardware model reuses
one mesh for both left shins, both right shins, and the two wheels on a
shin, so that merge drops ``br_*``, ``bl_*``, and ``wheel2`` from the scene
tree and makes one visibility toggle hide the matching part on the other
leg. This subclass does not merge across bodies.
"""

from __future__ import annotations

import numpy as np
from mjviser.conversions import (
    get_body_name,
    group_geoms_by_visual_compat,
    is_fixed_body,
    merge_geoms,
)
from mjviser.scene import ViserMujocoScene, _MeshGroup


class PerBodyMujocoScene(ViserMujocoScene):
    """Viser scene with one node per body, even when meshes match."""

    def _create_mesh_handles_by_group(self) -> None:
        # Group geoms by body so a shared mesh still gets its own scene node.
        body_group_geoms: dict[tuple[int, int], list[int]] = {}
        for geom_id in range(self.mj_model.ngeom):
            body_id = int(self.mj_model.geom_bodyid[geom_id])
            if is_fixed_body(self.mj_model, body_id):
                continue
            if self.mj_model.geom_rgba[geom_id, 3] == 0:
                continue
            geom_group = int(self.mj_model.geom_group[geom_id])
            body_group_geoms.setdefault((body_id, geom_group), []).append(geom_id)

        with self.server.atomic():
            for (body_id, group_id), geom_indices in body_group_geoms.items():
                subgroups = group_geoms_by_visual_compat(self.mj_model, geom_indices)
                is_mocap = bool(self.mj_model.body_mocapid[body_id] >= 0)
                body_name = get_body_name(self.mj_model, body_id)
                for sub_idx, sub_geom_ids in enumerate(subgroups):
                    self._add_body_mesh_group(
                        body_id=body_id,
                        body_name=body_name,
                        group_id=group_id,
                        sub_idx=sub_idx,
                        sub_geom_ids=sub_geom_ids,
                        is_mocap=is_mocap,
                    )

    def _add_body_mesh_group(
        self,
        body_id: int,
        body_name: str,
        group_id: int,
        sub_idx: int,
        sub_geom_ids: list[int],
        is_mocap: bool,
    ) -> None:
        """Adds one batched handle for a single body's geom subgroup."""
        mesh = merge_geoms(self.mj_model, sub_geom_ids)
        # Coarse meshes skip level-of-detail; dense CAD meshes get one step.
        lod_ratio = 1000.0 / mesh.vertices.shape[0]
        suffix = f"/sub{sub_idx}" if sub_idx > 0 else ""
        visible = group_id < 6 and self.geom_groups_visible[group_id]
        handle = self.server.scene.add_batched_meshes_trimesh(
            f"/bodies/{body_name}/group{group_id}{suffix}",
            mesh,
            batched_wxyzs=np.tile([1.0, 0.0, 0.0, 0.0], (self.num_envs, 1)),
            batched_positions=np.zeros((self.num_envs, 3)),
            lod=((2.0, lod_ratio),) if lod_ratio < 0.5 else "off",
            visible=visible,
        )
        mocap_ids = None
        if is_mocap:
            mocap_ids = np.array([self.mj_model.body_mocapid[body_id]], dtype=np.int32)
        self._mesh_groups.append(
            _MeshGroup(
                handle,
                np.array([body_id], dtype=np.int32),
                group_id,
                mocap_ids,
            )
        )
