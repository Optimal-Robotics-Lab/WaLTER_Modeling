# WaLTER naming convention

This is the default naming convention for all models in this repo. New models must follow it. The hardware model (`WaLTER_Modeling/hardware`) is the reference implementation. The parametric model still uses its older names and will be migrated.

The names are defined as constants and regex patterns in [`WaLTER_Modeling/naming.py`](../WaLTER_Modeling/naming.py). Generators should build names with those helpers instead of typing them out.

## General rules

- All names are lowercase snake_case.
- Leg prefixes:
  - `fr`: front right
  - `fl`: front left
  - `br`: back right
  - `bl`: back left

  Front means the robot's +x direction. `{leg}` below stands for any of these four.
- Joints and bodies use different words, so a name alone tells you which kind of element it is. A `wheel` is always a body and an `axle` is always a joint.
- Every other element type has a fixed suffix: `_col`, `_vis`, `_motor`, `_site`, or `_mesh`.

## Bodies

| Name | Description |
| --- | --- |
| `back_torso` | Rear half of the chassis. Root body. The `br` and `bl` legs attach here. |
| `front_torso` | Front half of the chassis. The `fr` and `fl` legs attach here. |
| `{leg}_thigh` | Thigh link. |
| `{leg}_shin` | Shin link. |
| `{leg}_wheel1` | Wheel at the +x end of the shin, with the robot in its zero pose. |
| `{leg}_wheel2` | Wheel at the -x end of the shin, with the robot in its zero pose. |

The wheel numbers belong to the physical wheels and don't change as the knee rotates. They're numbered rather than called "front" and "rear" to avoid confusion with the leg prefixes.

Each body's frame sits at the joint that connects it to its parent.

## Joints

Joints are named for what they do and have no suffix.

| Name | Connects |
| --- | --- |
| `root` | Free joint on `back_torso` |
| `waist` | `back_torso` to `front_torso` |
| `{leg}_hip` | torso half to `{leg}_thigh` |
| `{leg}_knee` | `{leg}_thigh` to `{leg}_shin` |
| `{leg}_axle1` | `{leg}_shin` to `{leg}_wheel1` |
| `{leg}_axle2` | `{leg}_shin` to `{leg}_wheel2` |

Examples: `fr_hip`, `bl_knee`, `fl_axle2`.

## Geoms

Visual and contact geometry are always separate geoms.

| Pattern | Kind | Default class | Settings |
| --- | --- | --- | --- |
| `{body}_col` | Contact (collision) | `collision` | group 3, `contype=1`, `conaffinity=1`, `density=0` |
| `{body}_vis` | Visual only | `visual` | group 2, `contype=0`, `conaffinity=0`, `density=0` |

- When a body has more than one geom of the same kind, add an index or a short descriptor: `back_torso_col_0`, `back_torso_vis_battery`.
- Geoms never add mass. Mass and inertia are set explicitly on each body.
- Contact geoms should be primitives (box, capsule, cylinder, sphere, ellipsoid) wherever possible. Use convex mesh pieces only when primitives can't approximate the shape.

## Other elements

| Element | Pattern | Examples |
| --- | --- | --- |
| Actuator | `{joint}_motor` | `fr_hip_motor`, `bl_axle1_motor` |
| Site | `{descriptor}_site` | `imu_site`, `fr_wheel1_contact_site` |
| Sensor | `{joint or site}_{quantity}` | `fr_knee_pos`, `fr_knee_vel`, `fr_knee_torque`, `imu_gyro`, `imu_accel` |
| Mesh asset | `{part}_vis_mesh` or `{part}_col_mesh` | `thigh_vis_mesh`, `back_torso_col_mesh` |

- Mesh files have the same name as the asset (`thigh_vis_mesh.obj`). Visual meshes go in `assets/meshes/visual/` and collision meshes in `assets/meshes/collision/`.
- Meshes are modeled for the left legs. Right legs automatically get a mirrored copy of the left mesh, named with `_right` after the part: `thigh_right_vis_mesh`. If a right-side part really is a different shape, add its own file with that same name and set it as `mesh_right` in the config.
- The `waist` joint is passive, so it has no motor.

## Mesh files

- Use OBJ for visual meshes. OBJ keeps normals and texture coordinates and is smaller than STL. If you must use STL, export it as binary, because MuJoCo rejects ASCII STL.
- MuJoCo collides against the convex hull of a mesh, so detail in a collision mesh is wasted. If a part needs a non-convex contact shape, split it into low-poly convex pieces with a tool such as CoACD or V-HACD.
- Export in meters, with each part in its own body frame and the origin at that body's parent joint.
