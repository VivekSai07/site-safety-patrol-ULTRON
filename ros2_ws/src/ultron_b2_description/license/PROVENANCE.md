# B2 description provenance

`urdf/b2.urdf.xacro` is a hand-written primitive-geometry URDF whose kinematic
values (leg joint origins, axes, limits, link masses and inertias) are
transcribed from Unitree Robotics' official B2 MJCF:

- Source: https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/b2/b2.xml
- Git blob SHA: `88e1a789b67e90adbb120237b03a11cb54b1b210` (copy in the team repo at
  `src/ultron/assets/b2/upstream.xml`, fetched 27 Nov 2026).
- Licence: BSD-3-Clause, retained in `UNITREE_B2_LICENSE`.

No meshes are copied. Visual/collision geometry uses the MJCF primitive
collision boxes/cylinders. Sensor mounts are **not** part of the upstream model:
they come from `calibration/{sim,real}.yaml` with a per-value `source`.
No claim of validated dynamics, walking control or real sensor mounting
follows from this description.
