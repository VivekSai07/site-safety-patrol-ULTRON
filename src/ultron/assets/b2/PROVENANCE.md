# Unitree B2 model provenance

`upstream.xml` is Unitree Robotics' B2 MJCF fetched 27 November 2026 from:
https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/b2/b2.xml
Git blob SHA: `88e1a789b67e90adbb120237b03a11cb54b1b210`.

Retained under the adjacent upstream BSD-3-Clause LICENSE. Source is preserved unmodified. `ultron.scene.make_scene` removes mesh assets and mesh geoms at runtime, retaining the manufacturer's joint hierarchy, inertial definitions and primitive collision geometry; it adds a synthetic camera and construction scene. No official mesh files are required. No claim of validated B2 dynamics, walking control, sensor mounting or real-hardware compatibility follows from using this description.
