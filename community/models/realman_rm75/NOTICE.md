# RealMan RM75-B model assets

Copyright 2024 realman-robotics. Licensed under Apache-2.0; see [LICENSE](LICENSE)
and the unmodified upstream [license notice](UPSTREAM_LICENSE).

Source: [RealManRobot/rm_models](https://github.com/RealManRobot/rm_models/tree/b63dddb20b7620d14cfff32678de60bb3a201ebc/RM75/urdf/RM75-B),
pinned commit `b63dddb20b7620d14cfff32678de60bb3a201ebc` (retrieved 2026-10-07).

- `RM75-B.urdf` is copied unchanged from `RM75/urdf/RM75-B/urdf/RM75-B.urdf`.
- The eight `meshes/*.STL.gz` files are lossless gzip encodings (mtime zero) of
  the corresponding upstream STL files. `mesh-sha256.json` records the original,
  uncompressed bytes. No vertex coordinates or mesh topology were changed.
- No ROS package, launch file, driver, controller or dynamics implementation is used.
  The Community adapter reads the XML with Python's standard library and resolves
  these bundled meshes directly, without a ROS package resolver.

The adapter uses the URDF's joint origins, axes and position limits, including its
rounded radian values. Each visible link uses a conservative local bounding box
with 1 mm expansion for sampled collision checks; these are not triangle-accurate
contact checks. Body, installation transforms and gripper geometry are Community
reference parameters, not manufacturer calibration of the assembled robot.

Independent FK reference: [official RM75 parameter and MDH table](https://develop.realman-robotics.com/robot4th/robotParameter/RM75OntologyParameters/).
RM75-B uses `d7 = 0.144 m`. The same documentation lists different flange variants.
Only RM75-B is enabled here: the repository's RM75-6F URDF at this commit has
`d7 = 0.1725 m`, while the current documentation lists `0.1612 m`; importing that
variant without resolving the revision would silently change the flange pose.
The configuration rejects unsupported model names rather than substituting RM75-B.
