# Vendored URDF

The canonical OpenArm URDF the crate's `description` module embeds
(`include_str!`) and hands to the agnostic kinematics/dynamics. It is bimanual; a
single 7-joint chain (base link to joint7's child) is extracted for the left or
right arm at load time.

- Source repo: https://github.com/enactic/openarm_description
- License: Apache-2.0 (from the upstream repository)

## `openarm_v10.urdf` (OpenArm V1.0)

Derived from the pre-expanded V1.0 bimanual URDF with the parallel-link finger
geometry stripped (the OpenArm has no hand; the gripper is a separate
single-DoF node in this repo).

- Upstream path: `assets/robot/openarm_v1.0/urdf/example/v1.urdf`
- Upstream commit: `3c65e5f889c486eab85b64df7fc8518eb13d1e4b`
- Chain: `openarm_{side}_link0` to `openarm_{side}_link7`.
- Modifications from upstream:
  - Removed `openarm_{left,right}_{left,right}_finger` links.
  - Removed `openarm_{left,right}_finger_joint{1,2}` prismatic joints.
  - Removed the commented-out xacro finger templates that survived expansion.
  - Kept `openarm_{left,right}_hand_tcp` fixed frames for future IK work.

A new revision drops in by adding its URDF here and a `Version` variant in
`src/lib.rs`.
