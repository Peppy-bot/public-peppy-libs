"""The named whole-arm postures, in joint_link radians.

Constants, matching openarm's description-owned postures: a posture is a fact
of the arm, not a deployment knob. Both are validated against the deployed
URDF's joint limits at startup.
"""

from __future__ import annotations

# The lerobot calibration midpoint: every joint at the center of its
# calibrated travel, the pose lerobot-calibrate is anchored on.
HOME_POSITIONS_RAD = (0.0, 0.0, 0.0, 0.0, 0.0)

# Tucked and raised, FK-verified on SO-ARM100's so101_new_calib.urdf: the
# grasp frame pulls in to x=0.27 m and rises to z=0.28 m (the home pose sits
# extended at x=0.39 m, z=0.23 m), well inside every joint limit.
READY_POSITIONS_RAD = (0.0, -1.0, 1.0, -0.5, 0.0)
