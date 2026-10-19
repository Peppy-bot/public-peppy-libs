"""The named whole-arm postures, in joint_link radians.

Constants, matching openarm's description-owned postures: a posture is a fact
of the arm, not a deployment knob. Both are validated against the deployed
URDF's joint limits at startup.
"""

from __future__ import annotations

import math

# The collapsed park pose the arm ships in and rests in, measured on
# hardware and pulled inside the model: the physical fold reaches -1.80 rad
# on the shoulder, past the URDF's -1.745 floor (the model under-states the
# real travel), so the target rests just inside it. Folded onto the base,
# stable, minimal holding torque, safe to release.
HOME_POSITIONS_RAD = (-0.05, -1.72, 1.68, 1.24, 0.32)

# The working posture: every other joint at the center of its calibrated
# travel, where lerobot-calibrate homes it, and the wrist rolled a quarter
# turn from there, which turns the jaw's travel across the arm's plane and
# puts the camera mount above the jaw, so the wrist camera looks down onto
# the grasp with its picture upright. Where work starts.
READY_POSITIONS_RAD = (0.0, 0.0, 0.0, 0.0, -math.pi / 2)
