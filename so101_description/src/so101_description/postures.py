"""The named whole-arm postures, in joint_link radians.

Constants, matching openarm's description-owned postures: a posture is a fact
of the arm, not a deployment knob. Both are validated against the deployed
URDF's joint limits at startup.
"""

from __future__ import annotations

# The collapsed park pose the arm ships in and rests in, measured on
# hardware: folded onto its base, stable, minimal holding torque, safe to
# release. Where move_to_home parks the arm before power-off.
HOME_POSITIONS_RAD = (-0.05, -1.8, 1.68, 1.24, 0.32)

# The calibration middle: every joint at the center of its calibrated
# travel (the lerobot-calibrate homing pose), maximally far from every hard
# stop. Where work starts.
READY_POSITIONS_RAD = (0.0, 0.0, 0.0, 0.0, 0.0)
