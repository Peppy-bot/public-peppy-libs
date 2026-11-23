"""The rule a MuJoCo model of a base's wheel is held to, read from the numbers
of the model's actuators: the wheel's rate loop is one velocity servo, its kv
the loop's proportional gain, and one integrated-velocity servo, its kp the
loop's integral gain, both on the wheel's joint with gear 1, the integrated
one's activation unclamped, as a wheel turns without end. The MuJoCo engine
that runs the model and the builder that writes the Isaac stage from it hold
it to this one rule.

`wheel_rate_loop` reads a model by MuJoCo's arrays of its actuators
(`MjModel.actuator_*`, `nu`) and their names (`model.actuator(index).name`),
so it needs no MuJoCo here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# MuJoCo's enumerations the rule reads, as include/mujoco/mjmodel.h numbers
# them: mjtTrn mjTRN_JOINT, mjtDyn mjDYN_NONE and mjDYN_INTEGRATOR, mjtGain
# mjGAIN_FIXED and mjtBias mjBIAS_AFFINE.
TRN_JOINT = 0
DYN_NONE = 0
DYN_INTEGRATOR = 1
GAIN_FIXED = 0
BIAS_AFFINE = 1

_PROPORTIONAL = "proportional"
_INTEGRAL = "integral"


@dataclass(frozen=True)
class RateLoopActuators:
    """The two actuators of a wheel's rate loop, by their index in the model,
    with the loop's gains: the velocity servo's kv (N*m*s/rad) and the
    integrated-velocity servo's kp (N*m/rad)."""

    proportional: int
    integral: int
    proportional_gain: float
    integral_gain: float


def _part(model, actuator: int) -> str | None:
    """The part `actuator` plays in a wheel's rate loop: proportional for a
    velocity servo, whose force is its gain times its control less the
    joint's rate, its bias [0, 0, -kv], and integral for an
    integrated-velocity servo, whose force is its gain times its activation
    less the joint's angle, its bias [0, -kp, 0], each of a fixed gain that
    is a positive number; None for any other."""
    gain = float(model.actuator_gainprm[actuator][0])
    fixed_affine = (
        int(model.actuator_gaintype[actuator]) == GAIN_FIXED
        and int(model.actuator_biastype[actuator]) == BIAS_AFFINE
        and math.isfinite(gain)
        and gain > 0.0
    )
    if not fixed_affine:
        return None
    bias = [float(value) for value in model.actuator_biasprm[actuator][:3]]
    dynamics = int(model.actuator_dyntype[actuator])
    if dynamics == DYN_NONE and bias == [0.0, 0.0, -gain]:
        return _PROPORTIONAL
    if dynamics == DYN_INTEGRATOR and bias == [0.0, -gain, 0.0]:
        return _INTEGRAL
    return None


def wheel_rate_loop(model, wheel: str, joint: int) -> RateLoopActuators:
    """The rate loop of the wheel `wheel`, the joint of index `joint` of
    `model`, as the rule holds its actuators to. Anything else raises
    ValueError naming the wheel and what its MJCF needs."""
    where = f"wheel '{wheel}'"
    drivers = [
        actuator
        for actuator in range(int(model.nu))
        if int(model.actuator_trntype[actuator]) == TRN_JOINT
        and int(model.actuator_trnid[actuator][0]) == joint
    ]
    parts = {_part(model, actuator): actuator for actuator in drivers}
    if len(drivers) != 2 or set(parts) != {_PROPORTIONAL, _INTEGRAL}:
        raise ValueError(
            f"{where}: {len(drivers)} actuators drive it; drive the wheel with one <velocity> "
            "actuator, its kv the loop's proportional gain, and one <intvelocity> actuator, its "
            "kp the loop's integral gain"
        )
    for actuator in drivers:
        gear = float(model.actuator_gear[actuator][0])
        if gear != 1.0:
            raise ValueError(
                f"{where}: its actuator '{model.actuator(actuator).name}' has gear {gear:g}; a "
                "wheel's setpoints are its rates, so give the actuator gear 1"
            )
    integral = parts[_INTEGRAL]
    if model.actuator_actlimited[integral]:
        raise ValueError(
            f"{where}: its actuator '{model.actuator(integral).name}' clamps its activation; a "
            "wheel turns without end, so give the <intvelocity> no actrange"
        )
    proportional = parts[_PROPORTIONAL]
    return RateLoopActuators(
        proportional=proportional,
        integral=integral,
        proportional_gain=float(model.actuator_gainprm[proportional][0]),
        integral_gain=float(model.actuator_gainprm[integral][0]),
    )
