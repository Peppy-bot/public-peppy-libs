"""The rule a MuJoCo model of a base's wheel is held to, read from its
actuators' numbers: one velocity servo and one integrated-velocity servo on
the wheel's joint, of gear 1, the integrated one's activation unclamped."""

from __future__ import annotations

import re
from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from sim_robot_core.rate_loop import (
    BIAS_AFFINE,
    DYN_INTEGRATOR,
    DYN_NONE,
    GAIN_FIXED,
    TRN_JOINT,
    RateLoopActuators,
    wheel_rate_loop,
)

WHEEL = 3
OTHER_JOINT = 4


@dataclass(frozen=True)
class Actuator:
    name: str
    joint: int
    dynamics: int
    gain: float
    bias: tuple[float, float, float]
    gear: float = 1.0
    activation_limited: bool = False


def velocity(name: str, kv: float, joint: int = WHEEL, gear: float = 1.0) -> Actuator:
    return Actuator(name, joint, DYN_NONE, kv, (0.0, 0.0, -kv), gear)


def intvelocity(name: str, kp: float, joint: int = WHEEL, activation_limited: bool = False) -> Actuator:
    return Actuator(name, joint, DYN_INTEGRATOR, kp, (0.0, -kp, 0.0), activation_limited=activation_limited)


def position(name: str, kp: float) -> Actuator:
    return Actuator(name, WHEEL, DYN_NONE, kp, (0.0, -kp, 0.0))


class Model:
    """A MuJoCo model's actuators, by the arrays MjModel gives them."""

    def __init__(self, *actuators: Actuator) -> None:
        self._actuators = actuators
        self.nu = len(actuators)
        self.actuator_trntype = [TRN_JOINT] * self.nu
        self.actuator_trnid = [[actuator.joint, -1] for actuator in actuators]
        self.actuator_gear = [[actuator.gear, 0.0, 0.0, 0.0, 0.0, 0.0] for actuator in actuators]
        self.actuator_dyntype = [actuator.dynamics for actuator in actuators]
        self.actuator_gaintype = [GAIN_FIXED] * self.nu
        self.actuator_biastype = [BIAS_AFFINE] * self.nu
        self.actuator_gainprm = [[actuator.gain] + [0.0] * 9 for actuator in actuators]
        self.actuator_biasprm = [list(actuator.bias) + [0.0] * 7 for actuator in actuators]
        self.actuator_actlimited = [actuator.activation_limited for actuator in actuators]

    def actuator(self, index: int):
        return SimpleNamespace(name=self._actuators[index].name)


def test_a_wheel_driven_by_its_two_servos_gives_its_loop():
    model = Model(velocity("arm", 9.0, joint=OTHER_JOINT), velocity("p", 64.0), intvelocity("i", 128.0))

    assert wheel_rate_loop(model, "left_wheel", WHEEL) == RateLoopActuators(
        proportional=1, integral=2, proportional_gain=64.0, integral_gain=128.0
    )


@pytest.mark.parametrize(
    ("actuators", "refusal"),
    [
        (
            (position("p", 64.0), intvelocity("i", 128.0)),
            "wheel 'left_wheel': 2 actuators drive it; drive the wheel with one <velocity> actuator",
        ),
        (
            (intvelocity("i", 128.0),),
            "wheel 'left_wheel': 1 actuators drive it; drive the wheel with one <velocity> actuator",
        ),
        (
            (velocity("p", 64.0, gear=2.0), intvelocity("i", 128.0)),
            "wheel 'left_wheel': its actuator 'p' has gear 2; a wheel's setpoints are its rates",
        ),
        (
            (velocity("p", 64.0), intvelocity("i", 128.0, activation_limited=True)),
            "wheel 'left_wheel': its actuator 'i' clamps its activation; a wheel turns without end",
        ),
    ],
    ids=["position_servo", "one_servo", "geared", "clamped_integral"],
)
def test_a_wheel_driven_otherwise_is_refused_saying_what_its_mjcf_needs(actuators, refusal):
    with pytest.raises(ValueError, match=f"^{re.escape(refusal)}"):
        wheel_rate_loop(Model(*actuators), "left_wheel", WHEEL)
