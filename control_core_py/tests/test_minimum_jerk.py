import itertools
import math

import pytest

from control_core_py import minimum_jerk

CAPS = (2.0, 2.0, 2.0, 2.0, 2.0)
START = (0.0, 0.0, 0.0, 0.0, 0.0)
END = (1.0, -0.5, 0.25, 0.0, 2.0)


def test_endpoints():
    profile = minimum_jerk.plan(START, END, 2.0, CAPS)
    assert profile.sample(0.0) == START
    assert profile.sample(profile.duration_s) == END
    assert profile.sample(profile.duration_s + 5.0) == END


def test_monotone_toward_target_per_joint():
    profile = minimum_jerk.plan(START, END, 2.0, CAPS)
    steps = [profile.sample(t * profile.duration_s / 100) for t in range(101)]
    for j, (s, e) in enumerate(zip(START, END)):
        deltas = [b[j] - a[j] for a, b in itertools.pairwise(steps)]
        if e > s:
            assert all(d >= -1e-12 for d in deltas)
        elif e < s:
            assert all(d <= 1e-12 for d in deltas)
        else:
            assert all(abs(d) < 1e-12 for d in deltas)


def test_duration_floor_respects_velocity_caps():
    # Peak velocity of a min-jerk profile is 15/8 * delta / T.
    profile = minimum_jerk.plan(START, END, 0.0, CAPS)
    widest = max(abs(e - s) for s, e in zip(START, END))
    expected_floor = widest * 15.0 / (8.0 * CAPS[0])
    assert math.isclose(profile.duration_s, expected_floor)

    dt = profile.duration_s / 10_000
    peak = max(
        abs(b - a) / dt
        for t in range(10_000)
        for a, b in [
            (
                profile.sample(t * dt)[4],
                profile.sample((t + 1) * dt)[4],
            )
        ]
    )
    assert peak <= CAPS[4] * 1.01


def test_requested_duration_wins_when_slower():
    profile = minimum_jerk.plan(START, END, 30.0, CAPS)
    assert profile.duration_s == 30.0


def test_nonfinite_inputs_are_refused():
    # A NaN endpoint would zero the duration floor and bypass the caps.
    with pytest.raises(ValueError, match="non-finite"):
        minimum_jerk.plan((math.nan, *START[1:]), END, 0.0, CAPS)
    with pytest.raises(ValueError, match="non-finite"):
        minimum_jerk.plan(START, END, math.inf, CAPS)


def test_zero_move_completes_immediately():
    profile = minimum_jerk.plan(START, START, 0.0, CAPS)
    assert profile.duration_s == 0.0
    assert profile.done(0.0)
    assert profile.sample(0.0) == START


@pytest.mark.parametrize("t", [-1.0, 0.5, 10.0])
def test_samples_stay_within_bounds(t):
    profile = minimum_jerk.plan(START, END, 2.0, CAPS)
    sample = profile.sample(t)
    for j, value in enumerate(sample):
        low, high = sorted((START[j], END[j]))
        assert low - 1e-12 <= value <= high + 1e-12
