"""The types of path_timing_py, the extension module built from src/lib.rs."""

from collections.abc import Iterable
from typing import Final, final

type _Joints = tuple[float, ...]
"""A configuration of the joints, one position per joint."""

JOINT_PATH_CORNER_DEVIATION: Final[float]
MIN_JOINT_CHANGE_RAD: Final[float]

def distinct_joint_waypoints(waypoints: Iterable[Iterable[float]]) -> list[_Joints]: ...

@final
class JointTrajectory:
    @staticmethod
    def through(
        waypoints: Iterable[Iterable[float]],
        velocity_limits: Iterable[float],
        acceleration_limits: Iterable[float],
        requested_s: float,
    ) -> JointTrajectory: ...
    @property
    def duration_s(self) -> float: ...
    @property
    def start(self) -> _Joints: ...
    @property
    def end(self) -> _Joints: ...
    def sample(self, t_s: float) -> _Joints: ...
    def done(self, t_s: float) -> bool: ...
