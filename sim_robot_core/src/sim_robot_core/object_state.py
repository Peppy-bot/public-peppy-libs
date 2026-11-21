"""What a simulation engine reports on object_state: one record per object it
spawned, and one per robot standing in its scene, from the feedback of its
attach goal that reports it standing until it leaves, giving the robot's name
and model and the world pose of its robot frame, the root link of its
description, with its velocities there in world axes; and a snapshot of both
at one capture."""

from __future__ import annotations

from dataclasses import dataclass

# The velocities of a robot frame fixed to the world.
AT_REST = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RobotRecord:
    """One standing robot in a snapshot: the name it stands under, its model,
    and the world pose of its robot frame, the root link of its description
    (orientation a unit quaternion x, y, z, w), with its world-axis
    velocities there."""

    robot: str
    model: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]

    @staticmethod
    def fixed(
        robot: str,
        model: str,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> RobotRecord:
        """The record of a robot whose robot frame is fixed to the world at
        `position` and `orientation`: zero velocities, as object_state:v1
        states for such a robot."""
        return RobotRecord(
            robot=robot,
            model=model,
            position=position,
            orientation=orientation,
            linear_velocity=AT_REST,
            angular_velocity=AT_REST,
        )

    def fields(self) -> dict:
        """The record as the generated object_state item types take it."""
        return {
            "robot": self.robot,
            "model": self.model,
            "position": list(self.position),
            "orientation": list(self.orientation),
            "linear_velocity": list(self.linear_velocity),
            "angular_velocity": list(self.angular_velocity),
        }


@dataclass(frozen=True)
class ObjectRecord:
    """One spawned object in a snapshot: its pose in world (orientation a
    unit quaternion x, y, z, w), its velocities along world's axes at its
    origin, and the name, asset, physics, mass and scale it was spawned
    with."""

    object_id: str
    name: str
    asset_id: str
    physics: str
    mass: float
    scale: float
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]

    def fields(self) -> dict:
        """The record as the generated object_state item types take it."""
        return {
            "object_id": self.object_id,
            "asset_id": self.asset_id,
            "physics": self.physics,
            "mass": self.mass,
            "scale": self.scale,
            "position": list(self.position),
            "orientation": list(self.orientation),
            "linear_velocity": list(self.linear_velocity),
            "angular_velocity": list(self.angular_velocity),
        }


@dataclass(frozen=True)
class ObjectStateSnapshot:
    """Every spawned object and every standing robot at one capture, in
    world, and the simulated instant of that capture."""

    timestamp_s: float
    objects: tuple[ObjectRecord, ...]
    robots: tuple[RobotRecord, ...]

    def record_of(self, object_id: str) -> ObjectRecord | None:
        """The record of the object `object_id`, or None when the capture
        holds no such object."""
        return next(
            (record for record in self.objects if record.object_id == object_id),
            None,
        )
