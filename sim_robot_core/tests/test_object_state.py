"""A robot's and an object's records on object_state: a robot fixed to the
world at rest, each record as the generated item types take it, and a
snapshot's record of an object."""

from sim_robot_core.object_state import AT_REST, ObjectRecord, ObjectStateSnapshot, RobotRecord

BLOCK = ObjectRecord(
    object_id="obj_1",
    name="red block",
    asset_id="props/blocks/red_block",
    physics="dynamic",
    mass=0.1,
    scale=1.0,
    position=(0.5, 0.0, 0.8),
    orientation=(0.0, 0.0, 0.0, 1.0),
    linear_velocity=(0.0, 0.0, -0.2),
    angular_velocity=(0.0, 0.0, 0.0),
)


def test_a_robot_fixed_to_the_world_stands_at_its_placement_at_rest():
    record = RobotRecord.fixed("alpha", "openarm_v2", (0.0, -1.5, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert (record.linear_velocity, record.angular_velocity) == (AT_REST, AT_REST)
    assert record.fields() == {
        "robot": "alpha",
        "model": "openarm_v2",
        "position": [0.0, -1.5, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "linear_velocity": [0.0, 0.0, 0.0],
        "angular_velocity": [0.0, 0.0, 0.0],
    }


def test_a_rolling_robots_record_carries_its_velocities():
    record = RobotRecord(
        "india", "botshare_base", (1.0, 0.5, 0.0), (0.0, 0.0, 0.0, 1.0), (0.2, 0.0, 0.0), (0.0, 0.0, 0.5)
    )
    assert (record.fields()["linear_velocity"], record.fields()["angular_velocity"]) == (
        [0.2, 0.0, 0.0],
        [0.0, 0.0, 0.5],
    )


def test_an_objects_record_gives_the_fields_object_state_carries():
    assert BLOCK.fields() == {
        "object_id": "obj_1",
        "asset_id": "props/blocks/red_block",
        "physics": "dynamic",
        "mass": 0.1,
        "scale": 1.0,
        "position": [0.5, 0.0, 0.8],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "linear_velocity": [0.0, 0.0, -0.2],
        "angular_velocity": [0.0, 0.0, 0.0],
    }


def test_a_snapshot_gives_the_record_of_each_object_it_holds_and_of_no_other():
    snapshot = ObjectStateSnapshot(timestamp_s=1.5, objects=(BLOCK,), robots=())
    assert snapshot.record_of("obj_1") == BLOCK
    assert snapshot.record_of("obj_2") is None
