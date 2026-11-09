"""The brain_model members against the core: get_model, list_models,
forget, and switch_model from the decision of a goal to its end, and the
lines a switch and a forget leave in the log."""

import fake_peppygen
import pytest
from conftest import CAMERA_16X12, FakeCtx, FakeDetector, FakeManipulator, FakeSource, failed_with, make_brain, records_of, with_frames

from brain_core.handlers import forget, get_model, get_state, list_models, switch_model
from brain_core.perception.camera import CameraModel
from brain_core.ports import FailureKind, Refusal, Role


def switch_goal(role: str, model: str) -> FakeCtx:
    return FakeCtx(fake_peppygen.switch_model, role=role, model=model)


async def brain_offering():
    """A brain whose perception offers "fast" and "careful", "fast" at the
    start, and whose manipulation offers "fake"."""
    source = FakeSource({
        Role.PERCEPTION: {"fast": FakeDetector(), "careful": FakeDetector()},
        Role.MANIPULATION: {"fake": FakeManipulator()},
    })
    return await make_brain(source=source, perception_model="fast", manipulation_model="fake"), source


async def test_get_model_names_the_active_model_of_each_role_in_order():
    brain, _ = await brain_offering()
    response = get_model.handle(brain, None)
    assert (response.roles, response.models) == (["perception", "manipulation"], ["fast", "fake"])


async def test_get_model_says_why_perception_still_starts_until_its_frames_and_camera_come():
    brain, _ = await brain_offering()
    assert get_model.handle(brain, None).starting == ["no perception source: no camera frame received", ""]
    with_frames(brain)
    assert get_model.handle(brain, None).starting == ["", ""]


async def test_get_model_says_a_model_that_loads_is_starting_and_one_that_failed_is_not():
    loading = FakeDetector(unavailable="perception_model 'fake' is still loading", starting=True)
    brain = await make_brain(detector=loading, manipulator=FakeManipulator())
    with_frames(brain)
    assert get_model.handle(brain, None).starting == ["no perception source: perception_model 'fake' is still loading", ""]
    failed = FakeDetector(unavailable="perception_model 'fake' could not load: no weights")
    brain = await make_brain(detector=failed, manipulator=FakeManipulator())
    assert get_model.handle(brain, None).starting == ["", ""], "a failed load does not become ready"


async def test_get_model_does_not_wait_for_a_vacant_slot_or_a_none_model():
    brain = await make_brain(detector=FakeDetector(), manipulator=FakeManipulator())
    brain.perceiver.frames.vacant = True
    assert get_model.handle(brain, None).starting == ["", ""]
    brain = await make_brain()
    assert get_model.handle(brain, None).starting == ["", ""], "the none models never serve"


async def test_get_model_does_not_wait_for_a_model_that_loads_behind_a_vacant_slot():
    loading = FakeDetector(unavailable="perception_model 'fake' is still loading", starting=True)
    brain = await make_brain(detector=loading, manipulator=FakeManipulator())
    brain.perceiver.frames.vacant = True
    assert get_model.handle(brain, None).starting == ["", ""], "perception never serves with no camera"


async def test_get_model_does_not_wait_for_a_camera_the_robot_does_not_carry():
    loading = FakeDetector(unavailable="perception_model 'fake' is still loading", starting=True)
    brain = await make_brain(detector=loading, manipulator=FakeManipulator())
    with_frames(brain)
    # The camera has answered its intrinsics; the robot does not place it.
    brain.perceiver.set_camera(CameraModel().with_intrinsics(CAMERA_16X12))
    brain.perceiver.pose.update("the robot carries no camera named 'chest' (it carries: wrist_left)", starting=False)
    assert get_model.handle(brain, None).starting == ["", ""]


async def test_list_models_groups_every_accepted_model_by_role_in_get_models_order():
    brain, _ = await brain_offering()
    response = list_models.handle(brain, None)
    assert response.roles == ["perception", "perception", "manipulation"]
    assert response.models == ["fast", "careful", "fake"]


async def test_a_switch_names_its_role_and_model_and_get_model_reports_it(caplog):
    brain, source = await brain_offering()
    ctx = switch_goal("perception", "careful")
    with records_of(caplog, "brain_core.brain") as log:
        await switch_model.run(brain, ctx)
    assert ctx.completed == dict(role="perception", model="careful", action_time=0.0)
    assert get_model.handle(brain, None).models == ["careful", "fake"]
    assert source.prepared == [(Role.PERCEPTION, "careful")]
    assert [record.getMessage() for record in log.records] == ["switch_model perception 'careful' succeeded after 0.00 s"]


async def test_a_switch_to_the_active_model_succeeds():
    brain, _ = await brain_offering()
    ctx = switch_goal("manipulation", "fake")
    await switch_model.run(brain, ctx)
    assert ctx.completed["model"] == "fake"


async def test_a_role_the_brain_does_not_have_is_refused_at_parse_and_the_models_never_see_it():
    brain, source = await brain_offering()
    asked_at_start = list(source.asked)
    for role in ("arm", "", "Perception"):
        ctx = switch_goal(role, "careful")
        with pytest.raises(Refusal, match=f"^this brain runs no model for the role '{role}'$") as raised:
            switch_model.decide(brain, ctx.request().data)
        assert raised.value.kind is FailureKind.REJECTED
        await switch_model.run(brain, ctx)
        # A failed switch gives no result.
        assert failed_with(ctx, FailureKind.REJECTED, f"this brain runs no model for the role '{role}'")
    assert source.asked == asked_at_start and source.prepared == []


async def test_a_source_without_every_role_stops_the_node():
    source = FakeSource({Role.PERCEPTION: {"fast": FakeDetector()}})
    with pytest.raises(ValueError, match="^the model source has no role manipulation: a brain runs a model for every role, 'none' included$"):
        await make_brain(source=source, perception_model="fast")


async def test_a_model_the_role_does_not_offer_is_refused_with_the_sources_words():
    brain, _ = await brain_offering()
    ctx = switch_goal("manipulation", "careful")
    with pytest.raises(Refusal, match="^'careful' is not a manipulation model here; see list_models$") as raised:
        switch_model.decide(brain, ctx.request().data)
    assert raised.value.kind is FailureKind.REJECTED
    await switch_model.run(brain, ctx)
    assert failed_with(ctx, FailureKind.REJECTED, "'careful' is not a manipulation model here; see list_models")
    assert get_model.handle(brain, None).models == ["fast", "fake"]


async def test_a_switch_is_refused_at_once_while_its_role_is_busy_and_once_the_node_shuts_down():
    brain, source = await brain_offering()
    switch_model.decide(brain, switch_goal("perception", "careful").request().data)
    busy = brain.sequencer.start(Role.PERCEPTION, "identify_item", 0)
    with pytest.raises(Refusal, match="^identify_item is running$") as raised:
        switch_model.decide(brain, switch_goal("perception", "careful").request().data)
    assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    # The other role is free.
    switch_model.decide(brain, switch_goal("manipulation", "fake").request().data)
    brain.sequencer.finish(busy)
    brain.sequencer.close()
    with pytest.raises(Refusal, match="^the node is shutting down$") as raised:
        switch_model.decide(brain, switch_goal("perception", "careful").request().data)
    assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    assert source.prepared == []


async def test_forget_answers_what_the_brain_forgot_and_leaves_no_gripper_holding(caplog):
    brain = await make_brain(detector=FakeDetector(), manipulator=FakeManipulator())
    mug = brain.state.mint_from_pose((0.4, -0.2, 0.45), None, now_ns=1)
    brain.state.set_held(brain.state.grippers["right_gripper"], mug.item_id)
    assert get_state.handle(brain, None).holding == [False, True]
    with records_of(caplog, "brain_core.handlers.forget") as log:
        response = forget.handle(brain, None)
        again = forget.handle(brain, None)
    assert response.item_ids == [mug.item_id]
    assert again.item_ids == []
    state = get_state.handle(brain, None)
    assert (state.holding, state.held_item_ids) == ([False, False], ["", ""])
    assert [record.getMessage() for record in log.records] == [
        f"forget: the brain forgot {mug.item_id}",
        "forget: the brain forgot no item",
    ]
