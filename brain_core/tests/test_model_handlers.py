"""The brain_model members against the core: get_model, list_models and
switch_model from goal to result, and the line a switch leaves in the
log."""

import fake_peppygen
import pytest
from conftest import FakeCtx, FakeDetector, FakeManipulator, FakeSource, make_brain, records_of

from brain_core.handlers import get_model, list_models, switch_model
from brain_core.ports import Role


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
    assert ctx.completed == dict(success=True, message="", role="perception", model="careful", action_time=0.0)
    assert get_model.handle(brain, None).models == ["careful", "fake"]
    assert source.prepared == [(Role.PERCEPTION, "careful")]
    assert [record.getMessage() for record in log.records] == ["switch_model perception 'careful' succeeded after 0.00 s"]


async def test_a_switch_to_the_active_model_succeeds():
    brain, _ = await brain_offering()
    ctx = switch_goal("manipulation", "fake")
    await switch_model.run(brain, ctx)
    assert ctx.completed["success"] is True and ctx.completed["model"] == "fake"


async def test_a_role_the_brain_does_not_have_is_refused_at_parse_and_the_models_never_see_it():
    brain, source = await brain_offering()
    asked_at_start = list(source.asked)
    for role in ("arm", "", "Perception"):
        ctx = switch_goal(role, "careful")
        await switch_model.run(brain, ctx)
        # A failed switch names no role and no model.
        assert ctx.completed == dict(
            success=False, message=f"this brain runs no model for the role '{role}'", role="", model="", action_time=0.0
        )
    assert source.asked == asked_at_start and source.prepared == []


async def test_a_source_without_every_role_stops_the_node():
    source = FakeSource({Role.PERCEPTION: {"fast": FakeDetector()}})
    with pytest.raises(ValueError, match="^the model source has no role manipulation: a brain runs a model for every role, 'none' included$"):
        await make_brain(source=source, perception_model="fast")


async def test_a_model_the_role_does_not_offer_is_refused_with_the_sources_words():
    brain, _ = await brain_offering()
    ctx = switch_goal("manipulation", "careful")
    await switch_model.run(brain, ctx)
    assert ctx.completed == dict(
        success=False, message="'careful' is not a manipulation model here; see list_models", role="", model="", action_time=0.0
    )
    assert get_model.handle(brain, None).models == ["fast", "fake"]
