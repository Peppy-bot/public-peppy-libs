"""The rules in State, each one a test: gripper resolution, the holding
flag, item ids across looks and runs, and the best match for a description."""

import pytest

from brain_core.limbs import Limb, parse_limbs
from brain_core.ports import Coverage, Detection, Refusal
from brain_core.state import State, best_match, new_run_token
from brain_core.words import content_words, named_by, normalised

# A scan that can name anything, and a search by description, which covers nothing.
EVERY = Coverage(every_label=True)
NOTHING = Coverage()


TWO_ARMS = parse_limbs("left_arm:left_gripper,right_arm:right_gripper")
ONE_ARM = (Limb("left_arm", "left_gripper"),)


def make_state(run_token: str = "t0") -> State:
    return State(TWO_ARMS, run_token)


def test_a_region_follows_the_look_that_last_saw_the_item():
    state = State(ONE_ARM, "t0")
    [cup] = state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9, region=(1.0, 2.0, 3.0, 4.0))], now_ns=1, coverage=EVERY)
    assert cup.region == (1.0, 2.0, 3.0, 4.0)
    # A search by description sees it elsewhere in its own picture.
    [again] = state.remember([Detection("cup", (0.51, 0.1, 0.7), 0.8, region=(5.0, 6.0, 7.0, 8.0))], now_ns=2, coverage=NOTHING)
    assert again is cup and cup.region == (5.0, 6.0, 7.0, 8.0)
    # A look that keeps the item without seeing it clears the region: its
    # picture is not the one the region was read in.
    state.remember([Detection("bowl", (0.9, 0.0, 0.7), 0.9, region=(0.0, 0.0, 1.0, 1.0))], now_ns=3, coverage=NOTHING)
    assert cup.region is None
    # An item addressed by pose has no picture.
    assert state.mint_from_pose((0.2, 0.2, 0.7), None, now_ns=4).region is None


def test_a_width_follows_the_look_that_last_saw_the_item_and_stays_when_one_does_not():
    state = State(ONE_ARM, "t0")
    [cup] = state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9, width_m=0.08)], now_ns=1, coverage=EVERY)
    assert cup.width_m == 0.08
    state.remember([Detection("cup", (0.51, 0.1, 0.7), 0.8, width_m=0.07)], now_ns=2, coverage=NOTHING)
    assert cup.width_m == 0.07
    # A look that keeps the item without seeing it leaves its width: the
    # item has not changed.
    state.remember([Detection("bowl", (0.9, 0.0, 0.7), 0.9, width_m=0.2)], now_ns=3, coverage=NOTHING)
    assert cup.width_m == 0.07
    # An item addressed by pose has no width.
    assert state.mint_from_pose((0.2, 0.2, 0.7), None, now_ns=4).width_m is None


def test_a_top_follows_the_look_that_last_saw_the_item_and_a_placed_item_has_none():
    state = State(ONE_ARM, "t0")
    [cup] = state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9, top_z=0.78)], now_ns=1, coverage=EVERY)
    assert cup.top_z == 0.78
    state.remember([Detection("cup", (0.51, 0.1, 0.7), 0.8, top_z=0.77)], now_ns=2, coverage=NOTHING)
    assert cup.top_z == 0.77
    # A look that keeps the item without seeing it leaves its top.
    state.remember([Detection("bowl", (0.9, 0.0, 0.7), 0.9, top_z=0.75)], now_ns=3, coverage=NOTHING)
    assert cup.top_z == 0.77
    # An item place_item put down stands where no look measured its top.
    left = state.grippers["left_gripper"]
    state.set_held(left, cup.item_id)
    state.clear_held_placed(left, (0.6, -0.2, 0.75), None)
    assert cup.top_z is None
    # An item addressed by pose has no top.
    assert state.mint_from_pose((0.2, 0.2, 0.7), None, now_ns=4).top_z is None


def test_each_gripper_carries_the_arm_its_limb_names_in_the_limbs_order():
    state = make_state()
    assert state.gripper_names() == ["left_gripper", "right_gripper"]
    assert state.grippers["left_gripper"].arm == "left_arm"
    assert state.grippers["right_gripper"].arm == "right_arm"
    # The SO-101 names its one limb with no side.
    so101 = State(parse_limbs("arm:gripper"), "t0")
    assert so101.grippers["gripper"].arm == "arm"


def test_a_gripper_without_a_side_in_its_name_is_taken_in_the_limbs_order():
    state = State(parse_limbs("arm_b:gripper_b,arm_a:gripper_a"), "t0")
    assert state.gripper_for_grab("", (0.5, -0.2, 0.7)).name == "gripper_b"
    state.set_held(state.grippers["gripper_b"], "cup_1")
    assert state.gripper_for_grab("", (0.5, -0.2, 0.7)).name == "gripper_a"


def test_the_run_token_must_be_letters_and_digits():
    for token in ("", "a-b", "a b", "a_b"):
        with pytest.raises(ValueError, match="run_token must be letters and digits"):
            State(ONE_ARM, token)
    token = new_run_token()
    assert len(token) == 6 and int(token, 16) >= 0
    assert State(ONE_ARM, token).run_token == token


def test_named_gripper_for_a_grab_must_exist_and_be_free():
    state = make_state()
    with pytest.raises(Refusal, match="unknown gripper 'claw'"):
        state.gripper_for_grab("claw", None)
    left = state.gripper_for_grab("left_gripper", None)
    state.set_held(left, "cup_1")
    with pytest.raises(Refusal, match="already holds item 'cup_1'"):
        state.gripper_for_grab("left_gripper", None)


def test_an_empty_name_picks_a_free_gripper_on_the_items_side():
    state = make_state()
    assert state.gripper_for_grab("", (0.5, 0.2, 0.7)).name == "left_gripper"
    assert state.gripper_for_grab("", (0.5, -0.2, 0.7)).name == "right_gripper"
    state.set_held(state.grippers["right_gripper"], "cup_1")
    assert state.gripper_for_grab("", (0.5, -0.2, 0.7)).name == "left_gripper"
    state.set_held(state.grippers["left_gripper"], "cup_2")
    with pytest.raises(Refusal, match="no gripper is free"):
        state.gripper_for_grab("", (0.5, 0.0, 0.7))


def test_the_holder_rules_for_drop_and_place():
    state = make_state()
    with pytest.raises(Refusal, match="no gripper holds an item"):
        state.holder("")
    with pytest.raises(Refusal, match="holds nothing"):
        state.holder("left_gripper")
    state.set_held(state.grippers["left_gripper"], "cup_1")
    assert state.holder("").name == "left_gripper"
    state.set_held(state.grippers["right_gripper"], "cup_2")
    with pytest.raises(Refusal, match="more than one gripper"):
        state.holder("")
    assert state.holder("right_gripper").name == "right_gripper"
    assert state.forget_dropped(state.grippers["right_gripper"]) == "cup_2"
    assert state.snapshot() == (["left_gripper", "right_gripper"], [True, False], ["cup_1", ""])


def test_a_scan_keeps_ids_for_items_that_stayed_and_drops_the_ones_that_left():
    state = make_state()
    first = state.remember(
        [Detection("cup", (0.50, 0.10, 0.70), 0.9), Detection("cup", (0.50, -0.20, 0.70), 0.8), Detection("banana", (0.40, 0.0, 0.70), 0.7)],
        now_ns=1,
        coverage=EVERY,
    )
    assert [item.item_id for item in first] == ["cup_1-t0", "cup_2-t0", "banana_1-t0"]
    # The first cup moved 2 cm, the second one is gone, a bowl appeared.
    second = state.remember(
        [Detection("cup", (0.52, 0.10, 0.70), 0.95), Detection("bowl", (0.60, 0.0, 0.70), 0.6)],
        now_ns=2,
        coverage=EVERY,
    )
    assert [item.item_id for item in second] == ["cup_1-t0", "bowl_1-t0"]
    assert set(state.items) == {"cup_1-t0", "bowl_1-t0"}
    assert state.items["cup_1-t0"].position == (0.52, 0.10, 0.70)
    # Beyond the match radius the same label is a new item, and a dropped
    # number is never minted again.
    third = state.remember([Detection("cup", (0.80, 0.10, 0.70), 0.9)], now_ns=3, coverage=EVERY)
    assert third[0].item_id == "cup_3-t0"
    assert set(state.items) == {"cup_3-t0"}


def test_a_scan_never_drops_an_item_a_gripper_holds():
    state = make_state()
    state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9)], now_ns=1, coverage=EVERY)
    state.set_held(state.grippers["left_gripper"], "cup_1-t0")
    state.remember([], now_ns=2, coverage=EVERY)
    assert "cup_1-t0" in state.items


def test_an_identify_search_refreshes_without_dropping_the_rest():
    state = make_state()
    state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9), Detection("banana", (0.4, 0.0, 0.7), 0.7)], now_ns=1, coverage=EVERY)
    found = state.remember([Detection("cup", (0.51, 0.1, 0.7), 0.8)], now_ns=2, coverage=NOTHING)
    assert found[0].item_id == "cup_1-t0"
    assert found[0].position == (0.51, 0.1, 0.7) and found[0].confidence == 0.8 and found[0].seen_at_ns == 2
    assert set(state.items) == {"cup_1-t0", "banana_1-t0"}


def test_a_search_under_other_words_keeps_the_scans_id_and_label():
    state = make_state()
    vocabulary_scan = Coverage(frozenset({"mustard bottle", "apple"}))
    scanned = state.remember([Detection("mustard bottle", (0.5, 0.0, 0.7), 0.9)], now_ns=1, coverage=vocabulary_scan)[0]
    # The words route labels the same bottle with the caller's description.
    found = state.remember([Detection("yellow bottle", (0.51, 0.0, 0.7), 0.96)], now_ns=2, coverage=NOTHING)[0]
    assert found.item_id == scanned.item_id == "mustard_bottle_1-t0"
    assert found.label == "mustard bottle"
    rescanned = state.remember([Detection("mustard bottle", (0.5, 0.0, 0.7), 0.9)], now_ns=3, coverage=vocabulary_scan)[0]
    assert rescanned.item_id == "mustard_bottle_1-t0"
    assert set(state.items) == {"mustard_bottle_1-t0"}


def test_a_scan_under_other_words_keeps_the_searchs_id_and_takes_its_label():
    state = make_state()
    found = state.remember([Detection("red apple", (0.5, 0.1, 0.7), 1.0)], now_ns=1, coverage=NOTHING)[0]
    assert found.item_id == "red_apple_1-t0"
    # An open-vocabulary scan names the same apple in its own words.
    scanned = state.remember([Detection("apple", (0.51, 0.1, 0.7), 1.0)], now_ns=2, coverage=EVERY)[0]
    assert scanned.item_id == "red_apple_1-t0" and scanned.label == "apple"
    # Under a label the scan names, the apple is dropped once it is gone.
    state.remember([], now_ns=3, coverage=Coverage(frozenset({"apple"})))
    assert state.items == {}


def test_items_sharing_a_place_keep_their_own_ids():
    state = make_state()
    first = state.remember(
        [Detection("bowl", (0.50, 0.00, 0.70), 0.9), Detection("apple", (0.51, 0.01, 0.72), 0.8)], now_ns=1, coverage=EVERY
    )
    assert [item.item_id for item in first] == ["bowl_1-t0", "apple_1-t0"]
    # The apple now comes first and sits nearer the bowl's last position
    # than the bowl does: the label decides between the two.
    second = state.remember(
        [Detection("apple", (0.50, 0.00, 0.71), 0.95), Detection("bowl", (0.52, 0.00, 0.70), 0.9)], now_ns=2, coverage=EVERY
    )
    assert [item.item_id for item in second] == ["apple_1-t0", "bowl_1-t0"]
    assert [item.label for item in second] == ["apple", "bowl"]


def test_an_identify_search_takes_no_near_item_whose_label_shares_no_word():
    state = make_state()
    bowl = state.remember([Detection("bowl", (0.231, -0.005, 0.383), 0.0)], now_ns=1, coverage=NOTHING)[0]
    # The mug stands in the bowl, 1 cm from the bowl's place.
    mug = state.remember([Detection("the mug", (0.240, -0.004, 0.390), 0.0)], now_ns=2, coverage=NOTHING)[0]
    assert (mug.item_id, mug.label) == ("the_mug_1-t0", "the mug")
    assert bowl.item_id == "bowl_1-t0" and bowl.position == (0.231, -0.005, 0.383)
    again = state.remember([Detection("mug", (0.239, -0.004, 0.390), 0.0)], now_ns=3, coverage=NOTHING)[0]
    assert again.item_id == "the_mug_1-t0"
    assert set(state.items) == {"bowl_1-t0", "the_mug_1-t0"}


def test_a_scan_takes_no_near_item_by_its_place_when_it_does_not_cover_its_label():
    state = make_state()
    state.remember([Detection("bowl", (0.50, 0.00, 0.70), 0.9)], now_ns=1, coverage=NOTHING)
    # The scan sees only the mug in the bowl and cannot name a bowl: it
    # does not rename the bowl.
    [mug] = state.remember([Detection("mug", (0.51, 0.00, 0.71), 0.9)], now_ns=2, coverage=Coverage(frozenset({"mug"})))
    assert (mug.item_id, mug.label) == ("mug_1-t0", "mug")
    assert state.items["bowl_1-t0"].label == "bowl"


def test_a_covering_scan_that_names_an_item_in_other_words_keeps_its_id_and_renames_it():
    state = make_state()
    [mug] = state.remember([Detection("ceramic mug", (0.50, 0.00, 0.70), 0.9)], now_ns=1, coverage=EVERY)
    # The open-vocabulary scan names the same mug in words that share none
    # with its label: it would drop the mug unseen, so it is the mug.
    [cup] = state.remember([Detection("coffee cup", (0.51, 0.00, 0.70), 0.9)], now_ns=2, coverage=EVERY)
    assert cup is mug
    assert (cup.item_id, cup.label, cup.position) == ("ceramic_mug_1-t0", "coffee cup", (0.51, 0.00, 0.70))
    assert set(state.items) == {"ceramic_mug_1-t0"}


def test_a_covering_scan_keeps_the_ids_of_a_mug_and_the_bowl_it_stands_in_whatever_their_order():
    state = make_state()
    state.remember([Detection("bowl", (0.50, 0.00, 0.70), 0.9)], now_ns=1, coverage=EVERY)
    # The mug in the bowl comes first and lies nearer the bowl's place than
    # the bowl does: the bowl's own detection takes the bowl's id.
    mug, bowl = state.remember(
        [Detection("mug", (0.50, 0.00, 0.71), 0.9), Detection("bowl", (0.52, 0.00, 0.70), 0.9)], now_ns=2, coverage=EVERY
    )
    assert (mug.item_id, bowl.item_id) == ("mug_1-t0", "bowl_1-t0")
    assert bowl.position == (0.52, 0.00, 0.70)
    assert set(state.items) == {"mug_1-t0", "bowl_1-t0"}


def test_a_covering_scan_takes_by_its_place_only_an_item_no_word_of_the_look_took():
    state = make_state()
    state.remember(
        [Detection("ceramic mug", (0.50, 0.00, 0.70), 0.9), Detection("bowl", (0.51, 0.00, 0.70), 0.9)], now_ns=1, coverage=EVERY
    )
    # The scan names the mug in other words, and sees the bowl: the coffee
    # cup takes the mug's id, never the bowl's.
    cup, bowl = state.remember(
        [Detection("coffee cup", (0.505, 0.00, 0.70), 0.9), Detection("bowl", (0.51, 0.00, 0.70), 0.9)], now_ns=2, coverage=EVERY
    )
    assert (cup.item_id, bowl.item_id) == ("ceramic_mug_1-t0", "bowl_1-t0")


def test_a_label_of_stopwords_alone_takes_no_known_item():
    state = make_state()
    state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9)], now_ns=1, coverage=EVERY)
    [it] = state.remember([Detection("it", (0.5, 0.1, 0.7), 0.9)], now_ns=2, coverage=NOTHING)
    assert it.item_id == "it_1-t0"


def test_a_scan_drops_only_the_unseen_items_it_could_name():
    state = make_state()
    vocabulary_scan = Coverage(frozenset({"cup", "bowl"}))
    state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9)], now_ns=1, coverage=vocabulary_scan)
    # Only the words route finds a blue ball: the scan's vocabulary has no such name.
    state.remember([Detection("blue ball", (0.4, -0.1, 0.7), 0.9)], now_ns=2, coverage=NOTHING)
    assert set(state.items) == {"cup_1-t0", "blue_ball_1-t0"}
    state.remember([], now_ns=3, coverage=vocabulary_scan)
    assert set(state.items) == {"blue_ball_1-t0"}
    state.remember([], now_ns=4, coverage=EVERY)
    assert state.items == {}


def test_no_detection_matches_an_item_a_gripper_holds():
    state = make_state()
    state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9)], now_ns=1, coverage=EVERY)
    state.set_held(state.grippers["left_gripper"], "cup_1-t0")
    # Another cup stands where the held one was grabbed.
    seen = state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9)], now_ns=2, coverage=EVERY)
    assert seen[0].item_id == "cup_2-t0"
    assert set(state.items) == {"cup_1-t0", "cup_2-t0"}


def test_a_placed_item_keeps_its_id_where_it_was_put_and_a_dropped_one_is_forgotten():
    state = make_state()
    state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9), Detection("bowl", (0.4, -0.1, 0.7), 0.9)], now_ns=1, coverage=EVERY)
    left, right = state.grippers["left_gripper"], state.grippers["right_gripper"]
    state.set_held(left, "cup_1-t0")
    assert state.clear_held_placed(left, (0.6, -0.2, 0.75), (0.0, 0.0, 0.0, 1.0)) == "cup_1-t0"
    assert not left.holding
    assert state.items["cup_1-t0"].position == (0.6, -0.2, 0.75)
    assert state.items["cup_1-t0"].orientation == (0.0, 0.0, 0.0, 1.0)
    state.set_held(right, "bowl_1-t0")
    assert state.forget_dropped(right) == "bowl_1-t0"
    assert not right.holding and "bowl_1-t0" not in state.items
    with pytest.raises(Refusal, match="^unknown item 'bowl_1-t0': scan or identify it again$"):
        state.item_to_grab("bowl_1-t0")
    # The next look keeps the placed cup's id and gives the bowl a new one,
    # even where it was grabbed.
    seen = state.remember(
        [Detection("cup", (0.61, -0.2, 0.74), 0.9), Detection("bowl", (0.4, -0.1, 0.7), 0.9)], now_ns=2, coverage=EVERY
    )
    assert [item.item_id for item in seen] == ["cup_1-t0", "bowl_2-t0"]


def test_forget_drops_every_item_and_every_hold_and_no_id_comes_back():
    state = make_state()
    state.remember([Detection("mug", (0.4, -0.2, 0.45), 0.9), Detection("bowl", (0.4, 0.1, 0.42), 0.9)], now_ns=1, coverage=EVERY)
    right = state.grippers["right_gripper"]
    state.set_held(right, "mug_1-t0")
    assert state.forget() == ["mug_1-t0", "bowl_1-t0"]
    assert state.items == {}
    assert state.snapshot() == (["left_gripper", "right_gripper"], [False, False], ["", ""])
    with pytest.raises(Refusal, match="^unknown item 'mug_1-t0': scan or identify it again$"):
        state.item_to_grab("mug_1-t0")
    # The freed gripper takes the item on its side again.
    assert state.gripper_for_grab("", (0.4, -0.2, 0.45)) is right
    # A new world's mug where the old one stood gets a new id.
    seen = state.remember([Detection("mug", (0.4, -0.2, 0.45), 0.9)], now_ns=2, coverage=EVERY)
    assert [item.item_id for item in seen] == ["mug_2-t0"]
    assert state.forget() == ["mug_2-t0"]
    assert state.forget() == []


def test_forget_names_a_held_item_that_it_no_longer_knows():
    state = make_state()
    item = state.mint_from_pose((0.4, -0.2, 0.45), None, now_ns=1)
    assert state.forget() == ["item_1-t0"]
    # A grab that ran during the forget ends after it and holds its item.
    state.set_held(state.grippers["right_gripper"], item.item_id)
    assert state.forget() == ["item_1-t0"]
    assert not state.grippers["right_gripper"].holding


def test_ids_carry_the_run_token_so_an_earlier_runs_id_is_refused():
    before, after = make_state("aaaaaa"), make_state("bbbbbb")
    apple = Detection("apple", (0.5, 0.0, 0.7), 0.9)
    old = before.remember([apple], now_ns=1, coverage=EVERY)[0]
    new = after.remember([apple], now_ns=1, coverage=EVERY)[0]
    assert (old.item_id, new.item_id) == ("apple_1-aaaaaa", "apple_1-bbbbbb")
    with pytest.raises(Refusal, match="unknown item 'apple_1-aaaaaa'"):
        after.item(old.item_id)


def test_unknown_items_are_refused_and_pose_grabs_get_an_id():
    state = make_state()
    with pytest.raises(Refusal, match="^unknown item 'cup_9-t0': scan or identify it again$"):
        state.item("cup_9-t0")
    with pytest.raises(Refusal, match="unknown item 'cup_9-t0'"):
        state.item_to_grab("cup_9-t0")
    item = state.mint_from_pose((0.5, 0.0, 0.7), None, now_ns=1)
    assert item.item_id == "item_1-t0"
    assert state.item("item_1-t0") is item
    assert state.item_to_grab("item_1-t0") is item


def test_an_item_a_gripper_holds_cannot_be_grabbed_again():
    state = make_state()
    state.remember([Detection("cup", (0.5, 0.1, 0.7), 0.9), Detection("bowl", (0.4, -0.1, 0.7), 0.9)], now_ns=1, coverage=EVERY)
    assert state.holder_of("cup_1-t0") is None
    state.set_held(state.grippers["left_gripper"], "cup_1-t0")
    assert state.holder_of("cup_1-t0") is state.grippers["left_gripper"]
    with pytest.raises(Refusal, match="item 'cup_1-t0' is held by gripper 'left_gripper'"):
        state.item_to_grab("cup_1-t0")
    assert state.item_to_grab("bowl_1-t0").item_id == "bowl_1-t0"
    state.clear_held_placed(state.grippers["left_gripper"], (0.6, 0.1, 0.7), None)
    assert state.item_to_grab("cup_1-t0").item_id == "cup_1-t0"


def test_labels_become_clean_id_stems():
    state = make_state()
    item = state.remember([Detection("Cheez-It cracker box", (0.5, 0.0, 0.7), 0.9)], now_ns=1, coverage=EVERY)[0]
    assert item.item_id == "cheez_it_cracker_box_1-t0"
    assert item.label == "Cheez-It cracker box"


def test_best_match_prefers_the_exact_label_then_a_word_then_confidence():
    cup = Detection("cup", (0, 0, 0), 0.6)
    red_cup = Detection("red cup", (0, 0, 0), 0.5)
    bowl = Detection("bowl", (0, 0, 0), 0.9)
    assert best_match([cup, red_cup, bowl], "red cup") is red_cup
    assert best_match([cup, bowl], "cup") is cup
    assert best_match([red_cup, bowl], "cup") is red_cup
    assert best_match([cup, red_cup, bowl], "the cup please") is cup
    assert best_match([cup, bowl], "") is bowl
    assert best_match([cup, bowl], "spoon") is None
    assert best_match([], "cup") is None


def test_best_match_never_matches_part_of_a_word_or_a_stopword():
    thermos = Detection("thermos", (0, 0, 0), 0.9)
    candle = Detection("candle", (0, 0, 0), 0.8)
    feather = Detection("feather", (0, 0, 0), 0.7)
    seen = [thermos, candle, feather]
    assert best_match(seen, "the coffee can please") is None
    assert best_match(seen, "can") is None
    assert best_match(seen, "the mug") is None
    assert best_match(seen, "the") is None
    assert best_match(seen, "the candle") is candle


def test_a_description_names_a_label_by_whole_words_stopwords_aside():
    labels = ["coffee can", "cracker box", "candle", "red coffee can"]
    assert named_by("coffee can", labels) == [0]
    assert named_by("Coffee_Can", labels) == [0]
    assert named_by("the coffee can please", labels) == [0, 3]
    assert named_by("can", labels) == [0, 3]
    assert named_by("red can", labels) == [3]
    assert named_by("box can", labels) == []
    assert named_by("coffee tin", labels) == []
    assert named_by("cand", labels) == []
    assert named_by("the", labels) == [] and named_by("  ", labels) == []
    assert normalised("  Red_Coffee   Can ") == "red coffee can"
    assert content_words("the Red coffee can, please") == {"red", "coffee", "can,"}


def test_a_pose_grab_or_a_placement_makes_the_position_a_grasp_point_and_a_look_the_seen_point():
    state = make_state()
    posed = state.mint_from_pose((0.5, 0.0, 0.7), (0.0, 1.0, 0.0, 0.0), now_ns=1)
    assert posed.position_is_grasp_point
    # A look sees the item there: its position is the seen point again,
    # and the orientation the look's.
    [seen] = state.remember([Detection("cup", (0.5, 0.0, 0.72), 0.9)], now_ns=2, coverage=NOTHING)
    assert seen is posed
    assert not posed.position_is_grasp_point and posed.orientation is None and posed.position == (0.5, 0.0, 0.72)
    left = state.grippers["left_gripper"]
    state.set_held(left, posed.item_id)
    state.clear_held_placed(left, (0.6, 0.1, 0.75), None)
    assert posed.position_is_grasp_point and posed.position == (0.6, 0.1, 0.75)
