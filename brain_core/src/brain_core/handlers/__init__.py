"""One module per exposed member. Each reads the goal, applies the
contract's name and lane rules through the brain's core, looks through the
brain (`Brain.look`) or calls the manipulator, and returns the result
fields; the brain ends the goal and logs its end. A handler reads its
backend once, at the start of its body, and uses that one to the end. The
handler of an action with a feedback topic makes the reporter of its goal
context (`stages.reporter_of`) and hands it to the look or, bound to the
labels of the goal's items, to the manipulator, so the goal publishes each
stage as it starts once the rules of its names passed.

A member that refuses a goal before it accepts it has an `admit` too: the
checks the brain makes at once, and the goal the handler then runs
(`brain.Decider`). item_perception and brain_model refuse by failure kinds
there; grab_item and place_item parse their target there and place it in
the robot frame. The goal of a member with no `admit` is its request
data.

The `ZERO` dict in each action module is the fields the contract says are
zero or empty for a goal with no result of its own: a cancelled goal, and
an item_manipulation goal whose success is false. A cancelled scan_items
also gives the robot frame and the time of the cancel
(`Brain.run_guarded`'s `stamped`).
"""
