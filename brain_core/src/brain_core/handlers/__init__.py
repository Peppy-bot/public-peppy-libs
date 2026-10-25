"""One module per exposed member. Each reads the goal, applies the
contract's name and lane rules through the brain's core, looks through the
brain (`Brain.look`) or calls the manipulator, and returns the result
fields; the brain ends the goal and logs its end. A handler reads its
backend once, at the start of its body, and uses that one to the end.

A member whose contract reports by failure kinds, item_perception and
brain_model, has a `decide` too: the checks the brain makes at once,
before it accepts the goal (`brain.decider`).

The `ZERO` dict in each action module is the fields the contract says are
zero or empty for a goal with no result of its own: a cancelled goal, and
an item_manipulation goal whose success is false.
"""
