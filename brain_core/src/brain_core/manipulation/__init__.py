"""Manipulation: the manipulators every brain can run.

- `NoneManipulator`, `manipulation_model: "none"`: moves nothing and
  refuses every sequence.
- `ScriptedManipulator`, `manipulation_model: "scripted"`: grabs from
  above through a fixed sequence of moves, with the grasp a
  `CentreProposer` gives (`grasp` holds the geometry).
"""
