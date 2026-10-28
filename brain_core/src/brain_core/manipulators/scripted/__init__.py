"""The scripted manipulator, `manipulation_model: "scripted"`: a grab, a
drop and a place through a fixed sequence of moves (`manipulator`), the
grasp it asks for (`centre_proposer`), the geometry of its moves (`grasp`)
and the port of the aligner a node may put between the move above the
item and the descent (`aligning`)."""

from .centre_proposer import CentreProposer
from .manipulator import ScriptedManipulator

__all__ = ["CentreProposer", "ScriptedManipulator"]
