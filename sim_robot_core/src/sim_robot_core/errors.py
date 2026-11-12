"""The error of a model an engine cannot stand: an entry that does not read,
a model the engine has no entry for, or an entry that does not fit the
description of its robot."""


class ModelEntryError(ValueError):
    """A model an engine cannot stand: an entry that does not read, a model
    the engine has no entry for, or an entry that does not fit the
    description of its robot. `str()` names the entry or the model, and
    why."""
