"""The checks of the values a model entry holds, which its cameras and its
lidars share: each refuses through `fail`, the caller's refusal, which names
the entry and the sensor."""

from __future__ import annotations


def reject_unknown_keys(obj: dict, allowed: frozenset, what: str, fail) -> None:
    """Refuses a key of `obj` outside `allowed`: a typo, such as `dpeth`."""
    unknown = sorted(set(obj) - allowed)
    if unknown:
        raise fail(f"{what} has unknown key(s) {unknown}; allowed: {sorted(allowed)}")


def is_number(value) -> bool:
    """Whether `value` is an int or a float, a bool not among them."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def positive_int(value, fail, field: str) -> int:
    """`value`, refused unless it is an int above 0, a bool not among them."""
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise fail(f"{field} must be a positive integer, got {value!r}")
    return value
