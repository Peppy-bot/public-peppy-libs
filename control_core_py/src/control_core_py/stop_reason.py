"""The reason a robot stop gives each move it ends, as robot_stop's stop names it: the stop, and the
reason of the stop request. control_core::stop_reason in Python."""


def stop_reason(request_reason: str) -> str:
    """The reason every move a stop for `request_reason` ends is cancelled for:
    `stopped: <request_reason>`, and `stopped` for an empty request reason."""
    if not request_reason:
        return "stopped"
    return f"stopped: {request_reason}"
