//! The reason a robot stop gives each move it ends, as robot_stop's stop
//! names it: the stop, and the reason of the stop request.

/// The reason every move a stop for `request_reason` ends is cancelled for:
/// `stopped: <request_reason>`, and `stopped` for an empty request reason.
pub fn stop_reason(request_reason: &str) -> String {
    if request_reason.is_empty() {
        return "stopped".to_string();
    }
    format!("stopped: {request_reason}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_stop_reason_names_the_stop_and_the_reason_of_its_request() {
        assert_eq!(
            stop_reason("the operator asked"),
            "stopped: the operator asked"
        );
        assert_eq!(stop_reason(""), "stopped");
    }
}
