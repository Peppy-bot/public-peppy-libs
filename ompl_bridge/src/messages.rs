//! OMPL's warnings and errors, which `bridge.cpp` hands to the caller's `tracing` subscriber
//! under the target `ompl`, as MoveIt's OMPL planner manager hands them to its logger. A
//! panic in the subscriber ends here, as one in the caller's functions does.

use crate::callbacks::caught;

/// Logs OMPL's warning `text` as a warning.
pub(crate) fn log_warning(text: &[u8]) {
    logged(text, |text| tracing::warn!(target: "ompl", "{text}"));
}

/// Logs OMPL's error `text` as an error.
pub(crate) fn log_error(text: &[u8]) {
    logged(text, |text| tracing::error!(target: "ompl", "{text}"));
}

/// Hands `text`, its bytes read as UTF-8, to `log`, under [`caught`].
fn logged(text: &[u8], log: impl FnOnce(&str)) {
    let text = String::from_utf8_lossy(text);
    let _ = caught(|| {
        log(&text);
        Ok(())
    });
}
