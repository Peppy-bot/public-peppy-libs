//! The caller's flag that ends a plan, at any part of it.

use std::sync::atomic::{AtomicBool, Ordering};

/// Whether the caller's flag `stop`, if any, is set.
pub(crate) fn is_set(stop: Option<&AtomicBool>) -> bool {
    stop.is_some_and(|stop| stop.load(Ordering::Relaxed))
}
