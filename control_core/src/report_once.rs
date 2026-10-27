//! One report per run of repeated failures.

use std::sync::atomic::{AtomicBool, Ordering};

/// Marks a run of repeated failures, so the first of the run is reported and
/// the rest are suppressed until the run ends: a loop whose every outcome is
/// a failure writes one line for the whole run. Shareable between tasks:
/// every method takes `&self`.
#[derive(Debug, Default)]
pub struct ReportOnce {
    in_run: AtomicBool,
}

impl ReportOnce {
    /// Notes one failure, and whether it is the first of its run: the one to
    /// report.
    pub fn first_of_run(&self) -> bool {
        !self.in_run.swap(true, Ordering::SeqCst)
    }

    /// Ends the run, so the next failure is reported again.
    pub fn clear(&self) {
        self.in_run.store(false, Ordering::SeqCst);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_first_failure_of_a_run_is_reported_and_the_rest_are_not() {
        let report = ReportOnce::default();
        assert!(report.first_of_run());
        assert!(!report.first_of_run());
        assert!(!report.first_of_run());
    }

    #[test]
    fn a_cleared_run_reports_its_next_failure() {
        let report = ReportOnce::default();
        report.clear();
        assert!(report.first_of_run());
        report.clear();
        assert!(report.first_of_run());
        assert!(!report.first_of_run());
    }
}
