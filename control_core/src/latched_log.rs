//! A failure logged once per run of repeated failures.

use std::borrow::Cow;
use std::fmt::Display;

use tracing::{error, warn};

use crate::report_once::ReportOnce;

/// The level a [`LatchedLog`] writes its line at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Severity {
    Warn,
    Error,
}

/// One line when an operation starts failing, `"{context} failing,
/// suppressing repeats: {error}"`, silence while it keeps failing, re-armed
/// by the first success. Shareable between tasks: every method takes
/// `&self`.
pub struct LatchedLog {
    context: Cow<'static, str>,
    severity: Severity,
    run: ReportOnce,
}

impl LatchedLog {
    /// A latch that warns, naming `context`, the operation it watches: a
    /// fixed name, or one built at runtime.
    pub fn warn(context: impl Into<Cow<'static, str>>) -> Self {
        Self::at(Severity::Warn, context)
    }

    /// A latch that logs at error level, naming `context`.
    pub fn error(context: impl Into<Cow<'static, str>>) -> Self {
        Self::at(Severity::Error, context)
    }

    fn at(severity: Severity, context: impl Into<Cow<'static, str>>) -> Self {
        Self {
            context: context.into(),
            severity,
            run: ReportOnce::default(),
        }
    }

    /// Notes a failure: logs the first of a run, quietly after.
    pub fn failure(&self, error: impl Display) {
        if !self.run.first_of_run() {
            return;
        }
        match self.severity {
            Severity::Warn => warn!("{} failing, suppressing repeats: {error}", self.context),
            Severity::Error => error!("{} failing, suppressing repeats: {error}", self.context),
        }
    }

    /// Notes a success, which ends the run.
    pub fn success(&self) {
        self.run.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::test_support::Logged;

    #[test]
    fn a_run_latches_until_a_success_rearms_it() {
        let (_logged, _guard) = Logged::capture();
        let latch = LatchedLog::error("simulation video_stream receive");
        latch.failure("down");
        assert!(!latch.run.first_of_run(), "the failure opened the run");
        latch.success();
        assert!(latch.run.first_of_run(), "the success ended it");
    }

    #[test]
    fn a_latch_names_a_context_built_at_runtime_at_its_level() {
        let side = "left";
        let latch = LatchedLog::warn(format!("{side} arm command publish"));
        assert_eq!(latch.context, "left arm command publish");
        assert_eq!(latch.severity, Severity::Warn);
    }
}
