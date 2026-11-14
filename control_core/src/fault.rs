//! The fault that ends a node after its setup has returned, the error the node exits with
//! for it, and the task whose panic raises it.

use std::error::Error;
use std::future::Future;
use std::sync::{Arc, Mutex};

use tracing::error;

use crate::sync::lock_ignoring_poison;

/// Why a node ended: the error of the first fault raised, with each error it comes from.
pub type Reason = Box<dyn Error + Send + Sync>;

/// The fault that ends a node after its setup has returned: the first raise records its
/// reason and stops the node, and the node's `main` exits with the reason it takes back.
/// Clones share one fault.
#[derive(Clone, Debug, Default)]
pub struct Fault(Arc<Mutex<Option<Reason>>>);

impl Fault {
    /// Logs `reason` at error level, with each error it comes from, and records it unless
    /// a fault came first, then calls `stop`, which ends the node.
    pub fn raise(&self, reason: impl Into<Reason>, stop: impl FnOnce()) {
        let reason = reason.into();
        error!("{}", with_sources(reason.as_ref()));
        lock_ignoring_poison(&self.0).get_or_insert(reason);
        stop();
    }

    /// The reason of the first fault raised, which the node exits with: none when no
    /// fault was raised.
    pub fn take(&self) -> Option<Reason> {
        lock_ignoring_poison(&self.0).take()
    }

    /// Runs `task` on a task of its own until it ends. A panic raises this fault, naming
    /// `what` and the panic, and calls `stop`; a task that returns, or that the runtime
    /// cancels as it shuts down, raises nothing.
    pub async fn raise_on_panic(
        self,
        what: &'static str,
        task: impl Future<Output = ()> + Send + 'static,
        stop: impl FnOnce(),
    ) {
        let Err(ended) = tokio::spawn(task).await else {
            return;
        };
        if ended.is_panic() {
            self.raise(Panicked { what, panic: ended }, stop);
        }
    }
}

/// A task of the node that panicked: `what` names the task, and the panic says why.
#[derive(Debug, thiserror::Error)]
#[error("{what} stopped")]
struct Panicked {
    what: &'static str,
    #[source]
    panic: tokio::task::JoinError,
}

/// `error` and each error it comes from, joined by ": ".
fn with_sources(error: &(dyn Error + 'static)) -> String {
    std::iter::successors(Some(error), |&error| error.source())
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(": ")
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::cell::Cell;

    use crate::test_support::Logged;

    #[derive(Debug, thiserror::Error)]
    #[error("the robot's initializer serves so101")]
    struct Undriven(#[source] std::io::Error);

    #[test]
    fn a_raise_logs_the_reason_with_its_sources_and_stops_the_node() {
        let (logged, _guard) = Logged::capture();
        let fault = Fault::default();
        let stopped = Cell::new(false);
        fault.raise(
            Undriven(std::io::Error::other("it drives openarm_v1 and openarm_v2")),
            || stopped.set(true),
        );
        assert!(stopped.get());
        assert_eq!(
            logged.lines_holding(
                "the robot's initializer serves so101: it drives openarm_v1 and openarm_v2"
            ),
            1
        );
        let reason = fault.take().expect("the raised reason");
        assert_eq!(reason.to_string(), "the robot's initializer serves so101");
        assert!(reason.source().is_some(), "the reason keeps its source");
    }

    #[test]
    fn the_first_fault_is_the_reason_and_every_raise_stops_the_node() {
        let (_logged, _guard) = Logged::capture();
        let fault = Fault::default();
        let stops = Cell::new(0);
        fault.raise("the panel stopped", || stops.set(stops.get() + 1));
        fault
            .clone()
            .raise("the reader stopped", || stops.set(stops.get() + 1));
        assert_eq!(stops.get(), 2);
        assert_eq!(
            fault.take().map(|reason| reason.to_string()).as_deref(),
            Some("the panel stopped")
        );
        assert!(fault.take().is_none(), "the reason is taken once");
    }

    #[test]
    fn a_node_that_raised_no_fault_ends_with_none() {
        assert!(Fault::default().take().is_none());
    }

    #[tokio::test]
    async fn a_task_that_panics_raises_the_fault_naming_the_task_and_the_panic() {
        let (logged, _guard) = Logged::capture();
        let fault = Fault::default();
        let stopped = Cell::new(false);
        fault
            .clone()
            .raise_on_panic("the bring-up", async { panic!("no reply on can0") }, || {
                stopped.set(true)
            })
            .await;
        assert!(stopped.get(), "the panic stops the node");
        let reason = fault.take().expect("the panic raises the fault");
        assert_eq!(reason.to_string(), "the bring-up stopped");
        let panic = reason.source().expect("the panic").to_string();
        assert!(
            panic.contains("panicked with message \"no reply on can0\""),
            "{panic}"
        );
        assert_eq!(
            logged.lines_holding(&format!("the bring-up stopped: {panic}")),
            1
        );
    }

    #[tokio::test]
    async fn a_task_that_returns_raises_nothing() {
        let fault = Fault::default();
        let stopped = Cell::new(false);
        fault
            .clone()
            .raise_on_panic("the bring-up", async {}, || stopped.set(true))
            .await;
        assert!(!stopped.get());
        assert!(fault.take().is_none());
    }
}
