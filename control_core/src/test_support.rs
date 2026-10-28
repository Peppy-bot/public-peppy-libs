//! What the loop tests share: a record of when a scripted loop listened, the
//! cancel a scripted step fires, and the text logged while a test runs.

use std::cell::{Cell, RefCell};
use std::future::Future;
use std::sync::{Arc, Mutex};

use tokio::sync::oneshot;
use tokio::time::Instant;

use crate::sync::lock_ignoring_poison;

/// When a scripted loop listened for each request or message, on the test's
/// clock. A listen past the most the test expects fails the test at once, so
/// a loop that runs on past a cancel ends the test with a failure.
pub(crate) struct Listened {
    most: usize,
    pub(crate) at: RefCell<Vec<Instant>>,
}

impl Listened {
    pub(crate) fn at_most(most: usize) -> Self {
        Self {
            most,
            at: RefCell::default(),
        }
    }

    /// Notes one listen and gives how many there were.
    pub(crate) fn note(&self) -> usize {
        let mut listens = self.at.borrow_mut();
        listens.push(Instant::now());
        assert!(
            listens.len() <= self.most,
            "listen {} is past the {} the test expects",
            listens.len(),
            self.most
        );
        listens.len()
    }

    pub(crate) fn count(&self) -> usize {
        self.at.borrow().len()
    }
}

/// The sender that cancels a loop, held so a scripted step can fire it, and
/// the future the loop watches.
pub(crate) fn cancellation() -> (Cell<Option<oneshot::Sender<()>>>, impl Future<Output = ()>) {
    let (cancel, cancelled) = oneshot::channel();
    (Cell::new(Some(cancel)), async move {
        let _ = cancelled.await;
    })
}

pub(crate) fn cancel(sender: &Cell<Option<oneshot::Sender<()>>>) {
    if let Some(sender) = sender.take() {
        let _ = sender.send(());
    }
}

/// The text logged on this thread while the capture's guard lives. Every
/// test that reaches a log site a capture counts runs under a capture: a site
/// first reached on a thread with no subscriber can cache itself as never
/// logged, and a counting test then reads no line.
#[derive(Clone, Default)]
pub(crate) struct Logged(Arc<Mutex<Vec<u8>>>);

impl std::io::Write for Logged {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        lock_ignoring_poison(&self.0).extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl Logged {
    pub(crate) fn capture() -> (Self, tracing::subscriber::DefaultGuard) {
        let logged = Self::default();
        let writer = logged.clone();
        let guard = tracing::subscriber::set_default(
            tracing_subscriber::fmt()
                .with_writer(move || writer.clone())
                .with_ansi(false)
                .finish(),
        );
        (logged, guard)
    }

    pub(crate) fn lines_holding(&self, text: &str) -> usize {
        String::from_utf8_lossy(&lock_ignoring_poison(&self.0))
            .lines()
            .filter(|line| line.contains(text))
            .count()
    }
}
