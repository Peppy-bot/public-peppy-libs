//! The loop that asks another node until it answers, for a node that starts serving
//! only once it has the answer, and the pause after a failed ask.

use std::fmt::Display;
use std::future::Future;
use std::time::Duration;

use crate::latched_log::LatchedLog;
use crate::time::sleep_unless_cancelled;

/// Pause after a failed ask before the loop asks again, so a node that is not up yet
/// is asked once a second.
const RETRY: Duration = Duration::from_secs(1);

/// Asks `ask` until it answers, and gives the answer, or `None` once `cancelled`
/// resolves. A cancel ends the loop at once, while it asks or waits. The first failure
/// of a run is logged at warn level as `"{question} failing, suppressing repeats:
/// {error}"`, and the loop asks again 1 s after each failure.
///
/// A loop spawned onto the runtime takes an `async move` closure whose captures hold
/// no reference, a `&'static str` included: rustc cannot prove `Send` for the future of
/// an async closure that captures one.
pub async fn ask<T, E: Display>(
    question: &str,
    cancelled: impl Future<Output = ()>,
    mut ask: impl AsyncFnMut() -> Result<T, E>,
) -> Option<T> {
    let mut cancelled = std::pin::pin!(cancelled);
    let failures = LatchedLog::warn(question.to_owned());
    loop {
        let asked = tokio::select! {
            biased;
            () = &mut cancelled => return None,
            asked = ask() => asked,
        };
        match asked {
            Ok(answer) => return Some(answer),
            Err(e) => failures.failure(e),
        }
        if !sleep_unless_cancelled(RETRY, &mut cancelled).await {
            return None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::cell::RefCell;

    use tokio::time::Instant;

    use crate::test_support::{Listened, Logged, cancel, cancellation};

    /// The answers of a node, played in order. An ask past the end of the script fails
    /// the test.
    struct Script(RefCell<Vec<Result<u8, &'static str>>>);

    impl Script {
        fn of(answers: Vec<Result<u8, &'static str>>) -> Self {
            Self(RefCell::new(answers.into_iter().rev().collect()))
        }

        fn next(&self) -> Result<u8, &'static str> {
            self.0
                .borrow_mut()
                .pop()
                .expect("an ask past the end of the script")
        }
    }

    #[tokio::test]
    async fn the_first_answer_ends_the_loop() {
        let script = Script::of(vec![Ok(7)]);
        let answer = ask("get_identity", std::future::pending(), async || {
            script.next()
        })
        .await;
        assert_eq!(answer, Some(7));
    }

    #[tokio::test]
    async fn a_cancel_that_already_happened_ends_the_loop_before_it_asks() {
        let asked = Listened::at_most(0);
        let answer = ask("get_identity", std::future::ready(()), async || {
            asked.note();
            Ok::<_, &str>(7)
        })
        .await;
        assert_eq!((answer, asked.count()), (None, 0));
    }

    /// On a paused clock that moves only when every task waits: the bound runs out only
    /// when the loop waits on after the cancel with nothing left to wake it.
    #[tokio::test(start_paused = true)]
    async fn a_cancel_ends_the_loop_while_it_asks() {
        let (sender, cancelled) = cancellation();
        let asked = Listened::at_most(1);
        let asking = ask("get_identity", cancelled, async || {
            asked.note();
            std::future::pending::<Result<u8, &str>>().await
        });
        let cancel_once_asking = async {
            while asked.count() == 0 {
                tokio::task::yield_now().await;
            }
            cancel(&sender);
        };
        let (answer, ()) = tokio::time::timeout(Duration::from_secs(1), async {
            tokio::join!(asking, cancel_once_asking)
        })
        .await
        .expect("the cancel ends the loop");
        assert_eq!(answer, None);
    }

    /// On a paused clock that moves only when every task waits: the pause is the
    /// clock's, not the host's.
    #[tokio::test(start_paused = true)]
    async fn a_failure_is_waited_out_before_the_loop_asks_again() {
        let (_logged, _guard) = Logged::capture();
        let script = Script::of(vec![Err("no reply"), Ok(7)]);
        let asked = Listened::at_most(2);
        let answer = ask("get_identity", std::future::pending(), async || {
            asked.note();
            script.next()
        })
        .await;
        let asks = asked.at.borrow();
        assert_eq!((answer, asks.len()), (Some(7), 2));
        assert!(asks[1] - asks[0] >= RETRY);
    }

    #[tokio::test(start_paused = true)]
    async fn a_cancel_ends_the_loop_while_it_waits_out_a_failure() {
        let (_logged, _guard) = Logged::capture();
        let (sender, cancelled) = cancellation();
        let asked = Listened::at_most(1);
        let start = Instant::now();
        let answer = ask("get_identity", cancelled, async || {
            asked.note();
            cancel(&sender);
            Err::<u8, _>("no reply")
        })
        .await;
        assert_eq!((answer, asked.count()), (None, 1));
        assert!(Instant::now() - start < RETRY);
    }

    /// The asks fail, and the loop logs the first failure alone, naming the question.
    #[tokio::test(start_paused = true)]
    async fn the_first_failure_of_a_run_is_logged() {
        let (logged, _guard) = Logged::capture();
        let script = Script::of(vec![Err("no reply"), Err("no reply"), Ok(7)]);
        ask("robot_identity", std::future::pending(), async || {
            script.next()
        })
        .await;
        assert_eq!(
            logged.lines_holding("robot_identity failing, suppressing repeats: no reply"),
            1
        );
    }
}
