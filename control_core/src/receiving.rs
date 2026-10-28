//! The loop that receives one subscription for the life of a node, and the
//! pause after a receive error.

use std::fmt::Display;
use std::future::Future;
use std::ops::ControlFlow;
use std::time::Duration;

use crate::latched_log::LatchedLog;
use crate::time::sleep_unless_cancelled;

/// Pause after a receive error before the loop receives again, so a broken
/// subscription cannot hot-spin a loop.
const RECEIVE_ERROR_BACKOFF: Duration = Duration::from_millis(100);

/// Receives `subscription` one message at a time until `cancelled`
/// resolves, the subscription closes, or `on_message` breaks:
/// `next_message` receives the next message, or `None` once the subscription
/// has closed, and `on_message` handles it. A cancel ends the loop at once,
/// while it waits for a message or waits out an error. The first error of a
/// run is logged at error level as `"{subscription} receive failing,
/// suppressing repeats: {error}"`, a received message ends the run, and each
/// error is waited out for 100 ms before the loop receives again.
///
/// Both closures are async so `next_message` can hold the subscription it
/// owns mutably across the receive. A loop spawned onto the runtime takes
/// `async move` closures whose captures hold no reference, a `&'static str`
/// included: rustc cannot prove `Send` for the future of an async closure
/// that captures one.
pub async fn receive<M, E: Display>(
    subscription: &str,
    cancelled: impl Future<Output = ()>,
    mut next_message: impl AsyncFnMut() -> Result<Option<M>, E>,
    mut on_message: impl AsyncFnMut(M) -> ControlFlow<()>,
) {
    let mut cancelled = std::pin::pin!(cancelled);
    let receive_errors = LatchedLog::error(format!("{subscription} receive"));
    loop {
        let received = tokio::select! {
            biased;
            () = &mut cancelled => return,
            received = next_message() => received,
        };
        let message = match received {
            Ok(Some(message)) => message,
            Ok(None) => return,
            Err(e) => {
                receive_errors.failure(e);
                if !sleep_unless_cancelled(RECEIVE_ERROR_BACKOFF, &mut cancelled).await {
                    return;
                }
                continue;
            }
        };
        receive_errors.success();
        if on_message(message).await.is_break() {
            return;
        }
    }
}

/// A receive error from one of the subscriptions a loop selects over, with
/// the tag that names that subscription: `"{tag}: {error}"`.
#[derive(Debug, PartialEq)]
pub struct Tagged<T, E> {
    pub tag: T,
    pub error: E,
}

impl<T: Display, E: Display> Display for Tagged<T, E> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.tag, self.error)
    }
}

/// One receive on the subscription `tag` names, with `tag` carried on its
/// message or on its error, so a `next_message` that selects over several
/// subscriptions hands [`receive`] which one each came from.
pub fn tagged<T, M, E>(
    tag: T,
    received: Result<Option<M>, E>,
) -> Result<Option<(T, M)>, Tagged<T, E>> {
    match received {
        Ok(message) => Ok(message.map(|message| (tag, message))),
        Err(error) => Err(Tagged { tag, error }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::cell::RefCell;

    use tokio::time::Instant;

    use crate::test_support::{Listened, Logged, cancel, cancellation};

    /// A subscription's receives, played in order. A receive past the end of
    /// the script fails the test.
    struct Script<M>(RefCell<Vec<Result<Option<M>, &'static str>>>);

    impl<M> Script<M> {
        fn of(receives: Vec<Result<Option<M>, &'static str>>) -> Self {
            Self(RefCell::new(receives.into_iter().rev().collect()))
        }

        fn next(&self) -> Result<Option<M>, &'static str> {
            self.0
                .borrow_mut()
                .pop()
                .expect("a receive past the end of the script")
        }
    }

    #[tokio::test]
    async fn a_cancel_that_already_happened_ends_the_loop_before_it_receives() {
        let listened = Listened::at_most(0);
        receive(
            "cancelled",
            std::future::ready(()),
            async || {
                listened.note();
                Ok::<_, String>(Some(()))
            },
            async |()| ControlFlow::Continue(()),
        )
        .await;
        assert_eq!(listened.count(), 0);
    }

    /// On a paused clock that moves only when every task waits: the bound
    /// runs out only when the loop waits on after the cancel with nothing
    /// left to wake it.
    #[tokio::test(start_paused = true)]
    async fn a_cancel_ends_the_loop_while_it_waits_for_a_message() {
        let (sender, cancelled) = cancellation();
        let listened = Listened::at_most(1);
        let receiving = receive(
            "waiting",
            cancelled,
            async || {
                listened.note();
                std::future::pending::<Result<Option<()>, String>>().await
            },
            async |()| ControlFlow::Continue(()),
        );
        let cancel_once_listening = async {
            while listened.count() == 0 {
                tokio::task::yield_now().await;
            }
            cancel(&sender);
        };
        tokio::time::timeout(Duration::from_secs(1), async {
            tokio::join!(receiving, cancel_once_listening)
        })
        .await
        .expect("the cancel ends the loop");
        assert_eq!(listened.count(), 1);
    }

    #[tokio::test]
    async fn messages_are_handled_in_order_until_the_subscription_closes() {
        let script = Script::of(vec![Ok(Some(1)), Ok(Some(2)), Ok(None)]);
        let mut handled = Vec::new();
        receive(
            "closing",
            std::future::pending(),
            async || script.next(),
            async |message| {
                handled.push(message);
                ControlFlow::Continue(())
            },
        )
        .await;
        assert_eq!(handled, [1, 2]);
    }

    #[tokio::test]
    async fn a_handler_that_breaks_ends_the_loop() {
        let script = Script::of(vec![Ok(Some(1)), Ok(Some(2))]);
        let mut handled = Vec::new();
        receive(
            "breaking",
            std::future::pending(),
            async || script.next(),
            async |message| {
                handled.push(message);
                ControlFlow::Break(())
            },
        )
        .await;
        assert_eq!(handled, [1]);
    }

    /// On a paused clock that moves only when every task waits: the pause
    /// is the clock's, not the host's.
    #[tokio::test(start_paused = true)]
    async fn an_error_is_waited_out_before_the_loop_receives_again() {
        let (_logged, _guard) = Logged::capture();
        let script = Script::of(vec![Err("the session is closed"), Ok(None)]);
        let listened = Listened::at_most(2);
        receive(
            "failing",
            std::future::pending(),
            async || {
                listened.note();
                script.next()
            },
            async |()| ControlFlow::Continue(()),
        )
        .await;
        let listens = listened.at.borrow();
        assert_eq!(listens.len(), 2);
        assert!(listens[1] - listens[0] >= RECEIVE_ERROR_BACKOFF);
    }

    #[tokio::test(start_paused = true)]
    async fn a_cancel_ends_the_loop_while_it_waits_out_an_error() {
        let (_logged, _guard) = Logged::capture();
        let (sender, cancelled) = cancellation();
        let listened = Listened::at_most(1);
        let start = Instant::now();
        receive(
            "failing",
            cancelled,
            async || {
                listened.note();
                cancel(&sender);
                Err::<Option<()>, _>("the session is closed")
            },
            async |()| ControlFlow::Continue(()),
        )
        .await;
        assert_eq!(listened.count(), 1);
        assert!(Instant::now() - start < RECEIVE_ERROR_BACKOFF);
    }

    /// Receives 1, 2, 4 and 5 fail and receive 3 brings a message, so the
    /// errors make two runs.
    #[tokio::test(start_paused = true)]
    async fn the_first_error_of_each_run_is_logged() {
        let (logged, _guard) = Logged::capture();
        let script = Script::of(vec![
            Err("the session is closed"),
            Err("the session is closed"),
            Ok(Some(())),
            Err("the session is closed"),
            Err("the session is closed"),
            Ok(None),
        ]);
        receive(
            "joint_setpoints",
            std::future::pending(),
            async || script.next(),
            async |()| ControlFlow::Continue(()),
        )
        .await;
        assert_eq!(
            logged.lines_holding(
                "joint_setpoints receive failing, suppressing repeats: the session is closed"
            ),
            2
        );
    }

    #[test]
    fn a_tagged_receive_carries_its_tag_on_the_message_or_the_error() {
        assert_eq!(
            tagged("left", Ok::<_, &str>(Some(1))),
            Ok(Some(("left", 1)))
        );
        assert_eq!(tagged("left", Ok::<Option<u8>, &str>(None)), Ok(None));
        let error = tagged("right", Err::<Option<u8>, _>("the message does not decode"))
            .expect_err("an error stays an error");
        assert_eq!(
            error,
            Tagged {
                tag: "right",
                error: "the message does not decode"
            }
        );
        assert_eq!(error.to_string(), "right: the message does not decode");
    }

    /// A node spawns each loop onto the runtime's worker threads, which
    /// takes a `Send` future, with closures that borrow what they own across
    /// an await.
    #[tokio::test]
    async fn the_loop_runs_as_a_spawned_task() {
        let (_messages, mut incoming) = tokio::sync::mpsc::channel::<u32>(1);
        let (handled, _forwarded) = tokio::sync::mpsc::channel::<u32>(1);
        let receiving = tokio::spawn(receive(
            "spawned",
            std::future::ready(()),
            async move || Ok::<_, String>(incoming.recv().await),
            async move |message| match handled.send(message).await {
                Ok(()) => ControlFlow::Continue(()),
                Err(_) => ControlFlow::Break(()),
            },
        ));
        receiving.await.expect("the loop ends at the cancel");
    }
}
