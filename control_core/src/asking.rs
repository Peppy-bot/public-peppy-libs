//! The loop that asks another node until it answers, for a node that starts serving
//! only once it has the answer, and the pause after a failed ask; the wait until another
//! node answers that it is ready; and the ask of a loop of the node's own, which answers
//! each request it takes on a channel, within a timeout.

use std::fmt::Display;
use std::future::Future;
use std::time::Duration;

use tokio::sync::{mpsc, oneshot};

use crate::latched_log::LatchedLog;
use crate::time::sleep_unless_cancelled;

/// Pause after a failed ask before the loop asks again, so a node that is not up yet
/// is asked once a second.
const RETRY: Duration = Duration::from_secs(1);

/// Pause after a failed ask or a not-ready answer before the wait until another node
/// is ready asks again, so a node that is not ready is asked twice a second.
const READY_RETRY: Duration = Duration::from_millis(500);

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
    ask: impl AsyncFnMut() -> Result<T, E>,
) -> Option<T> {
    ask_pausing(question, cancelled, RETRY, ask, |_| true).await
}

/// [`ask`] until an answer that `settles` takes, pausing `pause` after each failure and
/// after each answer it does not take. An answer it does not take ends a run of failures
/// and is logged at no level.
async fn ask_pausing<T, E: Display>(
    question: &str,
    cancelled: impl Future<Output = ()>,
    pause: Duration,
    mut ask: impl AsyncFnMut() -> Result<T, E>,
    settles: impl Fn(&T) -> bool,
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
            Ok(answer) if settles(&answer) => return Some(answer),
            Ok(_) => failures.success(),
            Err(e) => failures.failure(e),
        }
        if !sleep_unless_cancelled(pause, &mut cancelled).await {
            return None;
        }
    }
}

/// Asks `ask_ready` whether another node is ready until it answers that it is, as [`ask`]
/// asks until an answer, asking again 0.5 s after each failed ask and each answer of not
/// ready. A failed ask is logged as [`ask`] logs it. An answer of not ready, which a node
/// gives while it comes up, is logged at no level and ends a run of failures. True once the
/// node is ready, false once `cancelled` resolves.
pub async fn ask_until_ready<E: Display>(
    question: &str,
    cancelled: impl Future<Output = ()>,
    ask_ready: impl AsyncFnMut() -> Result<bool, E>,
) -> bool {
    ask_pausing(question, cancelled, READY_RETRY, ask_ready, |ready| *ready)
        .await
        .is_some()
}

/// Hands `request` to the loop that `requests` feeds, the loop `name` names, and gives the answer
/// the loop sends on `answer`, waiting at most `within` for both: for its place in the loop's
/// queue and for the answer. The error says, in words that name the loop, what went wrong:
/// - the loop is not running;
/// - it dropped the request;
/// - it did not take and answer the request within `within`, which leaves the request
///   unqueued when the queue had no place for it, and queued when it had one; the error
///   then says to call again and to check the node's log for what holds the loop up.
pub async fn ask_loop<R, T>(
    name: &str,
    requests: &mpsc::Sender<R>,
    request: R,
    answer: oneshot::Receiver<T>,
    within: Duration,
) -> Result<T, String> {
    let asked = async {
        requests
            .send(request)
            .await
            .map_err(|_| format!("the {name} is not running"))?;
        answer
            .await
            .map_err(|_| format!("the {name} dropped the request"))
    };
    tokio::time::timeout(within, asked)
        .await
        .unwrap_or_else(|_| Err(not_answered(name, within)))
}

/// The error of an ask that the loop `name` names did not answer within `within`, with what its
/// caller does next.
fn not_answered(name: &str, within: Duration) -> String {
    format!(
        "the {name} did not answer within {within:?}: call again, and check the node's log for \
         what holds up the {name}"
    )
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

    /// A loop whose one place in its queue holds an earlier request: the ask fails naming its
    /// time, and the loop never takes it.
    #[tokio::test(start_paused = true)]
    async fn an_ask_the_loop_does_not_take_within_its_time_fails_naming_it() {
        let (requests, mut queued) = mpsc::channel(1);
        requests.try_send("earlier").expect("a free queue");
        let (_reply, answer) = oneshot::channel::<u8>();
        let within = Duration::from_millis(10);
        assert_eq!(
            ask_loop("control loop", &requests, "stop", answer, within).await,
            Err(format!(
                "the control loop did not answer within {within:?}: call again, and check the \
                 node's log for what holds up the control loop"
            ))
        );
        assert_eq!(queued.try_recv(), Ok("earlier"));
        assert!(queued.try_recv().is_err(), "the timed-out ask never queued");
    }

    /// A loop that queues the request and never answers it: the ask fails naming its time,
    /// and the request stays queued.
    #[tokio::test(start_paused = true)]
    async fn an_ask_the_loop_queues_and_never_answers_fails_naming_its_time() {
        let (requests, mut queued) = mpsc::channel(1);
        let (_reply, answer) = oneshot::channel::<u8>();
        let within = Duration::from_millis(10);
        let asked = tokio::time::timeout(
            2 * within,
            ask_loop("control loop", &requests, "stop", answer, within),
        )
        .await
        .expect("the ask ends within its time");
        assert_eq!(asked, Err(not_answered("control loop", within)));
        assert_eq!(queued.try_recv(), Ok("stop"), "the request was queued");
    }

    /// A loop that takes the request and answers it gives its answer; one that is gone, or
    /// that drops the request, gives the error that says so.
    #[tokio::test]
    async fn an_ask_gives_the_loops_answer_or_says_why_it_has_none() {
        let within = Duration::from_secs(30);
        let (requests, mut queued) = mpsc::channel::<oneshot::Sender<u8>>(1);
        let (reply, answer) = oneshot::channel();
        let answering = tokio::spawn(async move {
            let reply = queued.recv().await.expect("the request");
            reply.send(7).expect("the asker waits");
            let dropped = queued.recv().await.expect("the second request");
            drop(dropped);
        });
        assert_eq!(
            ask_loop("coordinator", &requests, reply, answer, within).await,
            Ok(7)
        );
        let (reply, answer) = oneshot::channel();
        assert_eq!(
            ask_loop("coordinator", &requests, reply, answer, within).await,
            Err("the coordinator dropped the request".to_string())
        );
        answering.await.expect("the loop ends");
        let (reply, answer) = oneshot::channel();
        assert_eq!(
            ask_loop("coordinator", &requests, reply, answer, within).await,
            Err("the coordinator is not running".to_string())
        );
    }

    /// The pause of the wait until another node is ready, as its doc states it.
    const HALF_A_SECOND: Duration = Duration::from_millis(500);

    /// A node that answers not ready, then ready: the wait asks again after the pause and
    /// ends once it is ready, and logs nothing for the answer of not ready.
    #[tokio::test(start_paused = true)]
    async fn the_wait_until_ready_asks_again_until_the_node_is_ready() {
        let (logged, _guard) = Logged::capture();
        let answers = RefCell::new(vec![Ok::<_, &str>(true), Ok(false)]);
        let asked = Listened::at_most(2);
        let ready = ask_until_ready("robot_init", std::future::pending(), async || {
            asked.note();
            answers.borrow_mut().pop().expect("an ask past the answers")
        })
        .await;
        assert!(ready);
        let asks = asked.at.borrow();
        assert_eq!(asks.len(), 2);
        assert_eq!(asks[1] - asks[0], HALF_A_SECOND);
        assert_eq!(logged.lines_holding("robot_init"), 0);
    }

    /// A node whose first ask fails, as a service with no producer yet fails: the wait logs
    /// the failure, asks again after the pause and ends once the node is ready, so a failed
    /// ask opens no gate.
    #[tokio::test(start_paused = true)]
    async fn a_failed_ask_of_the_wait_until_ready_is_logged_and_asked_again() {
        let (logged, _guard) = Logged::capture();
        let answers = RefCell::new(vec![Ok(true), Err("unbound")]);
        let asked = Listened::at_most(2);
        let ready = ask_until_ready("robot_init", std::future::pending(), async || {
            asked.note();
            answers.borrow_mut().pop().expect("an ask past the answers")
        })
        .await;
        assert!(ready);
        let asks = asked.at.borrow();
        assert_eq!(asks.len(), 2);
        assert_eq!(asks[1] - asks[0], HALF_A_SECOND);
        assert_eq!(
            logged.lines_holding("robot_init failing, suppressing repeats: unbound"),
            1
        );
    }

    /// Asks that keep failing: a cancel during the pause ends the wait unready.
    #[tokio::test(start_paused = true)]
    async fn a_cancel_ends_the_wait_until_ready_while_the_asks_fail() {
        let (_logged, _guard) = Logged::capture();
        let (sender, cancelled) = cancellation();
        let asked = Listened::at_most(2);
        let ready = ask_until_ready("robot_init", cancelled, async || {
            if asked.note() == 2 {
                cancel(&sender);
            }
            Err::<bool, _>("unbound")
        })
        .await;
        assert!(!ready);
        assert_eq!(asked.count(), 2);
    }

    #[tokio::test]
    async fn a_cancel_ends_the_wait_until_ready() {
        let ready = ask_until_ready("robot_init", std::future::ready(()), async || {
            Ok::<_, &str>(true)
        })
        .await;
        assert!(!ready);
    }
}
