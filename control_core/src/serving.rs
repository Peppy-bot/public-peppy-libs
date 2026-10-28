//! The loop that serves one service for the life of a node, and the pause
//! after a runtime error.

use std::fmt::Display;
use std::future::Future;
use std::time::Duration;

use crate::latched_log::LatchedLog;
use crate::time::sleep_unless_cancelled;

/// Pause after a service error before the loop listens again, so a broken
/// transport cannot hot-spin a loop.
const RETRY_BACKOFF: Duration = Duration::from_secs(1);

/// Serves `service` one request at a time until `cancelled` resolves:
/// `next_request` listens for the next request and answers it (a generated
/// `handle_next_request`). A cancel ends the loop at once, while it waits for
/// a request or waits out an error. The first error of a run is logged at
/// error level as `"{service} failing, suppressing repeats: {error}"`, a
/// served request ends the run, and each error is waited out for 1 s before
/// the loop listens again.
pub async fn serve<F, E>(
    service: &str,
    cancelled: impl Future<Output = ()>,
    mut next_request: impl FnMut() -> F,
) where
    F: Future<Output = Result<(), E>>,
    E: Display,
{
    let mut cancelled = std::pin::pin!(cancelled);
    let service_errors = LatchedLog::error(service.to_owned());
    loop {
        let served = tokio::select! {
            biased;
            () = &mut cancelled => return,
            served = next_request() => served,
        };
        let Err(e) = served else {
            service_errors.success();
            continue;
        };
        service_errors.failure(e);
        if !sleep_unless_cancelled(RETRY_BACKOFF, &mut cancelled).await {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use tokio::time::Instant;

    use crate::test_support::{Listened, Logged, cancel, cancellation};

    #[tokio::test]
    async fn a_cancel_that_already_happened_ends_the_loop_before_it_listens() {
        let listened = Listened::at_most(0);
        serve("cancelled", std::future::ready(()), || async {
            listened.note();
            Ok::<(), String>(())
        })
        .await;
        assert_eq!(listened.count(), 0);
    }

    /// On a paused clock that moves only when every task waits: the bound
    /// runs out only when the loop waits on after the cancel with nothing
    /// left to wake it.
    #[tokio::test(start_paused = true)]
    async fn a_cancel_ends_the_loop_while_it_waits_for_a_request() {
        let (sender, cancelled) = cancellation();
        let listened = Listened::at_most(1);
        let serving = serve("waiting", cancelled, || async {
            listened.note();
            std::future::pending::<Result<(), String>>().await
        });
        let cancel_once_listening = async {
            while listened.count() == 0 {
                tokio::task::yield_now().await;
            }
            cancel(&sender);
        };
        tokio::time::timeout(Duration::from_secs(1), async {
            tokio::join!(serving, cancel_once_listening)
        })
        .await
        .expect("the cancel ends the loop");
        assert_eq!(listened.count(), 1);
    }

    #[tokio::test]
    async fn requests_are_served_one_after_another_until_cancelled() {
        let (sender, cancelled) = cancellation();
        let listened = Listened::at_most(3);
        serve("served", cancelled, || async {
            if listened.note() == 3 {
                cancel(&sender);
            }
            Ok::<(), String>(())
        })
        .await;
        assert_eq!(listened.count(), 3);
    }

    /// On a paused clock that moves only when every task waits: the pause
    /// is the clock's, not the host's.
    #[tokio::test(start_paused = true)]
    async fn an_error_is_waited_out_before_the_loop_listens_again() {
        let (_logged, _guard) = Logged::capture();
        let (sender, cancelled) = cancellation();
        let listened = Listened::at_most(2);
        serve("failing", cancelled, || async {
            if listened.note() == 1 {
                return Err("the session is closed".to_owned());
            }
            cancel(&sender);
            Ok(())
        })
        .await;
        let listens = listened.at.borrow();
        assert_eq!(listens.len(), 2);
        assert!(listens[1] - listens[0] >= RETRY_BACKOFF);
    }

    #[tokio::test(start_paused = true)]
    async fn a_cancel_ends_the_loop_while_it_waits_out_an_error() {
        let (_logged, _guard) = Logged::capture();
        let (sender, cancelled) = cancellation();
        let listened = Listened::at_most(1);
        let start = Instant::now();
        serve("failing", cancelled, || async {
            listened.note();
            cancel(&sender);
            Err::<(), _>("the session is closed")
        })
        .await;
        assert_eq!(listened.count(), 1);
        assert!(Instant::now() - start < RETRY_BACKOFF);
    }

    /// Requests 1, 2, 4 and 5 fail and request 3 is served, so the errors
    /// make two runs.
    #[tokio::test(start_paused = true)]
    async fn the_first_error_of_each_run_is_logged() {
        let (logged, _guard) = Logged::capture();
        let (sender, cancelled) = cancellation();
        let listened = Listened::at_most(5);
        serve("get_limb_names", cancelled, || async {
            let listen = listened.note();
            if listen == 5 {
                cancel(&sender);
            }
            match listen {
                3 => Ok(()),
                _ => Err("the session is closed"),
            }
        })
        .await;
        assert_eq!(
            logged.lines_holding(
                "get_limb_names failing, suppressing repeats: the session is closed"
            ),
            2
        );
    }

    /// A node spawns each loop onto the runtime's worker threads, which
    /// takes a `Send` future, with a request closure that borrows what the
    /// task owns, as a generated `handle_next_request(&runner, ..)` does.
    #[tokio::test]
    async fn the_loop_runs_as_a_spawned_task() {
        let serving = tokio::spawn(async move {
            let requests = tokio::sync::Semaphore::new(0);
            serve("spawned", std::future::ready(()), || async {
                requests.acquire().await.map(drop)
            })
            .await;
        });
        serving.await.expect("the loop ends at the cancel");
    }
}
