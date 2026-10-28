//! A setpoint stream: the latest setpoint, published on a fixed interval
//! until cancelled.

use std::future::Future;
use std::time::Duration;

use tokio::time::MissedTickBehavior;

use crate::latched_log::LatchedLog;

/// Where a setpoint stream sends each message: a node wraps its pairing
/// slot's publisher in one, and a test supplies a fake.
pub trait SetpointSink {
    type Message;

    /// Sends one message, or says why it was not sent.
    fn send(&self, message: Self::Message) -> impl Future<Output = Result<(), String>> + Send;
}

/// Publishes the setpoint `next_message` builds to `sink` every `period`
/// until `cancelled` resolves. A tick whose `next_message` returns `None`
/// publishes nothing. A failed build or send warns once per failure burst,
/// naming the operation `"{label} command publish"`, and the next tick tries
/// again.
///
/// # Panics
///
/// When `period` is zero, which `tokio::time::interval` refuses.
pub async fn stream_setpoints<S: SetpointSink>(
    sink: S,
    period: Duration,
    cancelled: impl Future<Output = ()>,
    label: String,
    mut next_message: impl FnMut() -> Option<Result<S::Message, String>>,
) {
    // An interval holds the publish cadence at `period` whatever each tick's
    // work costs. A missed tick delays the schedule, so a scheduling hiccup
    // is followed by one publish and then the period again.
    let mut ticker = tokio::time::interval(period);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let publish_warn = LatchedLog::warn(format!("{label} command publish"));
    let mut cancelled = std::pin::pin!(cancelled);

    loop {
        tokio::select! {
            _ = &mut cancelled => return,
            _ = ticker.tick() => {}
        }

        let Some(built) = next_message() else {
            continue;
        };
        let result = match built {
            Ok(msg) => sink.send(msg).await,
            Err(e) => Err(e),
        };
        match result {
            Ok(()) => publish_warn.success(),
            Err(e) => publish_warn.failure(&e),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};

    use tokio::sync::oneshot;

    use super::*;

    const PERIOD: Duration = Duration::from_millis(5);
    /// Ticks to let run before judging what was published.
    const TICKS: u32 = 20;
    /// Only every third tick has something to stream, as a disengaged arm
    /// does: the rest must skip.
    const TICKS_PER_MESSAGE: usize = 3;

    /// A sink that counts what reached it, and can refuse every send.
    struct Recorder {
        sent: Arc<AtomicUsize>,
        fails: bool,
    }

    impl SetpointSink for Recorder {
        type Message = u8;

        async fn send(&self, _message: u8) -> Result<(), String> {
            self.sent.fetch_add(1, Ordering::SeqCst);
            if self.fails {
                return Err("publisher is down".to_string());
            }
            Ok(())
        }
    }

    /// The sender that cancels a stream, and the future the stream watches.
    fn cancellation() -> (oneshot::Sender<()>, impl Future<Output = ()>) {
        let (cancel, cancelled) = oneshot::channel();
        (cancel, async move {
            let _ = cancelled.await;
        })
    }

    #[tokio::test(start_paused = true)]
    async fn a_tick_with_nothing_to_stream_is_skipped_and_the_stream_lives_on() {
        let sent = Arc::new(AtomicUsize::new(0));
        let asked = Arc::new(AtomicUsize::new(0));
        let (cancel, cancelled) = cancellation();
        let stream = {
            let (sent, asked) = (sent.clone(), asked.clone());
            tokio::spawn(async move {
                stream_setpoints(
                    Recorder { sent, fails: false },
                    PERIOD,
                    cancelled,
                    "test".to_string(),
                    move || {
                        let tick = asked.fetch_add(1, Ordering::SeqCst);
                        (tick % TICKS_PER_MESSAGE == TICKS_PER_MESSAGE - 1).then_some(Ok(1u8))
                    },
                )
                .await
            })
        };

        tokio::time::sleep(PERIOD * TICKS).await;
        let asked_count = asked.load(Ordering::SeqCst);
        let sent_count = sent.load(Ordering::SeqCst);
        assert!(
            asked_count >= TICKS as usize,
            "every tick asks for a message: {asked_count}"
        );
        assert_eq!(
            sent_count,
            asked_count / TICKS_PER_MESSAGE,
            "only the ticks with a message publish, and none repeats"
        );
        assert!(
            !stream.is_finished(),
            "a skipped tick must not end the task"
        );

        cancel.send(()).expect("the stream is still running");
        tokio::time::timeout(PERIOD * TICKS, stream)
            .await
            .expect("cancellation ends the task")
            .expect("task");
    }

    #[tokio::test(start_paused = true)]
    async fn a_failing_send_keeps_the_stream_running() {
        let sent = Arc::new(AtomicUsize::new(0));
        let (cancel, cancelled) = cancellation();
        let stream = {
            let sent = sent.clone();
            tokio::spawn(async move {
                stream_setpoints(
                    Recorder { sent, fails: true },
                    PERIOD,
                    cancelled,
                    "test".to_string(),
                    || Some(Ok(1u8)),
                )
                .await
            })
        };

        tokio::time::sleep(PERIOD * TICKS).await;
        assert!(
            sent.load(Ordering::SeqCst) >= TICKS as usize,
            "a failed publish is retried on the next tick"
        );
        assert!(
            !stream.is_finished(),
            "a failing publisher must not end the task"
        );
        cancel.send(()).expect("the stream is still running");
        tokio::time::timeout(PERIOD * TICKS, stream)
            .await
            .expect("cancellation ends the task")
            .expect("task");
    }
}
