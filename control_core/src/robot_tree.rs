//! A robot's tree of frames as a node holds it: the messages of transforms taken into a
//! [`FrameTree`] by the node's followers ([`follow_moving`], [`follow_static`]), and
//! lookups in it that wait on the node's clock until the tree can answer them, as
//! tf2_ros's buffer waits in `canTransform` with a timeout.
//!
//! Each message and each lookup looks at the node's clock first, so a clock that went back
//! since the look before drops every moving stamp before the tree changes or answers, as
//! the transforms contract asks. A simulation clock counts each of its steps back at its
//! feed ([`BackSteps`]), so a step back and a step forward between two looks still drop
//! them.

use std::collections::HashMap;
use std::fmt::Display;
use std::future::Future;
use std::ops::ControlFlow;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use frame_tree::{
    Found, FrameName, FrameTree, LookupError, LookupTime, MessageError, MovingMessage, Publisher,
    Stamp, StaticMessage, TransformError, ZeroWindow,
};
use tokio::sync::watch;
use tracing::warn;

use crate::latched_log::LatchedLog;
use crate::node_clock::{BackSteps, NodeClock};
use crate::receiving;
use crate::sync::lock_ignoring_poison;

/// A robot's tree of frames, shared by the followers that fill it and the lookups that
/// read it, on the node's clock.
#[derive(Debug)]
pub struct RobotTree {
    held: Mutex<Held>,
    /// Changes at each message the tree does not refuse whole, so a lookup that waits looks
    /// again.
    taken: watch::Sender<()>,
    clock: NodeClock,
}

/// The tree and the watch of the clock's steps back, which change together.
#[derive(Debug)]
struct Held {
    tree: FrameTree,
    back_steps: BackSteps,
}

/// What the tree refused of a message: the whole message, or some of its transforms,
/// each with its reason.
#[derive(Debug, thiserror::Error)]
pub enum Refused {
    #[error(transparent)]
    Message(#[from] MessageError),
    #[error("{}", listed(.0))]
    Transforms(Vec<TransformError>),
}

impl Refused {
    /// What a take refused: nothing, the message whole, or each transform it refused.
    fn of(taken: Result<Vec<TransformError>, MessageError>) -> Result<(), Self> {
        let refused = taken?;
        if refused.is_empty() {
            return Ok(());
        }
        Err(Self::Transforms(refused))
    }
}

fn listed(refused: &[TransformError]) -> String {
    refused
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join("; ")
}

/// Resolves once a simulation `clock` reads before `start_ns`; never for the wall clock,
/// whose reading the wait looks at when its span has passed, nor once the simulation
/// clock's feed is gone.
async fn back_before(clock: &NodeClock, start_ns: u64) {
    let Some(mut ticks) = clock.ticks() else {
        return std::future::pending().await;
    };
    if ticks.wait_for(|now| *now < start_ns).await.is_err() {
        std::future::pending::<()>().await;
    }
}

impl RobotTree {
    /// An empty tree that keeps each moving transform's stamps within `window` of its
    /// child frame's newest, and reads and waits on `clock`.
    pub fn new(window: Duration, clock: NodeClock) -> Result<Self, ZeroWindow> {
        Ok(Self {
            held: Mutex::new(Held {
                tree: FrameTree::new(window)?,
                back_steps: clock.back_steps(),
            }),
            taken: watch::Sender::new(()),
            clock,
        })
    }

    /// Takes a message of `transforms` from `publisher`. A message the tree does not refuse
    /// whole wakes the lookups that wait.
    pub fn take_moving(
        &self,
        publisher: &Publisher,
        message: &MovingMessage,
    ) -> Result<(), Refused> {
        let taken = self.on_the_clock(|tree| tree.take_moving(publisher, message));
        self.wake_unless_refused_whole(&taken);
        Refused::of(taken)
    }

    /// Takes a message of `static_transforms` from `publisher`, as its whole static set.
    /// A message the tree does not refuse whole wakes the lookups that wait.
    pub fn take_static(
        &self,
        publisher: &Publisher,
        message: &StaticMessage,
    ) -> Result<(), Refused> {
        let taken = self.on_the_clock(|tree| tree.take_static(publisher, message));
        self.wake_unless_refused_whole(&taken);
        Refused::of(taken)
    }

    /// The place of `source` in `target` at `time`: at once when the tree answers it, or
    /// once a message the tree takes lets it answer, or the failure of the last lookup
    /// once the node's clock reads `timeout` past the call, or reads before the call. A
    /// timeout of zero looks once. When the wait ends, the lookup looks a last time, as
    /// `canTransform` does. A clock that went back ends the wait at any step back below
    /// the call, the tree then holding a new timeline; tf2_ros ends its wait when its clock
    /// reads more than 3 s before the call (tf2_ros/src/buffer.cpp:162). A simulation
    /// clock's step back ends the wait at its tick; the wall clock's, when the wait's
    /// span has passed.
    pub async fn lookup_within(
        &self,
        target: &FrameName,
        source: &FrameName,
        time: LookupTime,
        timeout: Duration,
    ) -> Result<Found, LookupError> {
        let mut taken = self.taken.subscribe();
        let timeout_ns = u64::try_from(timeout.as_nanos()).unwrap_or(u64::MAX);
        let start_ns = self.clock.now_ns();
        let deadline_ns = start_ns.saturating_add(timeout_ns);
        loop {
            taken.borrow_and_update();
            let failure = match self.on_the_clock(|tree| tree.lookup(target, source, time)) {
                Ok(found) => return Ok(found),
                Err(failure) => failure,
            };
            let now_ns = self.clock.now_ns();
            if now_ns >= deadline_ns || now_ns < start_ns {
                return Err(failure);
            }
            // The tree holds the sender of `taken`, so `changed` never gives an error.
            // The ends of the wait come first, and the lookup then looks a last time.
            tokio::select! {
                biased;
                () = self.clock.until(deadline_ns) => {}
                () = back_before(&self.clock, start_ns) => {}
                _ = taken.changed() => {}
            }
        }
    }

    /// Runs `act` on the tree once the tree looked at the node's clock: a clock that went
    /// back since the look before drops every moving stamp first, as tf2_ros's buffer
    /// clears on a jump back in time, with a warning once the tree is free again.
    fn on_the_clock<T>(&self, act: impl FnOnce(&mut FrameTree) -> T) -> T {
        let (went_back, acted) = {
            let mut guard = lock_ignoring_poison(&self.held);
            let held = &mut *guard;
            let went_back = held.back_steps.went_back();
            if went_back {
                held.tree.clear_moving();
            }
            (went_back, act(&mut held.tree))
        };
        if went_back {
            warn!(
                "the node's clock went back since the tree last looked at it, and reads {}: the \
                 tree drops the stamps of every moving transform and keeps the static ones",
                Stamp::from_nanos(self.clock.now_ns())
            );
        }
        acted
    }

    /// Wakes the lookups that wait, for a message the tree did not refuse whole.
    fn wake_unless_refused_whole(&self, taken: &Result<Vec<TransformError>, MessageError>) {
        if taken.is_ok() {
            self.taken.send_replace(());
        }
    }
}

/// Takes into `tree` each message of `transforms` that `next` receives with its publisher,
/// until `cancelled` resolves or the subscription closes, as [`receiving::receive`]
/// receives a subscription. A message the tree refuses, whole or in part, is logged once
/// for each run of refusals of its publisher, naming the publisher.
pub async fn follow_moving<E: Display>(
    tree: Arc<RobotTree>,
    cancelled: impl Future<Output = ()>,
    next: impl AsyncFnMut() -> Result<Option<(Publisher, MovingMessage)>, E>,
) {
    follow("transforms", tree, cancelled, next, RobotTree::take_moving).await;
}

/// Takes into `tree` each message of `static_transforms` that `next` receives with its
/// publisher, as [`follow_moving`] takes each message of `transforms`.
pub async fn follow_static<E: Display>(
    tree: Arc<RobotTree>,
    cancelled: impl Future<Output = ()>,
    next: impl AsyncFnMut() -> Result<Option<(Publisher, StaticMessage)>, E>,
) {
    follow(
        "static_transforms",
        tree,
        cancelled,
        next,
        RobotTree::take_static,
    )
    .await;
}

/// Takes into `tree` with `take` each message of the topic `topic` that `next` receives,
/// with a latch of refusals for each publisher.
async fn follow<M, E: Display>(
    topic: &'static str,
    tree: Arc<RobotTree>,
    cancelled: impl Future<Output = ()>,
    next: impl AsyncFnMut() -> Result<Option<(Publisher, M)>, E>,
    take: fn(&RobotTree, &Publisher, &M) -> Result<(), Refused>,
) {
    let taking = format!("taking {topic}");
    let mut refused: HashMap<Publisher, LatchedLog> = HashMap::new();
    receiving::receive(topic, cancelled, next, async move |(publisher, message)| {
        let taken = take(&tree, &publisher, &message);
        let latch = refused
            .entry(publisher)
            .or_insert_with_key(|publisher| LatchedLog::warn(format!("{taking} from {publisher}")));
        match taken {
            Ok(()) => latch.success(),
            Err(error) => latch.failure(error),
        }
        ControlFlow::Continue(())
    })
    .await;
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::time::{SystemTime, UNIX_EPOCH};

    use frame_tree::TransformErrorReason;

    use crate::node_clock::TickFeed;
    use crate::test_support::Logged;

    const WINDOW: Duration = Duration::from_secs(10);
    const WAIT: Duration = Duration::from_secs(5);
    const NANOS_PER_SECOND: u64 = 1_000_000_000;
    /// The time on the runtime's paused clock a test grants a task to end. The runtime moves
    /// that clock only once every task waits, so it runs out at once for a task that waits
    /// on, and never for one that ends.
    const TO_END: Duration = Duration::from_secs(1);

    fn backbone() -> Publisher {
        Publisher {
            core_node: "core".into(),
            instance_id: "backbone".into(),
        }
    }

    fn initializer() -> Publisher {
        Publisher {
            core_node: "core".into(),
            instance_id: "init_inst".into(),
        }
    }

    fn frame(name: &str) -> FrameName {
        FrameName::parse(name).expect("a frame name")
    }

    fn nanos(seconds: u64) -> u64 {
        seconds * NANOS_PER_SECOND
    }

    fn stamp(seconds: u64) -> Stamp {
        Stamp::from_nanos(nanos(seconds))
    }

    fn time(seconds: u64) -> SystemTime {
        UNIX_EPOCH + Duration::from_secs(seconds)
    }

    /// An empty tree on a simulation clock that reads `seconds`, and the feed of the
    /// clock's ticks.
    fn tree_at(seconds: u64) -> (Arc<RobotTree>, TickFeed) {
        let (clock, ticks) = NodeClock::simulation();
        ticks.tick(nanos(seconds));
        let tree = RobotTree::new(WINDOW, clock).expect("a window above 0");
        (Arc::new(tree), ticks)
    }

    /// The arm link `x` m along the base link's x, measured at `seconds`.
    fn arm_at(seconds: u64, x: f64) -> MovingMessage {
        MovingMessage {
            timestamp: time(seconds),
            child_frames: vec!["arm".into()],
            parent_frames: vec!["base_link".into()],
            positions: vec![x, 0.0, 0.0],
            orientations: vec![0.0, 0.0, 0.0, 1.0],
            start_timestamp: time(1),
        }
    }

    /// The camera fixed 1 m above the arm link.
    fn camera_on_the_arm() -> StaticMessage {
        StaticMessage {
            child_frames: vec!["camera".into()],
            parent_frames: vec!["arm".into()],
            positions: vec![0.0, 0.0, 1.0],
            orientations: vec![0.0, 0.0, 0.0, 1.0],
        }
    }

    async fn camera_at(
        tree: &RobotTree,
        time: LookupTime,
        timeout: Duration,
    ) -> Result<Found, LookupError> {
        tree.lookup_within(&frame("base_link"), &frame("camera"), time, timeout)
            .await
    }

    /// A lookup of the camera at `seconds` within `WAIT`, on a task of its own.
    fn waiting_for_the_camera_at(
        tree: &Arc<RobotTree>,
        seconds: u64,
    ) -> tokio::task::JoinHandle<Result<Found, LookupError>> {
        let tree = Arc::clone(tree);
        tokio::spawn(async move { camera_at(&tree, LookupTime::At(stamp(seconds)), WAIT).await })
    }

    /// Whether `waiting` still waits once every other task waits.
    async fn still_waits<T>(waiting: &mut tokio::task::JoinHandle<T>) -> bool {
        tokio::time::timeout(TO_END, waiting).await.is_err()
    }

    /// What `waiting` ends with, which it must end with before every other task waits.
    async fn ended<T>(waiting: tokio::task::JoinHandle<T>) -> T {
        tokio::time::timeout(TO_END, waiting)
            .await
            .expect("the lookup ends")
            .expect("the lookup's task ends without a panic")
    }

    fn position(found: Result<Found, LookupError>) -> [f64; 3] {
        found.expect("a lookup that succeeds").placement.position()
    }

    fn is_extrapolation_of_the_arm(failure: &Result<Found, LookupError>) -> bool {
        matches!(
            failure,
            Err(LookupError::Extrapolation { transform, .. }) if transform.child == frame("arm")
        )
    }

    #[tokio::test(start_paused = true)]
    async fn a_lookup_waits_until_a_message_lets_the_tree_answer_it() {
        let (tree, _ticks) = tree_at(21);
        tree.take_static(&initializer(), &camera_on_the_arm())
            .expect("taken");
        tree.take_moving(&backbone(), &arm_at(20, 1.0))
            .expect("taken");
        assert!(matches!(
            camera_at(&tree, LookupTime::At(stamp(21)), Duration::ZERO).await,
            Err(LookupError::Extrapolation { .. })
        ));
        let mut waiting = waiting_for_the_camera_at(&tree, 21);
        assert!(
            still_waits(&mut waiting).await,
            "the lookup waits for the stamp of 21 s"
        );
        tree.take_moving(&backbone(), &arm_at(21, 2.0))
            .expect("taken");
        let found = ended(waiting).await.expect("a lookup that succeeds");
        assert_eq!(found.placement.position(), [2.0, 0.0, 1.0]);
        assert_eq!(found.time, Some(stamp(21)));
        assert_eq!(
            position(camera_at(&tree, LookupTime::At(stamp(20)), Duration::ZERO).await),
            [1.0, 0.0, 1.0]
        );
    }

    #[tokio::test(start_paused = true)]
    async fn a_lookup_at_the_latest_time_answers_the_newest_stamp_of_the_chain() {
        let (tree, _ticks) = tree_at(21);
        tree.take_static(&initializer(), &camera_on_the_arm())
            .expect("taken");
        tree.take_moving(&backbone(), &arm_at(20, 1.0))
            .expect("taken");
        tree.take_moving(&backbone(), &arm_at(21, 2.0))
            .expect("taken");
        let found = camera_at(&tree, LookupTime::Latest, Duration::ZERO)
            .await
            .expect("a lookup at the latest time");
        assert_eq!(found.placement.position(), [2.0, 0.0, 1.0]);
        assert_eq!(found.time, Some(stamp(21)));
        let fixed = tree
            .lookup_within(
                &frame("arm"),
                &frame("camera"),
                LookupTime::Latest,
                Duration::ZERO,
            )
            .await
            .expect("a static chain");
        assert_eq!(fixed.time, None, "a static chain holds at every time");
    }

    #[tokio::test(start_paused = true)]
    async fn a_lookup_the_tree_cannot_answer_fails_once_the_node_clock_passes_its_wait() {
        let (tree, ticks) = tree_at(21);
        tree.take_static(&initializer(), &camera_on_the_arm())
            .expect("taken");
        tree.take_moving(&backbone(), &arm_at(20, 1.0))
            .expect("taken");
        // The lookup starts its wait at 21 s on the node's clock, which then moves on to
        // 1 ns before the wait runs out, then to its end.
        let mut waiting = waiting_for_the_camera_at(&tree, 30);
        assert!(still_waits(&mut waiting).await);
        ticks.tick(nanos(21) + nanos(5) - 1);
        assert!(
            still_waits(&mut waiting).await,
            "the wait runs 5 s of the node's clock"
        );
        ticks.tick(nanos(21) + nanos(5));
        let failure = ended(waiting).await;
        assert!(is_extrapolation_of_the_arm(&failure), "{failure:?}");
    }

    #[tokio::test(start_paused = true)]
    async fn a_waiting_lookup_ends_with_its_failure_when_the_clock_goes_back_below_its_call() {
        let (_logged, _guard) = Logged::capture();
        let (tree, ticks) = tree_at(1_000);
        tree.take_static(&initializer(), &camera_on_the_arm())
            .expect("taken");
        tree.take_moving(&backbone(), &arm_at(999, 1.0))
            .expect("taken");
        let mut waiting = waiting_for_the_camera_at(&tree, 1_001);
        // A step back that stays at or after the call keeps the wait; one below it ends
        // the wait far before the clock reads the deadline again.
        assert!(still_waits(&mut waiting).await);
        ticks.tick(nanos(1_002));
        ticks.tick(nanos(1_000));
        assert!(
            still_waits(&mut waiting).await,
            "the clock reads the call's start, not before it"
        );
        ticks.tick(nanos(1_000) - 1);
        let failure = ended(waiting).await;
        assert!(is_extrapolation_of_the_arm(&failure), "{failure:?}");
    }

    #[tokio::test(start_paused = true)]
    async fn a_lookup_whose_wait_runs_out_looks_a_last_time() {
        let (tree, ticks) = tree_at(21);
        tree.take_static(&initializer(), &camera_on_the_arm())
            .expect("taken");
        let mut waiting = waiting_for_the_camera_at(&tree, 20);
        assert!(still_waits(&mut waiting).await);
        // The message and the end of the wait come together: the lookup that looks at the
        // end of its wait finds the message.
        ticks.tick(nanos(26));
        tree.take_moving(&backbone(), &arm_at(20, 1.0))
            .expect("taken");
        assert_eq!(position(ended(waiting).await), [1.0, 0.0, 1.0]);
    }

    #[tokio::test(start_paused = true)]
    async fn the_moving_stamps_are_dropped_when_the_clock_goes_back_and_the_static_ones_kept() {
        let (_logged, _guard) = Logged::capture();
        let (tree, ticks) = tree_at(100);
        tree.take_static(&initializer(), &camera_on_the_arm())
            .expect("taken");
        tree.take_moving(&backbone(), &arm_at(20, 1.0))
            .expect("taken");
        ticks.tick(nanos(101));
        assert_eq!(
            position(camera_at(&tree, LookupTime::At(stamp(20)), Duration::ZERO).await),
            [1.0, 0.0, 1.0]
        );
        // A lookup looks at the clock that went back before it looks in the tree.
        ticks.tick(nanos(50));
        assert!(matches!(
            camera_at(&tree, LookupTime::At(stamp(20)), Duration::ZERO).await,
            Err(LookupError::Extrapolation { .. })
        ));
        let fixed = tree
            .lookup_within(
                &frame("arm"),
                &frame("camera"),
                LookupTime::At(stamp(20)),
                Duration::ZERO,
            )
            .await;
        assert_eq!(position(fixed), [0.0, 0.0, 1.0]);
    }

    #[tokio::test(start_paused = true)]
    async fn a_step_back_and_forward_between_two_looks_drops_the_moving_stamps() {
        let (logged, _guard) = Logged::capture();
        let (tree, ticks) = tree_at(100);
        tree.take_moving(&backbone(), &arm_at(20, 1.0))
            .expect("taken");
        // Back to 10 s and on to 200 s, with no message and no lookup between.
        ticks.tick(nanos(10));
        ticks.tick(nanos(200));
        assert!(matches!(
            tree.lookup_within(
                &frame("base_link"),
                &frame("arm"),
                LookupTime::At(stamp(20)),
                Duration::ZERO,
            )
            .await,
            Err(LookupError::Extrapolation { .. })
        ));
        assert_eq!(
            logged.lines_holding(
                "the node's clock went back since the tree last looked at it, and reads \
                 200.000000000 s"
            ),
            1
        );
    }

    #[tokio::test(start_paused = true)]
    async fn a_message_looks_at_the_clock_that_went_back_before_the_tree_takes_it() {
        let (_logged, _guard) = Logged::capture();
        let (tree, ticks) = tree_at(100);
        tree.take_moving(&backbone(), &arm_at(20, 1.0))
            .expect("taken");
        ticks.tick(nanos(50));
        tree.take_moving(&backbone(), &arm_at(40, 3.0))
            .expect("taken");
        let arm = |seconds| {
            let tree = Arc::clone(&tree);
            async move {
                tree.lookup_within(
                    &frame("base_link"),
                    &frame("arm"),
                    LookupTime::At(stamp(seconds)),
                    Duration::ZERO,
                )
                .await
            }
        };
        assert!(
            arm(20).await.is_err(),
            "the stamp of 20 s went with the clock"
        );
        assert_eq!(position(arm(40).await), [3.0, 0.0, 0.0]);
    }

    #[tokio::test(start_paused = true)]
    async fn two_publishers_of_one_transform_break_the_tree_there() {
        let (tree, _ticks) = tree_at(21);
        tree.take_moving(&backbone(), &arm_at(20, 1.0))
            .expect("taken");
        tree.take_moving(&initializer(), &arm_at(20, 1.0))
            .expect("taken");
        let failure = tree
            .lookup_within(
                &frame("base_link"),
                &frame("arm"),
                LookupTime::At(stamp(20)),
                Duration::ZERO,
            )
            .await
            .expect_err("the arm link has two publishers");
        assert!(
            failure.to_string().contains("instance backbone of core")
                && failure.to_string().contains("instance init_inst of core"),
            "{failure}"
        );
    }

    #[test]
    fn a_refused_transform_and_a_message_refused_whole_are_named() {
        let (tree, _ticks) = tree_at(21);
        let mut looped = arm_at(20, 1.0);
        looped.parent_frames = vec!["arm".into()];
        assert!(matches!(
            tree.take_moving(&backbone(), &looped),
            Err(Refused::Transforms(refused))
                if refused.len() == 1 && refused[0].reason == TransformErrorReason::SelfTransform
        ));
        tree.take_moving(&backbone(), &arm_at(20, 1.0))
            .expect("taken");
        let old = tree
            .take_moving(&backbone(), &arm_at(5, 1.0))
            .expect_err("a stamp older than the window");
        assert!(old.to_string().contains("\"arm\""), "{old}");
        let mut ragged = camera_on_the_arm();
        ragged.positions.pop();
        assert!(matches!(
            tree.take_static(&initializer(), &ragged),
            Err(Refused::Message(MessageError::Shape { .. }))
        ));
    }

    #[tokio::test(start_paused = true)]
    async fn a_message_refused_whole_wakes_no_lookup() {
        let (tree, _ticks) = tree_at(21);
        tree.take_static(&initializer(), &camera_on_the_arm())
            .expect("taken");
        let mut waiting = waiting_for_the_camera_at(&tree, 21);
        assert!(still_waits(&mut waiting).await);
        let woken = tree.taken.subscribe();
        let mut ragged = arm_at(21, 1.0);
        ragged.positions.pop();
        assert!(tree.take_moving(&backbone(), &ragged).is_err());
        assert!(!woken.has_changed().expect("the tree holds the sender"));
        assert!(still_waits(&mut waiting).await);
        tree.take_moving(&backbone(), &arm_at(21, 1.0))
            .expect("taken");
        assert!(woken.has_changed().expect("the tree holds the sender"));
        assert_eq!(position(ended(waiting).await), [1.0, 0.0, 1.0]);
    }

    #[tokio::test(start_paused = true)]
    async fn the_followers_take_each_message_and_log_a_run_of_refusals_once() {
        let (logged, _guard) = Logged::capture();
        let (tree, _ticks) = tree_at(21);
        let mut ragged = arm_at(20, 1.0);
        ragged.positions.pop();
        let mut received = vec![
            Ok(Some((backbone(), ragged.clone()))),
            Ok(Some((backbone(), ragged))),
            Err("a message that does not decode"),
            Ok(Some((backbone(), arm_at(20, 1.0)))),
            Ok(None),
            Ok(Some((backbone(), arm_at(21, 9.0)))),
        ]
        .into_iter();
        follow_moving(Arc::clone(&tree), std::future::pending(), async || {
            received.next().expect("a receive past the end")
        })
        .await;
        assert_eq!(
            logged.lines_holding(
                "taking transforms from instance backbone of core failing, suppressing repeats"
            ),
            1
        );
        assert_eq!(logged.lines_holding("transforms receive failing"), 1);
        let arm = tree
            .lookup_within(
                &frame("base_link"),
                &frame("arm"),
                LookupTime::Latest,
                Duration::ZERO,
            )
            .await
            .expect("the message of 20 s, and none after the subscription closed");
        assert_eq!(arm.time, Some(stamp(20)));

        let mut fixed = vec![
            Ok::<_, &str>(Some((initializer(), camera_on_the_arm()))),
            Ok(None),
        ]
        .into_iter();
        follow_static(Arc::clone(&tree), std::future::pending(), async || {
            fixed.next().expect("a receive past the end")
        })
        .await;
        assert_eq!(
            position(camera_at(&tree, LookupTime::At(stamp(20)), Duration::ZERO).await),
            [1.0, 0.0, 1.0]
        );
    }

    #[tokio::test(start_paused = true)]
    async fn each_publishers_refusals_are_logged_once_a_run_whatever_the_others_send() {
        let (logged, _guard) = Logged::capture();
        let (tree, _ticks) = tree_at(21);
        let mut ragged = arm_at(20, 1.0);
        ragged.positions.pop();
        let camera = |seconds, x| MovingMessage {
            child_frames: vec!["camera".into()],
            ..arm_at(seconds, x)
        };
        let mut received = (0..3)
            .flat_map(|second| {
                [
                    Ok::<_, &str>(Some((initializer(), ragged.clone()))),
                    Ok(Some((backbone(), camera(10 + second, 1.0)))),
                ]
            })
            .chain([Ok(None)]);
        follow_moving(Arc::clone(&tree), std::future::pending(), async || {
            received.next().expect("a receive past the end")
        })
        .await;
        assert_eq!(
            logged.lines_holding(
                "taking transforms from instance init_inst of core failing, suppressing repeats"
            ),
            1,
            "the backbone's messages leave the initializer's run of refusals"
        );
        assert_eq!(
            logged.lines_holding("taking transforms from instance backbone of core"),
            0
        );
    }

    #[test]
    fn a_followers_future_is_send_so_a_node_spawns_it() {
        fn spawnable(_: impl Future<Output = ()> + Send) {}
        let (tree, _ticks) = tree_at(21);
        spawnable(follow_moving(
            Arc::clone(&tree),
            std::future::pending(),
            async || Ok::<_, String>(None),
        ));
        spawnable(follow_static(tree, std::future::pending(), async || {
            Ok::<_, String>(None)
        }));
    }

    #[test]
    fn a_window_of_zero_is_refused() {
        let clock = NodeClock::simulation().0;
        assert!(RobotTree::new(Duration::ZERO, clock).is_err());
    }
}
