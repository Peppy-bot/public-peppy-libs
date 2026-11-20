//! A robot's tree of frames as a node holds it: the messages of transforms taken into a
//! [`FrameTree`] by the node's followers ([`follow_moving`], [`follow_static`]), and
//! lookups in it that wait on the node's clock until the tree can answer them, as
//! tf2_ros's buffer waits in `canTransform` with a timeout: the place of a frame in another
//! at a time ([`RobotTree::lookup_within`]) or at one time in another at another through the
//! root of its chain ([`RobotTree::lookup_within_through`]), and a value of the geometry types
//! placed in a frame ([`RobotTree::place`]) or in the root of its frame's chain
//! ([`RobotTree::place_in_root`]), refused with a [`PlaceError`].
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

use crate::geometry::{GeometryError, Placed, Stamped};
use crate::latched_log::LatchedLog;
use crate::node_clock::{BackSteps, NodeClock};
use crate::receiving;
use crate::sync::lock_ignoring_poison;

/// A value placed in a target frame, and the time of that frame's place it is in: none for a
/// value that holds at every time, as a lookup through static transforms alone places it.
/// [`report_timestamp`] gives the time as the timestamp of a report that leaves it out for such a
/// value.
///
/// [`report_timestamp`]: crate::geometry::report_timestamp
#[derive(Debug, Clone, PartialEq)]
pub struct PlacedValue<T> {
    pub value: T,
    pub time: Option<Stamp>,
}

/// Why a value was not placed: the lookup that places it failed, or a coordinate the placement
/// reaches is not finite, which the value's parse refuses.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PlaceError {
    #[error(transparent)]
    Lookup(#[from] LookupError),
    #[error(transparent)]
    Value(#[from] GeometryError),
}

impl PlaceError {
    /// The failure kind the contracts give this error: the lookup's, or `rejected` for a value
    /// no contract takes.
    pub fn failure_kind(&self) -> &'static str {
        match self {
            PlaceError::Lookup(error) => error.failure_kind(),
            PlaceError::Value(_) => "rejected",
        }
    }
}

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
        self.answered_within(timeout, |tree| tree.lookup(target, source, time))
            .await
    }

    /// The place of `source` at `source_time` in `target` at `target_time`, either time the
    /// latest time, waiting as [`RobotTree::lookup_within`] waits. Equal times give the lookup
    /// at that time. Different times go through the root of `source`'s chain at `source_time`,
    /// the frame that holds still across a move of the robot (odom or map on a mobile base, the
    /// robot frame on a fixed one), as tf2's advanced `lookupTransform` goes through a fixed
    /// frame; the answer holds at the target's time. A source the tree does not know, looked up
    /// in itself, is the identity at the target's time, as tf2's lookup answers a frame in itself
    /// before it checks its frames (buffer_core.cpp:830-846): no transform moves it between the
    /// two times.
    pub async fn lookup_within_through(
        &self,
        target: &FrameName,
        target_time: LookupTime,
        source: &FrameName,
        source_time: LookupTime,
        timeout: Duration,
    ) -> Result<Found, LookupError> {
        if target_time == source_time {
            return self
                .lookup_within(target, source, source_time, timeout)
                .await;
        }
        self.answered_within(timeout, |tree| {
            if source == target && !tree.knows(source) {
                return tree.lookup(target, source, target_time);
            }
            let fixed = tree.root_at(source, source_time)?;
            tree.lookup_through(target, target_time, source, source_time, &fixed)
        })
        .await
    }

    /// The answer of `look` on the tree: at once when it succeeds, or once a message the tree
    /// takes lets it succeed, or its last failure once the node's clock reads `timeout` past
    /// the call, or reads before the call, as [`RobotTree::lookup_within`] waits.
    async fn answered_within<A>(
        &self,
        timeout: Duration,
        look: impl Fn(&FrameTree) -> Result<A, LookupError>,
    ) -> Result<A, LookupError> {
        let mut taken = self.taken.subscribe();
        let timeout_ns = u64::try_from(timeout.as_nanos()).unwrap_or(u64::MAX);
        let start_ns = self.clock.now_ns();
        let deadline_ns = start_ns.saturating_add(timeout_ns);
        loop {
            taken.borrow_and_update();
            let failure = match self.on_the_clock(|tree| look(tree)) {
                Ok(answer) => return Ok(answer),
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

    /// The value of `stamped` in `target` at the latest time, with the time of `target`'s place it
    /// is in (none for a value that holds at every time), or the failure of the lookup that places
    /// it or the refusal of the placed value ([`PlaceError`]), waiting within `timeout` as
    /// [`RobotTree::lookup_within`] waits. A value at the latest time is placed by the place of its
    /// frame in `target` then. A value at a time is placed where its frame stood at that time:
    /// through the root of its frame's chain at that time, the frame that holds still across a move
    /// of the robot (odom or map on a mobile base, the robot frame on a fixed one), as tf2's
    /// advanced `lookupTransform` places a value through a fixed frame. So a robot-frame pose from
    /// before a base move names the same place on the floor after it, as Nav2 places a goal at its
    /// stamp.
    pub async fn place<T: Placed>(
        &self,
        stamped: &Stamped<T>,
        target: &FrameName,
        timeout: Duration,
    ) -> Result<PlacedValue<T>, PlaceError> {
        let found = self
            .lookup_within_through(
                target,
                LookupTime::Latest,
                stamped.frame(),
                stamped.time(),
                timeout,
            )
            .await?;
        Ok(PlacedValue {
            value: stamped.value().placed_by(&found.placement)?,
            time: found.time,
        })
    }

    /// `stamped` placed in the root of its frame's chain at its time, stamped in that root at the
    /// latest time, or the failure of the lookup that places it or the refusal of the placed value
    /// ([`PlaceError`]), waiting within `timeout` as [`RobotTree::lookup_within`] waits. The root
    /// holds still across a move of the robot (odom or map on a mobile base, the robot frame on a
    /// fixed one), so the value names one place at every later time. A frame with no parent is its
    /// own root, and its value comes back as it was.
    pub async fn place_in_root<T: Placed>(
        &self,
        stamped: &Stamped<T>,
        timeout: Duration,
    ) -> Result<Stamped<T>, PlaceError> {
        let (root, found) = self
            .answered_within(timeout, |tree| {
                let root = tree.root_at(stamped.frame(), stamped.time())?;
                let found = tree.lookup(&root, stamped.frame(), stamped.time())?;
                Ok((root, found))
            })
            .await?;
        Ok(Stamped::at_latest(
            root,
            stamped.value().placed_by(&found.placement)?,
        ))
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

    use frame_tree::{TransformErrorReason, TransformRecord};

    use crate::geometry::{Point, Pose};

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

    const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

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
            transforms: vec![TransformRecord::new(
                "base_link",
                "arm",
                [x, 0.0, 0.0],
                IDENTITY,
            )],
            start_timestamp: time(1),
        }
    }

    /// The camera fixed 1 m above the arm link.
    fn camera_on_the_arm() -> StaticMessage {
        StaticMessage {
            transforms: vec![TransformRecord::new(
                "arm",
                "camera",
                [0.0, 0.0, 1.0],
                IDENTITY,
            )],
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

    /// The point at the camera's origin, in the camera frame at `timestamp`, or at the latest
    /// time for none.
    fn camera_origin(timestamp: Option<SystemTime>) -> Stamped<Point> {
        Stamped::parse(
            "camera",
            timestamp,
            Point::parse([0.0; 3]).expect("a point"),
        )
        .expect("a stamped point")
    }

    // A point is placed where its frame stood at its time, and at the newest stamp of the chain
    // for no timestamp, as Nav2 places a goal at its stamp; the Unix epoch is a time like any
    // other, which the tree does not hold.
    #[tokio::test(start_paused = true)]
    async fn a_point_is_placed_where_its_frame_stood_at_its_time() {
        let (tree, _ticks) = tree_at(21);
        tree.take_static(&initializer(), &camera_on_the_arm())
            .expect("taken");
        tree.take_moving(&backbone(), &arm_at(18, 0.5))
            .expect("taken");
        tree.take_moving(&backbone(), &arm_at(20, 0.9))
            .expect("taken");
        let base = frame("base_link");
        let then = tree
            .place(&camera_origin(Some(time(18))), &base, Duration::ZERO)
            .await;
        assert_eq!(then.expect("placed").value.position(), [0.5, 0.0, 1.0]);
        let latest = tree
            .place(&camera_origin(None), &base, Duration::ZERO)
            .await;
        assert_eq!(latest.expect("placed").value.position(), [0.9, 0.0, 1.0]);
        let unknown = Stamped::parse(
            "gripper",
            Some(time(18)),
            Point::parse([0.0; 3]).expect("a point"),
        )
        .expect("a stamped point");
        assert!(matches!(
            tree.place(&unknown, &base, Duration::ZERO).await,
            Err(PlaceError::Lookup(LookupError::UnknownFrame { .. }))
        ));
        for before in [time(5), UNIX_EPOCH] {
            assert!(matches!(
                tree.place(&camera_origin(Some(before)), &base, Duration::ZERO)
                    .await,
                Err(PlaceError::Lookup(LookupError::Extrapolation { .. }))
            ));
        }
    }

    // A robot-frame point stamped before the base moves names the same place on the floor after
    // it: it is placed through the root of its frame's chain, odom; one at the latest time stands
    // where the robot frame stands then.
    #[tokio::test(start_paused = true)]
    async fn a_robot_frame_point_from_before_a_base_move_keeps_its_place_on_the_floor() {
        let (tree, _ticks) = tree_at(21);
        let base_at = |seconds, x| MovingMessage {
            transforms: vec![TransformRecord::new(
                "odom",
                "base_footprint",
                [x, 0.0, 0.0],
                IDENTITY,
            )],
            ..arm_at(seconds, x)
        };
        tree.take_moving(&backbone(), &base_at(18, 0.0))
            .expect("taken");
        tree.take_moving(&backbone(), &base_at(20, 1.0))
            .expect("taken");
        let base = frame("base_footprint");
        let ahead = |timestamp| {
            Stamped::parse(
                "base_footprint",
                timestamp,
                Point::parse([1.0, 0.0, 0.0]).unwrap(),
            )
            .unwrap()
        };
        let before = tree
            .place(&ahead(Some(time(18))), &base, Duration::ZERO)
            .await;
        assert_eq!(
            before.as_ref().expect("placed").value.position(),
            [0.0, 0.0, 0.0]
        );
        assert_eq!(before.map(|placed| placed.time), Ok(Some(stamp(20))));
        let latest = tree.place(&ahead(None), &base, Duration::ZERO).await;
        assert_eq!(latest.expect("placed").value.position(), [1.0, 0.0, 0.0]);
        assert!(matches!(
            tree.place(&ahead(Some(time(5))), &base, Duration::ZERO)
                .await,
            Err(PlaceError::Lookup(LookupError::Extrapolation { .. }))
        ));
    }

    // A value is placed in the root of its frame's chain where its frame stood at its time, and
    // comes back stamped in that root at the latest time; a frame with no parent is its own
    // root, and a frame the tree does not know is refused.
    #[tokio::test(start_paused = true)]
    async fn a_value_is_placed_in_the_root_of_its_frames_chain_at_its_time() {
        let (tree, _ticks) = tree_at(21);
        let base_at = |seconds, x| MovingMessage {
            transforms: vec![TransformRecord::new(
                "odom",
                "base_footprint",
                [x, 0.0, 0.0],
                IDENTITY,
            )],
            ..arm_at(seconds, x)
        };
        tree.take_moving(&backbone(), &base_at(18, 0.5))
            .expect("taken");
        tree.take_moving(&backbone(), &base_at(20, 1.0))
            .expect("taken");
        let ahead = |frame: &str| {
            Stamped::parse(
                frame,
                Some(time(18)),
                Point::parse([1.0, 0.0, 0.0]).unwrap(),
            )
            .unwrap()
        };
        let in_odom = tree
            .place_in_root(&ahead("base_footprint"), Duration::ZERO)
            .await
            .expect("placed");
        assert_eq!(in_odom.frame(), &frame("odom"));
        assert_eq!(in_odom.time(), LookupTime::Latest);
        assert_eq!(in_odom.value().position(), [1.5, 0.0, 0.0]);
        let in_itself = tree
            .place_in_root(&ahead("odom"), Duration::ZERO)
            .await
            .expect("placed");
        assert_eq!(in_itself.frame(), &frame("odom"));
        assert_eq!(in_itself.value().position(), [1.0, 0.0, 0.0]);
        assert!(matches!(
            tree.place_in_root(&ahead("gripper"), Duration::ZERO).await,
            Err(PlaceError::Lookup(LookupError::UnknownFrame { .. }))
        ));
    }

    // A value whose placement reaches a coordinate that is not finite is refused as its parse
    // refuses one, with `rejected`; a failed lookup keeps its own kind.
    #[tokio::test(start_paused = true)]
    async fn a_value_placed_past_the_finite_numbers_is_refused_as_its_parse_refuses_it() {
        let (tree, _ticks) = tree_at(21);
        tree.take_static(
            &initializer(),
            &StaticMessage {
                transforms: vec![TransformRecord::new(
                    "base_link",
                    "far",
                    [f64::MAX, 0.0, 0.0],
                    IDENTITY,
                )],
            },
        )
        .expect("taken");
        let beyond =
            Stamped::parse("far", None, Point::parse([f64::MAX, 0.0, 0.0]).unwrap()).unwrap();
        let refused = tree
            .place(&beyond, &frame("base_link"), Duration::ZERO)
            .await
            .expect_err("refused");
        assert_eq!(
            refused.to_string(),
            "a position must be finite: got x inf, y 0, z 0; give a finite number of meters on \
             each axis"
        );
        assert_eq!(refused.failure_kind(), "rejected");
        let in_root = tree.place_in_root(&beyond, Duration::ZERO).await;
        assert_eq!(
            in_root.map_err(|error| error.failure_kind()),
            Err("rejected")
        );
        let unknown = tree
            .place(&beyond, &frame("gripper"), Duration::ZERO)
            .await
            .expect_err("refused");
        assert_eq!(unknown.failure_kind(), "unknown_frame");
    }

    // A lookup across two times goes through the root of the source's chain at the source's
    // time, either time the latest; equal times give the lookup at that time.
    #[tokio::test(start_paused = true)]
    async fn a_lookup_across_two_times_goes_through_the_root_and_equal_times_do_not() {
        let (tree, _ticks) = tree_at(21);
        let base_at = |seconds, x| MovingMessage {
            transforms: vec![TransformRecord::new(
                "odom",
                "base_footprint",
                [x, 0.0, 0.0],
                IDENTITY,
            )],
            ..arm_at(seconds, x)
        };
        tree.take_moving(&backbone(), &base_at(18, 0.0))
            .expect("taken");
        tree.take_moving(&backbone(), &base_at(20, 1.0))
            .expect("taken");
        let base = frame("base_footprint");
        let at = |seconds| LookupTime::At(stamp(seconds));
        let position =
            |found: Result<Found, LookupError>| found.expect("found").placement.position();
        let then_in_now = tree
            .lookup_within_through(&base, LookupTime::Latest, &base, at(18), Duration::ZERO)
            .await;
        assert_eq!(position(then_in_now), [-1.0, 0.0, 0.0]);
        let now_in_then = tree
            .lookup_within_through(&base, at(18), &base, LookupTime::Latest, Duration::ZERO)
            .await;
        assert_eq!(position(now_in_then), [1.0, 0.0, 0.0]);
        let equal = tree
            .lookup_within_through(&base, at(18), &base, at(18), Duration::ZERO)
            .await;
        assert_eq!(position(equal), [0.0, 0.0, 0.0]);
    }

    // A frame the tree does not know, looked up in itself across two times, is the identity at
    // the target's time, so a value in it at a time is placed as it was; in another frame it is
    // refused.
    #[tokio::test(start_paused = true)]
    async fn an_unknown_frame_in_itself_across_two_times_is_the_identity() {
        let (tree, _ticks) = tree_at(21);
        let base = frame("base_footprint");
        let at = |seconds| LookupTime::At(stamp(seconds));
        let now = tree
            .lookup_within_through(&base, LookupTime::Latest, &base, at(18), Duration::ZERO)
            .await
            .expect("the identity");
        assert_eq!(now.placement, frame_tree::Placement::IDENTITY);
        assert_eq!(now.time, None);
        let then = tree
            .lookup_within_through(&base, at(20), &base, at(18), Duration::ZERO)
            .await
            .expect("the identity");
        assert_eq!(then.time, Some(stamp(20)));
        let ahead = Stamped::parse(
            "base_footprint",
            Some(time(18)),
            Point::parse([1.0, 0.0, 0.0]).unwrap(),
        )
        .unwrap();
        let placed = tree.place(&ahead, &base, Duration::ZERO).await;
        assert_eq!(placed.expect("placed").value.position(), [1.0, 0.0, 0.0]);
        assert!(matches!(
            tree.place(&ahead, &frame("odom"), Duration::ZERO).await,
            Err(PlaceError::Lookup(LookupError::UnknownFrame { .. }))
        ));
    }

    // Values of one frame at one time are placed by one answer of the tree, each where its frame
    // stood then, and refused whole where one leaves the finite numbers.
    #[tokio::test(start_paused = true)]
    async fn values_of_one_frame_and_one_time_are_placed_together() {
        let (tree, _ticks) = tree_at(21);
        tree.take_static(
            &initializer(),
            &StaticMessage {
                transforms: vec![
                    TransformRecord::new("arm", "camera", [0.0, 0.0, 1.0], IDENTITY),
                    TransformRecord::new("base_link", "far", [f64::MAX, 0.0, 0.0], IDENTITY),
                ],
            },
        )
        .expect("taken");
        tree.take_moving(&backbone(), &arm_at(18, 0.5))
            .expect("taken");
        tree.take_moving(&backbone(), &arm_at(20, 0.9))
            .expect("taken");
        let points = |frame: &str, timestamp, positions: &[[f64; 3]]| {
            let points: Vec<Point> = positions
                .iter()
                .map(|p| Point::parse(*p).unwrap())
                .collect();
            Stamped::parse(frame, timestamp, points).unwrap()
        };
        let base = frame("base_link");
        let placed = tree
            .place(
                &points("camera", Some(time(18)), &[[0.0; 3], [1.0, 0.0, 0.0]]),
                &base,
                Duration::ZERO,
            )
            .await
            .expect("placed");
        let positions: Vec<[f64; 3]> = placed.value.iter().map(|p| p.position()).collect();
        assert_eq!(positions, [[0.5, 0.0, 1.0], [1.5, 0.0, 1.0]]);
        let one_past = points("far", None, &[[0.0; 3], [f64::MAX, 0.0, 0.0]]);
        let refused = tree.place(&one_past, &base, Duration::ZERO).await;
        assert_eq!(
            refused.map_err(|error| error.failure_kind()),
            Err("rejected")
        );
    }

    // A pose's orientation turns by the place of its frame.
    #[tokio::test(start_paused = true)]
    async fn a_pose_turns_with_its_frame() {
        let (tree, _ticks) = tree_at(21);
        let half = std::f64::consts::FRAC_PI_4;
        let quarter_turn = [0.0, 0.0, half.sin(), half.cos()];
        tree.take_static(
            &initializer(),
            &StaticMessage {
                transforms: vec![TransformRecord::new(
                    "base_link",
                    "turned",
                    [1.0, 0.0, 0.0],
                    quarter_turn,
                )],
            },
        )
        .expect("taken");
        let base = frame("base_link");
        let pose = Stamped::parse(
            "turned",
            None,
            Pose::parse([1.0, 0.0, 0.0], quarter_turn).unwrap(),
        )
        .unwrap();
        let placed = tree
            .place(&pose, &base, Duration::ZERO)
            .await
            .expect("placed")
            .value;
        let [x, y, z] = placed.position().position();
        assert!(
            (x - 1.0).abs() < 1e-12 && (y - 1.0).abs() < 1e-12 && z == 0.0,
            "{x} {y} {z}"
        );
        let [qx, qy, qz, qw] = placed.orientation().quaternion();
        assert!(
            qx.abs() < 1e-12 && qy.abs() < 1e-12 && (qz - 1.0).abs() < 1e-12 && qw.abs() < 1e-12
        );
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
        looped.transforms[0].frame = "arm".into();
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
        let mut twice = camera_on_the_arm();
        twice.transforms.push(twice.transforms[0].clone());
        assert!(matches!(
            tree.take_static(&initializer(), &twice),
            Err(Refused::Message(MessageError::RepeatedChild { .. }))
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
        let mut twice = arm_at(21, 1.0);
        twice.transforms.push(twice.transforms[0].clone());
        assert!(tree.take_moving(&backbone(), &twice).is_err());
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
        let mut twice = arm_at(20, 1.0);
        twice.transforms.push(twice.transforms[0].clone());
        let mut received = vec![
            Ok(Some((backbone(), twice.clone()))),
            Ok(Some((backbone(), twice))),
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
        // A message stamped before the epoch, which the tree refuses whole.
        let untimely = MovingMessage {
            timestamp: UNIX_EPOCH - Duration::from_nanos(1),
            ..arm_at(20, 1.0)
        };
        let camera = |seconds, x| MovingMessage {
            transforms: vec![TransformRecord::new(
                "base_link",
                "camera",
                [x, 0.0, 0.0],
                IDENTITY,
            )],
            ..arm_at(seconds, x)
        };
        let mut received = (0..3)
            .flat_map(|second| {
                [
                    Ok::<_, &str>(Some((initializer(), untimely.clone()))),
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
