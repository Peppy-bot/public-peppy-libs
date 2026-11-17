//! The lookups of tf2's buffer: `FrameTree` takes each transform lookup scenario of the ROS
//! reference harness step by step, as tf2's `BufferCore` took it, and the evidence is that
//! it takes the same transforms, refuses the same lookups with the same kind of error, and
//! answers every other lookup with tf2's time and a transform within `POSITION_TOLERANCE`
//! of tf2's position and within `ORIENTATION_TOLERANCE` of its orientation.
//!
//! A static step sends the transform with every static transform sent before it, as
//! tf2_ros's static broadcaster sends its whole set, which `take_static` takes. The
//! scenarios send each child frame's moving stamps once, since a stamp sent again replaces
//! the held one here and tf2 keeps both.
//!
//! Parsing normalizes each quaternion it takes, and tf2 keeps a quaternion within its
//! tolerance as given (buffer_core.cpp:210-266). The scenarios send quaternions of unit norm
//! to their rounding, so the normalization moves each by its rounding alone, and both refuse
//! the one quaternion off unit that they send.

use std::time::{Duration, SystemTime};

use frame_tree::{
    BrokenTree, FrameName, FrameTree, HeldStamps, LookupError, LookupTime, MovingMessage,
    Publisher, Stamp, StaticMessage,
};
use ros_reference::transform_lookup::{Case, Cases, Found, SentTransform, Tf2Error};

/// The largest distance allowed between our position and tf2's, in meters. The two compute
/// the same operations in the same order, so the difference measured is at the bits of the
/// libm sine and arccosine each links.
const POSITION_TOLERANCE: f64 = 1e-9;

/// The largest difference allowed between our orientation and tf2's, in radians, measured as
/// twice the norm of the difference of the two quaternions, which bounds the angle between
/// them to first order. It is at the bits of libm for the reason `POSITION_TOLERANCE` is.
const ORIENTATION_TOLERANCE: f64 = 1e-9;

/// The start of every moving transform: tf2 has no start, and no transform of the scenarios
/// starts over.
const START: SystemTime = SystemTime::UNIX_EPOCH;

/// The one publisher of the scenarios.
fn publisher() -> Publisher {
    Publisher {
        core_node: "ros_reference".into(),
        instance_id: "transform_lookup".into(),
    }
}

#[test]
fn every_lookup_agrees_with_tf2() {
    let scenarios = Cases::load_all();
    assert!(!scenarios.is_empty(), "the harness holds a scenario");
    let mut worst = (0.0_f64, 0.0_f64);
    let mut lookups = 0;
    for scenario in &scenarios {
        let mut tree = FrameTree::new(Duration::from_nanos(scenario.cache_window_ns))
            .unwrap_or_else(|e| panic!("{}: {e}", scenario.name));
        let mut static_set: Vec<SentTransform> = Vec::new();
        for (index, case) in scenario.cases.iter().enumerate() {
            let at = format!("{} step {index}", scenario.name);
            match case {
                Case::Moving {
                    stamp,
                    transform,
                    taken,
                } => {
                    let message = MovingMessage {
                        timestamp: SystemTime::UNIX_EPOCH + Duration::from_nanos(*stamp),
                        child_frames: vec![transform.child.clone()],
                        parent_frames: vec![transform.parent.clone()],
                        positions: transform.position.to_vec(),
                        orientations: transform.orientation.to_vec(),
                        start_timestamp: START,
                    };
                    let refused = tree
                        .take_moving(&publisher(), &message)
                        .unwrap_or_else(|e| panic!("{at}: {e}"));
                    assert_eq!(refused.is_empty(), *taken, "{at}: {transform:?} taken");
                }
                Case::Static { transform, taken } => {
                    match static_set
                        .iter_mut()
                        .find(|held| held.child == transform.child)
                    {
                        Some(held) => *held = transform.clone(),
                        None => static_set.push(transform.clone()),
                    }
                    let message = StaticMessage {
                        child_frames: static_set.iter().map(|sent| sent.child.clone()).collect(),
                        parent_frames: static_set.iter().map(|sent| sent.parent.clone()).collect(),
                        positions: static_set.iter().flat_map(|sent| sent.position).collect(),
                        orientations: static_set
                            .iter()
                            .flat_map(|sent| sent.orientation)
                            .collect(),
                    };
                    let refused = tree
                        .take_static(&publisher(), &message)
                        .unwrap_or_else(|e| panic!("{at}: {e}"));
                    let took = !refused
                        .iter()
                        .any(|refused| refused.child == transform.child);
                    assert_eq!(took, *taken, "{at}: {transform:?} taken");
                }
                Case::Lookup {
                    target,
                    source,
                    time,
                    answer,
                } => {
                    lookups += 1;
                    let ours = lookup(&tree, target, source, *time);
                    match (ours, answer) {
                        (Ok(ours), Ok(tf2)) => {
                            assert_eq!(
                                ours.time, tf2.time,
                                "{at}: the time of {source} in {target}"
                            );
                            let (position, orientation) = differences(&ours, tf2);
                            assert!(
                                position <= POSITION_TOLERANCE
                                    && orientation <= ORIENTATION_TOLERANCE,
                                "{at}: {source} in {target} is {ours:?}, tf2 gives {tf2:?}"
                            );
                            worst = (worst.0.max(position), worst.1.max(orientation));
                        }
                        (Err(ours), Err(tf2)) => {
                            assert_eq!(ours, *tf2, "{at}: the error of {source} in {target}");
                        }
                        (ours, tf2) => {
                            panic!("{at}: {source} in {target} is {ours:?}, tf2 gives {tf2:?}")
                        }
                    }
                }
            }
        }
    }
    println!(
        "{lookups} lookups; the largest difference from tf2: {:e} m, {:e} rad",
        worst.0, worst.1
    );
}

/// Our answer to a lookup, as the reference writes tf2's: an empty frame name refused as tf2
/// refuses it.
fn lookup(
    tree: &FrameTree,
    target: &str,
    source: &str,
    time: Option<u64>,
) -> Result<Found, Tf2Error> {
    let (Ok(target), Ok(source)) = (FrameName::parse(target), FrameName::parse(source)) else {
        return Err(Tf2Error::InvalidArgument);
    };
    let time = time.map_or(LookupTime::Latest, |nanos| {
        LookupTime::At(Stamp::from_nanos(nanos))
    });
    match tree.lookup(&target, &source, time) {
        Ok(found) => Ok(Found {
            position: found.placement.position(),
            orientation: found.placement.orientation(),
            time: found.time.map(Stamp::as_nanos),
        }),
        Err(error) => Err(tf2_error(error)),
    }
}

/// The exception tf2 throws for each error of ours.
fn tf2_error(error: LookupError) -> Tf2Error {
    match error {
        LookupError::UnknownFrame { .. } => Tf2Error::UnknownFrame,
        LookupError::NoChain { .. } => Tf2Error::Connectivity,
        LookupError::Extrapolation {
            requested, held, ..
        } => match (requested, held) {
            (_, HeldStamps::One(_)) => Tf2Error::NoDataForExtrapolation,
            (LookupTime::At(time), HeldStamps::Range { newest, .. }) if time > newest => {
                Tf2Error::ForwardExtrapolation
            }
            (LookupTime::At(_), HeldStamps::Range { .. }) => Tf2Error::BackwardExtrapolation,
            (requested, held) => {
                panic!(
                    "a scenario clears no transform and has no latest time without a value: {requested}, {held:?}"
                )
            }
        },
        LookupError::BrokenTree(BrokenTree::Cycle { .. }) => Tf2Error::Loop,
        LookupError::BrokenTree(broken @ BrokenTree::TwoPublishers { .. }) => {
            panic!("a scenario has one publisher: {broken}")
        }
    }
}

/// The distance of our position from tf2's, in meters, and the difference of our
/// orientation from tf2's, as twice the norm of the difference of the quaternions, in
/// radians.
fn differences(ours: &Found, tf2: &Found) -> (f64, f64) {
    let norm = |values: &mut dyn Iterator<Item = f64>| values.map(|d| d * d).sum::<f64>().sqrt();
    let position = norm(&mut ours.position.iter().zip(tf2.position).map(|(a, b)| a - b));
    let orientation = 2.0
        * norm(
            &mut ours
                .orientation
                .iter()
                .zip(tf2.orientation)
                .map(|(a, b)| a - b),
        );
    (position, orientation)
}
