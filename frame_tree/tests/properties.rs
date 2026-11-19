//! Properties of lookups on drawn trees: a lookup and its reverse undo each other, the
//! lookups along a chain compose, a static chain holds at every time, a moving transform
//! holds the blend of its two samples between their stamps and no value outside them, and a
//! lookup never blends the samples of a transform across a start.

use std::time::{Duration, SystemTime};

use frame_tree::{
    FrameName, FrameTree, LookupError, LookupTime, MovingMessage, Placement, Publisher, Stamp,
    StaticMessage, TransformRecord,
};
use hegel::TestCase;
use hegel::generators as gs;

/// The difference allowed between two points that two routes through a tree place: the
/// rounding of a few rotations of points within meters of the origin.
const TOLERANCE: f64 = 1e-9;

/// The one publisher of every drawn tree.
fn publisher() -> Publisher {
    Publisher {
        core_node: "properties".into(),
        instance_id: "drawn".into(),
    }
}

fn name(index: usize) -> String {
    format!("frame_{index}")
}

fn frame(index: usize) -> FrameName {
    FrameName::parse(&name(index)).unwrap()
}

/// A unit quaternion [x, y, z, w]: a drawn axis turned by a drawn angle.
fn rotation(tc: &TestCase) -> [f64; 4] {
    let axis: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0)));
    let norm = axis.iter().map(|c| c * c).sum::<f64>().sqrt();
    tc.assume(norm > 1e-3);
    let angle: f64 = tc.draw(
        gs::floats()
            .min_value(-std::f64::consts::PI)
            .max_value(std::f64::consts::PI),
    );
    let (sin, cos) = (angle / 2.0).sin_cos();
    [
        axis[0] / norm * sin,
        axis[1] / norm * sin,
        axis[2] / norm * sin,
        cos,
    ]
}

fn point(tc: &TestCase) -> [f64; 3] {
    tc.draw(gs::arrays(gs::floats().min_value(-2.0).max_value(2.0)))
}

/// The position and orientation of a transform, as a message sends them.
#[derive(Clone, Copy)]
struct Pose {
    position: [f64; 3],
    orientation: [f64; 4],
}

fn pose(tc: &TestCase) -> Pose {
    Pose {
        position: point(tc),
        orientation: rotation(tc),
    }
}

/// A point of the placed frame in the frame it is placed in: R(q) p + t.
fn apply(placement: &Placement, p: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = placement.orientation();
    let [px, py, pz] = p;
    // q p q*, expanded for a unit quaternion.
    let (tx, ty, tz) = (
        2.0 * (y * pz - z * py),
        2.0 * (z * px - x * pz),
        2.0 * (x * py - y * px),
    );
    let rotated = [
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    ];
    let position = placement.position();
    std::array::from_fn(|i| rotated[i] + position[i])
}

fn assert_close(a: [f64; 3], b: [f64; 3], what: &str) {
    assert!(
        a.iter().zip(b).all(|(a, b)| (a - b).abs() <= TOLERANCE),
        "{what}: {a:?} is not {b:?}"
    );
}

/// A tree of 2 to 8 frames, each frame after the first placed static in a frame before it.
fn static_tree(tc: &TestCase) -> (FrameTree, usize) {
    let frames: usize = tc.draw(gs::integers::<usize>().min_value(2).max_value(8));
    let transforms = (1..frames)
        .map(|child| {
            let parent = tc.draw(gs::integers::<usize>().min_value(0).max_value(child - 1));
            let Pose {
                position,
                orientation,
            } = pose(tc);
            TransformRecord::new(&name(parent), &name(child), position, orientation)
        })
        .collect();
    let message = StaticMessage { transforms };
    let mut tree = FrameTree::new(Duration::from_secs(1)).unwrap();
    let refused = tree.take_static(&publisher(), &message).unwrap();
    assert!(refused.is_empty(), "{refused:?}");
    (tree, frames)
}

fn draw_frame(tc: &TestCase, frames: usize) -> FrameName {
    frame(tc.draw(gs::integers::<usize>().min_value(0).max_value(frames - 1)))
}

fn placement_of(
    tree: &FrameTree,
    target: &FrameName,
    source: &FrameName,
    time: LookupTime,
) -> Placement {
    tree.lookup(target, source, time)
        .unwrap_or_else(|e| panic!("{source} in {target}: {e}"))
        .placement
}

#[hegel::test]
fn a_lookup_and_its_reverse_undo_each_other(tc: TestCase) {
    let (tree, frames) = static_tree(&tc);
    let (a, b) = (draw_frame(&tc, frames), draw_frame(&tc, frames));
    let p = point(&tc);
    let b_in_a = placement_of(&tree, &a, &b, LookupTime::Latest);
    let a_in_b = placement_of(&tree, &b, &a, LookupTime::Latest);
    assert_close(apply(&b_in_a, apply(&a_in_b, p)), p, "there and back");
}

#[hegel::test]
fn the_lookups_along_a_chain_compose(tc: TestCase) {
    let (tree, frames) = static_tree(&tc);
    let (a, b, c) = (
        draw_frame(&tc, frames),
        draw_frame(&tc, frames),
        draw_frame(&tc, frames),
    );
    let stamp = Stamp::from_nanos(tc.draw(gs::integers::<u64>()));
    let time = LookupTime::At(stamp);
    let p = point(&tc);
    let found = tree.lookup(&a, &c, time).unwrap();
    assert_eq!(
        found.time,
        Some(stamp),
        "a static chain holds at the lookup's time"
    );
    let through_b = apply(
        &placement_of(&tree, &a, &b, time),
        apply(&placement_of(&tree, &b, &c, time), p),
    );
    assert_close(apply(&found.placement, p), through_b, "through b");
}

/// Sends the transform frame_0 -> frame_1 at `stamp`, last started over `start` ns after the
/// Unix epoch.
fn send(tree: &mut FrameTree, stamp: u64, pose: Pose, start: u64) {
    let message = MovingMessage {
        timestamp: SystemTime::UNIX_EPOCH + Duration::from_nanos(stamp),
        transforms: vec![TransformRecord::new(
            &name(0),
            &name(1),
            pose.position,
            pose.orientation,
        )],
        start_timestamp: SystemTime::UNIX_EPOCH + Duration::from_nanos(start),
    };
    let refused = tree.take_moving(&publisher(), &message).unwrap();
    assert!(refused.is_empty(), "{refused:?}");
}

/// The blend of two samples' positions at `ratio`, as tf2 interpolates a position.
fn blend(first: &Pose, second: &Pose, ratio: f64) -> [f64; 3] {
    std::array::from_fn(|i| (1.0 - ratio) * first.position[i] + ratio * second.position[i])
}

#[hegel::test]
fn a_moving_transform_blends_its_samples_between_their_stamps_alone(tc: TestCase) {
    const SPAN: u64 = 1_000_000;
    let start: u64 = tc.draw(gs::integers::<u64>().min_value(1).max_value(u64::MAX / 2));
    let span: u64 = tc.draw(gs::integers::<u64>().min_value(1).max_value(SPAN));
    let (first, second) = (pose(&tc), pose(&tc));
    let mut tree = FrameTree::new(Duration::from_nanos(SPAN)).unwrap();
    send(&mut tree, start, first, 0);
    send(&mut tree, start + span, second, 0);
    let offset: u64 = tc.draw(gs::integers::<u64>().min_value(0).max_value(span));
    let at = LookupTime::At(Stamp::from_nanos(start + offset));
    let blended = placement_of(&tree, &frame(0), &frame(1), at);
    let along = blend(&first, &second, offset as f64 / span as f64);
    assert_close(blended.position(), along, "the position");
    for outside in [start - 1, start + span + 1] {
        let error = tree
            .lookup(
                &frame(0),
                &frame(1),
                LookupTime::At(Stamp::from_nanos(outside)),
            )
            .unwrap_err();
        assert!(
            matches!(error, LookupError::Extrapolation { .. }),
            "{error}"
        );
    }
}

#[hegel::test]
fn a_lookup_never_blends_across_a_start(tc: TestCase) {
    const SPAN: u64 = 1_000_000;
    let gap = || gs::integers::<u64>().min_value(1).max_value(SPAN);
    let before: u64 = tc.draw(gs::integers::<u64>().min_value(1).max_value(u64::MAX / 4));
    let restart = before + tc.draw(gap());
    let after = restart + tc.draw(gap());
    let (old, first, second) = (pose(&tc), pose(&tc), pose(&tc));
    let (old_start, new_start): (u64, u64) = (tc.draw(gs::integers()), tc.draw(gs::integers()));
    tc.assume(old_start != new_start);
    let mut tree = FrameTree::new(Duration::from_nanos(4 * SPAN)).unwrap();
    send(&mut tree, before, old, old_start);
    send(&mut tree, restart, first, new_start);
    send(&mut tree, after, second, new_start);
    let time: u64 = tc.draw(gs::integers::<u64>().min_value(before).max_value(after));
    let found = tree.lookup(
        &frame(0),
        &frame(1),
        LookupTime::At(Stamp::from_nanos(time)),
    );
    if time < restart {
        assert!(
            matches!(found, Err(LookupError::Extrapolation { .. })),
            "before the new start: {found:?}"
        );
        return;
    }
    let ratio = (time - restart) as f64 / (after - restart) as f64;
    let placement = found.unwrap_or_else(|e| panic!("{e}")).placement;
    assert_close(
        placement.position(),
        blend(&first, &second, ratio),
        "after it",
    );
}
