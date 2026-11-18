//! The rules of the tree, each on a small tree built for it: the lookups tf2 gives, and
//! where the tree departs from tf2 (two publishers, a cycle, a static set replaced, a clear
//! that keeps the static transforms, a transform that starts over, a stamp sent again).

use std::time::{Duration, SystemTime};

use frame_tree::{
    BrokenTree, FrameName, FrameTransform, FrameTree, HeldStamps, LookupError, LookupTime,
    MovingMessage, Placement, Publisher, Stamp, StaticMessage, TransformError,
    TransformErrorReason, TransformName, ZeroWindow,
};

const WINDOW: Duration = Duration::from_nanos(100);

/// The difference allowed where the arithmetic is exact but for its rounding.
const TOLERANCE: f64 = 1e-12;

fn frame(name: &str) -> FrameName {
    FrameName::parse(name).unwrap()
}

fn at(nanos: u64) -> LookupTime {
    LookupTime::At(Stamp::from_nanos(nanos))
}

/// The frame an unknown-frame failure names.
fn unknown_frame(found: Result<frame_tree::Found, LookupError>) -> FrameName {
    match found {
        Err(LookupError::UnknownFrame { frame, .. }) => frame,
        other => panic!("an unknown frame, not {other:?}"),
    }
}

/// A transform as a message sends it.
struct Sent {
    parent: &'static str,
    child: &'static str,
    position: [f64; 3],
    orientation: [f64; 4],
}

/// The transform `parent` -> `child` at `position`, turned by `yaw` about z.
fn transform(parent: &'static str, child: &'static str, position: [f64; 3], yaw: f64) -> Sent {
    Sent {
        parent,
        child,
        position,
        orientation: [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()],
    }
}

/// The arrays of a message: its child frames, its parent frames, its positions and its
/// orientations.
fn arrays(sent: &[Sent]) -> (Vec<String>, Vec<String>, Vec<f64>, Vec<f64>) {
    (
        sent.iter().map(|sent| sent.child.to_owned()).collect(),
        sent.iter().map(|sent| sent.parent.to_owned()).collect(),
        sent.iter().flat_map(|sent| sent.position).collect(),
        sent.iter().flat_map(|sent| sent.orientation).collect(),
    )
}

/// The start of every transform of a test but one that starts over.
const START: u64 = 1;

fn start(seconds: u64) -> SystemTime {
    SystemTime::UNIX_EPOCH + Duration::from_secs(seconds)
}

/// The time `nanos` after the Unix epoch, as a message carries it.
fn time(nanos: u64) -> SystemTime {
    SystemTime::UNIX_EPOCH + Duration::from_nanos(nanos)
}

/// The publisher named `instance_id`, an instance of one core node.
fn publisher(instance_id: &str) -> Publisher {
    Publisher {
        core_node: "core".into(),
        instance_id: instance_id.into(),
    }
}

fn moving(stamp: u64, transforms: Vec<Sent>) -> MovingMessage {
    started(stamp, transforms, START)
}

/// A message whose transforms last started over at `from` s.
fn started(stamp: u64, transforms: Vec<Sent>, from: u64) -> MovingMessage {
    let (child_frames, parent_frames, positions, orientations) = arrays(&transforms);
    MovingMessage {
        timestamp: time(stamp),
        child_frames,
        parent_frames,
        positions,
        orientations,
        start_timestamp: start(from),
    }
}

fn transform_name(parent: &str, child: &str) -> TransformName {
    TransformName {
        parent: frame(parent),
        child: frame(child),
    }
}

fn fixed(transforms: Vec<Sent>) -> StaticMessage {
    let (child_frames, parent_frames, positions, orientations) = arrays(&transforms);
    StaticMessage {
        child_frames,
        parent_frames,
        positions,
        orientations,
    }
}

fn tree() -> FrameTree {
    FrameTree::new(WINDOW).unwrap()
}

fn position_of(tree: &FrameTree, target: &str, source: &str, time: LookupTime) -> [f64; 3] {
    tree.lookup(&frame(target), &frame(source), time)
        .unwrap_or_else(|e| panic!("{source} in {target}: {e}"))
        .placement
        .position()
}

fn assert_close(got: [f64; 3], want: [f64; 3]) {
    assert!(
        got.iter().zip(want).all(|(g, w)| (g - w).abs() < TOLERANCE),
        "{got:?} is not {want:?}"
    );
}

/// odom -> base, moving: x = stamp / 10 at stamps 10 and 20; base -> lidar, static, 1 m
/// ahead and turned a quarter turn; base -> arm, moving: y = 1 at stamps 12 and 18.
fn robot() -> FrameTree {
    let mut tree = tree();
    tree.take_static(
        &publisher("model"),
        &fixed(vec![transform(
            "base",
            "lidar",
            [1.0, 0.0, 0.0],
            std::f64::consts::FRAC_PI_2,
        )]),
    )
    .unwrap();
    for stamp in [10, 20] {
        let x = stamp as f64 / 10.0;
        assert!(
            tree.take_moving(
                &publisher("base"),
                &moving(stamp, vec![transform("odom", "base", [x, 0.0, 0.0], 0.0)])
            )
            .unwrap()
            .is_empty()
        );
    }
    for stamp in [12, 18] {
        assert!(
            tree.take_moving(
                &publisher("arm"),
                &moving(stamp, vec![transform("base", "arm", [0.0, 1.0, 0.0], 0.0)])
            )
            .unwrap()
            .is_empty()
        );
    }
    tree
}

#[test]
fn a_lookup_composes_the_chain_and_interpolates_each_moving_transform() {
    let tree = robot();
    assert_close(position_of(&tree, "odom", "lidar", at(15)), [2.5, 0.0, 0.0]);
    // The lidar is turned a quarter turn, so odom's x is the lidar's -y.
    assert_close(position_of(&tree, "lidar", "odom", at(15)), [0.0, 2.5, 0.0]);
    assert_close(position_of(&tree, "lidar", "arm", at(15)), [1.0, 1.0, 0.0]);
    // The static transform holds at every time; base's transform only within its samples.
    assert_close(
        position_of(&tree, "base", "lidar", at(1_000)),
        [1.0, 0.0, 0.0],
    );
}

#[test]
fn the_latest_time_is_the_earliest_newest_stamp_on_the_chain() {
    let tree = robot();
    let found = tree
        .lookup(&frame("odom"), &frame("arm"), LookupTime::Latest)
        .unwrap();
    assert_eq!(found.time, Some(Stamp::from_nanos(18)));
    assert_close(found.placement.position(), [1.8, 1.0, 0.0]);
    // A chain of static transforms holds at every time.
    let found = tree
        .lookup(&frame("lidar"), &frame("base"), LookupTime::Latest)
        .unwrap();
    assert_eq!(found.time, None);
    // The transforms above the frames' common ancestor do not count.
    let found = tree
        .lookup(&frame("lidar"), &frame("arm"), LookupTime::Latest)
        .unwrap();
    assert_eq!(found.time, Some(Stamp::from_nanos(18)));
}

#[test]
fn a_frame_in_itself_is_the_identity_known_or_not() {
    let tree = robot();
    for name in ["base", "lidar", "nowhere"] {
        let found = tree.lookup(&frame(name), &frame(name), at(3)).unwrap();
        assert_eq!(found.placement, Placement::IDENTITY);
        assert_eq!(found.time, Some(Stamp::from_nanos(3)));
    }
    let latest = |name| {
        tree.lookup(&frame(name), &frame(name), LookupTime::Latest)
            .unwrap()
            .time
    };
    assert_eq!(latest("base"), Some(Stamp::from_nanos(20)));
    assert_eq!(latest("lidar"), None);
    assert_eq!(latest("odom"), None);
}

#[test]
fn a_moving_transform_has_no_value_outside_its_samples() {
    let tree = robot();
    // The source's chain fails at its first transform without a value; the target's at its own.
    for (target, source, time, child) in [
        ("odom", "base", 25, "base"),
        ("odom", "base", 5, "base"),
        ("odom", "arm", 11, "arm"),
        ("arm", "odom", 25, "arm"),
    ] {
        let error = tree
            .lookup(&frame(target), &frame(source), at(time))
            .unwrap_err();
        let LookupError::Extrapolation {
            transform,
            requested,
            ..
        } = &error
        else {
            panic!("at {time}: {error}");
        };
        assert_eq!(transform.child.as_str(), child, "at {time}: {error}");
        assert_eq!(*requested, at(time));
    }
    let error = tree
        .lookup(&frame("odom"), &frame("base"), at(25))
        .unwrap_err();
    assert_eq!(
        error,
        LookupError::Extrapolation {
            transform: transform_name("odom", "base"),
            requested: at(25),
            held: HeldStamps::Range {
                oldest: Stamp::from_nanos(10),
                newest: Stamp::from_nanos(20)
            },
        }
    );
    assert_eq!(
        error.to_string(),
        "the transform odom -> base has no value at 0.000000025 s: it holds samples from \
         0.000000010 s to 0.000000020 s; look up a time it holds, or the latest time"
    );
}

#[test]
fn both_chains_failing_report_the_transform_with_the_newer_samples() {
    let mut tree = tree();
    tree.take_moving(
        &publisher("a"),
        &moving(10, vec![transform("root", "a", [0.0; 3], 0.0)]),
    )
    .unwrap();
    tree.take_moving(
        &publisher("a"),
        &moving(20, vec![transform("root", "a", [0.0; 3], 0.0)]),
    )
    .unwrap();
    tree.take_moving(
        &publisher("b"),
        &moving(30, vec![transform("root", "b", [0.0; 3], 0.0)]),
    )
    .unwrap();
    tree.take_moving(
        &publisher("b"),
        &moving(40, vec![transform("root", "b", [0.0; 3], 0.0)]),
    )
    .unwrap();
    let child_of = |error: LookupError| match error {
        LookupError::Extrapolation { transform, .. } => transform.child,
        error => panic!("{error}"),
    };
    // At 50 neither holds a value; b holds the newer samples.
    assert_eq!(
        child_of(tree.lookup(&frame("b"), &frame("a"), at(50)).unwrap_err()),
        frame("b")
    );
    assert_eq!(
        child_of(tree.lookup(&frame("a"), &frame("b"), at(50)).unwrap_err()),
        frame("b")
    );
}

#[test]
fn an_unknown_frame_and_two_trees_are_refused_by_name() {
    let mut tree = robot();
    tree.take_static(
        &publisher("other"),
        &fixed(vec![transform("table", "cup", [0.0; 3], 0.0)]),
    )
    .unwrap();
    for (target, source) in [("nowhere", "cup"), ("cup", "nowhere")] {
        assert_eq!(
            unknown_frame(tree.lookup(&frame(target), &frame(source), at(15))),
            frame("nowhere")
        );
    }
    let no_chain = Err(LookupError::NoChain {
        target_frame: frame("cup"),
        source_frame: frame("lidar"),
    });
    assert_eq!(
        tree.lookup(&frame("cup"), &frame("lidar"), at(15)),
        no_chain
    );
    assert_eq!(
        tree.lookup(&frame("cup"), &frame("lidar"), LookupTime::Latest),
        no_chain
    );
}

#[test]
fn two_publishers_of_a_transform_break_the_lookups_that_need_it() {
    let mut tree = robot();
    tree.take_moving(
        &publisher("impostor"),
        &moving(15, vec![transform("odom", "base", [9.0, 0.0, 0.0], 0.0)]),
    )
    .unwrap();
    let broken = LookupError::BrokenTree(BrokenTree::TwoPublishers {
        child: frame("base"),
        first: publisher("base"),
        second: publisher("impostor"),
    });
    for time in [at(15), LookupTime::Latest] {
        for (target, source) in [("odom", "lidar"), ("lidar", "odom")] {
            assert_eq!(
                tree.lookup(&frame(target), &frame(source), time),
                Err(broken.clone())
            );
        }
        // The frames below base meet below the broken transform.
        assert_close(position_of(&tree, "lidar", "arm", time), [1.0, 1.0, 0.0]);
    }
    assert_eq!(
        broken.to_string(),
        "two publishers, instance base of core and instance impostor of core, publish the \
         transform of base: each transform of the tree has one publisher, so stop the one that \
         does not own it"
    );
}

#[test]
fn two_publishers_hold_a_transform_until_the_window_of_its_newest_stamp_passes_one() {
    let mut tree = robot();
    let send = |tree: &mut FrameTree, publisher: &Publisher, stamp| {
        tree.take_moving(
            publisher,
            &moving(stamp, vec![transform("odom", "base", [1.0, 0.0, 0.0], 0.0)]),
        )
        .unwrap()
    };
    send(&mut tree, &publisher("impostor"), 15);
    // The impostor's stamp 15 lies within the window of 100 of base's newest, 30.
    send(&mut tree, &publisher("base"), 30);
    let error = tree
        .lookup(&frame("odom"), &frame("lidar"), at(30))
        .unwrap_err();
    assert!(
        matches!(
            error,
            LookupError::BrokenTree(BrokenTree::TwoPublishers { .. })
        ),
        "{error}"
    );
    // At 120 it lies past it, and the impostor's transform goes.
    send(&mut tree, &publisher("base"), 120);
    assert_close(
        position_of(&tree, "odom", "lidar", at(120)),
        [2.0, 0.0, 0.0],
    );
    // Between base's samples at 20 and 30, x blends from 2.0 to 1.0.
    assert_close(position_of(&tree, "odom", "lidar", at(25)), [2.5, 0.0, 0.0]);
}

#[test]
fn a_clear_drops_every_moving_sample_and_keeps_the_frames_and_static_ones() {
    let mut tree = robot();
    tree.clear_moving();
    let error = tree
        .lookup(&frame("odom"), &frame("lidar"), at(15))
        .unwrap_err();
    assert_eq!(
        error,
        LookupError::Extrapolation {
            transform: transform_name("odom", "base"),
            requested: at(15),
            held: HeldStamps::Empty,
        }
    );
    assert_eq!(
        error.to_string(),
        "the transform odom -> base has no value at 0.000000015 s: it holds no sample since \
         the clock went back; look up a time it holds, or the latest time"
    );
    // At the latest time a transform with no stamp gives its child no parent.
    assert_eq!(
        tree.lookup(&frame("odom"), &frame("lidar"), LookupTime::Latest),
        Err(LookupError::NoChain {
            target_frame: frame("odom"),
            source_frame: frame("lidar"),
        })
    );
    assert_close(position_of(&tree, "base", "lidar", at(15)), [1.0, 0.0, 0.0]);
    let latest = tree
        .lookup(&frame("base"), &frame("base"), LookupTime::Latest)
        .unwrap();
    assert_eq!(latest.time, None);
    // The next sample holds again, at its stamp alone.
    tree.take_moving(
        &publisher("base"),
        &moving(5, vec![transform("odom", "base", [0.5, 0.0, 0.0], 0.0)]),
    )
    .unwrap();
    assert_close(position_of(&tree, "odom", "lidar", at(5)), [1.5, 0.0, 0.0]);
    let error = tree
        .lookup(&frame("odom"), &frame("lidar"), at(10))
        .unwrap_err();
    assert!(
        matches!(
            error,
            LookupError::Extrapolation {
                held: HeldStamps::One(_),
                ..
            }
        ),
        "{error}"
    );
}

#[test]
fn a_new_start_from_a_publisher_drops_its_stamps_of_the_child_frame() {
    let mut tree = robot();
    // The backbone's odometry starts over at 30: its transform of base starts over.
    let sent = vec![transform("odom", "base", [0.0; 3], 0.0)];
    assert!(
        tree.take_moving(&publisher("base"), &started(30, sent, 2))
            .unwrap()
            .is_empty()
    );
    assert_eq!(
        tree.lookup(&frame("odom"), &frame("base"), at(20)),
        Err(LookupError::Extrapolation {
            transform: transform_name("odom", "base"),
            requested: at(20),
            held: HeldStamps::One(Stamp::from_nanos(30)),
        })
    );
    // The samples of the new start blend.
    let sent = vec![transform("odom", "base", [4.0, 0.0, 0.0], 0.0)];
    assert!(
        tree.take_moving(&publisher("base"), &started(40, sent, 2))
            .unwrap()
            .is_empty()
    );
    assert_close(position_of(&tree, "odom", "base", at(35)), [2.0, 0.0, 0.0]);
    // The arm's transform, of another child frame, keeps its samples.
    assert_close(position_of(&tree, "base", "arm", at(15)), [0.0, 1.0, 0.0]);
}

#[test]
fn starts_are_compared_for_equality_and_per_publisher() {
    let mut tree = robot();
    // An earlier start is a new start all the same.
    let sent = vec![transform("odom", "base", [5.0, 0.0, 0.0], 0.0)];
    assert!(
        tree.take_moving(&publisher("base"), &started(50, sent, 0))
            .unwrap()
            .is_empty()
    );
    assert!(tree.lookup(&frame("odom"), &frame("base"), at(20)).is_err());
    // A second publisher's first message has no start to compare, and its start leaves
    // the first publisher's samples.
    let sent = vec![transform("odom", "base", [5.0, 0.0, 0.0], 0.0)];
    tree.take_moving(&publisher("impostor"), &started(51, sent, 7))
        .unwrap();
    let error = tree
        .lookup(&frame("odom"), &frame("base"), at(50))
        .unwrap_err();
    assert!(
        matches!(
            error,
            LookupError::BrokenTree(BrokenTree::TwoPublishers { .. })
        ),
        "{error}"
    );
}

#[test]
fn a_stamp_sent_again_replaces_the_held_one() {
    let mut tree = robot();
    let sent = vec![transform("odom", "base", [9.0, 0.0, 0.0], 0.0)];
    assert!(
        tree.take_moving(&publisher("base"), &moving(20, sent))
            .unwrap()
            .is_empty()
    );
    assert_close(position_of(&tree, "odom", "base", at(20)), [9.0, 0.0, 0.0]);
    assert_close(position_of(&tree, "odom", "base", at(15)), [5.0, 0.0, 0.0]);
}

#[test]
fn a_transform_that_closes_a_cycle_breaks_the_lookups_that_need_it() {
    let mut tree = tree();
    tree.take_static(
        &publisher("model"),
        &fixed(vec![
            transform("b", "a", [1.0, 0.0, 0.0], 0.0),
            transform("c", "b", [1.0, 0.0, 0.0], 0.0),
            transform("a", "c", [1.0, 0.0, 0.0], 0.0),
            transform("a", "leaf", [0.0, 1.0, 0.0], 0.0),
            transform("a", "twig", [0.0, 2.0, 0.0], 0.0),
            transform("root", "outside", [0.0; 3], 0.0),
        ]),
    )
    .unwrap();
    // A frame of the cycle in its ancestor: the walk reaches it before it closes the cycle.
    assert_close(position_of(&tree, "b", "leaf", at(1)), [1.0, 1.0, 0.0]);
    for time in [at(1), LookupTime::Latest] {
        let error = tree
            .lookup(&frame("outside"), &frame("leaf"), time)
            .unwrap_err();
        assert!(
            matches!(error, LookupError::BrokenTree(BrokenTree::Cycle { .. })),
            "{error}"
        );
    }
    let error = tree
        .lookup(&frame("outside"), &frame("leaf"), at(1))
        .unwrap_err();
    assert_eq!(
        error,
        LookupError::BrokenTree(BrokenTree::Cycle {
            transforms: vec![
                transform_name("b", "a"),
                transform_name("c", "b"),
                transform_name("a", "c")
            ]
        })
    );
    assert_eq!(
        error.to_string(),
        "the transforms b -> a, c -> b, a -> c form a cycle: the chain of parents of each frame \
         ends at a root, so fix the publisher of one of them"
    );
}

#[test]
fn a_static_set_replaces_the_publishers_last_one() {
    let mut tree = robot();
    tree.take_static(
        &publisher("impostor"),
        &fixed(vec![transform("base", "lidar", [5.0, 0.0, 0.0], 0.0)]),
    )
    .unwrap();
    let broken = tree
        .lookup(&frame("base"), &frame("lidar"), at(15))
        .unwrap_err();
    assert!(
        matches!(
            broken,
            LookupError::BrokenTree(BrokenTree::TwoPublishers { .. })
        ),
        "{broken}"
    );
    // The impostor's next set omits the lidar: its transform goes, and the model's holds again.
    tree.take_static(&publisher("impostor"), &fixed(vec![]))
        .unwrap();
    assert_close(position_of(&tree, "base", "lidar", at(15)), [1.0, 0.0, 0.0]);
    // The model's next set moves the lidar.
    tree.take_static(
        &publisher("model"),
        &fixed(vec![transform("base", "lidar", [2.0, 0.0, 0.0], 0.0)]),
    )
    .unwrap();
    assert_close(position_of(&tree, "base", "lidar", at(15)), [2.0, 0.0, 0.0]);
    // A dropped transform leaves its frames known, as tf2 forgets no frame.
    tree.take_static(&publisher("model"), &fixed(vec![]))
        .unwrap();
    assert_eq!(
        tree.lookup(&frame("base"), &frame("lidar"), at(15)),
        Err(LookupError::NoChain {
            target_frame: frame("base"),
            source_frame: frame("lidar")
        })
    );
}

#[test]
fn a_transform_sent_static_then_moving_holds_its_last_kind() {
    let mut tree = tree();
    tree.take_static(
        &publisher("p"),
        &fixed(vec![transform("a", "b", [1.0, 0.0, 0.0], 0.0)]),
    )
    .unwrap();
    tree.take_moving(
        &publisher("p"),
        &moving(10, vec![transform("a", "b", [2.0, 0.0, 0.0], 0.0)]),
    )
    .unwrap();
    assert_close(position_of(&tree, "a", "b", at(10)), [2.0, 0.0, 0.0]);
    assert!(tree.lookup(&frame("a"), &frame("b"), at(11)).is_err());
    tree.take_static(
        &publisher("p"),
        &fixed(vec![transform("a", "b", [3.0, 0.0, 0.0], 0.0)]),
    )
    .unwrap();
    assert_close(position_of(&tree, "a", "b", at(11)), [3.0, 0.0, 0.0]);
}

#[test]
fn a_sample_older_than_the_window_is_refused_by_its_child_frame() {
    let mut tree = tree();
    let insert = |tree: &mut FrameTree, stamp| {
        tree.take_moving(
            &publisher("p"),
            &moving(stamp, vec![transform("a", "b", [0.0; 3], 0.0)]),
        )
        .unwrap()
    };
    assert!(insert(&mut tree, 500).is_empty());
    assert!(insert(&mut tree, 400).is_empty());
    let refused = insert(&mut tree, 399);
    assert_eq!(
        refused,
        vec![TransformError {
            index: 0,
            child: "b".into(),
            reason: TransformErrorReason::BeforeWindow {
                stamp: Stamp::from_nanos(399),
                newest: Stamp::from_nanos(500),
            },
        }]
    );
    assert_eq!(
        refused[0].to_string(),
        "transform 0 (child frame \"b\") is refused: its timestamp 0.000000399 s is more than \
         the cache window before 0.000000500 s, the newest stamp held of its child frame: send \
         it within the cache window of the newest, or set a wider cache window"
    );
}

#[test]
fn an_unknown_frame_names_every_frame_the_tree_holds() {
    let failure = robot()
        .lookup(&frame("camera"), &frame("base"), at(15))
        .expect_err("no transform names camera");
    assert_eq!(
        failure,
        LookupError::UnknownFrame {
            frame: frame("camera"),
            held: ["arm", "base", "lidar", "odom"].map(frame).to_vec(),
        }
    );
    assert_eq!(
        failure.to_string(),
        "no transform of the tree names the frame camera: look up one of the frames it holds: \
         arm, base, lidar, odom"
    );
    assert_eq!(
        tree()
            .lookup(&frame("camera"), &frame("base"), at(15))
            .expect_err("an empty tree")
            .to_string(),
        "no transform of the tree names the frame camera: the tree holds no frame yet, since \
         no publisher's transform has reached it"
    );
}

#[test]
fn a_frame_name_is_matched_byte_for_byte() {
    let tree = robot();
    assert_eq!(
        unknown_frame(tree.lookup(&frame("/odom"), &frame("base"), at(15))),
        frame("/odom")
    );
    let mut rig = self::tree();
    rig.take_static(
        &publisher("model"),
        &fixed(vec![transform("/rig", "camera", [1.0, 0.0, 0.0], 0.0)]),
    )
    .unwrap();
    assert_close(position_of(&rig, "/rig", "camera", at(15)), [1.0, 0.0, 0.0]);
}

#[test]
fn a_cache_window_of_zero_is_refused() {
    assert_eq!(FrameTree::new(Duration::ZERO).unwrap_err(), ZeroWindow);
    assert_eq!(
        ZeroWindow.to_string(),
        "the cache window is 0 s: give a window above 0 s, as the transforms contract asks"
    );
}

#[test]
fn an_unknown_target_comes_before_an_unknown_source() {
    let tree = robot();
    assert_eq!(
        unknown_frame(tree.lookup(&frame("nowhere"), &frame("elsewhere"), at(15))),
        frame("nowhere")
    );
}

/// root -> a, moving: samples at `a`; root -> b, moving: samples at `b`, from `b_publishers`.
fn siblings(a: &[u64], b: &[u64], b_publishers: &[&'static str]) -> FrameTree {
    let mut tree = tree();
    for &stamp in a {
        tree.take_moving(
            &publisher("a"),
            &moving(stamp, vec![transform("root", "a", [0.0; 3], 0.0)]),
        )
        .unwrap();
    }
    for &stamp in b {
        for name in b_publishers {
            tree.take_moving(
                &publisher(name),
                &moving(stamp, vec![transform("root", "b", [0.0; 3], 0.0)]),
            )
            .unwrap();
        }
    }
    tree
}

fn failing_child(error: LookupError) -> FrameName {
    match error {
        LookupError::Extrapolation { transform, .. } => transform.child,
        LookupError::BrokenTree(BrokenTree::TwoPublishers { child, .. }) => child,
        error => panic!("{error}"),
    }
}

#[test]
fn the_source_walks_failure_comes_before_the_target_walks() {
    // a has no value at 50; b breaks the tree.
    let tree = siblings(&[10], &[10, 20], &["b", "other"]);
    for time in [at(50), at(5)] {
        let error = tree.lookup(&frame("b"), &frame("a"), time).unwrap_err();
        assert_eq!(failing_child(error), frame("a"), "at {time}");
    }
    // b broken on the source's walk, a with no value on the target's.
    let error = tree.lookup(&frame("a"), &frame("b"), at(50)).unwrap_err();
    assert_eq!(failing_child(error), frame("b"));
}

#[test]
fn of_two_missing_values_a_tie_names_the_sources() {
    // Both hold samples up to 20; at 50 neither has a value.
    let tree = siblings(&[10, 20], &[10, 20], &["b"]);
    let error = tree.lookup(&frame("b"), &frame("a"), at(50)).unwrap_err();
    assert_eq!(failing_child(error), frame("a"));
    let error = tree.lookup(&frame("a"), &frame("b"), at(50)).unwrap_err();
    assert_eq!(failing_child(error), frame("b"));
}

#[test]
fn a_cycle_on_the_targets_walk_comes_after_the_sources_failure() {
    let mut tree = tree();
    tree.take_moving(
        &publisher("a"),
        &moving(10, vec![transform("root", "a", [0.0; 3], 0.0)]),
    )
    .unwrap();
    tree.take_static(
        &publisher("model"),
        &fixed(vec![
            transform("g", "f", [0.0; 3], 0.0),
            transform("f", "g", [0.0; 3], 0.0),
            transform("f", "leaf", [0.0; 3], 0.0),
        ]),
    )
    .unwrap();
    let error = tree
        .lookup(&frame("leaf"), &frame("a"), at(50))
        .unwrap_err();
    assert_eq!(failing_child(error), frame("a"));
}

#[test]
fn the_latest_time_and_a_time_name_the_same_broken_transform() {
    let mut tree = siblings(&[10, 20], &[10, 20], &["b", "other"]);
    tree.take_moving(
        &publisher("impostor"),
        &moving(20, vec![transform("root", "a", [0.0; 3], 0.0)]),
    )
    .unwrap();
    for time in [LookupTime::Latest, at(15)] {
        let error = tree.lookup(&frame("b"), &frame("a"), time).unwrap_err();
        assert_eq!(failing_child(error), frame("a"), "at {time}");
    }
}

#[test]
fn the_latest_time_leaves_out_the_transforms_above_the_common_ancestor() {
    // root -> mid holds 5 alone; mid -> b holds 10 and 20, mid -> c holds 10 and 30.
    let mut tree = tree();
    tree.take_moving(
        &publisher("mid"),
        &moving(5, vec![transform("root", "mid", [0.0; 3], 0.0)]),
    )
    .unwrap();
    for (stamp, child) in [(10, "b"), (20, "b"), (10, "c"), (30, "c")] {
        tree.take_moving(
            &publisher("leaves"),
            &moving(stamp, vec![transform("mid", child, [0.0; 3], 0.0)]),
        )
        .unwrap();
    }
    let found = tree
        .lookup(&frame("c"), &frame("b"), LookupTime::Latest)
        .unwrap();
    assert_eq!(found.time, Some(Stamp::from_nanos(20)));
}

#[test]
fn the_latest_time_counts_the_targets_transform_into_the_common_parent() {
    // mid -> b holds 10 and 20; mid -> c holds 5 and 15, the oldest newest stamp.
    let mut tree = tree();
    for (stamp, child) in [(10, "b"), (20, "b"), (5, "c"), (15, "c")] {
        tree.take_moving(
            &publisher("leaves"),
            &moving(stamp, vec![transform("mid", child, [0.0; 3], 0.0)]),
        )
        .unwrap();
    }
    let found = tree
        .lookup(&frame("c"), &frame("b"), LookupTime::Latest)
        .unwrap();
    assert_eq!(found.time, Some(Stamp::from_nanos(15)));
}

#[test]
fn a_publishers_stamp_a_window_before_the_newest_is_still_held() {
    let mut tree = robot();
    let send = |tree: &mut FrameTree, publisher: &Publisher, stamp| {
        tree.take_moving(
            publisher,
            &moving(stamp, vec![transform("odom", "base", [0.0; 3], 0.0)]),
        )
        .unwrap()
    };
    send(&mut tree, &publisher("impostor"), 20);
    // The impostor's newest, 20, plus the window, 100, is base's newest, 120.
    send(&mut tree, &publisher("base"), 120);
    let error = tree
        .lookup(&frame("odom"), &frame("base"), at(120))
        .unwrap_err();
    assert!(
        matches!(
            error,
            LookupError::BrokenTree(BrokenTree::TwoPublishers { .. })
        ),
        "{error}"
    );
    send(&mut tree, &publisher("base"), 121);
    assert!(tree.lookup(&frame("odom"), &frame("base"), at(121)).is_ok());
}

#[test]
fn a_frame_of_two_publishers_in_itself_has_no_latest_time() {
    let mut tree = robot();
    tree.take_moving(
        &publisher("impostor"),
        &moving(15, vec![transform("odom", "base", [0.0; 3], 0.0)]),
    )
    .unwrap();
    let found = tree
        .lookup(&frame("base"), &frame("base"), LookupTime::Latest)
        .unwrap();
    assert_eq!(found.time, None);
}

#[test]
fn no_chain_names_both_causes_it_can_have() {
    let mut tree = robot();
    tree.clear_moving();
    let error = tree
        .lookup(&frame("odom"), &frame("lidar"), LookupTime::Latest)
        .unwrap_err();
    assert_eq!(
        error.to_string(),
        "no chain of transforms joins lidar to odom: they are in two trees; check that each \
         transform between them is published, and after the clock went back, that each \
         moving one has sent a stamp since"
    );
}

#[test]
fn a_cleared_transform_holds_again_from_its_next_stamp() {
    let mut tree = robot();
    assert_close(position_of(&tree, "odom", "base", at(15)), [1.5, 0.0, 0.0]);
    tree.clear_moving();
    assert!(tree.lookup(&frame("odom"), &frame("base"), at(15)).is_err());
    tree.take_moving(
        &publisher("base"),
        &moving(30, vec![transform("odom", "base", [3.0, 0.0, 0.0], 0.0)]),
    )
    .unwrap();
    assert_close(position_of(&tree, "odom", "base", at(30)), [3.0, 0.0, 0.0]);
}

#[test]
fn a_message_gives_each_transform_it_refused_in_its_order() {
    let mut tree = robot();
    tree.take_moving(
        &publisher("arm"),
        &moving(500, vec![transform("base", "arm", [0.0; 3], 0.0)]),
    )
    .unwrap();
    // Its first transform has a NaN coordinate, its second a stamp older than arm's newest
    // by more than the window, and its third its child frame for its parent.
    let mut message = moving(
        5,
        vec![
            transform("odom", "base", [0.0; 3], 0.0),
            transform("base", "arm", [0.0; 3], 0.0),
            transform("mast", "mast", [0.0; 3], 0.0),
        ],
    );
    message.positions[0] = f64::NAN;
    let refused = tree.take_moving(&publisher("arm"), &message).unwrap();
    let named: Vec<(usize, &str)> = refused
        .iter()
        .map(|error| (error.index, error.child.as_str()))
        .collect();
    assert_eq!(named, vec![(0, "base"), (1, "arm"), (2, "mast")]);
    assert!(
        matches!(refused[1].reason, TransformErrorReason::BeforeWindow { .. }),
        "{}",
        refused[1]
    );
    // A message whose arrays do not line up is refused whole and changes nothing.
    let short = MovingMessage {
        orientations: vec![0.0, 0.0, 0.0, 1.0],
        ..moving(
            600,
            vec![
                transform("base", "arm", [0.0; 3], 0.0),
                transform("odom", "base", [0.0; 3], 0.0),
            ],
        )
    };
    assert!(tree.take_moving(&publisher("arm"), &short).is_err());
    assert_close(position_of(&tree, "odom", "base", at(15)), [1.5, 0.0, 0.0]);
}

#[test]
fn a_publishers_transforms_reach_the_tree_through_their_messages() {
    let transforms =
        [FrameTransform::parse("odom", "base", [2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).unwrap()];
    let mut tree = tree();
    let fixed = StaticMessage::from_transforms(&transforms).unwrap();
    assert!(
        tree.take_static(&publisher("model"), &fixed)
            .unwrap()
            .is_empty()
    );
    assert_close(position_of(&tree, "odom", "base", at(1)), [2.0, 0.0, 0.0]);
    let moving = MovingMessage::from_transforms(time(5), &transforms, start(START)).unwrap();
    assert!(
        tree.take_moving(&publisher("model"), &moving)
            .unwrap()
            .is_empty()
    );
    let latest = tree
        .lookup(&frame("odom"), &frame("base"), LookupTime::Latest)
        .unwrap();
    assert_eq!(latest.time, Some(Stamp::from_nanos(5)));
}

#[test]
fn two_publishers_are_named_by_their_instances() {
    let mut tree = tree();
    for instance_id in ["left", "right"] {
        let publisher = Publisher {
            core_node: "odometry".into(),
            instance_id: instance_id.into(),
        };
        let sent = vec![transform("odom", "base", [0.0; 3], 0.0)];
        assert!(
            tree.take_moving(&publisher, &moving(10, sent))
                .unwrap()
                .is_empty()
        );
    }
    let error = tree
        .lookup(&frame("odom"), &frame("base"), at(10))
        .unwrap_err();
    assert!(
        error.to_string().starts_with(
            "two publishers, instance left of odometry and instance right of odometry, \
             publish the transform of base"
        ),
        "{error}"
    );
}
