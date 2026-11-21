//! The botshare base's disabled pairs, which robot_description/tools/botshare_base.py
//! writes into its SRDF: the base is rigid but for the wheels of its differential base,
//! each pair of links with collision geometry that a joint joins, or that links with no
//! collision geometry alone join between them, is Adjacent, and each other pair is Never
//! where the bounds of the two links with every joint at zero stand apart. A wheel turns about
//! its axis, on which its cylinder is centered, so its bounds hold at every angle.

use std::collections::{BTreeMap, BTreeSet};

use collision_geometry::nalgebra::Point3;
use collision_geometry::shapes::{PlacedShape, Shape};
use robot_description::chain_kinematics::pose_of_origin;
use robot_description::urdf_rs::{Geometry, JointType};
use robot_description::{DisabledReason, RobotDescription};

const MODEL: &str = "botshare_base";

/// The least and the greatest corner of the box along the robot frame's axes that holds a
/// link's collision elements.
type Bounds = (Point3<f64>, Point3<f64>);

fn description() -> RobotDescription {
    robot_description::model(MODEL)
        .expect("robot_description bundles the botshare base")
        .description()
}

/// The shape of a collision element, a box, a cylinder or a sphere.
fn shape_of(geometry: &Geometry) -> Shape {
    match *geometry {
        Geometry::Box { ref size } => Shape::cuboid(size.0),
        Geometry::Cylinder { radius, length } => Shape::cylinder(radius, length),
        Geometry::Sphere { radius } => Shape::sphere(radius),
        ref other => panic!("the botshare base's collision elements are primitives: {other:?}"),
    }
    .expect("a collision element of positive dimensions")
}

/// The bounds of each link with collision geometry, every joint at zero.
fn link_bounds(description: &RobotDescription) -> BTreeMap<String, Bounds> {
    description
        .link_frames_at_zero()
        .into_iter()
        .filter(|(link, _)| !link.collision.is_empty())
        .map(|(link, frame)| {
            let bounds = link
                .collision
                .iter()
                .map(|collision| {
                    let shape = shape_of(&collision.geometry);
                    PlacedShape::new(&shape, frame * pose_of_origin(&collision.origin)).bounds()
                })
                .reduce(|(low, high), (other_low, other_high)| {
                    (low.inf(&other_low), high.sup(&other_high))
                })
                .expect("a link with collision geometry");
            (link.name.clone(), bounds)
        })
        .collect()
}

/// Whether two bounds stand apart along an axis.
fn apart((low, high): Bounds, (other_low, other_high): Bounds) -> bool {
    (0..3).any(|axis| high[axis] < other_low[axis] || other_high[axis] < low[axis])
}

/// Whether links with no collision geometry alone stand between `a` and `b` on the tree:
/// a joint joins them, or a path through links with none of it.
fn joined(
    parent_of: &BTreeMap<&str, &str>,
    with_geometry: &BTreeSet<&str>,
    a: &str,
    b: &str,
) -> bool {
    let up = |link: &str| -> Vec<String> {
        std::iter::successors(Some(link.to_owned()), |at| {
            parent_of.get(at.as_str()).map(|above| above.to_string())
        })
        .collect()
    };
    let (from_a, from_b) = (up(a), up(b));
    let meeting = from_a
        .iter()
        .find(|link| from_b.contains(link))
        .expect("two links of one tree meet at its root");
    // The links of `path` after its first and before the meeting.
    let between = |path: &[String]| -> Vec<String> {
        path.iter()
            .take_while(|link| *link != meeting)
            .skip(1)
            .cloned()
            .collect()
    };
    let mut inside = between(&from_a);
    inside.extend(between(&from_b));
    if meeting != a && meeting != b {
        inside.push(meeting.clone());
    }
    inside
        .iter()
        .all(|link| !with_geometry.contains(link.as_str()))
}

fn ordered(a: &str, b: &str) -> (String, String) {
    if a <= b {
        (a.to_owned(), b.to_owned())
    } else {
        (b.to_owned(), a.to_owned())
    }
}

// The base moves only by the wheels of its differential base: every other joint is fixed.
#[test]
fn the_base_is_rigid_but_for_its_wheels() {
    let description = description();
    let base = description
        .differential_base()
        .expect("the botshare base is a differential base");
    let moving: BTreeSet<&str> = description
        .robot()
        .joints
        .iter()
        .filter(|joint| joint.joint_type != JointType::Fixed)
        .map(|joint| joint.name.as_str())
        .collect();
    assert_eq!(
        moving,
        BTreeSet::from([base.left.joint.as_str(), base.right.joint.as_str()])
    );
}

// The SRDF disables exactly the pairs the bounds derive: Adjacent for the joined pairs,
// Never for every other pair whose bounds stand apart.
#[test]
fn the_srdf_disables_the_pairs_the_bounds_derive() {
    let description = description();
    let bounds = link_bounds(&description);
    let with_geometry: BTreeSet<&str> = bounds.keys().map(String::as_str).collect();
    let parent_of: BTreeMap<&str, &str> = description
        .robot()
        .joints
        .iter()
        .map(|joint| (joint.child.link.as_str(), joint.parent.link.as_str()))
        .collect();
    let mut derived = BTreeMap::new();
    for (k, (first, first_bounds)) in bounds.iter().enumerate() {
        for (second, second_bounds) in bounds.iter().skip(k + 1) {
            let pair = ordered(first, second);
            if joined(&parent_of, &with_geometry, first, second) {
                derived.insert(pair, DisabledReason::Adjacent);
            } else if apart(*first_bounds, *second_bounds) {
                derived.insert(pair, DisabledReason::Never);
            }
        }
    }
    let committed: BTreeMap<(String, String), DisabledReason> = description
        .disabled_pairs()
        .iter()
        .map(|pair| (ordered(&pair.link1, &pair.link2), pair.reason))
        .collect();
    assert_eq!(committed, derived);
}
