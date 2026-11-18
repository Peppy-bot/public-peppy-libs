//! What the loader will not accept, and why each one is a refusal rather than a
//! caveat.
//!
//! Every case here would otherwise produce a plausible wrong answer: a pose the
//! mechanism cannot hold, a joint confined to a range it does not have, or a `q`
//! entry silently ignored. A chain that loads is one whose forward kinematics
//! can be believed.

use chain_kinematics::{Chain, ChainError, ChainSpec, JointSelection, TreeError};

/// A two-joint chain, with `extra` spliced in before the closing tag so each
/// test varies exactly the one thing it is about.
fn urdf(limit_b: &str, extra: &str) -> String {
    format!(
        r#"<?xml version="1.0"?><robot name="t">
  <link name="base"/><link name="a"/><link name="b"/><link name="tip"/>
  <joint name="j_a" type="revolute">
    <parent link="base"/><child link="a"/><axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
  </joint>
  <joint name="j_b" type="revolute">
    <parent link="a"/><child link="b"/><axis xyz="0 1 0"/>
    <origin xyz="0 0 0.1"/>
    {limit_b}
  </joint>
  <joint name="j_tip" type="fixed">
    <parent link="b"/><child link="tip"/><origin xyz="0 0 0.1"/>
  </joint>
  {extra}
</robot>"#
    )
}

const BOUNDED: &str = r#"<limit lower="-2.0" upper="2.0" effort="1" velocity="1"/>"#;

fn build<const N: usize>(urdf: &str, joints: JointSelection<'_>) -> Result<Chain<N>, ChainError> {
    let robot = urdf_rs::read_from_string(urdf).expect("parse fixture");
    Chain::<N>::from_urdf(
        &robot,
        &ChainSpec {
            base_link: None,
            tip_link: "tip",
            joints,
        },
    )
}

#[test]
fn the_two_joint_fixture_loads() {
    build::<2>(&urdf(BOUNDED, ""), JointSelection::PathOrder).expect("fixture is a valid chain");
}

#[test]
fn a_joint_named_twice_is_refused() {
    // Otherwise the later slot wins and the earlier entry of `q` moves nothing,
    // so the caller commands two joints and the arm honours one.
    let err = build::<2>(&urdf(BOUNDED, ""), JointSelection::Named(&["j_a", "j_a"]))
        .expect_err("a joint cannot be driven by two entries of q");
    assert!(
        matches!(&err, ChainError::DuplicateJoint(name) if name == "j_a"),
        "expected a duplicate-joint refusal, got {err}"
    );
}

#[test]
fn an_actuated_joint_with_no_declared_range_is_refused() {
    let err = build::<2>(&urdf("", ""), JointSelection::PathOrder)
        .expect_err("a joint with no <limit> has no range to clamp into");
    assert!(
        matches!(&err, ChainError::NoRange { joint } if joint == "j_b"),
        "expected a no-range refusal, got {err}"
    );
}

#[test]
fn an_unbounded_actuated_joint_is_refused() {
    // Not confined to a default range: a joint that turns freely would then be
    // clamped out of motion the mechanism has.
    let unbounded = r#"<limit lower="-INF" upper="INF" effort="1" velocity="1"/>"#;
    let err = build::<2>(&urdf(unbounded, ""), JointSelection::PathOrder)
        .expect_err("an infinite bound is not a bound");
    assert!(
        matches!(&err, ChainError::Tree(TreeError::JointLimits { joint, lower, upper })
            if joint == "j_b" && lower.is_infinite() && upper.is_infinite()),
        "expected a limits refusal, got {err}"
    );
}

#[test]
fn an_actuated_continuous_joint_is_refused_for_its_range() {
    // A continuous joint declares no range, whatever <limit> it carries.
    let continuous = urdf(BOUNDED, "").replacen(
        r#"<joint name="j_b" type="revolute">"#,
        r#"<joint name="j_b" type="continuous">"#,
        1,
    );
    let err = build::<2>(&continuous, JointSelection::PathOrder)
        .expect_err("a continuous joint has no range to clamp into");
    assert!(
        matches!(&err, ChainError::NoRange { joint } if joint == "j_b"),
        "expected a no-range refusal, got {err}"
    );
}

#[test]
fn a_mimic_coupling_onto_the_chain_is_refused() {
    // `q` poses only the joints it names, so the follower would stay behind and
    // the reported tip would be a pose the mechanism cannot reach.
    let follower = r#"<link name="f"/>
      <joint name="j_f" type="revolute">
        <parent link="b"/><child link="f"/><axis xyz="0 0 1"/>
        <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
        <mimic joint="j_b"/>
      </joint>"#;
    let err = build::<2>(&urdf(BOUNDED, follower), JointSelection::PathOrder)
        .expect_err("a coupling with one end on the chain is not drivable here");
    assert!(
        matches!(&err, ChainError::MimicCoupling { follower, leader }
            if follower == "j_f" && leader == "j_b"),
        "expected a mimic refusal, got {err}"
    );
}

#[test]
fn a_mimic_coupling_clear_of_the_chain_loads() {
    // A gripper's two fingers mimic each other past the tip. That is another
    // mechanism's business, and refusing it would reject every real gripper.
    let fingers = r#"<link name="f1"/><link name="f2"/>
      <joint name="j_f1" type="prismatic">
        <parent link="tip"/><child link="f1"/><axis xyz="0 1 0"/>
        <limit lower="0.0" upper="0.04" effort="1" velocity="1"/>
      </joint>
      <joint name="j_f2" type="prismatic">
        <parent link="tip"/><child link="f2"/><axis xyz="0 -1 0"/>
        <limit lower="0.0" upper="0.04" effort="1" velocity="1"/>
        <mimic joint="j_f1"/>
      </joint>"#;
    build::<2>(&urdf(BOUNDED, fingers), JointSelection::PathOrder)
        .expect("a coupling below the tip is not this chain's concern");
}

#[test]
fn the_midpoint_is_the_centre_of_the_range() {
    let chain = build::<2>(&urdf(BOUNDED, ""), JointSelection::PathOrder).expect("fixture loads");
    let [a, b] = chain.limits();
    assert_eq!(a.midpoint(), 0.0);
    assert_eq!(b.midpoint(), 0.0);
    assert!(a.contains(a.midpoint()) && b.contains(b.midpoint()));
}

#[test]
fn a_multi_slot_joint_anywhere_is_refused() {
    // A floating joint needs six configuration values; posing it as anything
    // simpler silently welds or misplaces every link below it.
    let floater = r#"<link name="free"/>
      <joint name="j_free" type="floating">
        <parent link="base"/><child link="free"/>
      </joint>"#;
    let err = build::<2>(&urdf(BOUNDED, floater), JointSelection::PathOrder)
        .expect_err("a floating joint cannot be posed by one value");
    assert!(
        matches!(
            &err,
            ChainError::Tree(TreeError::MultiPositionJoint { joint, kind })
                if joint == "j_free" && kind == "floating"
        ),
        "expected a multi-slot refusal, got {err}"
    );
}

/// The two-joint fixture with `edit` applied to its joint `j_b` after parsing, so a
/// value no URDF text parses into, such as a NaN, reaches the tree.
fn build_with_j_b(edit: impl FnOnce(&mut urdf_rs::Joint)) -> Result<Chain<2>, ChainError> {
    let mut robot = urdf_rs::read_from_string(&urdf(BOUNDED, "")).expect("parse fixture");
    edit(
        robot
            .joints
            .iter_mut()
            .find(|joint| joint.name == "j_b")
            .expect("the fixture's j_b"),
    );
    Chain::<2>::from_urdf(
        &robot,
        &ChainSpec {
            base_link: None,
            tip_link: "tip",
            joints: JointSelection::PathOrder,
        },
    )
}

#[test]
fn a_joint_whose_axis_gives_no_direction_is_refused() {
    // Normalizing such an axis gives NaN, and every pose below the joint with it.
    for axis in [
        [0.0, 0.0, 0.0],
        [1e-13, 0.0, 0.0],
        [f64::NAN, 0.0, 1.0],
        [0.0, f64::INFINITY, 0.0],
    ] {
        let err = build_with_j_b(|joint| joint.axis.xyz = urdf_rs::Vec3(axis))
            .expect_err("an axis with no direction cannot be turned about");
        assert!(
            matches!(
                &err,
                ChainError::Tree(TreeError::JointAxis { joint, axis: refused })
                    if joint == "j_b" && refused.iter().zip(axis).all(|(a, b)| a.total_cmp(&b).is_eq())
            ),
            "expected an axis refusal for {axis:?}, got {err}"
        );
    }
}

#[test]
fn a_moving_joint_whose_limits_are_not_finite_and_in_order_is_refused() {
    // A joint confined between such limits has no position a caller can name.
    for (lower, upper) in [
        (1.0, -1.0),
        (f64::NAN, 1.0),
        (-1.0, f64::INFINITY),
        (f64::NEG_INFINITY, 1.0),
    ] {
        let err = build_with_j_b(|joint| {
            joint.limit.lower = lower;
            joint.limit.upper = upper;
        })
        .expect_err("limits out of order or not finite bound no range");
        assert!(
            matches!(
                &err,
                ChainError::Tree(TreeError::JointLimits { joint, lower: l, upper: u })
                    if joint == "j_b" && l.total_cmp(&lower).is_eq() && u.total_cmp(&upper).is_eq()
            ),
            "expected a limits refusal for [{lower}, {upper}], got {err}"
        );
    }
}

#[test]
fn a_link_named_twice_is_refused() {
    // Every joint addresses its parent and child by name, so a repeated link
    // name makes the tree ambiguous: whichever link the lookup happens to find
    // decides where the chain runs, and forward kinematics reports a pose built
    // from the wrong body.
    let duplicate = r#"<link name="a"/>"#;
    let err = build::<2>(&urdf(BOUNDED, duplicate), JointSelection::PathOrder)
        .expect_err("two links cannot share a name");
    assert!(
        matches!(&err, ChainError::Tree(TreeError::LinkTwice(link)) if link == "a"),
        "expected a duplicate-link refusal, got {err}"
    );
}

#[test]
fn a_urdf_joint_given_twice_is_refused() {
    // A joint is named by the spec, the SRDF and a mimic: a repeated joint name
    // leaves which of the two each of them means to whichever lookup runs.
    let twice = r#"<link name="spare"/>
      <joint name="j_a" type="fixed"><parent link="tip"/><child link="spare"/></joint>"#;
    let err = build::<2>(&urdf(BOUNDED, twice), JointSelection::PathOrder)
        .expect_err("two joints cannot share a name");
    assert!(
        matches!(&err, ChainError::Tree(TreeError::JointTwice(joint)) if joint == "j_a"),
        "expected a duplicate-joint refusal, got {err}"
    );
}

#[test]
fn a_joint_naming_a_link_the_urdf_lacks_is_refused() {
    let dangling = r#"<joint name="j_dangling" type="fixed">
        <parent link="tip"/><child link="nowhere"/>
      </joint>"#;
    let err = build::<2>(&urdf(BOUNDED, dangling), JointSelection::PathOrder)
        .expect_err("a joint to no link joins nothing");
    assert_eq!(
        err.to_string(),
        "the joint \"j_dangling\" names the link \"nowhere\", and the URDF has no such link"
    );
}

#[test]
fn a_link_with_two_parents_is_refused() {
    // A closed loop is not a tree. Walking it from the tip up would reach the
    // base by more than one route, and only one of them carries the transforms
    // this chain would then pose.
    let second_parent = r#"<joint name="j_loop" type="revolute">
        <parent link="base"/><child link="b"/><axis xyz="1 0 0"/>
        <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
      </joint>"#;
    let err = build::<2>(&urdf(BOUNDED, second_parent), JointSelection::PathOrder)
        .expect_err("a link reached two ways is not a tree");
    assert!(
        matches!(
            &err,
            ChainError::Tree(TreeError::LinkOnTwoJoints { link, first, second })
                if link == "b" && first == "j_b" && second == "j_loop"
        ),
        "expected a two-parent refusal, got {err}"
    );
}

#[test]
fn a_urdf_with_two_roots_is_refused() {
    // Two disconnected trees have no common frame, so "the base" is a choice
    // rather than a fact, and every pose would be expressed against a root the
    // caller never named.
    let orphan = r#"<link name="island"/>"#;
    let err = build::<2>(&urdf(BOUNDED, orphan), JointSelection::PathOrder)
        .expect_err("two roots leave the base ambiguous");
    assert!(
        matches!(&err, ChainError::Tree(TreeError::RootLinks { roots }) if roots == &["base", "island"]),
        "expected a multiple-root refusal, got {err}"
    );
}

#[test]
fn a_ring_of_links_beside_the_tree_is_refused() {
    // A ring has no root of its own: every link in it has a parent joint, so a
    // URDF carrying one alongside a real tree still presents exactly one root and
    // no link with two parents. Neither of the refusals above sees it, and every
    // walk here follows a link's parent upward, which around a ring never ends.
    let ring = r#"<link name="ring_x"/><link name="ring_y"/>
      <joint name="j_ring_out" type="revolute">
        <parent link="ring_x"/><child link="ring_y"/><axis xyz="0 0 1"/>
        <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
      </joint>
      <joint name="j_ring_back" type="revolute">
        <parent link="ring_y"/><child link="ring_x"/><axis xyz="0 0 1"/>
        <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
      </joint>"#;
    let err = build::<2>(&urdf(BOUNDED, ring), JointSelection::PathOrder)
        .expect_err("a ring is not part of the tree, and cannot be walked out of");
    assert!(
        matches!(
            &err,
            ChainError::Tree(TreeError::JointLoop { joints }) if joints == &["j_ring_out", "j_ring_back"]
        ),
        "expected a loop refusal, got {err}"
    );
}
