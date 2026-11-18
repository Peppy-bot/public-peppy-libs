//! A joint's pose at a position: the child link's frame in the parent link's frame,
//! read from a one-joint URDF of each kind with a drawn origin and axis.

use chain_kinematics::nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use chain_kinematics::{JointKind, Mimic, Tree, TreeError};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{angles, points, unit_vectors};

/// How far two poses computed from the same joint may differ, in meters and radians:
/// a few rounding errors of composing two isometries.
const TOLERANCE: f64 = 1e-12;

/// The URDF joint types, by what each does to its child.
const TURNING: &[&str] = &["revolute", "continuous"];
const SLIDING: &[&str] = &["prismatic"];
const HOLDING: &[&str] = &["fixed"];
const EVERY_KIND: &[&str] = &["revolute", "continuous", "prismatic", "fixed"];

/// A drawn joint: its URDF type, its origin and its axis.
struct DrawnJoint {
    kind: &'static str,
    xyz: Vector3<f64>,
    rpy: [f64; 3],
    axis: Vector3<f64>,
}

impl DrawnJoint {
    /// A joint of one of `kinds`, with any origin and any axis.
    fn draw(tc: &TestCase, kinds: &'static [&'static str]) -> Self {
        let kind: usize = tc.draw(gs::integers().min_value(0).max_value(kinds.len() - 1));
        Self {
            kind: kinds[kind],
            xyz: tc.draw(points()),
            rpy: [tc.draw(angles()), tc.draw(angles()), tc.draw(angles())],
            axis: tc.draw(unit_vectors()),
        }
    }

    fn tree(&self) -> Tree {
        let [roll, pitch, yaw] = self.rpy;
        let urdf = format!(
            r#"<?xml version="1.0"?><robot name="one_joint">
<link name="parent"/><link name="child"/>
<joint name="joint" type="{kind}"><parent link="parent"/><child link="child"/>
<origin xyz="{x} {y} {z}" rpy="{roll} {pitch} {yaw}"/><axis xyz="{ax} {ay} {az}"/>
<limit lower="-4" upper="4" effort="1" velocity="1"/></joint></robot>"#,
            kind = self.kind,
            x = self.xyz.x,
            y = self.xyz.y,
            z = self.xyz.z,
            ax = self.axis.x,
            ay = self.axis.y,
            az = self.axis.z,
        );
        let robot = urdf_rs::read_from_string(&urdf).expect("parse the one-joint URDF");
        Tree::from_robot(&robot).expect("a one-joint URDF is a tree")
    }

    /// The origin as the URDF specification defines it: the translation `xyz` after the
    /// extrinsic roll, pitch and yaw of `rpy`.
    fn origin(&self) -> Isometry3<f64> {
        let [roll, pitch, yaw] = self.rpy;
        Isometry3::from_parts(
            Translation3::from(self.xyz),
            UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        )
    }
}

fn assert_close(actual: &Isometry3<f64>, expected: &Isometry3<f64>) {
    let miss = actual.inverse() * expected;
    assert!(
        miss.translation.vector.norm() <= TOLERANCE && miss.rotation.angle() <= TOLERANCE,
        "pose {actual} is not {expected}"
    );
}

/// At position 0 every joint holds its child at the joint's origin.
#[hegel::test]
fn every_joint_sits_at_its_origin_at_zero(tc: TestCase) {
    let joint = DrawnJoint::draw(&tc, EVERY_KIND);
    assert_close(&joint.tree().joint(0).pose_at(0.0), &joint.origin());
}

/// A revolute or continuous joint turns its child by the position about its axis, in
/// the frame of its origin, so the points of the axis stay where they are.
#[hegel::test]
fn a_turning_joint_turns_about_its_axis(tc: TestCase) {
    let joint = DrawnJoint::draw(&tc, TURNING);
    let q = tc.draw(angles());
    let turned = joint.origin().inverse() * joint.tree().joint(0).pose_at(q);
    assert!(
        turned.translation.vector.norm() <= TOLERANCE,
        "a turn moves no point of the axis"
    );
    let expected = UnitQuaternion::from_scaled_axis(joint.axis * q);
    assert!(
        turned.rotation.angle_to(&expected) <= TOLERANCE,
        "the turn is {q} rad about the axis"
    );
    let on_axis = Point3::from(joint.axis * tc.draw(gs::floats().min_value(-1.0).max_value(1.0)));
    assert!(
        (turned * on_axis - on_axis).norm() <= TOLERANCE,
        "a point of the axis stays"
    );
}

/// A prismatic joint slides its child by the position along its axis, in the frame of
/// its origin, without turning it.
#[hegel::test]
fn a_sliding_joint_slides_along_its_axis(tc: TestCase) {
    let joint = DrawnJoint::draw(&tc, SLIDING);
    let q: f64 = tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
    let slid = joint.origin().inverse() * joint.tree().joint(0).pose_at(q);
    assert!(slid.rotation.angle() <= TOLERANCE, "a slide turns nothing");
    assert!(
        (slid.translation.vector - joint.axis * q).norm() <= TOLERANCE,
        "the slide is {q} m along the axis"
    );
}

/// A fixed joint holds its child at its origin whatever position it is given.
#[hegel::test]
fn a_fixed_joint_ignores_the_position(tc: TestCase) {
    let joint = DrawnJoint::draw(&tc, HOLDING);
    let q: f64 = tc.draw(gs::floats().min_value(-4.0).max_value(4.0));
    assert_close(&joint.tree().joint(0).pose_at(q), &joint.origin());
}

/// A continuous joint turns as a revolute one does, and declares no range whatever
/// `<limit>` its URDF gives.
#[hegel::test]
fn a_continuous_joint_turns_and_declares_no_range(tc: TestCase) {
    let mut joint = DrawnJoint::draw(&tc, &["continuous"]);
    let tree = joint.tree();
    let continuous = tree.joint(0);
    assert!(matches!(
        continuous.placement.kind,
        JointKind::Continuous { .. }
    ));
    assert_eq!(continuous.limit, None);
    let q = tc.draw(angles());
    let pose = continuous.pose_at(q);
    joint.kind = "revolute";
    assert_close(&pose, &joint.tree().joint(0).pose_at(q));
}

/// Two prismatic joints off one base, `follower` carrying `mimic`.
fn mimic_urdf(mimic: &str) -> String {
    format!(
        r#"<?xml version="1.0"?><robot name="mimic">
<link name="base"/><link name="a"/><link name="b"/>
<joint name="leader" type="prismatic"><parent link="base"/><child link="a"/>
<axis xyz="0 1 0"/><limit lower="0" upper="0.04" effort="1" velocity="1"/></joint>
<joint name="follower" type="prismatic"><parent link="base"/><child link="b"/>
<axis xyz="0 1 0"/>{mimic}<limit lower="-0.03" upper="0.01" effort="1" velocity="1"/></joint>
</robot>"#
    )
}

/// A joint's `<mimic>` is read whole: the joint it follows, and its multiplier and
/// offset, 1 and 0 where the `<mimic>` gives none.
#[test]
fn a_mimic_is_read_with_its_multiplier_and_offset() {
    let read = |mimic: &str| {
        let robot = urdf_rs::read_from_string(&mimic_urdf(mimic)).expect("parse the mimic URDF");
        let tree = Tree::from_robot(&robot).expect("a tree");
        let follower = tree.joint_index("follower").expect("the follower");
        tree.joint(follower).mimic.clone()
    };
    let full = read(r#"<mimic joint="leader" multiplier="-1" offset="0.01"/>"#)
        .expect("the follower mimics");
    assert_eq!(
        full,
        Mimic::new("leader".to_string(), -1.0, 0.01).expect("a finite mimic")
    );
    assert_eq!(full.position_from(0.02), -0.01);
    let bare = read(r#"<mimic joint="leader"/>"#).expect("the follower mimics");
    assert_eq!((bare.multiplier(), bare.offset()), (1.0, 0.0));
    assert_eq!(read(""), None);
}

/// A mimic whose multiplier or offset is not finite is refused, naming the joint it
/// follows; the tree that reads one is refused, naming the joint that mimics.
#[test]
fn a_mimic_that_is_not_finite_is_refused() {
    for (multiplier, offset) in [(f64::NAN, 0.0), (1.0, f64::INFINITY)] {
        let error = Mimic::new("leader".to_string(), multiplier, offset)
            .expect_err("a mimic that is not finite");
        assert_eq!(error.joint, "leader");
    }
    let mut robot = urdf_rs::read_from_string(&mimic_urdf(r#"<mimic joint="leader"/>"#))
        .expect("parse the mimic URDF");
    let follower = robot
        .joints
        .iter_mut()
        .find(|joint| joint.name == "follower")
        .expect("the follower");
    follower.mimic.as_mut().expect("a mimic").multiplier = Some(f64::NAN);
    let error = Tree::from_robot(&robot).expect_err("a tree with a mimic that is not finite");
    assert!(
        matches!(&error, TreeError::JointMimic { joint, .. } if joint == "follower"),
        "{error}"
    );
}
