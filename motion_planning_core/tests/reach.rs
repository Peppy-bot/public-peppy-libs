//! The reach of the arm's joints: on a straight joint motion, no point of a link or of a
//! body a link carries moves farther than the reach bounds, and along a rounded path no
//! farther than the lever of the stretch's tangent; on OpenArm v2's left arm with its
//! fingers and an object held past its grasp point, and on drawn robots of revolute and
//! prismatic joints, each carrying a part.

mod common;

use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{CarriedPart, ChainGeometry, Rider, RiderBody};
use collision_geometry::shapes::{Shape, ShapeKind};
use control_core::path_timing::RoundedPath;
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{
    ChainProperty, TIP, configs, link_name, on_a_drawn_robot, placements, unit_vectors,
};
use motion_planning_core::{Arm, Body, RiderOffTheArm};
use nalgebra::{Isometry3, Point3, Vector3};
use urdf_rs::{
    Axis, Collision, Geometry, Joint, JointLimit, JointType, Link, LinkName, Robot, Vec3,
};

use common::{JOINTS, LEFT_ARM, left_fingers};

/// The points of `body` in its own frame that move farthest under a rigid motion, with
/// the pose that places them: a piece's vertices, each pushed out by the hull's inflation
/// along `out`, and the corners of a box or the vertices of a mesh.
fn extreme_points(body: &Body, out: &Vector3<f64>) -> (Vec<Point3<f64>>, Isometry3<f64>) {
    match body {
        Body::Piece(piece) => {
            let hull = piece.hull();
            let points = hull
                .vertices()
                .iter()
                .map(|vertex| vertex + out * hull.inflation())
                .collect();
            (points, *piece.pose())
        }
        Body::Shape(shape) => {
            let points = match shape.shape().kind() {
                ShapeKind::Box { size } => (0..8)
                    .map(|i| {
                        let side = |bit: usize, axis: usize| {
                            let half = size[axis] / 2.0;
                            if i & bit == 0 { -half } else { half }
                        };
                        Point3::new(side(1, 0), side(2, 1), side(4, 2))
                    })
                    .collect(),
                ShapeKind::Mesh { vertices, .. } => vertices.to_vec(),
                kind => panic!("the test's objects are boxes and meshes, not {kind:?}"),
            };
            (points, *shape.pose())
        }
    }
}

/// Asserts that no point of `arm` moves farther than `bound` from where it stands at
/// `from` to where it stands at `to`, to the rounding of the arithmetic.
fn assert_moves_within<const N: usize>(
    arm: &Arm<N>,
    from: &[f64; N],
    to: &[f64; N],
    bound: f64,
    out: &Vector3<f64>,
) {
    for ((link, a), (_, b)) in arm.bodies(from).iter().zip(&arm.bodies(to)) {
        let (points, at_a) = extreme_points(a, out);
        let (_, at_b) = extreme_points(b, out);
        let moved = points
            .iter()
            .map(|point| (at_b * point - at_a * point).norm())
            .fold(0.0, f64::max);
        assert!(
            moved <= bound * (1.0 + 1e-9) + 1e-12,
            "a point of {link} moves {moved} m, past the bound {bound} m"
        );
    }
}

/// On drawn configurations and drawn turns of drawn joints of `arm`, no point of a link
/// or a rider moves farther than the reach bounds, the sum over the joints of each one's
/// turn times its reach. Along the rounded path through three drawn configurations, from
/// one drawn position to another, none moves farther than the lever of the stretch's
/// largest tangent shares times its length.
fn assert_moves_within_reach<const N: usize>(tc: &TestCase, arm: &Arm<N>) {
    let limits = arm.geometry().chain().limits();
    let out = tc.draw(unit_vectors());
    let from = tc.draw(configs(limits));
    let other = tc.draw(configs(limits));
    let turned: Vec<bool> = (0..N).map(|_| tc.draw(gs::booleans())).collect();
    let to: [f64; N] = std::array::from_fn(|i| if turned[i] { other[i] } else { from[i] });
    assert_moves_within(arm, &from, &to, arm.displacement(&from, &to), &out);
    let waypoints: Vec<Vec<f64>> = (0..3).map(|_| tc.draw(configs(limits)).to_vec()).collect();
    let path = RoundedPath::joint_path(&waypoints);
    let position = || -> f64 { tc.draw(gs::floats().min_value(0.0).max_value(path.length())) };
    let (a, b) = (position(), position());
    let (start, end) = (a.min(b), a.max(b));
    let at = |s: f64| -> [f64; N] { path.at(s).as_slice().try_into().expect("a configuration") };
    let share: [f64; N] = path
        .max_abs_tangent(start, end)
        .try_into()
        .expect("a share of the tangent per joint");
    let lever = arm.lever_along(&share);
    assert_moves_within(arm, &at(start), &at(end), lever * (end - start), &out);
}

/// How far past the grasp point, along its z axis, the hand holds the object of the
/// test: past every link.
const OBJECT_AHEAD_M: f64 = 0.15;

#[hegel::test]
fn no_point_of_openarm_moves_farther_than_its_reach(tc: TestCase) {
    let object = Shape::cuboid([0.06, 0.04, 0.08]).expect("a box");
    let zero = [0.0; JOINTS];
    let chain = LEFT_ARM.chain();
    let grasp_point =
        chain.at(&zero).link_pose_world(JOINTS - 1).inverse() * chain.world_ee_pose(&zero);
    let held = Rider {
        name: "object",
        segment: JOINTS - 1,
        offset: grasp_point * Isometry3::translation(0.0, 0.0, OBJECT_AHEAD_M),
        body: RiderBody::Shape(&object),
    };
    let riders: Vec<Rider> = left_fingers().into_iter().chain([held]).collect();
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the fingers and the object ride the hand");
    assert_moves_within_reach(&tc, &arm);
}

/// The link of a dressed robot on a prismatic joint off a drawn link of its chain.
const PART: &str = "part";

/// A box from 1 cm to 20 cm on a side, on a drawn frame of its link.
#[hegel::composite]
fn boxes(tc: &TestCase) -> Collision {
    Collision {
        name: None,
        origin: tc.draw(placements()),
        geometry: Geometry::Box {
            size: Vec3(tc.draw(gs::arrays(gs::floats().min_value(0.01).max_value(0.2)))),
        },
    }
}

/// `robot`, drawn with `joints` actuated joints, with a box on every link and a box on
/// [`PART`], which a prismatic joint the chain does not drive hangs off a drawn link that
/// a joint of the chain moves, the part's joint at 0 inside its limits.
fn dressed(tc: &TestCase, robot: &Robot, joints: usize) -> Robot {
    let carrier = link_name(tc.draw(gs::integers::<usize>().min_value(1).max_value(joints)));
    let part_joint = Joint {
        name: format!("{PART}_joint"),
        joint_type: JointType::Prismatic,
        origin: tc.draw(placements()),
        parent: LinkName { link: carrier },
        child: LinkName {
            link: PART.to_owned(),
        },
        axis: Axis {
            xyz: Vec3(tc.draw(unit_vectors()).into()),
        },
        limit: JointLimit {
            lower: -0.1,
            upper: 0.1,
            effort: 1.0,
            velocity: 1.0,
            ..JointLimit::default()
        },
        calibration: None,
        dynamics: None,
        mimic: None,
        safety_controller: None,
    };
    let part = Link {
        name: PART.to_owned(),
        inertial: Default::default(),
        visual: Vec::new(),
        collision: Vec::new(),
    };
    Robot {
        links: robot
            .links
            .iter()
            .cloned()
            .chain([part])
            .map(|link| Link {
                collision: vec![tc.draw(boxes())],
                ..link
            })
            .collect(),
        joints: robot.joints.iter().cloned().chain([part_joint]).collect(),
        ..robot.clone()
    }
}

struct MovesWithinReach;

impl ChainProperty for MovesWithinReach {
    fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
        let urdf = urdf_rs::write_to_string(&dressed(tc, robot, N)).expect("a drawn robot writes");
        let geometry = ChainGeometry::<N>::from_urdf(
            &urdf,
            "",
            &ChainSpec {
                base_link: None,
                tip_link: TIP,
                joints: JointSelection::PathOrder,
            },
        )
        .expect("a dressed robot fits");
        let riders: Vec<Rider> = geometry.carried().iter().map(CarriedPart::rider).collect();
        assert_eq!(
            riders.iter().map(|rider| rider.name).collect::<Vec<_>>(),
            [PART],
            "the chain carries the part"
        );
        let arm = Arm::new(&geometry, None)
            .riding(&riders)
            .expect("the part rides a link of the chain");
        assert_moves_within_reach(tc, &arm);
    }
}

// On drawn robots of one to seven revolute and prismatic joints, each carrying a part off
// a drawn link, no point moves farther than the reach bounds or the lever of a stretch of
// a rounded path.
#[hegel::test]
fn no_point_of_a_drawn_robot_moves_farther_than_its_reach(tc: TestCase) {
    on_a_drawn_robot::<MovesWithinReach>(&tc);
}

// A rider on the child link of no joint of the arm is refused, naming it.
#[test]
fn a_rider_off_the_arm_is_refused() {
    let fingers = left_fingers();
    let off = [Rider {
        segment: JOINTS,
        ..fingers[0]
    }];
    assert_eq!(
        Arm::new(&LEFT_ARM, None).riding(&off).err(),
        Some(RiderOffTheArm {
            name: fingers[0].name.to_string(),
            segment: JOINTS,
            last: JOINTS - 1,
        })
    );
}
