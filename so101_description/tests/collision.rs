//! SO-101's collision geometry as collision_geometry fits and places it: the arm's
//! chain, the base and the jaw hold every link with a body; each link's pieces, one per
//! collision element, hold its mesh within the fit's budget; each link stands where the
//! URDF's joints put it; and the jaw shuts toward the tool point.

use std::path::{Path, PathBuf};
use std::sync::LazyLock;

use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{BodyGeometry, ChainGeometry, LinkGeometry, Piece};
use collision_geometry::fit::fit_pieces;
use collision_geometry::gjk::{self, Hull, Placed, Vertex};
use collision_geometry::hull::exact_hull;
use collision_geometry::nalgebra::{
    Isometry3, Point3, Translation3, Unit, UnitQuaternion, Vector3,
};
use collision_geometry::stl::load_stl;
use collision_geometry::urdf_collision::UrdfCollisions;
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, configs};
use so101_description::{
    ARM_DOF, BASE_LINK, JAW_LINK, TCP_LINK, URDF, jaw_position_rad, write_meshes_to,
};
use urdf_rs::{Geometry, JointType, Pose, Robot};

/// How much larger than its mesh a piece may stand, in meters: collision_geometry's fit's
/// budget, 1 mm.
const FIT_BUDGET_M: f64 = 0.001;

/// The meshes, written once per test binary.
static MESHES_DIR: LazyLock<PathBuf> = LazyLock::new(|| {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("so101_meshes");
    write_meshes_to(&dir).expect("the meshes write");
    dir
});

static ROBOT: LazyLock<Robot> =
    LazyLock::new(|| urdf_rs::read_from_string(URDF).expect("the URDF parses"));

static COLLISIONS: LazyLock<UrdfCollisions> =
    LazyLock::new(|| UrdfCollisions::from_urdf(URDF).expect("the URDF's collisions read"));

/// The arm as a planner reads it: the chain from the base link to the tool point.
static ARM: LazyLock<ChainGeometry<ARM_DOF>> = LazyLock::new(|| {
    ChainGeometry::from_urdf(
        URDF,
        meshes_dir(),
        &ChainSpec {
            base_link: Some(BASE_LINK),
            tip_link: TCP_LINK,
            joints: JointSelection::PathOrder,
        },
    )
    .expect("the arm fits")
});

/// The jaw's pieces, in its own frame.
static JAW: LazyLock<Vec<Hull>> = LazyLock::new(|| fitted(JAW_LINK));

fn meshes_dir() -> &'static str {
    MESHES_DIR.to_str().expect("the mesh directory is UTF-8")
}

fn fitted(link: &str) -> Vec<Hull> {
    let surfaces = COLLISIONS
        .link_surfaces(link, meshes_dir())
        .expect("the link's meshes read");
    fit_pieces(link, &surfaces, None).expect("the link fits")
}

/// The vertices of each collision mesh of `link`, one list per `<collision>` element, each
/// scaled and placed in the link's frame by its element.
fn element_vertices(link: &str) -> Vec<Vec<Point3<f64>>> {
    let link = ROBOT
        .links
        .iter()
        .find(|held| held.name == link)
        .expect("the URDF holds the link");
    link.collision
        .iter()
        .map(|collision| {
            let Geometry::Mesh { filename, scale } = &collision.geometry else {
                panic!("{}'s collision geometry is a mesh", link.name);
            };
            let [sx, sy, sz] = scale.as_ref().map_or([1.0; 3], |scale| **scale);
            let file = Path::new(meshes_dir()).join(
                filename
                    .strip_prefix("collision/")
                    .expect("a mesh under collision/"),
            );
            let origin = isometry(&collision.origin);
            load_stl(file.to_str().expect("a mesh's path is UTF-8"))
                .expect("the mesh reads")
                .into_iter()
                .map(|vertex| origin * Point3::new(vertex.x * sx, vertex.y * sy, vertex.z * sz))
                .collect()
        })
        .collect()
}

/// A URDF origin as a rigid transform: roll, pitch and yaw about the fixed axes, then
/// the translation.
fn isometry(pose: &Pose) -> Isometry3<f64> {
    let [x, y, z] = *pose.xyz;
    let [roll, pitch, yaw] = *pose.rpy;
    Isometry3::from_parts(
        Translation3::new(x, y, z),
        UnitQuaternion::from_euler_angles(roll, pitch, yaw),
    )
}

/// The joint whose child is `link`.
fn parent_joint(link: &str) -> Option<&'static urdf_rs::Joint> {
    ROBOT.joints.iter().find(|joint| joint.child.link == link)
}

/// The arm's revolute joints, from the base link to the tool point.
fn arm_joints() -> Vec<&'static str> {
    let mut joints = Vec::new();
    let mut link = TCP_LINK;
    while let Some(joint) = parent_joint(link) {
        if joint.joint_type == JointType::Revolute {
            joints.push(joint.name.as_str());
        }
        link = &joint.parent.link;
    }
    joints.reverse();
    joints
}

/// Where the URDF's joints put `link` in the frame of the base link, each revolute
/// joint at its position in `positions`.
fn urdf_pose(link: &str, positions: &[(&str, f64)]) -> Isometry3<f64> {
    let Some(joint) = parent_joint(link) else {
        return Isometry3::identity();
    };
    let motion = match joint.joint_type {
        JointType::Fixed => UnitQuaternion::identity(),
        JointType::Revolute => {
            let (_, position) = positions
                .iter()
                .find(|(name, _)| *name == joint.name)
                .expect("a position for each revolute joint");
            let axis = Unit::new_normalize(Vector3::from(*joint.axis.xyz));
            UnitQuaternion::from_axis_angle(&axis, *position)
        }
        ref other => panic!("{} is a {other:?} joint", joint.name),
    };
    urdf_pose(&joint.parent.link, positions) * isometry(&joint.origin) * motion
}

fn assert_same_pose(link: &str, placed: &Isometry3<f64>, urdf: &Isometry3<f64>) {
    let apart = (placed.translation.vector - urdf.translation.vector).norm();
    let turned = placed.rotation.angle_to(&urdf.rotation);
    assert!(
        apart <= TOLERANCE && turned <= TOLERANCE,
        "{link} stands {apart} m and {turned} rad from where the URDF puts it"
    );
}

// The arm's chain carries the links its joints move, the base stands fixed below them
// and the jaw rides a joint the chain does not drive: together they are every link
// with a body.
#[test]
fn the_arm_the_base_and_the_jaw_are_every_link_with_a_body() {
    let base = BodyGeometry::from_urdf(URDF, meshes_dir(), &[]).expect("the base fits");
    assert_eq!(base.link_names().collect::<Vec<_>>(), [BASE_LINK]);
    let mut held: Vec<&str> = ARM.links().iter().map(LinkGeometry::name).collect();
    held.extend([BASE_LINK, JAW_LINK]);
    held.sort_unstable();
    let mut with_a_body: Vec<&str> = ROBOT
        .links
        .iter()
        .filter(|link| !link.collision.is_empty())
        .map(|link| link.name.as_str())
        .collect();
    with_a_body.sort_unstable();
    assert_eq!(held, with_a_body);
}

// Each link holds one piece per collision element, which holds every vertex of the
// element's mesh and stands nowhere farther than the fit's budget beyond the mesh's hull.
#[test]
fn each_links_pieces_hold_its_collision_meshes_within_the_fits_budget() {
    let base = fitted(BASE_LINK);
    let links = ARM
        .links()
        .iter()
        .map(|link| {
            let hulls: Vec<&Hull> = link.pieces().iter().map(Piece::hull).collect();
            (link.name(), hulls)
        })
        .chain([
            (BASE_LINK, base.iter().collect()),
            (JAW_LINK, JAW.iter().collect()),
        ]);
    for (link, pieces) in links {
        let elements = element_vertices(link);
        assert_eq!(pieces.len(), elements.len(), "{link}");
        for (piece, vertices) in pieces.into_iter().zip(&elements) {
            for vertex in vertices {
                let held = piece
                    .planes()
                    .iter()
                    .all(|(normal, offset)| normal.dot(&vertex.coords) <= offset + TOLERANCE);
                assert!(held, "{link}'s piece leaves out {vertex}");
            }
            let mesh = Hull::new(&exact_hull(vertices).expect("a mesh spans a solid"), 0.0)
                .expect("the mesh's hull is a hull");
            for corner in piece.vertices() {
                let beyond = gjk::distance(&Vertex(*corner), &mesh).distance;
                assert!(
                    beyond <= FIT_BUDGET_M + TOLERANCE,
                    "{link}'s piece stands {beyond} m beyond its mesh"
                );
            }
        }
    }
}

// At any configuration of the arm and any opening of the jaw, each link the arm
// carries stands where the URDF's joints put it, and the jaw, placed on the gripper
// link by its joint at the opening's position, stands where the URDF puts it there.
#[hegel::test]
fn each_link_stands_where_the_urdfs_joints_put_it(tc: TestCase) {
    let q = tc.draw(configs(ARM.chain().limits()));
    let opening = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
    let jaw_position = jaw_position_rad(opening).expect("a drawn opening is finite");
    let jaw_joint = parent_joint(JAW_LINK).expect("a joint carries the jaw");
    let positions: Vec<(&str, f64)> = arm_joints()
        .into_iter()
        .zip(q)
        .chain([(jaw_joint.name.as_str(), jaw_position)])
        .collect();

    let placed = ARM.place(&q);
    for (link, pose) in placed.links() {
        assert_same_pose(link.name(), pose, &urdf_pose(link.name(), &positions));
    }
    let (_, carrier) = placed
        .links()
        .find(|(link, _)| link.name() == jaw_joint.parent.link)
        .expect("the arm carries the jaw's link");
    let joint = COLLISIONS
        .parent_joint(JAW_LINK)
        .expect("the jaw has a joint");
    let jaw = carrier
        * joint
            .offset(jaw_position)
            .expect("the jaw's joint places it");
    assert_same_pose(JAW_LINK, &jaw, &urdf_pose(JAW_LINK, &positions));
}

// The jaw shuts at its joint's lower limit: shut, its pieces stand nearer the tool
// point than fully open.
#[test]
fn the_shut_jaw_stands_nearer_the_tool_point_than_the_open_one() {
    let jaw = COLLISIONS
        .parent_joint(JAW_LINK)
        .expect("the jaw has a joint");
    let tool = COLLISIONS
        .parent_joint(TCP_LINK)
        .expect("the tool point has a joint");
    assert_eq!(jaw.parent_link, tool.parent_link);
    let tool_point = Point3::from(tool.origin.translation.vector);
    let distance_at = |opening: f64| {
        let position = jaw_position_rad(opening).expect("a finite opening has a position");
        let pose = jaw.offset(position).expect("the jaw's joint places it");
        JAW.iter()
            .map(|piece| gjk::distance(&Vertex(tool_point), &Placed::new(piece, pose)).distance)
            .fold(f64::INFINITY, f64::min)
    };
    assert!(distance_at(0.0) < distance_at(1.0));
}
