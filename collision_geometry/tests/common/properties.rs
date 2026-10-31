//! The Hegel generators this crate's property tests draw beyond
//! `kinematics_properties`: drawn robots dressed in collision elements, mounted on a
//! continuous joint the chain does not drive, with links the chain's joints do not move,
//! and paddings.

use std::path::{Path, PathBuf};
use std::sync::LazyLock;

use collision_geometry::chain::Padding;
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{link_name, placements, unit_vectors};
use urdf_rs::{
    Axis, Collision, Geometry, Joint, JointLimit, JointType, Link, LinkName, Robot, Vec3,
};

/// A mesh a drawn mesh collision scales: a cube of unit side about its origin.
const CUBE: &str = "cube.stl";

/// A mesh a drawn mesh collision scales: a wedge from a square of side 0.1 at x = 0 to a
/// point at x = 1, whose vertices' mean lies off the center of its box.
pub const WEDGE: &str = "wedge.stl";

/// The root of a dressed robot, below which a continuous joint turns `link_0`.
const MOUNT: &str = "mount";

/// A link of a dressed robot on a continuous joint of its chain that the chain does not
/// drive, between a drawn link and the next.
pub const SPUN: &str = "spun";

/// A link fixed to the root of a dressed robot.
const STAND: &str = "stand";

/// A link on a prismatic joint off a drawn link of a dressed robot.
const FINGER: &str = "finger";

/// The directory holding [`CUBE`] and [`WEDGE`], written once per test binary.
pub static MESHES_DIR: LazyLock<PathBuf> = LazyLock::new(|| {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("chain_meshes");
    std::fs::create_dir_all(&dir).expect("create the mesh directory");
    std::fs::write(dir.join(CUBE), cube_stl()).expect("write the cube mesh");
    std::fs::write(dir.join(WEDGE), wedge_stl()).expect("write the wedge mesh");
    dir
});

/// A binary STL of the unit cube about the origin, two triangles a face.
fn cube_stl() -> Vec<u8> {
    const FACES: [[usize; 4]; 6] = [
        [0, 1, 3, 2],
        [4, 6, 7, 5],
        [0, 4, 5, 1],
        [2, 3, 7, 6],
        [0, 2, 6, 4],
        [1, 5, 7, 3],
    ];
    let corner = |i: usize| [i & 1, (i >> 1) & 1, (i >> 2) & 1].map(|bit| bit as f32 - 0.5);
    let triangles: Vec<[[f32; 3]; 3]> = FACES
        .iter()
        .flat_map(|&[a, b, c, d]| [[a, b, c], [a, c, d]])
        .map(|triangle| triangle.map(corner))
        .collect();
    stl_of(&triangles)
}

/// A binary STL of the [`WEDGE`]: its square base, two triangles, and four triangles to
/// its point.
fn wedge_stl() -> Vec<u8> {
    let base = [
        [0.0, -0.05, -0.05],
        [0.0, 0.05, -0.05],
        [0.0, 0.05, 0.05],
        [0.0, -0.05, 0.05],
    ];
    let point = [1.0, 0.0, 0.0];
    let triangles: Vec<[[f32; 3]; 3]> = [[base[0], base[2], base[1]], [base[0], base[3], base[2]]]
        .into_iter()
        .chain((0..4).map(|i| [base[i], base[(i + 1) % 4], point]))
        .collect();
    stl_of(&triangles)
}

/// A binary STL of `triangles`, each three corners.
fn stl_of(triangles: &[[[f32; 3]; 3]]) -> Vec<u8> {
    let header = [0_u8; 80]
        .into_iter()
        .chain((triangles.len() as u32).to_le_bytes());
    let facets = triangles.iter().flat_map(|triangle| {
        let normal = [0_u8; 12];
        let corners = triangle.iter().flatten().flat_map(|c| c.to_le_bytes());
        let attribute = [0_u8; 2];
        normal.into_iter().chain(corners).chain(attribute)
    });
    header.chain(facets).collect()
}

/// A length from 1 cm to 20 cm, the size of a drawn collision element.
fn sizes() -> gs::FloatGenerator<f64> {
    gs::floats().min_value(0.01).max_value(0.2)
}

/// The geometry of a drawn collision element: the cube or the wedge mesh scaled along
/// each axis, a box, a cylinder, or a sphere.
#[hegel::composite]
fn geometries(tc: &TestCase) -> Geometry {
    match tc.draw(gs::integers::<u8>().min_value(0).max_value(3)) {
        0 => Geometry::Mesh {
            filename: format!(
                "package://drawn/meshes/{}",
                if tc.draw(gs::booleans()) { CUBE } else { WEDGE }
            ),
            scale: Some(Vec3(tc.draw(gs::arrays(sizes())))),
        },
        1 => Geometry::Box {
            size: Vec3(tc.draw(gs::arrays(sizes()))),
        },
        2 => Geometry::Cylinder {
            radius: tc.draw(sizes()) / 2.0,
            length: tc.draw(sizes()),
        },
        _ => Geometry::Sphere {
            radius: tc.draw(sizes()) / 2.0,
        },
    }
}

/// One or two collision elements, each on any frame [`placements`] draws.
#[hegel::composite]
fn collisions(tc: &TestCase) -> Vec<Collision> {
    let count = tc.draw(gs::integers::<usize>().min_value(1).max_value(2));
    (0..count)
        .map(|_| Collision {
            name: None,
            origin: tc.draw(placements()),
            geometry: tc.draw(geometries()),
        })
        .collect()
}

/// `robot`, drawn with `joints` actuated joints, mounted on [`MOUNT`] by a continuous
/// joint the chain does not drive, with [`SPUN`] on a second such joint between a drawn
/// link and the next, with [`collisions`] on every link and a turned [`WEDGE`] on a
/// drawn link, and two more links with collisions that the chain's joints do not move:
/// [`STAND`], fixed to `link_0`, and [`FINGER`], on a prismatic joint off a drawn moving
/// link.
#[hegel::composite]
pub fn dressed(tc: &TestCase, robot: Robot, joints: usize) -> Robot {
    let finger_parent = tc.draw(gs::integers::<usize>().min_value(1).max_value(joints));
    let spun_parent = link_name(tc.draw(gs::integers::<usize>().min_value(1).max_value(joints)));
    let wedge_link = link_name(tc.draw(gs::integers::<usize>().min_value(1).max_value(joints)));
    let branch = |name: &str, parent: String, joint_type: JointType, limit: JointLimit| Joint {
        name: format!("{name}_joint"),
        joint_type,
        origin: tc.draw(placements()),
        parent: LinkName { link: parent },
        child: LinkName {
            link: name.to_owned(),
        },
        axis: Axis::default(),
        limit,
        calibration: None,
        dynamics: None,
        mimic: None,
        safety_controller: None,
    };
    let continuous = |name: &str, parent: &str, child: &str| Joint {
        name: format!("{name}_joint"),
        joint_type: JointType::Continuous,
        origin: tc.draw(placements()),
        parent: LinkName {
            link: parent.to_owned(),
        },
        child: LinkName {
            link: child.to_owned(),
        },
        axis: Axis {
            xyz: Vec3(tc.draw(unit_vectors()).into()),
        },
        limit: JointLimit::default(),
        calibration: None,
        dynamics: None,
        mimic: None,
        safety_controller: None,
    };
    let mount = continuous(MOUNT, MOUNT, &link_name(0));
    let spin = continuous(SPUN, &spun_parent, SPUN);
    let stand = branch(STAND, link_name(0), JointType::Fixed, JointLimit::default());
    let finger = branch(
        FINGER,
        link_name(finger_parent),
        JointType::Prismatic,
        JointLimit {
            lower: 0.0,
            upper: 0.05,
            effort: 1.0,
            velocity: 1.0,
            ..JointLimit::default()
        },
    );
    let added = [MOUNT, SPUN, STAND, FINGER].map(|name| Link {
        name: name.to_owned(),
        inertial: Default::default(),
        visual: Vec::new(),
        collision: Vec::new(),
    });
    let wedge = Collision {
        name: None,
        origin: tc.draw(placements()),
        geometry: Geometry::Mesh {
            filename: format!("package://drawn/meshes/{WEDGE}"),
            scale: Some(Vec3(tc.draw(gs::arrays(sizes())))),
        },
    };
    let links = robot
        .links
        .into_iter()
        .chain(added)
        .map(|link| {
            let drawn = tc.draw(collisions());
            let collision = if link.name == wedge_link {
                drawn.into_iter().chain([wedge.clone()]).collect()
            } else {
                drawn
            };
            Link { collision, ..link }
        })
        .collect();
    // The joint below the spun link's parent hangs from the spun link.
    let joints = robot
        .joints
        .into_iter()
        .map(|joint| {
            if joint.parent.link == spun_parent {
                Joint {
                    parent: LinkName {
                        link: SPUN.to_owned(),
                    },
                    ..joint
                }
            } else {
                joint
            }
        })
        .chain([mount, spin, stand, finger])
        .collect();
    Robot {
        links,
        joints,
        ..robot
    }
}

/// A padding that grows each piece: a scale from 1 to 2 and an offset up to 0.2 m, the
/// size of a drawn collision element.
#[hegel::composite]
pub fn paddings(tc: &TestCase) -> Padding {
    Padding::new(
        tc.draw(gs::floats().min_value(1.0).max_value(2.0)),
        tc.draw(gs::floats().min_value(0.0).max_value(0.2)),
    )
    .expect("a drawn padding is valid")
}
