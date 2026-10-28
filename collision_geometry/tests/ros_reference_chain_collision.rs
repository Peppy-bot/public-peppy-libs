//! The collision geometry of a chain against MoveIt: `ChainGeometry` places the links
//! that a serial chain read from a URDF carries at a configuration of its joints, and its
//! `Mask` grows each collision element as MoveIt's robot filter grows its body and tells
//! whether a point lies inside a grown link or the grown chain. The evidence: at every
//! configuration of every chain collision scenario of the ROS reference harness, OpenArm
//! v2 dressed in its meshes and an arm dressed in boxes, cylinders and spheres and a
//! turned wedge, the chain
//! carries the links that MoveIt's chain carries, each at MoveIt's transform within
//! [`FRAME_TOLERANCE`], and for each padding the filter answers as MoveIt's robot filter
//! on every probe point outside the band of each body.
//!
//! MoveIt grows the convex hull of each mesh; the filter grows the piece fitted about it,
//! which lies at most its fit error outside that hull. A padding moves each face of a
//! hull out by at most `scale - 1` times its distance from the center the hull grows
//! about, MoveIt's for the mesh's hull, plus the offset. So a point deeper inside a piece
//! than its fit error lies inside the mesh's hull, so inside its grown link and both
//! grown chains; and a point farther beyond a face of every piece, and beyond a face of
//! every mesh's hull, than the padding moves that face lies outside both grown chains.
//! Between the two lies the band of a mesh, about as wide as its fit error plus the
//! padding. Inside the bands the two filters grow different hulls about different
//! centers, and for each padding the upper end of the Wilson score interval, at
//! [`BAND_CONFIDENCE`], of the share of the probes on which they agree reaches
//! [`BAND_AGREEMENT`]. At the committed counts that fails a filter that grows a mesh with
//! no offset; one that grows a mesh with no scale passes it and fails the properties of
//! `chain_geometry.rs`, and one that grows every element with no scale fails on the
//! probes about the grown primitives. A primitive grows alike in both filters, about its
//! element's origin, which MoveIt's center equals, so its band is [`TOLERANCE`] about the
//! surface of the grown primitive. Each padding holds probes deep inside a body and
//! beyond every band.

use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{ChainGeometry, Padding, Piece, PlacedChain};
use collision_geometry::gjk::{self, Hull, Vertex};
use collision_geometry::hull::{ConvexHull, exact_hull};
use collision_geometry::nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, Vector3};
use collision_geometry::urdf_collision::CollisionGeometry;
use kinematics_properties::TOLERANCE;
use ros_reference::LinkPose;
use ros_reference::chain_collision::{Cases, ChainCase};

#[path = "common/growth.rs"]
mod growth;
use growth::grown_primitive_excess;

/// The largest miss of a link frame against MoveIt's, in meters or as a rotation matrix
/// entry: rounding of the same chain of transforms, composed in another order.
const FRAME_TOLERANCE: f64 = 1e-12;

/// The joint count of every chain that the test poses.
const CHAIN_JOINTS: usize = 7;

/// The share that the upper end of the Wilson score interval, at [`BAND_CONFIDENCE`], of
/// the share of the probes inside the bands of a padding on which the two filters agree
/// reaches.
const BAND_AGREEMENT: f64 = 0.9;

/// The standard normal quantile of the two-sided 99% Wilson score interval.
const BAND_CONFIDENCE: f64 = 2.576;

fn moveit_pose(pose: &LinkPose) -> Isometry3<f64> {
    let rows = pose.rotation.map(|entry| entry.0);
    Isometry3::from_parts(
        Translation3::from(Vector3::from(pose.translation.map(|entry| entry.0))),
        Rotation3::from_matrix_unchecked(Matrix3::from_row_slice(&rows)).into(),
    )
}

fn pose_of<'a>(placed: &'a PlacedChain, link: &str) -> &'a Isometry3<f64> {
    placed
        .links()
        .find(|(geometry, _)| geometry.name() == link)
        .map(|(_, pose)| pose)
        .unwrap_or_else(|| panic!("the chain carries {link}"))
}

/// Whether `point` lies farther beyond a face of the hull of `planes` than `padding`
/// moves that face out from `center`, by more than [`TOLERANCE`], the rounding of two
/// hulls of one mesh.
fn beyond_the_padding(
    planes: &[(Vector3<f64>, f64)],
    center: &Point3<f64>,
    padding: Padding,
    point: &Point3<f64>,
) -> bool {
    planes.iter().any(|(normal, offset)| {
        let moved =
            (padding.scale() - 1.0) * (offset - normal.dot(&center.coords)) + padding.offset();
        normal.dot(&point.coords) - offset > moved + TOLERANCE
    })
}

/// Where a probe point lies against a body.
#[derive(PartialEq)]
enum Side {
    /// Inside the body in both filters.
    Deep,
    /// Outside the body in both filters.
    Beyond,
    /// In the band, where the two filters may differ.
    Band,
}

/// How the test bands a body of a carried link.
enum Body<'a> {
    /// A mesh: its piece, the convex hull of its surface, the center MoveIt grows that
    /// hull about, and how far the piece's farthest vertex lies outside that hull.
    Mesh {
        piece: &'a Hull,
        hull: ConvexHull,
        moveit_center: Point3<f64>,
        fit_error: f64,
    },
    /// A primitive, which both filters grow alike.
    Primitive(&'a Piece),
}

impl Body<'_> {
    /// Where `point`, in the frame of the body's link, lies against the body grown by
    /// `padding`.
    fn side(&self, padding: Padding, point: &Point3<f64>) -> Side {
        match self {
            Body::Mesh {
                piece,
                hull,
                moveit_center,
                fit_error,
            } => {
                if gjk::distance(&Vertex(*point), *piece).distance < -(fit_error + TOLERANCE) {
                    Side::Deep
                } else if beyond_the_padding(piece.planes(), &piece.center(), padding, point)
                    && beyond_the_padding(&hull.planes, moveit_center, padding, point)
                {
                    Side::Beyond
                } else {
                    Side::Band
                }
            }
            Body::Primitive(piece) => {
                let excess = grown_primitive_excess(piece.element(), padding, point)
                    .expect("a primitive grows as itself");
                if excess < -TOLERANCE {
                    Side::Deep
                } else if excess > TOLERANCE {
                    Side::Beyond
                } else {
                    Side::Band
                }
            }
        }
    }
}

/// The upper end of the Wilson score interval, at [`BAND_CONFIDENCE`], of the share that
/// `agreed` of `trials` probes make.
fn wilson_upper(agreed: usize, trials: usize) -> f64 {
    let (n, z) = (trials as f64, BAND_CONFIDENCE);
    let share = agreed as f64 / n;
    let spread = z * (share * (1.0 - share) / n + z * z / (4.0 * n * n)).sqrt();
    (share + z * z / (2.0 * n) + spread) / (1.0 + z * z / n)
}

/// Every chain collision scenario of the harness.
fn scenarios() -> Vec<Cases> {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a chain collision scenario"
    );
    scenarios
}

#[test]
fn each_chain_carries_moveits_links_at_moveits_frames() {
    for cases in scenarios() {
        for chain in &cases.chains {
            let at = format!("{} {}", cases.name, chain.name);
            let geometry = geometry_of(&cases, chain);
            let mut carried: Vec<&str> = geometry.links().iter().map(|link| link.name()).collect();
            carried.sort_unstable();
            let mut moved: Vec<&str> = chain.links.iter().map(String::as_str).collect();
            moved.sort_unstable();
            assert_eq!(carried, moved, "{at}: the carried links");
            for (index, configuration) in chain.configurations.iter().enumerate() {
                let placed = geometry.place(&positions(&configuration.positions));
                for pose in &configuration.links {
                    let (ours, theirs) = (pose_of(&placed, &pose.link), moveit_pose(pose));
                    let translation = (ours.translation.vector - theirs.translation.vector).amax();
                    let rotation = (ours.rotation.to_rotation_matrix().matrix()
                        - theirs.rotation.to_rotation_matrix().matrix())
                    .amax();
                    assert!(
                        translation <= FRAME_TOLERANCE && rotation <= FRAME_TOLERANCE,
                        "{at} configuration {index} {}: misses MoveIt's frame by \
                         {translation:e} m and {rotation:e} in rotation",
                        pose.link
                    );
                }
            }
        }
    }
}

#[test]
fn the_robot_filter_answers_as_moveits_outside_the_band() {
    for cases in scenarios() {
        let mut counts = vec![[0_usize; 4]; cases.paddings.len()];
        for chain in &cases.chains {
            let at = format!("{} {}", cases.name, chain.name);
            let geometry = geometry_of(&cases, chain);
            let bodies: Vec<(usize, Body)> = geometry
                .links()
                .iter()
                .enumerate()
                .flat_map(|(link, geometry)| {
                    let centers: Vec<Point3<f64>> = chain
                        .bodies
                        .iter()
                        .filter(|body| body.link == geometry.name())
                        .map(|body| Point3::from(body.center.map(|entry| entry.0)))
                        .collect();
                    assert_eq!(
                        centers.len(),
                        geometry.pieces().len(),
                        "{at} {}: MoveIt holds one body per collision element",
                        geometry.name()
                    );
                    geometry
                        .pieces()
                        .iter()
                        .zip(centers)
                        .map(|(piece, moveit_center)| {
                            (link, body_of(&cases, piece, moveit_center, &at))
                        })
                        .collect::<Vec<_>>()
                })
                .collect();
            for (padding, grown) in cases.paddings.iter().enumerate() {
                let padding_of = Padding::new(grown.scale, grown.offset).expect("a valid padding");
                let mask = geometry.mask(padding_of);
                let [deep, far, band, agreed] = &mut counts[padding];
                for (index, configuration) in chain.configurations.iter().enumerate() {
                    let q = positions(&configuration.positions);
                    let (placed, grown_chain) = (geometry.place(&q), mask.at(&q));
                    let poses: Vec<&Isometry3<f64>> =
                        placed.links().map(|(_, pose)| pose).collect();
                    for (probe, &moveit_inside) in configuration
                        .probes
                        .iter()
                        .zip(&configuration.inside[padding])
                    {
                        let point = pose_of(&placed, &probe.link) * Point3::from(probe.position);
                        let sides: Vec<Side> = bodies
                            .iter()
                            .map(|(link, body)| {
                                body.side(padding_of, &(poses[*link].inverse() * point))
                            })
                            .collect();
                        let ours = grown_chain.contains(&point);
                        let context =
                            format!("{at} configuration {index}, padding {grown:?}, {point}");
                        let deep_in = bodies
                            .iter()
                            .zip(&sides)
                            .find(|(_, side)| **side == Side::Deep);
                        if let Some(((link, _), _)) = deep_in {
                            *deep += 1;
                            let link = &grown_chain.links()[*link];
                            assert!(
                                link.contains(&point) && ours && moveit_inside,
                                "{context} deep in {}: its link {}, ours {ours}, MoveIt's \
                                 {moveit_inside}",
                                link.name(),
                                link.contains(&point)
                            );
                        } else if sides.iter().all(|side| *side == Side::Beyond) {
                            *far += 1;
                            assert!(
                                !ours && !moveit_inside,
                                "{context} beyond every band: ours {ours}, MoveIt's \
                                 {moveit_inside}"
                            );
                        } else {
                            *band += 1;
                            *agreed += usize::from(ours == moveit_inside);
                        }
                    }
                }
            }
        }
        for (grown, [deep, far, band, agreed]) in cases.paddings.iter().zip(&counts) {
            let at = format!("{} padding {grown:?}", cases.name);
            assert!(
                *deep > 0 && *far > 0,
                "{at} holds probes deep and beyond the bands: {deep} deep, {far} beyond"
            );
            assert!(
                *band == 0 || wilson_upper(*agreed, *band) >= BAND_AGREEMENT,
                "{at}: the filters agree on {agreed} of the {band} probes inside the bands"
            );
        }
    }
}

/// How the test bands the body of `piece`, whose center, as MoveIt grows it, is
/// `moveit_center`. A primitive grows about its element's origin, which MoveIt's center
/// equals within [`FRAME_TOLERANCE`].
fn body_of<'a>(cases: &Cases, piece: &'a Piece, moveit_center: Point3<f64>, at: &str) -> Body<'a> {
    let element = piece.element();
    if !matches!(element.geometry, CollisionGeometry::Mesh { .. }) {
        let origin = Point3::from(element.origin.translation.vector);
        assert!(
            (moveit_center - origin).amax() <= FRAME_TOLERANCE,
            "{at}: MoveIt grows a primitive about {moveit_center}, and its origin is {origin}"
        );
        return Body::Primitive(piece);
    }
    let surface = element
        .surface(mesh_directory(cases))
        .expect("the mesh reads");
    let hull = exact_hull(&surface).expect("a mesh hull");
    let reference = Hull::new(&hull, 0.0).expect("a hull");
    let fit_error = piece
        .hull()
        .vertices()
        .iter()
        .map(|v| gjk::distance(&Vertex(*v), &reference).distance)
        .fold(0.0, f64::max);
    Body::Mesh {
        piece: piece.hull(),
        hull,
        moveit_center,
        fit_error,
    }
}

/// The scenario's mesh directory, or no directory for a scenario whose URDF names no
/// collision mesh.
fn mesh_directory(cases: &Cases) -> &str {
    cases.meshes.as_deref().map_or("", |meshes| {
        meshes.to_str().expect("the mesh directory is UTF-8")
    })
}

/// The geometry of `chain`, which refuses a chain of another joint count than
/// [`CHAIN_JOINTS`].
fn geometry_of(cases: &Cases, chain: &ChainCase) -> ChainGeometry<CHAIN_JOINTS> {
    assert_eq!(
        chain.joints.len(),
        CHAIN_JOINTS,
        "{} {}: the parity test poses chains of {CHAIN_JOINTS} joints",
        cases.name,
        chain.name
    );
    let joints: Vec<&str> = chain.joints.iter().map(String::as_str).collect();
    ChainGeometry::<CHAIN_JOINTS>::from_urdf(
        &cases.urdf,
        mesh_directory(cases),
        &ChainSpec {
            base_link: None,
            tip_link: &chain.tip_link,
            joints: JointSelection::Named(&joints),
        },
    )
    .unwrap_or_else(|e| panic!("{} {}: {e}", cases.name, chain.name))
}

fn positions(positions: &[f64]) -> [f64; CHAIN_JOINTS] {
    positions
        .try_into()
        .expect("the harness pairs one position per joint")
}
