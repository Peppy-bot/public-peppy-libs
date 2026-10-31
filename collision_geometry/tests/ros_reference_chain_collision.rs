//! The collision geometry of a chain against MoveIt: `ChainGeometry` places the links
//! that a serial chain read from a URDF carries at a configuration of its joints, and its
//! `Mask` grows each collision element by a padding and tells whether a point lies inside
//! a grown link or the grown chain. The evidence: at every configuration of every chain
//! collision scenario of the ROS reference harness, OpenArm v2 dressed in its meshes and
//! an arm dressed in boxes, cylinders and spheres and a turned wedge, the chain carries
//! the links that MoveIt's chain carries, each at MoveIt's transform within
//! [`FRAME_TOLERANCE`], and for each padding the filter holds to its rule on every probe,
//! and answers as MoveIt's robot filter outside the band of each body.
//!
//! The rule: a grown element holds every point within the offset of the element scaled
//! about its center, and no point farther. A mesh scales from its piece, which holds the
//! convex hull of the mesh and lies within its fit error of it, so a probe within the
//! offset of that hull, scaled about the piece's center, lies inside its grown link, and
//! a probe farther than the offset plus the scaled fit error from the hull of every
//! mesh, and farther than the offset from every scaled primitive, lies outside the grown
//! chain, each by [`TOLERANCE`], the rounding of two hulls of one mesh.
//!
//! MoveIt's robot filter grows a body by sharp planes, as the crate's README states
//! (The robot filter). The two filters agree on a probe inside a mesh's hull, or within
//! the offset less [`TOLERANCE`] of a scaled primitive, which both hold, and on a probe
//! that the rule puts outside and that lies outside MoveIt's grown box of every mesh
//! and beyond the sharp growth of every primitive, which neither holds. Between the two
//! lies the band of a body, where MoveIt's edges reach past the offset and its planes
//! can stop short of it. Inside the bands, MoveIt deletes probes that lie beyond the
//! offset of every scaled body, and keeps probes within the offset of a scaled body:
//! [`BAND`] holds the counts on the committed probes, and the test holds the filter to
//! them. A filter that holds every point of a mesh within its faces moved out by the
//! offset, or grows a box with sharp edges, puts a probe farther than the offset from
//! every body inside the grown chain, and fails the rule. Each padding holds probes
//! deep inside a body, within the offset of one, and beyond every band.

use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{ChainGeometry, Padding, Piece, PlacedChain};
use collision_geometry::gjk::{self, Hull, Vertex};
use collision_geometry::hull::{ConvexHull, exact_hull};
use collision_geometry::nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, Vector3};
use collision_geometry::urdf_collision::{Collision, CollisionGeometry};
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

/// The probes inside the bands of each scenario, for each of its paddings in order, by
/// how the two filters answer on them.
const BAND: [(&str, [Band; 3]); 2] = [
    (
        "chain_collision_openarm_v2",
        [
            Band::new(13, 0, 0, 0),
            Band::new(88, 0, 31, 0),
            Band::new(48, 0, 6, 2),
        ],
    ),
    (
        "chain_collision_primitive_arm",
        [
            Band::new(2, 0, 0, 0),
            Band::new(0, 2, 0, 0),
            Band::new(0, 0, 0, 0),
        ],
    ),
];

/// How the two filters answer on the probes inside the bands of a padding.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
struct Band {
    /// The probes on which the two agree.
    agreed: usize,
    /// The probes that MoveIt deletes, and that lie beyond the offset of every scaled
    /// body.
    moveit_deletes_beyond: usize,
    /// The probes that MoveIt keeps, and that lie within the offset of the hull of a
    /// scaled mesh, or of a scaled primitive.
    moveit_keeps_within: usize,
    /// The probes that MoveIt keeps, that lie within the offset of the piece of a scaled
    /// mesh, and beyond the offset of every scaled hull and primitive: within the fit
    /// error of a scaled piece past the offset of its hull.
    moveit_keeps_within_a_fit_error: usize,
}

impl Band {
    const fn new(
        agreed: usize,
        moveit_deletes_beyond: usize,
        moveit_keeps_within: usize,
        moveit_keeps_within_a_fit_error: usize,
    ) -> Self {
        Band {
            agreed,
            moveit_deletes_beyond,
            moveit_keeps_within,
            moveit_keeps_within_a_fit_error,
        }
    }
}

fn moveit_pose(pose: &LinkPose) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(Vector3::from(pose.translation)),
        Rotation3::from_matrix_unchecked(Matrix3::from_row_slice(&pose.rotation)).into(),
    )
}

fn pose_of<'a>(placed: &'a PlacedChain, link: &str) -> &'a Isometry3<f64> {
    placed
        .links()
        .find(|(geometry, _)| geometry.name() == link)
        .map(|(_, pose)| pose)
        .unwrap_or_else(|| panic!("the chain carries {link}"))
}

/// Where a probe point lies against a body grown by a padding.
struct Side {
    /// Inside the body in both filters.
    deep: bool,
    /// Outside the body in both filters.
    beyond: bool,
    /// Within the offset of the scaled body, less [`TOLERANCE`]: inside it by the rule.
    within: bool,
    /// Farther from the scaled body than the offset, by [`TOLERANCE`], and its piece's
    /// scaled fit error for a mesh: outside it by the rule.
    past: bool,
    /// Within the offset of the hull of the scaled mesh, or of the scaled primitive.
    within_the_offset_of_its_shape: bool,
}

/// How the test places a probe against a body of a carried link.
enum Body<'a> {
    Mesh(Box<MeshBody>),
    /// A primitive, which both filters grow about its element's origin.
    Primitive(&'a Piece),
}

/// A mesh: the convex hull of its surface, that hull scaled about the center of its
/// piece's vertices for each padding, its box along the axes of its element's frame, and
/// how far the piece's farthest vertex lies outside the hull.
struct MeshBody {
    hull: Hull,
    scaled_hulls: Vec<Hull>,
    link_to_element: Isometry3<f64>,
    box_center: Point3<f64>,
    box_half_extents: Vector3<f64>,
    fit_error: f64,
}

impl Body<'_> {
    /// Where `point`, in the frame of the body's link, lies against the body grown by
    /// the padding `padding`, the `index`th of the scenario.
    fn side(&self, padding: Padding, index: usize, point: &Point3<f64>) -> Side {
        let offset = padding.offset();
        match self {
            Body::Mesh(mesh) => {
                let MeshBody {
                    hull,
                    scaled_hulls,
                    link_to_element,
                    box_center,
                    box_half_extents,
                    fit_error,
                } = mesh.as_ref();
                let from_scaled_hull =
                    gjk::distance(&Vertex(*point), &scaled_hulls[index]).distance;
                let past = from_scaled_hull > offset + padding.scale() * fit_error + TOLERANCE;
                let beyond_the_box = ((link_to_element * point - box_center).abs()
                    - box_half_extents * padding.scale())
                .add_scalar(-offset)
                .max()
                    > TOLERANCE;
                Side {
                    deep: gjk::distance(&Vertex(*point), hull).distance < -TOLERANCE,
                    beyond: past && beyond_the_box,
                    within: from_scaled_hull <= offset - TOLERANCE,
                    past,
                    within_the_offset_of_its_shape: from_scaled_hull <= offset,
                }
            }
            Body::Primitive(piece) => {
                let from_scaled =
                    signed_distance_from_scaled_primitive(piece.element(), padding.scale(), point);
                let sharp_excess = grown_primitive_excess(piece.element(), padding, point)
                    .expect("a primitive grows as itself");
                Side {
                    deep: from_scaled <= offset - TOLERANCE,
                    beyond: sharp_excess > TOLERANCE,
                    within: from_scaled <= offset - TOLERANCE,
                    past: from_scaled > offset + TOLERANCE,
                    within_the_offset_of_its_shape: from_scaled <= offset,
                }
            }
        }
    }
}

/// The signed distance from `point`, in the frame of its link, to the primitive `element`
/// scaled by `scale` about its origin: negative inside it, by the depth.
fn signed_distance_from_scaled_primitive(
    element: &Collision,
    scale: f64,
    point: &Point3<f64>,
) -> f64 {
    let local = element.origin.inverse() * point;
    let signed = |excess: &[f64]| {
        let outside = excess
            .iter()
            .map(|e| e.max(0.0).powi(2))
            .sum::<f64>()
            .sqrt();
        let inside = excess
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max)
            .min(0.0);
        outside + inside
    };
    match element.geometry {
        CollisionGeometry::Mesh { .. } => panic!("a mesh is no primitive"),
        CollisionGeometry::Box { size } => {
            signed((local.coords.abs() - size * (scale / 2.0)).as_slice())
        }
        CollisionGeometry::Cylinder { radius, length } => signed(&[
            local.coords.xy().norm() - radius * scale,
            local.z.abs() - length * scale / 2.0,
        ]),
        CollisionGeometry::Sphere { radius } => local.coords.norm() - radius * scale,
    }
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
fn the_robot_filter_holds_to_its_rule_and_answers_as_moveits_outside_the_band() {
    for cases in scenarios() {
        let mut counts = vec![([0_usize; 3], Band::default()); cases.paddings.len()];
        for chain in &cases.chains {
            let at = format!("{} {}", cases.name, chain.name);
            let geometry = geometry_of(&cases, chain);
            let paddings: Vec<Padding> = cases
                .paddings
                .iter()
                .map(|grown| Padding::new(grown.scale, grown.offset).expect("a valid padding"))
                .collect();
            let bodies: Vec<(usize, Body)> = geometry
                .links()
                .iter()
                .enumerate()
                .flat_map(|(link, geometry)| {
                    let centers: Vec<Point3<f64>> = chain
                        .bodies
                        .iter()
                        .filter(|body| body.link == geometry.name())
                        .map(|body| Point3::from(body.center))
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
                            (link, body_of(&cases, piece, moveit_center, &paddings, &at))
                        })
                        .collect::<Vec<_>>()
                })
                .collect();
            for (index, (grown, padding)) in cases.paddings.iter().zip(&paddings).enumerate() {
                let mask = geometry.mask(*padding);
                let ([deep, within, far], band) = &mut counts[index];
                for (configuration_index, configuration) in chain.configurations.iter().enumerate()
                {
                    let q = positions(&configuration.positions);
                    let (placed, grown_chain) = (geometry.place(&q), mask.at(&q));
                    let poses: Vec<&Isometry3<f64>> =
                        placed.links().map(|(_, pose)| pose).collect();
                    for (probe, &moveit_inside) in configuration
                        .probes
                        .iter()
                        .zip(&configuration.inside[index])
                    {
                        let point = pose_of(&placed, &probe.link) * Point3::from(probe.position);
                        let sides: Vec<(usize, Side)> = bodies
                            .iter()
                            .map(|(link, body)| {
                                (
                                    *link,
                                    body.side(*padding, index, &(poses[*link].inverse() * point)),
                                )
                            })
                            .collect();
                        let ours = grown_chain.contains(&point);
                        let context = format!(
                            "{at} configuration {configuration_index}, padding {grown:?}, {point}"
                        );
                        *within += usize::from(sides.iter().any(|(_, side)| side.within));
                        for (link, side) in &sides {
                            let link = &grown_chain.links()[*link];
                            assert!(
                                !side.within || link.contains(&point),
                                "{context} lies within the offset of a scaled body of {}, \
                                 and outside its grown link",
                                link.name()
                            );
                        }
                        assert!(
                            !sides.iter().all(|(_, side)| side.past) || !ours,
                            "{context} lies past the offset of every scaled body, and inside \
                             the grown chain"
                        );
                        if sides.iter().any(|(_, side)| side.deep) {
                            *deep += 1;
                            assert!(
                                ours && moveit_inside,
                                "{context} deep in a body: ours {ours}, MoveIt's {moveit_inside}"
                            );
                        } else if sides.iter().all(|(_, side)| side.beyond) {
                            *far += 1;
                            assert!(
                                !ours && !moveit_inside,
                                "{context} beyond every band: ours {ours}, MoveIt's \
                                 {moveit_inside}"
                            );
                        } else if ours == moveit_inside {
                            band.agreed += 1;
                        } else if moveit_inside {
                            band.moveit_deletes_beyond += 1;
                        } else if sides
                            .iter()
                            .any(|(_, side)| side.within_the_offset_of_its_shape)
                        {
                            band.moveit_keeps_within += 1;
                        } else {
                            band.moveit_keeps_within_a_fit_error += 1;
                        }
                    }
                }
            }
        }
        for (grown, ([deep, within, far], _)) in cases.paddings.iter().zip(&counts) {
            println!(
                "{} padding {grown:?}: {deep} probes deep, {within} within the offset of a \
                 scaled body, {far} beyond the bands",
                cases.name
            );
            assert!(
                *deep > 0 && *within > 0 && *far > 0,
                "{} padding {grown:?} holds probes deep, within the offset of a body and \
                 beyond the bands: {deep}, {within} and {far}",
                cases.name
            );
        }
        let bands: Vec<Band> = counts.iter().map(|(_, band)| *band).collect();
        let expected = BAND
            .iter()
            .find(|(name, _)| *name == cases.name)
            .map(|(_, bands)| bands.to_vec())
            .unwrap_or_else(|| panic!("BAND holds no counts of {}", cases.name));
        assert_eq!(
            bands, expected,
            "{}: the probes inside the bands of each padding",
            cases.name
        );
    }
}

/// How the test places probes against the body of `piece`, for each of `paddings`, whose
/// center, as MoveIt grows it, is `moveit_center`. A primitive grows about its element's
/// origin, which MoveIt's center equals within [`FRAME_TOLERANCE`].
fn body_of<'a>(
    cases: &Cases,
    piece: &'a Piece,
    moveit_center: Point3<f64>,
    paddings: &[Padding],
    at: &str,
) -> Body<'a> {
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
    let exact = exact_hull(&surface).expect("a mesh hull");
    let hull = Hull::new(&exact, 0.0).expect("a hull");
    let fit_error = piece
        .hull()
        .vertices()
        .iter()
        .map(|v| gjk::distance(&Vertex(*v), &hull).distance)
        .fold(0.0, f64::max);
    let center = piece.hull().center();
    let scaled_hulls = paddings
        .iter()
        .map(|padding| {
            let scaled = ConvexHull {
                vertices: exact
                    .vertices
                    .iter()
                    .map(|v| center + (v - center) * padding.scale())
                    .collect(),
                faces: exact.faces.clone(),
                planes: exact
                    .planes
                    .iter()
                    .map(|&(normal, plane)| {
                        let from_center = normal.dot(&center.coords);
                        (
                            normal,
                            from_center + (plane - from_center) * padding.scale(),
                        )
                    })
                    .collect(),
            };
            Hull::new(&scaled, 0.0).expect("a scaled hull")
        })
        .collect();
    let link_to_element = element.origin.inverse();
    let (lower, upper) = exact.vertices.iter().map(|v| link_to_element * v).fold(
        (
            Point3::from(Vector3::repeat(f64::INFINITY)),
            Point3::from(Vector3::repeat(f64::NEG_INFINITY)),
        ),
        |(lower, upper), v| (lower.inf(&v), upper.sup(&v)),
    );
    Body::Mesh(Box::new(MeshBody {
        hull,
        scaled_hulls,
        link_to_element,
        box_center: collision_geometry::nalgebra::center(&lower, &upper),
        box_half_extents: (upper - lower) / 2.0,
        fit_error,
    }))
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
