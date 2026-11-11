//! The collision pieces of a chain of any shape: which links the chain carries, where
//! they stand, what each piece holds, and which points each grown link holds: every
//! point within the offset of its scaled elements, and no point farther from all of
//! them. The body fixed to the root around a chain: which links it holds, and which
//! points it holds, by the chain's rule.

use chain_kinematics::{Chain, ChainSpec, JointSelection};
use collision_geometry::GeometryError;
use collision_geometry::chain::{
    BodyGeometry, BodyMask, ChainGeometry, LinkGeometry, Padding, Piece, PlacedMask,
};
use collision_geometry::gjk::Hull;
use collision_geometry::hull::exact_hull;
use collision_geometry::nalgebra::{
    Isometry3, Point3, Translation3, UnitQuaternion, UnitVector3, Vector3,
};
use collision_geometry::urdf_collision::{CollisionGeometry, UrdfCollisions};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{
    ChainProperty, TIP, TOLERANCE, angles, configs, joint_name, link_name, on_a_drawn_robot,
    placements, unit_vectors,
};
use urdf_rs::{Axis, Joint, JointLimit, JointType, Link, LinkName, Robot, Vec3};

mod common;
use common::growth::grown_primitive_excess;
use common::properties::{CUBE, MESHES_DIR, MOUNT, SPUN, dressed, paddings};

/// The joints a dressed robot's chain drives: its drawn joints, from its mount to its
/// tip, past the continuous joints of the mount and of [`SPUN`], which the chain holds
/// at 0.
fn driven_joints<const N: usize>() -> Vec<String> {
    (1..=N).map(joint_name).collect()
}

/// The chain from the root to the tip, driven by `joints`.
fn spec<'a>(joints: &'a [&'a str]) -> ChainSpec<'a> {
    ChainSpec {
        base_link: None,
        tip_link: TIP,
        joints: JointSelection::Named(joints),
    }
}

/// The URDF of a drawn robot, dressed in collision elements.
fn dressed_urdf<const N: usize>(tc: &TestCase, robot: &Robot) -> String {
    urdf_rs::write_to_string(&tc.draw(dressed(robot.clone(), N))).expect("a drawn robot writes")
}

fn meshes_dir() -> &'static str {
    MESHES_DIR.to_str().expect("the mesh directory is UTF-8")
}

fn geometry_of<const N: usize>(urdf: &str) -> ChainGeometry<N> {
    let names = driven_joints::<N>();
    let joints: Vec<&str> = names.iter().map(String::as_str).collect();
    ChainGeometry::<N>::from_urdf(urdf, meshes_dir(), &spec(&joints)).expect("a dressed robot fits")
}

/// The index of a drawn item of `items`.
fn drawn_index<T>(tc: &TestCase, items: &[T]) -> usize {
    tc.draw(
        gs::integers::<usize>()
            .min_value(0)
            .max_value(items.len() - 1),
    )
}

/// A drawn point of the face `face` of `hull`, in the hull's frame, at a drawn vertex of
/// the face, on a drawn edge or inside the face, each as likely, with the vertices of the
/// face that the draw weighs. A weight of 0 puts the point on a part of that feature, so
/// every face through all the weighed vertices passes through the point.
fn point_on_face(tc: &TestCase, hull: &Hull, face: usize) -> (Point3<f64>, Vec<usize>) {
    let corners = hull.faces()[face];
    let count = tc.draw(gs::integers::<usize>().min_value(1).max_value(3));
    let first = tc.draw(gs::integers::<usize>().min_value(0).max_value(2));
    let weighed: Vec<(usize, f64)> = (0..count)
        .map(|k| {
            let vertex = corners[(first + k) % 3];
            (vertex, tc.draw(gs::floats().min_value(0.0).max_value(1.0)))
        })
        .collect();
    let total: f64 = weighed.iter().map(|(_, weight)| weight).sum();
    tc.assume(total > 0.0);
    let point = weighed
        .iter()
        .fold(Vector3::zeros(), |sum, &(vertex, weight)| {
            sum + hull.vertices()[vertex].coords * (weight / total)
        });
    let vertices = weighed.into_iter().map(|(vertex, _)| vertex).collect();
    (Point3::from(point), vertices)
}

/// The point that the collision element of `piece` scales about, in its link's frame:
/// the mean of the vertices of a mesh's piece, or a primitive's origin.
fn element_center(piece: &Piece) -> Point3<f64> {
    match piece.element().geometry {
        CollisionGeometry::Mesh { .. } => piece.hull().center(),
        _ => Point3::from(piece.element().origin.translation.vector),
    }
}

/// A drawn sign: -1, 0 or 1.
fn sign(tc: &TestCase) -> f64 {
    f64::from(tc.draw(gs::integers::<i8>().min_value(-1).max_value(1)))
}

/// The unit direction of `terms`, each a direction and a drawn weight, weighed by the
/// weights taken relative to the largest: the largest term enters at full length, so a
/// term whose relative weight leaves the normal range of floats moves the sum by at most
/// the smallest float. `None` when every weight is 0.
fn weighed_direction(terms: &[(UnitVector3<f64>, f64)]) -> Option<Vector3<f64>> {
    let largest = terms.iter().map(|&(_, weight)| weight).fold(0.0, f64::max);
    (largest > 0.0).then(|| {
        terms
            .iter()
            .fold(Vector3::zeros(), |sum, (direction, weight)| {
                sum + direction.into_inner() * (weight / largest)
            })
            .normalize()
    })
}

/// The [`weighed_direction`] of `terms`, the test case rejected when every weight is 0.
fn drawn_direction(tc: &TestCase, terms: &[(UnitVector3<f64>, f64)]) -> Vector3<f64> {
    weighed_direction(terms).unwrap_or_else(|| tc.reject())
}

/// A weighed direction has unit length and the direction of its largest term however
/// small the weights, as the surface directions that
/// a_point_beyond_the_offset_of_every_scaled_element_of_a_link_is_outside_it drew at
/// 10,000 cases weigh them: a cap's axis weighted 5.667e-162, whose square underflows,
/// and a face normal weighted below the normal range of floats.
#[test]
fn a_weighed_direction_holds_its_largest_term_however_small_the_weights() {
    let axis = Vector3::z_axis();
    let normal = UnitVector3::new_normalize(Vector3::new(-0.930, -0.367, -0.020));
    assert_eq!(
        weighed_direction(&[(axis, 5.667004161121765e-162)]),
        Some(axis.into_inner())
    );
    let along_normal = weighed_direction(&[(normal, 5e-324)]).expect("a weight above 0");
    assert!((along_normal - normal.into_inner()).amax() <= f64::EPSILON);
    assert_eq!(weighed_direction(&[(axis, 0.0), (normal, 0.0)]), None);
}

/// A drawn point of the surface of the collision element of `piece` scaled by `scale`
/// about its center, with a drawn outward unit direction of the surface's normal cone
/// there, both in the frame of its link: a point `t` meters from the surface point along
/// that direction lies `t` meters from the scaled element. A mesh's point lies on a
/// drawn face of its piece, at a vertex, on an edge or inside the face, and its
/// direction weighs the normals of the faces through it; a box's lies on a face, an edge
/// or a corner, a cylinder's on its side, a cap or a rim, and a sphere's anywhere.
fn scaled_surface_point(tc: &TestCase, piece: &Piece, scale: f64) -> (Point3<f64>, Vector3<f64>) {
    let weight = || tc.draw(gs::floats().min_value(0.0).max_value(1.0));
    let fraction = || tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
    let element = piece.element();
    let (point, terms) = match element.geometry {
        CollisionGeometry::Mesh { .. } => {
            let hull = piece.hull();
            let (on_face, vertices) = point_on_face(tc, hull, drawn_index(tc, hull.faces()));
            let terms: Vec<(UnitVector3<f64>, f64)> = hull
                .faces()
                .iter()
                .zip(hull.planes())
                .filter(|(face, _)| vertices.iter().all(|vertex| face.contains(vertex)))
                .map(|(_, (normal, _))| (UnitVector3::new_normalize(*normal), weight()))
                .collect();
            let center = hull.center();
            return (
                center + (on_face - center) * scale,
                drawn_direction(tc, &terms),
            );
        }
        CollisionGeometry::Box { size } => {
            let half_extents = size * (scale / 2.0);
            let sides = Vector3::from_fn(|_, _| sign(tc));
            tc.assume(sides != Vector3::zeros());
            let point = Vector3::from_fn(|i, _| {
                if sides[i] == 0.0 {
                    fraction() * half_extents[i]
                } else {
                    sides[i] * half_extents[i]
                }
            });
            let terms = (0..3)
                .filter(|&i| sides[i] != 0.0)
                .map(|i| {
                    (
                        UnitVector3::new_normalize(Vector3::ith(i, sides[i])),
                        weight(),
                    )
                })
                .collect();
            (point, terms)
        }
        CollisionGeometry::Cylinder { radius, length } => {
            let (radius, half_length) = (radius * scale, length * scale / 2.0);
            let (sin, cos) = tc.draw(angles()).sin_cos();
            let radial = Vector3::new(cos, sin, 0.0);
            let end = sign(tc);
            let on_side = end == 0.0 || tc.draw(gs::booleans());
            let across = if on_side { radius } else { radius * weight() };
            let along = if end == 0.0 {
                fraction() * half_length
            } else {
                end * half_length
            };
            let outward = on_side.then(|| (UnitVector3::new_normalize(radial), weight()));
            let axial =
                (end != 0.0).then(|| (UnitVector3::new_normalize(Vector3::z() * end), weight()));
            (
                radial * across + Vector3::z() * along,
                outward.into_iter().chain(axial).collect(),
            )
        }
        CollisionGeometry::Sphere { radius } => {
            let direction = tc.draw(unit_vectors());
            (
                direction * (radius * scale),
                vec![(UnitVector3::new_normalize(direction), 1.0)],
            )
        }
    };
    (
        element.origin * Point3::from(point),
        element.origin.rotation * drawn_direction(tc, &terms),
    )
}

/// How far `point`, in the frame of its link, lies beyond the farthest face of the
/// collision element of `piece` scaled by `scale` about its center, negative inside, so
/// no farther than its distance from the scaled element: a face of a mesh's piece, a
/// face of a box, the side or a cap of a cylinder, or the surface of a sphere.
fn beyond_the_scaled_faces(piece: &Piece, scale: f64, point: &Point3<f64>) -> f64 {
    let scaled = Padding::new(scale, 0.0).expect("a valid padding");
    grown_primitive_excess(piece.element(), scaled, point).unwrap_or_else(|| {
        let hull = piece.hull();
        let center = hull.center();
        hull.planes()
            .iter()
            .map(|(normal, plane)| {
                let from_center = normal.dot(&center.coords);
                normal.dot(&point.coords) - (from_center + (plane - from_center) * scale)
            })
            .fold(f64::NEG_INFINITY, f64::max)
    })
}

/// The chain carries the links its joints move, the tip fixed below the last one and the
/// spun link on the continuous joint of the path that the chain holds at 0 included, and
/// no other: not the mount at the root, nor the stand fixed to `link_0` below the
/// mount's continuous joint, nor the finger on a joint off the path that the chain does
/// not drive. Each link carries one piece per collision element. The tip stands where
/// the chain's own kinematics put it.
#[hegel::test]
fn the_chain_carries_the_links_its_joints_move(tc: TestCase) {
    struct CarriesMovedLinks;
    impl ChainProperty for CarriesMovedLinks {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let urdf = dressed_urdf::<N>(tc, robot);
            let geometry = geometry_of::<N>(&urdf);
            let mut carried: Vec<&str> = geometry.links().iter().map(LinkGeometry::name).collect();
            carried.sort_unstable();
            let mut moved: Vec<String> = (1..=N)
                .map(link_name)
                .chain([TIP.to_owned(), SPUN.to_owned()])
                .collect();
            moved.sort_unstable();
            assert_eq!(carried, moved);

            let collisions = UrdfCollisions::from_urdf(&urdf).expect("the written URDF reads");
            for link in geometry.links() {
                assert_eq!(
                    link.pieces().len(),
                    collisions.collisions_of(link.name()).len(),
                    "{}: one piece per collision element",
                    link.name()
                );
            }

            let q = tc.draw(configs(geometry.chain().limits()));
            let names = driven_joints::<N>();
            let joints: Vec<&str> = names.iter().map(String::as_str).collect();
            let chain = Chain::<N>::from_urdf(
                &urdf_rs::read_from_string(&urdf).expect("the written URDF reads"),
                &spec(&joints),
            )
            .expect("a drawn robot is a valid chain");
            let tip = chain.world_ee_pose(&q);
            let placed = geometry.place(&q);
            let (_, tip_pose) = placed
                .links()
                .find(|(link, _)| link.name() == TIP)
                .expect("the chain carries its tip");
            assert!(
                (tip_pose.translation.vector - tip.translation.vector).norm() < TOLERANCE
                    && tip_pose.rotation.angle_to(&tip.rotation) < TOLERANCE,
                "the tip stands at {tip_pose}, the chain puts it at {tip}"
            );
        }
    }
    on_a_drawn_robot::<CarriesMovedLinks>(&tc);
}

/// Each piece holds the surface of its collision element: a mesh, or the polytope
/// about a box, a cylinder or a sphere.
#[hegel::test]
fn each_piece_holds_its_collision_surface(tc: TestCase) {
    struct PieceHoldsSurface;
    impl ChainProperty for PieceHoldsSurface {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let urdf = dressed_urdf::<N>(tc, robot);
            let geometry = geometry_of::<N>(&urdf);
            for link in geometry.links() {
                for piece in link.pieces() {
                    let surface = piece
                        .element()
                        .surface(meshes_dir())
                        .expect("the surface reads");
                    for point in &surface {
                        let escape = piece
                            .hull()
                            .planes()
                            .iter()
                            .map(|(normal, offset)| normal.dot(&point.coords) - offset)
                            .fold(f64::NEG_INFINITY, f64::max);
                        assert!(
                            escape <= TOLERANCE,
                            "a point of a collision surface of {} lies {escape} m outside its piece",
                            link.name()
                        );
                    }
                }
            }
        }
    }
    on_a_drawn_robot::<PieceHoldsSurface>(&tc);
}

/// A floor on a joint raises its lower limit as the chain's own floor does, and leaves
/// every other limit and every link where it stands.
#[hegel::test]
fn a_floor_raises_one_lower_limit_alone(tc: TestCase) {
    struct FloorRaisesOneLimit;
    impl ChainProperty for FloorRaisesOneLimit {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let urdf = dressed_urdf::<N>(tc, robot);
            let joint = tc.draw(gs::integers::<usize>().min_value(0).max_value(N - 1));
            let limits = geometry_of::<N>(&urdf).chain().limits();
            let floor = tc.draw(
                gs::floats()
                    .min_value(limits[joint].lo - 1.0)
                    .max_value(limits[joint].hi + 1.0),
            );
            let floored = geometry_of::<N>(&urdf).with_lower_floor(joint, floor);
            let names = driven_joints::<N>();
            let joints: Vec<&str> = names.iter().map(String::as_str).collect();
            let expected = Chain::<N>::from_urdf(
                &urdf_rs::read_from_string(&urdf).expect("the written URDF reads"),
                &spec(&joints),
            )
            .expect("a drawn robot is a valid chain")
            .with_lower_floor(joint, floor)
            .limits();
            assert_eq!(floored.chain().limits(), expected);
            assert!(
                (0..N)
                    .filter(|&i| i != joint)
                    .all(|i| floored.chain().limits()[i] == limits[i]),
                "a floor on joint {joint} moved another limit"
            );
            let q = tc.draw(configs(floored.chain().limits()));
            let poses = |geometry: &ChainGeometry<N>| -> Vec<Isometry3<f64>> {
                geometry.place(&q).links().map(|(_, pose)| *pose).collect()
            };
            assert_eq!(poses(&floored), poses(&geometry_of::<N>(&urdf)));
        }
    }
    on_a_drawn_robot::<FloorRaisesOneLimit>(&tc);
}

/// The bounding sphere of each piece holds the piece wherever the chain stands.
#[hegel::test]
fn each_bounding_sphere_holds_its_piece(tc: TestCase) {
    struct SphereHoldsPiece;
    impl ChainProperty for SphereHoldsPiece {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let geometry = geometry_of::<N>(&dressed_urdf::<N>(tc, robot));
            let q = tc.draw(configs(geometry.chain().limits()));
            for (link, piece) in geometry.place(&q).pieces() {
                let (center, radius) = piece.bounding_sphere();
                for vertex in piece.vertices() {
                    let from_center = (vertex - center).norm();
                    assert!(
                        from_center <= radius + TOLERANCE,
                        "a vertex of {link} lies {from_center} m from the center of its sphere of radius {radius} m"
                    );
                }
            }
        }
    }
    on_a_drawn_robot::<SphereHoldsPiece>(&tc);
}

/// Every point within the offset of a scaled collision element lies inside its grown
/// link, and inside the grown chain: a point of the segment from the element's center
/// to a drawn point of its scaled surface, short of the surface by [`TOLERANCE`] of the
/// segment, or a point beyond that surface point along a direction of the normal cone
/// there, by no more than the offset less [`TOLERANCE`].
#[hegel::test]
fn a_point_within_the_offset_of_a_scaled_element_is_inside_its_link(tc: TestCase) {
    struct WithinTheOffsetIsInside;
    impl ChainProperty for WithinTheOffsetIsInside {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let geometry = geometry_of::<N>(&dressed_urdf::<N>(tc, robot));
            let q = tc.draw(configs(geometry.chain().limits()));
            let padding = tc.draw(paddings());
            let placed = geometry.place(&q);
            let links: Vec<(&LinkGeometry, &Isometry3<f64>)> = placed.links().collect();
            let index = drawn_index(tc, &links);
            let (link, pose) = links[index];
            let piece = &link.pieces()[drawn_index(tc, link.pieces())];
            let (surface, direction) = scaled_surface_point(tc, piece, padding.scale());
            let fraction = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
            let local = if padding.offset() < TOLERANCE || tc.draw(gs::booleans()) {
                let center = element_center(piece);
                center + (surface - center) * ((1.0 - TOLERANCE) * fraction)
            } else {
                surface + direction * ((padding.offset() - TOLERANCE) * fraction)
            };
            let point = pose * local;
            let mask = geometry.mask(padding);
            let grown = mask.at(&q);
            assert!(
                grown.links()[index].contains(&point),
                "{point} lies within the offset of a scaled element of {}, padding \
                 {padding:?}, and outside the grown link",
                link.name()
            );
            assert!(
                grown.contains(&point),
                "{point} lies outside the grown chain"
            );
        }
    }
    on_a_drawn_robot::<WithinTheOffsetIsInside>(&tc);
}

/// No point farther than the offset from every scaled collision element of a link lies
/// inside the grown link: a point beyond a drawn point of the surface of a scaled element,
/// along a direction of the normal cone there, by the offset and 2 [`TOLERANCE`] to 1 cm
/// more, that lies beyond a face of every other scaled element of its link by more than
/// the offset.
#[hegel::test]
fn a_point_beyond_the_offset_of_every_scaled_element_of_a_link_is_outside_it(tc: TestCase) {
    struct BeyondTheOffsetIsOutside;
    impl ChainProperty for BeyondTheOffsetIsOutside {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let geometry = geometry_of::<N>(&dressed_urdf::<N>(tc, robot));
            let q = tc.draw(configs(geometry.chain().limits()));
            let padding = tc.draw(paddings());
            let placed = geometry.place(&q);
            let links: Vec<(&LinkGeometry, &Isometry3<f64>)> = placed.links().collect();
            let index = drawn_index(tc, &links);
            let (link, pose) = links[index];
            let piece = &link.pieces()[drawn_index(tc, link.pieces())];
            let (surface, direction) = scaled_surface_point(tc, piece, padding.scale());
            let beyond = tc.draw(gs::floats().min_value(2.0 * TOLERANCE).max_value(0.01));
            let local = surface + direction * (padding.offset() + beyond);
            tc.assume(link.pieces().iter().all(|other| {
                std::ptr::eq(other, piece)
                    || beyond_the_scaled_faces(other, padding.scale(), &local)
                        > padding.offset() + TOLERANCE
            }));
            let point = pose * local;
            let mask = geometry.mask(padding);
            let grown = mask.at(&q);
            assert!(
                !grown.links()[index].contains(&point),
                "{point} lies beyond the offset of every scaled element of {}, padding \
                 {padding:?}, and inside the grown link",
                link.name()
            );
        }
    }
    on_a_drawn_robot::<BeyondTheOffsetIsOutside>(&tc);
}

/// The grown chain answers for a point as its links do, one by one, each screening every
/// piece: the reaches of the grown pieces skip only pieces that do not hold the point,
/// and hold only points that a piece holds. A drawn point lies beyond a drawn point of
/// the surface of a scaled element, along a direction of the normal cone there, by up to
/// one and a half times the offset, or on the segment from the element's center to that
/// surface point. The same point with a drawn coordinate not a number, or infinite, lies
/// inside neither the chain nor any of its links.
#[hegel::test]
fn the_grown_chain_answers_as_its_links_one_by_one(tc: TestCase) {
    struct AnswersAsItsLinks;
    impl ChainProperty for AnswersAsItsLinks {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let geometry = geometry_of::<N>(&dressed_urdf::<N>(tc, robot));
            let q = tc.draw(configs(geometry.chain().limits()));
            let padding = tc.draw(paddings());
            let placed = geometry.place(&q);
            let links: Vec<(&LinkGeometry, &Isometry3<f64>)> = placed.links().collect();
            let (link, pose) = links[drawn_index(tc, &links)];
            let piece = &link.pieces()[drawn_index(tc, link.pieces())];
            let (surface, direction) = scaled_surface_point(tc, piece, padding.scale());
            let local = if tc.draw(gs::booleans()) {
                let out = tc.draw(gs::floats().min_value(0.0).max_value(1.5));
                surface + direction * (padding.offset() * out)
            } else {
                let center = element_center(piece);
                let along = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
                center + (surface - center) * along
            };
            let point = pose * local;
            let mask = geometry.mask(padding);
            let grown = mask.at(&q);
            let one_by_one = |point: &Point3<f64>| grown.links().iter().any(|l| l.contains(point));
            assert_eq!(
                grown.contains(&point),
                one_by_one(&point),
                "{point}, padding {padding:?}: the grown chain answers on the left, its links \
                 one by one on the right"
            );
            let axis = tc.draw(gs::integers::<usize>().min_value(0).max_value(2));
            let value = tc.draw(gs::sampled_from(vec![
                f64::NAN,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ]));
            let broken = Point3::from(Vector3::from_fn(
                |i, _| if i == axis { value } else { point[i] },
            ));
            assert_eq!(
                (grown.contains(&broken), one_by_one(&broken)),
                (false, false),
                "{broken}, padding {padding:?}: the grown chain, then its links one by one"
            );
        }
    }
    on_a_drawn_robot::<AnswersAsItsLinks>(&tc);
}

/// A chain of one driven joint whose link, [`TIP`], holds the collision element of the
/// URDF `<geometry>` body `geometry` at its origin, the link's frame the root's at 0.
fn one_link(geometry: &str) -> ChainGeometry<1> {
    geometry_of::<1>(&format!(
        r#"<robot name="one_link">
             <link name="{root}"/>
             <joint name="{joint}" type="revolute">
               <parent link="{root}"/>
               <child link="{TIP}"/>
               <axis xyz="0 0 1"/>
               <limit lower="-1" upper="1" effort="1" velocity="1"/>
             </joint>
             <link name="{TIP}"><collision><geometry>{geometry}</geometry></collision></link>
           </robot>"#,
        root = link_name(0),
        joint = joint_name(1),
    ))
}

/// The padding of the one-link cases: a scale of 1.1 and an offset of 3 cm.
fn one_link_padding() -> Padding {
    Padding::new(1.1, 0.03).expect("a valid padding")
}

/// Whether the grown chain of [`one_link`] at 0 holds `point`, asked of the chain and of
/// its link, which answer alike.
fn one_link_holds(grown: &PlacedMask, point: &Point3<f64>) -> bool {
    let link = grown.links()[0].contains(point);
    assert_eq!(
        grown.contains(point),
        link,
        "{point}: the grown chain answers on the left, its link on the right"
    );
    link
}

/// A point beyond the rim of a scaled cylinder by 0.8 of the offset along its radius and
/// along its axis lies 1.13 offsets from the cylinder, outside the grown link, and one 0.9
/// offsets beyond the rim along the same diagonal lies inside it. A point 1.1 offsets out
/// from the cylinder's side, halfway along it, lies outside.
#[test]
fn a_point_past_the_rim_or_the_side_of_a_scaled_cylinder_is_inside_it_to_the_offset() {
    let padding = one_link_padding();
    let geometry = one_link(r#"<cylinder radius="0.05" length="0.2"/>"#);
    let mask = geometry.mask(padding);
    let grown = mask.at(&[0.0]);
    let (radius, half_length) = (0.05 * padding.scale(), 0.1 * padding.scale());
    let past_the_rim = |step: f64| Point3::new(radius + step, 0.0, half_length + step);
    let point = past_the_rim(0.8 * padding.offset());
    assert!(
        !one_link_holds(&grown, &point),
        "{point} lies 1.13 offsets from the scaled cylinder, padding {padding:?}, and \
         inside the grown link"
    );
    let point = past_the_rim(0.9 * padding.offset() / 2f64.sqrt());
    assert!(
        one_link_holds(&grown, &point),
        "{point} lies 0.9 offsets from the scaled cylinder's rim, padding {padding:?}, and \
         outside the grown link"
    );
    let point = Point3::new(radius + 1.1 * padding.offset(), 0.0, 0.0);
    assert!(
        !one_link_holds(&grown, &point),
        "{point} lies 1.1 offsets from the scaled cylinder's side, padding {padding:?}, and \
         inside the grown link"
    );
}

/// A point beyond a corner of a scaled box by 0.7 of the offset along each axis lies 1.21
/// offsets from the box, outside the grown link, and one 0.9 offsets beyond the corner
/// along the same diagonal lies inside it. A point 1.1 offsets out from the middle of the
/// box's nearest face lies outside.
#[test]
fn a_point_past_the_corner_or_a_face_of_a_scaled_box_is_inside_it_to_the_offset() {
    let padding = one_link_padding();
    let geometry = one_link(r#"<box size="0.1 0.2 0.3"/>"#);
    let mask = geometry.mask(padding);
    let grown = mask.at(&[0.0]);
    let corner = Vector3::new(0.05, 0.1, 0.15) * padding.scale();
    let point = Point3::from(corner.add_scalar(0.7 * padding.offset()));
    assert!(
        !one_link_holds(&grown, &point),
        "{point} lies 1.21 offsets from the scaled box, padding {padding:?}, and inside \
         the grown link"
    );
    let point = Point3::from(corner.add_scalar(0.9 * padding.offset() / 3f64.sqrt()));
    assert!(
        one_link_holds(&grown, &point),
        "{point} lies 0.9 offsets from the scaled box's corner, padding {padding:?}, and \
         outside the grown link"
    );
    let point = Point3::new(corner.x + 1.1 * padding.offset(), 0.0, 0.0);
    assert!(
        !one_link_holds(&grown, &point),
        "{point} lies 1.1 offsets from the scaled box's nearest face, padding {padding:?}, \
         and inside the grown link"
    );
}

/// Each vertex of the scaled piece of a mesh, moved out by half the offset along the
/// normal of each face through it, lies half the offset from the piece, the vertex its
/// nearest point, and inside the grown link. The mesh is a box, two faces to a side, so
/// a vertex that one face of a side holds and the other lacks, moved along the side's
/// normal, lies as high above the other face, which lies farther than the offset from it.
/// Each vertex moved out by 0.9 offsets along the line from the piece's center, which
/// leaves the vertex its nearest point, lies inside too, and a point 1.1 offsets out from
/// the middle of the piece's nearest face lies outside.
#[test]
fn a_point_past_a_vertex_or_a_face_of_a_scaled_mesh_is_inside_it_to_the_offset() {
    let padding = one_link_padding();
    let geometry = one_link(&format!(
        r#"<mesh filename="package://drawn/meshes/{CUBE}" scale="0.1 0.2 0.3"/>"#
    ));
    let hull = geometry.links()[0].pieces()[0].hull();
    let center = hull.center();
    let mask = geometry.mask(padding);
    let grown = mask.at(&[0.0]);
    for (face, (normal, _)) in hull.faces().iter().zip(hull.planes()) {
        for vertex in face.map(|vertex| hull.vertices()[vertex]) {
            let scaled = center + (vertex - center) * padding.scale();
            let point = scaled + normal * (padding.offset() / 2.0);
            assert!(
                one_link_holds(&grown, &point),
                "{point} lies half the offset from the scaled piece, padding {padding:?}, \
                 and outside the grown link"
            );
            let point = scaled + (scaled - center).normalize() * (0.9 * padding.offset());
            assert!(
                one_link_holds(&grown, &point),
                "{point} lies 0.9 offsets from the scaled piece, padding {padding:?}, and \
                 outside the grown link"
            );
        }
    }
    let (normal, height) = hull
        .planes()
        .iter()
        .map(|(normal, plane)| (normal, plane - normal.dot(&center.coords)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .expect("a piece has faces");
    let point = center + normal * (height * padding.scale() + 1.1 * padding.offset());
    assert!(
        !one_link_holds(&grown, &point),
        "{point} lies 1.1 offsets from the scaled piece's nearest face, padding \
         {padding:?}, and inside the grown link"
    );
}

/// The root a body test hangs a dressed robot's mount from: it holds no collision element.
const HANGER: &str = "hanger";

/// The joint from [`HANGER`] to the mount.
const HANG: &str = "hang_joint";

/// `robot`, a dressed robot, with its mount hung from [`HANGER`] by [`HANG`], a joint of
/// kind `kind` at `origin` that turns about its z axis where it moves.
fn hung(robot: &Robot, kind: JointType, origin: urdf_rs::Pose) -> String {
    let hanger = Link {
        name: HANGER.to_owned(),
        inertial: Default::default(),
        visual: Vec::new(),
        collision: Vec::new(),
    };
    let hang = Joint {
        name: HANG.to_owned(),
        joint_type: kind,
        origin,
        parent: LinkName {
            link: HANGER.to_owned(),
        },
        child: LinkName {
            link: MOUNT.to_owned(),
        },
        axis: Axis {
            xyz: Vec3([0.0, 0.0, 1.0]),
        },
        limit: JointLimit {
            lower: -1.0,
            upper: 1.0,
            effort: 1.0,
            velocity: 1.0,
            ..JointLimit::default()
        },
        calibration: None,
        dynamics: None,
        mimic: None,
        safety_controller: None,
    };
    let robot = Robot {
        links: robot.links.iter().cloned().chain([hanger]).collect(),
        joints: robot.joints.iter().cloned().chain([hang]).collect(),
        ..robot.clone()
    };
    urdf_rs::write_to_string(&robot).expect("a hung robot writes")
}

/// The body of a dressed robot hung by a fixed joint at a drawn placement holds the mount
/// alone: not the empty hanger at the root, nor the stand fixed to `link_0` below the
/// mount's continuous joint, nor any link a joint moves. Its grown mount answers for every
/// point as the grown chain that a revolute joint at that placement carries the mount on
/// answers at 0: a point beyond a drawn point of the surface of a scaled element of the
/// mount, along a direction of the normal cone there, by up to one and a half times the
/// offset, [`TOLERANCE`] or more off the grown element's surface, or on the segment from
/// the element's center to that surface point, short of it by [`TOLERANCE`] of the
/// segment or more.
#[hegel::test]
fn the_body_answers_for_a_point_as_a_chain_carrying_its_link(tc: TestCase) {
    struct AnswersAsAChain;
    impl ChainProperty for AnswersAsAChain {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let dressed = tc.draw(dressed(robot.clone(), N));
            let origin = tc.draw(placements());
            let body = BodyGeometry::from_urdf(
                &hung(&dressed, JointType::Fixed, origin.clone()),
                meshes_dir(),
                &[],
            )
            .expect("a dressed body fits");
            assert_eq!(body.link_names().collect::<Vec<_>>(), [MOUNT]);
            let carrier = ChainGeometry::<1>::from_urdf(
                &hung(&dressed, JointType::Revolute, origin),
                meshes_dir(),
                &ChainSpec {
                    base_link: None,
                    tip_link: MOUNT,
                    joints: JointSelection::Named(&[HANG]),
                },
            )
            .expect("a dressed mount on a joint fits");
            let placed = carrier.place(&[0.0]);
            let [(link, pose)] = <[_; 1]>::try_from(placed.links().collect::<Vec<_>>())
                .expect("the chain carries the mount alone");
            assert_eq!(link.name(), MOUNT);
            let padding = tc.draw(paddings());
            let piece = &link.pieces()[drawn_index(tc, link.pieces())];
            let (surface, direction) = scaled_surface_point(tc, piece, padding.scale());
            let local = if tc.draw(gs::booleans()) {
                let out: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.5));
                tc.assume((out - 1.0).abs() * padding.offset() >= TOLERANCE);
                surface + direction * (padding.offset() * out)
            } else {
                let center = element_center(piece);
                let along = tc.draw(gs::floats().min_value(0.0).max_value(1.0 - TOLERANCE));
                center + (surface - center) * along
            };
            let point = pose * local;
            let grown_body = body.mask(padding);
            let grown_chain = carrier.mask(padding);
            assert_eq!(
                grown_body.placed().contains(&point),
                grown_chain.at(&[0.0]).contains(&point),
                "{point}, padding {padding:?}: the grown body answers on the left, the \
                 grown chain on the right"
            );
        }
    }
    on_a_drawn_robot::<AnswersAsAChain>(&tc);
}

/// The overlap of the two clip regions that a test decomposes a body's link into, as the
/// regions' rules ask of adjoining regions.
const OVERLAP_M: f64 = 0.005;

/// The mean of `points`.
fn mean(points: &[&Point3<f64>]) -> Point3<f64> {
    Point3::from(
        points
            .iter()
            .fold(Vector3::zeros(), |sum, p| sum + p.coords)
            / points.len() as f64,
    )
}

/// A drawn point inside the convex hull of `surface`, off its boundary: a weighted mean of
/// four of its points, moved [`TOLERANCE`] of the way toward the mean of all its points,
/// which lies inside the hull.
fn inside_surface(tc: &TestCase, surface: &[Point3<f64>]) -> Point3<f64> {
    let weighted: Vec<(Point3<f64>, f64)> = (0..4)
        .map(|_| {
            let vertex = surface[drawn_index(tc, surface)];
            (vertex, tc.draw(gs::floats().min_value(0.0).max_value(1.0)))
        })
        .collect();
    let total: f64 = weighted.iter().map(|(_, weight)| weight).sum();
    tc.assume(total > 0.0);
    let drawn = Point3::from(
        weighted
            .iter()
            .fold(Vector3::zeros(), |sum, (vertex, weight)| {
                sum + vertex.coords * (weight / total)
            }),
    );
    let middle = mean(&surface.iter().collect::<Vec<_>>());
    drawn + (middle - drawn) * TOLERANCE
}

/// A body whose mount, hung by a fixed joint at a drawn placement, is decomposed into two
/// clip regions in the frame of the root, split across a drawn axis in the middle half of
/// the mount's surfaces and overlapping by [`OVERLAP_M`], holds a drawn point inside the
/// hull of a surface of the mount, off its boundary, for every padding, which grows each
/// region's piece about its center, and no point farther from the surfaces' middle than
/// the pieces reach: a scaled piece lies within the scale times the surfaces' diameter of
/// their middle, and its grown points within the offset more.
#[hegel::test]
fn a_body_decomposed_into_regions_holds_its_link_alone(tc: TestCase) {
    struct Decomposed;
    impl ChainProperty for Decomposed {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let dressed = tc.draw(dressed(robot.clone(), N));
            let urdf = hung(&dressed, JointType::Fixed, tc.draw(placements()));
            let surfaces = UrdfCollisions::from_urdf(&urdf)
                .expect("a hung robot's collisions read")
                .fixed_surfaces_in_root(MOUNT, meshes_dir())
                .expect("the mount is fixed to the root");
            let points: Vec<&Point3<f64>> = surfaces.iter().flatten().collect();
            let axis = tc.draw(gs::integers::<usize>().min_value(0).max_value(2));
            let (low, high) = points
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), p| {
                    (low.min(p[axis]), high.max(p[axis]))
                });
            let split = low + (high - low) * tc.draw(gs::floats().min_value(0.25).max_value(0.75));
            let (mut below, mut above) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            below[axis] = split + OVERLAP_M / 2.0;
            above[axis] = split - OVERLAP_M / 2.0;
            let regions = [[[f64::NEG_INFINITY; 3], below], [above, [f64::INFINITY; 3]]];
            let body = BodyGeometry::from_urdf(&urdf, meshes_dir(), &[(MOUNT, &regions)])
                .expect("a decomposed body fits");
            assert_eq!(body.link_names().collect::<Vec<_>>(), [MOUNT]);

            let padding = tc.draw(paddings());
            let grown = body.mask(padding);
            let placed = grown.placed();
            let point = inside_surface(tc, &surfaces[drawn_index(tc, &surfaces)]);
            assert!(
                placed.contains(&point),
                "{point} lies inside a surface of {MOUNT}, and outside its regions grown by \
                 {padding:?}"
            );
            let middle = mean(&points);
            let radius = points
                .iter()
                .map(|p| (*p - middle).norm())
                .fold(0.0, f64::max);
            let reach = radius * (1.0 + 2.0 * padding.scale()) + padding.offset() + 0.01;
            let far = middle + tc.draw(unit_vectors()) * reach;
            assert!(
                !placed.contains(&far),
                "{far} lies beyond the reach of the regions grown by {padding:?}, and inside \
                 them"
            );
        }
    }
    on_a_drawn_robot::<Decomposed>(&tc);
}

/// A robot whose body is a box at the root, a cylinder on a link fixed at a turned origin
/// beside it, and a cube mesh on a link fixed above it, whose arm and hand a revolute
/// joint moves.
const PLACED_BODY: &str = r#"<robot name="placed_body">
    <link name="base">
      <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
    </link>
    <joint name="plate_joint" type="fixed">
      <parent link="base"/><child link="plate"/>
      <origin xyz="0.5 0 0" rpy="0 0 1.5707963267948966"/>
    </joint>
    <link name="plate">
      <collision>
        <origin xyz="0 0.1 0"/>
        <geometry><cylinder radius="0.05" length="0.2"/></geometry>
      </collision>
    </link>
    <joint name="torso_joint" type="fixed">
      <parent link="base"/><child link="torso"/><origin xyz="0 0 1"/>
    </joint>
    <link name="torso">
      <collision>
        <geometry><mesh filename="package://drawn/meshes/cube.stl" scale="0.4 0.4 0.4"/></geometry>
      </collision>
    </link>
    <joint name="arm_joint" type="revolute">
      <parent link="base"/><child link="arm"/><origin xyz="-0.5 0 0"/>
      <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
    </joint>
    <link name="arm">
      <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
    </link>
    <joint name="hand_joint" type="fixed">
      <parent link="arm"/><child link="hand"/><origin xyz="0 0 0.3"/>
    </joint>
    <link name="hand">
      <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
    </link>
  </robot>"#;

/// The clip regions that split [`PLACED_BODY`]'s torso, the cube from z = 0.8 m to 1.2 m
/// in the root's frame, at z = 1 m, overlapping by [`OVERLAP_M`].
const TORSO_HALVES: [[[f64; 3]; 2]; 2] = [
    [
        [f64::NEG_INFINITY; 3],
        [f64::INFINITY, f64::INFINITY, 1.0 + OVERLAP_M / 2.0],
    ],
    [
        [f64::NEG_INFINITY, f64::NEG_INFINITY, 1.0 - OVERLAP_M / 2.0],
        [f64::INFINITY; 3],
    ],
];

/// The body of [`PLACED_BODY`], its torso split into [`TORSO_HALVES`].
fn placed_body() -> BodyGeometry {
    BodyGeometry::from_urdf(PLACED_BODY, meshes_dir(), &[("torso", &TORSO_HALVES)])
        .expect("the placed body fits")
}

/// The body holds the root's box, the plate's cylinder where its turned joint puts it,
/// at x = 0.4 m along the root's z, and the torso's halves where the root's frame puts
/// them, each to the offset of 3 cm and not 3.3 cm past it; it holds no point of the arm
/// or the hand. Scaled by 1.5, the torso's upper half, from z = 0.9975 m to 1.2 m about
/// its middle, reaches z = 1.2506 m.
#[test]
fn a_body_stands_where_its_fixed_joints_place_it() {
    let body = placed_body();
    let mut names: Vec<&str> = body.link_names().collect();
    names.sort_unstable();
    assert_eq!(names, ["base", "plate", "torso"]);
    let offset = 0.03;
    let grown = body.mask(Padding::new(1.0, offset).expect("a valid padding"));
    let placed = grown.placed();
    let within = 0.9 * offset;
    let beyond = 1.1 * offset;
    for (inside, outside) in [
        ([0.1 + within, 0.0, 0.0], [0.1 + beyond, 0.0, 0.0]),
        ([0.4, 0.05 + within, 0.0], [0.4, 0.05 + beyond, 0.0]),
        ([0.4, 0.0, 0.1 + within], [0.4, 0.0, 0.1 + beyond]),
        ([0.0, 0.0, 1.2 + within], [0.0, 0.0, 1.2 + beyond]),
        ([0.2 + within, 0.0, 0.85], [0.2 + beyond, 0.0, 0.85]),
    ] {
        let (inside, outside) = (Point3::from(inside), Point3::from(outside));
        assert!(placed.contains(&inside), "{inside} lies outside the body");
        assert!(!placed.contains(&outside), "{outside} lies inside the body");
    }
    for moved in [[-0.5, 0.0, 0.0], [-0.5, 0.0, 0.3]] {
        let point = Point3::from(moved);
        assert!(
            !placed.contains(&point),
            "{point}, inside a link a joint moves, lies inside the body"
        );
    }
    let scaled = body.mask(Padding::new(1.5, 0.0).expect("a valid padding"));
    let placed = scaled.placed();
    assert!(placed.contains(&Point3::new(0.0, 0.0, 1.245)));
    assert!(!placed.contains(&Point3::new(0.0, 0.0, 1.256)));
}

/// Clip regions for a link a joint moves, for a link the robot does not have, or twice
/// for one link of the body, are refused, naming the link.
#[test]
fn regions_off_the_body_or_given_twice_are_refused() {
    for link in ["arm", "hand", "nowhere"] {
        let refused = BodyGeometry::from_urdf(PLACED_BODY, meshes_dir(), &[(link, &TORSO_HALVES)]);
        assert!(
            matches!(&refused, Err(GeometryError::RegionsOffTheBody { body }) if body == link),
            "regions for {link}: {refused:?}"
        );
    }
    let refused = BodyGeometry::from_urdf(
        PLACED_BODY,
        meshes_dir(),
        &[("torso", &TORSO_HALVES), ("torso", &TORSO_HALVES)],
    );
    assert!(
        matches!(&refused, Err(GeometryError::RegionsTwice { body }) if body == "torso"),
        "{refused:?}"
    );
}

/// A cube of half side 0.1 m, as a fitted piece, rounded by `rounding` meters.
fn cube_piece(rounding: f64) -> Hull {
    let corners: Vec<Point3<f64>> = (0..8)
        .map(|i| {
            let side = |bit: usize| if i & bit == 0 { -0.1 } else { 0.1 };
            Point3::new(side(1), side(2), side(4))
        })
        .collect();
    Hull::new(&exact_hull(&corners).expect("a cube's hull"), rounding).expect("a cube")
}

/// A part a caller places grows where it stands: a cube of half side 0.1 m, turned a
/// quarter about z and moved 1 m along x, holds at no padding a point 0.09 m past its
/// middle along each axis and not one 0.11 m past; scaled by 2, 0.19 m and not 0.21 m; and
/// not the root's origin.
#[test]
fn a_placed_part_grows_where_it_stands() {
    let hull = cube_piece(0.0);
    let pose = Isometry3::from_parts(
        Translation3::new(1.0, 0.0, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
    );
    let parts = || [("part", std::slice::from_ref(&hull), pose)];
    for (scale, within, beyond) in [(1.0, 0.09, 0.11), (2.0, 0.19, 0.21)] {
        let grown = BodyMask::of(parts(), Padding::new(scale, 0.0).expect("a valid padding"));
        let placed = grown.placed();
        for axis in [Vector3::x(), Vector3::y(), Vector3::z()] {
            let (inside, outside) = (
                pose * Point3::from(axis * within),
                pose * Point3::from(axis * beyond),
            );
            assert!(
                placed.contains(&inside),
                "{inside} lies outside the part scaled by {scale}"
            );
            assert!(
                !placed.contains(&outside),
                "{outside} lies inside the part scaled by {scale}"
            );
        }
        assert!(
            !placed.contains(&Point3::origin()),
            "the part stands at the root"
        );
    }
}

/// A part a caller places is a fitted piece, which carries no rounding: a rounded one is
/// refused.
#[test]
#[should_panic(expected = "a fitted piece carries no rounding")]
fn a_rounded_part_is_refused() {
    let hull = cube_piece(0.05);
    let parts = [("part", std::slice::from_ref(&hull), Isometry3::identity())];
    BodyMask::of(parts, Padding::new(1.0, 0.0).expect("a valid padding"));
}
