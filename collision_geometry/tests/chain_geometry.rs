//! The collision pieces of a chain of any shape: which links the chain carries, where
//! they stand, what each piece holds, and which points each grown link holds: every
//! point within the offset of its scaled elements, and no point farther from all of
//! them.

use chain_kinematics::{Chain, ChainSpec, JointSelection};
use collision_geometry::chain::{ChainGeometry, LinkGeometry, Padding, Piece};
use collision_geometry::gjk::Hull;
use collision_geometry::nalgebra::{Isometry3, Point3, Vector3};
use collision_geometry::urdf_collision::{CollisionGeometry, UrdfCollisions};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{
    ChainProperty, TIP, TOLERANCE, angles, configs, joint_name, link_name, on_a_drawn_robot,
    unit_vectors,
};
use urdf_rs::Robot;

mod common;
use common::growth::grown_primitive_excess;
use common::properties::{CUBE, MESHES_DIR, SPUN, dressed, paddings};

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
    let (point, direction) = match element.geometry {
        CollisionGeometry::Mesh { .. } => {
            let hull = piece.hull();
            let (on_face, vertices) = point_on_face(tc, hull, drawn_index(tc, hull.faces()));
            let direction = hull
                .faces()
                .iter()
                .zip(hull.planes())
                .filter(|(face, _)| vertices.iter().all(|vertex| face.contains(vertex)))
                .fold(Vector3::zeros(), |sum, (_, (normal, _))| {
                    sum + normal * weight()
                });
            tc.assume(direction.norm() > 0.0);
            let center = hull.center();
            return (center + (on_face - center) * scale, direction.normalize());
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
            (point, Vector3::from_fn(|i, _| sides[i] * weight()))
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
            let outward = if on_side {
                radial * weight()
            } else {
                Vector3::zeros()
            };
            (
                radial * across + Vector3::z() * along,
                outward + Vector3::z() * (end * weight()),
            )
        }
        CollisionGeometry::Sphere { radius } => {
            let direction = tc.draw(unit_vectors());
            (direction * (radius * scale), direction)
        }
    };
    tc.assume(direction.norm() > 0.0);
    (
        element.origin * Point3::from(point),
        element.origin.rotation * direction.normalize(),
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

/// A point beyond the rim of a scaled cylinder by 0.8 of the offset along its radius and
/// along its axis lies 1.13 offsets from the cylinder, outside the grown link.
#[test]
fn a_point_past_the_rim_of_a_scaled_cylinder_is_outside_it() {
    let padding = one_link_padding();
    let geometry = one_link(r#"<cylinder radius="0.05" length="0.2"/>"#);
    let step = 0.8 * padding.offset();
    let point = Point3::new(
        0.05 * padding.scale() + step,
        0.0,
        0.1 * padding.scale() + step,
    );
    assert!(
        !geometry.mask(padding).at(&[0.0]).contains(&point),
        "{point} lies 1.13 offsets from the scaled cylinder, padding {padding:?}, and \
         inside the grown link"
    );
}

/// A point beyond a corner of a scaled box by 0.7 of the offset along each axis lies 1.21
/// offsets from the box, outside the grown link.
#[test]
fn a_point_past_the_corner_of_a_scaled_box_is_outside_it() {
    let padding = one_link_padding();
    let geometry = one_link(r#"<box size="0.1 0.2 0.3"/>"#);
    let corner = Vector3::new(0.05, 0.1, 0.15) * padding.scale();
    let point = Point3::from(corner.add_scalar(0.7 * padding.offset()));
    assert!(
        !geometry.mask(padding).at(&[0.0]).contains(&point),
        "{point} lies 1.21 offsets from the scaled box, padding {padding:?}, and inside \
         the grown link"
    );
}

/// Each vertex of the scaled piece of a mesh, moved out by half the offset along the
/// normal of each face through it, lies half the offset from the piece, the vertex its
/// nearest point, and inside the grown link. The mesh is a box, two faces to a side, so
/// a vertex that one face of a side holds and the other lacks, moved along the side's
/// normal, lies as high above the other face, which lies farther than the offset from it.
#[test]
fn a_vertex_of_a_scaled_mesh_moved_out_by_half_the_offset_is_inside_it() {
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
                grown.contains(&point),
                "{point} lies half the offset from the scaled piece, padding {padding:?}, \
                 and outside the grown link"
            );
        }
    }
}
