//! The collision pieces of a chain of any shape: which links the chain carries, where
//! they stand, what each piece holds, and which points each grown link holds.

use chain_kinematics::{Chain, ChainSpec, JointSelection};
use collision_geometry::chain::{ChainGeometry, LinkGeometry, Padding, Piece};
use collision_geometry::gjk::Hull;
use collision_geometry::nalgebra::{Isometry3, Point3, Vector3};
use collision_geometry::urdf_collision::{Collision, CollisionGeometry, UrdfCollisions};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{
    ChainProperty, TIP, TOLERANCE, angles, configs, joint_name, link_name, on_a_drawn_robot,
    unit_vectors,
};
use urdf_rs::Robot;

mod common;
use common::growth::grown_primitive_excess;
use common::properties::{MESHES_DIR, SPUN, WEDGE, dressed, paddings};

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

/// A drawn point of the face `face` of `hull`, in the hull's frame.
fn point_on_face(tc: &TestCase, hull: &Hull, face: usize) -> Point3<f64> {
    let weights: Vec<f64> = (0..3)
        .map(|_| tc.draw(gs::floats().min_value(0.0).max_value(1.0)))
        .collect();
    let total: f64 = weights.iter().sum();
    tc.assume(total > 0.0);
    Point3::from(
        hull.faces()[face]
            .iter()
            .zip(&weights)
            .fold(Vector3::zeros(), |sum, (&vertex, weight)| {
                sum + hull.vertices()[vertex].coords * (weight / total)
            }),
    )
}

/// How far `padding` moves a face of `hull` out along its normal, at most: `scale - 1`
/// times the face's distance from the center of the hull's vertices, plus the offset.
/// The face moves through its first vertex `v`, grown away from the center `c` to
/// `scale + offset / |v - c|` times its distance from it, and the face lies no farther
/// from `c` than `v` does.
fn face_move_bound(hull: &Hull, face: usize, padding: Padding) -> f64 {
    let (normal, offset) = hull.planes()[face];
    let from_center = offset - normal.dot(&hull.center().coords);
    (padding.scale() - 1.0) * from_center + padding.offset()
}

/// Whether `point`, in the frame of its link, lies outside the collision element of
/// `piece` grown by `padding`, by more than [`TOLERANCE`]: for a mesh, farther beyond a
/// face of the piece than `padding` moves that face; for a primitive, outside the
/// primitive grown about its origin.
fn beyond_the_padding(piece: &Piece, padding: Padding, point: &Point3<f64>) -> bool {
    match grown_primitive_excess(piece.element(), padding, point) {
        Some(excess) => excess > TOLERANCE,
        None => {
            let hull = piece.hull();
            hull.planes()
                .iter()
                .enumerate()
                .any(|(face, (normal, plane))| {
                    normal.dot(&point.coords) - plane
                        > face_move_bound(hull, face, padding) + TOLERANCE
                })
        }
    }
}

/// A drawn point on the surface of the primitive `element` grown by `padding`, about its
/// origin, with the surface's outward unit normal there, both in the frame of its link;
/// `None` for a mesh. On a box, a point of a face; on a cylinder, of its side or of a
/// cap; on a sphere, of any direction. A point of a face, of the side or of a cap lies
/// [`TOLERANCE`] or more inside its edges.
fn grown_primitive_surface(
    tc: &TestCase,
    element: &Collision,
    padding: Padding,
) -> Option<(Point3<f64>, Vector3<f64>)> {
    let (scale, offset) = (padding.scale(), padding.offset());
    let fraction = || tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
    let sign = || if tc.draw(gs::booleans()) { 1.0 } else { -1.0 };
    let (point, normal) = match element.geometry {
        CollisionGeometry::Mesh { .. } => return None,
        CollisionGeometry::Box { size } => {
            let axis = tc.draw(gs::integers::<usize>().min_value(0).max_value(2));
            let side = sign();
            let point = Vector3::from_fn(|i, _| {
                if i == axis {
                    side * (size[i] * scale / 2.0 + offset)
                } else {
                    fraction() * (size[i] * scale / 2.0 + offset - TOLERANCE)
                }
            });
            (Point3::from(point), Vector3::ith(axis, side))
        }
        CollisionGeometry::Cylinder { radius, length } => {
            let (sin, cos) = tc.draw(angles()).sin_cos();
            if tc.draw(gs::booleans()) {
                let radial = Vector3::new(cos, sin, 0.0);
                let along =
                    Vector3::z() * (fraction() * (length * scale / 2.0 + offset - TOLERANCE));
                (
                    Point3::from(radial * (radius * scale + offset) + along),
                    radial,
                )
            } else {
                let end = sign();
                let across = (radius * scale + offset - TOLERANCE) * fraction().abs();
                let point = Point3::new(
                    across * cos,
                    across * sin,
                    end * (length * scale / 2.0 + offset),
                );
                (point, Vector3::z() * end)
            }
        }
        CollisionGeometry::Sphere { radius } => {
            let direction = tc.draw(unit_vectors());
            (
                Point3::from(direction * (radius * scale + offset)),
                direction,
            )
        }
    };
    Some((element.origin * point, element.origin.rotation * normal))
}

/// The point beyond `on_face`, a point of a face of the mesh piece `piece`, on the ray
/// from the center `c` of the piece's vertices, at `t` times the distance of `on_face`
/// from `c`: `t` is `1 + fraction` of the way to `min(scale + offset / R, (1 + scale +
/// offset / e) / 2)`, with `R` the farthest vertex's distance from `c` and `e` the
/// largest half extent of the piece's box along the axes of its element's frame. Each
/// grown face lies at least `scale + offset / R` times as far from `c` as the face, and
/// the grown box holds the box scaled by that bound about any point of it, so the point
/// lies inside the grown piece.
fn padded_point(
    piece: &Piece,
    padding: Padding,
    on_face: Point3<f64>,
    fraction: f64,
) -> Point3<f64> {
    let hull = piece.hull();
    let center = hull.center();
    let farthest = hull
        .vertices()
        .iter()
        .map(|v| (v - center).norm())
        .fold(0.0, f64::max);
    let link_to_element = piece.element().origin.inverse();
    let (lower, upper) = hull.vertices().iter().map(|v| link_to_element * v).fold(
        (
            Vector3::repeat(f64::INFINITY),
            Vector3::repeat(f64::NEG_INFINITY),
        ),
        |(lower, upper), v| (lower.inf(&v.coords), upper.sup(&v.coords)),
    );
    let largest_half_extent = ((upper - lower) / 2.0).max();
    let (scale, offset) = (padding.scale(), padding.offset());
    let widest =
        (scale + offset / farthest).min((1.0 + scale + offset / largest_half_extent) / 2.0);
    let t = 1.0 + (widest - 1.0) * fraction;
    center + (on_face - center) * t
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

/// A point between a collision element and the element grown by the padding lies inside
/// its grown link, and inside the grown chain. For a mesh, the point lies beyond a drawn
/// face of its piece, where [`padded_point`] puts it; for a primitive, [`TOLERANCE`]
/// inside a drawn point of the surface of the grown primitive.
#[hegel::test]
fn a_point_between_an_element_and_its_padding_is_inside_its_link(tc: TestCase) {
    struct PaddedPointIsInside;
    impl ChainProperty for PaddedPointIsInside {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let urdf = dressed_urdf::<N>(tc, robot);
            let geometry = geometry_of::<N>(&urdf);
            let q = tc.draw(configs(geometry.chain().limits()));
            let padding = tc.draw(paddings());
            let placed = geometry.place(&q);
            let links: Vec<(&LinkGeometry, &Isometry3<f64>)> = placed.links().collect();
            let index = drawn_index(tc, &links);
            let (link, pose) = links[index];
            let piece = &link.pieces()[drawn_index(tc, link.pieces())];
            let local = match grown_primitive_surface(tc, piece.element(), padding) {
                Some((surface, normal)) => surface - normal * TOLERANCE,
                None => {
                    let hull = piece.hull();
                    let on_face = point_on_face(tc, hull, drawn_index(tc, hull.faces()));
                    let fraction = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
                    padded_point(piece, padding, on_face, fraction)
                }
            };
            let point = pose * local;
            let mask = geometry.mask(padding);
            let grown = mask.at(&q);
            assert!(
                grown.links()[index].contains(&point),
                "{point} lies between an element of {} and its padding {padding:?}, and outside the grown link",
                link.name()
            );
            assert!(
                grown.contains(&point),
                "{point} lies outside the grown chain"
            );
        }
    }
    on_a_drawn_robot::<PaddedPointIsInside>(&tc);
}

/// The wedge whose vertices' mean lies off the center of its box, scaled to a square of
/// side 1 cm at x = 0 and a point at x = 0.1 m, grown by a scale of 1.1 and an offset of
/// 1 cm, on which the inside property once drew a point beyond the grown box: every
/// point that [`padded_point`] puts beyond a corner or the centroid of a face of its
/// piece lies inside its grown link.
#[test]
fn the_padded_points_of_a_wedge_lie_inside_its_link() {
    let urdf = format!(
        r#"<robot name="wedge">
          <link name="base"/>
          <link name="arm">
            <collision><geometry>
              <mesh filename="package://drawn/meshes/{WEDGE}" scale="0.1 0.1 0.1"/>
            </geometry></collision>
          </link>
          <joint name="turn" type="revolute">
            <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>"#
    );
    let geometry = ChainGeometry::<1>::from_urdf(
        &urdf,
        meshes_dir(),
        &ChainSpec {
            base_link: None,
            tip_link: "arm",
            joints: JointSelection::PathOrder,
        },
    )
    .expect("the wedge fits");
    let padding = Padding::new(1.1, 0.01).expect("a valid padding");
    let mask = geometry.mask(padding);
    let grown = mask.at(&[0.0]);
    let piece = &geometry.links()[0].pieces()[0];
    for face in piece.hull().faces() {
        let corners = face.map(|vertex| piece.hull().vertices()[vertex]);
        let centroid = Point3::from(corners.iter().map(|c| c.coords).sum::<Vector3<f64>>() / 3.0);
        for on_face in corners.into_iter().chain([centroid]) {
            let point = padded_point(piece, padding, on_face, 1.0);
            assert!(
                grown.links()[0].contains(&point),
                "{point}, padded from {on_face}, lies outside the grown wedge"
            );
        }
    }
}

/// The box of a mesh grows along the axes of its element's frame, as MoveIt aligns a
/// mesh's box: on the wedge of [`the_padded_points_of_a_wedge_lie_inside_its_link`],
/// turned 45 degrees about z by its origin, a point on the wedge's axis 1 mm inside the
/// end of its grown box lies inside its grown link, and one 1 mm beyond it, which a box
/// along the axes of the link's frame would hold, lies outside.
#[test]
fn the_box_of_a_turned_wedge_grows_along_its_element() {
    let urdf = format!(
        r#"<robot name="wedge">
          <link name="base"/>
          <link name="arm">
            <collision><origin xyz="0 0 0" rpy="0 0 {}"/><geometry>
              <mesh filename="package://drawn/meshes/{WEDGE}" scale="0.1 0.1 0.1"/>
            </geometry></collision>
          </link>
          <joint name="turn" type="revolute">
            <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>"#,
        std::f64::consts::FRAC_PI_4
    );
    let geometry = ChainGeometry::<1>::from_urdf(
        &urdf,
        meshes_dir(),
        &ChainSpec {
            base_link: None,
            tip_link: "arm",
            joints: JointSelection::PathOrder,
        },
    )
    .expect("the wedge fits");
    let padding = Padding::new(1.1, 0.01).expect("a valid padding");
    let grown = geometry.mask(padding);
    let grown = grown.at(&[0.0]);
    let piece = &geometry.links()[0].pieces()[0];
    let origin = piece.element().origin;
    let (lower, upper) = piece
        .hull()
        .vertices()
        .iter()
        .map(|v| (origin.inverse() * v).x)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lower, upper), x| {
            (lower.min(x), upper.max(x))
        });
    let end = (lower + upper) / 2.0 + (upper - lower) / 2.0 * padding.scale() + padding.offset();
    let on_axis = |x: f64| origin * Point3::new(x, 0.0, 0.0);
    assert!(
        grown.links()[0].contains(&on_axis(end - 0.001)),
        "1 mm inside the end of the grown box"
    );
    assert!(
        !grown.links()[0].contains(&on_axis(end + 0.001)),
        "1 mm beyond the end of the grown box"
    );
}

/// A point farther outside every collision element than the padding grows it lies
/// outside every grown link. The point lies beyond a drawn element: for a mesh, beyond a
/// drawn face of its piece by more than the padding moves the face; for a primitive,
/// beyond a drawn point of the surface of the grown primitive.
#[hegel::test]
fn a_point_beyond_the_padding_of_every_element_is_outside(tc: TestCase) {
    struct BeyondPaddingIsOutside;
    impl ChainProperty for BeyondPaddingIsOutside {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let urdf = dressed_urdf::<N>(tc, robot);
            let geometry = geometry_of::<N>(&urdf);
            let q = tc.draw(configs(geometry.chain().limits()));
            let padding = tc.draw(paddings());
            let placed = geometry.place(&q);
            let links: Vec<(&LinkGeometry, &Isometry3<f64>)> = placed.links().collect();
            let (link, pose) = links[drawn_index(tc, &links)];
            let piece = &link.pieces()[drawn_index(tc, link.pieces())];
            let beyond = tc.draw(gs::floats().min_value(2.0 * TOLERANCE).max_value(0.01));
            let local = match grown_primitive_surface(tc, piece.element(), padding) {
                Some((surface, normal)) => surface + normal * beyond,
                None => {
                    let hull = piece.hull();
                    let face = drawn_index(tc, hull.faces());
                    let (normal, _) = hull.planes()[face];
                    point_on_face(tc, hull, face)
                        + normal * (face_move_bound(hull, face, padding) + beyond)
                }
            };
            let point = pose * local;
            tc.assume(links.iter().all(|(link, pose)| {
                let local = pose.inverse() * point;
                link.pieces()
                    .iter()
                    .all(|piece| beyond_the_padding(piece, padding, &local))
            }));
            let mask = geometry.mask(padding);
            let grown = mask.at(&q);
            for link in grown.links() {
                assert!(
                    !link.contains(&point),
                    "{point} lies beyond the padding {padding:?} of every element, and inside {}",
                    link.name()
                );
            }
            assert!(
                !grown.contains(&point),
                "{point} lies outside every grown element, and inside the grown chain"
            );
        }
    }
    on_a_drawn_robot::<BeyondPaddingIsOutside>(&tc);
}
