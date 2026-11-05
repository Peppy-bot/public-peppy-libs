//! The shapes of collision objects: a box mesh measures a body as the box does outside
//! it and holds the inside a box holds, two box meshes measure each other as their boxes
//! do, an L-shaped mesh leaves its open corner free and holds what lies inside its
//! walls, a shape grown by a padding holds the points that far from it and no farther
//! and a scaled shape the points its scale reaches, a point that is not finite lies in
//! no shape, a shape's bounds hold it, a mesh whose mirrored half meets the other at
//! -0.0 is closed, and a shape refuses what it cannot be, an open mesh among them.

use collision_geometry::chain::Padding;
use collision_geometry::gjk::Vertex;
use collision_geometry::nalgebra::{Isometry3, Point3, Vector3};
use collision_geometry::shapes::{PlacedShape, Shape, ShapeError, ShapeKind};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, poses, unit_vectors};

/// The padding that grows nothing.
fn none() -> Padding {
    Padding::new(1.0, 0.0).expect("the padding that grows nothing")
}

/// The box every test here measures: 0.4 by 0.2 by 0.1 m.
const SIZE: [f64; 3] = [0.4, 0.2, 0.1];

/// The box of `size` as a mesh of twelve triangles, each corner repeated per triangle
/// as an STL file holds it.
fn box_mesh(size: [f64; 3]) -> Shape {
    let h = Vector3::from(size) / 2.0;
    let corner = |i: usize| {
        Point3::new(
            if i & 1 == 0 { -h.x } else { h.x },
            if i & 2 == 0 { -h.y } else { h.y },
            if i & 4 == 0 { -h.z } else { h.z },
        )
    };
    // Two triangles per face, wound outward.
    let faces: [[usize; 4]; 6] = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for [a, b, c, d] in faces {
        for corners in [[a, b, c], [a, c, d]] {
            let base = vertices.len() as u32;
            vertices.extend(corners.map(corner));
            triangles.push([base, base + 1, base + 2]);
        }
    }
    Shape::mesh(vertices, triangles).expect("a box mesh")
}

/// The corners of the L every test here measures, counterclockwise seen from above: a
/// bar 1 m along x and one 1 m along y, each 0.2 m wide, meeting at the origin.
const L_CORNERS: [[f64; 2]; 6] = [
    [0.0, 0.0],
    [1.0, 0.0],
    [1.0, 0.2],
    [0.2, 0.2],
    [0.2, 1.0],
    [0.0, 1.0],
];

/// The L of `L_CORNERS` from z = 0 to z = 1, a closed solid: each cap four triangles,
/// each side two.
fn l_mesh() -> Shape {
    let vertices: Vec<Point3<f64>> = [0.0, 1.0]
        .iter()
        .flat_map(|&z| L_CORNERS.map(|[x, y]| Point3::new(x, y, z)))
        .collect();
    let (bottom, top) = (0_u32, 6_u32);
    let cap = [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5]];
    let caps = cap.iter().flat_map(|&[a, b, c]| {
        [
            [bottom + a, bottom + c, bottom + b],
            [top + a, top + b, top + c],
        ]
    });
    let sides = (0..6_u32).flat_map(|i| {
        let j = (i + 1) % 6;
        [
            [bottom + i, bottom + j, top + j],
            [bottom + i, top + j, top + i],
        ]
    });
    Shape::mesh(vertices, caps.chain(sides).collect()).expect("an L mesh")
}

/// A sphere's center drawn about the box's center, at most 1 m away along a drawn
/// direction, and a radius up to 5 cm.
fn sphere_about(tc: &TestCase, pose: &Isometry3<f64>) -> (Point3<f64>, f64) {
    let away = tc.draw(unit_vectors()) * tc.draw(gs::floats().min_value(0.0).max_value(1.0));
    (
        pose * Point3::from(away),
        tc.draw(gs::floats().min_value(0.001).max_value(0.05)),
    )
}

/// Whether the sphere at `center` of `radius` lies wholly inside the box at `pose`.
fn wholly_inside(pose: &Isometry3<f64>, center: Point3<f64>, radius: f64) -> bool {
    let local = pose.inverse_transform_point(&center);
    (0..3).all(|i| local[i].abs() + radius < SIZE[i] / 2.0)
}

/// Outside a box, its mesh measures a sphere as the box does; inside, both put it at a
/// negative distance.
#[hegel::test]
fn a_box_mesh_measures_a_sphere_as_the_box_does(tc: TestCase) {
    let pose = tc.draw(poses());
    let (cuboid, mesh) = (Shape::cuboid(SIZE).expect("a box"), box_mesh(SIZE));
    let (box_, mesh_) = (
        PlacedShape::new(&cuboid, pose),
        PlacedShape::new(&mesh, pose),
    );
    let (center, radius) = sphere_about(&tc, &pose);
    let ball_shape = Shape::sphere(radius).expect("a sphere");
    let ball = PlacedShape::new(
        &ball_shape,
        Isometry3::translation(center.x, center.y, center.z),
    );
    let (by_box, by_mesh) = (
        box_.distance_to_shape(&ball),
        mesh_.distance_to_shape(&ball),
    );
    if by_box > 0.0 {
        assert!(
            (by_box - by_mesh).abs() < TOLERANCE,
            "the box measures {by_box}, its mesh {by_mesh}"
        );
    } else if wholly_inside(&pose, center, radius) {
        assert!(
            by_mesh < 0.0,
            "a sphere wholly inside the box mesh lies at {by_mesh}"
        );
    }
}

/// Two box meshes apart measure each other, from either one, as their boxes do; two that
/// overlap lie at no positive distance.
#[hegel::test]
fn two_box_meshes_measure_each_other_as_their_boxes_do(tc: TestCase) {
    let (pose_a, pose_b) = (tc.draw(poses()), tc.draw(poses()));
    let small = [0.1, 0.3, 0.2];
    let (box_a, box_b) = (
        Shape::cuboid(SIZE).expect("a box"),
        Shape::cuboid(small).expect("a box"),
    );
    let (mesh_a, mesh_b) = (box_mesh(SIZE), box_mesh(small));
    let by_boxes =
        PlacedShape::new(&box_a, pose_a).distance_to_shape(&PlacedShape::new(&box_b, pose_b));
    let (a, b) = (
        PlacedShape::new(&mesh_a, pose_a),
        PlacedShape::new(&mesh_b, pose_b),
    );
    for (by_meshes, order) in [
        (a.distance_to_shape(&b), "a to b"),
        (b.distance_to_shape(&a), "b to a"),
    ] {
        if by_boxes > 0.0 {
            assert!(
                (by_meshes - by_boxes).abs() < TOLERANCE,
                "{order}: the boxes lie {by_boxes} apart, their meshes {by_meshes}"
            );
        } else {
            assert!(
                by_meshes < TOLERANCE,
                "{order}: the boxes overlap, and their meshes lie {by_meshes} apart"
            );
        }
    }
}

/// A box mesh wholly inside another lies at a negative distance from it, from either
/// one: its surface's distance to the other's, negated.
#[test]
fn a_box_mesh_inside_another_lies_at_a_negative_distance() {
    let (outer, inner) = (box_mesh(SIZE), box_mesh([0.1, 0.1, 0.02]));
    let (outer, inner) = (
        PlacedShape::new(&outer, Isometry3::identity()),
        PlacedShape::new(&inner, Isometry3::identity()),
    );
    // The inner box's faces lie 4 cm from the outer's along z, the nearest pair.
    for depth in [
        outer.distance_to_shape(&inner),
        inner.distance_to_shape(&outer),
    ] {
        assert!(
            (depth + 0.04).abs() < TOLERANCE,
            "a box mesh wholly inside another lies at {depth}"
        );
    }
}

/// A box mesh holds the points the box holds, read from either side of its surface.
#[hegel::test]
fn a_box_mesh_holds_the_inside_of_the_box(tc: TestCase) {
    let pose = tc.draw(poses());
    let (cuboid, mesh) = (Shape::cuboid(SIZE).expect("a box"), box_mesh(SIZE));
    let (box_, mesh_) = (
        PlacedShape::new(&cuboid, pose),
        PlacedShape::new(&mesh, pose),
    );
    let local = Point3::from(Vector3::from(SIZE).component_mul(&Vector3::new(
        tc.draw(gs::floats().min_value(-0.7).max_value(0.7)),
        tc.draw(gs::floats().min_value(-0.7).max_value(0.7)),
        tc.draw(gs::floats().min_value(-0.7).max_value(0.7)),
    )));
    let h = Vector3::from(SIZE) / 2.0;
    // Points within the tolerance of a face are left to the surface's rounding.
    tc.assume((0..3).all(|i| (local[i].abs() - h[i]).abs() > TOLERANCE));
    let point = pose * local;
    assert_eq!(
        mesh_.contains(&point, none()),
        box_.contains(&point, none()),
        "at {local}"
    );
}

/// A sphere in the open corner of the L clears both of its bars, though it lies inside
/// their convex hull; a sphere across a bar's face overlaps it, and one wholly inside a
/// bar lies at a negative distance.
#[test]
fn an_l_shaped_mesh_leaves_its_open_corner_free() {
    let l = l_mesh();
    let placed = PlacedShape::new(&l, Isometry3::identity());
    let ball = Shape::sphere(0.1).expect("a sphere");
    let at = |x: f64, y: f64| PlacedShape::new(&ball, Isometry3::translation(x, y, 0.5));
    let in_the_corner = placed.distance_to_shape(&at(0.5, 0.5));
    assert!(
        (in_the_corner - 0.2).abs() < TOLERANCE,
        "the ball in the corner clears the bars by {in_the_corner}"
    );
    assert!(placed.distance_to_shape(&at(0.5, 0.25)) < 0.0);
    let small = Shape::sphere(0.05).expect("a sphere");
    let inside = PlacedShape::new(&small, Isometry3::translation(0.6, 0.1, 0.5));
    let depth = placed.distance_to_shape(&inside);
    assert!(
        (depth + 0.05).abs() < TOLERANCE,
        "a ball wholly inside a bar lies at {depth}"
    );
    assert!(placed.contains(&Point3::new(0.1, 0.6, 0.5), none()));
    assert!(!placed.contains(&Point3::new(0.5, 0.5, 0.5), none()));
    assert!(placed.contains(&Point3::new(0.5, 0.21, 0.5), offset(0.02)));
}

/// The padding of `offset` meters and no scale.
fn offset(offset: f64) -> Padding {
    Padding::new(1.0, offset).expect("a finite offset of 0 or more")
}

/// A point at a distance from a primitive lies inside it grown by a padding of that
/// distance, and outside it grown by a smaller one: a padding grows a box and a cylinder
/// by rounding their edges, as a link's mask grows them.
#[hegel::test]
fn a_padding_holds_every_point_that_far_from_a_primitive(tc: TestCase) {
    let shapes = [
        Shape::cuboid(SIZE).expect("a box"),
        Shape::sphere(0.1).expect("a sphere"),
        Shape::cylinder(0.05, 0.3).expect("a cylinder"),
    ];
    let index = tc.draw(gs::integers::<usize>().min_value(0).max_value(2));
    let placed = PlacedShape::new(&shapes[index], tc.draw(poses()));
    let center = placed.pose() * Point3::origin();
    let point =
        center + tc.draw(unit_vectors()) * tc.draw(gs::floats().min_value(0.0).max_value(0.5));
    let distance = placed.distance_to(&Vertex(point), (point, 0.0));
    assert!(
        placed.contains(&point, offset(distance.max(0.0) + TOLERANCE)),
        "a point {distance} from the shape lies outside a padding of that"
    );
    if distance > TOLERANCE {
        assert!(
            !placed.contains(&point, offset(distance - TOLERANCE)),
            "a point {distance} from the shape lies inside a smaller padding"
        );
    }
}

/// A point with a coordinate that is not a finite number lies inside no shape, however
/// far a padding grows it.
#[test]
fn a_point_not_finite_lies_inside_no_shape() {
    let shapes = [
        Shape::cuboid(SIZE).expect("a box"),
        Shape::sphere(0.1).expect("a sphere"),
        Shape::cylinder(0.05, 0.3).expect("a cylinder"),
        box_mesh(SIZE),
    ];
    for shape in &shapes {
        let placed = PlacedShape::new(shape, Isometry3::identity());
        for coordinate in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let point = Point3::new(0.0, coordinate, 0.0);
            assert!(
                !placed.contains(&point, offset(1e6)),
                "{:?} holds {point}",
                shape.kind()
            );
        }
    }
}

/// A shape scaled about its center holds a point along each of its axes up to the scale
/// times its own reach and no farther: a box mesh's reach is its box's.
#[hegel::test]
fn a_scale_grows_a_shape_about_its_center(tc: TestCase) {
    let shapes = [
        Shape::cuboid(SIZE).expect("a box"),
        Shape::sphere(0.1).expect("a sphere"),
        Shape::cylinder(0.05, 0.3).expect("a cylinder"),
        box_mesh(SIZE),
    ];
    let index = tc.draw(gs::integers::<usize>().min_value(0).max_value(3));
    let scale = tc.draw(gs::floats().min_value(0.5).max_value(3.0));
    let padding = Padding::new(scale, 0.0).expect("a positive scale");
    let pose = tc.draw(poses());
    let placed = PlacedShape::new(&shapes[index], pose);
    let half = Vector3::from(SIZE) / 2.0;
    // The shape's reach along each axis of its own frame.
    let reach = match shapes[index].kind() {
        ShapeKind::Box { .. } | ShapeKind::Mesh { .. } => half,
        ShapeKind::Sphere { radius } => Vector3::repeat(radius),
        ShapeKind::Cylinder { radius, height } => Vector3::new(radius, radius, height / 2.0),
    };
    for (axis, extent) in reach.iter().enumerate() {
        let along = |share: f64| pose * Point3::from(Vector3::ith(axis, share * scale * extent));
        assert!(
            placed.contains(&along(0.99), padding),
            "axis {axis} at 0.99 of the scale"
        );
        assert!(
            !placed.contains(&along(1.01), padding),
            "axis {axis} at 1.01 of the scale"
        );
    }
}

/// A shape's bounds hold every point it holds, and touch it on each side: a box's and a
/// mesh's are the bounds of their corners, and a cylinder's rim reaches each side of
/// its bounds.
#[hegel::test]
fn a_shapes_bounds_hold_it(tc: TestCase) {
    let shapes = [
        Shape::cuboid(SIZE).expect("a box"),
        Shape::sphere(0.1).expect("a sphere"),
        Shape::cylinder(0.05, 0.3).expect("a cylinder"),
        l_mesh(),
    ];
    let index = tc.draw(gs::integers::<usize>().min_value(0).max_value(3));
    let placed = PlacedShape::new(&shapes[index], tc.draw(poses()));
    let (min, max) = placed.bounds();
    let (center, radius) = placed.bounding_sphere();
    let point =
        center + tc.draw(unit_vectors()) * tc.draw(gs::floats().min_value(0.0).max_value(radius));
    if placed.contains(&point, none()) {
        assert!(
            (0..3).all(|i| min[i] - TOLERANCE <= point[i] && point[i] <= max[i] + TOLERANCE),
            "{point} lies inside the shape and outside its bounds {min} to {max}"
        );
    }
    // The points that reach each side of the bounds, in the shape's frame, and how far
    // short of a side the nearest of them may fall.
    let (corners, slack): (Vec<Point3<f64>>, f64) = match shapes[index].kind() {
        ShapeKind::Box { size } => {
            let half = Vector3::from(size) / 2.0;
            let corner = |i: usize| {
                let sign = |bit: usize| if i & bit == 0 { -1.0 } else { 1.0 };
                Point3::new(sign(1) * half.x, sign(2) * half.y, sign(4) * half.z)
            };
            ((0..8).map(corner).collect(), TOLERANCE)
        }
        ShapeKind::Mesh { vertices, .. } => (vertices.to_vec(), TOLERANCE),
        // The rim, sampled every tenth of a degree: a side of the bounds lies past the
        // nearest sample by at most the radius times the cosine's fall over half that.
        ShapeKind::Cylinder { radius, height } => {
            let rim = (0..3600)
                .flat_map(|k| {
                    let turn = f64::from(k) * std::f64::consts::TAU / 3600.0;
                    [-height / 2.0, height / 2.0]
                        .map(|z| Point3::new(radius * turn.cos(), radius * turn.sin(), z))
                })
                .collect();
            let fall = radius * (1.0 - (std::f64::consts::TAU / 7200.0).cos());
            (rim, fall + TOLERANCE)
        }
        // The points of the sphere along each axis of the frame the bounds lie in.
        ShapeKind::Sphere { radius } => {
            let poles = [-1.0, 1.0]
                .iter()
                .flat_map(|&s| {
                    [Vector3::x(), Vector3::y(), Vector3::z()].map(|axis| {
                        let local = placed.pose().inverse_transform_vector(&axis);
                        Point3::from(local * s * radius)
                    })
                })
                .collect();
            (poles, TOLERANCE)
        }
    };
    let placed_corners: Vec<Point3<f64>> = corners.iter().map(|c| placed.pose() * c).collect();
    for i in 0..3 {
        let low = placed_corners
            .iter()
            .map(|c| c[i])
            .fold(f64::INFINITY, f64::min);
        let high = placed_corners
            .iter()
            .map(|c| c[i])
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (low - min[i]).abs() <= slack && (high - max[i]).abs() <= slack,
            "axis {i}: the shape spans {low} to {high}, its bounds {} to {}",
            min[i],
            max[i]
        );
    }
}

/// A shape refuses a dimension that is not a finite length more than 0, and a mesh with
/// no triangle of area, a corner past its vertices, a vertex that is not finite, or an
/// edge that does not border two triangles.
#[test]
fn a_shape_refuses_what_it_cannot_be() {
    assert_eq!(
        Shape::cuboid([0.1, 0.0, 0.1]),
        Err(ShapeError::Dimension {
            shape: "box",
            dimension: "y side",
            value: 0.0
        })
    );
    assert!(matches!(
        Shape::sphere(f64::NAN),
        Err(ShapeError::Dimension { .. })
    ));
    assert!(matches!(
        Shape::cylinder(0.1, -1.0),
        Err(ShapeError::Dimension { .. })
    ));
    assert_eq!(
        Shape::mesh(vec![Point3::origin()], Vec::new()),
        Err(ShapeError::EmptyMesh)
    );
    assert_eq!(
        Shape::mesh(vec![Point3::origin(); 3], vec![[0, 1, 2]]),
        Err(ShapeError::EmptyMesh)
    );
    assert_eq!(
        Shape::mesh(vec![Point3::origin(); 3], vec![[0, 1, 3]]),
        Err(ShapeError::VertexIndex {
            triangle: 0,
            vertex: 3,
            vertices: 3
        })
    );
    assert_eq!(
        Shape::mesh(
            vec![
                Point3::origin(),
                Point3::new(f64::INFINITY, 0.0, 0.0),
                Point3::origin()
            ],
            vec![[0, 1, 2]]
        ),
        Err(ShapeError::NonFinite(1))
    );
    // Two walls meeting along the z axis: an L of open sheets.
    let walls = Shape::mesh(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
        ],
        vec![[0, 1, 2], [0, 2, 3], [0, 4, 5], [0, 5, 3]],
    );
    assert_eq!(
        walls,
        Err(ShapeError::OpenMesh {
            from: 0,
            to: 1,
            triangles: 1
        })
    );
    assert!(
        matches!(
            Shape::cuboid(SIZE).expect("a box").kind(),
            ShapeKind::Box { size } if size == SIZE
        ),
        "a box reads back its sides"
    );
}

/// An octahedron whose half at x <= 0 mirrors its half at x >= 0, each corner repeated
/// per triangle as an STL file holds it: the mirror writes the corners on the plane
/// x = 0 with -0.0, which join the half's 0.0, so the mesh is closed and holds its
/// center.
#[test]
fn a_mirrored_closed_mesh_is_closed() {
    let rim = [
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, -1.0, 0.0),
        Point3::new(0.0, 0.0, -1.0),
    ];
    let tip = Point3::new(1.0, 0.0, 0.0);
    let half: Vec<[Point3<f64>; 3]> = (0..4).map(|i| [tip, rim[i], rim[(i + 1) % 4]]).collect();
    let mirror = |p: Point3<f64>| Point3::new(-p.x, p.y, p.z);
    let mirrored = half
        .iter()
        .map(|&[a, b, c]| [mirror(a), mirror(c), mirror(b)]);
    let corners: Vec<Point3<f64>> = half.iter().copied().chain(mirrored).flatten().collect();
    assert!(
        corners.iter().any(|p| p.x == 0.0 && p.x.is_sign_negative()),
        "the mirror writes -0.0"
    );
    let triangles = (0..corners.len() as u32 / 3)
        .map(|t| [3 * t, 3 * t + 1, 3 * t + 2])
        .collect();
    let octahedron = Shape::mesh(corners, triangles).expect("a closed mesh");
    assert!(
        PlacedShape::new(&octahedron, Isometry3::identity()).contains(&Point3::origin(), none())
    );
}
