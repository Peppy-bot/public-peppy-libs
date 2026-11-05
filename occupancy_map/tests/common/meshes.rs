//! Closed meshes about the origin for a collision object: a sphere whose vertices lie at
//! radii of the caller's choice, an L and a box with a box hollowed out of it.

use collision_geometry::shapes::Shape;
use nalgebra::Point3;

/// A sphere of `rings` rings of `around` vertices between its two poles, each vertex at
/// the distance from the center `radius` gives, the poles first.
pub fn sphere(rings: usize, around: usize, mut radius: impl FnMut() -> f64) -> Shape {
    let latitude = |i: usize| std::f64::consts::PI * i as f64 / (rings + 1) as f64;
    let longitude = |j: usize| std::f64::consts::TAU * j as f64 / around as f64;
    let north = Point3::new(0.0, 0.0, radius());
    let south = Point3::new(0.0, 0.0, -radius());
    let ring_vertices: Vec<Point3<f64>> = (1..=rings)
        .flat_map(|i| (0..around).map(move |j| (latitude(i), longitude(j))))
        .map(|(theta, phi)| {
            let r = radius();
            Point3::new(
                r * theta.sin() * phi.cos(),
                r * theta.sin() * phi.sin(),
                r * theta.cos(),
            )
        })
        .collect();
    let ring = |i: usize, j: usize| (2 + i * around + j % around) as u32;
    let caps = (0..around).flat_map(|j| {
        [
            [0, ring(0, j), ring(0, j + 1)],
            [1, ring(rings - 1, j + 1), ring(rings - 1, j)],
        ]
    });
    let bands = (0..rings - 1).flat_map(|i| {
        (0..around).flat_map(move |j| {
            [
                [ring(i, j), ring(i + 1, j), ring(i + 1, j + 1)],
                [ring(i, j), ring(i + 1, j + 1), ring(i, j + 1)],
            ]
        })
    });
    let vertices = [north, south].into_iter().chain(ring_vertices).collect();
    Shape::mesh(vertices, caps.chain(bands).collect()).expect("a closed sphere")
}

/// The corners of the L, counterclockwise seen from above: a bar along x and one along
/// y, each `side` long and a fifth of it wide, meeting at the origin.
const L_CORNERS: [[f64; 2]; 6] = [
    [0.0, 0.0],
    [1.0, 0.0],
    [1.0, 0.2],
    [0.2, 0.2],
    [0.2, 1.0],
    [0.0, 1.0],
];

/// The L of `L_CORNERS` scaled to `side`, from z = 0 to z = `side`: each cap four
/// triangles, each side two.
pub fn l_mesh(side: f64) -> Shape {
    let vertices: Vec<Point3<f64>> = [0.0, side]
        .iter()
        .flat_map(|&z| L_CORNERS.map(|[x, y]| Point3::new(x * side, y * side, z)))
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

/// The four corners of each face of a box, by the index of a corner whose bit `axis`
/// sets the upper side on that axis.
const BOX_FACES: [[u32; 4]; 6] = [
    [0, 2, 3, 1],
    [4, 5, 7, 6],
    [0, 1, 5, 4],
    [2, 6, 7, 3],
    [0, 4, 6, 2],
    [1, 3, 7, 5],
];

/// The eight corners of a box of `size` about the origin, ordered as `BOX_FACES` names
/// them.
fn box_corners(size: [f64; 3]) -> [Point3<f64>; 8] {
    std::array::from_fn(|corner| {
        Point3::from(std::array::from_fn(|axis| {
            let half = size[axis] / 2.0;
            if corner & (1 << axis) == 0 {
                -half
            } else {
                half
            }
        }))
    })
}

/// The twelve triangles of the box whose corners start at vertex `first`.
fn box_triangles(first: u32) -> impl Iterator<Item = [u32; 3]> {
    BOX_FACES.into_iter().flat_map(move |[a, b, c, d]| {
        [[a, b, c], [a, c, d]].map(|triangle| triangle.map(|corner| first + corner))
    })
}

/// A box of `size` about the origin.
pub fn box_mesh(size: [f64; 3]) -> Shape {
    Shape::mesh(box_corners(size).to_vec(), box_triangles(0).collect()).expect("a box mesh")
}

/// A box of `size` about the origin with a box of `share` of its size hollowed out of
/// its middle: two closed surfaces, one inside the other.
pub fn hollow_box(size: [f64; 3], share: f64) -> Shape {
    let vertices = box_corners(size)
        .into_iter()
        .chain(box_corners(size.map(|s| s * share)))
        .collect();
    let triangles = box_triangles(0).chain(box_triangles(8)).collect();
    Shape::mesh(vertices, triangles).expect("a hollow box")
}
