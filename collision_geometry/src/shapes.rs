//! The shapes of collision objects: boxes, spheres, cylinders and closed triangle meshes,
//! placed in a frame, measured against a convex body, and tested for the points they
//! hold.
//!
//! A box, a sphere and a cylinder are convex: [`gjk::distance`] measures a convex body
//! against them through their support functions. A mesh is a closed solid, each of its
//! edges shared by two triangles, its triangles held in a bounding volume hierarchy as
//! FCL holds a mesh: a body's distance to it is the least over the triangles the
//! hierarchy cannot rule out, so a body passes through the open corner of an L-shaped
//! mesh, and a body wholly inside it lies at a negative distance, its distance to the
//! surface taken as a depth.

use std::collections::HashMap;

use nalgebra::{Isometry3, Point3, Vector3};

use crate::chain::Padding;
use crate::gjk::{self, Support, Vertex};

/// The most triangles a leaf of a mesh's hierarchy holds.
const LEAF_TRIANGLES: usize = 4;
/// The directions the inside test casts its rays along: none along an axis or a
/// diagonal, where the edges and the vertices of a modeled mesh line up.
const RAY_DIRECTIONS: [[f64; 3]; 3] = [
    [0.539_8, 0.621_1, 0.568_3],
    [-0.711_3, 0.312_7, 0.629_6],
    [0.208_9, -0.874_1, 0.438_5],
];
/// The least distance along a ray at which a crossing counts, and the least determinant
/// of a ray against a triangle's plane: below it the ray runs along the plane.
const RAY_EPS: f64 = 1e-12;

/// Why a shape cannot be built.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum ShapeError {
    #[error("a {shape}'s {dimension} is {value}; it is a finite number more than 0")]
    Dimension {
        shape: &'static str,
        dimension: &'static str,
        value: f64,
    },
    #[error("a mesh has at least one triangle of three distinct corners")]
    EmptyMesh,
    #[error("triangle {triangle} names vertex {vertex}; the mesh has {vertices} vertices")]
    VertexIndex {
        triangle: usize,
        vertex: u32,
        vertices: usize,
    },
    #[error("vertex {0} of the mesh is not finite")]
    NonFinite(usize),
    #[error(
        "the mesh is open: the edge from vertex {from} to vertex {to} borders {triangles} \
         triangles; a mesh is a closed solid, each of its edges bordering two"
    )]
    OpenMesh {
        from: u32,
        to: u32,
        triangles: usize,
    },
}

/// A shape of a collision object, in its own frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Shape(Form);

#[derive(Debug, Clone, PartialEq)]
enum Form {
    Box { size: Vector3<f64> },
    Sphere { radius: f64 },
    Cylinder { radius: f64, height: f64 },
    Mesh(TriangleMesh),
}

/// A shape as its parts read.
#[derive(Debug, Clone, Copy)]
pub enum ShapeKind<'a> {
    /// A box about the origin, its sides along the axes, `size` long.
    Box { size: [f64; 3] },
    /// A sphere about the origin.
    Sphere { radius: f64 },
    /// A cylinder about the z axis, centered on the origin, `height` along it.
    Cylinder { radius: f64, height: f64 },
    /// A closed mesh of triangles, each three indices into `vertices`.
    Mesh {
        vertices: &'a [Point3<f64>],
        triangles: &'a [[u32; 3]],
    },
}

fn dimension(shape: &'static str, name: &'static str, value: f64) -> Result<f64, ShapeError> {
    if value.is_finite() && value > 0.0 {
        return Ok(value);
    }
    Err(ShapeError::Dimension {
        shape,
        dimension: name,
        value,
    })
}

impl Shape {
    /// A box about the origin, its sides along the axes, of the lengths `size`.
    pub fn cuboid(size: [f64; 3]) -> Result<Self, ShapeError> {
        let [x, y, z] = size;
        Ok(Shape(Form::Box {
            size: Vector3::new(
                dimension("box", "x side", x)?,
                dimension("box", "y side", y)?,
                dimension("box", "z side", z)?,
            ),
        }))
    }

    /// A sphere about the origin.
    pub fn sphere(radius: f64) -> Result<Self, ShapeError> {
        Ok(Shape(Form::Sphere {
            radius: dimension("sphere", "radius", radius)?,
        }))
    }

    /// A cylinder about the z axis, centered on the origin, `height` along it.
    pub fn cylinder(radius: f64, height: f64) -> Result<Self, ShapeError> {
        Ok(Shape(Form::Cylinder {
            radius: dimension("cylinder", "radius", radius)?,
            height: dimension("cylinder", "height", height)?,
        }))
    }

    /// A closed mesh of `triangles`, each three indices into `vertices`.
    pub fn mesh(vertices: Vec<Point3<f64>>, triangles: Vec<[u32; 3]>) -> Result<Self, ShapeError> {
        Ok(Shape(Form::Mesh(TriangleMesh::new(vertices, triangles)?)))
    }

    /// The shape's parts.
    pub fn kind(&self) -> ShapeKind<'_> {
        match &self.0 {
            Form::Box { size } => ShapeKind::Box {
                size: (*size).into(),
            },
            Form::Sphere { radius } => ShapeKind::Sphere { radius: *radius },
            Form::Cylinder { radius, height } => ShapeKind::Cylinder {
                radius: *radius,
                height: *height,
            },
            Form::Mesh(mesh) => ShapeKind::Mesh {
                vertices: &mesh.vertices,
                triangles: &mesh.triangles,
            },
        }
    }
}

/// A shape placed by a pose.
#[derive(Debug, Clone, Copy)]
pub struct PlacedShape<'a> {
    shape: &'a Shape,
    pose: Isometry3<f64>,
}

impl<'a> PlacedShape<'a> {
    pub fn new(shape: &'a Shape, pose: Isometry3<f64>) -> Self {
        Self { shape, pose }
    }

    pub fn pose(&self) -> &Isometry3<f64> {
        &self.pose
    }

    /// The center and the radius of a sphere that holds the shape.
    pub fn bounding_sphere(&self) -> (Point3<f64>, f64) {
        let center = Point3::from(self.pose.translation.vector);
        match &self.shape.0 {
            Form::Box { size } => (center, size.norm() / 2.0),
            Form::Sphere { radius } => (center, *radius),
            Form::Cylinder { radius, height } => (center, radius.hypot(height / 2.0)),
            Form::Mesh(mesh) => (self.pose * mesh.center, mesh.radius),
        }
    }

    /// The least and the greatest corner of the smallest box along the frame's axes that
    /// holds the shape.
    pub fn bounds(&self) -> (Point3<f64>, Point3<f64>) {
        let center = Point3::from(self.pose.translation.vector);
        let rotation = self.pose.rotation.to_rotation_matrix();
        let half = match &self.shape.0 {
            Form::Box { size } => rotation.matrix().abs() * (size / 2.0),
            &Form::Sphere { radius } => Vector3::repeat(radius),
            &Form::Cylinder { radius, height } => {
                // Along an axis at a share `a` of the cylinder's own: its half height
                // times `a`, and its radius times the share across it.
                let axis = rotation * Vector3::z();
                axis.map(|a| a.abs() * height / 2.0 + radius * (1.0 - a * a).max(0.0).sqrt())
            }
            Form::Mesh(mesh) => {
                let placed: Vec<Point3<f64>> =
                    mesh.used_vertices().map(|v| self.pose * v).collect();
                let bounds = Bounds::of(&placed);
                return (bounds.min, bounds.max);
            }
        };
        (center - half, center + half)
    }

    /// The signed distance from the shape to a convex `body` that `sphere` holds:
    /// negative when they overlap, and when the body lies wholly inside a mesh.
    pub fn distance_to<C: Support>(&self, body: &C, sphere: (Point3<f64>, f64)) -> f64 {
        match &self.shape.0 {
            Form::Mesh(mesh) => {
                let local = InFrame {
                    body,
                    frame: self.pose,
                };
                let (center, radius) = sphere;
                mesh.distance_to(&local, (self.pose.inverse_transform_point(&center), radius))
            }
            form => gjk::distance(&Convex::new(form, self.pose), body).distance,
        }
    }

    /// The signed distance between this shape and `other`: negative when they overlap,
    /// and when one lies wholly inside the other.
    pub fn distance_to_shape(&self, other: &PlacedShape) -> f64 {
        if let Some(convex) = Convex::of(self) {
            return other.distance_to(&convex, self.bounding_sphere());
        }
        if let Some(convex) = Convex::of(other) {
            return self.distance_to(&convex, other.bounding_sphere());
        }
        let (Form::Mesh(mine), Form::Mesh(theirs)) = (&self.shape.0, &other.shape.0) else {
            unreachable!("a shape without a convex form is a mesh")
        };
        // Each of my triangles against the other mesh: negative where one lies inside it.
        let surfaces = (0..mine.triangles.len())
            .map(|t| {
                let triangle = mine.triangle(t).placed(&self.pose);
                other.distance_to(&triangle, triangle.bounding_sphere())
            })
            .fold(f64::INFINITY, f64::min);
        let one_of_theirs = other.pose * theirs.vertices[theirs.triangles[0][0] as usize];
        if surfaces > 0.0 && mine.inside(&self.pose.inverse_transform_point(&one_of_theirs)) {
            return -surfaces;
        }
        surfaces
    }

    /// Whether `point` lies inside the shape grown by `padding`, by the rule of a link's
    /// [`Mask`](crate::chain::Mask): within the offset of the shape scaled about its
    /// center. A box, a sphere and a cylinder scale about their origin; a mesh scales
    /// about the center of its bounds, and holds its inside and every point within the
    /// offset of a triangle. A point with a coordinate that is not a finite number lies
    /// inside no shape.
    pub fn contains(&self, point: &Point3<f64>, padding: Padding) -> bool {
        if !point.coords.iter().all(|c| c.is_finite()) {
            return false;
        }
        let p = self.pose.inverse_transform_point(point);
        let (scale, offset) = (padding.scale(), padding.offset());
        match &self.shape.0 {
            Form::Box { size } => distance_outside_box(&p, &(size * (scale / 2.0))) <= offset,
            Form::Sphere { radius } => p.coords.norm() - radius * scale <= offset,
            Form::Cylinder { radius, height } => {
                distance_outside_cylinder(&p, radius * scale, height * scale / 2.0) <= offset
            }
            Form::Mesh(mesh) => {
                let unscaled = mesh.center + (p - mesh.center) / scale;
                scale * mesh.distance_to(&Vertex(unscaled), (unscaled, 0.0)) <= offset
            }
        }
    }
}

/// How far `local`, in a box's frame, lies outside the box of `half_extents` about the
/// origin along the frame's axes: 0 inside.
pub(crate) fn distance_outside_box(local: &Point3<f64>, half_extents: &Vector3<f64>) -> f64 {
    (local.coords.abs() - half_extents)
        .sup(&Vector3::zeros())
        .norm()
}

/// How far `local`, in a cylinder's frame, lies outside the cylinder of `radius` about
/// the frame's z axis, `half_length` along it either way from the origin: 0 inside.
pub(crate) fn distance_outside_cylinder(local: &Point3<f64>, radius: f64, half_length: f64) -> f64 {
    let radial = (local.coords.xy().norm() - radius).max(0.0);
    let axial = (local.z.abs() - half_length).max(0.0);
    radial.hypot(axial)
}

/// A box, a sphere or a cylinder, placed, as GJK measures it: a sphere is its center
/// rounded by its radius.
struct Convex {
    form: ConvexForm,
    pose: Isometry3<f64>,
}

#[derive(Clone, Copy)]
enum ConvexForm {
    Box { half: Vector3<f64> },
    Sphere { radius: f64 },
    Cylinder { radius: f64, half_height: f64 },
}

impl Convex {
    /// The convex form of a shape that is not a mesh, placed.
    fn new(form: &Form, pose: Isometry3<f64>) -> Self {
        let form = match form {
            Form::Box { size } => ConvexForm::Box { half: size / 2.0 },
            Form::Sphere { radius } => ConvexForm::Sphere { radius: *radius },
            Form::Cylinder { radius, height } => ConvexForm::Cylinder {
                radius: *radius,
                half_height: height / 2.0,
            },
            Form::Mesh(_) => unreachable!("a mesh has no convex form"),
        };
        Self { form, pose }
    }

    /// The convex form of `shape`, unless it is a mesh.
    fn of(shape: &PlacedShape) -> Option<Self> {
        match &shape.shape.0 {
            Form::Mesh(_) => None,
            form => Some(Self::new(form, shape.pose)),
        }
    }
}

impl Support for Convex {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        let d = self.pose.inverse_transform_vector(dir);
        let local = match self.form {
            ConvexForm::Box { half } => {
                Point3::from(half.zip_map(&d, |h, d| if d < 0.0 { -h } else { h }))
            }
            ConvexForm::Sphere { .. } => Point3::origin(),
            ConvexForm::Cylinder {
                radius,
                half_height,
            } => {
                let across = d.x.hypot(d.y);
                let (x, y) = if across > 0.0 {
                    (radius * d.x / across, radius * d.y / across)
                } else {
                    (0.0, 0.0)
                };
                Point3::new(x, y, if d.z < 0.0 { -half_height } else { half_height })
            }
        };
        self.pose * local
    }

    fn radius(&self) -> f64 {
        match self.form {
            ConvexForm::Sphere { radius } => radius,
            ConvexForm::Box { .. } | ConvexForm::Cylinder { .. } => 0.0,
        }
    }
}

/// A body as seen from `frame`: its support taken in the world and brought into the
/// frame, so a mesh measures it in the mesh's own frame.
struct InFrame<'a, C> {
    body: &'a C,
    frame: Isometry3<f64>,
}

impl<C: Support> Support for InFrame<'_, C> {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        let world = self.body.core_support(&(self.frame.rotation * dir));
        self.frame.inverse_transform_point(&world)
    }

    fn radius(&self) -> f64 {
        self.body.radius()
    }
}

/// One triangle, as GJK measures it.
struct Triangle([Point3<f64>; 3]);

impl Triangle {
    fn placed(&self, pose: &Isometry3<f64>) -> Triangle {
        Triangle(self.0.map(|v| pose * v))
    }

    fn bounding_sphere(&self) -> (Point3<f64>, f64) {
        let center = Point3::from((self.0[0].coords + self.0[1].coords + self.0[2].coords) / 3.0);
        let radius = self
            .0
            .iter()
            .map(|v| (v - center).norm())
            .fold(0.0, f64::max);
        (center, radius)
    }
}

impl Support for Triangle {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        *self
            .0
            .iter()
            .max_by(|a, b| a.coords.dot(dir).total_cmp(&b.coords.dot(dir)))
            .expect("a triangle has three vertices")
    }
}

/// A closed mesh of triangles in a bounding volume hierarchy.
#[derive(Debug, Clone, PartialEq)]
struct TriangleMesh {
    vertices: Vec<Point3<f64>>,
    triangles: Vec<[u32; 3]>,
    /// The hierarchy's nodes, its root first.
    nodes: Vec<Node>,
    /// The center and the radius of a sphere that holds the vertices.
    center: Point3<f64>,
    radius: f64,
}

/// An axis-aligned box.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Bounds {
    min: Point3<f64>,
    max: Point3<f64>,
}

impl Bounds {
    fn of<'p>(points: impl IntoIterator<Item = &'p Point3<f64>>) -> Bounds {
        points.into_iter().fold(
            Bounds {
                min: Point3::from(Vector3::repeat(f64::INFINITY)),
                max: Point3::from(Vector3::repeat(f64::NEG_INFINITY)),
            },
            |b, p| Bounds {
                min: b.min.inf(p),
                max: b.max.sup(p),
            },
        )
    }

    fn distance_to(&self, p: &Point3<f64>) -> f64 {
        (self.min - p)
            .sup(&(p - self.max))
            .sup(&Vector3::zeros())
            .norm()
    }

    /// Whether the ray from `origin` along `dir` meets the box at a distance of 0 or more.
    fn hit_by(&self, origin: &Point3<f64>, dir: &Vector3<f64>) -> bool {
        let (mut near, mut far) = (0.0_f64, f64::INFINITY);
        for i in 0..3 {
            if dir[i] == 0.0 {
                if origin[i] < self.min[i] || origin[i] > self.max[i] {
                    return false;
                }
                continue;
            }
            let (a, b) = (
                (self.min[i] - origin[i]) / dir[i],
                (self.max[i] - origin[i]) / dir[i],
            );
            near = near.max(a.min(b));
            far = far.min(a.max(b));
        }
        near <= far
    }
}

#[derive(Debug, Clone, PartialEq)]
enum Node {
    Leaf {
        bounds: Bounds,
        triangles: Vec<u32>,
    },
    Inner {
        bounds: Bounds,
        left: usize,
        right: usize,
    },
}

impl Node {
    fn bounds(&self) -> &Bounds {
        match self {
            Node::Leaf { bounds, .. } | Node::Inner { bounds, .. } => bounds,
        }
    }
}

impl TriangleMesh {
    fn new(vertices: Vec<Point3<f64>>, triangles: Vec<[u32; 3]>) -> Result<Self, ShapeError> {
        if let Some(index) = vertices
            .iter()
            .position(|v| !v.coords.iter().all(|c| c.is_finite()))
        {
            return Err(ShapeError::NonFinite(index));
        }
        for (triangle, corners) in triangles.iter().enumerate() {
            if let Some(&vertex) = corners.iter().find(|&&v| v as usize >= vertices.len()) {
                return Err(ShapeError::VertexIndex {
                    triangle,
                    vertex,
                    vertices: vertices.len(),
                });
            }
        }
        refuse_open(&vertices, &triangles)?;
        let used = Bounds::of(triangles.iter().flatten().map(|&v| &vertices[v as usize]));
        let center = nalgebra::center(&used.min, &used.max);
        let radius = triangles
            .iter()
            .flatten()
            .map(|&v| (vertices[v as usize] - center).norm())
            .fold(0.0, f64::max);
        let mut mesh = TriangleMesh {
            vertices,
            triangles,
            nodes: Vec::new(),
            center,
            radius,
        };
        let all: Vec<u32> = (0..mesh.triangles.len() as u32).collect();
        mesh.build(all);
        Ok(mesh)
    }

    fn triangle(&self, t: usize) -> Triangle {
        Triangle(self.triangles[t].map(|v| self.vertices[v as usize]))
    }

    /// Each vertex a triangle names, once per corner.
    fn used_vertices(&self) -> impl Iterator<Item = &Point3<f64>> {
        self.triangles
            .iter()
            .flatten()
            .map(|&v| &self.vertices[v as usize])
    }

    /// Adds the node of `triangles` and those below it, and answers its index.
    fn build(&mut self, mut triangles: Vec<u32>) -> usize {
        let bounds = Bounds::of(
            triangles
                .iter()
                .flat_map(|&t| self.triangles[t as usize])
                .map(|v| &self.vertices[v as usize]),
        );
        let index = self.nodes.len();
        if triangles.len() <= LEAF_TRIANGLES {
            self.nodes.push(Node::Leaf { bounds, triangles });
            return index;
        }
        let centroid = |t: u32| self.triangle(t as usize).bounding_sphere().0;
        let centroids: Vec<Point3<f64>> = triangles.iter().map(|&t| centroid(t)).collect();
        let spread = Bounds::of(&centroids);
        let extent = spread.max - spread.min;
        let axis = extent.imax();
        triangles.sort_by(|&a, &b| centroid(a)[axis].total_cmp(&centroid(b)[axis]));
        let upper = triangles.split_off(triangles.len() / 2);
        self.nodes.push(Node::Leaf {
            bounds,
            triangles: Vec::new(),
        });
        let left = self.build(triangles);
        let right = self.build(upper);
        self.nodes[index] = Node::Inner {
            bounds,
            left,
            right,
        };
        index
    }

    /// The least signed distance from `body`, in the mesh's frame, to the mesh's
    /// triangles; `sphere` holds the body. A body wholly inside the mesh answers its
    /// distance to the surface, negated.
    fn distance_to<C: Support>(&self, body: &C, sphere: (Point3<f64>, f64)) -> f64 {
        let (center, radius) = sphere;
        let mut nearest = f64::INFINITY;
        let mut stack = vec![0];
        while let Some(index) = stack.pop() {
            let node = &self.nodes[index];
            if node.bounds().distance_to(&center) - radius >= nearest {
                continue;
            }
            match node {
                Node::Leaf { triangles, .. } => {
                    for &t in triangles {
                        nearest =
                            nearest.min(gjk::distance(body, &self.triangle(t as usize)).distance);
                    }
                }
                Node::Inner { left, right, .. } => stack.extend([*left, *right]),
            }
        }
        if nearest > 0.0 && self.inside(&body.core_support(&Vector3::x())) {
            return -nearest;
        }
        nearest
    }

    /// Whether `point`, in the mesh's frame, lies inside the mesh: most of the inside
    /// test's rays cross the surface an odd number of times.
    fn inside(&self, point: &Point3<f64>) -> bool {
        let odd = RAY_DIRECTIONS
            .iter()
            .filter(|dir| self.crossings(point, &Vector3::from(**dir).normalize()) % 2 == 1)
            .count();
        odd * 2 > RAY_DIRECTIONS.len()
    }

    /// How many triangles the ray from `origin` along `dir` crosses.
    fn crossings(&self, origin: &Point3<f64>, dir: &Vector3<f64>) -> usize {
        let mut count = 0;
        let mut stack = vec![0];
        while let Some(index) = stack.pop() {
            let node = &self.nodes[index];
            if !node.bounds().hit_by(origin, dir) {
                continue;
            }
            match node {
                Node::Leaf { triangles, .. } => {
                    count += triangles
                        .iter()
                        .filter(|&&t| crosses(origin, dir, &self.triangle(t as usize)))
                        .count();
                }
                Node::Inner { left, right, .. } => stack.extend([*left, *right]),
            }
        }
        count
    }
}

/// Whether the ray from `origin` along `dir` crosses `triangle` (Moller and Trumbore).
fn crosses(origin: &Point3<f64>, dir: &Vector3<f64>, triangle: &Triangle) -> bool {
    let [a, b, c] = triangle.0;
    let (ab, ac) = (b - a, c - a);
    let p = dir.cross(&ac);
    let det = ab.dot(&p);
    if det.abs() < RAY_EPS {
        return false;
    }
    let to_origin = origin - a;
    let u = to_origin.dot(&p) / det;
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let q = to_origin.cross(&ab);
    let v = dir.dot(&q) / det;
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    ac.dot(&q) / det > RAY_EPS
}

/// Refuses a mesh that is not closed: one with an edge that does not border exactly two
/// of `triangles`, vertices at the same position joined, as a mesh read from STL repeats
/// them per triangle, or one with no triangle of three distinct corners. A triangle with
/// two corners at one position has no area and borders no edge.
fn refuse_open(vertices: &[Point3<f64>], triangles: &[[u32; 3]]) -> Result<(), ShapeError> {
    let mut joined: HashMap<[u64; 3], usize> = HashMap::new();
    let mut id = |v: u32| {
        let key = vertices[v as usize].coords.map(position_bits).into();
        let next = joined.len();
        *joined.entry(key).or_insert(next)
    };
    // Each edge of each triangle of three distinct corners: its vertex indices as the
    // triangle names them, and its corners joined.
    let edges: Vec<([u32; 2], (usize, usize))> = triangles
        .iter()
        .filter_map(|&corners| {
            let ids = corners.map(&mut id);
            let distinct = ids[0] != ids[1] && ids[1] != ids[2] && ids[0] != ids[2];
            distinct.then_some((corners, ids))
        })
        .flat_map(|(corners, ids)| {
            [(0, 1), (1, 2), (2, 0)].map(|(x, y)| {
                let joined = (ids[x].min(ids[y]), ids[x].max(ids[y]));
                ([corners[x], corners[y]], joined)
            })
        })
        .collect();
    if edges.is_empty() {
        return Err(ShapeError::EmptyMesh);
    }
    let mut borders: HashMap<(usize, usize), usize> = HashMap::new();
    for (_, joined) in &edges {
        *borders.entry(*joined).or_insert(0) += 1;
    }
    match edges.iter().find(|(_, joined)| borders[joined] != 2) {
        Some(([from, to], joined)) => Err(ShapeError::OpenMesh {
            from: *from,
            to: *to,
            triangles: borders[joined],
        }),
        None => Ok(()),
    }
}

/// The bits by which a vertex's coordinate joins the vertices at its position: -0.0
/// joins 0.0.
fn position_bits(coordinate: f64) -> u64 {
    if coordinate == 0.0 {
        0.0_f64.to_bits()
    } else {
        coordinate.to_bits()
    }
}
