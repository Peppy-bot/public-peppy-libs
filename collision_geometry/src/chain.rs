//! The collision pieces of a serial chain, placed at a joint configuration.
//!
//! A [`ChainGeometry`] holds the convex pieces of every link whose pose the chain's
//! joints decide: the child link of each joint the chain drives, and every link below
//! one across fixed joints and across the movable joints of the root-to-tip path that
//! the chain does not drive, which stand at 0 as the chain holds them, down to the tip
//! and past it. A link has one piece per `<collision>` element, fitted to that element's
//! surface in the link's frame, each a [`Hull`] with its bounding sphere. A link below a
//! movable joint off that path that the chain does not drive, such as a gripper finger,
//! is not part of the chain, and neither is a link that no driven joint moves, such as
//! one fixed to the root. Placed at a configuration, each piece gives its hull and its
//! bounding sphere in the frame of the URDF root.
//!
//! A [`Mask`] grows every collision element by a [`Padding`]: the element scaled about
//! its center, a mesh from its piece and a primitive as itself, and every point within
//! the offset of the scaled element. It tests whether a point lies inside a grown link,
//! or inside any link of the grown chain.

use chain_kinematics::{Chain, ChainSpec};
use nalgebra::{Isometry3, Point3, Vector3};

use crate::GeometryError;
use crate::fit::fit_pieces;
use crate::gjk::{self, Hull, Placed, Vertex};
use crate::hull::ConvexHull;
use crate::urdf_collision::{Collision, CollisionGeometry, UrdfCollisions};

/// The pieces of one link of a chain.
#[derive(Debug)]
pub struct LinkGeometry {
    name: String,
    /// The index, in a configuration, of the driven joint whose child link carries
    /// this link.
    segment: usize,
    /// This link's frame in the frame of its segment's link.
    offset: Isometry3<f64>,
    /// The link's convex pieces, one per `<collision>` element.
    pieces: Vec<Piece>,
}

impl LinkGeometry {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn pieces(&self) -> &[Piece] {
        &self.pieces
    }
}

/// A convex piece of a link, in the link's frame, and the `<collision>` element it is
/// fitted to.
#[derive(Debug)]
pub struct Piece {
    hull: Hull,
    element: Collision,
}

impl Piece {
    pub fn hull(&self) -> &Hull {
        &self.hull
    }

    pub fn element(&self) -> &Collision {
        &self.element
    }
}

/// The collision pieces of a serial chain with `N` driven joints.
#[derive(Debug)]
pub struct ChainGeometry<const N: usize> {
    chain: Chain<N>,
    links: Vec<LinkGeometry>,
}

impl<const N: usize> ChainGeometry<N> {
    /// Reads the chain from the URDF and fits the pieces of each of its links, reading
    /// the collision meshes from `meshes_dir`, where the URDF names them by their file
    /// names.
    pub fn from_urdf(
        urdf: &str,
        meshes_dir: &str,
        spec: &ChainSpec<'_>,
    ) -> Result<Self, GeometryError> {
        let robot = urdf_rs::read_from_string(urdf).map_err(|e| format!("parse URDF: {e}"))?;
        let chain = Chain::<N>::from_urdf(&robot, spec)?;
        let collisions = UrdfCollisions::from_urdf(urdf)?;
        let posed = chain.at(&[0.0; N]);
        let segments: Vec<String> = (0..N).map(|i| posed.link_name(i).to_owned()).collect();
        let path = path_to(&collisions, spec.tip_link)?;
        let mut links = Vec::new();
        for name in collisions.collision_link_names() {
            let Some((segment, offset)) = carrier(&collisions, &segments, &path, &name)? else {
                continue;
            };
            let pieces = collisions
                .collisions_of(&name)
                .into_iter()
                .map(|element| {
                    let surface = element.surface(meshes_dir)?;
                    let [hull] = <[Hull; 1]>::try_from(fit_pieces(&name, &[surface], None)?)
                        .expect("a fit without clip regions gives one piece per surface");
                    Ok(Piece {
                        hull,
                        element: element.clone(),
                    })
                })
                .collect::<Result<_, GeometryError>>()?;
            links.push(LinkGeometry {
                name,
                segment,
                offset,
                pieces,
            });
        }
        Ok(ChainGeometry { chain, links })
    }

    pub fn chain(&self) -> &Chain<N> {
        &self.chain
    }

    pub fn links(&self) -> &[LinkGeometry] {
        &self.links
    }

    /// Places every link at the configuration `q`.
    pub fn place(&self, q: &[f64; N]) -> PlacedChain<'_> {
        let posed = self.chain.at(q);
        let poses = self
            .links
            .iter()
            .map(|link| posed.link_pose_world(link.segment) * link.offset)
            .collect();
        PlacedChain {
            links: &self.links,
            poses,
        }
    }

    /// The chain's pieces grown by `padding`, to test points against.
    pub fn mask(&self, padding: Padding) -> Mask<'_, N> {
        let grown = self
            .links
            .iter()
            .map(|link| {
                link.pieces
                    .iter()
                    .map(|piece| GrownPiece::new(piece, padding))
                    .collect()
            })
            .collect();
        Mask {
            geometry: self,
            grown,
        }
    }
}

/// The links of a chain, placed at one configuration.
#[derive(Debug)]
pub struct PlacedChain<'a> {
    links: &'a [LinkGeometry],
    /// The pose of each link in the frame of the URDF root, in the order of `links`.
    poses: Vec<Isometry3<f64>>,
}

impl PlacedChain<'_> {
    /// Each link with its pose in the frame of the URDF root.
    pub fn links(&self) -> impl Iterator<Item = (&LinkGeometry, &Isometry3<f64>)> {
        self.links.iter().zip(&self.poses)
    }

    /// Every piece of every link, placed in the frame of the URDF root, with the name
    /// of its link.
    pub fn pieces(&self) -> impl Iterator<Item = (&str, Placed<'_>)> {
        self.links().flat_map(|(link, pose)| {
            link.pieces
                .iter()
                .map(move |piece| (link.name(), Placed::new(&piece.hull, *pose)))
        })
    }
}

/// How a [`Mask`] grows each collision element: the element scaled by `scale` about its
/// center, and every point within `offset` meters of the scaled element. A mesh scales
/// from its piece, about the mean of the piece's vertices; a box, a cylinder and a sphere
/// scale as themselves, about their element's origin.
///
/// So a grown element holds every point within the offset of the scaled element, and no
/// point farther from it. A mesh's piece holds the convex hull of the mesh and lies
/// within its fit error of it, so at a scale of 1 a grown mesh holds every point within
/// the offset of the mesh's convex hull, and no point farther from that hull than the
/// offset plus the fit error.
///
/// MoveIt's robot filter grows a body by sharp planes; the crate's README states how
/// (The robot filter).
#[derive(Debug, Clone, Copy)]
pub struct Padding {
    scale: f64,
    offset: f64,
}

impl Padding {
    /// A padding of a positive finite `scale` and a non-negative finite `offset` in
    /// meters.
    pub fn new(scale: f64, offset: f64) -> Result<Self, GeometryError> {
        if scale.is_finite() && scale > 0.0 && offset.is_finite() && offset >= 0.0 {
            Ok(Padding { scale, offset })
        } else {
            Err(GeometryError::Padding { scale, offset })
        }
    }

    pub fn scale(self) -> f64 {
        self.scale
    }

    /// The offset, in meters.
    pub fn offset(self) -> f64 {
        self.offset
    }
}

/// The pieces of a chain grown by a padding.
#[derive(Debug)]
pub struct Mask<'a, const N: usize> {
    geometry: &'a ChainGeometry<N>,
    /// The grown pieces of each link, in the order of the geometry's links.
    grown: Vec<Vec<GrownPiece>>,
}

impl<const N: usize> Mask<'_, N> {
    /// Places the grown chain at the configuration `q`.
    pub fn at(&self, q: &[f64; N]) -> PlacedMask<'_> {
        let placed = self.geometry.place(q);
        let links = self
            .geometry
            .links
            .iter()
            .zip(&placed.poses)
            .zip(&self.grown)
            .map(|((link, pose), pieces)| PlacedGrownLink {
                name: &link.name,
                pieces,
                root_in_link: pose.inverse(),
            })
            .collect();
        PlacedMask { links }
    }
}

/// A grown chain, placed at one configuration.
#[derive(Debug)]
pub struct PlacedMask<'a> {
    /// Each grown link, in the order of the geometry's links.
    links: Vec<PlacedGrownLink<'a>>,
}

impl PlacedMask<'_> {
    /// Each link of the chain, grown and placed, in the order of the geometry's links.
    pub fn links(&self) -> &[PlacedGrownLink<'_>] {
        &self.links
    }

    /// Whether `point`, in the frame of the URDF root, lies inside a grown link.
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        any_holds(|| self.links.iter().flat_map(|link| link.pieces_about(point)))
    }
}

/// A link grown by a padding, placed at one configuration.
#[derive(Debug)]
pub struct PlacedGrownLink<'a> {
    name: &'a str,
    pieces: &'a [GrownPiece],
    /// The frame of the URDF root in the link's frame.
    root_in_link: Isometry3<f64>,
}

impl PlacedGrownLink<'_> {
    pub fn name(&self) -> &str {
        self.name
    }

    /// Whether `point`, in the frame of the URDF root, lies inside a grown piece of the
    /// link.
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        any_holds(|| self.pieces_about(point))
    }

    /// Each grown piece of the link, with `point`, in the frame of the URDF root, in the
    /// link's frame.
    fn pieces_about(
        &self,
        point: &Point3<f64>,
    ) -> impl Iterator<Item = (&GrownPiece, Point3<f64>)> {
        let local = self.root_in_link * point;
        self.pieces.iter().map(move |piece| (piece, local))
    }
}

/// Whether a grown piece of `pieces`, each with a point in its link's frame, holds its
/// point: first by the screens of every piece, then by the distance from each piece
/// whose screens leave its point in its shell, so a piece that a screen answers for
/// spares every distance.
fn any_holds<'a, I>(pieces: impl Fn() -> I) -> bool
where
    I: Iterator<Item = (&'a GrownPiece, Point3<f64>)>,
{
    let screened = pieces().try_fold(false, |shell, (piece, local)| match piece.screen(&local) {
        Screen::Inside => None,
        Screen::Shell(_) => Some(true),
        Screen::Outside => Some(shell),
    });
    match screened {
        None => true,
        Some(false) => false,
        Some(true) => pieces().any(|(piece, local)| match piece.screen(&local) {
            Screen::Shell(hull) => gjk::distance(&Vertex(local), hull).distance <= piece.offset,
            Screen::Inside | Screen::Outside => false,
        }),
    }
}

/// What the screens of a grown piece tell of a point.
enum Screen<'a> {
    /// The point lies within the offset of the scaled element.
    Inside,
    /// The point lies farther than the offset from the scaled element.
    Outside,
    /// The point lies beyond this scaled piece of a mesh, and within each of its faces
    /// moved out by the offset: its distance from the piece tells.
    Shell(&'a Hull),
}

/// A collision element grown by a padding, in its link's frame: every point within
/// `offset` meters of the element scaled about its center.
#[derive(Debug)]
struct GrownPiece {
    scaled: Scaled,
    offset: f64,
}

/// A collision element scaled about its center, in its link's frame.
#[derive(Debug)]
enum Scaled {
    /// A mesh's piece, scaled about the mean of its vertices.
    Mesh(Hull),
    /// A box, with its scaled half extents along the axes of its frame.
    Box {
        link_to_box: Isometry3<f64>,
        half_extents: Vector3<f64>,
    },
    /// A cylinder about the z axis of its frame, with its scaled radius and half length.
    Cylinder {
        link_to_cylinder: Isometry3<f64>,
        radius: f64,
        half_length: f64,
    },
    /// A sphere, with its scaled radius.
    Sphere { center: Point3<f64>, radius: f64 },
}

impl GrownPiece {
    fn new(piece: &Piece, padding: Padding) -> Self {
        let Padding { scale, offset } = padding;
        let element = &piece.element;
        let scaled = match element.geometry {
            CollisionGeometry::Mesh { .. } => Scaled::Mesh(scaled_hull(&piece.hull, scale)),
            CollisionGeometry::Box { size } => Scaled::Box {
                link_to_box: element.origin.inverse(),
                half_extents: size * (scale / 2.0),
            },
            CollisionGeometry::Cylinder { radius, length } => Scaled::Cylinder {
                link_to_cylinder: element.origin.inverse(),
                radius: radius * scale,
                half_length: length * scale / 2.0,
            },
            CollisionGeometry::Sphere { radius } => Scaled::Sphere {
                center: Point3::from(element.origin.translation.vector),
                radius: radius * scale,
            },
        };
        GrownPiece { scaled, offset }
    }

    /// What the screens tell of `point`, in the link's frame. A primitive's screen is
    /// its distance from the scaled primitive, which answers for every point. A point
    /// farther than the offset from a mesh's bounding sphere, or beyond a face moved out
    /// by the offset, lies outside, and a point inside every face lies inside. A point
    /// between the two lies inside when it lies within the offset of the face it lies
    /// highest above, a part of the piece; every other point lies in the shell.
    fn screen(&self, point: &Point3<f64>) -> Screen<'_> {
        let offset = self.offset;
        let within = |distance: f64| {
            if distance <= offset {
                Screen::Inside
            } else {
                Screen::Outside
            }
        };
        match &self.scaled {
            Scaled::Mesh(hull) => screen_a_hull(hull, offset, point),
            Scaled::Box {
                link_to_box,
                half_extents,
            } => within(
                ((link_to_box * point).coords.abs() - half_extents)
                    .sup(&Vector3::zeros())
                    .norm(),
            ),
            Scaled::Cylinder {
                link_to_cylinder,
                radius,
                half_length,
            } => {
                let local = link_to_cylinder * point;
                let radial = (local.coords.xy().norm() - radius).max(0.0);
                let axial = (local.z.abs() - half_length).max(0.0);
                within(radial.hypot(axial))
            }
            Scaled::Sphere { center, radius } => within((point - center).norm() - radius),
        }
    }
}

/// What the screens of the scaled piece `hull` of a mesh, grown by `offset`, tell of
/// `point`; [`GrownPiece::screen`] names them.
fn screen_a_hull<'a>(hull: &'a Hull, offset: f64, point: &Point3<f64>) -> Screen<'a> {
    let reach = hull.bound_radius() + offset;
    if (point - hull.center()).norm_squared() > reach * reach {
        return Screen::Outside;
    }
    let highest = hull.planes().iter().enumerate().try_fold(
        (f64::NEG_INFINITY, 0),
        |(highest, at), (face, (normal, plane))| {
            let height = normal.dot(&point.coords) - plane;
            (height <= offset).then_some(if height > highest {
                (height, face)
            } else {
                (highest, at)
            })
        },
    );
    let Some((height, face)) = highest else {
        return Screen::Outside;
    };
    let corners = hull.faces()[face].map(|vertex| hull.vertices()[vertex]);
    if height <= 0.0 || gjk::distance_to_triangle(point, corners) <= offset {
        Screen::Inside
    } else {
        Screen::Shell(hull)
    }
}

/// `hull` scaled by `scale` about the mean of its vertices.
fn scaled_hull(hull: &Hull, scale: f64) -> Hull {
    assert_eq!(hull.inflation(), 0.0, "a fitted piece carries no rounding");
    let center = hull.center();
    let scaled = ConvexHull {
        vertices: hull
            .vertices()
            .iter()
            .map(|v| center + (v - center) * scale)
            .collect(),
        faces: hull.faces().to_vec(),
        planes: hull
            .planes()
            .iter()
            .map(|&(normal, plane)| {
                let from_center = normal.dot(&center.coords);
                (normal, from_center + (plane - from_center) * scale)
            })
            .collect(),
    };
    Hull::new(&scaled, 0.0).expect("a scaled piece is a hull")
}

/// The links on the path from the URDF root to `tip`, the tip included.
fn path_to(collisions: &UrdfCollisions, tip: &str) -> Result<Vec<String>, GeometryError> {
    let mut path = vec![tip.to_owned()];
    while let Some(joint) =
        collisions.parent_joint(path.last().expect("the path starts at the tip"))
    {
        if path.contains(&joint.parent_link) {
            return Err(format!("the URDF holds a cycle through '{}'", joint.parent_link).into());
        }
        path.push(joint.parent_link.clone());
    }
    Ok(path)
}

/// The driven joint whose child link carries `link`, with `link`'s frame in the frame
/// of that child link; `None` when the chain's joints do not decide where `link` stands.
/// The walk up from `link` crosses fixed joints, and the movable joints of the path that
/// the chain does not drive, which stand at 0 as the chain holds them.
fn carrier(
    collisions: &UrdfCollisions,
    segments: &[String],
    path: &[String],
    link: &str,
) -> Result<Option<(usize, Isometry3<f64>)>, GeometryError> {
    let mut offset = Isometry3::identity();
    let mut current = link.to_owned();
    let mut visited = Vec::new();
    loop {
        if let Some(segment) = segments.iter().position(|s| *s == current) {
            return Ok(Some((segment, offset)));
        }
        if visited.contains(&current) {
            return Err(format!("the URDF holds a cycle through '{current}'").into());
        }
        visited.push(current.clone());
        let Some(joint) = collisions.parent_joint(&current) else {
            return Ok(None);
        };
        if !joint.is_fixed() && !path.contains(&current) {
            return Ok(None);
        }
        let motion = joint
            .offset(0.0)
            .map_err(|e| format!("the joint above '{current}': {e}"))?;
        offset = motion * offset;
        current = joint.parent_link.clone();
    }
}
