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
//! A [`Mask`] grows every collision element as MoveIt's robot filter grows its body, by
//! a [`Padding`]: a mesh from its piece, a primitive as itself. It tests whether a point
//! lies inside a grown link, or inside any link of the grown chain.

use chain_kinematics::{Chain, ChainSpec};
use nalgebra::{Isometry3, Point3, Vector3};

use crate::GeometryError;
use crate::fit::fit_pieces;
use crate::gjk::{Hull, Placed};
use crate::urdf_collision::{Collision, CollisionGeometry, UrdfCollisions};

/// A point this far outside a face or the box of a grown mesh, in meters, counts as
/// inside it, as MoveIt's robot filter counts it for a face of a convex mesh. A piece's
/// own vertex then lies inside its grown box whichever way the box's center and extents
/// round.
const BOUNDARY_SLACK_M: f64 = 1e-9;

/// A vertex this close to the center of its piece, in meters, keeps its place when the
/// piece grows: it has no direction to move along. MoveIt's robot filter uses the same.
const CENTER_EPS_M: f64 = 1e-9;

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

/// How a [`Mask`] grows each collision element, as MoveIt's robot filter grows its
/// body. A mesh grows from its piece: each vertex moves away from the center of the
/// piece's vertices to `scale` times its distance from it, and `offset` meters
/// farther, and the piece's box along the axes of the mesh element's frame, as MoveIt
/// aligns a mesh's box, grows to `scale` times its size about its own center, and
/// `offset` meters farther on each side. A face moves out through its
/// grown first vertex: by `scale - 1` times its distance from the center, and by at most
/// `offset` more. Past a sharp edge, where two moved faces meet, the grown piece reaches
/// farther than its faces moved, as MoveIt's does. A primitive grows as itself, about
/// its element's origin: a box's half extents, a cylinder's radius and half length and
/// a sphere's radius each become `scale` times their own, plus `offset`.
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
        self.links.iter().any(|link| link.contains(point))
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
        let local = self.root_in_link * point;
        self.pieces.iter().any(|piece| piece.contains(&local))
    }
}

/// A collision element grown by a padding, in its link's frame, as MoveIt's robot filter
/// holds its body.
#[derive(Debug)]
enum GrownPiece {
    /// A mesh, grown from its piece as a convex mesh body: the piece's box, along the
    /// axes of the frame of the mesh's element, as MoveIt aligns a mesh's box, grown
    /// about its own center, and the planes of the piece's faces, each moved out through
    /// a grown vertex of the face. A point lies inside when it lies in both.
    Mesh {
        link_to_element: Isometry3<f64>,
        box_center: Point3<f64>,
        box_half_extents: Vector3<f64>,
        /// Each face's outward unit normal and its plane's offset along it.
        planes: Vec<(Vector3<f64>, f64)>,
    },
    /// A box, with its grown half extents along the axes of its frame.
    Box {
        link_to_box: Isometry3<f64>,
        half_extents: Vector3<f64>,
    },
    /// A cylinder about the z axis of its frame, with its grown radius and half length.
    Cylinder {
        link_to_cylinder: Isometry3<f64>,
        radius: f64,
        half_length: f64,
    },
    /// A sphere, with its grown radius.
    Sphere { center: Point3<f64>, radius: f64 },
}

impl GrownPiece {
    fn new(piece: &Piece, padding: Padding) -> Self {
        let Padding { scale, offset } = padding;
        let element = &piece.element;
        match element.geometry {
            CollisionGeometry::Mesh { .. } => Self::mesh(piece, padding),
            CollisionGeometry::Box { size } => GrownPiece::Box {
                link_to_box: element.origin.inverse(),
                half_extents: size * (scale / 2.0) + Vector3::repeat(offset),
            },
            CollisionGeometry::Cylinder { radius, length } => GrownPiece::Cylinder {
                link_to_cylinder: element.origin.inverse(),
                radius: radius * scale + offset,
                half_length: length * scale / 2.0 + offset,
            },
            CollisionGeometry::Sphere { radius } => GrownPiece::Sphere {
                center: Point3::from(element.origin.translation.vector),
                radius: radius * scale + offset,
            },
        }
    }

    fn mesh(piece: &Piece, padding: Padding) -> Self {
        let hull = &piece.hull;
        assert_eq!(hull.inflation(), 0.0, "a fitted piece carries no rounding");
        let vertices = hull.vertices();
        let link_to_element = piece.element.origin.inverse();
        let (lower, upper) = vertices.iter().map(|v| link_to_element * v).fold(
            (
                Point3::from(Vector3::repeat(f64::INFINITY)),
                Point3::from(Vector3::repeat(f64::NEG_INFINITY)),
            ),
            |(lower, upper), v| (lower.inf(&v), upper.sup(&v)),
        );
        let center = hull.center();
        let grown: Vec<Point3<f64>> = vertices
            .iter()
            .map(|v| {
                let away = v - center;
                let distance = away.norm();
                let reach = if distance > CENTER_EPS_M {
                    padding.offset / distance
                } else {
                    0.0
                };
                center + away * (padding.scale + reach)
            })
            .collect();
        let planes = hull
            .faces()
            .iter()
            .zip(hull.planes())
            .map(|(&[first, ..], &(normal, _))| (normal, normal.dot(&grown[first].coords)))
            .collect();
        GrownPiece::Mesh {
            link_to_element,
            box_center: nalgebra::center(&lower, &upper),
            box_half_extents: (upper - lower) * (padding.scale / 2.0)
                + Vector3::repeat(padding.offset),
            planes,
        }
    }

    fn contains(&self, point: &Point3<f64>) -> bool {
        match self {
            GrownPiece::Mesh {
                link_to_element,
                box_center,
                box_half_extents,
                planes,
            } => {
                let in_box = (link_to_element * point - box_center)
                    .abs()
                    .iter()
                    .zip(box_half_extents.iter())
                    .all(|(from_center, half_extent)| {
                        from_center - half_extent - BOUNDARY_SLACK_M <= 0.0
                    });
                in_box
                    && planes.iter().all(|(normal, offset)| {
                        normal.dot(&point.coords) - offset - BOUNDARY_SLACK_M <= 0.0
                    })
            }
            GrownPiece::Box {
                link_to_box,
                half_extents,
            } => (link_to_box * point)
                .coords
                .abs()
                .iter()
                .zip(half_extents.iter())
                .all(|(from_center, half_extent)| from_center <= half_extent),
            GrownPiece::Cylinder {
                link_to_cylinder,
                radius,
                half_length,
            } => {
                let local = link_to_cylinder * point;
                local.z.abs() <= *half_length
                    && local.x * local.x + local.y * local.y <= radius * radius
            }
            GrownPiece::Sphere { center, radius } => {
                (point - center).norm_squared() <= radius * radius
            }
        }
    }
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
