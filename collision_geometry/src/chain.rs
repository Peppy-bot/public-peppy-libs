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
//! one fixed to the root. Such a part below a link of the chain is a part the chain
//! carries ([`CarriedPart`]), placed with the joints between them at 0, which a
//! [`Rider`] places on the link that carries it. Placed at a configuration, each piece
//! gives its hull and its bounding sphere in the frame of the URDF root.
//!
//! A [`Mask`] grows every collision element by a [`Padding`]: the element scaled about
//! its center, a mesh from its piece and a primitive as itself, and every point within
//! the offset of the scaled element. It tests whether a point lies inside a grown link,
//! or inside any link of the grown chain. Placed, the chain holds the reach of each grown
//! piece in the frame of the URDF root, a ball that holds only points of the piece and
//! a ball that holds every point of it, and the box that holds every reach, so a point
//! screens only the pieces whose reach may hold it.
//!
//! A [`BodyGeometry`] holds the pieces of the links that fixed joints alone join to the
//! URDF root, the robot's body around its chains. Its [`BodyMask`] grows them by the same
//! rule, and stands where the body stands as a [`PlacedMask`].

use chain_kinematics::{Chain, ChainSpec, Posed};
use nalgebra::{Isometry3, Point3, Vector3};

use crate::GeometryError;
use crate::clip::ClipRegion;
use crate::fit::{fit_fixed_link, fit_pieces};
use crate::gjk::{self, Hull, Placed, Vertex};
use crate::hull::ConvexHull;
use crate::urdf_collision::{Collision, CollisionGeometry, ParentJoint, UrdfCollisions};

/// How far the outer reach of a grown piece reaches past the element's outer ball grown
/// by the offset, and the inner reach falls short of its inner ball grown by the offset,
/// in meters: far beyond the rounding of placing a point in the link's frame or a ball's
/// center in the root's, about 1e-15 m a meter out, so the outer reach holds every point
/// that the piece's screens hold, and the inner reach only points within the offset of
/// the element.
const REACH_MARGIN_M: f64 = 1e-9;

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

    /// The index, in a configuration, of the driven joint whose child link carries
    /// this link: the joints from it toward the base move it.
    pub fn segment(&self) -> usize {
        self.segment
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

/// A part of the robot that a link of a chain carries rigidly through a plan, such as a
/// gripper finger at the gripper's opening: its own link's name, the index of the driven
/// joint whose child link carries it, its frame in that link's frame, and its pieces in
/// its own frame.
pub struct Rider<'a> {
    pub link: &'a str,
    pub segment: usize,
    pub offset: Isometry3<f64>,
    pub pieces: &'a [Hull],
}

/// A link with collision elements below a link of a chain across a movable joint off the
/// chain's path, such as a gripper finger: its name, the index of the driven joint whose
/// child link carries it, its frame in that link's frame with every joint between them at
/// 0, and its pieces in its own frame, one per `<collision>` element as a link's.
#[derive(Debug)]
pub struct CarriedPart {
    link: String,
    segment: usize,
    offset: Isometry3<f64>,
    pieces: Vec<Hull>,
}

impl CarriedPart {
    /// The part as a rider of the link that carries it.
    pub fn rider(&self) -> Rider<'_> {
        Rider {
            link: &self.link,
            segment: self.segment,
            offset: self.offset,
            pieces: &self.pieces,
        }
    }
}

/// The collision pieces of a serial chain with `N` driven joints.
#[derive(Debug)]
pub struct ChainGeometry<const N: usize> {
    chain: Chain<N>,
    links: Vec<LinkGeometry>,
    carried: Vec<CarriedPart>,
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
        let mut carried = Vec::new();
        for name in collisions.collision_link_names() {
            if let Some((segment, offset)) = carrier(&collisions, &segments, &path, &name)? {
                links.push(LinkGeometry {
                    pieces: pieces_of(&collisions, &name, meshes_dir)?,
                    name,
                    segment,
                    offset,
                });
            } else if let Some((segment, offset)) = carrier_of_part(&collisions, &segments, &name)?
            {
                carried.push(CarriedPart {
                    pieces: pieces_of(&collisions, &name, meshes_dir)?
                        .into_iter()
                        .map(|piece| piece.hull)
                        .collect(),
                    link: name,
                    segment,
                    offset,
                });
            }
        }
        Ok(ChainGeometry {
            chain,
            links,
            carried,
        })
    }

    /// The geometry with the lower limit of joint `i` raised to at least `floor`, as
    /// [`Chain::with_lower_floor`] raises it: a control margin over the URDF's limit,
    /// which every reader of the chain's limits then keeps.
    pub fn with_lower_floor(self, i: usize, floor: f64) -> Self {
        Self {
            chain: self.chain.with_lower_floor(i, floor),
            ..self
        }
    }

    pub fn chain(&self) -> &Chain<N> {
        &self.chain
    }

    pub fn links(&self) -> &[LinkGeometry] {
        &self.links
    }

    /// The parts the chain's links carry off its path.
    pub fn carried(&self) -> &[CarriedPart] {
        &self.carried
    }

    /// Places every link at the configuration `q`.
    pub fn place(&self, q: &[f64; N]) -> PlacedChain<'_> {
        self.place_at(&self.chain.at(q))
    }

    /// Places every link where the chain posed at `posed` holds it.
    pub fn place_at(&self, posed: &Posed<N>) -> PlacedChain<'_> {
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

impl<'a> PlacedChain<'a> {
    /// Each link with its pose in the frame of the URDF root.
    pub fn links(&self) -> impl Iterator<Item = (&'a LinkGeometry, &Isometry3<f64>)> {
        self.links.iter().zip(&self.poses)
    }

    /// Every piece of every link, placed in the frame of the URDF root, with the name
    /// of its link. A piece borrows the chain's geometry alone.
    pub fn pieces(&self) -> impl Iterator<Item = (&'a str, Placed<'a>)> + '_ {
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
        PlacedMask::new(
            self.geometry
                .links
                .iter()
                .zip(placed.poses)
                .zip(&self.grown)
                .map(|((link, pose), pieces)| (link.name.as_str(), pieces.as_slice(), pose)),
        )
    }
}

/// The collision pieces of a robot's body: the links that fixed joints alone join to the
/// URDF root, which stand where they are whatever the robot's joints do.
#[derive(Debug)]
pub struct BodyGeometry {
    links: Vec<BodyLink>,
}

/// The pieces of one link of a body, and where the frame they stand in lies in the frame
/// of the URDF root.
#[derive(Debug)]
struct BodyLink {
    name: String,
    frame: Isometry3<f64>,
    pieces: BodyPieces,
}

impl BodyLink {
    /// The link `name` of a body, which stands at `pose` in the frame of the URDF root,
    /// with its pieces fitted from the meshes of `meshes_dir`: one per region of
    /// `regions`, and one per element without them.
    fn fitted(
        collisions: &UrdfCollisions,
        meshes_dir: &str,
        regions: Option<&[ClipRegion]>,
        name: String,
        pose: Isometry3<f64>,
    ) -> Result<Self, GeometryError> {
        let (frame, pieces) = match regions {
            None => (
                pose,
                BodyPieces::Elements(pieces_of(collisions, &name, meshes_dir)?),
            ),
            Some(regions) => (
                Isometry3::identity(),
                BodyPieces::Regions(fit_fixed_link(
                    collisions,
                    &name,
                    meshes_dir,
                    Some(regions),
                )?),
            ),
        };
        Ok(BodyLink {
            name,
            frame,
            pieces,
        })
    }
}

/// The pieces of a link of a body: one per `<collision>` element, in the link's frame;
/// or, for a link that clip regions decompose, one per region, in the frame of the URDF
/// root, each fitted to the region's slice of every element.
#[derive(Debug)]
enum BodyPieces {
    Elements(Vec<Piece>),
    Regions(Vec<Hull>),
}

impl BodyGeometry {
    /// Reads the body from the URDF and fits the pieces of each of its links from the
    /// meshes of `meshes_dir`. Each link that `regions` names is decomposed into its clip
    /// regions, each its `[min, max]` corners in the frame of the URDF root, meters, as
    /// the self model's builder takes a body's regions. Regions that name a link outside
    /// the body, or a link twice, are refused.
    pub fn from_urdf(
        urdf: &str,
        meshes_dir: &str,
        regions: &[(&str, &[[[f64; 3]; 2]])],
    ) -> Result<Self, GeometryError> {
        let collisions = UrdfCollisions::from_urdf(urdf)?;
        let regions = ClipRegion::by_body(regions)?;
        let links = collisions
            .collision_link_names()
            .into_iter()
            .filter_map(|name| {
                let pose = collisions.fixed_pose_in_root(&name).transpose()?;
                let link_regions = regions.get(&name).map(Vec::as_slice);
                Some(pose.map_err(GeometryError::from).and_then(|pose| {
                    BodyLink::fitted(&collisions, meshes_dir, link_regions, name, pose)
                }))
            })
            .collect::<Result<Vec<_>, _>>()?;
        if let Some(link) = regions
            .keys()
            .find(|link| !links.iter().any(|body| body.name == **link))
        {
            return Err(GeometryError::RegionsOffTheBody { body: link.clone() });
        }
        Ok(BodyGeometry { links })
    }

    /// The names of the body's links.
    pub fn link_names(&self) -> impl Iterator<Item = &str> {
        self.links.iter().map(|link| link.name.as_str())
    }

    /// The body's pieces grown by `padding`, to test points against: an element's piece
    /// as a chain's [`Mask`] grows it, and a region's as a mesh's piece.
    pub fn mask(&self, padding: Padding) -> BodyMask<'_> {
        BodyMask {
            links: self
                .links
                .iter()
                .map(|link| GrownLink {
                    name: &link.name,
                    frame: link.frame,
                    pieces: match &link.pieces {
                        BodyPieces::Elements(pieces) => pieces
                            .iter()
                            .map(|piece| GrownPiece::new(piece, padding))
                            .collect(),
                        BodyPieces::Regions(hulls) => hulls
                            .iter()
                            .map(|hull| GrownPiece::of_hull(hull, padding))
                            .collect(),
                    },
                })
                .collect(),
        }
    }
}

/// Pieces grown by a padding where they stand: a robot's body, or parts a caller places.
pub struct BodyMask<'a> {
    links: Vec<GrownLink<'a>>,
}

/// The grown pieces of a link of a body, and where the frame they stand in lies in the
/// frame of the URDF root.
struct GrownLink<'a> {
    name: &'a str,
    frame: Isometry3<f64>,
    pieces: Vec<GrownPiece>,
}

impl<'a> BodyMask<'a> {
    /// The parts `parts`, each its link's name, its pieces in its own frame and its pose
    /// in the frame of the URDF root, grown by `padding` as a mesh's piece grows.
    ///
    /// # Panics
    ///
    /// On a piece with a rounding: each piece is one that `fit_pieces` fits.
    pub fn of(
        parts: impl IntoIterator<Item = (&'a str, &'a [Hull], Isometry3<f64>)>,
        padding: Padding,
    ) -> Self {
        BodyMask {
            links: parts
                .into_iter()
                .map(|(name, hulls, frame)| GrownLink {
                    name,
                    frame,
                    pieces: hulls
                        .iter()
                        .map(|hull| GrownPiece::of_hull(hull, padding))
                        .collect(),
                })
                .collect(),
        }
    }

    /// The grown body where it stands, which answers for a point as a placed chain does.
    pub fn placed(&self) -> PlacedMask<'_> {
        PlacedMask::new(
            self.links
                .iter()
                .map(|link| (link.name, link.pieces.as_slice(), link.frame)),
        )
    }
}

/// Grown links where they stand: a grown chain placed at one configuration, or a grown
/// body.
#[derive(Debug)]
pub struct PlacedMask<'a> {
    /// Each grown link, in the order of the geometry's links.
    links: Vec<PlacedGrownLink<'a>>,
    /// The reach of each grown piece, link by link.
    reaches: Vec<Reach>,
    /// The box that holds every outer reach.
    bounds: ReachBox,
}

impl<'a> PlacedMask<'a> {
    /// The grown links `links`, each its name, its grown pieces and its pose in the frame
    /// of the URDF root, with the reach of each piece and the box that holds them.
    fn new(links: impl Iterator<Item = (&'a str, &'a [GrownPiece], Isometry3<f64>)>) -> Self {
        let posed: Vec<_> = links.collect();
        let reaches: Vec<Reach> = posed
            .iter()
            .enumerate()
            .flat_map(|(link, (_, pieces, pose))| {
                pieces
                    .iter()
                    .enumerate()
                    .map(move |(piece, grown)| Reach::of(grown, pose, link, piece))
            })
            .collect();
        let bounds = ReachBox::holding(&reaches);
        PlacedMask {
            links: posed
                .into_iter()
                .map(|(name, pieces, pose)| PlacedGrownLink {
                    name,
                    pieces,
                    root_in_link: pose.inverse(),
                })
                .collect(),
            reaches,
            bounds,
        }
    }

    /// Each grown link where it stands, in the order of the geometry's links.
    pub fn links(&self) -> &[PlacedGrownLink<'_>] {
        &self.links
    }

    /// Whether `point`, in the frame of the URDF root, lies inside a grown link: it
    /// answers as [`PlacedGrownLink::contains`] of each link. A point outside the box of
    /// the reaches lies inside none, a point within the inner reach of a piece lies
    /// inside it, and every other point is screened by the pieces whose outer reach
    /// holds it.
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        self.bounds.holds(point)
            && (self.reaches.iter().any(|reach| reach.surely_holds(point))
                || any_holds(|| {
                    self.reaches
                        .iter()
                        .filter(|reach| reach.may_hold(point))
                        .map(|reach| {
                            let link = &self.links[reach.link];
                            (&link.pieces[reach.piece], link.root_in_link * point)
                        })
                }))
    }
}

/// The reach of a grown piece, two balls about the center of its scaled element in the
/// frame of the URDF root: the inner reach, the largest ball that the element holds grown
/// by the offset less [`REACH_MARGIN_M`], holds only points the piece holds, and the
/// outer reach, the smallest ball that holds the element grown by the offset and
/// [`REACH_MARGIN_M`], holds every point the piece holds.
#[derive(Debug)]
struct Reach {
    center: Point3<f64>,
    inner_squared: f64,
    outer_squared: f64,
    /// The index of the piece's link in the placed chain, and of the piece in its link.
    link: usize,
    piece: usize,
}

impl Reach {
    /// The reach of `grown`, the piece of index `piece` of the link of index `link`, whose
    /// link stands at `pose` in the frame of the URDF root.
    fn of(grown: &GrownPiece, pose: &Isometry3<f64>, link: usize, piece: usize) -> Self {
        let Balls {
            center,
            inner,
            outer,
        } = grown.balls();
        Reach {
            center: pose * center,
            inner_squared: (inner + grown.offset - REACH_MARGIN_M).max(0.0).powi(2),
            outer_squared: (outer + grown.offset + REACH_MARGIN_M).powi(2),
            link,
            piece,
        }
    }

    /// Whether the inner reach holds `point`, so the piece holds it.
    fn surely_holds(&self, point: &Point3<f64>) -> bool {
        (point - self.center).norm_squared() <= self.inner_squared
    }

    /// Whether the outer reach holds `point`, so the piece's screens tell.
    fn may_hold(&self, point: &Point3<f64>) -> bool {
        (point - self.center).norm_squared() <= self.outer_squared
    }
}

/// A box square to the axes of the URDF root's frame.
#[derive(Debug)]
struct ReachBox {
    min: Vector3<f64>,
    max: Vector3<f64>,
}

impl ReachBox {
    /// The smallest box that holds the outer reach of every one of `reaches`; it holds no
    /// point when there is none.
    fn holding(reaches: &[Reach]) -> Self {
        reaches.iter().fold(
            ReachBox {
                min: Vector3::repeat(f64::INFINITY),
                max: Vector3::repeat(f64::NEG_INFINITY),
            },
            |bounds, reach| {
                let radius = Vector3::repeat(reach.outer_squared.sqrt());
                ReachBox {
                    min: bounds.min.inf(&(reach.center.coords - radius)),
                    max: bounds.max.sup(&(reach.center.coords + radius)),
                }
            },
        )
    }

    fn holds(&self, point: &Point3<f64>) -> bool {
        (0..3).all(|axis| self.min[axis] <= point[axis] && point[axis] <= self.max[axis])
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
    /// link. A point with a coordinate that is not a finite number lies inside none.
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        any_holds(|| self.pieces_about(point))
    }

    /// Each grown piece of the link, with `point`, in the frame of the URDF root, in the
    /// link's frame: the plain rule, every piece screened.
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

/// Two balls about the center of a scaled element: the largest that the element holds,
/// of radius `inner`, and the smallest that holds it, of radius `outer`.
struct Balls {
    center: Point3<f64>,
    inner: f64,
    outer: f64,
}

/// A collision element scaled about its center, in its link's frame.
#[derive(Debug)]
enum Scaled {
    /// A mesh's piece, scaled about the mean of its vertices, and the radius of the
    /// largest ball about that mean that it holds: the least height of the mean below a
    /// face.
    Mesh { hull: Hull, inscribed: f64 },
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
    /// The two balls of the scaled element about its center, in its link's frame. A
    /// mesh's center is its scaled piece's, and its outer ball the piece's bounding
    /// sphere; a box, a cylinder and a sphere stand about the element's origin, and their
    /// outer balls reach their corners, their rims and their surface.
    fn balls(&self) -> Balls {
        let center_of = |link_to_element: &Isometry3<f64>| {
            Point3::from(link_to_element.inverse().translation.vector)
        };
        match &self.scaled {
            Scaled::Mesh { hull, inscribed } => Balls {
                center: hull.center(),
                inner: *inscribed,
                outer: hull.bound_radius(),
            },
            Scaled::Box {
                link_to_box,
                half_extents,
            } => Balls {
                center: center_of(link_to_box),
                inner: half_extents.min(),
                outer: half_extents.norm(),
            },
            Scaled::Cylinder {
                link_to_cylinder,
                radius,
                half_length,
            } => Balls {
                center: center_of(link_to_cylinder),
                inner: radius.min(*half_length),
                outer: radius.hypot(*half_length),
            },
            Scaled::Sphere { center, radius } => Balls {
                center: *center,
                inner: *radius,
                outer: *radius,
            },
        }
    }

    fn new(piece: &Piece, padding: Padding) -> Self {
        let Padding { scale, offset } = padding;
        let element = &piece.element;
        let scaled = match element.geometry {
            CollisionGeometry::Mesh { .. } => return GrownPiece::of_hull(&piece.hull, padding),
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

    /// The piece `hull`, a fitted piece with no rounding, grown by `padding` as a mesh's
    /// piece grows: scaled about the mean of its vertices, with the largest ball about
    /// that mean that it holds.
    fn of_hull(hull: &Hull, padding: Padding) -> Self {
        let hull = scaled_hull(hull, padding.scale);
        let inscribed = hull
            .planes()
            .iter()
            .map(|(normal, plane)| plane - normal.dot(&hull.center().coords))
            .fold(f64::INFINITY, f64::min);
        GrownPiece {
            scaled: Scaled::Mesh { hull, inscribed },
            offset: padding.offset,
        }
    }

    /// What the screens tell of `point`, in the link's frame. A point with a coordinate
    /// that is not a finite number lies outside. A primitive's screen is its distance
    /// from the scaled primitive, which answers for every other point. A point farther
    /// than the offset from a mesh's bounding sphere, or beyond a face moved out by the
    /// offset, lies outside, and a point inside every face lies inside. A point between
    /// the two lies inside when it lies within the offset of the face it lies highest
    /// above, a part of the piece; every other point lies in the shell.
    fn screen(&self, point: &Point3<f64>) -> Screen<'_> {
        if !point.coords.iter().all(|c| c.is_finite()) {
            return Screen::Outside;
        }
        let offset = self.offset;
        let within = |distance: f64| {
            if distance <= offset {
                Screen::Inside
            } else {
                Screen::Outside
            }
        };
        match &self.scaled {
            Scaled::Mesh { hull, .. } => screen_a_hull(hull, offset, point),
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

/// The pieces of `link`, one per `<collision>` element, each fitted to the element's
/// surface in the link's frame, the meshes read from `meshes_dir`.
fn pieces_of(
    collisions: &UrdfCollisions,
    link: &str,
    meshes_dir: &str,
) -> Result<Vec<Piece>, GeometryError> {
    collisions
        .collisions_of(link)
        .into_iter()
        .map(|element| {
            let surface = element.surface(meshes_dir)?;
            let [hull] = <[Hull; 1]>::try_from(fit_pieces(link, &[surface], None)?)
                .expect("a fit without clip regions gives one piece per surface");
            Ok(Piece {
                hull,
                element: element.clone(),
            })
        })
        .collect()
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
    walk_up(collisions, segments, link, |joint, below| {
        joint.is_fixed() || path.iter().any(|on_path| on_path == below)
    })
}

/// The driven joint whose child link carries `link`, a part below a link of the chain
/// that `carrier` leaves out, with `link`'s frame in the frame of that child link, every
/// joint between them at 0; `None` when no driven joint moves `link`.
fn carrier_of_part(
    collisions: &UrdfCollisions,
    segments: &[String],
    link: &str,
) -> Result<Option<(usize, Isometry3<f64>)>, GeometryError> {
    walk_up(collisions, segments, link, |_, _| true)
}

/// The walk up from `link` to the child link of a driven joint, of `segments`, crossing
/// each joint that `crosses` takes, given the joint and the link below it, at 0: that
/// joint's index and `link`'s frame in its child link's frame; `None` where the walk
/// reaches the root or a joint it does not cross.
fn walk_up(
    collisions: &UrdfCollisions,
    segments: &[String],
    link: &str,
    crosses: impl Fn(&ParentJoint, &str) -> bool,
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
        if !crosses(joint, &current) {
            return Ok(None);
        }
        let motion = joint
            .offset(0.0)
            .map_err(|e| format!("the joint above '{current}': {e}"))?;
        offset = motion * offset;
        current = joint.parent_link.clone();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A chain of two revolute joints about z, the second 30 cm out, whose first link
    /// holds no collision element and carries a camera on a fixed joint and a finger on a
    /// revolute joint off the chain's path.
    const FINGER_ON_A_BARE_LINK: &str = r#"<robot name="finger_on_a_bare_link">
        <link name="base"/>
        <link name="link_1"/>
        <link name="link_2">
          <collision><geometry><box size="0.1 0.04 0.04"/></geometry></collision>
        </link>
        <link name="camera">
          <collision><geometry><box size="0.03 0.03 0.03"/></geometry></collision>
        </link>
        <link name="finger">
          <collision><geometry><box size="0.02 0.02 0.04"/></geometry></collision>
        </link>
        <joint name="joint_1" type="revolute">
          <parent link="base"/><child link="link_1"/><axis xyz="0 0 1"/>
          <limit lower="-3" upper="3" effort="1" velocity="1"/>
        </joint>
        <joint name="joint_2" type="revolute">
          <origin xyz="0.3 0 0"/>
          <parent link="link_1"/><child link="link_2"/><axis xyz="0 0 1"/>
          <limit lower="-3" upper="3" effort="1" velocity="1"/>
        </joint>
        <joint name="camera_mount" type="fixed">
          <origin xyz="0 0 0.1"/>
          <parent link="link_1"/><child link="camera"/>
        </joint>
        <joint name="finger_joint" type="revolute">
          <origin xyz="0.1 0.05 0"/>
          <parent link="link_1"/><child link="finger"/><axis xyz="1 0 0"/>
          <limit lower="0" upper="1" effort="1" velocity="1"/>
        </joint>
      </robot>"#;

    // A finger off the path of a chain, on a link that holds no collision element, is a
    // part the chain carries: the first joint's link carries it at the finger joint's
    // origin, and placed with the chain it stands where the first joint turns it. The
    // camera fixed to that link is a link of the chain.
    #[test]
    fn a_part_off_the_path_rides_the_driven_link_above_it() {
        let geometry = ChainGeometry::<2>::from_urdf(
            FINGER_ON_A_BARE_LINK,
            "",
            &ChainSpec {
                base_link: None,
                tip_link: "link_2",
                joints: chain_kinematics::JointSelection::PathOrder,
            },
        )
        .expect("the chain fits");
        let names: Vec<&str> = geometry.links().iter().map(LinkGeometry::name).collect();
        assert_eq!(names, ["link_2", "camera"]);
        let [part] = geometry.carried() else {
            panic!("one carried part: {:?}", geometry.carried());
        };
        let rider = part.rider();
        assert_eq!((rider.link, rider.segment), ("finger", 0));
        assert_eq!(rider.offset, Isometry3::translation(0.1, 0.05, 0.0));
        let q = [0.7, -0.3];
        let placed = geometry.chain().at(&q).link_pose_world(rider.segment) * rider.offset;
        let turned =
            Isometry3::rotation(Vector3::z() * 0.7) * Isometry3::translation(0.1, 0.05, 0.0);
        assert!(
            (placed.to_homogeneous() - turned.to_homogeneous()).norm() < 1e-12,
            "{placed} stands off {turned}"
        );
    }

    /// A padding takes a positive finite scale and a non-negative finite offset, and
    /// refuses a scale or an offset that is NaN, infinite or out of that range.
    #[test]
    fn a_padding_refuses_a_scale_or_an_offset_out_of_its_range() {
        assert!(Padding::new(1.0, 0.0).is_ok());
        for (scale, offset) in [
            (f64::NAN, 0.0),
            (1.0, f64::NAN),
            (f64::INFINITY, 0.0),
            (1.0, f64::INFINITY),
            (f64::NEG_INFINITY, 0.0),
            (1.0, f64::NEG_INFINITY),
            (0.0, 0.0),
            (-1.0, 0.0),
            (1.0, -0.01),
        ] {
            assert!(
                matches!(
                    Padding::new(scale, offset),
                    Err(GeometryError::Padding { .. })
                ),
                "a padding of scale {scale} and offset {offset} is accepted"
            );
        }
    }
}
