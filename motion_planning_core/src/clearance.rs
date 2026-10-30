//! Whether the arm at a configuration keeps its clearances. They form three terms, in
//! the order of every array of them:
//!
//! - The map and the objects, which stand still through a plan: each body of the arm, a
//!   piece of a link, a piece of a part a link carries or a shape of an object it
//!   carries, keeps the world clearance from every occupied voxel of the map and from
//!   each obstacle, an object of the scene where it stands or where another arm holds
//!   it; and each object the arm carries keeps it from each part of the robot the plan
//!   does not move.
//! - The robot's own body, where both bodies of a pair move: each pair of the self model
//!   keeps the own clearance.
//! - An attached object against the arm that carries it, where both bodies of a pair
//!   move: each object the arm carries keeps the held clearance from each piece of the
//!   arm's links and of the parts they carry, outside its touch links. A caller's
//!   clearance asks 0 of it (`term_clearances`): an attached object moves rigidly with its
//!   grasp link and keeps out of contact with the arm that carries it.
//!
//! Objects one arm carries are never checked against each other: they move rigidly
//! together.
//!
//! A pair allowed to touch keeps none; nor does an object the arm carries from its grasp
//! link and its touch links, or an obstacle another arm holds from that arm's grasp link
//! and its touch links.
//!
//! A body's bounding sphere screens first: against the map's distance field, and against
//! an obstacle's shape or a robot piece by the distance between their spheres. When the
//! sphere keeps the clearance, so does the body. A body whose sphere does not is
//! measured exactly: against the box of each occupied voxel near it, against the shape,
//! a mesh by its triangles, or against the piece's hull.

use collision_geometry::gjk::Placed;
use collision_geometry::shapes::PlacedShape;
use nalgebra::{Point3, Vector3};
use occupancy_map::{DistanceField, Key, VoxelMap};

use crate::arm::{Arm, Body};
use crate::scene::{OCCUPANCY_MAP, Objects};

/// What a state touches.
#[derive(Debug, PartialEq, Eq)]
pub enum Contact {
    /// A body of the arm, a link or what a link carries, and an occupied voxel of the
    /// map.
    Map { body: String },
    /// A body of the arm and an object of the scene the arm does not carry.
    Object { body: String, object: String },
    /// Two bodies of the robot: a pair its self model checks, or an object the arm
    /// carries and a link of the robot.
    Own { a: String, b: String },
}

impl std::fmt::Display for Contact {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Map { body } => write!(f, "{body} touches the occupancy map"),
            Self::Object { body, object } => write!(f, "{body} touches {object}"),
            Self::Own { a, b } => write!(f, "{a} touches {b}"),
        }
    }
}

/// A pair a state checks, and its distance as `Surroundings::readings` reads it.
#[derive(Debug, PartialEq)]
pub struct Reading {
    pub pair: Contact,
    /// The distance between the pair (m).
    pub distance: f64,
}

/// The pair and how far apart its bodies stand, as a message names them.
impl std::fmt::Display for Reading {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let distance = self.distance;
        match &self.pair {
            Contact::Map { body } => write!(f, "{body} {distance:.4} m from the map"),
            Contact::Object { body, object } => write!(f, "{body} {distance:.4} m from {object}"),
            Contact::Own { a, b } => write!(f, "{a} {distance:.4} m from {b}"),
        }
    }
}

/// The nearest of `readings`, if any.
fn nearest(readings: impl IntoIterator<Item = Reading>) -> Option<Reading> {
    readings
        .into_iter()
        .min_by(|a, b| a.distance.total_cmp(&b.distance))
}

/// How far past the least distance found a bound from bounding spheres, a body's screen
/// or a voxel's sphere, lies when it sets the body, the voxel or the pair aside: far
/// beyond the rounding of GJK's distance, some 1e-15 m, so what it sets aside lies no
/// nearer.
const SPHERE_BOUND_MARGIN_M: f64 = 1e-9;

/// The terms of the clearances, as the module names them.
pub const TERMS: usize = 3;
/// The place of each term in an array of them.
const WORLD: usize = 0;
const OWN: usize = 1;
const HELD: usize = 2;

/// The clearance a caller's `min_clearance_m` asks of each term: itself from the map and
/// the objects and within the robot's own body, and 0 between an attached object and the
/// links of the arm that carries it, outside its touch links.
pub fn term_clearances(min_clearance_m: f64) -> [f64; TERMS] {
    [min_clearance_m, min_clearance_m, 0.0]
}

/// What a state keeps clear of: the map and the objects as they stand, and the
/// clearances to keep.
pub struct Surroundings<'a> {
    map: &'a VoxelMap,
    field: &'a DistanceField,
    objects: Objects<'a>,
    /// The clearance of each term.
    clearances: [f64; TERMS],
}

impl<'a> Surroundings<'a> {
    /// The surroundings of a state on `map`, whose distances `field` holds, with no
    /// object, at which each pair of each term keeps the term's clearance of `clearances`,
    /// in meters.
    pub fn new(map: &'a VoxelMap, field: &'a DistanceField, clearances: [f64; TERMS]) -> Self {
        Self {
            map,
            field,
            objects: Objects::none(),
            clearances,
        }
    }

    /// The surroundings with the scene's `objects`.
    pub fn with_objects(self, objects: Objects<'a>) -> Self {
        Self { objects, ..self }
    }

    /// Whether `arm` at `q` keeps its clearances.
    pub fn clear<const N: usize>(&self, arm: &Arm<N>, q: &[f64; N]) -> bool {
        if own_distance(arm, q) < self.clearances[OWN] {
            return false;
        }
        let bodies = arm.bodies(q);
        self.on_the_map(&bodies)
            .all(|(_, body)| self.body_clear(body))
            && self
                .world_pairs(&bodies)
                .iter()
                .all(|pair| pair.keeps(self.clearances[WORLD]))
            && self
                .held_pairs(&bodies)
                .iter()
                .all(|pair| pair.keeps(self.clearances[HELD]))
    }

    /// The distance of `arm` at `q` from the bodies of each term, in meters: from the map
    /// and the objects, the least between the pairs of its self model, and the least
    /// between an attached object and the arm that carries it; each exact where it lies within
    /// its clearance, and a lower bound at or above the clearance elsewhere; infinite for a
    /// term that checks no pair. The bodies are measured against the map nearest screen
    /// first, and a body whose screen, a voxel whose bounding sphere, or a pair whose
    /// spheres' gap lies past the least distance found by `SPHERE_BOUND_MARGIN_M` is set
    /// aside: it lies no nearer.
    pub(crate) fn distances<const N: usize>(&self, arm: &Arm<N>, q: &[f64; N]) -> [f64; TERMS] {
        let bodies = arm.bodies(q);
        let world = self
            .world_pairs(&bodies)
            .iter()
            .fold(self.map_distance(&bodies), |least, pair| {
                pair.least_with(least, self.clearances[WORLD])
            });
        let held = self
            .held_pairs(&bodies)
            .iter()
            .fold(f64::INFINITY, |least, pair| {
                pair.least_with(least, self.clearances[HELD])
            });
        [world, own_distance(arm, q), held]
    }

    /// The pairs of each term that `arm` at `q` checks besides those of its self model, in
    /// the order of the terms, the robot's own body's, whose pairs its self model holds,
    /// empty; each as a contact names it, with its distance read against the clearance of
    /// its term: exact where it lies within the clearance, and a lower bound at or above
    /// the clearance elsewhere. A body of several pieces or shapes reads once per piece or
    /// shape, and each body reads the map once.
    fn readings_by_term<const N: usize>(
        &self,
        arm: &Arm<N>,
        q: &[f64; N],
    ) -> [Vec<Reading>; TERMS] {
        let bodies = arm.bodies(q);
        let on_the_map = self.on_the_map(&bodies).map(|(name, body)| Reading {
            pair: Contact::Map {
                body: name.to_string(),
            },
            distance: self.screened(body).map_or(f64::NEG_INFINITY, |screen| {
                self.body_distance_below(body, screen, f64::INFINITY)
            }),
        });
        let world = self.world_pairs(&bodies).into_iter().map(|pair| Reading {
            pair: pair.contact(),
            distance: pair.least_with(f64::INFINITY, self.clearances[WORLD]),
        });
        let held = self.held_pairs(&bodies).into_iter().map(|pair| Reading {
            pair: pair.contact(),
            distance: pair.least_with(f64::INFINITY, self.clearances[HELD]),
        });
        [
            on_the_map.chain(world).collect(),
            Vec::new(),
            held.collect(),
        ]
    }

    /// The nearest pair of each term of `arm` at `q`, read as `readings_by_term` reads it:
    /// a body of the arm and the map, an object or a part of the robot; a pair of its self
    /// model; and an object the arm carries and a link of the arm. None
    /// for a term that checks no pair.
    pub(crate) fn nearest_pairs<const N: usize>(
        &self,
        arm: &Arm<N>,
        q: &[f64; N],
    ) -> [Option<Reading>; TERMS] {
        let [world, _, held] = self.readings_by_term(arm, q);
        let modeled = arm.self_model().map(|model| {
            let (a, b) = model.nearest_pair(q);
            Reading {
                pair: Contact::Own { a, b },
                distance: own_distance(arm, q),
            }
        });
        [nearest(world), modeled, nearest(held)]
    }

    /// Each pair that `arm` at `q` checks besides those of its self model, read as
    /// `readings_by_term` reads it, the map's and the objects' first.
    pub fn readings<const N: usize>(&self, arm: &Arm<N>, q: &[f64; N]) -> Vec<Reading> {
        self.readings_by_term(arm, q)
            .into_iter()
            .flatten()
            .collect()
    }

    /// What `arm` at `q` touches, if anything: a pair of its self model before the map,
    /// the map before an object, and an object before a piece of the robot that an
    /// object the arm carries touches.
    pub fn contact<const N: usize>(&self, arm: &Arm<N>, q: &[f64; N]) -> Option<Contact> {
        let own = arm
            .self_model()
            .filter(|model| short_if_nan(model.clearance(q)) <= 0.0)
            .map(|model| {
                let (a, b) = model.nearest_pair(q);
                Contact::Own { a, b }
            });
        own.or_else(|| {
            let bodies = arm.bodies(q);
            let on_the_map = self
                .on_the_map(&bodies)
                .find(|(_, body)| {
                    self.near_voxels(body)
                        .any(|key| self.voxel_distance(body, key) <= 0.0)
                })
                .map(|(name, _)| Contact::Map {
                    body: name.to_string(),
                });
            on_the_map.or_else(|| {
                let world = self.world_pairs(&bodies);
                let held = self.held_pairs(&bodies);
                world
                    .iter()
                    .chain(&held)
                    .find(|pair| pair.touches())
                    .map(Pair::contact)
            })
        })
    }

    /// The bodies of `bodies` that keep a clearance from the map: each but an object
    /// allowed to touch it.
    fn on_the_map<'b>(
        &self,
        bodies: &'b [(&'b str, Body<'b>)],
    ) -> impl Iterator<Item = &'b (&'b str, Body<'b>)> {
        bodies
            .iter()
            .filter(|(name, _)| !self.objects.exempt(name, OCCUPANCY_MAP))
    }

    /// The pairs of the map and the objects besides the map's own: each body of `bodies`
    /// and each shape of each obstacle it may touch, and each object the arm carries and
    /// each piece of a part of the robot the plan does not move that it may touch.
    fn world_pairs<'p>(&'p self, bodies: &'p [(&'p str, Body<'p>)]) -> Vec<Pair<'p>> {
        let obstacles = bodies.iter().flat_map(|(name, body)| {
            self.objects
                .obstacles
                .iter()
                .filter(move |obstacle| {
                    !self.objects.exempt(name, obstacle.id()) && !obstacle.may_touch(name)
                })
                .flat_map(move |obstacle| {
                    obstacle.shapes().iter().map(move |shape| Pair::Obstacle {
                        body: (name, body),
                        object: obstacle.id(),
                        shape,
                    })
                })
        });
        let parts = carried(bodies).flat_map(|(object, shape)| {
            self.objects
                .parts
                .iter()
                .filter(move |part| !self.objects.exempt(object, part.link))
                .map(move |part| Pair::Piece {
                    object,
                    shape,
                    link: part.link,
                    piece: &part.piece,
                })
        });
        obstacles.chain(parts).collect()
    }

    /// The pairs of an attached object and the arm that carries it: each object the arm
    /// carries and each piece of the arm's links and of the parts they carry that it may
    /// not touch.
    fn held_pairs<'p>(&'p self, bodies: &'p [(&'p str, Body<'p>)]) -> Vec<Pair<'p>> {
        let with_pieces = carried(bodies).flat_map(|(object, shape)| {
            bodies
                .iter()
                .filter_map(|(link, body)| match body {
                    Body::Piece(piece) => Some((*link, piece)),
                    Body::Shape(_) => None,
                })
                .filter(move |(link, _)| !self.objects.exempt(object, link))
                .map(move |(link, piece)| Pair::Piece {
                    object,
                    shape,
                    link,
                    piece,
                })
        });
        with_pieces.collect()
    }

    /// The distance from `bodies` to the map, exact where it lies within the world
    /// clearance and a lower bound at or above it elsewhere, measured nearest screen
    /// first as `distances` says.
    fn map_distance(&self, bodies: &[(&str, Body)]) -> f64 {
        let mut screens: Vec<(Option<f64>, &Body)> = self
            .on_the_map(bodies)
            .map(|(_, body)| (self.screened(body), body))
            .collect();
        screens.sort_by(|a, b| {
            let nearest = |screen: Option<f64>| screen.unwrap_or(f64::NEG_INFINITY);
            nearest(a.0).total_cmp(&nearest(b.0))
        });
        screens
            .iter()
            .fold(f64::INFINITY, |least, (screen, body)| match *screen {
                None => f64::NEG_INFINITY,
                Some(screen) if screen - SPHERE_BOUND_MARGIN_M >= least => least,
                Some(screen) => self.body_distance_below(body, screen, least),
            })
    }

    /// Whether `body` keeps the world clearance from every occupied voxel. The field
    /// reads the distance between voxel centers, within half a voxel's diagonal of the
    /// sphere center's own distance to the nearest occupied center, which lies within
    /// another half diagonal of the nearest occupied box.
    fn body_clear(&self, body: &Body) -> bool {
        self.screened(body)
            .is_some_and(|screened| screened >= self.clearances[WORLD] || self.exactly_clear(body))
    }

    /// The distance from `body` to the map that its bounding sphere screens: the field's
    /// distance at the sphere's center less two half diagonals and the radius, as
    /// `body_clear` says. None at a center that is not a number, where the body keeps no
    /// clearance.
    fn screened(&self, body: &Body) -> Option<f64> {
        let (center, radius) = body.bounding_sphere();
        let distance = self.field.distance(center.into()).ok()?;
        Some(distance - self.half_diagonal() * 2.0 - radius)
    }

    /// Whether `body` keeps the world clearance from the box of every occupied voxel that
    /// its bounding sphere grown by the clearance can reach.
    fn exactly_clear(&self, body: &Body) -> bool {
        self.near_voxels(body)
            .all(|key| self.voxel_distance(body, key) >= self.clearances[WORLD])
    }

    /// The least of `least` and a lower bound on the distance from `body`, whose sphere
    /// screens `screen`, to the map: the screen where that keeps the world clearance,
    /// else the exact distance to the nearest voxel box within the clearance, or the
    /// clearance when none lies within it. A voxel whose bounding sphere lies past the
    /// least distance found by `SPHERE_BOUND_MARGIN_M` is not measured.
    fn body_distance_below(&self, body: &Body, screen: f64, least: f64) -> f64 {
        if screen >= self.clearances[WORLD] {
            return least.min(screen);
        }
        let (center, radius) = body.bounding_sphere();
        let slack = radius + self.half_diagonal() + SPHERE_BOUND_MARGIN_M;
        self.near_voxels(body)
            .fold(least.min(self.clearances[WORLD]), |nearest, key| {
                let bound = (Point3::from(self.map.center(key)) - center).norm() - slack;
                if bound >= nearest {
                    nearest
                } else {
                    nearest.min(self.voxel_distance(body, key))
                }
            })
    }

    /// The distance from `body` to the box of voxel `key`, minus infinity where it is not
    /// a number.
    fn voxel_distance(&self, body: &Body, key: Key) -> f64 {
        let voxel = self.map.voxel_box(key);
        short_if_nan(body.distance_to_convex(&voxel, voxel.bounding_sphere()))
    }

    /// The occupied voxels whose boxes may lie within the world clearance of `body`:
    /// those within its bounding sphere grown by the clearance.
    fn near_voxels(&self, body: &Body) -> impl Iterator<Item = Key> + use<> {
        let (center, radius) = body.bounding_sphere();
        let reach = radius + self.clearances[WORLD] + self.half_diagonal();
        let min = (center.coords - Vector3::repeat(reach)).into();
        let max = (center.coords + Vector3::repeat(reach)).into();
        self.map.occupied_in(min, max).into_iter()
    }

    fn half_diagonal(&self) -> f64 {
        self.map.voxel_size() * 3.0_f64.sqrt() / 2.0
    }
}

/// Each shape of an object the arm carries among `bodies`, with the object's id.
fn carried<'b>(
    bodies: &'b [(&'b str, Body<'b>)],
) -> impl Iterator<Item = (&'b str, &'b PlacedShape<'b>)> {
    bodies
        .iter()
        .filter_map(|(name, body)| body.shape().map(|shape| (*name, shape)))
}

/// A pair a term checks besides the map and the self model.
enum Pair<'p> {
    /// A body of the arm, by its name, and a shape of the obstacle `object`.
    Obstacle {
        body: (&'p str, &'p Body<'p>),
        object: &'p str,
        shape: &'p PlacedShape<'p>,
    },
    /// A shape of the object `object` the arm carries and a piece of the link `link` of
    /// the robot.
    Piece {
        object: &'p str,
        shape: &'p PlacedShape<'p>,
        link: &'p str,
        piece: &'p Placed<'p>,
    },
}

impl Pair<'_> {
    /// The distance between the bounding spheres of the pair's bodies: a lower bound on
    /// theirs.
    fn gap(&self) -> f64 {
        let (a, b) = match self {
            Self::Obstacle { body, shape, .. } => {
                (body.1.bounding_sphere(), shape.bounding_sphere())
            }
            Self::Piece { shape, piece, .. } => (shape.bounding_sphere(), piece.bounding_sphere()),
        };
        (a.0 - b.0).norm() - a.1 - b.1
    }

    /// The distance between the pair's bodies, minus infinity where it is not a number:
    /// negative where they overlap, and where one lies wholly inside the other's mesh.
    fn distance(&self) -> f64 {
        short_if_nan(match self {
            Self::Obstacle { body, shape, .. } => body.1.distance_to_shape(shape),
            Self::Piece { shape, piece, .. } => shape.distance_to(*piece, piece.bounding_sphere()),
        })
    }

    /// Whether the pair keeps `clearance`: by its spheres' gap, or else its distance.
    fn keeps(&self, clearance: f64) -> bool {
        self.gap() >= clearance || self.distance() >= clearance
    }

    /// Whether the pair's bodies touch.
    fn touches(&self) -> bool {
        self.gap() <= 0.0 && self.distance() <= 0.0
    }

    /// The least of `least` and the pair's distance as a term at `clearance` reads it:
    /// its spheres' gap, a lower bound, where that keeps the clearance, and else its
    /// distance. A gap past `least` by `SPHERE_BOUND_MARGIN_M` leaves `least`.
    fn least_with(&self, least: f64, clearance: f64) -> f64 {
        let gap = self.gap();
        if gap - SPHERE_BOUND_MARGIN_M >= least {
            least
        } else if gap >= clearance {
            least.min(gap)
        } else {
            least.min(self.distance())
        }
    }

    fn contact(&self) -> Contact {
        match self {
            Self::Obstacle { body, object, .. } => Contact::Object {
                body: body.0.to_string(),
                object: object.to_string(),
            },
            Self::Piece { object, link, .. } => Contact::Own {
                a: object.to_string(),
                b: link.to_string(),
            },
        }
    }
}

/// The least distance between the pairs of `arm`'s self model at `q`: infinite for an
/// arm with no self model, and minus infinity where the model answers no number.
fn own_distance<const N: usize>(arm: &Arm<N>, q: &[f64; N]) -> f64 {
    arm.self_model()
        .map_or(f64::INFINITY, |model| short_if_nan(model.clearance(q)))
}

/// `distance`, or minus infinity, short of every clearance, where it is not a number.
fn short_if_nan(distance: f64) -> f64 {
    if distance.is_nan() {
        f64::NEG_INFINITY
    } else {
        distance
    }
}

#[cfg(test)]
mod tests {
    use chain_kinematics::{ChainSpec, JointSelection};
    use collision_geometry::chain::ChainGeometry;
    use control_core::positive_finite::PositiveFinite;
    use hegel::TestCase;
    use hegel::generators as gs;
    use kinematics_properties::unit_vectors;
    use occupancy_map::{Bounds, DistanceField, Frame, Scan, VoxelMap};

    use super::*;
    use crate::arm::SelfModel;
    use crate::scene::{
        AttachRequest, ObjectFields, ObjectRequest, PlanningScene, RobotLinks, RobotPart,
        ShapeFields,
    };

    /// An arm of one revolute joint whose link is a 10 cm box.
    const ONE_BOX: &str = r#"<robot name="one_box">
        <link name="base"/>
        <link name="link_1">
          <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
        </link>
        <joint name="joint_1" type="revolute">
          <parent link="base"/><child link="link_1"/><axis xyz="0 0 1"/>
          <limit lower="-1" upper="1" effort="1" velocity="1"/>
        </joint>
      </robot>"#;

    /// An arm of two revolute joints, about z and then about y 35 cm out, whose first
    /// link holds two boxes and whose second holds a ball 4 cm in radius.
    const BOXES_AND_A_BALL: &str = r#"<robot name="boxes_and_a_ball">
        <link name="base"/>
        <link name="link_1">
          <collision>
            <origin xyz="0.1 0 0"/><geometry><box size="0.12 0.05 0.05"/></geometry>
          </collision>
          <collision>
            <origin xyz="0.27 0 0"/><geometry><box size="0.12 0.04 0.06"/></geometry>
          </collision>
        </link>
        <link name="link_2">
          <collision>
            <origin xyz="0.12 0 0"/><geometry><sphere radius="0.04"/></geometry>
          </collision>
        </link>
        <joint name="joint_1" type="revolute">
          <parent link="base"/><child link="link_1"/><axis xyz="0 0 1"/>
          <limit lower="-3" upper="3" effort="1" velocity="1"/>
        </joint>
        <joint name="joint_2" type="revolute">
          <origin xyz="0.35 0 0"/>
          <parent link="link_1"/><child link="link_2"/><axis xyz="0 1 0"/>
          <limit lower="-2" upper="2" effort="1" velocity="1"/>
        </joint>
      </robot>"#;

    fn geometry<const N: usize>(urdf: &str, tip_link: &str) -> ChainGeometry<N> {
        ChainGeometry::from_urdf(
            urdf,
            "",
            &ChainSpec {
                base_link: None,
                tip_link,
                joints: JointSelection::PathOrder,
            },
        )
        .expect("an arm of boxes")
    }

    /// A map of `voxel_m` voxels over the cube of half side 1 m about the origin, each
    /// voxel of `points` occupied, and its distance field.
    fn world(voxel_m: f64, points: &[[f64; 3]]) -> (VoxelMap, DistanceField) {
        let frame = Frame::parse("world").expect("a frame name");
        let mut map = VoxelMap::new(
            frame.clone(),
            PositiveFinite::parse(voxel_m).expect("positive"),
            Bounds {
                min: [-1.0; 3],
                max: [1.0; 3],
            },
        )
        .expect("a valid map");
        let relative: Vec<[f32; 3]> = points
            .iter()
            .map(|p| std::array::from_fn(|i| p[i] as f32))
            .collect();
        map.insert(
            &Scan {
                frame: &frame,
                sensor: camera_core::pose::Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0])
                    .expect("a pose"),
                points: &relative,
                max_range_m: PositiveFinite::parse(10.0).expect("positive"),
            },
            &[],
        )
        .expect("the scan stands in the map's frame");
        let field = DistanceField::of(&map);
        (map, field)
    }

    // A configuration that is not a number keeps no clearance from an empty map: the
    // field gives no distance at its pieces' centers, so it lies at minus infinity, where
    // a configuration of numbers keeps every clearance.
    #[test]
    fn a_configuration_that_is_not_a_number_keeps_no_clearance() {
        let geometry = geometry::<1>(ONE_BOX, "link_1");
        let arm = Arm::new(&geometry, None);
        let (map, field) = world(0.05, &[]);
        let surroundings = Surroundings::new(&map, &field, [0.0; TERMS]);
        assert!(surroundings.clear(&arm, &[0.0]));
        let not_a_number = [f64::NAN];
        assert!(!surroundings.clear(&arm, &not_a_number));
        assert_eq!(
            surroundings.distances(&arm, &not_a_number)[WORLD],
            f64::NEG_INFINITY
        );
    }

    /// A self model that answers no number.
    struct NotANumber;

    impl SelfModel<1> for NotANumber {
        fn clearance(&self, _: &[f64; 1]) -> f64 {
            f64::NAN
        }

        fn nearest_pair(&self, _: &[f64; 1]) -> (String, String) {
            ("a link".to_string(), "its neighbor".to_string())
        }
    }

    // A self model that answers no number at a configuration leaves the arm short of
    // every clearance of its own body there: not clear, in contact, and at minus
    // infinity.
    #[test]
    fn a_self_model_that_answers_no_number_counts_as_short() {
        let geometry = geometry::<1>(ONE_BOX, "link_1");
        let arm = Arm::new(&geometry, Some(&NotANumber));
        let (map, field) = world(0.05, &[]);
        let surroundings = Surroundings::new(&map, &field, [0.0; TERMS]);
        assert!(!surroundings.clear(&arm, &[0.0]));
        assert_eq!(
            surroundings.contact(&arm, &[0.0]),
            Some(Contact::Own {
                a: "a link".to_string(),
                b: "its neighbor".to_string()
            })
        );
        assert_eq!(surroundings.distances(&arm, &[0.0])[OWN], f64::NEG_INFINITY);
    }

    // On drawn configurations of an arm of boxes and a ball among drawn voxels just
    // outside the bounding spheres of its pieces, at a drawn clearance, the distance the
    // screens and the bounds set aside measure equals the least distance GJK measures
    // from every piece to every occupied voxel's box where that lies within the
    // clearance, and lies between the clearance and it elsewhere. Measured from each piece
    // after a least distance drawn up to 2 cm past the piece's own, it is the least of that
    // and the piece's distance as the screen and GJK measure it.
    #[hegel::test(test_cases = 300)]
    fn the_distance_from_the_map_sets_aside_no_nearer_body_or_voxel(tc: TestCase) {
        let geometry = geometry::<2>(BOXES_AND_A_BALL, "link_2");
        let arm = Arm::new(&geometry, None);
        let limits = geometry.chain().limits();
        let q: [f64; 2] = std::array::from_fn(|i| {
            tc.draw(gs::floats().min_value(limits[i].lo).max_value(limits[i].hi))
        });
        let pieces = arm.bodies(&q);
        let count = tc.draw(gs::integers::<usize>().min_value(1).max_value(12));
        let points: Vec<[f64; 3]> = (0..count)
            .map(|_| {
                let piece = tc.draw(
                    gs::integers::<usize>()
                        .min_value(0)
                        .max_value(pieces.len() - 1),
                );
                let (center, radius) = pieces[piece].1.bounding_sphere();
                let beyond: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.12));
                let p = center + tc.draw(unit_vectors()) * (radius + beyond);
                [p.x, p.y, p.z]
            })
            .collect();
        let voxel_m = tc.draw(gs::sampled_from(&[0.01, 0.02, 0.04]));
        let (map, field) = world(voxel_m, &points);
        let clearance: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.15));
        let surroundings = Surroundings::new(&map, &field, [clearance, 0.0, 0.0]);
        let from_piece = |piece: &Body| {
            map.occupied()
                .map(|key| surroundings.voxel_distance(piece, key))
                .fold(f64::INFINITY, f64::min)
        };
        let exact = pieces
            .iter()
            .map(|(_, piece)| from_piece(piece))
            .fold(f64::INFINITY, f64::min);
        let measured = surroundings.distances(&arm, &q)[0];
        if exact < clearance {
            assert_eq!(measured, exact);
        } else {
            assert!(
                clearance <= measured && measured <= exact,
                "{measured} lies outside {clearance} to {exact}"
            );
        }
        for (link, piece) in &pieces {
            let own = from_piece(piece);
            let screen = surroundings.screened(piece).expect("a piece at numbers");
            let past: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.02));
            let least = own.min(1.0) + past;
            let expected = if screen >= clearance {
                least.min(screen)
            } else {
                least.min(clearance).min(own)
            };
            assert_eq!(
                surroundings.body_distance_below(piece, screen, least),
                expected,
                "{link} after a least distance of {least}"
            );
        }
    }

    /// How far the gap of two bounding spheres, the distance of their centers less their
    /// radii, may round above the distance that GJK measures between the two balls: far
    /// beyond the some 1e-15 m by which GJK's distance rounds and the dozen roundings of
    /// the gap, each within half an ulp at the map's scale of 1 m, 1.1e-16 m.
    const SPHERE_GAP_ROUNDING_M: f64 = 1e-14;

    // On drawn configurations of the arm of boxes and a ball, among drawn objects just
    // outside the bounding spheres of its pieces, holding a drawn object on its tip,
    // link_2, beside a copy of the arm standing still at another drawn configuration, at
    // drawn clearances: the distance from the map and the objects that the spheres and the
    // bounds set aside measure equals the least distance from every body of the arm, the
    // held object's among them, to every shape of every object, and from the held object
    // to every piece of the copy, where that lies within the clearance, and lies between
    // the clearance and it, past it by no more than a gap of bounding spheres rounds,
    // elsewhere; and so does the distance of an attached object from the arm that carries
    // it, from the held object to the pieces of link_1, which it may not touch. The state
    // is clear exactly where both keep their clearance. The explicit cases are a ball
    // beside the arm's ball at a clearance of 0, whose gap rounds 4e-18 m above GJK's
    // distance, and a held cylinder 1.77 cm into link_1 beside two balls, an overlap
    // whose depth EPA measures twice.
    #[hegel::test(test_cases = 300)]
    #[hegel::explicit_test_case(
        draw_1 = 0.0,
        draw_2 = 1.0,
        draw_3 = 0.0,
        draw_4 = 0.0,
        count = 1usize,
        draw_5 = 1usize,
        beyond_1 = 0.109375,
        draw_6 = nalgebra::Vector3::new(0.982264602843857, 0.0, 0.1875),
        axis_1 = nalgebra::Vector3::new(1.0, 0.0, 0.0),
        angle_1 = 0.0,
        draw_7 = 1usize,
        draw_8 = 0.0625,
        draw_9 = 1usize,
        beyond_2 = 0.0625,
        draw_10 = nalgebra::Vector3::new(1.0, 0.0, 0.0),
        axis_2 = nalgebra::Vector3::new(1.0, 0.0, 0.0),
        angle_2 = 0.0,
        draw_11 = 1usize,
        draw_12 = 0.03125000000000002,
        draw_13 = 0.0,
        draw_14 = 0.0
    )]
    #[hegel::explicit_test_case(
        draw_1 = 0.0,
        draw_2 = 0.0,
        draw_3 = 0.0,
        draw_4 = 0.0,
        count = 2usize,
        draw_5 = 0usize,
        beyond_1 = 0.0,
        draw_6 = nalgebra::Vector3::new(1.0, 0.0, -0.0),
        axis_1 = nalgebra::Vector3::new(1.0, 0.0, 0.0),
        angle_1 = -1.9053867834828624e-287,
        draw_7 = 1usize,
        draw_8 = 0.06695332895289921,
        draw_9 = 0usize,
        beyond_2 = 0.0,
        draw_10 = nalgebra::Vector3::new(1.0, -0.0, 0.0),
        axis_2 = nalgebra::Vector3::new(1.0, -0.0, 0.0),
        angle_2 = 0.0,
        draw_11 = 1usize,
        draw_12 = 0.011627420761683958,
        draw_13 = 1usize,
        beyond_3 = 0.0,
        draw_14 = nalgebra::Vector3::new(0.8606927022888975, -0.5091248100678607, 0.0),
        axis_3 = nalgebra::Vector3::new(0.5403023058681398, 0.8414709848078965, 0.0),
        angle_3 = 0.0,
        draw_15 = 2usize,
        draw_16 = 0.03014184077798001,
        draw_17 = 0.06676713287257004,
        draw_18 = 0.0,
        draw_19 = 0.09454906582976332
    )]
    fn the_distance_from_the_objects_sets_aside_no_nearer_pair(tc: TestCase) {
        let geometry = geometry::<2>(BOXES_AND_A_BALL, "link_2");
        let limits = geometry.chain().limits();
        let q: [f64; 2] = std::array::from_fn(|i| {
            tc.draw(gs::floats().min_value(limits[i].lo).max_value(limits[i].hi))
        });
        let still: [f64; 2] = std::array::from_fn(|i| {
            tc.draw(gs::floats().min_value(limits[i].lo).max_value(limits[i].hi))
        });
        let parts: Vec<RobotPart> = Arm::new(&geometry, None)
            .bodies(&still)
            .into_iter()
            .map(|(_, body)| match body {
                Body::Piece(piece) => RobotPart {
                    link: "standing",
                    piece,
                },
                Body::Shape(_) => panic!("the bare arm carries no object"),
            })
            .collect();
        let spheres: Vec<(Point3<f64>, f64)> = Arm::new(&geometry, None)
            .bodies(&q)
            .iter()
            .map(|(_, body)| body.bounding_sphere())
            .collect();
        // The drawing closures take `tc` from the test, so an explicit case draws through
        // them too.
        let near_a_body = || -> [f64; 3] {
            let (center, radius) = spheres[tc.draw(
                gs::integers::<usize>()
                    .min_value(0)
                    .max_value(spheres.len() - 1),
            )];
            let beyond: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.12));
            (center + tc.draw(unit_vectors()) * (radius + beyond))
                .coords
                .into()
        };
        // A unit quaternion `[x, y, z, w]` drawn about a drawn axis.
        let drawn_orientation = || -> [f64; 4] {
            let axis = tc.draw(unit_vectors());
            let angle: f64 = tc.draw(gs::floats().min_value(-3.0).max_value(3.0));
            let q = nalgebra::UnitQuaternion::from_scaled_axis(axis * angle);
            [q.i, q.j, q.k, q.w]
        };
        // A shape of a drawn kind, a box, a sphere or a cylinder 1 cm to 8 cm across, as
        // motion_planning writes it, at its object's origin.
        let drawn_shape = || -> ShapeFields {
            let length = || gs::floats().min_value(0.01).max_value(0.08);
            let (kind, dimensions) =
                match tc.draw(gs::integers::<usize>().min_value(0).max_value(2)) {
                    0 => {
                        let size: [f64; 3] = tc.draw(gs::arrays(length()));
                        ("box", size.to_vec())
                    }
                    1 => ("sphere", vec![tc.draw(length()) / 2.0]),
                    _ => ("cylinder", vec![tc.draw(length()), tc.draw(length()) / 2.0]),
                };
            ShapeFields {
                kind: kind.to_string(),
                dimensions,
                mesh_vertices: Vec::new(),
                mesh_triangles: Vec::new(),
                position: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
            }
        };
        let count = tc.draw(gs::integers::<usize>().min_value(1).max_value(5));
        let mut requests: Vec<ObjectRequest> = (0..count)
            .map(|k| ObjectRequest {
                operation: "add".to_string(),
                object: ObjectFields {
                    id: format!("object_{k}"),
                    position: near_a_body(),
                    orientation: drawn_orientation(),
                    shapes: vec![drawn_shape()],
                },
            })
            .collect();
        requests.push(ObjectRequest {
            operation: "add".to_string(),
            object: ObjectFields {
                id: "held".to_string(),
                position: near_a_body(),
                orientation: drawn_orientation(),
                shapes: vec![drawn_shape()],
            },
        });
        let (mut map, _) = world(0.04, &[]);
        let names = ["link_1", "link_2"].map(String::from);
        let mut scene = PlanningScene::new(RobotLinks::new(names.clone(), ["link_2".to_string()]));
        scene
            .apply_collision_objects(&requests, &mut map)
            .expect("the drawn objects apply");
        scene
            .apply_attached_collision_object(
                &AttachRequest {
                    object_id: "held".to_string(),
                    link_name: "link_2".to_string(),
                    touch_links: Vec::new(),
                    operation: "attach".to_string(),
                },
                |_| Some(geometry.chain().world_ee_pose(&q)),
                &mut map,
            )
            .expect("the held object attaches to the tip");
        let field = DistanceField::of(&map);
        let objects = scene.for_arm("link_2", |link| panic!("{link} is no other arm's"));
        let riders = objects.riders(&geometry);
        let arm = Arm::new(&geometry, None)
            .riding(&riders)
            .expect("the held object rides the tip");
        let [world_clearance, held_clearance]: [f64; 2] =
            std::array::from_fn(|_| tc.draw(gs::floats().min_value(0.0).max_value(0.15)));
        let surroundings = Surroundings::new(&map, &field, [world_clearance, 0.0, held_clearance])
            .with_objects(objects.objects(&parts));
        let bodies = arm.bodies(&q);
        let from_objects = bodies.iter().flat_map(|(_, body)| {
            objects
                .obstacles()
                .iter()
                .flat_map(|obstacle| obstacle.shapes())
                .map(|shape| body.distance_to_shape(shape))
        });
        let from_parts = bodies
            .iter()
            .filter_map(|(_, body)| body.shape())
            .flat_map(|held| {
                parts
                    .iter()
                    .map(|part| held.distance_to(&part.piece, part.piece.bounding_sphere()))
            });
        let exact_world = from_objects.chain(from_parts).fold(f64::INFINITY, f64::min);
        let exact_held = bodies
            .iter()
            .filter_map(|(_, body)| body.shape())
            .flat_map(|held| {
                bodies
                    .iter()
                    .filter(|(name, _)| *name == "link_1")
                    .filter_map(move |(_, body)| match body {
                        Body::Piece(piece) => {
                            Some(held.distance_to(piece, piece.bounding_sphere()))
                        }
                        Body::Shape(_) => None,
                    })
            })
            .fold(f64::INFINITY, f64::min);
        let measured = surroundings.distances(&arm, &q);
        for (term, exact, clearance) in [
            (WORLD, exact_world, world_clearance),
            (HELD, exact_held, held_clearance),
        ] {
            if exact < clearance {
                assert_eq!(measured[term], exact, "term {term}");
            } else {
                assert!(
                    clearance <= measured[term] && measured[term] <= exact + SPHERE_GAP_ROUNDING_M,
                    "term {term}: {} lies outside {clearance} to {exact}",
                    measured[term]
                );
            }
        }
        assert_eq!(
            surroundings.clear(&arm, &q),
            exact_world >= world_clearance && exact_held >= held_clearance
        );
    }
}
