//! Whether the arm at a configuration keeps its clearance: from every occupied voxel of
//! the map, and between the pairs of its self model.
//!
//! A piece's bounding sphere screens against the map's distance field: when the sphere
//! clears every voxel by the clearance, so does the piece. A piece whose sphere does not
//! is checked exactly, hull against the box of each occupied voxel near it.

use collision_geometry::gjk::{self, Placed, Support};
use nalgebra::{Point3, Vector3};
use occupancy_map::{DistanceField, Key, VoxelMap};

use crate::arm::Arm;

/// The box of one voxel.
struct VoxelBox {
    center: Point3<f64>,
    half: f64,
}

impl Support for VoxelBox {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        self.center + dir.map(|d| if d < 0.0 { -self.half } else { self.half })
    }
}

/// What a state touches.
#[derive(Debug, PartialEq, Eq)]
pub enum Contact {
    /// A link of the arm, or a part a link carries, and an occupied voxel of the map.
    Map { link: String },
    /// Two links of the robot's own body, a pair its self model checks.
    Own { a: String, b: String },
}

impl std::fmt::Display for Contact {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Map { link } => write!(f, "{link} touches the occupancy map"),
            Self::Own { a, b } => write!(f, "{a} touches {b}"),
        }
    }
}

/// How far past the least distance found a bound from bounding spheres, a piece's screen
/// or a voxel's sphere, lies when it sets the piece or the voxel aside: far beyond the
/// rounding of GJK's distance, some 1e-15 m, so what it sets aside lies no nearer.
const SPHERE_BOUND_MARGIN_M: f64 = 1e-9;

/// What a state keeps clear of: the map as it stands, and the clearances to keep.
pub struct Surroundings<'a> {
    map: &'a VoxelMap,
    field: &'a DistanceField,
    /// The clearance a piece keeps from every occupied voxel.
    world: f64,
    /// The clearance each pair of the self model keeps.
    own: f64,
}

impl<'a> Surroundings<'a> {
    /// The surroundings of a state on `map`, whose distances `field` holds, at which
    /// each piece keeps `world` meters from every occupied voxel and each pair of the
    /// self model `own` meters.
    pub fn new(map: &'a VoxelMap, field: &'a DistanceField, world: f64, own: f64) -> Self {
        Self {
            map,
            field,
            world,
            own,
        }
    }

    /// Whether `arm` at `q` keeps its clearances.
    pub fn clear<const N: usize>(&self, arm: &Arm<N>, q: &[f64; N]) -> bool {
        own_distance(arm, q) >= self.own
            && arm
                .pieces(q)
                .iter()
                .all(|(_, piece)| self.piece_clear(piece))
    }

    /// The distance from `arm` at `q` to the map and the least between the pairs of its
    /// self model, in meters, in that order: each exact where it lies within its
    /// clearance, and a lower bound at or above the clearance elsewhere; infinite for an
    /// arm with no pieces or no self model. The pieces are measured nearest screen first,
    /// and a piece whose screen, or a voxel whose bounding sphere, lies past the least
    /// distance found by `SPHERE_BOUND_MARGIN_M` is set aside: it lies no nearer.
    pub(crate) fn distances<const N: usize>(&self, arm: &Arm<N>, q: &[f64; N]) -> [f64; 2] {
        let mut screens: Vec<(Option<f64>, Placed)> = arm
            .pieces(q)
            .into_iter()
            .map(|(_, piece)| (self.screened(&piece), piece))
            .collect();
        screens.sort_by(|a, b| {
            let nearest = |screen: Option<f64>| screen.unwrap_or(f64::NEG_INFINITY);
            nearest(a.0).total_cmp(&nearest(b.0))
        });
        let map = screens
            .iter()
            .fold(f64::INFINITY, |least, (screen, piece)| match *screen {
                None => f64::NEG_INFINITY,
                Some(screen) if screen - SPHERE_BOUND_MARGIN_M >= least => least,
                Some(screen) => self.piece_distance_below(piece, screen, least),
            });
        [map, own_distance(arm, q)]
    }

    /// The link of `arm` at `q` nearest the map and its distance from it, in meters: exact
    /// where it lies within the world clearance, a lower bound at or above it elsewhere.
    /// `None` for an arm with no pieces.
    pub(crate) fn nearest_to_map<const N: usize>(
        &self,
        arm: &Arm<N>,
        q: &[f64; N],
    ) -> Option<(String, f64)> {
        arm.pieces(q)
            .into_iter()
            .map(|(link, piece)| {
                let distance = self.screened(&piece).map_or(f64::NEG_INFINITY, |screen| {
                    self.piece_distance_below(&piece, screen, f64::INFINITY)
                });
                (link.to_string(), distance)
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }

    /// What `arm` at `q` touches, if anything: a pair of its own body before the map.
    pub fn contact<const N: usize>(&self, arm: &Arm<N>, q: &[f64; N]) -> Option<Contact> {
        let own = arm
            .self_model()
            .filter(|model| short_if_nan(model.clearance(q)) <= 0.0)
            .map(|model| {
                let (a, b) = model.nearest_pair(q);
                Contact::Own { a, b }
            });
        own.or_else(|| {
            arm.pieces(q)
                .into_iter()
                .find(|(_, piece)| {
                    self.near_voxels(piece)
                        .any(|key| self.voxel_distance(piece, key) <= 0.0)
                })
                .map(|(link, _)| Contact::Map {
                    link: link.to_string(),
                })
        })
    }

    /// Whether `piece` keeps the world clearance from every occupied voxel. The field
    /// reads the distance between voxel centers, within half a voxel's diagonal of the
    /// sphere center's own distance to the nearest occupied center, which lies within
    /// another half diagonal of the nearest occupied box.
    fn piece_clear(&self, piece: &Placed) -> bool {
        self.screened(piece)
            .is_some_and(|screened| screened >= self.world || self.hull_clear(piece))
    }

    /// The distance from `piece` to the map that its bounding sphere screens: the
    /// field's distance at the sphere's center less two half diagonals and the radius,
    /// as `piece_clear` says. None at a center that is not a number, where the piece
    /// keeps no clearance.
    fn screened(&self, piece: &Placed) -> Option<f64> {
        let (center, radius) = piece.bounding_sphere();
        let distance = self.field.distance(center.into()).ok()?;
        Some(distance - self.half_diagonal() * 2.0 - radius)
    }

    /// Whether `piece`'s hull keeps the world clearance from the box of every occupied
    /// voxel that its bounding sphere grown by the clearance can reach.
    fn hull_clear(&self, piece: &Placed) -> bool {
        self.near_voxels(piece)
            .all(|key| self.voxel_distance(piece, key) >= self.world)
    }

    /// The least of `least` and a lower bound on the distance from `piece`, whose sphere
    /// screens `screen`, to the map: the screen where that keeps the world clearance,
    /// else the exact distance to the nearest voxel box within the clearance, or the
    /// clearance when none lies within it. A voxel whose bounding sphere lies past the
    /// least distance found by `SPHERE_BOUND_MARGIN_M` is not measured.
    fn piece_distance_below(&self, piece: &Placed, screen: f64, least: f64) -> f64 {
        if screen >= self.world {
            return least.min(screen);
        }
        let (center, radius) = piece.bounding_sphere();
        let slack = radius + self.half_diagonal() + SPHERE_BOUND_MARGIN_M;
        self.near_voxels(piece)
            .fold(least.min(self.world), |nearest, key| {
                let bound = (Point3::from(self.map.center(key)) - center).norm() - slack;
                if bound >= nearest {
                    nearest
                } else {
                    nearest.min(self.voxel_distance(piece, key))
                }
            })
    }

    /// The distance GJK measures from `piece` to the box of voxel `key`, minus infinity
    /// where it is not a number.
    fn voxel_distance(&self, piece: &Placed, key: Key) -> f64 {
        short_if_nan(gjk::distance(piece, &self.voxel_box(key)).distance)
    }

    /// The occupied voxels whose boxes may lie within the world clearance of `piece`:
    /// those within its bounding sphere grown by the clearance.
    fn near_voxels(&self, piece: &Placed) -> impl Iterator<Item = Key> + use<> {
        let (center, radius) = piece.bounding_sphere();
        let reach = radius + self.world + self.half_diagonal();
        let min = (center.coords - Vector3::repeat(reach)).into();
        let max = (center.coords + Vector3::repeat(reach)).into();
        self.map.occupied_in(min, max).into_iter()
    }

    fn voxel_box(&self, key: Key) -> VoxelBox {
        VoxelBox {
            center: self.map.center(key).into(),
            half: self.map.voxel_size() / 2.0,
        }
    }

    fn half_diagonal(&self) -> f64 {
        self.map.voxel_size() * 3.0_f64.sqrt() / 2.0
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
        let surroundings = Surroundings::new(&map, &field, 0.0, 0.0);
        assert!(surroundings.clear(&arm, &[0.0]));
        let not_a_number = [f64::NAN];
        assert!(!surroundings.clear(&arm, &not_a_number));
        assert_eq!(
            surroundings.distances(&arm, &not_a_number)[0],
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
        let surroundings = Surroundings::new(&map, &field, 0.0, 0.0);
        assert!(!surroundings.clear(&arm, &[0.0]));
        assert_eq!(
            surroundings.contact(&arm, &[0.0]),
            Some(Contact::Own {
                a: "a link".to_string(),
                b: "its neighbor".to_string()
            })
        );
        assert_eq!(surroundings.distances(&arm, &[0.0])[1], f64::NEG_INFINITY);
    }

    // On drawn configurations of an arm of boxes and a ball among drawn voxels just
    // outside the bounding spheres of its pieces, at a drawn clearance, the distance the
    // screens and the bounds set aside measure equals the least distance GJK measures
    // from every piece to every occupied voxel's box where that lies within the
    // clearance, and lies between the clearance and it elsewhere. Measured from each piece
    // after a least distance drawn up to 2 cm past the piece's own, it is the least of that
    // and the piece's distance as the screen and GJK measure it.
    #[hegel::test(test_cases = 300)]
    fn the_distance_from_the_map_sets_aside_no_nearer_piece_or_voxel(tc: TestCase) {
        let geometry = geometry::<2>(BOXES_AND_A_BALL, "link_2");
        let arm = Arm::new(&geometry, None);
        let limits = geometry.chain().limits();
        let q: [f64; 2] = std::array::from_fn(|i| {
            tc.draw(gs::floats().min_value(limits[i].lo).max_value(limits[i].hi))
        });
        let pieces = arm.pieces(&q);
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
        let surroundings = Surroundings::new(&map, &field, clearance, 0.0);
        let from_piece = |piece: &Placed| {
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
                surroundings.piece_distance_below(piece, screen, least),
                expected,
                "{link} after a least distance of {least}"
            );
        }
    }
}
