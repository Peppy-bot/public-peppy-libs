//! What a robot's own answer, from its design, holds a reaching arm clear of: the surface it is
//! asked about, an empty plane at its height, as the top of a block under it that stands from the
//! front of the robot's body to the far end of the measured grid ([`SurfaceBlock`]). The answer
//! knows nothing else of the room.

use collision_geometry::nalgebra::{Isometry3, Translation3, UnitQuaternion};
use collision_geometry::shapes::{PlacedShape, Shape};
use robot_description::{OnShape, RobotDescription, UnsupportedShape, on_shape};
use self_collision_model::{CollisionError, SelfCollisionModel};

use crate::Grid;
use crate::design::SurfaceHeight;
use crate::reach::Clearance;

/// Why the bodies of a robot's arms are not built from its description.
#[derive(Debug, thiserror::Error)]
pub enum ArmBodiesError {
    #[error(transparent)]
    Shape(#[from] UnsupportedShape),
    #[error("the self model of the description: {0}")]
    Model(#[from] CollisionError),
}

/// The opening of a gripper that reaches for something: fully open, as it stands before it closes.
const REACHING_OPENING: f64 = 1.0;

/// The collision bodies of a robot's arms, from its description: each link of each arm's chain
/// and what the link carries, its fingers included, each gripper at [`REACHING_OPENING`]. With
/// them, the front of the robot's body: the farthest ahead any piece of the bodies no arm moves
/// stands, a torso, a base or a mount.
pub struct ArmBodies {
    pieces: Box<dyn ArmPieces + Send + Sync>,
    front: f64,
}

impl ArmBodies {
    /// The arm bodies of the robot `description` describes, its collision meshes read from
    /// `meshes_dir`.
    pub fn new(description: &RobotDescription, meshes_dir: &str) -> Result<Self, ArmBodiesError> {
        on_shape(
            description,
            Build {
                description,
                meshes_dir,
            },
        )?
    }

    /// How many joints each arm's chain has.
    pub fn joints_per_arm(&self) -> usize {
        self.pieces.joints_per_arm()
    }

    /// The clearance of the surface at `height`.
    pub fn over(&mut self, height: SurfaceHeight) -> SurfaceBlock<'_> {
        let far = Grid::STANDARD.x[1];
        let near = self.front.max(Grid::STANDARD.x[0]);
        let block = (near < far).then(|| {
            let size = [far - near, 2.0 * far, far];
            let center = [(near + far) / 2.0, 0.0, height.metres() - far / 2.0];
            let block = Shape::cuboid(size).expect("a block of a positive length each way");
            let pose =
                Isometry3::from_parts(Translation3::from(center), UnitQuaternion::identity());
            (block, pose)
        });
        SurfaceBlock {
            bodies: self,
            block,
        }
    }
}

/// The clearance of a surface at one height, in the robot frame: the block under its top stands
/// from the front of the robot's body, where the nearest edge of a surface the robot stands at
/// can be, to the far end of the measured grid ([`Grid::STANDARD`]), and reaches as far to each
/// side of the robot and as deep below the top as the grid reaches ahead, past anything an arm
/// that reaches the grid can bring under it. A pose of an arm is clear when no collision piece of
/// the bodies that arm moves overlaps the block; a piece may touch its top. A robot whose body
/// stands as far ahead as the grid reaches has no block before it, and every pose is clear.
pub struct SurfaceBlock<'a> {
    bodies: &'a mut ArmBodies,
    block: Option<(Shape, Isometry3<f64>)>,
}

impl Clearance for SurfaceBlock<'_> {
    fn clear(&mut self, arm: usize, joints: &[f64]) -> bool {
        let Some((block, pose)) = &self.block else {
            return true;
        };
        let block = PlacedShape::new(block, *pose);
        self.bodies.pieces.clear_of(arm, joints, &block)
    }
}

/// The bodies of a robot's arms at the shape its self model is compiled for.
trait ArmPieces {
    /// How many joints each arm's chain has.
    fn joints_per_arm(&self) -> usize;

    /// Whether arm `arm` at `joints` stands clear of `block`: no piece of the bodies the arm moves
    /// overlaps it. A pose that is not finite is not clear. Panics on a pose of another count of
    /// joints than [`Self::joints_per_arm`].
    fn clear_of(&mut self, arm: usize, joints: &[f64], block: &PlacedShape<'_>) -> bool;
}

struct Pieces<const N: usize, const K: usize> {
    model: SelfCollisionModel<N, K>,
}

impl<const N: usize, const K: usize> ArmPieces for Pieces<N, K> {
    fn joints_per_arm(&self) -> usize {
        N
    }

    fn clear_of(&mut self, arm: usize, joints: &[f64], block: &PlacedShape<'_>) -> bool {
        let joints: [f64; N] = joints
            .try_into()
            .unwrap_or_else(|_| panic!("a pose of an arm of {N} joints, given {}", joints.len()));
        // The pieces of the reaching arm's bodies stand where its own joints alone put them.
        let mut q = [[0.0; N]; K];
        q[arm] = joints;
        let Ok(bodies) = self.model.world_pieces_of_chain(arm, &q) else {
            return false;
        };
        bodies
            .iter()
            .flat_map(|(_, pieces)| pieces)
            .all(|piece| block.distance_to(piece, piece.bounding_sphere()) >= 0.0)
    }
}

struct Build<'a> {
    description: &'a RobotDescription,
    meshes_dir: &'a str,
}

impl OnShape for Build<'_> {
    type Output = Result<ArmBodies, ArmBodiesError>;

    fn on_shape<const N: usize, const D: usize, const K: usize>(self) -> Self::Output {
        let mut model =
            SelfCollisionModel::<N, K>::from_description(self.description, self.meshes_dir)?;
        model.set_gripper_openings(&[REACHING_OPENING; K]);
        let front = model
            .fixed_world_pieces()
            .iter()
            .flat_map(|(_, pieces)| pieces.iter().flat_map(|piece| piece.vertices()))
            .map(|vertex| vertex.x)
            .fold(f64::NEG_INFINITY, f64::max);
        Ok(ArmBodies {
            pieces: Box::new(Pieces { model }),
            front,
        })
    }
}

#[cfg(test)]
mod tests {
    use robot_description::states;

    use super::*;

    fn bodies(id: &str) -> (ArmBodies, Vec<Vec<f64>>) {
        let bundled = robot_description::model(id).expect("bundled");
        let description = bundled.description();
        let meshes = bundled.directory().join("meshes");
        let ready = description
            .arms()
            .iter()
            .map(|arm| {
                description
                    .state(&arm.name, states::READY)
                    .expect("ready")
                    .to_vec()
            })
            .collect();
        let bodies = ArmBodies::new(&description, meshes.to_str().expect("UTF-8")).expect("builds");
        (bodies, ready)
    }

    fn height(metres: f64) -> SurfaceHeight {
        SurfaceHeight::from_wire(metres).expect("a height")
    }

    // OpenArm v2's grasp points stand 0.57 m above its base point at Ready, over the grid.
    #[test]
    fn an_arm_at_ready_clears_a_surface_under_its_grasp_point_and_sinks_into_one_over_it() {
        let (mut bodies, ready) = bodies("openarm_v2");
        for (arm, joints) in ready.iter().enumerate() {
            assert!(bodies.over(height(0.30)).clear(arm, joints), "arm {arm}");
            assert!(!bodies.over(height(0.70)).clear(arm, joints), "arm {arm}");
        }
    }

    // Nothing of the SO-101 at Ready hangs below its base, which stands on the surface.
    #[test]
    fn an_so101_at_ready_clears_the_surface_it_stands_on() {
        let (mut bodies, ready) = bodies("so101");
        assert!(bodies.over(height(0.0)).clear(0, &ready[0]));
    }

    // The OpenArm's shoulders stand 0.64 m to 0.76 m above its base point and its torso's front
    // 0.095 m ahead of it. At Home an arm hangs beside the torso, behind that front, so the block
    // of a surface at 0.70 m stands ahead of it; at Ready the forearm reaches forward under it.
    #[test]
    fn the_block_stands_from_the_front_of_the_body() {
        let (mut bodies, ready) = bodies("openarm_v2");
        assert!((bodies.front - 0.095).abs() < 0.001, "{}", bodies.front);
        let description = robot_description::model("openarm_v2")
            .unwrap()
            .description();
        for (arm, ready) in ready.iter().enumerate() {
            let home = description
                .state(&description.arms()[arm].name, states::HOME)
                .expect("home");
            assert!(
                bodies.over(height(0.70)).clear(arm, home),
                "arm {arm} at home"
            );
            assert!(
                !bodies.over(height(0.70)).clear(arm, ready),
                "arm {arm} at ready"
            );
        }
    }

    #[test]
    fn a_pose_that_is_not_finite_is_not_clear() {
        let (mut bodies, ready) = bodies("openarm_v2");
        let mut joints = ready[0].clone();
        joints[3] = f64::NAN;
        assert!(!bodies.over(height(0.0)).clear(0, &joints));
    }

    #[test]
    fn the_block_reaches_as_far_to_each_side_and_as_deep_as_the_grid_reaches_ahead() {
        let (mut bodies, _) = bodies("openarm_v2");
        let front = bodies.front;
        let far = Grid::STANDARD.x[1];
        let over = bodies.over(height(0.70));
        let (block, pose) = over.block.as_ref().expect("a block before the body");
        let (low, high) = PlacedShape::new(block, *pose).bounds();
        let sides = [
            (low.x, front),
            (high.x, far),
            (low.y, -far),
            (high.y, far),
            (low.z, 0.70 - far),
            (high.z, 0.70),
        ];
        for (side, (got, want)) in sides.into_iter().enumerate() {
            assert!((got - want).abs() < 1e-9, "side {side}: {got} != {want}");
        }
    }

    #[test]
    fn a_body_that_stands_as_far_ahead_as_the_grid_reaches_leaves_every_pose_clear() {
        let (mut bodies, ready) = bodies("openarm_v2");
        bodies.front = Grid::STANDARD.x[1];
        for (arm, joints) in ready.iter().enumerate() {
            assert!(bodies.over(height(0.70)).clear(arm, joints), "arm {arm}");
        }
    }
}
