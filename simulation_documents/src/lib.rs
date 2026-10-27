//! The documents of the simulation side.
//!
//! A world (`world/v1`, [`World`]) says where simulated robots work: the
//! catalogue release and the scene, the robots with their joins and stands,
//! the scene objects moved or removed, the spawned objects, the lighting, the
//! materials, the daylight and the named locations. A simulation keeps a [`SetupRecord`] of
//! its setup calls, and the world document of a live simulation is that record
//! with the robots of the stack ([`World::from_setup`]). A [`Judge`] checks
//! ground truth in a live world and gives a verdict.
//!
//! The world document carries the root key `simulation_schema`, and takes the
//! root key rule, the manifest and the digest from `documents_core`. The
//! setup record, the start setup, the judges and the verdicts are JSON5 texts
//! of the simulation side, which carry no root key.

mod compose;
mod daylight;
mod error;
mod extends;
mod geometry;
mod judge;
mod names;
mod setup_record;
mod text;
mod world;

pub use compose::{StackRobot, StartSetup};
pub use daylight::{Date, Daylight, TimeOfDay, Weather};
pub use documents_core::{Digest, Manifest, Revision};
pub use error::Error;
pub use geometry::{Offset, Placement, Point, Pose};
pub use judge::{
    Band, BeltLine, Bound, Bounds, Check, ContentsOn, Control, Count, Holding, Judge, Landmark,
    LandmarkOn, NearPoint, Place, Reachable, Target, Verdict, Verdicts, WithinM,
};
pub use names::{Name, RobotName};
pub use setup_record::SetupRecord;
pub use world::{
    Catalogue, Cone, Feature, Finish, Join, Light, Location, Material, Object, Physics, Robot,
    Scene, SceneObject, Stand, World, WorldDocument,
};
