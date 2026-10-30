//! The documents of the simulation side.
//!
//! A world (`world/v1`, [`World`]) says where simulated robots work: the
//! catalogue release and the scene, the robots with their joins and stands,
//! the scene objects moved or removed, the spawned objects, the lighting, the
//! materials, the daylight and the named locations. A world pattern
//! ([`WorldPattern`]) is a world with samplers, which a [`Seed`] expands into
//! a concrete world ([`Expansion`]). A sweep (`sweep/v1`, [`Sweep`]) runs one
//! workflow revision in many worlds of its world patterns. A simulation keeps a [`SetupRecord`] of
//! its setup calls, and the world document of a live simulation is that record
//! with the robots of the stack ([`World::from_setup`]). A [`Judge`] checks
//! ground truth in a live world and gives a verdict. A [`ClaimHolder`] names
//! the call of an applier that holds the setup claim of a simulation.
//!
//! The world document carries the root key `simulation_schema`, and takes the
//! root key rule, the manifest and the digest from `documents_core`. The
//! setup record, the start setup, the judges and the verdicts are JSON5 texts
//! of the simulation side, which carry no root key.

mod claim;
mod compose;
mod daylight;
mod error;
mod extends;
mod geometry;
mod judge;
mod names;
mod pattern;
mod sampler;
mod setup_record;
mod stream;
mod sweep;
mod text;
mod world;

pub use claim::{ApplyFailureKind, ClaimCall, ClaimHolder};
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
pub use pattern::{
    Axis, Expansion, MAX_COUNT, Measure, Measurement, Measurements, MinSpacing, PlacementKind,
    PlacementSite, StandAt, WorkCellParameter, WorkCellParameters, WorldPattern,
};
pub use sampler::{Sampler, Segment, SitePath, SurfaceRef};
pub use setup_record::SetupRecord;
pub use stream::{Seed, Stream};
pub use sweep::{
    Expected, Fault, FaultAction, FaultExpected, FaultWhen, MAX_PUSH_S, Push, SWEEP, Seeds,
    StepRef, Sweep, SweepDocument, SweepJudges, SweptWorld,
};
/// The name of a work cell parameter: a name of the workflow side, since the
/// bundle of a signed-off sweep carries each parameter to the workflow
/// director.
pub use workflow_documents::Name as ParameterName;
pub use world::{
    Catalogue, Cone, Feature, Finish, Join, Light, Location, Material, Object, Physics, Robot,
    Scene, SceneObject, Stand, World, WorldDocument,
};
