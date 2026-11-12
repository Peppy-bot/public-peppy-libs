//! The crate's error types.

use collision_geometry::GeometryError;

/// A failure from building or querying a
/// [`SelfCollisionModel`](crate::SelfCollisionModel). The query variants
/// are distinct so a caller can react to each (a velocity-barrier caller treats
/// [`WitnessesCoincide`](Self::WitnessesCoincide) as a touch and holds or
/// escapes, while a bad-input variant is a genuine fault to surface).
#[derive(Debug, thiserror::Error)]
pub enum CollisionError {
    /// A query was handed a non-finite joint value, or a bound that is not a number.
    #[error("non-finite value in query input")]
    NonFinite,

    /// The model has no checked pairs, so there is nothing to measure.
    #[error("no checked pairs to evaluate")]
    NoPairs,

    /// The nearest pair's hull cores touch degenerately (the rounded surfaces
    /// overlap by the summed radii), so no separating direction, and thus no
    /// distance gradient, is defined.
    #[error("witnesses coincide (d={distance:+.4}); distance gradient undefined")]
    WitnessesCoincide { distance: f64 },

    /// Model construction failed; see [`BuildError`] for the specific reason.
    #[error(transparent)]
    Build(#[from] BuildError),
}

/// Why building a [`SelfCollisionModel`](crate::SelfCollisionModel) failed.
/// Each semantic failure is its own variant so a caller (and a test) can match the
/// reason structurally; the lower-level geometry/URDF/mesh failures share the
/// [`Geometry`](Self::Geometry) catch-all, since they carry an opaque reason.
#[derive(Debug, thiserror::Error)]
pub enum BuildError {
    /// Two arms of the description start their chains at one base link.
    #[error(
        "the arms '{first}' and '{second}' both start at '{base}'; the SRDF gives each arm \
         a chain from a base link of its own"
    )]
    SharedBase {
        first: String,
        second: String,
        base: String,
    },

    /// The description gives another count of arms than the model holds chains.
    #[error("the description gives {found} arms and the model holds {expected}")]
    Arms { expected: usize, found: usize },

    /// An arm of the description has another count of joints than the model's chains.
    #[error("arm '{arm}' of the description has {found} joints and the model's chains {expected}")]
    ArmJoints {
        arm: String,
        expected: usize,
        found: usize,
    },

    /// Two bodies resolved to the same name.
    #[error("duplicate body name '{name}'")]
    DuplicateBody { name: String },

    /// A link belongs to two chains.
    #[error("link '{name}' is shared between two chains")]
    SharedLink { name: String },

    /// A link on a moving joint off a chain link has collision geometry, so it is a
    /// finger, and no gripper of the description names its joint.
    #[error(
        "'{link}' rides a moving joint off a chain link; add its joint to its arm's gripper group in the SRDF"
    )]
    UnnamedFinger { link: String },

    /// A finger of a gripper of the description, with collision geometry, rides no
    /// moving joint off a chain link.
    #[error("the gripper finger '{link}' rides no moving joint off a chain link")]
    NoSuchFinger { link: String },

    /// A body's pieces could not be fitted from its mesh; see [`GeometryError`] for the
    /// specific reason.
    #[error(transparent)]
    Fit(#[from] GeometryError),

    /// A lower-level geometry, URDF, or mesh failure during assembly.
    #[error("{0}")]
    Geometry(String),
}

/// Why [`SelfCollisionModel::fingers`](crate::SelfCollisionModel::fingers) cannot give a
/// part a chain carries as a finger of that chain: the model and the chain come from
/// different robots. Each variant names the part, by its link.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum FingerError {
    /// The model holds no body of the part's name.
    #[error("the self model holds no body '{link}'; give the parts of this robot's chain")]
    NoBody { link: String },

    /// The model holds the part as a body that is no gripper finger.
    #[error(
        "the self model holds '{link}' as no gripper finger; give the parts of this robot's chain"
    )]
    NotAFinger { link: String },

    /// The model holds the part as no finger of the chain.
    #[error("the self model holds '{link}' as no finger of chain {chain}; give that chain's parts")]
    OtherChain { link: String, chain: usize },

    /// The model hangs the finger off another link of the chain than the chain carries it on.
    #[error(
        "the self model hangs '{link}' off link {model} of its chain and the chain carries it on link {chain}"
    )]
    OtherLink {
        link: String,
        model: usize,
        chain: usize,
    },
}

/// Lower-level helpers report opaque `String` reasons; fold them into the
/// [`Geometry`](BuildError::Geometry) catch-all so `?` propagates them.
impl From<String> for BuildError {
    fn from(reason: String) -> Self {
        BuildError::Geometry(reason)
    }
}

/// Building each chain is part of assembly; fold its error into the geometry
/// catch-all.
impl From<chain_kinematics::ChainError> for BuildError {
    fn from(source: chain_kinematics::ChainError) -> Self {
        BuildError::Geometry(source.to_string())
    }
}
