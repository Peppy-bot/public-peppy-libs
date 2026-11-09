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
    /// Two chains were given the same base link.
    #[error("two chains start at '{base}'; give each chain its own base link")]
    SharedBase { base: String },

    /// Two bodies resolved to the same name.
    #[error("duplicate body name '{name}'")]
    DuplicateBody { name: String },

    /// A link belongs to two chains.
    #[error("link '{name}' is shared between two chains")]
    SharedLink { name: String },

    /// A link on a moving joint off a chain link has collision geometry, so it is a
    /// finger, and the builder's fingers do not name it.
    #[error(
        "'{link}' rides a moving joint off a chain link; name it in the fingers, with the joint position at which it is shut"
    )]
    UnnamedFinger { link: String },

    /// The builder's fingers name a link that is no finger of a chain.
    #[error("the fingers name '{link}', which rides no moving joint off a chain link")]
    NoSuchFinger { link: String },

    /// A finger's shut position is neither limit of its joint.
    #[error(
        "finger '{link}' is shut at {shut}, which is neither limit of its joint, {lower} or {upper}"
    )]
    ShutOffTheLimits {
        link: String,
        shut: f64,
        lower: f64,
        upper: f64,
    },

    /// An exclusion names a pair the derived rules do not check.
    #[error("the exclusions name '{a}' and '{b}', a pair the model does not check")]
    NotChecked { a: String, b: String },

    /// Supplied clip regions were keyed by a name that is not a collision body.
    #[error("supplied clip regions name '{name}', which is not a collision body")]
    UnknownSuppliedBody { name: String },

    /// A body's pieces could not be fitted from its mesh and clip regions; see
    /// [`GeometryError`] for the specific reason.
    #[error(transparent)]
    Fit(#[from] GeometryError),

    /// A name (e.g. an exclusion) did not resolve to a known body.
    #[error("unknown body '{name}'")]
    UnknownBody { name: String },

    /// A checked-pair spec referenced an unknown body.
    #[error("pair references unknown body '{name}'")]
    UnknownPairBody { name: String },

    /// A checked-pair spec paired a body with itself.
    #[error("pair '{name}' against itself")]
    SelfPair { name: String },

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
