//! The crate's error types.

use collision_geometry::GeometryError;

/// A failure from building or querying a
/// [`SelfCollisionModel`](crate::SelfCollisionModel). The query variants
/// are distinct so a caller can react to each (a velocity-barrier caller treats
/// [`WitnessesCoincide`](Self::WitnessesCoincide) as a touch and holds or
/// escapes, while a bad-input variant is a genuine fault to surface).
#[derive(Debug, thiserror::Error)]
pub enum CollisionError {
    /// A query was handed a non-finite joint value (or threshold).
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
    /// Both chains were given the same base link; the self-collision model needs two.
    #[error(
        "left and right base links are both '{base}'; the self-collision model needs two chains"
    )]
    IdenticalBases { base: String },

    /// Two bodies resolved to the same name.
    #[error("duplicate body name '{name}'")]
    DuplicateBody { name: String },

    /// A link belongs to both chains.
    #[error("link '{name}' is shared between the two chains")]
    SharedLink { name: String },

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

/// Lower-level helpers report opaque `String` reasons; fold them into the
/// [`Geometry`](BuildError::Geometry) catch-all so `?` propagates them.
impl From<String> for BuildError {
    fn from(reason: String) -> Self {
        BuildError::Geometry(reason)
    }
}

/// Building the per-arm `srs_model` chains is part of assembly; fold its error
/// into the geometry catch-all.
impl From<srs_model::SrsError> for BuildError {
    fn from(source: srs_model::SrsError) -> Self {
        BuildError::Geometry(source.to_string())
    }
}
