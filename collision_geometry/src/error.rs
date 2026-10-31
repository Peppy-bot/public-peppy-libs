//! The crate's error type.

/// Why fitting a body's pieces, or building the geometry of a chain, failed. Each
/// semantic failure is its own variant so a caller (and a test) can match the reason;
/// the lower-level URDF and mesh failures share [`Geometry`](Self::Geometry), since
/// they carry an opaque reason.
#[derive(Debug, thiserror::Error)]
pub enum GeometryError {
    /// Supplied clip regions for a body were an empty list.
    #[error("supplied clip regions for '{body}' are empty; provide at least one region")]
    EmptyRegions { body: String },
    /// A body's own mesh could not be fitted (an empty, collinear, or coplanar cloud,
    /// or one no plane budget could circumscribe).
    #[error("'{body}' could not be fitted from its mesh: {reason}")]
    DegenerateBody { body: String, reason: String },
    /// A clip region caught too little of its body's mesh to bound a solid (an empty,
    /// collinear, or coplanar clipped slice).
    #[error("clip region {index} of '{body}' does not bound a solid slice of its mesh: {reason}")]
    DegenerateRegion {
        body: String,
        index: usize,
        reason: String,
    },
    /// The hulls fitted to a body's clip regions do not conservatively contain its
    /// mesh (the regions leave part of the surface uncovered).
    #[error("the region hulls for '{body}' do not contain its mesh: {kind}")]
    HullMissesMesh {
        body: String,
        kind: ContainmentFailure,
    },
    /// A `<collision>` element of `link` that the fit cannot hold: a shape it does not
    /// read, or a primitive whose size is not positive and finite.
    #[error("the collision of '{link}' cannot be fitted: {reason}")]
    UnsupportedCollision { link: String, reason: String },
    /// The bounds of a clip region of a body, by its index among the bounds given,
    /// that span no volume.
    #[error("clip region {index} of '{body}' spans no volume: {reason}")]
    InvalidRegion {
        body: String,
        index: usize,
        reason: String,
    },
    /// The chain itself could not be built from the URDF.
    #[error(transparent)]
    Chain(#[from] chain_kinematics::ChainError),
    /// A padding scale that is not a positive finite number, or a padding offset that
    /// is not a non-negative finite number.
    #[error(
        "padding needs a positive finite scale and a non-negative finite offset, got scale {scale} and offset {offset}"
    )]
    Padding { scale: f64, offset: f64 },
    /// A lower-level geometry, URDF, or mesh failure.
    #[error("{0}")]
    Geometry(String),
}

/// Which containment check a supplied hull failed (see
/// [`GeometryError::HullMissesMesh`]).
#[derive(Debug, thiserror::Error)]
pub enum ContainmentFailure {
    /// A mesh vertex lies outside every supplied piece.
    #[error("a vertex lies outside every piece")]
    VertexOutside,
    /// A mesh face slopes out through the gap between pieces.
    #[error("a face escapes the union of pieces")]
    FaceEscapes,
}

/// Lower-level helpers report opaque `String` reasons; fold them into the
/// [`Geometry`](GeometryError::Geometry) catch-all so `?` propagates them.
impl From<String> for GeometryError {
    fn from(reason: String) -> Self {
        GeometryError::Geometry(reason)
    }
}
