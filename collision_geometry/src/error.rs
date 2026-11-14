//! The crate's error type.

/// Why fitting a body's pieces, or building the geometry of a chain, failed. Each
/// semantic failure is its own variant so a caller (and a test) can match the reason;
/// the lower-level URDF and mesh failures share [`Geometry`](Self::Geometry), since
/// they carry an opaque reason.
#[derive(Debug, thiserror::Error)]
pub enum GeometryError {
    /// A body's own mesh could not be fitted (an empty, collinear, or coplanar cloud,
    /// or one no plane budget could circumscribe).
    #[error("'{body}' could not be fitted from its mesh: {reason}")]
    DegenerateBody { body: String, reason: String },
    /// The pieces of a body do not conservatively contain a mesh they stand in for
    /// (the pieces leave part of its surface uncovered).
    #[error("the pieces of '{body}' do not contain the mesh they stand in for: {kind}")]
    HullMissesMesh {
        body: String,
        kind: ContainmentFailure,
    },
    /// A `<collision>` element of `link` that the fit cannot hold: a shape it does not
    /// read, or a primitive whose size is not positive and finite.
    #[error("the collision of '{link}' cannot be fitted: {reason}")]
    UnsupportedCollision { link: String, reason: String },
    /// A link to fit in the frame of the URDF root that a movable joint carries, so no
    /// fixed joints alone join it to the root.
    #[error(
        "'{link}' hangs below a movable joint: fixed joints alone do not join it to the URDF root"
    )]
    NotFixedToTheRoot { link: String },
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
