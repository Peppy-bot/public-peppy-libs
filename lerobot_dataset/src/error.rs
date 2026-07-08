use std::path::PathBuf;

use thiserror::Error;

/// Errors from building a [`crate::DatasetConfig`].
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ConfigError {
    #[error("feature {0:?} declared more than once")]
    DuplicateFeature(String),
    #[error("feature {0:?} must have at least one dimension name")]
    EmptyFeature(String),
    #[error("feature key {0:?} collides with a bookkeeping column the writer adds itself")]
    ReservedFeature(String),
    #[error("dataset requires an \"observation.state\" feature")]
    MissingState,
    #[error("dataset requires an \"action\" feature")]
    MissingAction,
    #[error("camera key {0:?} must match observation.images.<name> with <name> in [a-z0-9_]+")]
    InvalidCameraKey(String),
    #[error("video crf {0} is out of the encoder range 0..=63")]
    CrfOutOfRange(u8),
    #[error("video gop must be non-zero")]
    ZeroGop,
}

/// Errors from constructing a [`crate::PixelFrame`] or appending a [`crate::Frame`].
#[derive(Debug, Error, PartialEq, Eq)]
pub enum FrameError {
    #[error("{feature} expects {expected} values, got {got}")]
    VectorLen {
        feature: String,
        expected: usize,
        got: usize,
    },
    #[error("pixel buffer for {width}x{height} {encoding} must be {expected} bytes, got {got}")]
    PixelBufferLen {
        encoding: &'static str,
        width: u32,
        height: u32,
        expected: usize,
        got: usize,
    },
    #[error("mjpeg buffer does not start with a JPEG SOI marker")]
    NotAJpeg,
    #[error("mjpeg frame failed to decode: {0}")]
    JpegDecode(String),
    #[error("frame is missing feature {0:?}")]
    MissingFeature(String),
    #[error("frame provides feature {0:?} more than once")]
    DuplicateValue(String),
    #[error("frame is missing camera {0:?}")]
    MissingCamera(String),
    #[error("value provided for a feature or camera the dataset does not declare")]
    UndeclaredValue,
}

/// Errors from the ffmpeg/ffprobe video pipeline.
#[derive(Debug, Error)]
pub enum VideoError {
    #[error(
        "ffmpeg not found on PATH; install ffmpeg (with libx264/libsvtav1) or add it to the container image"
    )]
    FfmpegNotFound(#[source] std::io::Error),
    #[error("ffprobe not found on PATH; it ships with ffmpeg")]
    FfprobeNotFound(#[source] std::io::Error),
    #[error("ffmpeg has no {0} encoder; install a build with it enabled")]
    EncoderUnavailable(&'static str),
    #[error("encoder for camera {camera:?} exited with {status}: {stderr}")]
    EncoderExited {
        camera: String,
        status: std::process::ExitStatus,
        stderr: String,
    },
    #[error("camera {camera:?} video has {probed} frames, expected {expected}")]
    FrameCountMismatch {
        camera: String,
        expected: u64,
        probed: u64,
    },
    #[error("ffprobe on camera {camera:?} video failed: {detail}")]
    ProbeFailed { camera: String, detail: String },
    #[error("concatenating episode video for camera {camera:?} failed with {status}: {stderr}")]
    ConcatFailed {
        camera: String,
        status: std::process::ExitStatus,
        stderr: String,
    },
}

/// Any failure from the dataset writer.
#[derive(Debug, Error)]
pub enum Error {
    #[error(transparent)]
    Config(#[from] ConfigError),
    #[error(transparent)]
    Frame(#[from] FrameError),
    #[error(transparent)]
    Video(#[from] VideoError),
    #[error("parquet failure at {path}")]
    Parquet {
        path: PathBuf,
        #[source]
        source: parquet::errors::ParquetError,
    },
    #[error("arrow failure at {path}")]
    Arrow {
        path: PathBuf,
        #[source]
        source: arrow::error::ArrowError,
    },
    #[error("io failure at {path}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("dataset root {0} already contains files")]
    RootNotEmpty(PathBuf),
    #[error("episode has zero frames; call abort() to drop an empty episode")]
    EmptyEpisode,
}

impl Error {
    pub(crate) fn io(path: impl Into<PathBuf>) -> impl FnOnce(std::io::Error) -> Error {
        let path = path.into();
        move |source| Error::Io { path, source }
    }

    pub(crate) fn parquet(
        path: impl Into<PathBuf>,
    ) -> impl FnOnce(parquet::errors::ParquetError) -> Error {
        let path = path.into();
        move |source| Error::Parquet { path, source }
    }

    pub(crate) fn arrow(
        path: impl Into<PathBuf>,
    ) -> impl FnOnce(arrow::error::ArrowError) -> Error {
        let path = path.into();
        move |source| Error::Arrow { path, source }
    }
}
