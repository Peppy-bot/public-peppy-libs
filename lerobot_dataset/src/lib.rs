//! Native writer for [LeRobot v3 datasets](https://huggingface.co/docs/lerobot/lerobot-dataset-v3).
//!
//! Data in, files out: tabular data and metadata go through arrow/parquet,
//! video through an ffmpeg subprocess (ffmpeg and ffprobe must be on PATH).
//! No robotics framework dependencies.
//!
//! Crash contract: the dataset on disk is valid and loadable after every
//! completed episode; the episode in flight when a process dies is lost.
//! Output compliance is locked by the Python-loader harness in `compliance/`.

mod atomic;
mod config;
mod data;
mod error;
mod frame;
#[cfg(test)]
mod golden_tests;
mod layout;
mod meta;
mod video;
mod writer;

pub use config::{
    CameraId, CameraSpec, DatasetConfig, DatasetConfigBuilder, SourceEncoding, VectorId,
    VideoCodec, VideoSettings,
};
pub use error::{ConfigError, Error, FrameError, VideoError};
pub use frame::{Frame, PixelFrame};
pub use writer::{DatasetSummary, DatasetWriter, EpisodeMeta, EpisodeWriter};
