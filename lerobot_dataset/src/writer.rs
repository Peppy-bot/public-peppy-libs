//! Dataset lifecycle. `DatasetWriter` owns the on-disk state; an
//! `EpisodeWriter` mutably borrows it, so the type system rules out a second
//! open episode, frames outside an episode, and finalize with an episode
//! open.
//!
//! Commit order at `end()`: tasks, data parquet, video concat, episodes row,
//! then info.json and stats.json. Every step lands via atomic rename, and the
//! episodes row plus info totals are the commit point: artifacts from a crash
//! mid-`end()` are unreferenced and invisible to the loader.

use std::path::{Path, PathBuf};

use crate::config::{CameraId, DatasetConfig, VectorId};
use crate::data::EpisodeData;
use crate::error::{Error, FrameError};
use crate::frame::Frame;
use crate::layout::{FileSlot, data_path, episodes_path, mb_to_bytes, video_path};
use crate::meta::episodes::{EpisodeRow, FeatureStatsEntry, VideoLocation};
use crate::meta::info::{Totals, build_info_json};
use crate::meta::stats::{
    DepthStatsAccumulator, FeatureStats, ImageStatsAccumulator, VideoStatsAccumulator, aggregate,
    vector_stats,
};
use crate::meta::tasks::TaskTable;
use crate::meta::{build_stats_json, write_info, write_stats};
use crate::video::concat::append_or_start;
use crate::video::encoder::{EncodeMode, EpisodeEncoder};
use crate::video::probe::probe_toolchain;
use crate::video::sample::downsampled_rgb;

const EPISODE_TMP_DIR: &str = ".episode-tmp";

/// Names of the bookkeeping features, in canonical (info.json) order.
const DEFAULT_FEATURE_KEYS: [&str; 5] = [
    "timestamp",
    "frame_index",
    "episode_index",
    "index",
    "task_index",
];

struct VideoFileState {
    slot: FileSlot,
    /// Frames already committed to the file at `slot`.
    frames: u64,
}

pub struct DatasetWriter {
    root: PathBuf,
    config: DatasetConfig,
    tasks: TaskTable,
    total_episodes: u64,
    total_frames: u64,
    data_slot: FileSlot,
    episodes_slot: FileSlot,
    videos: Vec<VideoFileState>,
    /// Running aggregate per feature, canonical order; None before episode 0.
    aggregated: Option<Vec<FeatureStatsEntry>>,
}

#[derive(Debug)]
pub struct EpisodeMeta {
    pub episode_index: u64,
    pub length: u64,
    /// Root-relative paths of chunk files that became immutable during this
    /// episode's commit (a data/video/episodes chunk rolled over, so the
    /// prior file is final and will never be rewritten). A mirror can upload
    /// exactly these; empty on episodes that did not trigger a rollover.
    pub finalized_files: Vec<PathBuf>,
}

#[derive(Debug)]
pub struct DatasetSummary {
    pub root: PathBuf,
    pub total_episodes: u64,
    pub total_frames: u64,
    pub total_tasks: u64,
}

impl DatasetWriter {
    /// Creates the v3 tree under `root` (which must not already contain
    /// files), probing the ffmpeg toolchain first when cameras are declared.
    pub fn create(root: impl Into<PathBuf>, config: DatasetConfig) -> Result<Self, Error> {
        let root = root.into();
        if root.exists()
            && std::fs::read_dir(&root)
                .map_err(Error::io(&root))?
                .next()
                .is_some()
        {
            return Err(Error::RootNotEmpty(root));
        }
        if !config.cameras.is_empty() {
            let needs_depth = config.cameras.iter().any(|c| c.is_depth());
            probe_toolchain(config.video.codec, needs_depth)?;
        }
        std::fs::create_dir_all(&root).map_err(Error::io(&root))?;

        let videos = config
            .cameras
            .iter()
            .map(|_| VideoFileState {
                slot: FileSlot::default(),
                frames: 0,
            })
            .collect();
        let writer = Self {
            root,
            config,
            tasks: TaskTable::default(),
            total_episodes: 0,
            total_frames: 0,
            data_slot: FileSlot::default(),
            episodes_slot: FileSlot::default(),
            videos,
            aggregated: None,
        };
        writer.tasks.write(&writer.root)?;
        write_info(
            &writer.root,
            &build_info_json(&writer.config, &writer.totals()),
        )?;
        write_stats(&writer.root, &build_stats_json(&[]))?;
        Ok(writer)
    }

    pub fn config(&self) -> &DatasetConfig {
        &self.config
    }

    pub fn begin_episode(&mut self, task: &str) -> Result<EpisodeWriter<'_>, Error> {
        let tmp_dir = self.root.join(EPISODE_TMP_DIR);
        if tmp_dir.exists() {
            std::fs::remove_dir_all(&tmp_dir).map_err(Error::io(&tmp_dir))?;
        }

        let fps = self.config.fps.get();
        let mut encoders = Vec::new();
        for camera in &self.config.cameras {
            let temp = tmp_dir.join(format!("{}.mp4", camera.key));
            let mode = match camera.depth {
                Some(depth) => EncodeMode::Depth(depth),
                None => EncodeMode::Color(self.config.video),
            };
            let encoder = match EpisodeEncoder::spawn(&camera.key, &camera.spec, &mode, fps, temp) {
                Ok(encoder) => encoder,
                Err(error) => {
                    drop(encoders);
                    return Err(error.into());
                }
            };
            encoders.push(encoder);
        }

        let vector_dims: Vec<usize> = self
            .config
            .vectors
            .iter()
            .map(|f| f.dim_names.len())
            .collect();
        Ok(EpisodeWriter {
            episode_index: self.total_episodes,
            task: task.to_string(),
            vectors: vector_dims.iter().map(|_| Vec::new()).collect(),
            vector_dims,
            image_stats: self
                .config
                .cameras
                .iter()
                .map(|camera| match &camera.depth {
                    None => VideoStatsAccumulator::Color(ImageStatsAccumulator::new()),
                    Some(depth) => VideoStatsAccumulator::Depth(DepthStatsAccumulator::new(
                        depth.quantization.depth_min_m * 1000.0,
                        depth.quantization.depth_max_m * 1000.0,
                    )),
                })
                .collect(),
            encoders,
            frames: 0,
            committed: false,
            dataset: self,
        })
    }

    /// Nothing is pending between episodes; finalize just reports totals.
    pub fn finalize(self) -> Result<DatasetSummary, Error> {
        Ok(DatasetSummary {
            total_episodes: self.total_episodes,
            total_frames: self.total_frames,
            total_tasks: self.tasks.len(),
            root: self.root,
        })
    }

    fn totals(&self) -> Totals {
        Totals {
            episodes: self.total_episodes,
            frames: self.total_frames,
            tasks: self.tasks.len(),
        }
    }

    fn roll_if_full(&self, slot: FileSlot, relative: &Path, limit_mb: u64) -> FileSlot {
        let full = std::fs::metadata(self.root.join(relative))
            .map(|m| m.len() >= mb_to_bytes(limit_mb))
            .unwrap_or(false);
        if full { slot.next() } else { slot }
    }
}

pub struct EpisodeWriter<'d> {
    dataset: &'d mut DatasetWriter,
    episode_index: u64,
    task: String,
    /// Frame-major flattened values per vector feature.
    vectors: Vec<Vec<f32>>,
    vector_dims: Vec<usize>,
    image_stats: Vec<VideoStatsAccumulator>,
    encoders: Vec<EpisodeEncoder>,
    frames: u64,
    /// True once end() or abort() has taken responsibility for the episode.
    committed: bool,
}

impl EpisodeWriter<'_> {
    pub fn frame_count(&self) -> u64 {
        self.frames
    }

    /// Validates the frame against the schema, buffers the numeric row, and
    /// streams pixels into each camera's encoder. Returns the frame's index
    /// within the episode.
    pub fn add_frame(&mut self, frame: Frame<'_>) -> Result<u64, Error> {
        let config = &self.dataset.config;
        let mut vector_seen = vec![false; config.vectors.len()];
        for (id, values) in frame.vectors {
            let VectorId(index) = *id;
            if index >= vector_seen.len() {
                return Err(FrameError::UndeclaredValue.into());
            }
            if std::mem::replace(&mut vector_seen[index], true) {
                return Err(FrameError::DuplicateValue(config.vectors[index].key.clone()).into());
            }
            if values.len() != self.vector_dims[index] {
                return Err(FrameError::VectorLen {
                    feature: config.vectors[index].key.clone(),
                    expected: self.vector_dims[index],
                    got: values.len(),
                }
                .into());
            }
        }
        if let Some(missing) = vector_seen.iter().position(|seen| !seen) {
            return Err(FrameError::MissingFeature(config.vectors[missing].key.clone()).into());
        }

        let mut image_seen = vec![false; config.cameras.len()];
        for (id, pixels) in frame.images {
            let CameraId(index) = *id;
            if index >= image_seen.len() {
                return Err(FrameError::UndeclaredValue.into());
            }
            if std::mem::replace(&mut image_seen[index], true) {
                return Err(FrameError::DuplicateValue(config.cameras[index].key.clone()).into());
            }
            let is_z16 = pixels.encoding() == crate::config::SourceEncoding::Z16;
            if is_z16 != config.cameras[index].is_depth() {
                return Err(
                    FrameError::CameraKindMismatch(config.cameras[index].key.clone()).into(),
                );
            }
        }
        if let Some(missing) = image_seen.iter().position(|seen| !seen) {
            return Err(FrameError::MissingCamera(config.cameras[missing].key.clone()).into());
        }

        for (VectorId(index), values) in frame.vectors {
            self.vectors[*index].extend_from_slice(values);
        }
        for (CameraId(index), pixels) in frame.images {
            let camera = &config.cameras[*index];
            match (&mut self.image_stats[*index], &camera.depth) {
                (VideoStatsAccumulator::Color(stats), None) => {
                    stats.add_frame(&downsampled_rgb(&camera.spec, pixels)?);
                }
                (VideoStatsAccumulator::Depth(stats), Some(depth)) => {
                    stats.add_frame(crate::video::depth::z16_to_mm(
                        pixels.bytes,
                        depth.depth_unit_m,
                    ));
                }
                _ => unreachable!("stats variant is built to match camera kind"),
            }
            self.encoders[*index].write_frame(pixels)?;
        }

        let frame_index = self.frames;
        self.frames += 1;
        Ok(frame_index)
    }

    /// Commits the episode. The dataset on disk is fully valid when this
    /// returns.
    pub fn end(mut self) -> Result<EpisodeMeta, Error> {
        self.committed = true;
        if self.frames == 0 {
            self.encoders.clear();
            return Err(Error::EmptyEpisode);
        }
        let episode_index = self.episode_index;
        let frames = self.frames;
        let task = std::mem::take(&mut self.task);
        let vectors = std::mem::take(&mut self.vectors);
        let vector_dims = std::mem::take(&mut self.vector_dims);
        let image_stats = std::mem::take(&mut self.image_stats);
        let encoders = std::mem::take(&mut self.encoders);
        let dataset = &mut *self.dataset;
        let fps = dataset.config.fps.get();

        let mut episode_videos: Vec<(PathBuf, u64)> = Vec::new();
        let mut open_encoders = encoders.into_iter();
        for encoder in open_encoders.by_ref() {
            match encoder.finish() {
                Ok(done) => episode_videos.push(done),
                Err(error) => {
                    drop(open_encoders);
                    return Err(error.into());
                }
            }
        }
        drop(open_encoders);

        let (task_index, task_is_new) = dataset.tasks.intern(&task);
        if task_is_new {
            dataset.tasks.write(&dataset.root)?;
        }

        let timestamps: Vec<f32> = (0..frames)
            .map(|k| (k as f64 / fps as f64) as f32)
            .collect();
        let frame_indices: Vec<i64> = (0..frames as i64).collect();
        let first_global_index = dataset.total_frames as i64;

        let episode_stats = build_episode_stats(
            &dataset.config,
            &vectors,
            &vector_dims,
            image_stats,
            &timestamps,
            episode_index as i64,
            first_global_index,
            task_index,
        );

        let mut finalized_files: Vec<PathBuf> = Vec::new();
        let previous_data_slot = dataset.data_slot;
        dataset.data_slot = dataset.roll_if_full(
            dataset.data_slot,
            &data_path(dataset.data_slot),
            dataset.config.data_files_size_in_mb,
        );
        if dataset.data_slot != previous_data_slot {
            finalized_files.push(data_path(previous_data_slot));
        }
        let (dataset_from_index, dataset_to_index) = crate::data::append_episode(
            &dataset.root,
            &dataset.config,
            dataset.data_slot,
            &EpisodeData {
                vectors,
                timestamps: timestamps.clone(),
                frame_indices: frame_indices.clone(),
                episode_index: episode_index as i64,
                first_global_index,
                task_index,
            },
        )?;

        let mut video_locations: Vec<(String, VideoLocation)> = Vec::new();
        for (camera_index, (temp_path, video_frames)) in episode_videos.iter().enumerate() {
            debug_assert_eq!(*video_frames, frames);
            let key = dataset.config.cameras[camera_index].key.clone();
            let state = &mut dataset.videos[camera_index];
            let current = dataset.root.join(video_path(&key, state.slot));
            let roll = std::fs::metadata(&current)
                .map(|m| m.len() >= mb_to_bytes(dataset.config.video_files_size_in_mb))
                .unwrap_or(false);
            if roll {
                finalized_files.push(video_path(&key, state.slot));
                state.slot = state.slot.next();
                state.frames = 0;
            }
            let shared = dataset.root.join(video_path(&key, state.slot));
            append_or_start(&shared, temp_path, &key, fps)?;
            let from_timestamp = state.frames as f64 / fps as f64;
            let to_timestamp = (state.frames + frames) as f64 / fps as f64;
            state.frames += frames;
            video_locations.push((
                key,
                VideoLocation {
                    slot: state.slot,
                    from_timestamp,
                    to_timestamp,
                },
            ));
        }
        let _ = std::fs::remove_dir_all(dataset.root.join(EPISODE_TMP_DIR));

        let previous_episodes_slot = dataset.episodes_slot;
        dataset.episodes_slot = dataset.roll_if_full(
            dataset.episodes_slot,
            &episodes_path(dataset.episodes_slot),
            dataset.config.data_files_size_in_mb,
        );
        if dataset.episodes_slot != previous_episodes_slot {
            finalized_files.push(episodes_path(previous_episodes_slot));
        }
        crate::meta::episodes::append_episode_row(
            &dataset.root,
            &EpisodeRow {
                episode_index: episode_index as i64,
                task,
                length: frames as i64,
                data_slot: dataset.data_slot,
                dataset_from_index,
                dataset_to_index,
                videos: video_locations,
                stats: episode_stats.clone(),
                episodes_slot: dataset.episodes_slot,
            },
        )?;

        dataset.total_episodes += 1;
        dataset.total_frames += frames;
        dataset.aggregated = Some(match dataset.aggregated.take() {
            None => episode_stats,
            Some(running) => running
                .into_iter()
                .zip(&episode_stats)
                .map(|(current, episode)| FeatureStatsEntry {
                    key: current.key.clone(),
                    is_image: current.is_image,
                    stats: aggregate(&[&current.stats, &episode.stats]),
                })
                .collect(),
        });
        write_info(
            &dataset.root,
            &build_info_json(&dataset.config, &dataset.totals()),
        )?;
        write_stats(
            &dataset.root,
            &build_stats_json(dataset.aggregated.as_deref().unwrap_or(&[])),
        )?;

        Ok(EpisodeMeta {
            episode_index,
            length: frames,
            finalized_files,
        })
    }

    /// Discards the episode: encoders killed, temp files removed, nothing on
    /// disk references it.
    pub fn abort(mut self) {
        self.committed = true;
        self.encoders.clear();
        let _ = std::fs::remove_dir_all(self.dataset.root.join(EPISODE_TMP_DIR));
    }
}

/// Dropping without `end`/`abort` discards the episode like `abort` (each
/// encoder's own Drop kills its ffmpeg and removes its temp file) and warns,
/// since losing frames silently is never intended.
impl Drop for EpisodeWriter<'_> {
    fn drop(&mut self) {
        if !self.committed {
            tracing::warn!(
                episode_index = self.episode_index,
                frames = self.frames,
                "episode writer dropped without end() or abort(); episode discarded"
            );
        }
    }
}

fn scalar_stats(values: impl Iterator<Item = f64>) -> FeatureStats {
    let rows: Vec<Vec<f64>> = values.map(|v| vec![v]).collect();
    vector_stats(&rows, 1)
}

/// Per-episode stats for every feature in canonical order: declared vectors,
/// cameras, then the five bookkeeping columns.
#[allow(clippy::too_many_arguments)]
fn build_episode_stats(
    config: &DatasetConfig,
    vectors: &[Vec<f32>],
    vector_dims: &[usize],
    image_stats: Vec<VideoStatsAccumulator>,
    timestamps: &[f32],
    episode_index: i64,
    first_global_index: i64,
    task_index: i64,
) -> Vec<FeatureStatsEntry> {
    let frames = timestamps.len();
    let mut entries = Vec::new();
    for ((feature, values), &dim) in config.vectors.iter().zip(vectors).zip(vector_dims) {
        let rows: Vec<Vec<f64>> = values
            .chunks_exact(dim)
            .map(|row| row.iter().map(|&v| v as f64).collect())
            .collect();
        entries.push(FeatureStatsEntry {
            key: feature.key.clone(),
            is_image: false,
            stats: vector_stats(&rows, dim),
        });
    }
    for (camera, accumulator) in config.cameras.iter().zip(image_stats) {
        entries.push(FeatureStatsEntry {
            key: camera.key.clone(),
            is_image: true,
            stats: accumulator.finish(),
        });
    }
    let default_stats = [
        scalar_stats(timestamps.iter().map(|&t| t as f64)),
        scalar_stats((0..frames as i64).map(|i| i as f64)),
        scalar_stats(std::iter::repeat_n(episode_index as f64, frames)),
        scalar_stats((0..frames as i64).map(|i| (first_global_index + i) as f64)),
        scalar_stats(std::iter::repeat_n(task_index as f64, frames)),
    ];
    for (key, stats) in DEFAULT_FEATURE_KEYS.iter().zip(default_stats) {
        entries.push(FeatureStatsEntry {
            key: (*key).to_string(),
            is_image: false,
            stats,
        });
    }
    entries
}
