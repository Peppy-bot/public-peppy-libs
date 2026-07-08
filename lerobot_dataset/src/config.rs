use std::num::{NonZeroU32, NonZeroU64};

use crate::error::ConfigError;

pub const STATE_KEY: &str = "observation.state";
pub const ACTION_KEY: &str = "action";
pub const CAMERA_KEY_PREFIX: &str = "observation.images.";

/// Bookkeeping columns the writer produces itself; user features may not shadow them.
pub const RESERVED_COLUMNS: [&str; 5] = [
    "timestamp",
    "frame_index",
    "episode_index",
    "index",
    "task_index",
];

/// Pixel layout of the buffers the caller will feed for a camera.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceEncoding {
    Rgb8,
    Bgr8,
    Yuyv,
    Mjpeg,
}

#[derive(Debug, Clone, Copy)]
pub struct CameraSpec {
    pub width: NonZeroU32,
    pub height: NonZeroU32,
    pub source: SourceEncoding,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoCodec {
    H264Libx264,
    Av1SvtAv1,
}

#[derive(Debug, Clone, Copy)]
pub struct VideoSettings {
    pub codec: VideoCodec,
    pub crf: u8,
    pub gop: u32,
}

impl Default for VideoSettings {
    fn default() -> Self {
        Self {
            codec: VideoCodec::H264Libx264,
            crf: 23,
            gop: 2,
        }
    }
}

/// Handle to a declared vector feature; obtained from [`DatasetConfig`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VectorId(pub(crate) usize);

/// Handle to a declared camera; obtained from [`DatasetConfig`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CameraId(pub(crate) usize);

#[derive(Debug, Clone)]
pub(crate) struct VectorFeature {
    pub key: String,
    pub dim_names: Vec<String>,
}

#[derive(Debug, Clone)]
pub(crate) struct CameraFeature {
    pub key: String,
    pub spec: CameraSpec,
}

#[derive(Debug, Clone)]
pub struct DatasetConfig {
    pub(crate) robot_type: String,
    pub(crate) fps: NonZeroU32,
    pub(crate) vectors: Vec<VectorFeature>,
    pub(crate) cameras: Vec<CameraFeature>,
    pub(crate) video: VideoSettings,
    pub(crate) data_files_size_in_mb: u64,
    pub(crate) video_files_size_in_mb: u64,
}

pub struct DatasetConfigBuilder {
    robot_type: String,
    fps: NonZeroU32,
    vectors: Vec<VectorFeature>,
    cameras: Vec<CameraFeature>,
    video: VideoSettings,
    data_files_size_in_mb: u64,
    video_files_size_in_mb: u64,
}

impl DatasetConfig {
    pub fn builder(robot_type: impl Into<String>, fps: NonZeroU32) -> DatasetConfigBuilder {
        DatasetConfigBuilder {
            robot_type: robot_type.into(),
            fps,
            vectors: Vec::new(),
            cameras: Vec::new(),
            video: VideoSettings::default(),
            data_files_size_in_mb: crate::layout::DATA_FILES_SIZE_IN_MB,
            video_files_size_in_mb: crate::layout::VIDEO_FILES_SIZE_IN_MB,
        }
    }

    pub fn fps(&self) -> NonZeroU32 {
        self.fps
    }

    pub fn vector_ids(&self) -> impl Iterator<Item = VectorId> {
        (0..self.vectors.len()).map(VectorId)
    }

    pub fn camera_ids(&self) -> impl Iterator<Item = CameraId> {
        (0..self.cameras.len()).map(CameraId)
    }

    pub fn vector_id(&self, key: &str) -> Option<VectorId> {
        self.vectors.iter().position(|f| f.key == key).map(VectorId)
    }

    pub fn camera_id(&self, key: &str) -> Option<CameraId> {
        self.cameras.iter().position(|c| c.key == key).map(CameraId)
    }

    pub fn vector_key(&self, id: VectorId) -> &str {
        &self.vectors[id.0].key
    }

    pub fn vector_dim(&self, id: VectorId) -> usize {
        self.vectors[id.0].dim_names.len()
    }

    pub fn camera_key(&self, id: CameraId) -> &str {
        &self.cameras[id.0].key
    }

    pub fn camera_spec(&self, id: CameraId) -> CameraSpec {
        self.cameras[id.0].spec
    }
}

impl DatasetConfigBuilder {
    /// Declares `observation.state`; `dim_names` become the feature's `names` in info.json.
    pub fn state(self, dim_names: Vec<String>) -> Self {
        self.vector(STATE_KEY, dim_names)
    }

    /// Declares `action`.
    pub fn action(self, dim_names: Vec<String>) -> Self {
        self.vector(ACTION_KEY, dim_names)
    }

    /// Declares an additional float32 vector feature, e.g. `observation.velocity`.
    pub fn vector_feature(self, key: impl Into<String>, dim_names: Vec<String>) -> Self {
        self.vector(key.into(), dim_names)
    }

    fn vector(mut self, key: impl Into<String>, dim_names: Vec<String>) -> Self {
        self.vectors.push(VectorFeature {
            key: key.into(),
            dim_names,
        });
        self
    }

    /// Declares a camera; `key` must be `observation.images.<name>`.
    pub fn camera(mut self, key: impl Into<String>, spec: CameraSpec) -> Self {
        self.cameras.push(CameraFeature {
            key: key.into(),
            spec,
        });
        self
    }

    pub fn video(mut self, settings: VideoSettings) -> Self {
        self.video = settings;
        self
    }

    /// Overrides the v3 default file rollover thresholds (100 MB data /
    /// 200 MB video); mainly for tests that force rollover cheaply.
    pub fn file_size_limits(mut self, data_mb: NonZeroU64, video_mb: NonZeroU64) -> Self {
        self.data_files_size_in_mb = data_mb.get();
        self.video_files_size_in_mb = video_mb.get();
        self
    }

    pub fn build(self) -> Result<DatasetConfig, ConfigError> {
        let mut seen: Vec<&str> = Vec::new();
        for key in self
            .vectors
            .iter()
            .map(|f| f.key.as_str())
            .chain(self.cameras.iter().map(|c| c.key.as_str()))
        {
            if RESERVED_COLUMNS.contains(&key) {
                return Err(ConfigError::ReservedFeature(key.to_string()));
            }
            if seen.contains(&key) {
                return Err(ConfigError::DuplicateFeature(key.to_string()));
            }
            seen.push(key);
        }
        for feature in &self.vectors {
            if feature.dim_names.is_empty() {
                return Err(ConfigError::EmptyFeature(feature.key.clone()));
            }
        }
        if !self.vectors.iter().any(|f| f.key == STATE_KEY) {
            return Err(ConfigError::MissingState);
        }
        if !self.vectors.iter().any(|f| f.key == ACTION_KEY) {
            return Err(ConfigError::MissingAction);
        }
        for camera in &self.cameras {
            let name = camera.key.strip_prefix(CAMERA_KEY_PREFIX);
            let valid = name.is_some_and(|n| {
                !n.is_empty()
                    && n.chars()
                        .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
            });
            if !valid {
                return Err(ConfigError::InvalidCameraKey(camera.key.clone()));
            }
        }
        if self.video.crf > 63 {
            return Err(ConfigError::CrfOutOfRange(self.video.crf));
        }
        if self.video.gop == 0 {
            return Err(ConfigError::ZeroGop);
        }
        Ok(DatasetConfig {
            robot_type: self.robot_type,
            fps: self.fps,
            vectors: self.vectors,
            cameras: self.cameras,
            video: self.video,
            data_files_size_in_mb: self.data_files_size_in_mb,
            video_files_size_in_mb: self.video_files_size_in_mb,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> DatasetConfigBuilder {
        DatasetConfig::builder("test_bot", NonZeroU32::new(30).unwrap())
            .state(vec!["j1".into(), "j2".into()])
            .action(vec!["j1".into()])
    }

    fn cam_spec() -> CameraSpec {
        CameraSpec {
            width: NonZeroU32::new(64).unwrap(),
            height: NonZeroU32::new(48).unwrap(),
            source: SourceEncoding::Rgb8,
        }
    }

    #[test]
    fn accepts_minimal_state_action() {
        let config = base().build().unwrap();
        assert_eq!(config.vector_dim(config.vector_id(STATE_KEY).unwrap()), 2);
        assert_eq!(config.vector_dim(config.vector_id(ACTION_KEY).unwrap()), 1);
    }

    #[test]
    fn accepts_extra_vector_and_cameras() {
        let config = base()
            .vector_feature("observation.velocity", vec!["j1".into(), "j2".into()])
            .camera("observation.images.wrist_left", cam_spec())
            .build()
            .unwrap();
        assert_eq!(config.camera_ids().count(), 1);
        assert!(config.vector_id("observation.velocity").is_some());
    }

    #[test]
    fn rejects_missing_state_or_action() {
        let fps = NonZeroU32::new(30).unwrap();
        let no_state = DatasetConfig::builder("b", fps)
            .action(vec!["a".into()])
            .build();
        assert_eq!(no_state.unwrap_err(), ConfigError::MissingState);
        let no_action = DatasetConfig::builder("b", fps)
            .state(vec!["s".into()])
            .build();
        assert_eq!(no_action.unwrap_err(), ConfigError::MissingAction);
    }

    #[test]
    fn rejects_duplicates_reserved_and_empty() {
        let dup = base().vector_feature(STATE_KEY, vec!["x".into()]).build();
        assert_eq!(
            dup.unwrap_err(),
            ConfigError::DuplicateFeature(STATE_KEY.into())
        );
        let reserved = base().vector_feature("timestamp", vec!["x".into()]).build();
        assert_eq!(
            reserved.unwrap_err(),
            ConfigError::ReservedFeature("timestamp".into())
        );
        let empty = base()
            .vector_feature("observation.velocity", vec![])
            .build();
        assert_eq!(
            empty.unwrap_err(),
            ConfigError::EmptyFeature("observation.velocity".into())
        );
    }

    #[test]
    fn rejects_bad_camera_keys() {
        for key in [
            "wrist",
            "observation.images.",
            "observation.images.Wrist-Cam",
        ] {
            let got = base().camera(key, cam_spec()).build();
            assert_eq!(got.unwrap_err(), ConfigError::InvalidCameraKey(key.into()));
        }
    }

    #[test]
    fn rejects_bad_video_settings() {
        let crf = base()
            .video(VideoSettings {
                crf: 64,
                ..Default::default()
            })
            .build();
        assert_eq!(crf.unwrap_err(), ConfigError::CrfOutOfRange(64));
        let gop = base()
            .video(VideoSettings {
                gop: 0,
                ..Default::default()
            })
            .build();
        assert_eq!(gop.unwrap_err(), ConfigError::ZeroGop);
    }
}
