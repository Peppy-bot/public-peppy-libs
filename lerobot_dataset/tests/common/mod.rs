// Each integration-test binary compiles this module and uses a subset of
// it, so per-binary dead-code analysis is meaningless here.
#![allow(dead_code)]

use std::num::{NonZeroU32, NonZeroU64};
use std::process::Command;

use lerobot_dataset::{CameraSpec, DatasetConfig, DatasetConfigBuilder, SourceEncoding};

/// Video-dependent tests require ffmpeg; failing loudly beats silently
/// skipping coverage.
pub fn require_ffmpeg() {
    if Command::new("ffmpeg").arg("-version").output().is_err() {
        panic!(
            "ffmpeg is not on PATH; install it or run with the compliance env: \
             PATH=\"$PWD/compliance/.pixi/envs/default/bin:$PATH\" cargo test"
        );
    }
}

pub fn tiny_camera() -> CameraSpec {
    CameraSpec {
        width: NonZeroU32::new(8).unwrap(),
        height: NonZeroU32::new(8).unwrap(),
        source: SourceEncoding::Rgb8,
    }
}

pub fn state_action_builder(state_dim: usize) -> DatasetConfigBuilder {
    DatasetConfig::builder("test_bot", NonZeroU32::new(30).unwrap())
        .state((0..state_dim).map(|i| format!("s{i}")).collect())
        .action(vec!["a0".into()])
}

pub fn one_mb() -> NonZeroU64 {
    NonZeroU64::new(1).unwrap()
}

/// Deterministic pseudo-random f32s that snappy cannot compress away, so
/// small frame counts produce real on-disk bytes for rollover tests.
pub struct Lcg(pub u64);

impl Lcg {
    pub fn next_f32(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 33) as f32) / (u32::MAX as f32)
    }

    pub fn fill(&mut self, n: usize) -> Vec<f32> {
        (0..n).map(|_| self.next_f32()).collect()
    }
}
