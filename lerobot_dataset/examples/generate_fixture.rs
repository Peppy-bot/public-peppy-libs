//! Writes the deterministic synthetic dataset used by the compliance
//! harness. Content mirrors compliance/dump_golden_schemas.py exactly, so the
//! same values exist in the golden (Python-written) and native (Rust-written)
//! datasets.

use std::num::NonZeroU32;

use lerobot_dataset::{
    CameraSpec, DatasetConfig, Frame, PixelFrame, SourceEncoding, VideoCodec, VideoSettings,
};

const FPS: u32 = 30;
const STATE_DIM: usize = 4;
const ACTION_DIM: usize = 3;
const WIDTH: u32 = 64;
const HEIGHT: u32 = 48;
const CAMERAS: [&str; 2] = ["observation.images.cam_a", "observation.images.cam_b"];
const EPISODES: [(&str, u64); 3] = [
    ("pick up the cube", 45),
    ("pick up the cube", 40),
    ("place the cube in the bin", 50),
];

fn state_value(episode: u64, frame: u64, dim: usize) -> f32 {
    episode as f32 + frame as f32 / 100.0 + dim as f32 / 1000.0
}

fn action_value(episode: u64, frame: u64, dim: usize) -> f32 {
    -(episode as f32) - frame as f32 / 100.0 - dim as f32 / 1000.0
}

/// A white 4px vertical bar marching 4px per frame over an episode-coded
/// background. Adjacent frames differ by ~12% of pixels at full contrast, so
/// codec noise and color-conversion bias cannot blur frame identity.
fn image(episode: u64, frame: u64) -> Vec<u8> {
    let background = (30 + episode * 40) as u8;
    let mut out = Vec::with_capacity((WIDTH * HEIGHT * 3) as usize);
    for _y in 0..HEIGHT as u64 {
        for x in 0..WIDTH as i64 {
            let offset = (x - frame as i64 * 4).rem_euclid(WIDTH as i64);
            let value = if offset < 4 { 255 } else { background };
            out.extend([value, value, value]);
        }
    }
    out
}

fn main() {
    let out = std::env::args()
        .nth(1)
        .expect("usage: generate_fixture <output_dir> [av1]");
    let codec = match std::env::args().nth(2).as_deref() {
        Some("av1") => VideoCodec::Av1SvtAv1,
        Some(other) => panic!("unknown codec {other:?}, expected \"av1\""),
        None => VideoCodec::H264Libx264,
    };

    let spec = CameraSpec {
        width: NonZeroU32::new(WIDTH).unwrap(),
        height: NonZeroU32::new(HEIGHT).unwrap(),
        source: SourceEncoding::Rgb8,
    };
    let mut builder = DatasetConfig::builder("synthetic_test", NonZeroU32::new(FPS).unwrap())
        .state((0..STATE_DIM).map(|i| format!("state_{i}")).collect())
        .action((0..ACTION_DIM).map(|i| format!("action_{i}")).collect())
        .video(VideoSettings {
            codec,
            ..VideoSettings::default()
        });
    for key in CAMERAS {
        builder = builder.camera(key, spec);
    }
    let config = builder.build().expect("valid config");

    let mut writer = lerobot_dataset::DatasetWriter::create(&out, config).expect("create dataset");
    let camera_ids: Vec<_> = writer.config().camera_ids().collect();
    let state_id = writer.config().vector_id("observation.state").unwrap();
    let action_id = writer.config().vector_id("action").unwrap();

    for (episode_index, (task, length)) in EPISODES.iter().enumerate() {
        let mut episode = writer.begin_episode(task).expect("begin episode");
        for frame in 0..*length {
            let state: Vec<f32> = (0..STATE_DIM)
                .map(|d| state_value(episode_index as u64, frame, d))
                .collect();
            let action: Vec<f32> = (0..ACTION_DIM)
                .map(|d| action_value(episode_index as u64, frame, d))
                .collect();
            let pixels = image(episode_index as u64, frame);
            let pixel_frames: Vec<_> = camera_ids
                .iter()
                .map(|&id| {
                    (
                        id,
                        PixelFrame::rgb8(spec.width, spec.height, &pixels).expect("sized buffer"),
                    )
                })
                .collect();
            episode
                .add_frame(Frame {
                    vectors: &[(state_id, &state), (action_id, &action)],
                    images: &pixel_frames,
                })
                .expect("add frame");
        }
        let meta = episode.end().expect("end episode");
        assert_eq!(meta.length, *length);
    }
    let summary = writer.finalize().expect("finalize");
    println!(
        "wrote {} episodes / {} frames to {}",
        summary.total_episodes,
        summary.total_frames,
        summary.root.display()
    );
}
