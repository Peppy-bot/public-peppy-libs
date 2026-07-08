//! The shared mp4 after episode concatenation carries an exact k/fps PTS
//! grid, which is what keeps the Python loader's tolerance check green.
//! Requires ffmpeg (see common::require_ffmpeg).

mod common;

use std::process::Command;

use common::{require_ffmpeg, state_action_builder, tiny_camera};
use lerobot_dataset::{DatasetWriter, Frame, PixelFrame};

const FPS: u32 = 30;
const TICKS_PER_FRAME: i64 = 512;

fn record(root: &std::path::Path, episode_lengths: &[usize]) {
    let spec = tiny_camera();
    let config = state_action_builder(2)
        .camera("observation.images.cam", spec)
        .build()
        .unwrap();
    let mut writer = DatasetWriter::create(root, config).unwrap();
    let state_id = writer.config().vector_id("observation.state").unwrap();
    let action_id = writer.config().vector_id("action").unwrap();
    let camera_id = writer.config().camera_id("observation.images.cam").unwrap();

    for (episode_index, &length) in episode_lengths.iter().enumerate() {
        let mut episode = writer.begin_episode("pts").unwrap();
        for frame in 0..length {
            let pixels = vec![(episode_index * 40 + frame) as u8; 8 * 8 * 3];
            let image = PixelFrame::rgb8(spec.width, spec.height, &pixels).unwrap();
            episode
                .add_frame(Frame {
                    vectors: &[(state_id, &[0.0, 1.0]), (action_id, &[2.0])],
                    images: &[(camera_id, image)],
                })
                .unwrap();
        }
        episode.end().unwrap();
    }
    writer.finalize().unwrap();
}

fn packet_pts(video: &std::path::Path) -> Vec<i64> {
    let output = Command::new("ffprobe")
        .args([
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "packet=pts",
            "-of",
            "csv=p=0",
        ])
        .arg(video)
        .output()
        .expect("ffprobe runs");
    let mut pts: Vec<i64> = String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| l.trim().parse().expect("integer pts"))
        .collect();
    pts.sort_unstable();
    pts
}

#[test]
fn concatenated_video_keeps_exact_pts_grid() {
    require_ffmpeg();
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().join("ds");
    let lengths = [7usize, 5, 9];
    record(&root, &lengths);

    let video = root.join("videos/observation.images.cam/chunk-000/file-000.mp4");
    let pts = packet_pts(&video);
    let total: usize = lengths.iter().sum();
    assert_eq!(pts.len(), total, "one packet per recorded frame");
    let expected: Vec<i64> = (0..total as i64).map(|k| k * TICKS_PER_FRAME).collect();
    assert_eq!(pts, expected, "PTS must sit exactly on the k/fps grid");

    // The grid in seconds is k/fps exactly; spot-check the timescale math.
    assert_eq!(TICKS_PER_FRAME * FPS as i64, 15360);
}
