//! The crash contract: whatever happens to an in-flight episode (abort, drop,
//! encoder death, empty end), the dataset on disk stays valid and the writer
//! keeps working. Requires ffmpeg (see common::require_ffmpeg).

mod common;

use common::{require_ffmpeg, state_action_builder, tiny_camera};
use lerobot_dataset::{DatasetWriter, Error, Frame, PixelFrame};

fn writer_with_camera(root: &std::path::Path) -> DatasetWriter {
    let config = state_action_builder(2)
        .camera("observation.images.cam", tiny_camera())
        .build()
        .unwrap();
    DatasetWriter::create(root, config).unwrap()
}

fn add_rgb_frames(writer: &mut DatasetWriter, task: &str, frames: usize) -> Result<u64, Error> {
    let spec = tiny_camera();
    let state_id = writer.config().vector_id("observation.state").unwrap();
    let action_id = writer.config().vector_id("action").unwrap();
    let camera_id = writer.config().camera_id("observation.images.cam").unwrap();
    let mut episode = writer.begin_episode(task)?;
    for k in 0..frames {
        let pixels = vec![k as u8; 8 * 8 * 3];
        let image = PixelFrame::rgb8(spec.width, spec.height, &pixels).unwrap();
        episode.add_frame(Frame {
            vectors: &[(state_id, &[0.0, 1.0]), (action_id, &[2.0])],
            images: &[(camera_id, image)],
        })?;
    }
    episode.end().map(|meta| meta.length)
}

fn info_totals(root: &std::path::Path) -> (u64, u64) {
    let info: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("meta/info.json")).unwrap())
            .unwrap();
    (
        info["total_episodes"].as_u64().unwrap(),
        info["total_frames"].as_u64().unwrap(),
    )
}

#[test]
fn abort_leaves_no_trace_and_writer_recovers() {
    require_ffmpeg();
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().join("ds");
    let mut writer = writer_with_camera(&root);
    let spec = tiny_camera();
    let state_id = writer.config().vector_id("observation.state").unwrap();
    let action_id = writer.config().vector_id("action").unwrap();
    let camera_id = writer.config().camera_id("observation.images.cam").unwrap();

    let mut episode = writer.begin_episode("doomed").unwrap();
    for _ in 0..4 {
        let pixels = vec![9u8; 8 * 8 * 3];
        let image = PixelFrame::rgb8(spec.width, spec.height, &pixels).unwrap();
        episode
            .add_frame(Frame {
                vectors: &[(state_id, &[0.0, 1.0]), (action_id, &[2.0])],
                images: &[(camera_id, image)],
            })
            .unwrap();
    }
    episode.abort();

    assert_eq!(info_totals(&root), (0, 0), "abort must not commit anything");
    assert!(
        !root.join(".episode-tmp").exists(),
        "abort sweeps the episode temp dir"
    );
    assert!(
        !root
            .join("videos/observation.images.cam/chunk-000/file-000.mp4")
            .exists(),
        "no shared video file may exist after an aborted first episode"
    );

    assert_eq!(add_rgb_frames(&mut writer, "real", 5).unwrap(), 5);
    assert_eq!(info_totals(&root), (1, 5));
}

#[test]
fn dropped_episode_is_discarded_and_writer_recovers() {
    require_ffmpeg();
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().join("ds");
    let mut writer = writer_with_camera(&root);
    let spec = tiny_camera();
    let state_id = writer.config().vector_id("observation.state").unwrap();
    let action_id = writer.config().vector_id("action").unwrap();
    let camera_id = writer.config().camera_id("observation.images.cam").unwrap();

    {
        let mut episode = writer.begin_episode("leaked").unwrap();
        let pixels = vec![1u8; 8 * 8 * 3];
        let image = PixelFrame::rgb8(spec.width, spec.height, &pixels).unwrap();
        episode
            .add_frame(Frame {
                vectors: &[(state_id, &[0.0, 1.0]), (action_id, &[2.0])],
                images: &[(camera_id, image)],
            })
            .unwrap();
        // Dropped without end() or abort(): encoder Drop kills ffmpeg.
    }

    assert_eq!(info_totals(&root), (0, 0));
    assert_eq!(add_rgb_frames(&mut writer, "real", 3).unwrap(), 3);
    assert_eq!(info_totals(&root), (1, 3));
}

#[test]
fn empty_episode_is_an_error_not_a_commit() {
    require_ffmpeg();
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().join("ds");
    let mut writer = writer_with_camera(&root);

    let episode = writer.begin_episode("empty").unwrap();
    assert!(matches!(episode.end(), Err(Error::EmptyEpisode)));
    assert_eq!(info_totals(&root), (0, 0));

    assert_eq!(add_rgb_frames(&mut writer, "real", 2).unwrap(), 2);
    assert_eq!(info_totals(&root), (1, 2));
}

/// A real 8x8 JPEG, produced by the same ffmpeg the encoder uses.
fn tiny_jpeg() -> Vec<u8> {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let mut child = Command::new("ffmpeg")
        .args([
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            "8x8",
            "-i",
            "pipe:0",
            "-frames:v",
            "1",
            "-f",
            "mjpeg",
            "pipe:1",
        ])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    child
        .stdin
        .take()
        .unwrap()
        .write_all(&[128u8; 8 * 8 * 3])
        .unwrap();
    let output = child.wait_with_output().unwrap();
    assert!(output.status.success());
    output.stdout
}

#[test]
fn video_frame_miscount_surfaces_and_dataset_stays_valid() {
    require_ffmpeg();
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().join("ds");
    let config = state_action_builder(2)
        .camera(
            "observation.images.cam",
            lerobot_dataset::CameraSpec {
                source: lerobot_dataset::SourceEncoding::Mjpeg,
                ..tiny_camera()
            },
        )
        .build()
        .unwrap();
    let mut writer = DatasetWriter::create(&root, config).unwrap();
    let state_id = writer.config().vector_id("observation.state").unwrap();
    let action_id = writer.config().vector_id("action").unwrap();
    let camera_id = writer.config().camera_id("observation.images.cam").unwrap();

    // Two concatenated JPEGs per frame buffer: the encoder emits twice as
    // many video frames as the episode recorded, which end() must reject.
    let jpeg = tiny_jpeg();
    let doubled = [jpeg.clone(), jpeg.clone()].concat();
    let mut episode = writer.begin_episode("doubled").unwrap();
    for _ in 0..3 {
        let image = PixelFrame::mjpeg(&doubled).unwrap();
        episode
            .add_frame(Frame {
                vectors: &[(state_id, &[0.0, 1.0]), (action_id, &[2.0])],
                images: &[(camera_id, image)],
            })
            .unwrap();
    }
    let error = episode
        .end()
        .expect_err("frame miscount must fail the episode");
    assert!(
        matches!(error, Error::Video(_)),
        "expected a video error, got {error:?}"
    );
    assert_eq!(info_totals(&root), (0, 0), "failed episode must not commit");
    assert!(
        !root
            .join("videos/observation.images.cam/chunk-000/file-000.mp4")
            .exists()
    );

    // The writer recovers: a well-formed mjpeg episode commits normally.
    let mut episode = writer.begin_episode("clean").unwrap();
    for _ in 0..2 {
        let image = PixelFrame::mjpeg(&jpeg).unwrap();
        episode
            .add_frame(Frame {
                vectors: &[(state_id, &[0.0, 1.0]), (action_id, &[2.0])],
                images: &[(camera_id, image)],
            })
            .unwrap();
    }
    episode.end().unwrap();
    assert_eq!(info_totals(&root), (1, 2));
}
