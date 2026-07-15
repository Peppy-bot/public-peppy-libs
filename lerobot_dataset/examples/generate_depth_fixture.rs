//! Writes a synthetic dataset with one color and one depth camera, for the
//! depth compliance check (compliance/check_depth.py). Depth values are a
//! per-frame ramp in millimetres so the Python loader's dequantized output
//! can be checked against known inputs.

use std::num::NonZeroU32;

use lerobot_dataset::{
    CameraSpec, DatasetConfig, DepthQuantization, DepthSpec, Frame, PixelFrame, SourceEncoding,
};

const FPS: u32 = 10;
const WIDTH: u32 = 32;
const HEIGHT: u32 = 24;
const EPISODES: [(&str, u64); 2] = [("reach", 12), ("grasp", 15)];

/// Depth in millimetres for a frame: a flat plane that steps every frame,
/// staying inside the default [0.01, 10] m quantization range.
fn depth_mm(episode: u64, frame: u64) -> u16 {
    (600 + episode * 400 + frame * 120) as u16
}

fn color(frame: u64) -> Vec<u8> {
    let mut out = Vec::with_capacity((WIDTH * HEIGHT * 3) as usize);
    for _y in 0..HEIGHT {
        for x in 0..WIDTH as i64 {
            let bar = (x - frame as i64 * 3).rem_euclid(WIDTH as i64) < 3;
            let v = if bar { 255 } else { 40 };
            out.extend([v, v, v]);
        }
    }
    out
}

fn main() {
    let out = std::env::args()
        .nth(1)
        .expect("usage: generate_depth_fixture <output_dir>");
    let (w, h) = (
        NonZeroU32::new(WIDTH).unwrap(),
        NonZeroU32::new(HEIGHT).unwrap(),
    );
    let color_spec = CameraSpec {
        width: w,
        height: h,
        source: SourceEncoding::Rgb8,
    };
    // depth_unit 0.001 m/LSB => z16 codes are already millimetres.
    let depth_spec = DepthSpec {
        width: w,
        height: h,
        depth_unit_m: 0.001,
        quantization: DepthQuantization::default(),
    };
    let config = DatasetConfig::builder("synthetic_depth", NonZeroU32::new(FPS).unwrap())
        .state(vec!["s0".into()])
        .action(vec!["a0".into()])
        .camera("observation.images.color", color_spec)
        .depth_camera("observation.images.depth", depth_spec)
        .build()
        .expect("valid config");

    let mut writer = lerobot_dataset::DatasetWriter::create(&out, config).expect("create");
    let state_id = writer.config().vector_id("observation.state").unwrap();
    let action_id = writer.config().vector_id("action").unwrap();
    let color_id = writer
        .config()
        .camera_id("observation.images.color")
        .unwrap();
    let depth_id = writer
        .config()
        .camera_id("observation.images.depth")
        .unwrap();

    for (episode, (task, length)) in EPISODES.iter().enumerate() {
        let mut ep = writer.begin_episode(task).expect("begin");
        for frame in 0..*length {
            let color_px = color(frame);
            let mm = depth_mm(episode as u64, frame);
            let depth_px: Vec<u8> =
                std::iter::repeat_n(mm.to_le_bytes(), (WIDTH * HEIGHT) as usize)
                    .flatten()
                    .collect();
            ep.add_frame(Frame {
                vectors: &[(state_id, &[frame as f32]), (action_id, &[-(frame as f32)])],
                images: &[
                    (color_id, PixelFrame::rgb8(w, h, &color_px).unwrap()),
                    (depth_id, PixelFrame::z16(w, h, &depth_px).unwrap()),
                ],
            })
            .expect("add_frame");
        }
        ep.end().expect("end");
    }
    let summary = writer.finalize().expect("finalize");
    println!(
        "wrote {} episodes / {} frames to {}",
        summary.total_episodes,
        summary.total_frames,
        summary.root.display()
    );
}
