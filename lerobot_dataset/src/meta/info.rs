//! `meta/info.json`: the dataset manifest the loader validates first.

use serde_json::{Map, Value, json};

use crate::config::DatasetConfig;
use crate::layout;
use crate::video::probe::codec_name;

pub struct Totals {
    pub episodes: u64,
    pub frames: u64,
    pub tasks: u64,
}

pub fn build_info_json(config: &DatasetConfig, totals: &Totals) -> Value {
    let mut features = Map::new();
    for feature in &config.vectors {
        features.insert(
            feature.key.clone(),
            json!({
                "dtype": "float32",
                "shape": [feature.dim_names.len()],
                "names": feature.dim_names,
            }),
        );
    }
    for camera in &config.cameras {
        let (w, h) = (camera.spec.width.get(), camera.spec.height.get());
        let feature = match &camera.depth {
            None => json!({
                "dtype": "video",
                "shape": [h, w, 3],
                "names": ["height", "width", "channels"],
                "info": {
                    "video.height": h,
                    "video.width": w,
                    "video.codec": codec_name(config.video.codec),
                    "video.pix_fmt": "yuv420p",
                    "video.fps": config.fps.get(),
                    "video.channels": 3,
                    "has_audio": false,
                    "video.g": config.video.gop,
                    "video.crf": config.video.crf,
                    "video.video_backend": "ffmpeg",
                    "is_depth_map": false,
                },
            }),
            // Matches lerobot 0.6's depth feature: single-channel gray12le
            // HEVC-lossless, with the quantization params the loader needs to
            // dequantize back to millimetres.
            Some(depth) => json!({
                "dtype": "video",
                "shape": [h, w, 1],
                "names": ["height", "width", "channels"],
                "info": {
                    "is_depth_map": true,
                    "depth_unit": "mm",
                    "video.height": h,
                    "video.width": w,
                    "video.codec": "hevc",
                    "video.pix_fmt": "gray12le",
                    "video.fps": config.fps.get(),
                    "video.channels": 1,
                    "has_audio": false,
                    "video.g": 2,
                    "video.crf": 30,
                    "video.preset": null,
                    "video.fast_decode": 0,
                    "video.video_backend": "pyav",
                    "video.extra_options": {},
                    "video.depth_min": depth.quantization.depth_min_m,
                    "video.depth_max": depth.quantization.depth_max_m,
                    "video.shift": depth.quantization.shift_m,
                    "video.use_log": depth.quantization.use_log,
                },
            }),
        };
        features.insert(camera.key.clone(), feature);
    }
    features.insert(
        "timestamp".into(),
        json!({"dtype": "float32", "shape": [1], "names": null}),
    );
    for name in layout::INT64_BOOKKEEPING_COLUMNS {
        features.insert(
            name.into(),
            json!({"dtype": "int64", "shape": [1], "names": null}),
        );
    }

    json!({
        "codebase_version": layout::CODEBASE_VERSION,
        "fps": config.fps.get(),
        "features": features,
        "total_episodes": totals.episodes,
        "total_frames": totals.frames,
        "total_tasks": totals.tasks,
        "chunks_size": layout::CHUNKS_SIZE,
        "data_files_size_in_mb": config.data_files_size_in_mb,
        "video_files_size_in_mb": config.video_files_size_in_mb,
        "data_path": layout::DATA_PATH_TEMPLATE,
        "video_path": layout::VIDEO_PATH_TEMPLATE,
        "robot_type": config.robot_type,
        "splits": {"train": format!("0:{}", totals.episodes)},
    })
}

#[cfg(test)]
mod tests {
    use std::num::NonZeroU32;

    use super::*;
    use crate::config::{CameraSpec, SourceEncoding};

    #[test]
    fn info_matches_v3_contract() {
        let config = DatasetConfig::builder("openarm", NonZeroU32::new(30).unwrap())
            .state(vec!["j1".into()])
            .action(vec!["j1".into()])
            .camera(
                "observation.images.front",
                CameraSpec {
                    width: NonZeroU32::new(640).unwrap(),
                    height: NonZeroU32::new(480).unwrap(),
                    source: SourceEncoding::Mjpeg,
                },
            )
            .build()
            .unwrap();
        let info = build_info_json(
            &config,
            &Totals {
                episodes: 2,
                frames: 90,
                tasks: 1,
            },
        );
        assert_eq!(info["codebase_version"], "v3.0");
        assert_eq!(info["splits"]["train"], "0:2");
        assert_eq!(
            info["data_path"],
            "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
        );
        let cam = &info["features"]["observation.images.front"];
        assert_eq!(cam["shape"], json!([480, 640, 3]));
        assert_eq!(cam["info"]["video.codec"], "h264");
        assert_eq!(cam["info"]["is_depth_map"], false);
        assert_eq!(info["features"]["timestamp"]["names"], Value::Null);
        let keys: Vec<String> = info["features"]
            .as_object()
            .unwrap()
            .keys()
            .cloned()
            .collect();
        assert_eq!(
            keys,
            [
                "observation.state",
                "action",
                "observation.images.front",
                "timestamp",
                "frame_index",
                "episode_index",
                "index",
                "task_index"
            ]
            .map(String::from)
        );
    }
}
