# lerobot_dataset

Native Rust writer for [LeRobot v3 datasets](https://huggingface.co/docs/lerobot/lerobot-dataset-v3).
Data in, files out: parquet data and metadata via arrow, video via an ffmpeg
subprocess. No robotics framework dependencies, so it is usable from any
recorder process.

## What it provides

- `DatasetConfig`: typed schema (state/action/extra vector features, cameras
  with source encodings rgb8/bgr8/yuyv/mjpeg, codec settings). Invalid
  configs are unrepresentable past `build()`.
- `DatasetWriter` / `EpisodeWriter`: episode lifecycle enforced by the borrow
  checker (no second open episode, no frames outside an episode, no finalize
  with an episode open). `add_frame` streams pixels straight into a
  per-camera ffmpeg encoder; numeric rows are buffered and committed at
  `end()`.
- Full v3 output: chunked data parquet, `meta/info.json`, `meta/stats.json`
  (with lerobot's histogram-estimated quantiles), `meta/tasks.parquet`
  (pandas-indexed, as the loader requires), `meta/episodes/` rows with
  per-episode stats and video time offsets, and per-camera shared mp4s with
  episodes concatenated via stream copy.

## Depth cameras

`depth_camera(key, DepthSpec)` records a depth stream as a single-channel
`gray12le` HEVC-lossless video with `is_depth_map` set, matching lerobot 0.6.
The caller feeds raw `z16` codes (`PixelFrame::z16`); the crate scales them to
millimetres via `depth_unit_m` and log-quantizes to 12-bit exactly as lerobot
does, so the Python loader dequantizes back to metres. Stats are kept in
millimetres (not normalized). The compliance harness verifies round-trip.

## Requirements

`ffmpeg` and `ffprobe` on PATH, with `libx264` (default color codec) or
`libsvtav1` (opt-in), and `libx265` when recording depth cameras. The writer
probes at `DatasetWriter::create` and fails fast with an actionable error.

## Crash contract

The dataset on disk is valid and loadable after every completed
`EpisodeWriter::end()`. Every file lands via temp + fsync + atomic rename,
and the episodes row plus info totals are the commit point: a process killed
mid-episode (or mid-`end()`) loses exactly the in-flight episode, and any
partial artifacts are unreferenced and invisible to the loader. There is no
resume: one `DatasetWriter::create` per dataset; record sessions into fresh
roots and merge offline.

## Out of scope

Reading datasets, HF Hub upload, resume/append, audio, depth video features,
and async APIs. Depth streams belong in a recorder's other sinks (e.g. MCAP).

## Known deviation from Python lerobot

Image stats accumulate over every frame (stride-downsampled, fixed [0, 255]
histogram range) instead of lerobot's after-the-fact sampling of ~N^0.75
frames, which a streaming writer cannot do without retaining all frames.
Structure and normalization are identical; values differ within histogram
bin width. Verified by the compliance harness.

## Compliance harness

`compliance/` pins the format to ground truth (dev-time only, never a
runtime dependency):

```sh
cd compliance
pixi run dump-golden            # regenerate golden fixtures with Python lerobot
cargo run --example generate_fixture -- /tmp/native_ds
pixi run check /tmp/native_ds   # load with LeRobotDataset, verify every frame
```

The Rust unit tests additionally assert schema parity field-for-field
against the golden dumps in `tests/fixtures/golden/`.

## Using from another crate

```toml
[dependencies]
lerobot_dataset = { git = "https://github.com/Peppy-bot/public-peppy-libs", rev = "<commit>", package = "lerobot_dataset" }
```
