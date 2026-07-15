use std::path::Path;
use std::process::Command;

use crate::config::VideoCodec;
use crate::error::VideoError;

pub fn encoder_name(codec: VideoCodec) -> &'static str {
    match codec {
        VideoCodec::H264Libx264 => "libx264",
        VideoCodec::Av1SvtAv1 => "libsvtav1",
    }
}

pub fn codec_name(codec: VideoCodec) -> &'static str {
    match codec {
        VideoCodec::H264Libx264 => "h264",
        VideoCodec::Av1SvtAv1 => "av1",
    }
}

/// Fails fast if ffmpeg/ffprobe are missing or ffmpeg lacks a required
/// encoder (the color codec, and libx265 when depth cameras are present).
pub fn probe_toolchain(codec: VideoCodec, needs_depth: bool) -> Result<(), VideoError> {
    let encoders = Command::new("ffmpeg")
        .args(["-hide_banner", "-encoders"])
        .output()
        .map_err(VideoError::FfmpegNotFound)?;
    let listing = String::from_utf8_lossy(&encoders.stdout);
    let has_encoder = |name: &str| {
        listing
            .lines()
            .any(|l| l.split_whitespace().nth(1) == Some(name))
    };

    let color = encoder_name(codec);
    if !has_encoder(color) {
        return Err(VideoError::EncoderUnavailable(color));
    }
    if needs_depth && !has_encoder(crate::video::encoder::DEPTH_ENCODER) {
        return Err(VideoError::EncoderUnavailable(
            crate::video::encoder::DEPTH_ENCODER,
        ));
    }
    Command::new("ffprobe")
        .arg("-version")
        .output()
        .map_err(VideoError::FfprobeNotFound)?;
    Ok(())
}

/// Frame count of the (single) video stream, via packet count.
pub fn count_frames(camera: &str, video: &Path) -> Result<u64, VideoError> {
    let output = Command::new("ffprobe")
        .args([
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-count_packets",
            "-show_entries",
            "stream=nb_read_packets",
            "-of",
            "csv=p=0",
        ])
        .arg(video)
        .output()
        .map_err(VideoError::FfprobeNotFound)?;
    let text = String::from_utf8_lossy(&output.stdout);
    text.trim()
        .parse()
        .map_err(|_| VideoError::FrameCountMismatch {
            camera: camera.to_string(),
            expected: 0,
            probed: 0,
        })
}
