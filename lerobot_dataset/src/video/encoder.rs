//! Per-episode, per-camera ffmpeg subprocess fed raw frames on stdin.
//!
//! PTS correctness is the whole game: `-framerate fps` on the input stamps
//! frame k at exactly k/fps, and a track timescale that is an integer
//! multiple of fps keeps those instants exactly representable in the mp4, so
//! the loader's decoded-PTS tolerance check (1e-4 s) passes by construction.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};

use crate::config::{CameraSpec, SourceEncoding, VideoSettings};
use crate::error::VideoError;
use crate::frame::PixelFrame;
use crate::video::probe::{count_frames, encoder_name};

pub fn track_timescale(fps: u32) -> u32 {
    fps * 512
}

pub fn input_args(spec: &CameraSpec, fps: u32) -> Vec<String> {
    let mut args: Vec<String> = match spec.source {
        SourceEncoding::Rgb8 | SourceEncoding::Bgr8 | SourceEncoding::Yuyv => {
            let pix_fmt = match spec.source {
                SourceEncoding::Rgb8 => "rgb24",
                SourceEncoding::Bgr8 => "bgr24",
                SourceEncoding::Yuyv => "yuyv422",
                SourceEncoding::Mjpeg => unreachable!(),
            };
            vec![
                "-f".into(),
                "rawvideo".into(),
                "-pix_fmt".into(),
                pix_fmt.into(),
                "-s".into(),
                format!("{}x{}", spec.width, spec.height),
            ]
        }
        SourceEncoding::Mjpeg => vec!["-f".into(), "mjpeg".into()],
    };
    args.extend([
        "-framerate".into(),
        fps.to_string(),
        "-i".into(),
        "pipe:0".into(),
    ]);
    args
}

pub fn output_args(video: &VideoSettings, fps: u32, dest: &Path) -> Vec<String> {
    vec![
        "-an".into(),
        "-c:v".into(),
        encoder_name(video.codec).into(),
        "-pix_fmt".into(),
        "yuv420p".into(),
        "-crf".into(),
        video.crf.to_string(),
        "-g".into(),
        video.gop.to_string(),
        "-video_track_timescale".into(),
        track_timescale(fps).to_string(),
        "-movflags".into(),
        "+faststart".into(),
        "-f".into(),
        "mp4".into(),
        dest.to_string_lossy().into_owned(),
    ]
}

pub struct EpisodeEncoder {
    camera: String,
    /// Some until `finish` consumes the process; Drop kills a leftover child.
    child: Option<Child>,
    stdin: Option<ChildStdin>,
    temp_path: PathBuf,
    frames: u64,
}

impl EpisodeEncoder {
    pub fn spawn(
        camera: &str,
        spec: &CameraSpec,
        video: &VideoSettings,
        fps: u32,
        temp_path: PathBuf,
    ) -> Result<Self, VideoError> {
        if let Some(parent) = temp_path.parent() {
            std::fs::create_dir_all(parent).map_err(VideoError::FfmpegNotFound)?;
        }
        let mut command = Command::new("ffmpeg");
        command
            .args(["-hide_banner", "-loglevel", "error", "-y"])
            .args(input_args(spec, fps))
            .args(output_args(video, fps, &temp_path))
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::piped());
        let mut child = command.spawn().map_err(VideoError::FfmpegNotFound)?;
        let stdin = child.stdin.take();
        Ok(Self {
            camera: camera.to_string(),
            child: Some(child),
            stdin,
            temp_path,
            frames: 0,
        })
    }

    pub fn write_frame(&mut self, frame: &PixelFrame<'_>) -> Result<(), VideoError> {
        let stdin = self
            .stdin
            .as_mut()
            .expect("stdin taken only in finish/abort");
        if stdin.write_all(frame.bytes).is_err() {
            return Err(self.exit_error());
        }
        self.frames += 1;
        Ok(())
    }

    /// Closes stdin, waits for a clean exit, and verifies the frame count.
    /// On failure the temp output is removed; on success it is handed to the
    /// caller (concat consumes it).
    pub fn finish(mut self) -> Result<(PathBuf, u64), VideoError> {
        drop(self.stdin.take());
        let child = self.child.take().expect("finish runs once");
        let output = child
            .wait_with_output()
            .map_err(VideoError::FfmpegNotFound)?;
        if !output.status.success() {
            let _ = std::fs::remove_file(&self.temp_path);
            return Err(VideoError::EncoderExited {
                camera: self.camera.clone(),
                status: output.status,
                stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
            });
        }
        let probed = count_frames(&self.camera, &self.temp_path)?;
        if probed != self.frames {
            let _ = std::fs::remove_file(&self.temp_path);
            return Err(VideoError::FrameCountMismatch {
                camera: self.camera.clone(),
                expected: self.frames,
                probed,
            });
        }
        Ok((self.temp_path.clone(), self.frames))
    }

    /// A failed stdin write means the encoder died; surface its status and stderr.
    fn exit_error(&mut self) -> VideoError {
        drop(self.stdin.take());
        let mut child = self
            .child
            .take()
            .expect("child present while stdin is open");
        let _ = child.kill();
        let mut stderr = String::new();
        if let Some(mut pipe) = child.stderr.take() {
            use std::io::Read;
            let _ = pipe.read_to_string(&mut stderr);
        }
        let _ = std::fs::remove_file(&self.temp_path);
        match child.wait() {
            Ok(status) => VideoError::EncoderExited {
                camera: self.camera.clone(),
                status,
                stderr,
            },
            Err(source) => VideoError::FfmpegNotFound(source),
        }
    }
}

/// An encoder dropped without `finish` is an aborted episode: kill the
/// process, reap it, and remove the partial temp output.
impl Drop for EpisodeEncoder {
    fn drop(&mut self) {
        drop(self.stdin.take());
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
            let _ = std::fs::remove_file(&self.temp_path);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::num::NonZeroU32;

    use super::*;
    use crate::config::VideoCodec;

    fn spec(source: SourceEncoding) -> CameraSpec {
        CameraSpec {
            width: NonZeroU32::new(64).unwrap(),
            height: NonZeroU32::new(48).unwrap(),
            source,
        }
    }

    #[test]
    fn rawvideo_args_carry_geometry_and_exact_rate() {
        let args = input_args(&spec(SourceEncoding::Yuyv), 30);
        assert_eq!(
            args,
            [
                "-f",
                "rawvideo",
                "-pix_fmt",
                "yuyv422",
                "-s",
                "64x48",
                "-framerate",
                "30",
                "-i",
                "pipe:0"
            ]
        );
    }

    #[test]
    fn mjpeg_args_skip_geometry() {
        let args = input_args(&spec(SourceEncoding::Mjpeg), 30);
        assert_eq!(args, ["-f", "mjpeg", "-framerate", "30", "-i", "pipe:0"]);
    }

    #[test]
    fn output_args_pin_timescale_and_codec() {
        let video = VideoSettings {
            codec: VideoCodec::Av1SvtAv1,
            crf: 30,
            gop: 2,
        };
        let args = output_args(&video, 30, Path::new("/tmp/x.mp4"));
        let joined = args.join(" ");
        assert!(joined.contains("-c:v libsvtav1"));
        assert!(joined.contains("-crf 30"));
        assert!(joined.contains("-g 2"));
        assert!(joined.contains("-video_track_timescale 15360"));
        assert!(joined.ends_with("/tmp/x.mp4"));
    }
}
