//! Appending a finished episode mp4 onto a camera's shared chunk file with
//! the concat demuxer and stream copy: no re-encode, and segment offsets stay
//! on the k/fps grid because every segment shares the same timescale and its
//! duration is exactly frames/fps.

use std::path::Path;
use std::process::Command;

use crate::error::{Error, VideoError};
use crate::video::encoder::track_timescale;

/// Moves (first episode in a file) or concat-appends `episode` into `shared`.
/// The episode temp file is consumed either way.
pub fn append_or_start(shared: &Path, episode: &Path, camera: &str, fps: u32) -> Result<(), Error> {
    let parent = shared.parent().expect("video paths always have a parent");
    std::fs::create_dir_all(parent).map_err(Error::io(parent))?;
    if !shared.exists() {
        std::fs::rename(episode, shared).map_err(Error::io(shared))?;
        return Ok(());
    }

    let list = parent.join(".concat-list.txt");
    let entries = [shared, episode].map(|p| {
        format!(
            "file '{}'",
            p.canonicalize().unwrap_or(p.to_path_buf()).display()
        )
    });
    std::fs::write(
        &list,
        format!("ffconcat version 1.0\n{}\n{}\n", entries[0], entries[1]),
    )
    .map_err(Error::io(&list))?;

    let merged = crate::atomic::replace_via_temp(shared, |_, temp_path| {
        let output = Command::new("ffmpeg")
            .args([
                "-hide_banner",
                "-loglevel",
                "error",
                "-y",
                "-f",
                "concat",
                "-safe",
                "0",
                "-i",
            ])
            .arg(&list)
            .args(["-c", "copy", "-video_track_timescale"])
            .arg(track_timescale(fps).to_string())
            .args(["-movflags", "+faststart", "-f", "mp4"])
            .arg(temp_path)
            .output()
            .map_err(|e| Error::Video(VideoError::FfmpegNotFound(e)))?;
        if !output.status.success() {
            return Err(Error::Video(VideoError::ConcatFailed {
                camera: camera.to_string(),
                status: output.status,
                stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
            }));
        }
        // ffmpeg's faststart pass replaces the file behind the temp path, so
        // sync by path; the handle held by replace_via_temp may be stale.
        std::fs::File::open(temp_path)
            .and_then(|f| f.sync_all())
            .map_err(Error::io(temp_path))?;
        Ok(())
    });
    let _ = std::fs::remove_file(&list);
    let _ = std::fs::remove_file(episode);
    merged
}
