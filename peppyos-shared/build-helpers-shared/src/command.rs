//! Running external commands from build scripts.

use std::io::{BufRead, Read};
use std::process::{Command, Stdio};

/// Runs a command, streaming its stdout and stderr as `cargo:warning=` lines.
/// Returns `true` on success.
///
/// Each output line is forwarded as `cargo:warning=[{label}] {line}` so the
/// user sees real-time progress during long-running build script operations.
///
/// Any stdout/stderr configuration already set on `command` is overridden
/// with `Stdio::piped()`; stdin is left inherited — pass
/// `.stdin(Stdio::null())` for tools that might prompt for input.
pub(crate) fn run_command_streaming(command: &mut Command, label: &str) -> bool {
    command.stdout(Stdio::piped()).stderr(Stdio::piped());

    let mut child = match command.spawn() {
        Ok(c) => c,
        Err(e) => {
            println!("cargo:warning=[{label}] Failed to spawn: {e}");
            return false;
        }
    };

    // Read stdout and stderr on separate threads to avoid deadlocks.
    // If both are read sequentially, a child that fills one pipe buffer
    // while we're blocked reading the other will hang indefinitely.
    fn stream_pipe(pipe: impl Read + Send + 'static, label: String) -> std::thread::JoinHandle<()> {
        std::thread::spawn(move || {
            for line in std::io::BufReader::new(pipe).lines().map_while(Result::ok) {
                println!("cargo:warning=[{}] {}", label, line);
            }
        })
    }

    let stderr_thread = stream_pipe(child.stderr.take().unwrap(), label.to_string());
    let stdout_thread = stream_pipe(child.stdout.take().unwrap(), label.to_string());

    stdout_thread.join().ok();
    stderr_thread.join().ok();
    let status = match child.wait() {
        Ok(status) => status,
        Err(e) => {
            println!("cargo:warning=[{label}] Failed to wait for child process: {e}");
            return false;
        }
    };

    if !status.success() {
        println!("cargo:warning=[{label}] Command failed with exit status: {status}");
    }
    status.success()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An absolute path that is guaranteed not to exist, for spawn-failure tests.
    /// Absolute so the result does not depend on `PATH` contents.
    fn missing_binary(dir: &tempfile::TempDir) -> std::path::PathBuf {
        dir.path().join("no-such-bin")
    }

    #[test]
    fn streaming_reports_success() {
        assert!(run_command_streaming(
            Command::new("echo").arg("hello world"),
            "test-echo"
        ));
    }

    #[test]
    fn streaming_reports_failure() {
        assert!(!run_command_streaming(
            &mut Command::new("false"),
            "test-fail"
        ));
    }

    #[test]
    fn streaming_reports_spawn_failure() {
        let dir = tempfile::tempdir().expect("temp dir");
        assert!(!run_command_streaming(
            &mut Command::new(missing_binary(&dir)),
            "test-spawn"
        ));
    }
}
