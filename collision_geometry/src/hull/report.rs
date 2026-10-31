//! The stream in memory that Qhull writes its messages to in place of the process's
//! stderr: the check of the points, and the report of a failed pass, whose first error
//! message names the failure. It holds no file descriptor.

use std::io;
use std::ptr::NonNull;

use qhull::{Qh, QhBuilder, QhError, sys};

/// The bytes of memory that hold Qhull's messages of one pass; a longer report is cut
/// short.
const REPORT_BYTES: usize = 4096;

/// The prefix of Qhull's error messages, each `QH6` and the rest of its number.
const ERROR_MESSAGE: &str = "QH6";

/// A stream that writes into memory this report owns.
pub(super) struct Report {
    file: NonNull<sys::FILE>,
    /// The memory the stream writes to: `REPORT_BYTES` bytes, zeroed when opened, freed
    /// once the stream is closed.
    buffer: NonNull<[u8]>,
}

impl Report {
    pub(super) fn open() -> io::Result<Self> {
        let buffer = NonNull::from(Box::leak(vec![0_u8; REPORT_BYTES].into_boxed_slice()));
        // SAFETY: `buffer` is `REPORT_BYTES` bytes of memory that this report owns, and
        // `Drop` frees it only after `fclose` has closed the stream that writes to it.
        let file = unsafe { sys::fmemopen(buffer.as_ptr().cast(), REPORT_BYTES, c"w".as_ptr()) };
        match NonNull::new(file) {
            Some(file) => Ok(Report { file, buffer }),
            None => {
                let error = io::Error::last_os_error();
                // SAFETY: `buffer` came from `Box::leak`, and no stream holds it.
                drop(unsafe { Box::from_raw(buffer.as_ptr()) });
                Err(error)
            }
        }
    }

    /// Builds the Qhull instance of `points` that `builder` configures, writing its
    /// messages to this stream, and gives what `read` reads from it. The instance drops
    /// before this returns, so it never outlives the stream.
    pub(super) fn build<T>(
        &self,
        builder: QhBuilder,
        points: impl IntoIterator<Item = [f64; 3]>,
        read: impl FnOnce(&Qh<'static>) -> T,
    ) -> Result<T, QhError<'static>> {
        let file = self.file.as_ptr();
        // SAFETY: the configuration runs on the live Qhull instance once it has read its
        // options, and through its raw pointer sets the two streams that Qhull writes its
        // messages to. The instance is built and dropped in this call, while this report
        // is borrowed, so the stream stays open while the instance writes to it.
        let builder = unsafe {
            builder.with_configure(move |qh| {
                let qh = &mut *Qh::raw_ptr(qh);
                qh.ferr = file;
                qh.qhmem.ferr = file;
                Ok(())
            })
        };
        let qh = builder.build_from_iter(points)?;
        Ok(read(&qh))
    }

    /// The first error message that Qhull wrote, if any.
    pub(super) fn error(&self) -> Option<String> {
        // SAFETY: the stream is open; the flush writes its buffered bytes into `buffer`.
        unsafe { sys::fflush(self.file.as_ptr()) };
        // SAFETY: `buffer` is live, and nothing writes to it while this slice lives: Qhull
        // writes only through the stream, whose bytes the flush has written out.
        let bytes = unsafe { self.buffer.as_ref() };
        let written = bytes.split(|&byte| byte == 0).next().unwrap_or_default();
        String::from_utf8_lossy(written)
            .lines()
            .find(|line| line.starts_with(ERROR_MESSAGE))
            .map(|line| line.trim_end().to_owned())
    }
}

impl Drop for Report {
    fn drop(&mut self) {
        // SAFETY: the stream is open, the Qhull instance that wrote to it is gone, and this
        // closes it once.
        unsafe { sys::fclose(self.file.as_ptr()) };
        // SAFETY: `buffer` came from `Box::leak`, and the stream that held it is closed.
        drop(unsafe { Box::from_raw(self.buffer.as_ptr()) });
    }
}
