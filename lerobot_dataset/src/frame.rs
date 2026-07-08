use std::num::NonZeroU32;

use crate::config::{CameraId, SourceEncoding, VectorId};
use crate::error::FrameError;

/// One camera frame whose buffer length has been proven against its encoding.
#[derive(Debug, Clone, Copy)]
pub struct PixelFrame<'a> {
    pub(crate) encoding: SourceEncoding,
    pub(crate) bytes: &'a [u8],
}

impl<'a> PixelFrame<'a> {
    pub fn rgb8(
        width: NonZeroU32,
        height: NonZeroU32,
        bytes: &'a [u8],
    ) -> Result<Self, FrameError> {
        Self::sized(SourceEncoding::Rgb8, "rgb8", width, height, 3, bytes)
    }

    pub fn bgr8(
        width: NonZeroU32,
        height: NonZeroU32,
        bytes: &'a [u8],
    ) -> Result<Self, FrameError> {
        Self::sized(SourceEncoding::Bgr8, "bgr8", width, height, 3, bytes)
    }

    pub fn yuyv(
        width: NonZeroU32,
        height: NonZeroU32,
        bytes: &'a [u8],
    ) -> Result<Self, FrameError> {
        Self::sized(SourceEncoding::Yuyv, "yuyv", width, height, 2, bytes)
    }

    pub fn mjpeg(bytes: &'a [u8]) -> Result<Self, FrameError> {
        if bytes.len() < 2 || bytes[0] != 0xFF || bytes[1] != 0xD8 {
            return Err(FrameError::NotAJpeg);
        }
        Ok(Self {
            encoding: SourceEncoding::Mjpeg,
            bytes,
        })
    }

    pub fn encoding(&self) -> SourceEncoding {
        self.encoding
    }

    fn sized(
        encoding: SourceEncoding,
        encoding_name: &'static str,
        width: NonZeroU32,
        height: NonZeroU32,
        bytes_per_pixel: usize,
        bytes: &'a [u8],
    ) -> Result<Self, FrameError> {
        let expected = width.get() as usize * height.get() as usize * bytes_per_pixel;
        if bytes.len() != expected {
            return Err(FrameError::PixelBufferLen {
                encoding: encoding_name,
                width: width.get(),
                height: height.get(),
                expected,
                got: bytes.len(),
            });
        }
        Ok(Self { encoding, bytes })
    }
}

/// One dataset frame: a value for every declared vector feature and camera.
#[derive(Debug, Clone, Copy)]
pub struct Frame<'a> {
    pub vectors: &'a [(VectorId, &'a [f32])],
    pub images: &'a [(CameraId, PixelFrame<'a>)],
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dims() -> (NonZeroU32, NonZeroU32) {
        (NonZeroU32::new(4).unwrap(), NonZeroU32::new(2).unwrap())
    }

    #[test]
    fn accepts_exact_buffer_lengths() {
        let (w, h) = dims();
        assert!(PixelFrame::rgb8(w, h, &[0u8; 24]).is_ok());
        assert!(PixelFrame::bgr8(w, h, &[0u8; 24]).is_ok());
        assert!(PixelFrame::yuyv(w, h, &[0u8; 16]).is_ok());
        assert!(PixelFrame::mjpeg(&[0xFF, 0xD8, 0xFF, 0xE0]).is_ok());
    }

    #[test]
    fn rejects_wrong_lengths_and_magic() {
        let (w, h) = dims();
        assert_eq!(
            PixelFrame::rgb8(w, h, &[0u8; 23]).unwrap_err(),
            FrameError::PixelBufferLen {
                encoding: "rgb8",
                width: 4,
                height: 2,
                expected: 24,
                got: 23,
            }
        );
        assert_eq!(
            PixelFrame::mjpeg(&[0x00, 0x01]).unwrap_err(),
            FrameError::NotAJpeg
        );
        assert_eq!(PixelFrame::mjpeg(&[]).unwrap_err(), FrameError::NotAJpeg);
    }
}
