//! Downsampled RGB extraction from camera frames, feeding image stats only.
//! The video pillar never touches pixels (ffmpeg converts); this path exists
//! because stats need RGB values. Mirrors lerobot's
//! `auto_downsample_height_width` stride rule.

use crate::config::{CameraSpec, SourceEncoding};
use crate::error::FrameError;
use crate::frame::PixelFrame;

const TARGET_SIZE: u32 = 150;
const MAX_SIZE_THRESHOLD: u32 = 300;

pub fn downsample_factor(width: u32, height: u32) -> u32 {
    if width.max(height) < MAX_SIZE_THRESHOLD {
        return 1;
    }
    if width > height {
        width / TARGET_SIZE
    } else {
        height / TARGET_SIZE
    }
}

/// Interleaved RGB of the frame, stride-downsampled. Used only for stats, so
/// YUYV uses the standard BT.601 limited-range conversion as a close stand-in
/// for whatever the camera's exact matrix was.
pub fn downsampled_rgb(spec: &CameraSpec, frame: &PixelFrame<'_>) -> Result<Vec<u8>, FrameError> {
    let (w, h) = (spec.width.get(), spec.height.get());
    match frame.encoding {
        SourceEncoding::Mjpeg => {
            let mut decoder = zune_jpeg::JpegDecoder::new(frame.bytes);
            decoder.set_options(
                zune_core::options::DecoderOptions::default()
                    .jpeg_set_out_colorspace(zune_core::colorspace::ColorSpace::RGB),
            );
            let pixels = decoder
                .decode()
                .map_err(|e| FrameError::JpegDecode(e.to_string()))?;
            let (jw, jh) = decoder
                .dimensions()
                .expect("dimensions available after decode");
            Ok(stride_rgb(&pixels, jw as u32, jh as u32))
        }
        SourceEncoding::Rgb8 => Ok(stride_pixels(frame.bytes, w, h, |px| [px[0], px[1], px[2]])),
        SourceEncoding::Bgr8 => Ok(stride_pixels(frame.bytes, w, h, |px| [px[2], px[1], px[0]])),
        SourceEncoding::Yuyv => Ok(stride_yuyv(frame.bytes, w, h)),
    }
}

fn stride_rgb(rgb: &[u8], width: u32, height: u32) -> Vec<u8> {
    stride_pixels(rgb, width, height, |px| [px[0], px[1], px[2]])
}

fn stride_pixels(
    bytes: &[u8],
    width: u32,
    height: u32,
    to_rgb: impl Fn(&[u8]) -> [u8; 3],
) -> Vec<u8> {
    let factor = downsample_factor(width, height) as usize;
    let (width, height) = (width as usize, height as usize);
    let mut out = Vec::with_capacity(width.div_ceil(factor) * height.div_ceil(factor) * 3);
    for y in (0..height).step_by(factor) {
        for x in (0..width).step_by(factor) {
            let base = (y * width + x) * 3;
            out.extend(to_rgb(&bytes[base..base + 3]));
        }
    }
    out
}

fn stride_yuyv(bytes: &[u8], width: u32, height: u32) -> Vec<u8> {
    let factor = downsample_factor(width, height) as usize;
    let (width, height) = (width as usize, height as usize);
    let mut out = Vec::with_capacity(width.div_ceil(factor) * height.div_ceil(factor) * 3);
    for y in (0..height).step_by(factor) {
        for x in (0..width).step_by(factor) {
            let index = y * width + x;
            let luma = bytes[index * 2] as i32;
            let pair = (index & !1) * 2;
            let cb = bytes[pair + 1] as i32;
            let cr = bytes[pair + 3] as i32;
            out.extend(bt601_to_rgb(luma, cb, cr));
        }
    }
    out
}

fn bt601_to_rgb(luma: i32, cb: i32, cr: i32) -> [u8; 3] {
    let c = 298 * (luma - 16);
    let d = cb - 128;
    let e = cr - 128;
    let clamp = |v: i32| ((v + 128) >> 8).clamp(0, 255) as u8;
    [
        clamp(c + 409 * e),
        clamp(c - 100 * d - 208 * e),
        clamp(c + 516 * d),
    ]
}

#[cfg(test)]
mod tests {
    use std::num::NonZeroU32;

    use super::*;

    fn spec(w: u32, h: u32, source: SourceEncoding) -> CameraSpec {
        CameraSpec {
            width: NonZeroU32::new(w).unwrap(),
            height: NonZeroU32::new(h).unwrap(),
            source,
        }
    }

    #[test]
    fn factor_matches_lerobot_rule() {
        assert_eq!(downsample_factor(64, 48), 1);
        assert_eq!(downsample_factor(299, 200), 1);
        assert_eq!(downsample_factor(640, 480), 4);
        assert_eq!(downsample_factor(480, 640), 4);
        assert_eq!(downsample_factor(1280, 720), 8);
    }

    #[test]
    fn rgb_passthrough_and_bgr_swap() {
        let spec_rgb = spec(2, 1, SourceEncoding::Rgb8);
        let frame = PixelFrame::rgb8(spec_rgb.width, spec_rgb.height, &[1, 2, 3, 4, 5, 6]).unwrap();
        assert_eq!(
            downsampled_rgb(&spec_rgb, &frame).unwrap(),
            [1, 2, 3, 4, 5, 6]
        );

        let spec_bgr = spec(1, 1, SourceEncoding::Bgr8);
        let frame = PixelFrame::bgr8(spec_bgr.width, spec_bgr.height, &[10, 20, 30]).unwrap();
        assert_eq!(downsampled_rgb(&spec_bgr, &frame).unwrap(), [30, 20, 10]);
    }

    #[test]
    fn yuyv_grey_converts_to_grey() {
        // Y=128, U=V=128 is mid grey in BT.601 limited range.
        let spec = spec(2, 1, SourceEncoding::Yuyv);
        let frame = PixelFrame::yuyv(spec.width, spec.height, &[128, 128, 128, 128]).unwrap();
        let rgb = downsampled_rgb(&spec, &frame).unwrap();
        assert_eq!(rgb.len(), 6);
        for v in rgb {
            assert!((v as i32 - 130).abs() <= 2, "expected mid grey, got {v}");
        }
    }

    #[test]
    fn large_frames_are_strided() {
        let spec = spec(600, 300, SourceEncoding::Rgb8);
        let bytes = vec![7u8; 600 * 300 * 3];
        let frame = PixelFrame::rgb8(spec.width, spec.height, &bytes).unwrap();
        let rgb = downsampled_rgb(&spec, &frame).unwrap();
        assert_eq!(rgb.len(), 150 * 75 * 3);
    }
}
