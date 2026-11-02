//! The encodings a camera's streams carry, by the names the topics spell.

use std::fmt;

/// The encoding of a color frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorEncoding {
    /// 8-bit packed `R,G,B`. 3 bytes/pixel.
    Rgb8,
    /// 8-bit packed `B,G,R`. OpenCV's default channel order.
    Bgr8,
    /// JPEG-compressed: smallest on the wire, and a consumer needs a JPEG
    /// decoder.
    Mjpeg,
    /// YCbCr 4:2:2 packed (Y0,U,Y1,V). 2 bytes/pixel.
    Yuyv,
}

impl ColorEncoding {
    pub const ALL: [Self; 4] = [Self::Rgb8, Self::Bgr8, Self::Mjpeg, Self::Yuyv];

    /// The name a topic's `encoding` field carries.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Rgb8 => "rgb8",
            Self::Bgr8 => "bgr8",
            Self::Mjpeg => "mjpeg",
            Self::Yuyv => "yuyv",
        }
    }

    /// The encoding `name` spells exactly. Each node words its own refusal of
    /// any other name.
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|encoding| encoding.name() == name)
    }
}

impl fmt::Display for ColorEncoding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// The encoding of a depth frame: one little-endian u16 sample per pixel.
pub const DEPTH_ENCODING: &str = "z16";

/// A depth frame whose length is its size's: one little-endian u16 sample per
/// pixel, row by row from the top left.
#[derive(Debug)]
pub struct DepthFrame<'a> {
    bytes: &'a [u8],
    width: u32,
    height: u32,
}

/// A depth frame whose length is not its size's.
#[derive(Debug, PartialEq)]
pub struct DepthFrameSizeError {
    bytes: usize,
    width: u32,
    height: u32,
    expected: usize,
}

impl fmt::Display for DepthFrameSizeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Self {
            bytes,
            width,
            height,
            expected,
        } = self;
        write!(
            f,
            "the depth frame is {bytes} bytes, a {width}x{height} {DEPTH_ENCODING} frame is {expected}"
        )
    }
}

impl std::error::Error for DepthFrameSizeError {}

impl<'a> DepthFrame<'a> {
    /// `bytes` as a `width` by `height` frame.
    pub fn parse(bytes: &'a [u8], width: u32, height: u32) -> Result<Self, DepthFrameSizeError> {
        let expected = width as usize * height as usize * 2;
        if bytes.len() != expected {
            return Err(DepthFrameSizeError {
                bytes: bytes.len(),
                width,
                height,
                expected,
            });
        }
        Ok(Self {
            bytes,
            width,
            height,
        })
    }

    /// The sample at column `u` and row `v`, when the frame holds that pixel.
    pub fn sample(&self, u: u32, v: u32) -> Option<u16> {
        if u >= self.width || v >= self.height {
            return None;
        }
        let at = (v as usize * self.width as usize + u as usize) * 2;
        Some(u16::from_le_bytes([self.bytes[at], self.bytes[at + 1]]))
    }

    /// Each pixel's column, row and sample, row by row from the top left.
    pub fn samples(&self) -> impl Iterator<Item = (u32, u32, u16)> + '_ {
        let width = self.width as usize;
        self.bytes
            .chunks_exact(2)
            .enumerate()
            .map(move |(index, sample)| {
                (
                    (index % width) as u32,
                    (index / width) as u32,
                    u16::from_le_bytes([sample[0], sample[1]]),
                )
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_encoding_parses_back_from_its_name() {
        for encoding in ColorEncoding::ALL {
            assert_eq!(ColorEncoding::from_name(encoding.name()), Some(encoding));
            assert_eq!(encoding.to_string(), encoding.name());
        }
    }

    #[test]
    fn the_names_are_the_topic_spellings() {
        assert_eq!(
            ColorEncoding::ALL.map(ColorEncoding::name),
            ["rgb8", "bgr8", "mjpeg", "yuyv"]
        );
        assert_eq!(DEPTH_ENCODING, "z16");
    }

    // A frame reads each pixel's sample where its row and column put it,
    // and a frame of another size's length is refused with both lengths.
    #[test]
    fn a_depth_frame_reads_its_samples_row_by_row() {
        let samples: [u16; 6] = [1, 2, 3, 400, 500, 600];
        let bytes: Vec<u8> = samples.iter().flat_map(|s| s.to_le_bytes()).collect();
        let frame = DepthFrame::parse(&bytes, 3, 2).expect("a 3x2 frame");
        assert_eq!(frame.sample(1, 1), Some(500));
        assert_eq!(frame.sample(3, 0), None);
        assert_eq!(frame.sample(0, 2), None);
        assert_eq!(
            frame.samples().collect::<Vec<_>>(),
            [
                (0, 0, 1),
                (1, 0, 2),
                (2, 0, 3),
                (0, 1, 400),
                (1, 1, 500),
                (2, 1, 600)
            ]
        );
        assert_eq!(
            DepthFrame::parse(&bytes, 2, 2).unwrap_err().to_string(),
            "the depth frame is 12 bytes, a 2x2 z16 frame is 8"
        );
    }

    #[test]
    fn a_name_is_spelled_exactly() {
        for other in ["RGB8", "Mjpeg", "z16", "", "nv12"] {
            assert_eq!(ColorEncoding::from_name(other), None, "{other:?}");
        }
    }
}
