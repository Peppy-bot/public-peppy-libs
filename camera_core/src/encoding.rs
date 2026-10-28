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

    #[test]
    fn a_name_is_spelled_exactly() {
        for other in ["RGB8", "Mjpeg", "z16", "", "nv12"] {
            assert_eq!(ColorEncoding::from_name(other), None, "{other:?}");
        }
    }
}
