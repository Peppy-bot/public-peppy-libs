//! The name of a coordinate frame, as tf names one: the frame a map is kept
//! in, and the frame a scan's sensor and a cutout stand in.

use std::fmt;

/// A coordinate frame's name: not empty, and not starting with a slash, as
/// tf2 takes a frame id.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame(String);

/// A name tf2 takes as no frame.
#[derive(Debug, thiserror::Error, PartialEq)]
#[error(
    "frame name {0:?} is not one tf takes: give a name that is not empty and does not start with '/'"
)]
pub struct FrameError(String);

impl Frame {
    /// The frame `name` names.
    pub fn parse(name: &str) -> Result<Self, FrameError> {
        if name.is_empty() || name.starts_with('/') {
            return Err(FrameError(name.to_owned()));
        }
        Ok(Self(name.to_owned()))
    }
}

impl fmt::Display for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_is_a_name_tf_takes() {
        assert_eq!(Frame::parse("world").expect("a name").to_string(), "world");
        for refused in ["", "/world"] {
            assert_eq!(Frame::parse(refused), Err(FrameError(refused.to_owned())));
        }
        assert_eq!(
            FrameError("/world".to_owned()).to_string(),
            "frame name \"/world\" is not one tf takes: give a name that is not empty and does \
             not start with '/'"
        );
    }
}
