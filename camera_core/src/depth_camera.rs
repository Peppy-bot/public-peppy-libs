//! A depth camera as camera_geometry:v1 describes it: the lens and the depth
//! model of its depth stream, which `get_depth_intrinsics` gives, where its
//! depth optical frame stands in the camera's frame, its color optical frame,
//! which `get_depth_to_color_extrinsics` gives, and the alignment both answers
//! describe. A node polls the two members itself and fills a
//! [`DepthIntrinsics`] and a [`DepthExtrinsics`] from its generated
//! responses; [`read_depth_camera`] reads them into a [`DepthCamera`], which
//! places the depth optical frame in the robot frame and admits a depth frame
//! whose header carries the camera's alignment and names the camera's frame.

use std::fmt;

use control_core::geometry::{GeometryError, Pose};
use frame_tree::{FrameName, FrameNameError, Transform};

use crate::pinhole::{DepthModel, Intrinsics, Pinhole, PinholeError, UnknownDepthModel};

/// camera_geometry:v1's answer of `get_depth_intrinsics`, which a node's poll
/// fills from its generated response field by field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthIntrinsics<'a> {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub distortion_model: &'a str,
    pub distortion: &'a [f64],
    /// What a depth sample measures, by its camera_geometry:v1 name.
    pub depth_model: &'a str,
    /// The alignment the lens describes.
    pub align_mode: &'a str,
}

/// camera_geometry:v1's answer of `get_depth_to_color_extrinsics`, which a
/// node's poll fills from its generated response field by field: the fields
/// of its `depth_to_color` and its `align_mode`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthExtrinsics<'a> {
    /// The camera's frame, its color optical frame.
    pub frame: &'a str,
    /// The depth optical frame's position in the camera's frame, in m.
    pub position: [f64; 3],
    /// The depth optical frame's orientation in the camera's frame, a
    /// quaternion [x, y, z, w].
    pub orientation: [f64; 4],
    /// The alignment the depth optical frame's pose describes.
    pub align_mode: &'a str,
}

/// A depth camera, read: its depth stream's lens and depth model, the
/// camera's frame, where the depth optical frame stands in it, and the
/// alignment both answers describe.
#[derive(Debug, Clone)]
pub struct DepthCamera {
    pub lens: Pinhole,
    pub depth_model: DepthModel,
    /// The camera's frame, its color optical frame, which each depth frame's
    /// header names.
    pub frame: FrameName,
    /// The depth optical frame in the camera's frame.
    pub depth_in_color: Transform,
    /// The alignment the lens and the depth optical frame's pose describe,
    /// which each depth frame's header carries.
    pub align_mode: String,
}

impl DepthCamera {
    /// Where the depth optical frame stands in the robot frame, the camera's
    /// frame standing at `color_in_robot`: a point goes through
    /// `depth_in_color` first.
    pub fn depth_in_robot(&self, color_in_robot: &Transform) -> Transform {
        color_in_robot.compose(&self.depth_in_color)
    }

    /// Admits a depth frame whose header carries `align_mode` and names
    /// `frame`, the camera's alignment and frame; refused for another
    /// alignment first, then for another frame.
    pub fn admits_frame(&self, align_mode: &str, frame: &str) -> Result<(), HeaderMismatch> {
        if align_mode != self.align_mode {
            return Err(HeaderMismatch::AlignMode {
                header: align_mode.to_owned(),
                camera: self.align_mode.clone(),
            });
        }
        if frame != self.frame.as_str() {
            return Err(HeaderMismatch::Frame {
                header: frame.to_owned(),
                camera: self.frame.clone(),
            });
        }
        Ok(())
    }
}

/// Why camera_geometry's two answers make no [`DepthCamera`].
#[derive(Debug)]
pub enum DepthCameraError {
    /// `get_depth_intrinsics` and `get_depth_to_color_extrinsics` describe
    /// two alignments.
    AlignModes {
        intrinsics: String,
        extrinsics: String,
    },
    /// `get_depth_intrinsics` gives no pinhole model.
    Lens(PinholeError),
    /// `get_depth_intrinsics` names no depth model camera_geometry:v1 names.
    DepthModel(UnknownDepthModel),
    /// `get_depth_to_color_extrinsics` gives a position or an orientation no
    /// contract takes.
    DepthPose(GeometryError),
    /// `get_depth_to_color_extrinsics` gives an empty frame.
    Frame(FrameNameError),
}

impl fmt::Display for DepthCameraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AlignModes {
                intrinsics,
                extrinsics,
            } => write!(
                f,
                "get_depth_intrinsics describes align_mode {intrinsics:?} and \
                 get_depth_to_color_extrinsics {extrinsics:?}"
            ),
            Self::Lens(error) => write!(f, "get_depth_intrinsics: {error}"),
            Self::DepthModel(error) => write!(f, "get_depth_intrinsics: {error}"),
            Self::DepthPose(error) => write!(f, "get_depth_to_color_extrinsics: {error}"),
            Self::Frame(error) => write!(
                f,
                "get_depth_to_color_extrinsics places the depth stream in no frame: {error}"
            ),
        }
    }
}

impl std::error::Error for DepthCameraError {}

/// Why a depth frame is no frame of a [`DepthCamera`], by its header.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HeaderMismatch {
    /// The header carries another alignment than the camera's.
    AlignMode { header: String, camera: String },
    /// The header names another frame than the camera's.
    Frame { header: String, camera: FrameName },
}

impl fmt::Display for HeaderMismatch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AlignMode { header, camera } => write!(
                f,
                "the depth frames carry align_mode {header:?}, and the camera's geometry \
                 describes {camera:?}"
            ),
            Self::Frame { header, camera } => write!(
                f,
                "the depth frames name the frame {header:?}, and the camera's geometry places \
                 the depth stream in {:?}",
                camera.as_str()
            ),
        }
    }
}

impl std::error::Error for HeaderMismatch {}

/// The depth camera `intrinsics` and `extrinsics` describe, its pose
/// normalized as every pose of the contracts is: refused when the two
/// describe two alignments, then for a lens or a depth model
/// camera_geometry:v1 does not take, then for a pose no contract takes, then
/// for an empty frame.
pub fn read_depth_camera(
    intrinsics: &DepthIntrinsics,
    extrinsics: &DepthExtrinsics,
) -> Result<DepthCamera, DepthCameraError> {
    if intrinsics.align_mode != extrinsics.align_mode {
        return Err(DepthCameraError::AlignModes {
            intrinsics: intrinsics.align_mode.to_owned(),
            extrinsics: extrinsics.align_mode.to_owned(),
        });
    }
    let lens = Pinhole::parse(&Intrinsics {
        width: intrinsics.width,
        height: intrinsics.height,
        fx: intrinsics.fx,
        fy: intrinsics.fy,
        cx: intrinsics.cx,
        cy: intrinsics.cy,
        distortion_model: intrinsics.distortion_model,
        distortion: intrinsics.distortion,
    })
    .map_err(DepthCameraError::Lens)?;
    let depth_model =
        DepthModel::parse(intrinsics.depth_model).map_err(DepthCameraError::DepthModel)?;
    let depth_in_color = Pose::parse(extrinsics.position, extrinsics.orientation)
        .map_err(DepthCameraError::DepthPose)?
        .transform();
    let frame = FrameName::parse(extrinsics.frame).map_err(DepthCameraError::Frame)?;
    Ok(DepthCamera {
        lens,
        depth_model,
        frame,
        depth_in_color,
        align_mode: intrinsics.align_mode.to_owned(),
    })
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_1_SQRT_2;

    use super::*;
    use crate::test_support::close;

    const NO_DISTORTION: &[f64] = &[];
    const ALIGNED: &str = "depth_to_color";
    const CAMERA_FRAME: &str = "chest_color_optical_frame";
    /// A quarter turn about +Z, which takes (x, y, z) to (-y, x, z).
    const QUARTER_TURN_ABOUT_Z: [f64; 4] = [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2];

    /// The answer of a 4 by 3 depth stream with a focal length of 2 pixels and
    /// its principal point at (1.5, 1), aligned to its color stream.
    fn intrinsics() -> DepthIntrinsics<'static> {
        DepthIntrinsics {
            width: 4,
            height: 3,
            fx: 2.0,
            fy: 2.0,
            cx: 1.5,
            cy: 1.0,
            distortion_model: "none",
            distortion: NO_DISTORTION,
            depth_model: "z",
            align_mode: ALIGNED,
        }
    }

    /// The answer that places the depth optical frame on the color one of
    /// `CAMERA_FRAME`.
    fn extrinsics() -> DepthExtrinsics<'static> {
        DepthExtrinsics {
            frame: CAMERA_FRAME,
            position: [0.0; 3],
            orientation: [0.0, 0.0, 0.0, 1.0],
            align_mode: ALIGNED,
        }
    }

    fn camera() -> DepthCamera {
        read_depth_camera(&intrinsics(), &extrinsics()).expect("a readable depth camera")
    }

    /// The transform of `position` and `orientation`, as the contracts parse a
    /// pose.
    fn transform(position: [f64; 3], orientation: [f64; 4]) -> Transform {
        Pose::parse(position, orientation)
            .expect("a finite position and a unit quaternion")
            .transform()
    }

    /// The refusal of `intrinsics` and `extrinsics`.
    fn refusal(intrinsics: &DepthIntrinsics, extrinsics: &DepthExtrinsics) -> DepthCameraError {
        read_depth_camera(intrinsics, extrinsics).expect_err("refused")
    }

    #[test]
    fn the_answers_make_the_depth_camera_in_the_frame_the_extrinsics_name() {
        let camera = camera();
        let lens = Pinhole::parse(&Intrinsics {
            width: 4,
            height: 3,
            fx: 2.0,
            fy: 2.0,
            cx: 1.5,
            cy: 1.0,
            distortion_model: "none",
            distortion: NO_DISTORTION,
        })
        .expect("a valid model");
        assert_eq!(camera.lens, lens);
        assert_eq!(camera.depth_model, DepthModel::Z);
        assert_eq!(camera.frame.as_str(), CAMERA_FRAME);
        assert_eq!(camera.align_mode, ALIGNED);
        assert!(close(
            camera.depth_in_color.apply([1.0, 2.0, 3.0]),
            [1.0, 2.0, 3.0],
            1e-12
        ));
    }

    // The depth optical frame stands 0.1 m along the color frame's x, turned a quarter about
    // its z: a point at the depth frame's (1, 0, 0) lies at (0.1, 1, 0) in the color frame.
    #[test]
    fn the_extrinsics_place_the_depth_optical_frame_in_the_color_one() {
        let turned = DepthExtrinsics {
            position: [0.1, 0.0, 0.0],
            orientation: QUARTER_TURN_ABOUT_Z,
            ..extrinsics()
        };
        let camera = read_depth_camera(&intrinsics(), &turned).expect("a readable depth camera");
        assert!(close(
            camera.depth_in_color.apply([1.0, 0.0, 0.0]),
            [0.1, 1.0, 0.0],
            1e-12
        ));
    }

    // The depth_to_color pose parses as each pose of the contracts does: an orientation
    // [0, 0, 0, 1.001], whose squared norm lies 0.002 from 1, within the 0.01 tf2 takes, is
    // taken normalized, so it moves no point.
    #[test]
    fn the_depth_pose_takes_an_orientation_tf2_takes_normalized() {
        let long = DepthExtrinsics {
            orientation: [0.0, 0.0, 0.0, 1.001],
            ..extrinsics()
        };
        let camera = read_depth_camera(&intrinsics(), &long).expect("a readable depth camera");
        assert!(close(
            camera.depth_in_color.apply([1.0, 2.0, 3.0]),
            [1.0, 2.0, 3.0],
            1e-12
        ));
    }

    // The color frame stands at (1, 0, 0) turned a quarter about z; the depth frame stands
    // 0.1 m along the color frame's x, which the turn points along the robot's y: at
    // (1, 0.1, 0) in the robot frame.
    #[test]
    fn the_depth_frame_goes_through_its_color_frame_first() {
        let camera = DepthCamera {
            depth_in_color: transform([0.1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            ..camera()
        };
        let color_in_robot = transform([1.0, 0.0, 0.0], QUARTER_TURN_ABOUT_Z);
        let depth_in_robot = camera.depth_in_robot(&color_in_robot);
        assert!(close(
            depth_in_robot.apply([0.0; 3]),
            [1.0, 0.1, 0.0],
            1e-12
        ));
    }

    #[test]
    fn answers_of_two_alignments_are_refused_naming_both() {
        let other = DepthExtrinsics {
            align_mode: "none",
            ..extrinsics()
        };
        let error = refusal(&intrinsics(), &other);
        assert!(
            matches!(
                &error,
                DepthCameraError::AlignModes { intrinsics, extrinsics }
                    if intrinsics == ALIGNED && extrinsics == "none"
            ),
            "{error:?}"
        );
        assert_eq!(
            error.to_string(),
            "get_depth_intrinsics describes align_mode \"depth_to_color\" and \
             get_depth_to_color_extrinsics \"none\""
        );
        // Two alignments are refused before an unreadable lens.
        let flat = DepthIntrinsics {
            fx: 0.0,
            ..intrinsics()
        };
        assert!(matches!(
            refusal(&flat, &other),
            DepthCameraError::AlignModes { .. }
        ));
    }

    // Each answer the camera cannot be read from is refused naming the member that gave it
    // and the value: a lens of a focal length of 0, an unknown depth model, an orientation
    // whose squared norm is 4, a position that is not a number, and an empty frame.
    #[test]
    fn answers_that_describe_no_depth_camera_are_refused_naming_the_member_and_the_value() {
        let flat = refusal(
            &DepthIntrinsics {
                fx: 0.0,
                ..intrinsics()
            },
            &extrinsics(),
        );
        assert!(matches!(flat, DepthCameraError::Lens(_)), "{flat:?}");
        assert!(
            flat.to_string()
                .starts_with("get_depth_intrinsics: the pinhole model 4x3 fx 0 "),
            "{flat}"
        );
        let disparity = refusal(
            &DepthIntrinsics {
                depth_model: "disparity",
                ..intrinsics()
            },
            &extrinsics(),
        );
        assert!(
            matches!(disparity, DepthCameraError::DepthModel(_)),
            "{disparity:?}"
        );
        assert!(
            disparity
                .to_string()
                .starts_with("get_depth_intrinsics: unknown depth_model 'disparity'"),
            "{disparity}"
        );
        let long = refusal(
            &intrinsics(),
            &DepthExtrinsics {
                orientation: [0.0, 0.0, 0.0, 2.0],
                ..extrinsics()
            },
        );
        assert!(matches!(long, DepthCameraError::DepthPose(_)), "{long:?}");
        assert!(
            long.to_string().starts_with(
                "get_depth_to_color_extrinsics: the orientation [0, 0, 0, 2] has a squared \
                 norm of 4"
            ),
            "{long}"
        );
        let nowhere = refusal(
            &intrinsics(),
            &DepthExtrinsics {
                position: [f64::NAN, 0.0, 0.0],
                ..extrinsics()
            },
        );
        assert!(
            matches!(nowhere, DepthCameraError::DepthPose(_)),
            "{nowhere:?}"
        );
        assert!(
            nowhere
                .to_string()
                .starts_with("get_depth_to_color_extrinsics: ")
                && nowhere.to_string().contains("NaN"),
            "{nowhere}"
        );
        let unframed = refusal(
            &intrinsics(),
            &DepthExtrinsics {
                frame: "",
                ..extrinsics()
            },
        );
        assert!(
            matches!(unframed, DepthCameraError::Frame(_)),
            "{unframed:?}"
        );
        assert_eq!(
            unframed.to_string(),
            "get_depth_to_color_extrinsics places the depth stream in no frame: a frame name is \
             empty: name a link of the robot's URDF, odom or map"
        );
    }

    #[test]
    fn a_depth_frame_of_the_cameras_alignment_and_frame_is_admitted() {
        assert_eq!(camera().admits_frame(ALIGNED, CAMERA_FRAME), Ok(()));
    }

    // A frame of another alignment, or naming another frame, is refused naming both values;
    // a frame that differs in both is refused for its alignment.
    #[test]
    fn a_depth_frame_of_another_alignment_or_frame_is_refused_naming_both() {
        let camera = camera();
        let other_alignment = camera
            .admits_frame("none", CAMERA_FRAME)
            .expect_err("another alignment");
        assert_eq!(
            other_alignment,
            HeaderMismatch::AlignMode {
                header: "none".to_owned(),
                camera: ALIGNED.to_owned(),
            }
        );
        assert_eq!(
            other_alignment.to_string(),
            "the depth frames carry align_mode \"none\", and the camera's geometry describes \
             \"depth_to_color\""
        );
        let other_frame = camera
            .admits_frame(ALIGNED, "head_color_optical_frame")
            .expect_err("another frame");
        assert_eq!(
            other_frame,
            HeaderMismatch::Frame {
                header: "head_color_optical_frame".to_owned(),
                camera: FrameName::parse(CAMERA_FRAME).expect("a frame name"),
            }
        );
        assert_eq!(
            other_frame.to_string(),
            "the depth frames name the frame \"head_color_optical_frame\", and the camera's \
             geometry places the depth stream in \"chest_color_optical_frame\""
        );
        assert!(matches!(
            camera.admits_frame("none", ""),
            Err(HeaderMismatch::AlignMode { .. })
        ));
    }
}
