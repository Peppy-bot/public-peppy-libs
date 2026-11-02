//! The camera a robot finds items with, and what it sees.

use camera_core::pinhole::{DepthModel, Pinhole};

use crate::verdict::View;

/// What the perception camera rule reads of one of a robot's cameras.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CameraFacts<'a> {
    /// The name the camera runs under on the robot (`chest`, `wrist_left`).
    pub name: &'a str,
    /// Whether the camera gives depth.
    pub gives_depth: bool,
    /// Whether an arm carries the camera, so that it moves with the arm.
    pub carried_by_arm: bool,
}

/// A robot carries more than one camera that could be its perception
/// camera.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error(
    "the robot carries {} depth cameras that no arm carries ({}), so it has no single perception camera",
    .0.len(),
    .0.join(", ")
)]
pub struct PerceptionCameraError(pub Vec<String>);

/// The robot's perception camera, of `cameras`: the one camera that gives
/// depth and that no arm carries, which stays where it is between a scan and
/// the next action. None when no camera is both; an error naming them when
/// more than one is.
pub fn perception_camera<'a>(
    cameras: &[CameraFacts<'a>],
) -> Result<Option<&'a str>, PerceptionCameraError> {
    let candidates: Vec<&'a str> = cameras
        .iter()
        .filter(|camera| camera.gives_depth && !camera.carried_by_arm)
        .map(|camera| camera.name)
        .collect();
    match candidates.as_slice() {
        [] => Ok(None),
        [one] => Ok(Some(one)),
        several => Err(PerceptionCameraError(
            several.iter().map(|name| name.to_string()).collect(),
        )),
    }
}

/// What a camera's depth stream measures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Depth {
    pub model: DepthModel,
    /// The samples it can read, nearest and farthest, in metres: a point
    /// whose sample falls outside them reads no depth.
    pub range: [f64; 2],
}

impl Depth {
    /// Whether the stream reads a depth for `local`, a point of the
    /// camera's optical frame.
    fn reads(&self, local: [f64; 3]) -> bool {
        let [nearest, farthest] = self.range;
        (nearest..=farthest).contains(&self.model.reading(local))
    }
}

/// A camera placed in a frame, and what it measures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Camera {
    /// Where its optical frame stands, in the frame of the points it is asked
    /// about.
    pub position: [f64; 3],
    /// The rotation of its optical frame in the frame of the points it is
    /// asked about, row-major: its columns are the optical frame's +X (to
    /// the right of the image), +Y (down it) and +Z (along the view).
    pub rotation: [f64; 9],
    /// The pinhole model of its colour stream, with its lens.
    pub pinhole: Pinhole,
    /// What its depth stream measures; none when it gives no depth.
    pub depth: Option<Depth>,
}

impl Camera {
    /// What the camera makes of `point`: seen when it lies inside its field
    /// of view and its depth stream reads a depth for it. What may hide the
    /// point is not asked.
    pub fn view_of(&self, point: [f64; 3]) -> View {
        let local = self.in_optical_frame(point);
        if !self.image_shows(local) {
            return View::OutsideField;
        }
        if self.depth.is_some_and(|depth| !depth.reads(local)) {
            return View::OutOfDepth;
        }
        View::Seen
    }

    /// Whether a pixel of the colour image shows `local`, a point of the
    /// optical frame: the pixel the pinhole model gives for it falls within
    /// the image's edges.
    fn image_shows(&self, local: [f64; 3]) -> bool {
        let inside = |pixel: f64, size: u32| (-0.5..=f64::from(size) - 0.5).contains(&pixel);
        self.pinhole.project(local).is_some_and(|(u, v)| {
            inside(u, self.pinhole.width()) && inside(v, self.pinhole.height())
        })
    }

    /// `point` in the camera's optical frame.
    fn in_optical_frame(&self, point: [f64; 3]) -> [f64; 3] {
        let d = std::array::from_fn::<f64, 3, _>(|i| point[i] - self.position[i]);
        let r = &self.rotation;
        // The transpose of the rotation takes a direction of the frame into
        // the optical frame: each optical axis, a column, dotted with it.
        std::array::from_fn(|axis| r[axis] * d[0] + r[3 + axis] * d[1] + r[6 + axis] * d[2])
    }
}

#[cfg(test)]
mod tests {
    use camera_core::pinhole::Intrinsics;

    use super::*;

    fn facts(name: &str, gives_depth: bool, carried_by_arm: bool) -> CameraFacts<'_> {
        CameraFacts {
            name,
            gives_depth,
            carried_by_arm,
        }
    }

    #[test]
    fn the_perception_camera_is_the_one_depth_camera_no_arm_carries() {
        let cameras = [
            facts("wrist_left", false, true),
            facts("wrist_right", false, true),
            facts("chest", true, false),
        ];
        assert_eq!(perception_camera(&cameras), Ok(Some("chest")));
    }

    #[test]
    fn a_depth_camera_an_arm_carries_is_no_perception_camera_and_neither_is_a_colour_one() {
        assert_eq!(perception_camera(&[facts("wrist", true, true)]), Ok(None));
        assert_eq!(perception_camera(&[facts("head", false, false)]), Ok(None));
        assert_eq!(perception_camera(&[]), Ok(None));
    }

    #[test]
    fn two_depth_cameras_on_the_base_are_refused_by_name() {
        let error = perception_camera(&[facts("chest", true, false), facts("head", true, false)])
            .unwrap_err();
        assert_eq!(
            error,
            PerceptionCameraError(vec!["chest".into(), "head".into()])
        );
        assert_eq!(
            error.to_string(),
            "the robot carries 2 depth cameras that no arm carries (chest, head), so it has no single perception camera"
        );
    }

    /// A camera 1 m up at the origin looking straight along +X, its image
    /// 64 by 48 pixels over a 60° vertical field of view, measuring depths
    /// from 0.2 to 3 m.
    fn ahead() -> Camera {
        Camera {
            position: [0.0, 0.0, 1.0],
            // Optical +X (image right) is the frame's -Y, +Y (image down) its
            // -Z, +Z (the view) its +X.
            rotation: [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0],
            pinhole: Pinhole::from_vertical_fov(60f64.to_radians(), 64, 48).expect("a model"),
            depth: Some(Depth {
                model: DepthModel::Z,
                range: [0.2, 3.0],
            }),
        }
    }

    #[test]
    fn the_camera_sees_a_point_inside_its_field_of_view_and_its_depth_range() {
        let camera = ahead();
        assert_eq!(
            camera.view_of([1.0, 0.0, 1.0]),
            View::Seen,
            "straight ahead"
        );
        let half_v = 30f64.to_radians().tan();
        let half_h = half_v * 64.0 / 48.0;
        assert_eq!(
            camera.view_of([1.0, 0.99 * half_h, 1.0]),
            View::Seen,
            "just inside the left edge"
        );
        assert_eq!(
            camera.view_of([1.0, 1.01 * half_h, 1.0]),
            View::OutsideField,
            "beyond the left edge"
        );
        assert_eq!(
            camera.view_of([1.0, -1.01 * half_h, 1.0]),
            View::OutsideField,
            "beyond the right edge"
        );
        assert_eq!(
            camera.view_of([1.0, 0.0, 1.0 - 0.99 * half_v]),
            View::Seen,
            "just above the lower edge"
        );
        assert_eq!(
            camera.view_of([1.0, 0.0, 1.0 - 1.01 * half_v]),
            View::OutsideField,
            "below the lower edge"
        );
        assert_eq!(
            camera.view_of([1.0, 0.0, 1.0 + 1.01 * half_v]),
            View::OutsideField,
            "above the upper edge"
        );
        assert_eq!(
            camera.view_of([-1.0, 0.0, 1.0]),
            View::OutsideField,
            "behind it"
        );
        assert_eq!(
            camera.view_of([0.1, 0.0, 1.0]),
            View::OutOfDepth,
            "nearer than its depth range"
        );
        assert_eq!(
            camera.view_of([3.5, 0.0, 1.0]),
            View::OutOfDepth,
            "farther than its depth range"
        );
    }

    #[test]
    fn a_range_camera_reads_the_straight_line_distance_and_a_z_camera_the_optical_z() {
        // 2.9 m ahead and 0.8 m to the left: optical z 2.9, inside the depth
        // range, but 3.01 m from the camera, beyond it.
        let point = [2.9, 0.8, 1.0];
        assert_eq!(ahead().view_of(point), View::Seen);
        let range = Camera {
            depth: Some(Depth {
                model: DepthModel::Range,
                range: [0.2, 3.0],
            }),
            ..ahead()
        };
        assert_eq!(range.view_of(point), View::OutOfDepth);
        assert_eq!(range.view_of([2.9, 0.7, 1.0]), View::Seen, "2.98 m away");
    }

    #[test]
    fn a_camera_sees_through_its_lens_and_not_past_where_the_lens_turns_back() {
        // A barrel lens pulls a point just beyond the ideal image's left edge
        // into the image. About 66 degrees off the axis its radial factor
        // falls to 0, which takes a point there to the middle of the image,
        // whose pixel looks along the axis.
        let focal = 24.0 / 30f64.to_radians().tan();
        let barrel = Camera {
            pinhole: Pinhole::parse(&Intrinsics {
                width: 64,
                height: 48,
                fx: focal,
                fy: focal,
                cx: 31.5,
                cy: 23.5,
                distortion_model: "plumb_bob",
                distortion: &[-0.2, 0.0, 0.0, 0.0, 0.0],
            })
            .expect("a model"),
            ..ahead()
        };
        let just_beyond = [1.0, 1.01 * 32.0 / focal, 1.0];
        assert_eq!(ahead().view_of(just_beyond), View::OutsideField);
        assert_eq!(barrel.view_of(just_beyond), View::Seen);
        assert_eq!(barrel.view_of([1.0, -5f64.sqrt(), 1.0]), View::OutsideField);
    }

    #[test]
    fn a_point_above_the_camera_in_front_of_a_camera_that_looks_down_is_outside_its_field() {
        // Tilted 60° below the horizon: the optical axis is (cos 60°, 0,
        // -sin 60°) and the image's down (-sin 60°, 0, -cos 60°).
        let (s, c) = 60f64.to_radians().sin_cos();
        let camera = Camera {
            rotation: [0.0, -s, c, -1.0, 0.0, 0.0, 0.0, -c, -s],
            ..ahead()
        };
        assert_eq!(
            camera.view_of([0.5, 0.0, 0.0]),
            View::Seen,
            "on the floor ahead"
        );
        assert_eq!(
            camera.view_of([0.3, 0.0, 1.2]),
            View::OutsideField,
            "above the camera, in front"
        );
    }

    #[test]
    fn a_camera_without_depth_reads_every_depth() {
        let colour_only = Camera {
            depth: None,
            ..ahead()
        };
        assert_eq!(colour_only.view_of([0.1, 0.0, 1.0]), View::Seen);
    }
}
