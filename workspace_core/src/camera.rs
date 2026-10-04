//! The camera a robot finds items with, and what it sees.

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

/// The pinhole model of a camera's colour stream, in pixels, under OpenCV's
/// conventions (those of `camera_geometry:v1`): `u` runs to the right of the
/// image and `v` down it, and the centre of the top-left pixel is (0, 0).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Intrinsics {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

impl Intrinsics {
    /// The model of an ideal pinhole with square pixels whose image of
    /// `width` by `height` pixels spans `fovy` radians vertically, its
    /// optical axis through the middle of the image: a rendered camera's.
    pub fn from_vertical_fov(fovy: f64, width: u32, height: u32) -> Self {
        let f = f64::from(height) / 2.0 / (fovy / 2.0).tan();
        Self {
            width,
            height,
            fx: f,
            fy: f,
            cx: (f64::from(width) - 1.0) / 2.0,
            cy: (f64::from(height) - 1.0) / 2.0,
        }
    }

    /// Whether the image holds the point `(x, y, z)` of the optical frame,
    /// `z` ahead of the camera: its pixel falls within the image's edges.
    fn holds(&self, [x, y, z]: [f64; 3]) -> bool {
        if z <= 0.0 {
            return false;
        }
        let u = self.fx * x / z + self.cx;
        let v = self.fy * y / z + self.cy;
        let inside = |pixel: f64, size: u32| (-0.5..=f64::from(size) - 0.5).contains(&pixel);
        inside(u, self.width) && inside(v, self.height)
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
    /// The pinhole model of its colour stream.
    pub intrinsics: Intrinsics,
    /// The depths it measures along its optical axis, nearest and farthest,
    /// in metres; none when it gives no depth.
    pub depth_range: Option<[f64; 2]>,
}

impl Camera {
    /// What the camera makes of `point`: seen when it lies inside its field
    /// of view and its depth range. What may hide the point is not asked.
    pub fn view_of(&self, point: [f64; 3]) -> View {
        let local = self.in_optical_frame(point);
        if !self.intrinsics.holds(local) {
            return View::OutsideField;
        }
        let depth = local[2];
        if self
            .depth_range
            .is_some_and(|[near, far]| depth < near || depth > far)
        {
            return View::OutOfDepth;
        }
        View::Seen
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

/// What `camera`, the robot's perception camera, makes of `point`:
/// [`View::NoCamera`] for a robot without one.
pub fn view_of(camera: Option<&Camera>, point: [f64; 3]) -> View {
    camera.map_or(View::NoCamera, |camera| camera.view_of(point))
}

#[cfg(test)]
mod tests {
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
            intrinsics: Intrinsics::from_vertical_fov(60f64.to_radians(), 64, 48),
            depth_range: Some([0.2, 3.0]),
        }
    }

    #[test]
    fn a_rendered_cameras_model_has_square_pixels_and_its_axis_through_the_middle() {
        let model = Intrinsics::from_vertical_fov(60f64.to_radians(), 64, 48);
        assert!((model.fy - 24.0 / 30f64.to_radians().tan()).abs() < 1e-9);
        assert_eq!(model.fx, model.fy);
        assert_eq!((model.cx, model.cy), (31.5, 23.5));
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
    fn a_camera_without_depth_measures_no_depth_range_and_no_camera_is_not_asked() {
        let colour_only = Camera {
            depth_range: None,
            ..ahead()
        };
        assert_eq!(colour_only.view_of([0.1, 0.0, 1.0]), View::Seen);
        assert_eq!(view_of(None, [1.0, 0.0, 1.0]), View::NoCamera);
        assert_eq!(view_of(Some(&ahead()), [1.0, 0.0, 1.0]), View::Seen);
    }
}
