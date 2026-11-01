//! A stream's pinhole model and the arithmetic between its pixels and the
//! points of its optical frame, under the conventions of camera_geometry:v1,
//! which are OpenCV's. The center of the first pixel is (0, 0); the optical
//! frame has +X to the right of the image, +Y down it and +Z along the view.

use std::fmt;

use crate::geometry::{Z_DEPTH_MODEL, pinhole_is_valid};

/// The most iterations of the fixed-point inverse of the Brown-Conrady
/// polynomial, whichever way a model runs it; it stops sooner at the point
/// it holds. A calibrated lens converges in a handful. Near where the
/// polynomial turns back it converges slowly, as one over the iterations: on
/// the lens k1 = -0.2 of a 64 by 48 image over 60 degrees, whose image
/// reaches past that fold, 100 bring 128,212 points drawn inside the fold
/// within 0.32 of a pixel, where 20 leave those from 97% of the fold's
/// radius out more than half a pixel off.
const UNDISTORT_ITERATIONS: usize = 100;

/// How far, in pixels, the ray of the pixel a lens takes a point to may pass
/// from the point for that pixel to show it: within the pixel. Past where a
/// lens polynomial turns back, it takes a point from outside the field into
/// the image, and that pixel's ray is another one.
const SHOWN_WITHIN_PIXELS: f64 = 0.5;

/// One stream's intrinsics as a camera_geometry:v1 answer carries them, for
/// [`Pinhole::parse`] to read.
pub struct Intrinsics<'a> {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub distortion_model: &'a str,
    pub distortion: &'a [f64],
}

/// The lens of a stream, as camera_geometry:v1 names it: none; "plumb_bob",
/// OpenCV's polynomial taking an ideal point to the distorted one; or
/// "inverse_plumb_bob", the same polynomial run from the distorted point to
/// the ideal one.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Lens {
    None,
    PlumbBob([f64; 5]),
    InversePlumbBob([f64; 5]),
}

impl Lens {
    /// The lens `model` names with `coefficients`, k1, k2, p1, p2, k3 for a
    /// polynomial, each a number; none for any other pair.
    fn parse(model: &str, coefficients: &[f64]) -> Option<Self> {
        let numbers = coefficients.iter().all(|c| c.is_finite());
        match (model, coefficients) {
            ("none", []) => Some(Lens::None),
            ("plumb_bob", &[k1, k2, p1, p2, k3]) if numbers => {
                Some(Lens::PlumbBob([k1, k2, p1, p2, k3]))
            }
            ("inverse_plumb_bob", &[k1, k2, p1, p2, k3]) if numbers => {
                Some(Lens::InversePlumbBob([k1, k2, p1, p2, k3]))
            }
            _ => None,
        }
    }
}

/// One stream's pinhole model: an image with a size, focal lengths to divide
/// by, a principal point that is a number, and a lens camera_geometry:v1
/// names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pinhole {
    width: u32,
    height: u32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    lens: Lens,
}

/// Why a stream's intrinsics make no pinhole model.
#[derive(Debug)]
pub enum PinholeError {
    /// A size of 0, a focal length that is not a positive number, or a
    /// principal point that is not a number.
    Unusable {
        width: u32,
        height: u32,
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
    },
    /// A distortion model camera_geometry:v1 does not name, or coefficients
    /// that do not fit the model.
    Lens {
        model: String,
        coefficients: Vec<f64>,
    },
}

impl fmt::Display for PinholeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unusable {
                width,
                height,
                fx,
                fy,
                cx,
                cy,
            } => write!(
                f,
                "the pinhole model {width}x{height} fx {fx} fy {fy} cx {cx} cy {cy} cannot be \
                 computed with: give a size above 0, focal lengths above 0 and a principal \
                 point of numbers"
            ),
            Self::Lens {
                model,
                coefficients,
            } => write!(
                f,
                "unknown distortion '{model}' with coefficients {coefficients:?} (expected \
                 'none' with none, or 'plumb_bob' or 'inverse_plumb_bob' with the five \
                 numbers k1, k2, p1, p2, k3)"
            ),
        }
    }
}

impl std::error::Error for PinholeError {}

/// What a depth sample of a stream measures, as camera_geometry:v1 names it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DepthModel {
    /// "z": the distance from the camera to the plane through the point,
    /// square to the optical axis.
    Z,
    /// "range": the straight-line distance from the camera to the point.
    Range,
}

impl DepthModel {
    /// The depth model `name` names.
    pub fn parse(name: &str) -> Result<Self, UnknownDepthModel> {
        match name {
            Z_DEPTH_MODEL => Ok(Self::Z),
            "range" => Ok(Self::Range),
            other => Err(UnknownDepthModel(other.to_owned())),
        }
    }

    /// The reading a depth sample of this model gives for `point` of the
    /// stream's optical frame, in the point's unit: the inverse of
    /// [`Pinhole::deproject`].
    pub fn reading(self, point: [f64; 3]) -> f64 {
        match self {
            Self::Z => point[2],
            Self::Range => point.iter().map(|c| c * c).sum::<f64>().sqrt(),
        }
    }
}

/// A depth model camera_geometry:v1 does not name.
#[derive(Debug, PartialEq)]
pub struct UnknownDepthModel(String);

impl fmt::Display for UnknownDepthModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "unknown depth_model '{}' (expected '{Z_DEPTH_MODEL}' or 'range')",
            self.0
        )
    }
}

impl std::error::Error for UnknownDepthModel {}

impl Pinhole {
    /// The pinhole model `intrinsics` describe.
    pub fn parse(intrinsics: &Intrinsics) -> Result<Self, PinholeError> {
        let &Intrinsics {
            width,
            height,
            fx,
            fy,
            cx,
            cy,
            distortion_model,
            distortion,
        } = intrinsics;
        if !pinhole_is_valid(width, height, fx, fy, cx, cy) {
            return Err(PinholeError::Unusable {
                width,
                height,
                fx,
                fy,
                cx,
                cy,
            });
        }
        let lens = Lens::parse(distortion_model, distortion).ok_or_else(|| PinholeError::Lens {
            model: distortion_model.to_owned(),
            coefficients: distortion.to_vec(),
        })?;
        Ok(Self {
            width,
            height,
            fx,
            fy,
            cx,
            cy,
            lens,
        })
    }

    /// The model of an ideal pinhole with square pixels whose image of
    /// `width` by `height` pixels spans `fovy` radians vertically, its
    /// optical axis through the middle of the image: a rendered camera's.
    pub fn from_vertical_fov(fovy: f64, width: u32, height: u32) -> Result<Self, PinholeError> {
        let focal = f64::from(height) / 2.0 / (fovy / 2.0).tan();
        Self::parse(&Intrinsics {
            width,
            height,
            fx: focal,
            fy: focal,
            cx: (f64::from(width) - 1.0) / 2.0,
            cy: (f64::from(height) - 1.0) / 2.0,
            distortion_model: "none",
            distortion: &[],
        })
    }

    /// The intrinsics of this model as camera_geometry:v1 carries them: the
    /// ones it was parsed from, its lens named as the contract names it.
    pub fn intrinsics(&self) -> Intrinsics<'_> {
        let (distortion_model, distortion): (&str, &[f64]) = match &self.lens {
            Lens::None => ("none", &[]),
            Lens::PlumbBob(k) => ("plumb_bob", k),
            Lens::InversePlumbBob(k) => ("inverse_plumb_bob", k),
        };
        Intrinsics {
            width: self.width,
            height: self.height,
            fx: self.fx,
            fy: self.fy,
            cx: self.cx,
            cy: self.cy,
            distortion_model,
            distortion,
        }
    }

    /// The image's width, in pixels.
    pub fn width(&self) -> u32 {
        self.width
    }

    /// The image's height, in pixels.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// The ideal image point (x, y) at unit depth that pixel (u, v) shows:
    /// the pixel through fx, fy, cx, cy, then the distortion taken off.
    pub fn ray(&self, u: f64, v: f64) -> (f64, f64) {
        let (xd, yd) = ((u - self.cx) / self.fx, (v - self.cy) / self.fy);
        match self.lens {
            Lens::None => (xd, yd),
            // The polynomial runs from the distorted point to the ideal one:
            // one application, nothing to invert.
            Lens::InversePlumbBob(k) => apply(&k, xd, yd),
            // It runs the other way, so the ideal point is found by
            // iterating it from the distorted one.
            Lens::PlumbBob(k) => invert(&k, xd, yd),
        }
    }

    /// The pixel that shows a point of this camera's optical frame: the
    /// pixel the lens takes the point to, when that pixel's ray passes within
    /// the pixel of the point. None for a point that is not in front of the
    /// camera, one with a coordinate that is not a finite number, and one beyond
    /// where the lens turns back.
    pub fn project(&self, point: [f64; 3]) -> Option<(f64, f64)> {
        let [px, py, pz] = point;
        if !point.iter().all(|c| c.is_finite()) || pz <= 0.0 {
            return None;
        }
        let (x, y) = (px / pz, py / pz);
        let (xd, yd) = match self.lens {
            Lens::None => (x, y),
            Lens::PlumbBob(k) => apply(&k, x, y),
            // Under the inverse model the polynomial undistorts, so the
            // distorted point is the iteration, from the ideal one.
            Lens::InversePlumbBob(k) => invert(&k, x, y),
        };
        let (u, v) = (self.fx * xd + self.cx, self.fy * yd + self.cy);
        let (xr, yr) = self.ray(u, v);
        let shown = (self.fx * (xr - x)).abs() <= SHOWN_WITHIN_PIXELS
            && (self.fy * (yr - y)).abs() <= SHOWN_WITHIN_PIXELS;
        shown.then_some((u, v))
    }

    /// The point of this camera's optical frame that depth pixel (u, v)
    /// shows, from its reading in meters, read as `model` says.
    pub fn deproject(&self, u: f64, v: f64, reading_m: f64, model: DepthModel) -> [f64; 3] {
        let (x, y) = self.ray(u, v);
        let z = match model {
            DepthModel::Z => reading_m,
            DepthModel::Range => reading_m / (x * x + y * y + 1.0).sqrt(),
        };
        [x * z, y * z, z]
    }
}

/// The Brown-Conrady polynomial applied once to (x, y).
fn apply(k: &[f64; 5], x: f64, y: f64) -> (f64, f64) {
    let (radial, dx, dy) = distortion_terms(k, x, y);
    (x * radial + dx, y * radial + dy)
}

/// The point the polynomial takes to (xd, yd), found by iterating it from
/// there: the fixed point of x = (xd - tangential) / radial.
fn invert(k: &[f64; 5], xd: f64, yd: f64) -> (f64, f64) {
    let (mut x, mut y) = (xd, yd);
    for _ in 0..UNDISTORT_ITERATIONS {
        let (radial, dx, dy) = distortion_terms(k, x, y);
        let next = ((xd - dx) / radial, (yd - dy) / radial);
        if next == (x, y) {
            break;
        }
        (x, y) = next;
    }
    (x, y)
}

/// The radial factor and the tangential shift the polynomial applies to the
/// point (x, y), with k1, k2, p1, p2, k3 as OpenCV orders them.
fn distortion_terms(k: &[f64; 5], x: f64, y: f64) -> (f64, f64, f64) {
    let [k1, k2, p1, p2, k3] = *k;
    let r2 = x * x + y * y;
    let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
    let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    (radial, dx, dy)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose::Pose;
    use crate::test_support::close;

    const IDEAL: Intrinsics<'static> = Intrinsics {
        width: 640,
        height: 360,
        fx: 369.0547,
        fy: 369.0547,
        cx: 319.5,
        cy: 179.5,
        distortion_model: "none",
        distortion: &[],
    };

    const LENS: [f64; 5] = [-0.055, 0.066, -0.0007, 0.0005, -0.021];

    fn ideal() -> Pinhole {
        Pinhole::parse(&IDEAL).expect("a valid model")
    }

    fn distorted() -> Pinhole {
        Pinhole::parse(&Intrinsics {
            distortion_model: "plumb_bob",
            distortion: &LENS,
            ..IDEAL
        })
        .expect("a valid model")
    }

    fn inverse() -> Pinhole {
        Pinhole::parse(&Intrinsics {
            distortion_model: "inverse_plumb_bob",
            distortion: &LENS,
            ..IDEAL
        })
        .expect("a valid model")
    }

    #[test]
    fn the_principal_point_looks_straight_ahead() {
        let point = ideal().deproject(319.5, 179.5, 1.25, DepthModel::Z);
        assert_eq!(point, [0.0, 0.0, 1.25]);
    }

    #[test]
    fn a_pixel_to_the_right_and_below_is_plus_x_and_plus_y() {
        // One focal length to the right of the principal point is 45 degrees.
        let camera = ideal();
        let point = camera.deproject(319.5 + 369.0547, 179.5, 2.0, DepthModel::Z);
        assert!(close(point, [2.0, 0.0, 2.0], 1e-12));
        let point = camera.deproject(319.5, 179.5 + 369.0547, 2.0, DepthModel::Z);
        assert!(close(point, [0.0, 2.0, 2.0], 1e-12));
    }

    #[test]
    fn a_range_reading_is_the_straight_line_to_the_point() {
        let camera = ideal();
        let along_the_axis = camera.deproject(319.5 + 369.0547, 179.5, 2.0, DepthModel::Z);
        let straight_line = (along_the_axis.iter().map(|c| c * c).sum::<f64>()).sqrt();
        let point = camera.deproject(319.5 + 369.0547, 179.5, straight_line, DepthModel::Range);
        assert!(close(point, along_the_axis, 1e-12));
    }

    #[test]
    fn deprojecting_what_was_projected_returns_the_point() {
        for camera in [ideal(), distorted(), inverse()] {
            for point in [[0.3, -0.2, 1.5], [-0.6, 0.35, 0.9], [0.0, 0.0, 3.0]] {
                let (u, v) = camera.project(point).expect("in front");
                let back = camera.deproject(u, v, point[2], DepthModel::Z);
                assert!(close(back, point, 1e-9), "{camera:?} {point:?} {back:?}");
            }
        }
    }

    #[test]
    fn a_point_beyond_where_the_lens_turns_back_shows_in_no_pixel() {
        // About 65 degrees off the axis the radial factor of LENS falls to
        // near 0, so the polynomial takes the point to a pixel by the center
        // of the image, whose ray looks along the axis.
        let beyond = [2.17, 0.0, 1.0];
        let (xd, yd) = apply(&LENS, beyond[0], beyond[1]);
        let (u, v) = (IDEAL.fx * xd + IDEAL.cx, IDEAL.fy * yd + IDEAL.cy);
        assert!(u > 0.0 && u < 640.0 && v > 0.0 && v < 360.0, "({u}, {v})");
        assert_eq!(distorted().project(beyond), None);
        assert_eq!(inverse().project(beyond), None);
        for camera in [ideal(), distorted(), inverse()] {
            for point in [
                [f64::NAN, 0.0, 1.0],
                [0.0, f64::INFINITY, 1.0],
                [0.0, 0.0, f64::NAN],
                [0.0, 0.0, f64::INFINITY],
                [1.0, 1.0, f64::INFINITY],
            ] {
                assert_eq!(camera.project(point), None, "{camera:?} {point:?}");
            }
        }
    }

    #[test]
    fn a_point_just_inside_the_fold_of_a_strong_lens_shows_where_the_lens_takes_it() {
        // k1 = -0.2 turns back at r = 1 / sqrt(0.6), 1.2910, which this image of
        // 64 by 48 pixels over 60 degrees reaches on its diagonals. A point at r
        // 1.2885, 30 degrees below the horizontal, converges slowly so near the
        // fold, and shows at the pixel the lens takes it to.
        let focal = 24.0 / 30f64.to_radians().tan();
        let strong = Pinhole::parse(&Intrinsics {
            width: 64,
            height: 48,
            fx: focal,
            fy: focal,
            cx: 31.5,
            cy: 23.5,
            distortion_model: "plumb_bob",
            distortion: &[-0.2, 0.0, 0.0, 0.0, 0.0],
        })
        .expect("a valid model");
        let (r, angle) = (1.2885, 30f64.to_radians());
        let point = [r * angle.cos(), r * angle.sin(), 1.0];
        let radial = 1.0 - 0.2 * r * r;
        let lens_takes_it_to = (
            focal * point[0] * radial + 31.5,
            focal * point[1] * radial + 23.5,
        );
        let (u, v) = strong.project(point).expect("shown inside the fold");
        assert!((u - lens_takes_it_to.0).abs() < 1e-9 && (v - lens_takes_it_to.1).abs() < 1e-9);
        assert!(u < 63.5 && v < 47.5, "({u}, {v}) lies in the image");
    }

    #[test]
    fn a_depth_reading_deprojects_to_the_point_it_reads() {
        for camera in [ideal(), distorted(), inverse()] {
            for model in [DepthModel::Z, DepthModel::Range] {
                let point = [-0.4, 0.25, 1.3];
                let (u, v) = camera.project(point).expect("in the image");
                let back = camera.deproject(u, v, model.reading(point), model);
                assert!(close(back, point, 1e-9), "{model:?} {back:?}");
            }
        }
    }

    #[test]
    fn a_model_gives_back_the_intrinsics_it_was_parsed_from() {
        for (distortion_model, distortion) in [
            ("none", &[][..]),
            ("plumb_bob", &LENS[..]),
            ("inverse_plumb_bob", &LENS[..]),
        ] {
            let model = Pinhole::parse(&Intrinsics {
                distortion_model,
                distortion,
                ..IDEAL
            })
            .expect("a valid model");
            let Intrinsics {
                width,
                height,
                fx,
                fy,
                cx,
                cy,
                distortion_model: model_name,
                distortion: coefficients,
            } = model.intrinsics();
            assert_eq!(
                (width, height, fx, fy, cx, cy),
                (
                    IDEAL.width,
                    IDEAL.height,
                    IDEAL.fx,
                    IDEAL.fy,
                    IDEAL.cx,
                    IDEAL.cy
                )
            );
            assert_eq!((model_name, coefficients), (distortion_model, distortion));
            assert_eq!(Pinhole::parse(&model.intrinsics()).ok(), Some(model));
        }
    }

    #[test]
    fn a_rendered_cameras_model_has_square_pixels_and_its_axis_through_the_middle() {
        let model = Pinhole::from_vertical_fov(60f64.to_radians(), 64, 48).expect("a model");
        assert_eq!((model.width(), model.height()), (64, 48));
        assert_eq!(model.ray(31.5, 23.5), (0.0, 0.0));
        // The top edge of the image, half a pixel above the first row, is
        // 30 degrees above the axis, and a pixel is as wide as it is tall.
        let half = 30f64.to_radians().tan();
        let (x, y) = model.ray(-0.5, -0.5);
        assert!((y + half).abs() < 1e-12, "{y}");
        assert!((x + half * 64.0 / 48.0).abs() < 1e-12, "{x}");
        for fovy in [0.0, -1.0, f64::NAN] {
            assert!(matches!(
                Pinhole::from_vertical_fov(fovy, 64, 48),
                Err(PinholeError::Unusable { .. })
            ));
        }
    }

    #[test]
    fn distortion_moves_a_corner_pixel_and_leaves_the_center() {
        let (ideal, distorted) = (ideal(), distorted());
        let corner = [0.75, 0.42, 1.0];
        let (u0, v0) = ideal.project(corner).expect("in front");
        let (u1, v1) = distorted.project(corner).expect("in front");
        assert!((u0 - u1).abs() > 1.0 || (v0 - v1).abs() > 1.0);
        let center = ideal.project([0.0, 0.0, 1.0]).expect("in front");
        let (u, v) = distorted.project([0.0, 0.0, 1.0]).expect("in front");
        assert!((center.0 - u).abs() < 1e-12 && (center.1 - v).abs() < 1e-12);
    }

    #[test]
    fn the_inverse_model_applies_the_polynomial_once_from_the_distorted_point() {
        // inverse_plumb_bob is one application to the distorted point, so
        // the ray is the closed form, and the same numbers under plumb_bob
        // give a different ray, because that model has to be inverted.
        let (u, v) = (IDEAL.cx + 0.3 * IDEAL.fx, IDEAL.cy - 0.2 * IDEAL.fy);
        let expected = apply(&LENS, 0.3, -0.2);
        let (x, y) = inverse().ray(u, v);
        assert!((x - expected.0).abs() < 1e-12 && (y - expected.1).abs() < 1e-12);
        let (xf, yf) = distorted().ray(u, v);
        assert!((x - xf).abs() > 1e-4 || (y - yf).abs() > 1e-4);
    }

    #[test]
    fn the_inverse_and_forward_models_agree_for_a_small_lens() {
        // To first order the inverse of the polynomial with coefficients c
        // is the polynomial with -c, so for a small lens the direct inverse
        // model with -c and the iterated forward model with c agree on a
        // ray, and a lens ten times smaller agrees a hundred times better.
        let lens = |scale: f64| [-0.5, 0.6, -0.07, 0.05, -0.2].map(|c| c * scale);
        let model = |distortion_model, distortion: &[f64]| {
            Pinhole::parse(&Intrinsics {
                distortion_model,
                distortion,
                ..IDEAL
            })
            .expect("a valid model")
        };
        let mut gaps = Vec::new();
        for scale in [0.01, 0.001] {
            let forward = model("plumb_bob", &lens(scale));
            let direct = model("inverse_plumb_bob", &lens(-scale));
            let (u, v) = (IDEAL.cx + 0.8 * IDEAL.fx, IDEAL.cy + 0.45 * IDEAL.fy);
            let (xf, yf) = forward.ray(u, v);
            let (xd, yd) = direct.ray(u, v);
            let gap = ((xf - xd).powi(2) + (yf - yd).powi(2)).sqrt();
            assert!(gap < 20.0 * scale * scale, "scale {scale}: gap {gap}");
            gaps.push(gap);
        }
        assert!(gaps[0] / gaps[1] > 50.0, "{gaps:?}");
    }

    #[test]
    fn intrinsics_the_contract_does_not_describe_are_refused() {
        let refused = |intrinsics: Intrinsics| Pinhole::parse(&intrinsics).unwrap_err();
        let fisheye = [0.1, 0.0, 0.0, 0.0];
        assert_eq!(
            refused(Intrinsics {
                distortion_model: "kannala_brandt",
                distortion: &fisheye,
                ..IDEAL
            })
            .to_string(),
            "unknown distortion 'kannala_brandt' with coefficients [0.1, 0.0, 0.0, 0.0] \
             (expected 'none' with none, or 'plumb_bob' or 'inverse_plumb_bob' with the five \
             numbers k1, k2, p1, p2, k3)"
        );
        for distortion in [&[0.1, 0.0][..], &[0.1, 0.0, f64::NAN, 0.0, 0.0]] {
            assert!(matches!(
                refused(Intrinsics {
                    distortion_model: "plumb_bob",
                    distortion,
                    ..IDEAL
                }),
                PinholeError::Lens { .. }
            ));
        }
        assert!(matches!(
            refused(Intrinsics {
                distortion_model: "none",
                distortion: &LENS,
                ..IDEAL
            }),
            PinholeError::Lens { .. }
        ));
        for unusable in [
            Intrinsics { width: 0, ..IDEAL },
            Intrinsics { fx: 0.0, ..IDEAL },
            Intrinsics {
                fy: f64::NAN,
                ..IDEAL
            },
            Intrinsics {
                cx: f64::INFINITY,
                ..IDEAL
            },
        ] {
            assert!(matches!(refused(unusable), PinholeError::Unusable { .. }));
        }
        assert_eq!(ideal().project([0.0, 0.0, -1.0]), None);
        assert_eq!(ideal().project([0.0, 0.0, 0.0]), None);
    }

    #[test]
    fn a_depth_model_is_one_the_contract_names() {
        assert!(matches!(DepthModel::parse("z"), Ok(DepthModel::Z)));
        assert!(matches!(DepthModel::parse("range"), Ok(DepthModel::Range)));
        assert_eq!(
            DepthModel::parse("disparity").unwrap_err().to_string(),
            "unknown depth_model 'disparity' (expected 'z' or 'range')"
        );
    }

    // A depth pixel's point, carried into the color frame by the
    // depth-to-color pose, shows in the color image at the pixel that sees
    // the same point. The depth imager sits beside the color one, turned a
    // little about the vertical, so the pose taken the wrong way puts the
    // point pixels away.
    #[test]
    fn a_depth_pixel_lands_where_the_color_camera_sees_it() {
        let depth = ideal();
        let color_intrinsics = Intrinsics {
            width: 1280,
            height: 720,
            fx: 738.1,
            fy: 738.1,
            cx: 639.5,
            cy: 359.5,
            ..IDEAL
        };
        let color = Pinhole::parse(&color_intrinsics).expect("a valid model");
        let (angle, offset) = (0.02_f64, [-0.063, 0.001, 0.002]);
        let depth_to_color =
            Pose::new(offset, [0.0, (angle / 2.0).sin(), 0.0, (angle / 2.0).cos()])
                .expect("a unit quaternion");
        // A point the color camera sees, and the same point in the depth
        // frame, turned back about +Y by hand.
        let seen = [0.21, -0.13, 1.4];
        let d = [0, 1, 2].map(|i| seen[i] - offset[i]);
        let (cos, sin) = (angle.cos(), angle.sin());
        let in_depth = [cos * d[0] - sin * d[2], d[1], sin * d[0] + cos * d[2]];
        let pixel = |intrinsics: &Intrinsics, p: [f64; 3]| {
            (
                intrinsics.fx * p[0] / p[2] + intrinsics.cx,
                intrinsics.fy * p[1] / p[2] + intrinsics.cy,
            )
        };
        let (u, v) = pixel(&IDEAL, in_depth);
        let point = depth.deproject(u, v, in_depth[2], DepthModel::Z);
        let shows = |pose: &Pose| color.project(pose.apply(point)).expect("in front");
        let want = pixel(&color_intrinsics, seen);
        let got = shows(&depth_to_color);
        assert!(
            (got.0 - want.0).abs() < 1e-9 && (got.1 - want.1).abs() < 1e-9,
            "{got:?} against {want:?}"
        );
        let reversed = shows(&depth_to_color.inverse());
        assert!((reversed.0 - want.0).abs() > 10.0, "{reversed:?}");
    }
}
