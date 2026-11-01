//! A depth frame as the points of its valid samples, in the depth stream's
//! optical frame, from its intrinsics.

use camera_core::encoding::{DepthFrame, DepthFrameSizeError};
use camera_core::pinhole::{DepthModel, Pinhole};

/// Why a depth frame gives no points.
#[derive(Debug, thiserror::Error, PartialEq)]
pub enum DepthError {
    #[error(transparent)]
    Size(#[from] DepthFrameSizeError),
    #[error("depth_unit must be a positive number of meters, got {0}")]
    Unit(f32),
}

/// The points that the samples of `frame`, a z16 depth frame of `camera`'s
/// size, read, in the depth stream's optical frame and in single precision
/// as a point cloud carries them: each sample times `depth_unit` meters, read
/// as `depth_model` says. A sample of 0 reads nothing and makes no point.
pub fn depth_points(
    frame: &[u8],
    camera: &Pinhole,
    depth_unit: f32,
    depth_model: DepthModel,
) -> Result<Vec<[f32; 3]>, DepthError> {
    let frame = DepthFrame::parse(frame, camera.width(), camera.height())?;
    if !camera_core::geometry::depth_unit_is_valid(depth_unit) {
        return Err(DepthError::Unit(depth_unit));
    }
    Ok(frame
        .samples()
        .filter(|&(_, _, sample)| sample != 0)
        .map(|(u, v, sample)| {
            let reading = f64::from(sample) * f64::from(depth_unit);
            camera
                .deproject(f64::from(u), f64::from(v), reading, depth_model)
                .map(|c| c as f32)
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use camera_core::pinhole::Intrinsics;

    fn camera() -> Pinhole {
        Pinhole::parse(&Intrinsics {
            width: 4,
            height: 3,
            fx: 2.0,
            fy: 2.0,
            cx: 1.5,
            cy: 1.0,
            distortion_model: "none",
            distortion: &[],
        })
        .expect("a valid model")
    }

    // A sample of 0 makes no point; the rest read their pixel's ray at their
    // depth.
    #[test]
    fn every_sample_but_the_empty_one_makes_a_point() {
        // Half a meter per count, exact in single precision: two counts read
        // one meter.
        let mut samples = [2_u16; 12];
        samples[5] = 0;
        let frame: Vec<u8> = samples.iter().flat_map(|s| s.to_le_bytes()).collect();
        let points = depth_points(&frame, &camera(), 0.5, DepthModel::Z).expect("a valid frame");
        assert_eq!(points.len(), 11);
        assert_eq!(points[0], [-0.75, -0.5, 1.0]);
        assert!(depth_points(&frame[..4], &camera(), 0.001, DepthModel::Z).is_err());
        assert_eq!(
            depth_points(&frame, &camera(), 0.0, DepthModel::Z).unwrap_err(),
            DepthError::Unit(0.0)
        );
    }
}
