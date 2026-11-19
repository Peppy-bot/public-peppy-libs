//! Properties of the projection of points to a scan, on drawn fans and drawn clouds.

use camera_core::laser_scan::NO_RETURN_MARK_M;
use hegel::TestCase;
use hegel::generators as gs;
use mapping_core::{LaserProjection, LaserProjectionParameters};

mod common;
use common::properties::{cloud, projection_parameters};

fn projection(tc: &TestCase) -> (LaserProjectionParameters, LaserProjection) {
    let parameters = projection_parameters(tc);
    let projection = LaserProjection::new(parameters);
    tc.assume(projection.is_ok());
    (parameters, projection.expect("assumed valid"))
}

#[hegel::test]
fn a_cloud_holds_the_nearest_range_of_each_bin_whatever_its_order(tc: TestCase) {
    let (_, projection) = projection(&tc);
    let points = cloud(&tc, 60);
    let scan = projection.project(&points);
    let nearest = points
        .iter()
        .map(|point| {
            projection
                .project(std::slice::from_ref(point))
                .ranges()
                .to_vec()
        })
        .fold(
            vec![NO_RETURN_MARK_M; projection.bins()],
            |nearest, ranges| {
                nearest
                    .iter()
                    .zip(&ranges)
                    .map(|(a, b)| a.min(*b))
                    .collect()
            },
        );
    assert_eq!(scan.ranges(), nearest);
    let reordered: Vec<[f32; 3]> = tc.draw(gs::permutations(points));
    assert_eq!(projection.project(&reordered), scan);
}

#[hegel::test]
fn a_point_inside_the_band_the_limits_and_the_fan_lands_in_the_bin_of_its_bearing(tc: TestCase) {
    let (parameters, projection) = projection(&tc);
    let bin: usize = tc.draw(gs::integers().min_value(0).max_value(projection.bins() - 1));
    // A bearing a tenth of an increment or more from the bin's ends, and a range and a
    // height a millimeter or more inside the limits and the band, so the rounding of the
    // point to single precision moves it to no other bin and out of no limit.
    let within: f64 = tc.draw(gs::floats().min_value(0.1).max_value(0.9));
    let bearing = parameters.angle_min + (bin as f64 + within) * parameters.angle_increment;
    tc.assume(bearing <= parameters.angle_max - 0.1 * parameters.angle_increment);
    let range: f64 = tc.draw(
        gs::floats()
            .min_value(parameters.range_min + 1e-3)
            .max_value(parameters.range_max - 1e-3),
    );
    let height: f64 = tc.draw(
        gs::floats()
            .min_value(parameters.min_height + 1e-3)
            .max_value(parameters.max_height - 1e-3),
    );
    let point = [
        (range * bearing.cos()) as f32,
        (range * bearing.sin()) as f32,
        height as f32,
    ];
    let scan = projection.project(&[point]).ranges().to_vec();
    let [x, y, _] = point.map(f64::from);
    let expected = x.hypot(y) as f32;
    assert!(
        (scan[bin] - expected).abs() <= expected * f32::EPSILON,
        "the bin of bearing {bearing} holds {}, and the point lies {expected} away",
        scan[bin]
    );
    assert_eq!(
        scan.iter()
            .filter(|&&range| range != NO_RETURN_MARK_M)
            .count(),
        1
    );
}
