//! Parity with pointcloud_to_laserscan 2.1.1. For each laser projection scenario,
//! `LaserProjection` gives the angles and the range limits that the reference wrote into
//! every scan, and for the points of each cloud, the ranges of the reference's scan, bit for
//! bit, a bin with no point holding `lidar_2d`'s no-return mark where the reference holds
//! +inf, as use_inf writes it. The callback computes the range and the bearing of a point
//! with glibc's single precision `hypotf` and `atan2f`, and the projection computes each in
//! double precision and rounds it; the committed outputs hold the amd64 image's bits
//! (README rule 4).

#[cfg(target_arch = "x86_64")]
use camera_core::laser_scan::NO_RETURN_MARK_M;
#[cfg(target_arch = "x86_64")]
use mapping_core::{LaserProjection, LaserProjectionParameters};
#[cfg(target_arch = "x86_64")]
use ros_reference::laser_projection::{Cases, Cloud, LaserScanParameters};

#[cfg(target_arch = "x86_64")]
fn projection(name: &str, parameters: &LaserScanParameters) -> LaserProjection {
    assert!(
        parameters.use_inf,
        "{name}: the reference writes +inf in a bin with no point, as use_inf does"
    );
    LaserProjection::new(LaserProjectionParameters {
        min_height: parameters.min_height,
        max_height: parameters.max_height,
        angle_min: parameters.angle_min,
        angle_max: parameters.angle_max,
        angle_increment: parameters.angle_increment,
        range_min: parameters.range_min,
        range_max: parameters.range_max,
    })
    .unwrap_or_else(|e| panic!("{name}: {e}"))
}

/// Whether our range of a bin is the reference's: the same bits, or our no-return mark
/// where the reference wrote +inf.
#[cfg(target_arch = "x86_64")]
fn agrees(ours: f32, reference: f32) -> bool {
    if ours == NO_RETURN_MARK_M {
        reference == f32::INFINITY
    } else {
        ours.to_bits() == reference.to_bits()
    }
}

/// The indices of the points of `cloud` that `projection` puts in `bin`, each projected
/// alone.
#[cfg(target_arch = "x86_64")]
fn points_in_bin(projection: &LaserProjection, cloud: &Cloud, bin: usize) -> Vec<usize> {
    (0..cloud.points.len())
        .filter(|&index| {
            projection.project(&cloud.points[index..=index]).ranges()[bin] != NO_RETURN_MARK_M
        })
        .collect()
}

/// The indices of the points of `cloud` whose range on the floor plane, in double
/// precision and rounded to single precision, is `range`.
#[cfg(target_arch = "x86_64")]
fn points_at_range(cloud: &Cloud, range: f32) -> Vec<usize> {
    (0..cloud.points.len())
        .filter(|&index| {
            let [x, y, _] = cloud.points[index].map(f64::from);
            (x * x + y * y).sqrt() as f32 == range
        })
        .collect()
}

/// The first difference between our scan of `cloud` and the reference's, named with the
/// points that give it, or none.
#[cfg(target_arch = "x86_64")]
fn first_difference(projection: &LaserProjection, cloud: &Cloud) -> Option<String> {
    let ours = projection.project(&cloud.points).ranges().to_vec();
    if ours.len() != cloud.ranges.len() {
        return Some(format!(
            "the scan holds {} ranges here and {} in the reference",
            ours.len(),
            cloud.ranges.len()
        ));
    }
    let bin = (0..ours.len()).find(|&bin| !agrees(ours[bin], cloud.ranges[bin]))?;
    let named = |indices: Vec<usize>| -> Vec<String> {
        indices
            .into_iter()
            .map(|index| format!("point {index} {:?}", cloud.points[index]))
            .collect()
    };
    Some(format!(
        "bin {bin} holds {} here, from {:?}, and {} in the reference, from {:?}",
        ours[bin],
        named(points_in_bin(projection, cloud, bin)),
        cloud.ranges[bin],
        named(points_at_range(cloud, cloud.ranges[bin])),
    ))
}

#[cfg(target_arch = "x86_64")]
#[test]
fn every_scan_equals_pointcloud_to_laserscans_bit_for_bit() {
    let all = Cases::load_all();
    assert!(
        !all.is_empty(),
        "the harness holds a laser projection scenario"
    );
    for cases in all {
        let projection = projection(&cases.name, &cases.laser_scan);
        // A scan carries its first angle, its increment and its range limits, the same for
        // every cloud, here those of the scan of no point, and the node writes the parameter
        // angle_max, in single precision.
        let scan = projection.project(&[]);
        let header = [
            ("angle_min", scan.min_angle_rad(), cases.scan.angle_min),
            (
                "angle_max",
                cases.laser_scan.angle_max as f32,
                cases.scan.angle_max,
            ),
            (
                "angle_increment",
                scan.angle_increment_rad(),
                cases.scan.angle_increment,
            ),
            ("range_min", scan.min_range_m(), cases.scan.range_min),
            ("range_max", scan.max_range_m(), cases.scan.range_max),
        ];
        for (name, ours, theirs) in header {
            assert_eq!(
                ours.to_bits(),
                theirs.to_bits(),
                "{}: {name} is {ours} here and {theirs} in the reference",
                cases.name
            );
        }
        for (index, cloud) in cases.clouds.iter().enumerate() {
            if let Some(difference) = first_difference(&projection, cloud) {
                panic!("{} cloud {index}: {difference}", cases.name);
            }
        }
    }
}
