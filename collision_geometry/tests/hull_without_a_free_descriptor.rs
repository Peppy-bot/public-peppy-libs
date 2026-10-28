//! Qhull builds a hull, hulls joggled a cloud whose first pass fails on rounding, and
//! reports a cloud it cannot hull, in a process that holds every file descriptor its
//! limit allows, where no temporary file can be created. A
//! test binary of its own: it lowers the process's limit of open files to
//! [`OPEN_FILES`], and takes every descriptor of the process while it runs.

use std::fs::File;

use collision_geometry::hull::exact_hull;
use collision_geometry::nalgebra::Point3;

#[path = "fixtures/thin_plate.rs"]
mod thin_plate;

/// The soft limit of open files the test sets before it takes them all.
const OPEN_FILES: libc::rlim_t = 64;

/// Lowers the soft limit of open files of this process to [`OPEN_FILES`], or to the
/// hard limit when that is lower.
fn lower_the_open_file_limit() {
    let mut limit = libc::rlimit {
        rlim_cur: 0,
        rlim_max: 0,
    };
    // SAFETY: `limit` is a valid rlimit for getrlimit to write.
    let read = unsafe { libc::getrlimit(libc::RLIMIT_NOFILE, &mut limit) };
    assert_eq!(read, 0, "{}", std::io::Error::last_os_error());
    limit.rlim_cur = OPEN_FILES.min(limit.rlim_max);
    // SAFETY: `limit` is a valid rlimit, with a soft limit no higher than its hard one.
    let set = unsafe { libc::setrlimit(libc::RLIMIT_NOFILE, &limit) };
    assert_eq!(set, 0, "{}", std::io::Error::last_os_error());
}

#[test]
fn a_hull_builds_with_no_file_descriptor_free() {
    lower_the_open_file_limit();
    let mut held = Vec::new();
    let full = loop {
        match File::open("/dev/null") {
            Ok(file) => held.push(file),
            Err(error) => break error,
        }
    };
    let corners: Vec<Point3<f64>> = (0..8)
        .map(|i| Point3::new((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64))
        .collect();
    let flat: Vec<Point3<f64>> = (0..9)
        .map(|i| Point3::new((i % 3) as f64, (i / 3) as f64, 0.0))
        .collect();
    let thin_plate: Vec<Point3<f64>> = thin_plate::THIN_PLATE_DUAL_POINTS
        .iter()
        .map(|&[x, y, z]| Point3::new(x, y, z))
        .collect();
    let (cube, joggled, plane) = (
        exact_hull(&corners),
        exact_hull(&thin_plate),
        exact_hull(&flat),
    );
    drop(held);
    assert_eq!(full.raw_os_error(), Some(libc::EMFILE), "{full}");
    assert_eq!(cube.expect("a cube hulls").vertices.len(), 8);
    joggled.expect("the thin plate hulls joggled");
    let error = plane.expect_err("a flat cloud has no hull");
    assert!(error.contains("QH6154"), "{error}");
}
