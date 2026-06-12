//! The checked-in capsule config must conservatively contain the meshes it
//! was fitted from. Containment of every mesh vertex implies containment of
//! the convex hull of each triangle (capsules are convex), so the capsule can
//! never report a larger distance than the true mesh allows.
//!
//! This also pins the fixture config to the fixture assets: regenerate with
//! the documented fit_capsules invocation after changing the URDF, the
//! meshes, or the fit, or this test fails.

use collision_model::config::CollisionConfig;
use collision_model::geometry::Capsule;
use collision_model::nalgebra::Point3;
use collision_model::urdf_collision::UrdfCollisions;

const ASSETS: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures");

/// Rounding the stored radius up to 1e-6 absorbs float noise; allow only that.
const TOL: f64 = 1e-9;

fn assets() -> (UrdfCollisions, collision_model::config::LoadedConfig, String) {
    let urdf = UrdfCollisions::from_file(&format!("{ASSETS}/openarm_v10.urdf")).expect("vendored urdf");
    let config = CollisionConfig::from_file(&format!("{ASSETS}/openarm_v10_capsules.json"))
        .expect("checked-in config")
        .parse()
        .expect("valid config");
    (urdf, config, format!("{ASSETS}/meshes"))
}

fn assert_contained(vertices: &[Point3<f64>], capsules: &[Capsule], what: &str) {
    // Vertices exactly, faces by sampling: a capsule UNION is not convex, so
    // vertex containment alone does not imply face containment across bands.
    let union_escape = |p: &Point3<f64>| {
        capsules
            .iter()
            .map(|c| collision_model::point_segment_distance(p, &c.a, &c.b) - c.radius)
            .fold(f64::INFINITY, f64::min)
    };
    let mut worst = f64::NEG_INFINITY;
    for tri in vertices.chunks_exact(3) {
        for w in [
            [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
            [0.5, 0.5, 0.0], [0.5, 0.0, 0.5], [0.0, 0.5, 0.5],
            [0.25, 0.75, 0.0], [0.75, 0.0, 0.25], [0.0, 0.25, 0.75],
            [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0],
        ] {
            let p = Point3::from(tri[0].coords * w[0] + tri[1].coords * w[1] + tri[2].coords * w[2]);
            worst = worst.max(union_escape(&p));
        }
    }
    assert!(
        worst <= TOL,
        "{what}: a mesh face point sticks {worst:.2e} m out of the capsule union; \
         regenerate with `cargo run --bin fit_capsules`",
    );
}

#[test]
fn fixed_bodies_contain_their_meshes() {
    let (urdf, config, meshes) = assets();
    assert_eq!(config.fixed.len(), 3, "body and two mount links");
    for (name, capsules) in &config.fixed {
        let vertices = urdf.fixed_vertices_in_root(name, &meshes).expect("fixed vertices");
        assert!(!vertices.is_empty());
        assert_contained(&vertices, capsules, name);
    }
}

#[test]
fn moving_links_contain_their_meshes() {
    let (urdf, config, meshes) = assets();
    for side in ["left", "right"] {
        for i in 1..=7 {
            let name = format!("openarm_{side}_link{i}");
            let capsules = config.links.get(&name).unwrap_or_else(|| panic!("config missing {name}"));
            let vertices = urdf.link_vertices(&name, &meshes).expect("link vertices");
            assert_contained(&vertices, capsules, &name);
        }
    }
}

#[test]
fn wrist_capsules_contain_fingers_across_full_travel() {
    let (urdf, config, meshes) = assets();
    for side in ["left", "right"] {
        let wrist = format!("openarm_{side}_link7");
        let capsules = &config.links[&wrist];
        assert!(capsules.len() >= 3, "{wrist}: wrist band(s) + two fingers, got {}", capsules.len());
        for finger in ["left_finger", "right_finger"] {
            let name = format!("openarm_{side}_{finger}");
            let upper = urdf.parent_joint(&name).expect("finger joint").upper_limit;
            // Travel is linear, so containment at the extremes and the middle
            // spot-checks the whole range (extremes alone prove it; the
            // middle guards against transform mistakes).
            for q in [0.0, upper / 2.0, upper] {
                let vertices = urdf.child_vertices_in_parent(&name, q, &meshes).expect("finger vertices");
                assert_contained(&vertices, capsules, &format!("{name}@{q:.3}"));
            }
        }
    }
}

#[test]
fn config_covers_exactly_the_expected_bodies() {
    let (_, config, _) = assets();
    assert_eq!(config.links.len(), 14, "7 links per side");
    let fixed: Vec<&str> = config.fixed.iter().map(|(n, _)| n.as_str()).collect();
    assert_eq!(fixed, ["openarm_body_link0", "openarm_left_link0", "openarm_right_link0"]);
}
