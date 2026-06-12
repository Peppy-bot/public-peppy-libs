//! Regenerate the OpenArm capsule config from the vendored URDF and STLs.
//!
//! Offline tool; the output JSON is checked in next to the assets and loaded
//! by the runtime. Run from the crate root after changing the URDF, the
//! meshes, or the fit:
//!
//! ```sh
//! cargo run --bin fit_capsules
//! ```

use collision_model::config::{BodyCapsules, CapsuleSpec, CollisionConfig};
use collision_model::fit::fit_capsules_adaptive;
use collision_model::geometry::Capsule;
use collision_model::nalgebra::Point3;
use collision_model::urdf_collision::UrdfCollisions;

const URDF_BASENAME: &str = "openarm_v10.urdf";
const OUTPUT_BASENAME: &str = "openarm_v10_capsules.json";
const SIDES: [&str; 2] = ["left", "right"];
/// Adaptive band search ceiling per body: the fixed torso is a compound
/// shape (wide base, slim column, shoulder yoke) worth many bands; limb
/// links taper at most a little, and extra capsules cost pairwise checks.
const MAX_BANDS_FIXED: usize = 8;
const MAX_BANDS_LINK: usize = 3;
/// Links that never move: the torso and each arm's mount link.
const FIXED_LINKS: [&str; 3] = ["openarm_body_link0", "openarm_left_link0", "openarm_right_link0"];
/// Endpoint/radius grid for the emitted JSON, 1 micrometer.
const ROUND: f64 = 1e-6;

fn main() {
    if let Err(e) = run() {
        eprintln!("fit_capsules: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let assets = format!("{}/assets", env!("CARGO_MANIFEST_DIR"));
    let meshes = format!("{assets}/meshes");
    let urdf = UrdfCollisions::from_file(&format!("{assets}/{URDF_BASENAME}"))?;

    let fixed = FIXED_LINKS
        .iter()
        .map(|link| {
            let vertices = urdf.fixed_vertices_in_root(link, &meshes)?;
            Ok(body(link, rounded_fit(&vertices, MAX_BANDS_FIXED)?))
        })
        .collect::<Result<Vec<_>, String>>()?;

    let mut links = Vec::new();
    for side in SIDES {
        for i in 1..=7 {
            let name = format!("openarm_{side}_link{i}");
            let mut capsules = rounded_fit(&urdf.link_vertices(&name, &meshes)?, MAX_BANDS_LINK)?;
            if i == 7 {
                capsules.extend(finger_capsules(&urdf, side, &name, &meshes)?);
            }
            links.push(body(&name, capsules));
        }
    }

    // Refitting changes capsules, not the pair classification; carry the
    // existing pairs (and their fingerprint) through. A changed fit makes
    // the fingerprint stale, so loading fails until classify_pairs reruns.
    // A present-but-corrupt config is an error, not a silent pairs wipe.
    let out = format!("{assets}/{OUTPUT_BASENAME}");
    let (pairs, pairs_fingerprint) = match CollisionConfig::from_file(&out) {
        Ok(c) => (c.pairs, c.pairs_fingerprint),
        Err(_) if !std::fs::exists(&out).unwrap_or(false) => (Vec::new(), None),
        Err(e) => return Err(format!("existing config is unreadable, refusing to discard its pairs: {e}")),
    };
    let config = CollisionConfig { source_urdf: URDF_BASENAME.into(), fixed, links, pairs, pairs_fingerprint };
    std::fs::write(&out, config.to_json_pretty() + "\n").map_err(|e| format!("write {out}: {e}"))?;
    println!("wrote {out}");
    Ok(())
}

/// One capsule per finger, fitted over the union of the finger's vertices at
/// both travel extremes and expressed in the wrist link's frame. Finger
/// travel is a pure translation, so containing both extremes contains every
/// intermediate opening; the runtime then needs no gripper state.
fn finger_capsules(
    urdf: &UrdfCollisions,
    side: &str,
    wrist: &str,
    meshes: &str,
) -> Result<Vec<Capsule>, String> {
    ["left_finger", "right_finger"]
        .iter()
        .map(|finger| {
            let name = format!("openarm_{side}_{finger}");
            let joint = urdf
                .parent_joint(&name)
                .ok_or_else(|| format!("finger '{name}' has no parent joint"))?;
            if joint.parent_link != wrist {
                return Err(format!("finger '{name}' hangs off '{}', expected '{wrist}'", joint.parent_link));
            }
            let mut vertices = urdf.child_vertices_in_parent(&name, joint.lower_limit, meshes)?;
            vertices.extend(urdf.child_vertices_in_parent(&name, joint.upper_limit, meshes)?);
            Ok(rounded_fit(&vertices, 1)?.remove(0))
        })
        .collect()
}

/// Fit, then snap to the rounding grid without losing containment: round the
/// endpoints, bound their displacement (at most one grid step), and round the
/// padded radius up.
fn rounded_fit(vertices: &[Point3<f64>], max_bands: usize) -> Result<Vec<Capsule>, String> {
    fit_capsules_adaptive(vertices, max_bands)?
        .into_iter()
        .map(|fitted| {
            let a = round_point(&fitted.a);
            let b = round_point(&fitted.b);
            let radius = ((fitted.radius + ROUND) / ROUND).ceil() * ROUND;
            Ok(Capsule { a, b, radius })
        })
        .collect()
}

fn round_point(p: &Point3<f64>) -> Point3<f64> {
    Point3::new(round(p.x), round(p.y), round(p.z))
}

fn round(x: f64) -> f64 {
    (x / ROUND).round() * ROUND
}

fn body(name: &str, capsules: Vec<Capsule>) -> BodyCapsules {
    BodyCapsules { name: name.into(), capsules: capsules.iter().map(CapsuleSpec::from_capsule).collect() }
}
