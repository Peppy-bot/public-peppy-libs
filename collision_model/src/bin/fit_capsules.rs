//! Generate a capsule config from a URDF and its collision meshes.
//!
//! Offline tool, robot-agnostic: chains and fixed bodies are named on the
//! command line, moving-link names come from walking each chain in the URDF,
//! and collision-bearing children hanging off a chain link (gripper fingers,
//! sensor pods) are baked into that link's capsule set across their full
//! joint travel. The output JSON is checked in wherever the consumer keeps
//! its config and loaded by the runtime.
//!
//! ```sh
//! cargo run --release --bin fit_capsules -- \
//!     --urdf tests/fixtures/openarm_v10.urdf --meshes tests/fixtures/meshes \
//!     --chain openarm_left_link0 --chain openarm_right_link0 \
//!     --fixed openarm_body_link0 --fixed openarm_left_link0 --fixed openarm_right_link0 \
//!     --out tests/fixtures/openarm_v10_capsules.json
//! ```

use collision_model::config::{BodyCapsules, CollisionConfig};
use collision_model::fit::fit_capsules_adaptive;
use collision_model::geometry::Capsule;
use collision_model::nalgebra::Point3;
use collision_model::urdf_collision::UrdfCollisions;
use srs_model::{ARM_DOF, Arm};

/// Endpoint/radius grid for the emitted JSON, 1 micrometer.
const ROUND: f64 = 1e-6;

struct Args {
    urdf: String,
    meshes: String,
    out: String,
    chains: Vec<String>,
    fixed: Vec<String>,
    /// Adaptive band search ceilings: compound fixed bodies (a torso) are
    /// worth many bands; limb links taper at most a little.
    max_bands_fixed: usize,
    max_bands_link: usize,
}

fn main() {
    if let Err(e) = run() {
        eprintln!("fit_capsules: {e}");
        std::process::exit(1);
    }
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        urdf: String::new(),
        meshes: String::new(),
        out: String::new(),
        chains: Vec::new(),
        fixed: Vec::new(),
        max_bands_fixed: 8,
        max_bands_link: 3,
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--urdf" => args.urdf = value()?,
            "--meshes" => args.meshes = value()?,
            "--out" => args.out = value()?,
            "--chain" => args.chains.push(value()?),
            "--fixed" => args.fixed.push(value()?),
            "--max-bands-fixed" => args.max_bands_fixed = value()?.parse().map_err(|e| format!("{e}"))?,
            "--max-bands-link" => args.max_bands_link = value()?.parse().map_err(|e| format!("{e}"))?,
            other => return Err(format!("unknown argument '{other}'")),
        }
    }
    if args.urdf.is_empty() || args.meshes.is_empty() || args.out.is_empty() || args.chains.is_empty() {
        return Err("required: --urdf <file> --meshes <dir> --out <file> --chain <base_link> (repeatable)".into());
    }
    Ok(args)
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let urdf = UrdfCollisions::from_file(&args.urdf)?;

    let fixed = args
        .fixed
        .iter()
        .map(|link| {
            let vertices = urdf.fixed_vertices_in_root(link, &args.meshes)?;
            Ok(body(link, rounded_fit(&vertices, args.max_bands_fixed)?))
        })
        .collect::<Result<Vec<_>, String>>()?;

    let chain_links: Vec<Vec<String>> = args
        .chains
        .iter()
        .map(|base| {
            let mut arm = Arm::from_urdf_file(&args.urdf, base)?;
            let posed = arm.at(&[0.0; ARM_DOF]);
            Ok((0..ARM_DOF).map(|i| posed.link_name(i)).collect())
        })
        .collect::<Result<_, String>>()?;
    // Chain links are themselves children of their parents; they move with
    // their own DOF and are modeled separately, so attachment baking must
    // not swallow them.
    let chain_set: std::collections::HashSet<&str> =
        chain_links.iter().flatten().map(String::as_str).collect();

    let mut links = Vec::new();
    for names in &chain_links {
        for name in names {
            let mut capsules = rounded_fit(&urdf.link_vertices(name, &args.meshes)?, args.max_bands_link)?;
            capsules.extend(attached_child_capsules(&urdf, name, &chain_set, &args.meshes)?);
            links.push(body(name, capsules));
        }
    }

    let source_urdf = args.urdf.rsplit('/').next().unwrap_or(&args.urdf).to_string();
    let config = CollisionConfig { source_urdf, fixed, links };
    std::fs::write(&args.out, config.to_json_pretty() + "\n").map_err(|e| format!("write {}: {e}", args.out))?;
    println!("wrote {}", args.out);
    Ok(())
}

/// Capsules for collision-bearing bodies attached directly below a chain
/// link (e.g. gripper fingers), expressed in the link's frame so the runtime
/// needs no extra state. A movable child is baked over the union of its
/// travel extremes; travel is a joint-space line, so containing both
/// extremes contains every intermediate position. Children without
/// collision meshes are skipped.
fn attached_child_capsules(
    urdf: &UrdfCollisions,
    link: &str,
    chain_set: &std::collections::HashSet<&str>,
    meshes: &str,
) -> Result<Vec<Capsule>, String> {
    urdf.children_of(link)
        .into_iter()
        .filter(|child| !chain_set.contains(child.as_str()) && !urdf.collisions_of(child).is_empty())
        .map(|child| {
            let joint = urdf.parent_joint(&child).expect("children_of implies a parent joint");
            let mut vertices = urdf.child_vertices_in_parent(&child, joint.lower_limit, meshes)?;
            if !joint.is_fixed {
                vertices.extend(urdf.child_vertices_in_parent(&child, joint.upper_limit, meshes)?);
            }
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
    BodyCapsules { name: name.into(), capsules }
}
