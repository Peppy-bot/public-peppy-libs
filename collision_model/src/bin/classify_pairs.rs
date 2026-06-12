//! Classify the structural pair set by sampling the reachable joint space,
//! then write the resulting pair list (with per-pair margins) into the
//! capsule config.
//!
//! Three outcomes per structural pair:
//!
//! - **dropped**: its capsule distance never came within [`FAR_FLOOR`] across
//!   every sample; it cannot approach, so checking it is wasted work.
//! - **margined**: it sits closer than [`HEADROOM`] at the reference poses
//!   (home, ready). That closeness is structural, not a fault, so the pair
//!   gets `margin = baseline - HEADROOM` and alarms only on getting closer
//!   than its rest baseline.
//! - **kept** as-is (margin 0) otherwise.
//!
//! Deterministic: a fixed-seed xorshift drives the sampling, so reruns give
//! identical artifacts.
//!
//! ```sh
//! cargo run --release --bin classify_pairs
//! ```

use std::collections::HashMap;

use collision_model::config::CollisionConfig;
use collision_model::{DualArmCollisionModel, PairSpec, openarm_structural_pairs};
use srs_model::{ARM_DOF, Arm, JointVec};

const URDF_BASENAME: &str = "openarm_v10.urdf";
const CONFIG_BASENAME: &str = "openarm_v10_capsules.json";
/// Sampled configurations (per arm, drawn independently).
const SAMPLES: usize = 30_000;
/// A pair whose sampled minimum never drops below this is dropped.
const FAR_FLOOR: f64 = 0.15;
/// Margined pairs read this much clearance at their worst reference pose.
/// This floor caps the rest-pose global minimum, so a runtime governor or
/// watchdog band must keep `d_safe` below it or rest poses throttle.
const HEADROOM: f64 = 0.04;
/// Reference poses that must read as clear: home, and the arm node's ready
/// pose. Both are clamped into each arm's own joint limits before use (the
/// all-zero home pose sits below the elbow's one-sided lower limit).
const REFERENCE: [JointVec; 2] = [[0.0; ARM_DOF], [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]];

fn main() {
    if let Err(e) = run() {
        eprintln!("classify_pairs: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let assets = format!("{}/assets", env!("CARGO_MANIFEST_DIR"));
    let urdf_path = format!("{assets}/{URDF_BASENAME}");
    let config_path = format!("{assets}/{CONFIG_BASENAME}");

    let mut config = CollisionConfig::from_file(&config_path)?;
    let loaded = config.clone().parse()?;
    let structural = openarm_structural_pairs();
    let urdf_text = std::fs::read_to_string(&urdf_path).map_err(|e| format!("read urdf: {e}"))?;
    let mut model = DualArmCollisionModel::with_pairs(
        &urdf_text,
        "openarm_left_link0",
        "openarm_right_link0",
        &loaded,
        &structural,
    )?;
    // The arms' j1/j2 ranges are mirrored, so each side samples its own limits.
    let limits_l = Arm::from_urdf_file(&urdf_path, "openarm_left_link0")?.limits();
    let limits_r = Arm::from_urdf_file(&urdf_path, "openarm_right_link0")?.limits();

    // Sampled minimum per pair across reachable space.
    let mut sampled_min: HashMap<(String, String), f64> = HashMap::new();
    let mut rng = XorShift(0x9E37_79B9_7F4A_7C15);
    for _ in 0..SAMPLES {
        let ql: JointVec = std::array::from_fn(|i| rng.uniform(limits_l[i].lo, limits_l[i].hi));
        let qr: JointVec = std::array::from_fn(|i| rng.uniform(limits_r[i].lo, limits_r[i].hi));
        for (a, b, d) in model.pair_distances_raw(&ql, &qr)? {
            sampled_min
                .entry((a, b))
                .and_modify(|m| *m = m.min(d))
                .or_insert(d);
        }
    }

    // Baseline per pair over the reference poses.
    let mut reference_min: HashMap<(String, String), f64> = HashMap::new();
    for q in &REFERENCE {
        let ql: JointVec = std::array::from_fn(|i| q[i].clamp(limits_l[i].lo, limits_l[i].hi));
        let qr: JointVec = std::array::from_fn(|i| q[i].clamp(limits_r[i].lo, limits_r[i].hi));
        for (a, b, d) in model.pair_distances_raw(&ql, &qr)? {
            reference_min
                .entry((a, b))
                .and_modify(|m| *m = m.min(d))
                .or_insert(d);
        }
    }

    // Every structural pair stays checked: sampling can prove a pair CAN
    // approach, never that it cannot, so distance alone never drops a pair.
    let mut kept = Vec::new();
    let (mut far, mut margined, mut baseline_overlapping) = (0, 0, 0);
    for spec in &structural {
        let key = (spec.a.clone(), spec.b.clone());
        let sampled = sampled_min[&key];
        if sampled >= FAR_FLOOR {
            far += 1;
            println!("far    {:40} sampled_min={sampled:+.3} (kept, margin 0)", format!("{} / {}", spec.a, spec.b));
        }
        let baseline = reference_min[&key];
        let margin = if baseline < HEADROOM {
            margined += 1;
            baseline - HEADROOM
        } else {
            0.0
        };
        if margin != 0.0 {
            println!(
                "margin {:40} baseline={baseline:+.3} sampled_min={sampled:+.3} margin={margin:+.3}",
                format!("{} / {}", spec.a, spec.b),
            );
            if baseline < 0.0 {
                baseline_overlapping += 1;
                println!(
                    "       WARNING: capsules already overlap at the reference pose; the alarm \
for this pair is baseline-relative, not an absolute pre-contact guarantee",
                );
            }
        }
        kept.push(PairSpec { a: spec.a.clone(), b: spec.b.clone(), margin: round_mm(margin) });
    }

    println!(
        "classified {} structural pairs: all kept; {margined} with margins ({baseline_overlapping} overlap at reference), {far} never approached in sampling",
        structural.len(),
    );
    config.pairs = kept;
    config.pairs_fingerprint = Some(config.capsules_fingerprint());
    std::fs::write(&config_path, config.to_json_pretty() + "\n").map_err(|e| format!("write {config_path}: {e}"))?;
    println!("wrote {config_path}");
    Ok(())
}

/// Round a margin to 0.1 mm so the artifact diffs stay readable.
fn round_mm(x: f64) -> f64 {
    (x * 10_000.0).round() / 10_000.0
}

/// xorshift64*: deterministic, dependency-free sampling.
struct XorShift(u64);

impl XorShift {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        let u = (self.next() >> 11) as f64 / (1u64 << 53) as f64;
        lo + (hi - lo) * u
    }
}
