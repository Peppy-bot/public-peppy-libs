//! Classify a candidate pair set by sampling the reachable joint space,
//! then write the resulting pair list (with per-pair margins) into the
//! capsule config.
//!
//! Robot-agnostic: the candidate pairs come from a JSON file (an array of
//! `{"a": ..., "b": ...}` body names), the chains and reference poses from
//! the command line. Two outcomes per candidate pair:
//!
//! - **kept (margin 0)**: the default. Sampling can prove a pair approaches,
//!   never that it cannot, so distance evidence alone never drops a pair;
//!   pairs that never approached are only reported.
//! - **margined**: closer than the headroom at a reference pose. That
//!   closeness is structural, not a fault, so the pair gets
//!   `margin = baseline - headroom` and alarms only on getting closer than
//!   its rest baseline. Pairs whose capsules already overlap at reference
//!   are flagged: their alarm is baseline-relative, not an absolute
//!   pre-contact guarantee.
//!
//! Deterministic: a fixed-seed xorshift drives the sampling, so reruns give
//! identical artifacts. The written pairs carry a fingerprint of the
//! capsules they were classified against.
//!
//! ```sh
//! cargo run --release --bin classify_pairs -- \
//!     --urdf tests/fixtures/openarm_v10.urdf \
//!     --config tests/fixtures/openarm_v10_capsules.json \
//!     --left openarm_left_link0 --right openarm_right_link0 \
//!     --candidates tests/fixtures/openarm_v10_pair_candidates.json \
//!     --reference 0,0,0,0,0,0,0 --reference 0,0,0,0.1,0,0,0
//! ```

use std::collections::HashMap;

use collision_model::config::CollisionConfig;
use collision_model::{DualArmCollisionModel, PairSpec};
use srs_model::{ARM_DOF, Arm, JointVec};

/// Sampled configurations (per arm, drawn independently).
const SAMPLES: usize = 30_000;
/// A pair whose sampled minimum never drops below this is reported as
/// never-approaching (but still kept).
const FAR_FLOOR: f64 = 0.15;
/// Margined pairs read this much clearance at their worst reference pose.
/// This floor caps the rest-pose global minimum, so a runtime governor or
/// watchdog band must keep `d_safe` below it or rest poses throttle.
const HEADROOM: f64 = 0.04;

struct Args {
    urdf: String,
    config: String,
    left: String,
    right: String,
    candidates: String,
    /// Poses that must read as clear (clamped into each arm's own limits).
    references: Vec<JointVec>,
}

fn main() {
    if let Err(e) = run() {
        eprintln!("classify_pairs: {e}");
        std::process::exit(1);
    }
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        urdf: String::new(),
        config: String::new(),
        left: String::new(),
        right: String::new(),
        candidates: String::new(),
        references: Vec::new(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--urdf" => args.urdf = value()?,
            "--config" => args.config = value()?,
            "--left" => args.left = value()?,
            "--right" => args.right = value()?,
            "--candidates" => args.candidates = value()?,
            "--reference" => args.references.push(parse_joints(&value()?)?),
            other => return Err(format!("unknown argument '{other}'")),
        }
    }
    if args.urdf.is_empty()
        || args.config.is_empty()
        || args.left.is_empty()
        || args.right.is_empty()
        || args.candidates.is_empty()
        || args.references.is_empty()
    {
        return Err(
            "required: --urdf <file> --config <json> --left <base> --right <base> \
             --candidates <json> --reference <q1,..,q7> (repeatable)"
                .into(),
        );
    }
    Ok(args)
}

fn parse_joints(s: &str) -> Result<JointVec, String> {
    let vals: Vec<f64> = s
        .split(',')
        .map(|p| p.trim().parse::<f64>().map_err(|e| format!("bad joint value '{p}': {e}")))
        .collect::<Result<_, _>>()?;
    vals.try_into().map_err(|v: Vec<f64>| format!("expected {ARM_DOF} joint values, got {}", v.len()))
}

fn run() -> Result<(), String> {
    let args = parse_args()?;

    let mut config = CollisionConfig::from_file(&args.config)?;
    let loaded = config.clone().parse()?;
    let candidates: Vec<PairSpec> = serde_json::from_str(
        &std::fs::read_to_string(&args.candidates).map_err(|e| format!("read candidates: {e}"))?,
    )
    .map_err(|e| format!("parse candidates: {e}"))?;

    let urdf_text = std::fs::read_to_string(&args.urdf).map_err(|e| format!("read urdf: {e}"))?;
    let mut model = DualArmCollisionModel::with_pairs(&urdf_text, &args.left, &args.right, &loaded, &candidates)?;
    // The arms' joint ranges can be mirrored, so each side samples its own.
    let limits_l = Arm::from_urdf(&urdf_text, &args.left)?.limits();
    let limits_r = Arm::from_urdf(&urdf_text, &args.right)?.limits();

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
    for q in &args.references {
        let ql: JointVec = std::array::from_fn(|i| q[i].clamp(limits_l[i].lo, limits_l[i].hi));
        let qr: JointVec = std::array::from_fn(|i| q[i].clamp(limits_r[i].lo, limits_r[i].hi));
        for (a, b, d) in model.pair_distances_raw(&ql, &qr)? {
            reference_min
                .entry((a, b))
                .and_modify(|m| *m = m.min(d))
                .or_insert(d);
        }
    }

    let mut kept = Vec::new();
    let (mut far, mut margined, mut baseline_overlapping) = (0, 0, 0);
    for spec in &candidates {
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
        "classified {} candidate pairs: all kept; {margined} with margins ({baseline_overlapping} overlap at reference), {far} never approached in sampling",
        candidates.len(),
    );
    config.pairs = kept;
    config.pairs_fingerprint = Some(config.capsules_fingerprint());
    std::fs::write(&args.config, config.to_json_pretty() + "\n").map_err(|e| format!("write {}: {e}", args.config))?;
    println!("wrote {}", args.config);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xorshift_uniform_stays_in_range_and_is_deterministic() {
        let mut a = XorShift(42);
        let mut b = XorShift(42);
        for _ in 0..1000 {
            let x = a.uniform(-2.0, 3.0);
            assert!((-2.0..=3.0).contains(&x));
            assert_eq!(x, b.uniform(-2.0, 3.0));
        }
    }

    #[test]
    fn parse_joints_validates_count() {
        assert!(parse_joints("0,0,0,0,0,0,0").is_ok());
        assert!(parse_joints("0,0,0").is_err());
    }
}
