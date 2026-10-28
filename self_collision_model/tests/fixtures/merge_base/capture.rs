//! Writes `distances.json` to stdout: the distances of the self-collision model of
//! public-peppy-libs at the merge base, built as the backbone of nodes-hub at its
//! merge base builds it, on configurations drawn with a fixed seed. `capture.sh`
//! builds it against the model crate of the merge base, under the name `reference`,
//! and passes the directory of `openarm_description` there as the one argument.
//!
//! For each hardware generation: every checked pair, and per configuration the arm
//! joints, the gripper openings, the distance of each checked pair (a model that
//! checks that pair alone answers it), and the model's nearest pair and distance.
//! The configurations are the home pose with the grippers open and closed,
//! `UNIFORM` configurations drawn over the joint limits, and for each threshold
//! of the backbone, `NEAR_EACH_THRESHOLD` configurations whose nearest distance lies
//! within `NEAR` of the threshold.

use openarm_description::{HardwareVersion, Side, home};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use reference::{BimanualCollisionModel as Model, ClipRegion, PairSpec, Point3};
use serde_json::{Value, json};

const SEED: u64 = 20261030;
const UNIFORM: usize = 100;
const NEAR_EACH_THRESHOLD: usize = 25;
/// The stop and the full-speed distances of the backbone, meters
/// (launchers-hub `openarm.json5`: `d_stop_m`, `d_safe_m`).
const THRESHOLDS: [f64; 2] = [0.005, 0.02];
/// How far from a threshold the nearest distance of a configuration drawn near it
/// lies, at most, meters.
const NEAR: f64 = 0.003;
const TORSO_BODY: &str = "openarm_body_link0";
const INF: f64 = f64::INFINITY;
/// The torso regions of the backbone at nodes-hub 9ccf97a6
/// (`openarm/backbone/src/torso.rs`), meters, URDF root frame.
const TORSO_REGIONS: [([f64; 3], [f64; 3]); 8] = [
    ([-INF, -INF, -INF], [INF, INF, 0.012]),
    ([-0.0335, -0.086, 0.006], [0.033, 0.086, 0.083]),
    ([-INF, -0.031, 0.009], [-0.0305, 0.031, 0.226]),
    ([-0.0335, -INF, 0.078], [INF, INF, 0.6025]),
    ([-INF, -INF, 0.5995], [INF, INF, 0.638]),
    ([-INF, -INF, 0.633], [INF, INF, 0.688]),
    ([-INF, -INF, 0.682], [INF, INF, 0.719]),
    ([-INF, -INF, 0.713], [INF, INF, INF]),
];

type Arms = [[f64; 7]; 2];

/// One configuration: the arm joints, left then right, and the gripper openings.
struct Configuration {
    arms: Arms,
    openings: [f64; 2],
}

fn build(version: HardwareVersion, meshes: &str, excluded: &[PairSpec]) -> Model {
    let regions = TORSO_REGIONS
        .iter()
        .map(|&(min, max)| ClipRegion::new(Point3::from(min), Point3::from(max)))
        .collect::<Result<Vec<_>, _>>()
        .expect("each torso region is a box");
    Model::builder(
        version.urdf(),
        meshes,
        version.base_link(Side::Left),
        version.base_link(Side::Right),
    )
    .regions(TORSO_BODY, regions)
    .exclude(excluded)
    .build()
    .expect("the model builds")
}

/// The nearest distance of `model` at `configuration`.
fn nearest(model: &mut Model, configuration: &Configuration) -> f64 {
    let [left, right] = configuration.openings;
    model.set_gripper_openings(left, right);
    let [q_left, q_right] = &configuration.arms;
    model
        .min_distance(q_left, q_right)
        .expect("a finite configuration")
        .distance
}

fn drawn_arms(rng: &mut StdRng, version: HardwareVersion) -> Arms {
    [Side::Left, Side::Right].map(|side| {
        version
            .joint_limits(side)
            .map(|[lower, upper]| rng.random_range(lower..=upper))
    })
}

fn drawn_openings(rng: &mut StdRng) -> [f64; 2] {
    [rng.random_range(0.0..=1.0), rng.random_range(0.0..=1.0)]
}

fn drawn(rng: &mut StdRng, version: HardwareVersion) -> Configuration {
    Configuration {
        arms: drawn_arms(rng, version),
        openings: drawn_openings(rng),
    }
}

/// A configuration whose nearest distance lies within `NEAR` of `threshold`: with
/// drawn gripper openings, on the segment between drawn arms clear of a drawn target
/// near the threshold and drawn arms closer than it, where the nearest distance meets
/// the target.
fn near(
    rng: &mut StdRng,
    version: HardwareVersion,
    model: &mut Model,
    threshold: f64,
) -> Configuration {
    let target = threshold + rng.random_range(-NEAR..=NEAR);
    let openings = drawn_openings(rng);
    let mut drawn_where = |clear: bool| loop {
        let arms = drawn_arms(rng, version);
        if (nearest(model, &Configuration { arms, openings }) > target) == clear {
            return arms;
        }
    };
    let (far, close) = (drawn_where(true), drawn_where(false));
    let between = |t: f64| Configuration {
        arms: std::array::from_fn(|side| {
            std::array::from_fn(|joint| {
                let (a, b) = (far[side][joint], close[side][joint]);
                a + t * (b - a)
            })
        }),
        openings,
    };
    let (mut clear, mut closer) = (0.0, 1.0);
    for _ in 0..40 {
        let middle = (clear + closer) / 2.0;
        if nearest(model, &between(middle)) > target {
            clear = middle;
        } else {
            closer = middle;
        }
    }
    let configuration = between(clear);
    let distance = nearest(model, &configuration);
    assert!(
        (distance - threshold).abs() <= NEAR,
        "the nearest distance {distance} m lies farther than {NEAR} m from {threshold} m"
    );
    configuration
}

fn capture(rng: &mut StdRng, version: HardwareVersion, meshes: &str) -> Value {
    let mut full = build(version, meshes, &[]);
    let pairs: Vec<(String, String)> = full
        .checked_pairs()
        .into_iter()
        .map(|(a, b)| (a.to_owned(), b.to_owned()))
        .collect();
    let mut alone: Vec<Model> = pairs
        .iter()
        .map(|kept| {
            let others: Vec<PairSpec> = pairs
                .iter()
                .filter(|pair| *pair != kept)
                .map(|(a, b)| PairSpec::new(a.as_str(), b.as_str()))
                .collect();
            build(version, meshes, &others)
        })
        .collect();
    let home = [home(Side::Left), home(Side::Right)];
    let mut configurations: Vec<Configuration> = [[1.0, 1.0], [0.0, 0.0]]
        .map(|openings| Configuration {
            arms: home,
            openings,
        })
        .into();
    configurations.extend((0..UNIFORM).map(|_| drawn(rng, version)));
    for threshold in THRESHOLDS {
        for _ in 0..NEAR_EACH_THRESHOLD {
            let configuration = near(rng, version, &mut full, threshold);
            configurations.push(configuration);
        }
    }
    let rows: Vec<Value> = configurations
        .iter()
        .map(|configuration| {
            let distances: Vec<f64> = alone
                .iter_mut()
                .map(|model| nearest(model, configuration))
                .collect();
            let [left, right] = configuration.openings;
            full.set_gripper_openings(left, right);
            let [q_left, q_right] = &configuration.arms;
            let answer = full
                .min_distance(q_left, q_right)
                .expect("a finite configuration");
            json!({
                "arms": configuration.arms,
                "openings": configuration.openings,
                "distances": distances,
                "nearest": {
                    "distance": answer.distance,
                    "pair": [answer.link_a, answer.link_b],
                },
            })
        })
        .collect();
    json!({
        "version": version.to_string(),
        "pairs": pairs.iter().map(|(a, b)| [a, b]).collect::<Vec<_>>(),
        "configurations": rows,
    })
}

fn main() {
    let description = std::env::args()
        .nth(1)
        .expect("the directory of openarm_description");
    let mut rng = StdRng::seed_from_u64(SEED);
    let models: Vec<Value> = [
        (HardwareVersion::V1, "meshes"),
        (HardwareVersion::V2, "meshes_v20"),
    ]
    .into_iter()
    .map(|(version, meshes)| capture(&mut rng, version, &format!("{description}/assets/{meshes}")))
    .collect();
    let fixture = json!({
        "source": "public-peppy-libs a817e346, torso regions of nodes-hub 9ccf97a6",
        "thresholds": THRESHOLDS,
        "models": models,
    });
    println!(
        "{}",
        serde_json::to_string_pretty(&fixture).expect("the fixture writes")
    );
}
