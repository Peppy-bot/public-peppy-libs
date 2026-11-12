//! OpenArm's self-collision models, built from the descriptions robot_description
//! bundles: where each generation's fingers hang, and v2's arms, distances and home.

use std::collections::BTreeSet;

use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::ChainGeometry;
use robot_description::{Arm, RobotDescription, states};
use self_collision_model::{CollisionError, FingerError};

#[path = "fixtures/models.rs"]
mod fixture;

use fixture::openarm::{ARM_DOF, Model};
use fixture::{arm_geometry, chain_links, description, meshes, model, posture};

/// The OpenArm models robot_description bundles, by model id.
const GENERATIONS: [&str; 2] = ["openarm_v1", "openarm_v2"];

/// The model id of OpenArm v1.
const V1: &str = "openarm_v1";
/// The model id of OpenArm v2.
const V2: &str = "openarm_v2";

/// The bodies that ride `arm`: the links of its chain and its gripper's fingers.
fn riders(description: &RobotDescription, arm: &Arm) -> Vec<String> {
    chain_links::<ARM_DOF>(description, arm)
        .into_iter()
        .chain(arm.gripper.fingers.iter().map(|finger| finger.link.clone()))
        .collect()
}

// Each arm's chain of v2 carries its gripper's fingers, and each finger hangs off the
// link of the chain that carries it at the pose the model places it at, which follows
// its arm's gripper opening.
#[test]
fn the_fingers_hang_where_the_model_places_them() {
    let description = description(V2);
    let meshes = meshes(V2);
    let mut model: Model = model(V2);
    let arms: Vec<ChainGeometry<ARM_DOF>> = description
        .arms()
        .iter()
        .map(|arm| arm_geometry(&description, &meshes, arm))
        .collect();
    let q = [
        [0.3, -0.4, 0.2, 1.1, -0.3, 0.2, 0.4],
        [-0.2, 0.3, -0.1, 0.9, 0.4, -0.3, -0.5],
    ];
    for openings in [[0.0, 0.0], [0.6, 0.2], [1.0, 1.0]] {
        model.set_gripper_openings(&openings);
        let hung: Vec<_> = description
            .arms()
            .iter()
            .zip(&arms)
            .enumerate()
            .flat_map(|(index, (arm, geometry))| {
                let fingers = model
                    .fingers(index, geometry.carried())
                    .expect("each arm's chain carries its own fingers");
                let carried: BTreeSet<&str> = fingers.iter().map(|finger| finger.name).collect();
                let named: BTreeSet<&str> = arm
                    .gripper
                    .fingers
                    .iter()
                    .map(|finger| finger.link.as_str())
                    .collect();
                assert_eq!(carried, named, "{} carries its gripper's fingers", arm.name);
                let posed = geometry.chain().at(&q[index]);
                fingers
                    .into_iter()
                    .map(|finger| {
                        let host = posed.link_name(finger.segment).to_string();
                        (finger.name.to_string(), host, finger.offset)
                    })
                    .collect::<Vec<_>>()
            })
            .collect();
        let placed = model.world_pieces(&q).expect("placed");
        let pose_of = |name: &str| {
            *placed
                .iter()
                .find(|(body, _)| *body == name)
                .unwrap_or_else(|| panic!("a body {name}"))
                .1[0]
                .pose()
        };
        for (link, host, offset) in &hung {
            let hanging = pose_of(host) * offset;
            let expected = pose_of(link);
            assert!(
                (hanging.translation.vector - expected.translation.vector).norm() < 1e-12
                    && hanging.rotation.angle_to(&expected.rotation) < 1e-12,
                "{link} at openings {openings:?} hangs at {hanging}, the model places it at \
                 {expected}"
            );
        }
    }
}

// The self model and the arm's chain place each finger alike when the gripper is
// closed: for each finger that the chain of each arm of each generation carries, the
// model's offset at an opening of 0 is the offset the chain carries it at, with its
// joint at 0, within 1e-12 m and rad.
#[test]
fn a_closed_finger_hangs_where_the_chain_carries_it() {
    for id in GENERATIONS {
        let description = description(id);
        let meshes = meshes(id);
        let mut model: Model = model(id);
        model.set_gripper_openings(&[0.0, 0.0]);
        for (index, arm) in description.arms().iter().enumerate() {
            let geometry = arm_geometry::<ARM_DOF>(&description, &meshes, arm);
            let carried = geometry.carried();
            assert_eq!(
                carried.len(),
                arm.gripper.fingers.len(),
                "{id} {} carries its gripper's fingers",
                arm.name
            );
            let fingers = model
                .fingers(index, carried)
                .expect("each arm's chain carries its own fingers");
            for (part, finger) in carried.iter().zip(fingers) {
                let at_zero = part.rider();
                let (closed, chain_offset) = (finger.offset, at_zero.offset);
                assert!(
                    (closed.translation.vector - chain_offset.translation.vector).norm() < 1e-12
                        && closed.rotation.angle_to(&chain_offset.rotation) < 1e-12,
                    "{id} {}: closed at {closed}, carried at {chain_offset}",
                    finger.name
                );
            }
        }
    }
}

/// A chain of one revolute joint whose link carries a part of each name of `parts`, each
/// on a revolute joint off its path.
fn strange_parts(parts: [&str; 3]) -> String {
    let part = |index: usize, name: &str| {
        format!(
            r#"
    <link name="{name}">
      <collision><geometry><box size="0.02 0.02 0.02"/></geometry></collision>
    </link>
    <joint name="part_{index}_joint" type="revolute">
      <parent link="link_1"/><child link="{name}"/><axis xyz="1 0 0"/>
      <limit lower="0" upper="1" effort="1" velocity="1"/>
    </joint>"#
        )
    };
    let parts: String = parts
        .iter()
        .enumerate()
        .map(|(index, name)| part(index, name))
        .collect();
    format!(
        r#"<robot name="strange_parts">
    <link name="base"/>
    <link name="link_1">
      <collision><geometry><box size="0.1 0.04 0.04"/></geometry></collision>
    </link>
    <joint name="joint_1" type="revolute">
      <parent link="base"/><child link="link_1"/><axis xyz="0 0 1"/>
      <limit lower="-1" upper="1" effort="1" velocity="1"/>
    </joint>{parts}
  </robot>"#
    )
}

// The self model refuses, naming the part, each part a chain carries that it holds as
// no finger of that arm on that link: a part of no body, a link of an arm, a finger of
// the other arm, and a finger the chain carries on another link than the model hangs it
// off.
#[test]
fn a_part_the_model_holds_as_no_finger_of_the_arm_is_refused() {
    let description = description(V2);
    let meshes = meshes(V2);
    let model: Model = model(V2);
    let [left, right] = description.arms() else {
        panic!("OpenArm has two arms");
    };
    let arm_link = chain_links::<ARM_DOF>(&description, left)[0].clone();
    let finger = left.gripper.fingers[0].link.clone();
    let strange = ChainGeometry::<1>::from_urdf(
        &strange_parts(["stranger", &arm_link, &finger]),
        "",
        &ChainSpec {
            base_link: None,
            tip_link: "link_1",
            joints: JointSelection::PathOrder,
        },
    )
    .expect("the strange chain fits");
    let refusal = |link: &str| {
        let part = strange
            .carried()
            .iter()
            .find(|part| part.rider().name == link)
            .unwrap_or_else(|| panic!("the strange chain carries {link}"));
        model
            .fingers(0, std::slice::from_ref(part))
            .expect_err("a part of another robot is refused")
    };
    assert_eq!(
        refusal("stranger"),
        FingerError::NoBody {
            link: "stranger".to_string()
        }
    );
    assert_eq!(
        refusal(&arm_link),
        FingerError::NotAFinger {
            link: arm_link.clone()
        }
    );
    let left_geometry = arm_geometry::<ARM_DOF>(&description, &meshes, left);
    let hand = left_geometry
        .carried()
        .iter()
        .map(|part| part.rider())
        .find(|rider| rider.name == finger)
        .unwrap_or_else(|| panic!("{} carries {finger}", left.name))
        .segment;
    assert_eq!(
        refusal(&finger),
        FingerError::OtherLink {
            link: finger.clone(),
            model: hand,
            chain: 0
        }
    );
    let first = left_geometry.carried()[0].rider().name.to_string();
    assert_eq!(
        model
            .fingers(1, left_geometry.carried())
            .expect_err(&format!(
                "the fingers of {} are refused for {}",
                left.name, right.name
            )),
        FingerError::OtherChain {
            link: first,
            chain: 1
        }
    );
}

// Every checked pair of OpenArm v2 holds a body of an arm, so the robot's least distance
// is the lesser of its arms' own, each over the pairs one of whose bodies rides it, and
// each arm's own pair holds a body of that arm. At each turn of the right arm at which a
// pair of the right arm alone is the robot's nearest, the left arm's own distance lies
// farther off.
#[test]
fn an_arms_distance_counts_the_pairs_it_rides() {
    let description = description(V2);
    let mut model: Model = model(V2);
    let riders: Vec<Vec<String>> = description
        .arms()
        .iter()
        .map(|arm| riders(&description, arm))
        .collect();
    let rides = |arm: usize, link: &str| riders[arm].iter().any(|rider| rider == link);
    // The left arm stands turned 0.05 rad at its second joint, between the right arm's
    // turns, so the arms never stand as mirror images, whose distances tie.
    let left = [0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut right_nearest_turns = 0;
    for turn in (-15..=15).map(|k| f64::from(k) * 0.1) {
        let right = [0.0, turn, 0.0, 0.0, 0.0, 0.0, 0.0];
        let robot = model.min_distance(&[left, right]).expect("a distance");
        let (nearest, a, b) = (
            robot.distance,
            robot.link_a.to_string(),
            robot.link_b.to_string(),
        );
        let of_left = model
            .min_distance_of_chain(0, &[left, right])
            .expect("a distance");
        assert!(rides(0, of_left.link_a) || rides(0, of_left.link_b));
        let of_left = of_left.distance;
        let of_right = model
            .min_distance_of_chain(1, &[left, right])
            .expect("a distance");
        assert!(rides(1, of_right.link_a) || rides(1, of_right.link_b));
        assert_eq!(nearest, of_left.min(of_right.distance), "turn {turn}");
        if !rides(0, &a) && !rides(0, &b) {
            assert!(
                of_left > nearest,
                "turn {turn}: {of_left} against {nearest}"
            );
            right_nearest_turns += 1;
        }
    }
    assert!(
        right_nearest_turns > 0,
        "no turn of the right arm whose pair is the robot's nearest"
    );
}

/// Postures of the left arm and the right, both folded inward.
const FOLDED_INWARD: [([f64; ARM_DOF], [f64; ARM_DOF]); 4] = [
    (
        [0.15, 0.1, 0.85, 0.5, -0.2, 0.1, 0.0],
        [-0.05, -0.25, -0.45, 0.35, 0.1, -0.1, 0.0],
    ),
    (
        [0.0, 0.3, 0.95, 0.45, 0.1, 0.0, 0.0],
        [0.0, -0.1, -0.55, 0.4, 0.0, 0.1, 0.0],
    ),
    (
        [0.25, -0.1, 0.6, 0.65, 0.0, 0.2, 0.1],
        [-0.1, 0.05, -0.7, 0.3, 0.0, -0.2, 0.0],
    ),
    (
        [0.0, 0.0, 0.95, 0.4, 0.1, 0.0, 0.2],
        [0.0, 0.0, -1.05, 0.4, -0.1, 0.1, 0.0],
    ),
];

// An arm's distance alone reads the pairs it rides that no body of the other arm is one
// of: on postures of both arms folded inward, its pair holds a body of that arm and none
// of the other's, and it equals the arm's own distance where that one's pair holds no
// body of the other arm, and lies at or past it elsewhere; on some posture a pair of the
// two arms is the arm's nearest.
#[test]
fn an_arms_distance_alone_sets_the_other_arm_aside() {
    let description = description(V2);
    let mut model: Model = model(V2);
    let riders: Vec<Vec<String>> = description
        .arms()
        .iter()
        .map(|arm| riders(&description, arm))
        .collect();
    let rides = |arm: usize, link: &str| riders[arm].iter().any(|rider| rider == link);
    let mut crossing = 0;
    for (left, right) in FOLDED_INWARD {
        for (arm, other) in [(0, 1), (1, 0)] {
            let own = model
                .min_distance_of_chain(arm, &[left, right])
                .expect("a distance");
            let own_crosses = rides(other, own.link_a) || rides(other, own.link_b);
            let own = own.distance;
            let alone = model
                .min_distance_of_chain_alone(arm, &[left, right])
                .expect("a distance");
            assert!(rides(arm, alone.link_a) || rides(arm, alone.link_b));
            assert!(!rides(other, alone.link_a) && !rides(other, alone.link_b));
            if own_crosses {
                crossing += 1;
                assert!(
                    alone.distance >= own,
                    "arm {arm}: {} against {own}",
                    alone.distance
                );
            } else {
                assert_eq!(alone.distance, own, "arm {arm}");
            }
        }
    }
    assert!(
        crossing > 0,
        "no posture whose arm's nearest pair is of the two arms"
    );
}

/// Checks `bounded`, which reads at a bound the distance `exact`: the distance itself with
/// no bound and at bounds past it, and at or above the bound and no larger than the
/// distance at the distance and at bounds under it. The count of reads under the distance.
fn reads_under_the_distance(exact: f64, mut bounded: impl FnMut(f64) -> f64) -> usize {
    for within in [f64::INFINITY, exact + 0.05, exact + 1e-6] {
        assert_eq!(bounded(within), exact, "within {within}");
    }
    [exact, exact - 1e-6, exact - 0.05, -1.0]
        .into_iter()
        .filter(|&within| {
            let read = bounded(within);
            assert!(
                within <= read && read <= exact,
                "within {within}: {read} against {exact}"
            );
            read < exact
        })
        .count()
}

// A distance read at a bound is the distance itself where that lies under the bound, and
// elsewhere at or above the bound and no larger than the distance: of every pair, of an
// arm's pairs and of an arm's pairs alone, on the postures of both arms folded inward. On
// some posture a read past its bound is a lower bound under the distance.
#[test]
fn a_distance_read_at_a_bound_is_exact_under_it_and_a_lower_bound_past_it() {
    let mut model: Model = model(V2);
    let mut under_the_distance = 0;
    for (left, right) in FOLDED_INWARD {
        let q = [left, right];
        let every_pair = model.min_distance(&q).expect("a distance").distance;
        under_the_distance += reads_under_the_distance(every_pair, |within| {
            model.min_distance_within(&q, within).expect("a distance")
        });
        for chain in [0, 1] {
            let of_arm = model
                .min_distance_of_chain(chain, &q)
                .expect("a distance")
                .distance;
            under_the_distance += reads_under_the_distance(of_arm, |within| {
                model
                    .min_distance_of_chain_within(chain, &q, within)
                    .expect("a distance")
            });
            let alone = model
                .min_distance_of_chain_alone(chain, &q)
                .expect("a distance")
                .distance;
            under_the_distance += reads_under_the_distance(alone, |within| {
                model
                    .min_distance_of_chain_alone_within(chain, &q, within)
                    .expect("a distance")
            });
        }
    }
    assert!(
        under_the_distance > 0,
        "no read past its bound that lies under the distance"
    );
    let (left, right) = FOLDED_INWARD[0];
    assert!(matches!(
        model.min_distance_within(&[left, right], f64::NAN),
        Err(CollisionError::NonFinite)
    ));
}

/// The self clearance the OpenArm launchers' motion planner checks a state at: its
/// 0.04 m clearance and 1 mm of spare for each of a pair's two moving links.
const PLANNER_SELF_CLEARANCE_M: f64 = 0.042;

// At home every pair of either OpenArm's bodies stands farther apart than the planner's self
// clearance, its grippers at any opening from shut to open, so a plan ends at home.
#[test]
fn at_home_every_pair_stands_beyond_the_planners_self_clearance_at_every_opening() {
    for id in [V1, V2] {
        let description = description(id);
        let mut model: Model = model(id);
        let home = posture(&description, states::HOME);
        for step in 0..=10 {
            let opening = f64::from(step) / 10.0;
            model.set_gripper_openings(&[opening, opening]);
            let nearest = model.min_distance(&home).expect("a distance");
            assert!(
                nearest.distance > PLANNER_SELF_CLEARANCE_M,
                "{id} at {opening}: {} - {} at {}",
                nearest.link_a,
                nearest.link_b,
                nearest.distance
            );
        }
    }
}
