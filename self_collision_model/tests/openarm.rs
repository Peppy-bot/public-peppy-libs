//! OpenArm's self-collision model, built from openarm_description alone.

use collision_geometry::chain::ChainGeometry;
use openarm_description::{ArmPair, HardwareVersion, Side};
use self_collision_model::{FingerError, openarm};
use srs_model::chain_kinematics::{ChainSpec, JointSelection};

// Each hardware generation's model builds from its embedded description, checks pairs,
// and finds both arms at their zero configuration clear of each other and the torso.
#[test]
fn each_generation_builds_from_its_description() {
    for version in [HardwareVersion::V1, HardwareVersion::V2] {
        let dir =
            std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{version:?}_meshes"));
        version.write_meshes_to(&dir).expect("write the meshes");
        let mut model =
            openarm(version, dir.to_str().expect("a UTF-8 directory")).expect("the model builds");
        assert!(
            !model.checked_pairs().is_empty(),
            "{version:?} checks no pair"
        );
        let zero = [0.0; 7];
        let proximity = model.min_distance(&zero, &zero).expect("a distance");
        assert!(
            proximity.distance > 0.0,
            "{version:?} collides at zero: {}",
            proximity.distance
        );
    }
}

// Each finger that a v2 arm's chain carries hangs off its hand, the link of the seventh
// joint, at the pose the model places it at, which follows its side's gripper opening:
// two fingers a side.
#[test]
fn the_fingers_hang_where_the_model_places_them() {
    let version = HardwareVersion::V2;
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("V2_finger_meshes");
    version.write_meshes_to(&dir).expect("write the meshes");
    let meshes = dir.to_str().expect("a UTF-8 directory");
    let mut model = openarm(version, meshes).expect("the model builds");
    let arm = |side: Side| {
        ChainGeometry::<7>::from_urdf(
            version.urdf(),
            meshes,
            &ChainSpec {
                base_link: Some(version.base_link(side)),
                tip_link: version.tcp_link(side),
                joints: JointSelection::PathOrder,
            },
        )
        .expect("the arm fits")
    };
    let arms = ArmPair::new(arm(Side::Left), arm(Side::Right));
    let (q_left, q_right) = (
        [0.3, -0.4, 0.2, 1.1, -0.3, 0.2, 0.4],
        [-0.2, 0.3, -0.1, 0.9, 0.4, -0.3, -0.5],
    );
    for (left, right) in [(0.0, 0.0), (0.6, 0.2), (1.0, 1.0)] {
        model.set_gripper_openings(left, right);
        let fingers: Vec<_> = Side::BOTH
            .into_iter()
            .flat_map(|side| {
                model
                    .fingers(side, arms[side].carried())
                    .expect("each arm's chain carries its own fingers")
                    .into_iter()
                    .map(|f| (f.link.to_string(), side, f.segment, f.offset))
                    .collect::<Vec<_>>()
            })
            .collect();
        for side in [Side::Left, Side::Right] {
            let on_side = fingers.iter().filter(|(_, s, _, _)| *s == side).count();
            assert_eq!(on_side, 2, "{side:?} has {on_side} fingers");
        }
        assert!(fingers.iter().all(|(_, _, segment, _)| *segment == 6));
        let placed = model.world_pieces(&q_left, &q_right).expect("placed");
        let pose_of = |name: &str| {
            *placed
                .iter()
                .find(|(body, _)| *body == name)
                .unwrap_or_else(|| panic!("a body {name}"))
                .1[0]
                .pose()
        };
        for (link, side, _, offset) in &fingers {
            let hand = pose_of(match side {
                Side::Left => "openarm_left_ee_base_link",
                Side::Right => "openarm_right_ee_base_link",
            });
            let hung = hand * offset;
            let expected = pose_of(link);
            assert!(
                (hung.translation.vector - expected.translation.vector).norm() < 1e-12
                    && hung.rotation.angle_to(&expected.rotation) < 1e-12,
                "{link} at openings {left} and {right} hangs at {hung}, the model places it \
                 at {expected}"
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
    for version in [HardwareVersion::V1, HardwareVersion::V2] {
        let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"))
            .join(format!("{version:?}_closed_finger_meshes"));
        version.write_meshes_to(&dir).expect("write the meshes");
        let meshes = dir.to_str().expect("a UTF-8 directory");
        let mut model = openarm(version, meshes).expect("the model builds");
        model.set_gripper_openings(0.0, 0.0);
        for side in Side::BOTH {
            let chain = ChainGeometry::<7>::from_urdf(
                version.urdf(),
                meshes,
                &ChainSpec {
                    base_link: Some(version.base_link(side)),
                    tip_link: version.tcp_link(side),
                    joints: JointSelection::PathOrder,
                },
            )
            .expect("the arm fits");
            let carried = chain.carried();
            assert_eq!(carried.len(), 2, "{version:?} {side:?} carries two fingers");
            let fingers = model
                .fingers(side, carried)
                .expect("each arm's chain carries its own fingers");
            for (part, finger) in carried.iter().zip(fingers) {
                let at_zero = part.rider();
                let (closed, chain_offset) = (finger.offset, at_zero.offset);
                assert!(
                    (closed.translation.vector - chain_offset.translation.vector).norm() < 1e-12
                        && closed.rotation.angle_to(&chain_offset.rotation) < 1e-12,
                    "{version:?} {}: closed at {closed}, carried at {chain_offset}",
                    finger.link
                );
            }
        }
    }
}

/// A chain of one revolute joint whose link carries three parts on revolute joints off
/// its path, each named as a part of OpenArm v2 that its self model refuses as a finger
/// of the chain's first link: a name it holds no body of, a link of an arm, and a finger
/// of the left hand.
const STRANGE_PARTS: &str = r#"<robot name="strange_parts">
    <link name="base"/>
    <link name="link_1">
      <collision><geometry><box size="0.1 0.04 0.04"/></geometry></collision>
    </link>
    <joint name="joint_1" type="revolute">
      <parent link="base"/><child link="link_1"/><axis xyz="0 0 1"/>
      <limit lower="-1" upper="1" effort="1" velocity="1"/>
    </joint>
    <link name="stranger">
      <collision><geometry><box size="0.02 0.02 0.02"/></geometry></collision>
    </link>
    <joint name="stranger_joint" type="revolute">
      <parent link="link_1"/><child link="stranger"/><axis xyz="1 0 0"/>
      <limit lower="0" upper="1" effort="1" velocity="1"/>
    </joint>
    <link name="openarm_left_link3">
      <collision><geometry><box size="0.02 0.02 0.02"/></geometry></collision>
    </link>
    <joint name="link3_joint" type="revolute">
      <parent link="link_1"/><child link="openarm_left_link3"/><axis xyz="1 0 0"/>
      <limit lower="0" upper="1" effort="1" velocity="1"/>
    </joint>
    <link name="openarm_left_ee_link1">
      <collision><geometry><box size="0.02 0.02 0.02"/></geometry></collision>
    </link>
    <joint name="finger_joint" type="revolute">
      <parent link="link_1"/><child link="openarm_left_ee_link1"/><axis xyz="1 0 0"/>
      <limit lower="0" upper="1" effort="1" velocity="1"/>
    </joint>
  </robot>"#;

// The self model refuses, naming the part, each part a chain carries that it holds as
// no finger of that arm on that link: a part of no body, a link of an arm, a finger of
// the other arm, and a finger the chain carries on another link than the model hangs it
// off.
#[test]
fn a_part_the_model_holds_as_no_finger_of_the_arm_is_refused() {
    let version = HardwareVersion::V2;
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("V2_refused_meshes");
    version.write_meshes_to(&dir).expect("write the meshes");
    let meshes = dir.to_str().expect("a UTF-8 directory");
    let model = openarm(version, meshes).expect("the model builds");
    let strange = ChainGeometry::<1>::from_urdf(
        STRANGE_PARTS,
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
            .find(|part| part.rider().link == link)
            .unwrap_or_else(|| panic!("the strange chain carries {link}"));
        model
            .fingers(Side::Left, std::slice::from_ref(part))
            .err()
            .expect("a part of another robot is refused")
    };
    let named = |link: &str| link.to_string();
    assert_eq!(
        refusal("stranger"),
        FingerError::NoBody {
            link: named("stranger")
        }
    );
    assert_eq!(
        refusal("openarm_left_link3"),
        FingerError::NotAFinger {
            link: named("openarm_left_link3")
        }
    );
    assert_eq!(
        refusal("openarm_left_ee_link1"),
        FingerError::OtherLink {
            link: named("openarm_left_ee_link1"),
            model: 6,
            chain: 0
        }
    );
    let left = ChainGeometry::<7>::from_urdf(
        version.urdf(),
        meshes,
        &ChainSpec {
            base_link: Some(version.base_link(Side::Left)),
            tip_link: version.tcp_link(Side::Left),
            joints: JointSelection::PathOrder,
        },
    )
    .expect("the left arm fits");
    let first = left.carried()[0].rider().link.to_string();
    assert_eq!(
        model
            .fingers(Side::Right, left.carried())
            .err()
            .expect("the left arm's fingers are refused for the right arm"),
        FingerError::OtherArm {
            link: first,
            side: Side::Right
        }
    );
}
