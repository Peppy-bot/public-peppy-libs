//! The distal payload: everything past the tip, lumped into one rigid body.
//!
//! Absolute values, hand-computed from the URDF, so this fails if the lumping
//! silently drops a body or places one in the wrong frame - which a
//! self-consistency check would not catch.

use chain_kinematics::{Chain, ChainSpec, JointSelection, Payload, Tree};

const OPENARM: &str = include_str!("fixtures/openarm_7dof.urdf");

#[test]
fn lumps_the_gripper_body_and_fingers_in_the_tip_frame() {
    let robot = urdf_rs::read_from_string(OPENARM).expect("parse OpenArm");
    let tree = Tree::from_robot(&robot).expect("build tree");
    let tip = tree.link_index("openarm_left_link7").expect("link7");

    let p = Payload::from_distal(&tree, tip);

    // Three bodies hang off link7, read straight from the URDF:
    //   hand          0.127 kg           at (0.000001, 0, 0.101951), fixed at the tip
    //   right finger  0.03602545343277134 kg at (0.0064528, -0.01702, 0.0219685)
    //   left finger   same mass              at (0.0064528, +0.01702, 0.0219685)
    // Both fingers sit on prismatic joints, frozen at zero, offset (0, 0, 0.1025)
    // from the hand - which is why their z in the tip frame is 0.1025 + 0.0219685.
    // The tcp frame is massless.
    const HAND: f64 = 0.127;
    const FINGER: f64 = 0.036_025_453_432_771_34;
    assert!(
        (p.mass - (HAND + 2.0 * FINGER)).abs() < 1e-12,
        "lumped mass = {} kg, expected the hand and two fingers",
        p.mass
    );
    // Mass-weighted average of the three, in the tip frame.
    assert!(
        (p.com.x - 0.002_336_372_635_248_108_6).abs() < 1e-12,
        "com.x = {}",
        p.com.x
    );
    assert!(
        p.com.y.abs() < 1e-12,
        "com.y = {} (the fingers mirror in y, so it must vanish)",
        p.com.y
    );
    assert!(
        (p.com.z - 0.110_101_710_393_099_5).abs() < 1e-12,
        "com.z = {}",
        p.com.z
    );
}

#[test]
fn the_payload_reaches_the_last_segment_of_a_built_chain() {
    // Lumping is only useful if a dynamics layer sees it, which means it has to
    // land in the last segment's inertial rather than being computed and dropped.
    let robot = urdf_rs::read_from_string(OPENARM).expect("parse OpenArm");
    let spec = |tip| ChainSpec {
        base_link: Some("openarm_left_link0"),
        tip_link: tip,
        joints: JointSelection::PathOrder,
    };
    let chain = Chain::<7>::from_urdf(&robot, &spec("openarm_left_link7")).expect("build chain");
    let q = [0.0; 7];

    // link7's own inertial mass, read straight from the URDF, versus what the
    // chain reports for that segment: the difference is the two fingers.
    let tree = Tree::from_robot(&robot).expect("build tree");
    let bare = tree
        .link(tree.link_index("openarm_left_link7").unwrap())
        .mass;
    let carried = chain.at(&q).mass(6);
    // The difference is the hand and its two fingers.
    assert!(
        (carried - bare - 0.199_050_906_865_542_66).abs() < 1e-12,
        "last segment carries {carried} kg against a bare link of {bare} kg"
    );
}
