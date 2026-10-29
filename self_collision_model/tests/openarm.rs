//! OpenArm's self-collision model, built from openarm_description alone.

use openarm_description::HardwareVersion;
use self_collision_model::openarm;

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
