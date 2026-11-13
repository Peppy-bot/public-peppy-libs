//! The description of each robot model of this repository, by its model id: the
//! directories of `models/`, embedded by the build script. A model's collision meshes go
//! where a builder of the URDF's collision geometry reads them by [`write_meshes`].

use std::path::Path;

use crate::RobotDescription;

/// One bundled robot model: its URDF, its SRDF and, behind the `meshes` feature, its
/// collision meshes.
#[derive(Debug)]
pub struct Model {
    id: &'static str,
    directory: &'static str,
    urdf: &'static str,
    srdf: &'static str,
    digest: &'static str,
    #[cfg(feature = "meshes")]
    meshes: &'static [(&'static str, &'static [u8])],
}

include!(concat!(env!("OUT_DIR"), "/models.rs"));

/// A model id of no bundled model.
#[derive(Debug, PartialEq, thiserror::Error)]
#[error("no robot model is named {id:?}; the models are {}", model_ids())]
pub struct UnknownModel {
    id: String,
}

fn model_ids() -> String {
    let ids: Vec<String> = models().map(|model| format!("{:?}", model.id)).collect();
    ids.join(", ")
}

/// The bundled models, in the order of their ids.
pub fn models() -> impl Iterator<Item = &'static Model> {
    MODELS.iter()
}

/// The bundled models that have an arm, in the order of their ids: the ones a node of
/// arms drives ([`crate::NodeArms`]) and a planner plans for.
pub fn models_with_arms() -> impl Iterator<Item = &'static Model> {
    models().filter(|model| !model.description().arms().is_empty())
}

/// The bundled model of the id `id`, as robot_identity gives it.
pub fn model(id: &str) -> Result<&'static Model, UnknownModel> {
    models()
        .find(|model| model.id == id)
        .ok_or_else(|| UnknownModel { id: id.to_string() })
}

impl Model {
    /// The model's id.
    pub fn id(&self) -> &'static str {
        self.id
    }

    /// The directory of `models/` the build bundled the model from, on the machine that
    /// built this crate: `<id>.urdf`, `<id>.srdf` and the collision meshes under
    /// `meshes/`, for a reader there that takes the files from disk.
    pub fn directory(&self) -> &'static Path {
        Path::new(self.directory)
    }

    /// The URDF's text.
    pub fn urdf(&self) -> &'static str {
        self.urdf
    }

    /// The SRDF's text.
    pub fn srdf(&self) -> &'static str {
        self.srdf
    }

    /// The SHA-256 digest, in lowercase hex, of the model's URDF, SRDF and collision
    /// meshes. Two builds that hold the same digest hold the same description.
    pub fn digest(&self) -> &'static str {
        self.digest
    }

    /// The model's description. Panics only if a bundled file is malformed, which this
    /// crate's tests rule out.
    pub fn description(&self) -> RobotDescription {
        RobotDescription::parse(self.urdf, self.srdf)
            .unwrap_or_else(|error| panic!("the bundled description of {}: {error}", self.id))
    }
}

#[cfg(feature = "meshes")]
impl Model {
    /// Each collision mesh the URDF names, by the name of its file, in the order of the
    /// names.
    pub fn meshes(&self) -> &'static [(&'static str, &'static [u8])] {
        self.meshes
    }

    /// Writes the collision meshes into `dir`, as [`write_meshes`] writes them.
    pub fn write_meshes_to(&self, dir: &Path) -> std::io::Result<()> {
        write_meshes(dir, self.meshes.iter().copied())
    }
}

/// Writes the collision meshes of a description, each its file's name and its bytes, into
/// `dir`, created if absent, under those names, where a builder of the URDF's collision
/// geometry reads them. A name that is not one file's name (empty, `.`, `..`, or holding
/// a path separator) is refused before anything is written.
pub fn write_meshes<'a>(
    dir: &Path,
    meshes: impl IntoIterator<Item = (&'a str, &'a [u8])>,
) -> std::io::Result<()> {
    let meshes: Vec<(&str, &[u8])> = meshes.into_iter().collect();
    if let Some((name, _)) = meshes.iter().find(|(name, _)| !is_a_file_name(name)) {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!("the mesh name {name:?} is not one file's name"),
        ));
    }
    std::fs::create_dir_all(dir)?;
    meshes
        .into_iter()
        .try_for_each(|(name, bytes)| std::fs::write(dir.join(name), bytes))
}

/// Whether `name` names one file of a directory: neither empty, `.` nor `..`, and holding
/// no path separator.
fn is_a_file_name(name: &str) -> bool {
    !matches!(name, "" | "." | "..") && !name.contains(['/', '\\'])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Finger;

    // Each mesh lands under its name; a name that is not one file's name writes nothing.
    #[test]
    fn meshes_land_under_their_names_and_a_path_is_refused() {
        let dir =
            std::env::temp_dir().join(format!("robot_description_meshes_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        write_meshes(&dir, [("a.stl", &b"a"[..]), ("b.stl", &b"bb"[..])])
            .expect("two meshes write");
        assert_eq!(std::fs::read(dir.join("b.stl")).expect("b lands"), b"bb");
        for name in [
            "",
            ".",
            "..",
            "../escaped.stl",
            "sub/inner.stl",
            "back\\slash.stl",
        ] {
            let refused = write_meshes(&dir.join("refused"), [(name, &b"x"[..])])
                .expect_err("a path is refused");
            assert_eq!(refused.kind(), std::io::ErrorKind::InvalidInput, "{name:?}");
        }
        assert!(
            !dir.join("refused").exists(),
            "nothing is written for a refused name"
        );
        std::fs::remove_dir_all(&dir).expect("the directory goes");
    }

    // Each model's directory holds its URDF and its SRDF, the files the build bundled.
    #[test]
    fn each_models_directory_holds_its_files() {
        for bundled in models() {
            let id = bundled.id();
            let read = |extension: &str| {
                std::fs::read_to_string(bundled.directory().join(format!("{id}.{extension}")))
                    .unwrap_or_else(|error| panic!("{id}.{extension}: {error}"))
            };
            assert_eq!(read("urdf"), bundled.urdf(), "{id}");
            assert_eq!(read("srdf"), bundled.srdf(), "{id}");
        }
    }

    #[test]
    fn a_model_is_found_by_its_id_and_an_unknown_id_names_the_models() {
        for bundled in models() {
            assert_eq!(
                model(bundled.id()).expect("a bundled id").id(),
                bundled.id()
            );
        }
        let ids: Vec<String> = models().map(|model| format!("{:?}", model.id())).collect();
        assert_eq!(
            model("no_such_robot").unwrap_err().to_string(),
            format!(
                "no robot model is named \"no_such_robot\"; the models are {}",
                ids.join(", ")
            )
        );
    }

    // The models with arms are every bundled model but the botshare base, which has none.
    #[test]
    fn the_models_with_arms_are_every_model_but_the_armless_base() {
        let all: Vec<&str> = models().map(Model::id).collect();
        let with_arms: Vec<&str> = models_with_arms().map(Model::id).collect();
        assert_eq!(all, ["botshare_base", "openarm_v1", "openarm_v2", "so101"]);
        assert_eq!(with_arms, ["openarm_v1", "openarm_v2", "so101"]);
    }

    #[test]
    fn every_arm_has_its_ready_home_and_start_postures_and_gripper_ends_at_joint_limits() {
        for bundled in models() {
            let description = bundled.description();
            for arm in description.arms() {
                assert!(
                    !arm.joints.is_empty(),
                    "{} {}: joints",
                    bundled.id(),
                    arm.name
                );
                for posture in [
                    crate::states::READY,
                    crate::states::HOME,
                    crate::states::START,
                ] {
                    let values = description
                        .state(&arm.name, posture)
                        .unwrap_or_else(|unknown| panic!("{}: {unknown}", bundled.id()));
                    assert_eq!(values.len(), arm.joints.len());
                }
                for finger in &arm.gripper.fingers {
                    let limits = description
                        .position_limits(&finger.joint)
                        .expect("a finger's joint");
                    let (closed, open) = (finger.span.closed(), finger.span.open());
                    assert!(
                        limits.contains(&closed) && limits.contains(&open),
                        "{} {}: {} shuts at {closed} and opens at {open}, its limits {limits:?}",
                        bundled.id(),
                        arm.name,
                        finger.joint,
                    );
                }
            }
        }
    }

    /// Each link's pose in the frame of the URDF's root, at `positions` of the joints they
    /// name and every other joint at zero.
    fn link_poses(
        description: &RobotDescription,
        positions: &[(&str, f64)],
    ) -> std::collections::HashMap<String, nalgebra::Isometry3<f64>> {
        let tree = chain_kinematics::Tree::from_robot(description.robot())
            .expect("a bundled URDF is a tree");
        let position = |joint: usize| {
            positions
                .iter()
                .find(|(name, _)| *name == tree.joint(joint).name)
                .map_or(0.0, |(_, q)| *q)
        };
        let root = tree.root();
        std::iter::once((
            tree.link(root).name.clone(),
            nalgebra::Isometry3::identity(),
        ))
        .chain(
            tree.subtree_from(root, &position)
                .into_iter()
                .map(|(link, pose)| (tree.link(link).name.clone(), pose)),
        )
        .collect()
    }

    /// The links the joint `joint` moves: its child and every link below it.
    fn moved_by<'d>(description: &'d RobotDescription, joint: &str) -> Vec<&'d str> {
        let joints = &description.robot().joints;
        let child = &joints
            .iter()
            .find(|candidate| candidate.name == joint)
            .expect("a joint of the URDF")
            .child
            .link;
        let mut moved = vec![child.as_str()];
        let mut at = 0;
        while let Some(&link) = moved.get(at) {
            moved.extend(
                joints
                    .iter()
                    .filter(|below| below.parent.link == link)
                    .map(|below| below.child.link.as_str()),
            );
            at += 1;
        }
        moved
    }

    // Each grasp frame's +z points from the grasp point away from the hand: every link of
    // the gripper that has collision geometry, the fingers' among them, has its origin at
    // -z of the grasp frame, the fingers shut and open.
    #[test]
    fn every_grippers_links_lie_at_minus_z_of_its_grasp_frame() {
        for bundled in models() {
            let description = bundled.description();
            for arm in description.arms() {
                let last = arm.joints.last().expect("an arm moves");
                let gripper: Vec<&str> = moved_by(&description, last)
                    .into_iter()
                    .filter(|link| {
                        description.robot().links.iter().any(|candidate| {
                            candidate.name == *link && !candidate.collision.is_empty()
                        })
                    })
                    .collect();
                assert!(
                    arm.gripper
                        .fingers
                        .iter()
                        .all(|finger| gripper.contains(&finger.link.as_str())),
                    "{} {}: the fingers ride the last joint",
                    bundled.id(),
                    arm.name
                );
                let openings: [fn(&Finger) -> f64; 2] =
                    [|finger| finger.span.closed(), |finger| finger.span.open()];
                for opening in openings {
                    let fingers: Vec<(&str, f64)> = arm
                        .gripper
                        .fingers
                        .iter()
                        .map(|finger| (finger.joint.as_str(), opening(finger)))
                        .collect();
                    let poses = link_poses(&description, &fingers);
                    let grasp = poses[&arm.grasp_link].inverse();
                    for link in &gripper {
                        let origin = grasp * poses[*link] * nalgebra::Point3::origin();
                        assert!(
                            origin.z < 0.0,
                            "{} {}: {link} stands at {origin} in the grasp frame",
                            bundled.id(),
                            arm.name
                        );
                    }
                }
            }
        }
    }

    #[cfg(feature = "meshes")]
    #[test]
    fn the_digest_is_of_the_embedded_files() {
        for bundled in models() {
            let meshes: Vec<(String, Vec<u8>)> = bundled
                .meshes()
                .iter()
                .map(|(name, bytes)| (name.to_string(), bytes.to_vec()))
                .collect();
            assert_eq!(
                bundled.digest(),
                crate::digest::digest_of(
                    bundled.urdf().as_bytes(),
                    bundled.srdf().as_bytes(),
                    &meshes
                ),
                "{}",
                bundled.id()
            );
        }
    }

    #[cfg(feature = "meshes")]
    #[test]
    fn the_bundled_meshes_are_the_collision_meshes_the_urdf_names() {
        use std::collections::BTreeSet;

        use crate::description::mesh_name;
        for bundled in models() {
            let description = bundled.description();
            let named: BTreeSet<&str> = description
                .robot()
                .links
                .iter()
                .flat_map(|link| &link.collision)
                .filter_map(|collision| match &collision.geometry {
                    urdf_rs::Geometry::Mesh { filename, .. } => Some(mesh_name(filename)),
                    _ => None,
                })
                .collect();
            let held: BTreeSet<&str> = bundled.meshes().iter().map(|(name, _)| *name).collect();
            assert_eq!(held, named, "{}", bundled.id());
        }
    }
}
