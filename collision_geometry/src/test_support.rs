//! What the unit tests share: where a bundled model's collision meshes are.

/// The directory of the collision meshes of the robot_description model `model`.
pub(crate) fn bundled_meshes(model: &str) -> String {
    robot_description::model(model)
        .expect("robot_description bundles the model")
        .directory()
        .join("meshes")
        .to_str()
        .expect("the model's directory is UTF-8")
        .to_owned()
}
