//! The OpenArm torso's pieces, which `examples/openarm_torso_pieces.rs` writes: on each
//! OpenArm model of robot_description, the torso link's collision elements are the pieces,
//! one element per piece naming the piece's file, each committed file is the piece the
//! example writes now, and the pieces hold the stand's whole mesh.

use std::collections::BTreeSet;
use std::path::Path;

use collision_geometry::fit::{fit_fixed_link, verify_contains};
use collision_geometry::urdf_collision::UrdfCollisions;
use robot_description::mesh_name;
use robot_description::urdf_rs::Geometry;

#[path = "fixtures/openarm_torso.rs"]
mod openarm_torso;

use openarm_torso::{
    MODELS, PIECES, TORSO, model_meshes, piece_element, piece_file, piece_stls, stand,
};

#[test]
fn each_models_torso_elements_are_the_pieces_the_example_writes() {
    let pieces: BTreeSet<(String, String)> = PIECES
        .iter()
        .map(|(name, _)| (piece_element(name), piece_file(name)))
        .collect();
    for id in MODELS {
        let description = robot_description::model(id)
            .expect("robot_description bundles the model")
            .description();
        let torso = description
            .robot()
            .links
            .iter()
            .find(|link| link.name == TORSO)
            .unwrap_or_else(|| panic!("{id} has the link {TORSO}"));
        let elements: BTreeSet<(String, String)> = torso
            .collision
            .iter()
            .map(|collision| {
                let Geometry::Mesh { filename, .. } = &collision.geometry else {
                    panic!("{id}: each element of {TORSO} is a mesh");
                };
                (
                    collision.name.clone().unwrap_or_default(),
                    mesh_name(filename).to_string(),
                )
            })
            .collect();
        assert_eq!(
            elements.len(),
            torso.collision.len(),
            "{id}: each element once"
        );
        assert_eq!(elements, pieces, "{id}");
    }
}

// The clip is plain IEEE arithmetic and each value is rounded to f32 once, so the bytes the
// example writes are the same on every machine: each committed file is compared byte for
// byte.
#[test]
fn each_committed_piece_is_the_piece_the_example_writes() {
    let pieces = piece_stls().expect("the pieces cut");
    for id in MODELS {
        for (name, bytes) in &pieces {
            let path = Path::new(&model_meshes(id)).join(piece_file(name));
            let committed =
                std::fs::read(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
            assert!(
                committed == *bytes,
                "{} is not the piece {name} the example writes now: run `cargo run --example \
                 openarm_torso_pieces`",
                path.display()
            );
        }
    }
}

#[test]
fn each_models_torso_pieces_hold_the_stands_mesh() {
    let stand = stand().expect("the stand's mesh reads");
    for id in MODELS {
        let model = robot_description::model(id).expect("robot_description bundles the model");
        let collisions = UrdfCollisions::from_urdf(model.urdf()).expect("the URDF reads");
        let pieces = fit_fixed_link(&collisions, TORSO, &model_meshes(id)).expect("the torso fits");
        verify_contains(TORSO, &pieces, &stand).unwrap_or_else(|error| panic!("{id}: {error}"));
    }
}
