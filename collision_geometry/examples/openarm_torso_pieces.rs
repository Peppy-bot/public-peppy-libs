//! Writes the collision pieces of the OpenArm torso: the stand's mesh
//! (`tests/assets/body_link0_symp.stl`, upstream's, the same in both generations) cut into
//! the slices of eight clip regions, one binary STL per slice, into the meshes of each
//! OpenArm model of robot_description, whose URDFs name the slices as the collision
//! elements of `openarm_body_link0`. Each slice is fitted as a piece of its own, so the
//! torso's eight pieces follow its column, gusset and head as closely as a link's piece
//! follows the link's mesh, and the grippers at rest stand clear of them.
//!
//! `cargo run --example openarm_torso_pieces`
//!
//! Feature extents measured from the torso mesh (mm, URDF root frame, which is the
//! torso's):
//!
//! ```text
//!  plate   z 0..8, full footprint x [-155, 95], y [-95, 95]
//!  flare   skirt necking y +/-85 (z 12.6) -> +/-30 (z 70), x +/-25
//!  gusset  y +/-30 web from the plate (x -153..-118 at z 8) up a straight
//!          hypotenuse to the column rear (x -30 at z 161..221)
//!  column  x/y +/-30 square (2 mm corner fillets), z 19..604
//!  head    z 603..773; flares to y +/-79 by z 625, x peaks [-85, 65] at
//!          z 695, tapers to about (+/-30, +/-44) at z 770
//! ```
//!
//! Where each bound sits (`tests/fixtures/openarm_torso.rs`):
//! - Adjacent regions overlap by at least 3 mm, so each ~1 mm patch of the large wall
//!   triangles that cross a cut lies inside a single piece, as `verify_contains` checks.
//! - Bounds sit about 1 mm off large flat mesh faces, so no piece holds a fragment of a
//!   face that lies along its bound: the gusset region starts at z = 9 mm, 1 mm above the
//!   plate's top face.
//! - The gusset region, cut on its own, fits a wedge along the hypotenuse, leaving the
//!   empty triangle above the diagonal outside every piece.
//! - The head is banded in z so each piece follows the taper.

use std::path::Path;

#[path = "../tests/fixtures/openarm_torso.rs"]
mod openarm_torso;

use openarm_torso::{MODELS, model_meshes, piece_file, piece_stls};

fn main() -> Result<(), String> {
    for (name, bytes) in piece_stls()? {
        for model in MODELS {
            let path = Path::new(&model_meshes(model)).join(piece_file(name));
            std::fs::write(&path, &bytes)
                .map_err(|error| format!("{}: {error}", path.display()))?;
            println!("{}", path.display());
        }
    }
    Ok(())
}
