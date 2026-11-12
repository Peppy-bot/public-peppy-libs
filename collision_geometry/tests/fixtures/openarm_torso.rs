// The OpenArm torso's collision pieces, which `examples/openarm_torso_pieces.rs` writes and
// `tests/openarm_torso_pieces.rs` holds to the stand's mesh, to the OpenArm URDFs and to the
// committed files: the stand's mesh, the clip region of each piece, each piece's STL and the
// files the pieces go to. Each includer reads some of them.
#![allow(dead_code)]

use collision_geometry::clip::ClipRegion;
use collision_geometry::nalgebra::Point3;
use collision_geometry::stl::load_stl;

/// The stand's mesh, upstream's, the same in both OpenArm generations: the torso link's
/// collision mesh as Enactic's URDFs name it.
pub const STAND_MESH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/assets/body_link0_symp.stl"
);

/// The scale of the stand's mesh, millimeters, as the URDFs' elements give it.
pub const MESH_SCALE: f64 = 0.001;

/// The torso link, whose collision elements the pieces are.
pub const TORSO: &str = "openarm_body_link0";

/// The OpenArm models of robot_description whose torso the pieces are.
pub const MODELS: [&str; 2] = ["openarm_v1", "openarm_v2"];

const INF: f64 = f64::INFINITY;

/// Each piece's name and its clip region's `[min, max]` corners, meters, in the torso's
/// frame.
pub const PIECES: [(&str, [[f64; 3]; 2]); 8] = [
    // The full-footprint base, everything below the flare skirt.
    ("plate", [[-INF, -INF, -INF], [INF, INF, 0.012]]),
    // Skirt and column bottom, fenced off the gusset at x -0.0335.
    ("flare", [[-0.0335, -0.086, 0.006], [0.033, 0.086, 0.083]]),
    // The diagonal web behind the column; fits a tight wedge.
    ("gusset", [[-INF, -0.031, 0.009], [-0.0305, 0.031, 0.226]]),
    // The square shaft, fenced off the gusset at x -0.0335.
    ("column", [[-0.0335, -INF, 0.078], [INF, INF, 0.6025]]),
    // The head's bands: collar and skirt, waist and mid, wide band, top taper.
    ("band_collar", [[-INF, -INF, 0.5995], [INF, INF, 0.638]]),
    ("band_waist", [[-INF, -INF, 0.633], [INF, INF, 0.688]]),
    ("band_wide", [[-INF, -INF, 0.682], [INF, INF, 0.719]]),
    ("band_top", [[-INF, -INF, 0.713], [INF, INF, INF]]),
];

/// The stand's mesh, meters: each three points one triangle.
pub fn stand() -> Result<Vec<Point3<f64>>, String> {
    Ok(load_stl(STAND_MESH)?
        .into_iter()
        .map(|point| point * MESH_SCALE)
        .collect())
}

/// Each piece's name and its binary STL, in the order of [`PIECES`]: the stand's mesh
/// clipped to the piece's region.
pub fn piece_stls() -> Result<Vec<(&'static str, Vec<u8>)>, String> {
    let soup = stand()?;
    PIECES
        .iter()
        .map(|&(name, [min, max])| {
            let region = ClipRegion::new(Point3::from(min), Point3::from(max))?;
            Ok((name, stl_bytes(name, &region.slice(&soup))))
        })
        .collect()
}

/// The binary STL of `soup` (each three points one triangle, meters), its vertices in the
/// mesh's millimeters, each facet's normal from its corners' winding, zero for a facet of
/// no area.
fn stl_bytes(name: &str, soup: &[Point3<f64>]) -> Vec<u8> {
    let (triangles, _) = soup.as_chunks::<3>();
    let mut header = [b' '; 80];
    let title = format!("openarm torso piece {name}");
    header[..title.len()].copy_from_slice(title.as_bytes());
    let count = u32::try_from(triangles.len()).expect("a slice of the torso fits an STL");
    let facets = triangles.iter().flat_map(|[a, b, c]| {
        let normal = (b - a)
            .cross(&(c - a))
            .try_normalize(0.0)
            .unwrap_or_default();
        let corners = [*a, *b, *c].map(|corner| corner / MESH_SCALE);
        [normal.x, normal.y, normal.z]
            .into_iter()
            .chain(
                corners
                    .into_iter()
                    .flat_map(|corner| [corner.x, corner.y, corner.z]),
            )
            .flat_map(|value| (value as f32).to_le_bytes())
            .chain([0, 0])
            .collect::<Vec<u8>>()
    });
    header
        .into_iter()
        .chain(count.to_le_bytes())
        .chain(facets)
        .collect()
}

/// The file of the piece `name` among a model's meshes.
pub fn piece_file(name: &str) -> String {
    format!("body_link0_{name}.stl")
}

/// The name of the torso's collision element of the piece `name`.
pub fn piece_element(name: &str) -> String {
    format!("{TORSO}_{name}_collision")
}

/// The directory of the meshes of the robot_description model `model`.
pub fn model_meshes(model: &str) -> String {
    robot_description::model(model)
        .expect("robot_description bundles the model")
        .directory()
        .join("meshes")
        .to_str()
        .expect("the model's directory is UTF-8")
        .to_owned()
}
