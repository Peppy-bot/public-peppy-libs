//! Writes the table of the bundled models: each directory of `models/` is one model,
//! named by its model id, holding `<id>.urdf`, `<id>.srdf` and the collision meshes under
//! `meshes/`. The table embeds each model's files and carries the digest of them all,
//! computed here so that a build without the `meshes` feature holds it too.

use std::fmt::Write as _;
use std::path::Path;
use std::{env, fs};

#[path = "src/digest.rs"]
mod digest;

fn main() {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").expect("cargo sets CARGO_MANIFEST_DIR");
    let models_dir = Path::new(&manifest_dir).join("models");
    println!("cargo:rerun-if-changed={}", models_dir.display());

    let mut table = String::from("static MODELS: &[Model] = &[\n");
    for id in sorted_names(&models_dir) {
        let model_dir = models_dir.join(&id);
        let urdf = model_dir.join(format!("{id}.urdf"));
        let srdf = model_dir.join(format!("{id}.srdf"));
        let meshes_dir = model_dir.join("meshes");
        let mesh_names = sorted_names(&meshes_dir);
        let meshes: Vec<(String, Vec<u8>)> = mesh_names
            .iter()
            .map(|name| (name.clone(), read(&meshes_dir.join(name))))
            .collect();
        let digest = digest::digest_of(&read(&urdf), &read(&srdf), &meshes);

        writeln!(table, "    Model {{").unwrap();
        writeln!(table, "        id: {id:?},").unwrap();
        writeln!(
            table,
            "        directory: {:?},",
            model_dir.display().to_string()
        )
        .unwrap();
        writeln!(
            table,
            "        urdf: include_str!({:?}),",
            urdf.display().to_string()
        )
        .unwrap();
        writeln!(
            table,
            "        srdf: include_str!({:?}),",
            srdf.display().to_string()
        )
        .unwrap();
        writeln!(table, "        digest: {digest:?},").unwrap();
        writeln!(table, "        #[cfg(feature = \"meshes\")]").unwrap();
        writeln!(table, "        meshes: &[").unwrap();
        for name in &mesh_names {
            let path = meshes_dir.join(name).display().to_string();
            writeln!(table, "            ({name:?}, include_bytes!({path:?})),").unwrap();
        }
        writeln!(table, "        ],").unwrap();
        writeln!(table, "    }},").unwrap();
    }
    table.push_str("];\n");

    let out_dir = env::var("OUT_DIR").expect("cargo sets OUT_DIR");
    fs::write(Path::new(&out_dir).join("models.rs"), table).expect("the table is written");
}

/// The names of the entries of `dir`, sorted.
fn sorted_names(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = fs::read_dir(dir)
        .unwrap_or_else(|error| panic!("{}: {error}", dir.display()))
        .map(|entry| {
            let name = entry.expect("a directory entry").file_name();
            name.into_string().expect("a UTF-8 file name")
        })
        .collect();
    names.sort();
    names
}

fn read(path: &Path) -> Vec<u8> {
    fs::read(path).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
}
