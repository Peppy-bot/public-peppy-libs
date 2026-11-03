//! A module of `build.rs`: a patch of OMPL's sources, a unified diff whose every file
//! changes in place under the release's tree, its paths named as `a/<path>` and
//! `b/<path>`. A hunk applies where its context and its removed lines stand in the file,
//! line for line; a patch that does not apply changes no file.

use std::fs;
use std::path::{Path, PathBuf};

use diffy::patch_set::{FileOperation, ParseOptions, PatchKind, PatchSet};

/// Applies `patch` to the files under `tree`, or answers why it does not apply.
pub fn apply(tree: &Path, patch: &str) -> Result<(), String> {
    let patched = PatchSet::parse(patch, ParseOptions::unidiff())
        .map(|file| {
            let file = file.map_err(|error| format!("the patch does not parse: {error}"))?;
            let path = tree.join(changed_in_place(file.operation())?);
            let PatchKind::Text(text) = file.patch() else {
                return Err(format!(
                    "the patch changes {} as a binary file; it changes text alone",
                    path.display()
                ));
            };
            let original = fs::read_to_string(&path)
                .map_err(|error| format!("{} cannot be read: {error}", path.display()))?;
            let changed = diffy::apply(&original, text).map_err(|error| {
                format!("the patch does not apply to {}: {error}", path.display())
            })?;
            Ok((path, changed))
        })
        .collect::<Result<Vec<(PathBuf, String)>, String>>()?;
    for (path, changed) in patched {
        fs::write(&path, changed)
            .map_err(|error| format!("{} cannot be written: {error}", path.display()))?;
    }
    Ok(())
}

/// The path, under the tree, of the file that `operation` changes in place.
fn changed_in_place(operation: &FileOperation<'_, str>) -> Result<PathBuf, String> {
    match operation.strip_prefix(1) {
        FileOperation::Modify { original, modified } if original == modified => {
            Ok(PathBuf::from(original.as_ref()))
        }
        other => Err(format!(
            "the patch holds {other:?}; each of its files changes in place, from a/<path> to \
             b/<path>"
        )),
    }
}
