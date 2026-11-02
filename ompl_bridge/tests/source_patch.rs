//! The patch the build applies to OMPL's extracted sources: it changes each of its files
//! in place, line for line as its hunks say; a hunk that does not match its file, a file
//! the patch creates, and a patch that does not parse are refused, with a reason that
//! names what failed, and a refused patch changes no file.

#[path = "../source_patch.rs"]
mod source_patch;

use std::fs;
use std::path::{Path, PathBuf};

use source_patch::apply;

/// The header of the tree here, before the patch.
const HEADER: &str = "namespace ompl\n{\n    void setSeed();\n}\n";

/// The source of the tree here, before the patch.
const SOURCE: &str = "void setSeed()\n{\n    seed();\n}\n";

/// A patch of both files of the tree here, in the shape of the crate's patch.
const PATCH: &str = "\
--- a/src/util/Seeds.h
+++ b/src/util/Seeds.h
@@ -1,4 +1,5 @@
 namespace ompl
 {
     void setSeed();
+    void setThreadSeed();
 }
--- a/src/util/src/Seeds.cpp
+++ b/src/util/src/Seeds.cpp
@@ -2,3 +2,7 @@
 {
     seed();
 }
+void setThreadSeed()
+{
+    threadSeed();
+}
";

/// The header once the patch applies.
const PATCHED_HEADER: &str =
    "namespace ompl\n{\n    void setSeed();\n    void setThreadSeed();\n}\n";

/// The source once the patch applies.
const PATCHED_SOURCE: &str =
    "void setSeed()\n{\n    seed();\n}\nvoid setThreadSeed()\n{\n    threadSeed();\n}\n";

/// A tree of this test's own, `name`, under cargo's scratch directory for tests, holding
/// `header` and `source` at the paths the patch names.
fn tree(name: &str, header: &str, source: &str) -> PathBuf {
    let tree = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name);
    if tree.exists() {
        fs::remove_dir_all(&tree).expect("the removal of an earlier run's tree");
    }
    fs::create_dir_all(tree.join("src/util/src")).expect("the test's tree");
    fs::write(tree.join("src/util/Seeds.h"), header).expect("the header");
    fs::write(tree.join("src/util/src/Seeds.cpp"), source).expect("the source");
    tree
}

/// The header and the source of `tree`.
fn files(tree: &Path) -> (String, String) {
    (
        fs::read_to_string(tree.join("src/util/Seeds.h")).expect("the header"),
        fs::read_to_string(tree.join("src/util/src/Seeds.cpp")).expect("the source"),
    )
}

#[test]
fn a_patch_changes_each_of_its_files_in_place() {
    let tree = tree("a_patch_changes_each_of_its_files_in_place", HEADER, SOURCE);
    apply(&tree, PATCH).expect("the patch applies");
    assert_eq!(
        files(&tree),
        (PATCHED_HEADER.to_string(), PATCHED_SOURCE.to_string())
    );
}

// The source differs from the patch's context in its second file, so the patch changes
// neither file, and says which file and which hunk did not apply.
#[test]
fn a_hunk_that_does_not_match_its_file_is_refused_and_changes_no_file() {
    let name = "a_hunk_that_does_not_match_its_file_is_refused_and_changes_no_file";
    let other_source = "void setSeed()\n{\n    reseed();\n}\n";
    let tree = tree(name, HEADER, other_source);
    let refusal = apply(&tree, PATCH).expect_err("the patch does not apply");
    assert_eq!(
        refusal,
        format!(
            "the patch does not apply to {}: error applying hunk #1",
            tree.join("src/util/src/Seeds.cpp").display()
        )
    );
    assert_eq!(files(&tree), (HEADER.to_string(), other_source.to_string()));
}

#[test]
fn a_patch_that_creates_a_file_is_refused() {
    let tree = tree("a_patch_that_creates_a_file_is_refused", HEADER, SOURCE);
    let creating = "--- /dev/null\n+++ b/src/util/New.h\n@@ -0,0 +1 @@\n+new\n";
    let refusal = apply(&tree, creating).expect_err("the patch is refused");
    assert!(
        refusal.starts_with("the patch holds Create(") && refusal.contains("src/util/New.h"),
        "{refusal}"
    );
    assert!(!tree.join("src/util/New.h").exists());
}

#[test]
fn a_patch_that_does_not_parse_is_refused() {
    let tree = tree("a_patch_that_does_not_parse_is_refused", HEADER, SOURCE);
    let refusal = apply(&tree, "a patch of nothing\n").expect_err("the patch is refused");
    assert!(
        refusal.starts_with("the patch does not parse: "),
        "{refusal}"
    );
    assert_eq!(files(&tree), (HEADER.to_string(), SOURCE.to_string()));
}
