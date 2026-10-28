#!/usr/bin/env bash
# Writes distances.json next to this script: the distances of the self-collision model
# of public-peppy-libs at its merge base, a817e346, on the configurations capture.rs
# draws. self_collision_model/tests/merge_base_distances.rs holds the model of this
# tree to them. Run it from any directory of the repository:
#
#   self_collision_model/tests/fixtures/merge_base/capture.sh
#
# It checks the merge base out in a temporary worktree, and builds capture.rs in
# release against the crate there that self_collision_model was renamed from, named
# `reference`, with capture.lock, which the build may not change: the Cargo.lock of that
# crate at the merge base with the packages of the capture added and the edges of that
# crate's dev-dependencies, rand and serde_json, dropped. It writes
# distances.json in the temporary directory, and moves it next to this script once the
# capture has run to its end.
set -euo pipefail

merge_base=a817e346
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository=$(git -C "$here" rev-parse --show-toplevel)
work=$(mktemp -d)
trap 'git -C "$repository" worktree remove --force "$work/tree" 2>/dev/null; rm -rf "$work"' EXIT

git -C "$repository" worktree add --detach --quiet "$work/tree" "$merge_base"
manifest=$(git -C "$repository" log --follow --diff-filter=R --format= --name-status \
  -- self_collision_model/Cargo.toml | awk 'NR == 1 { print $2 }')
crate=$(dirname "$manifest")
package=$(sed -n 's/^name = "\(.*\)"$/\1/p' "$work/tree/$manifest" | head -n 1)

mkdir -p "$work/capture/src"
cp "$here/capture.rs" "$work/capture/src/main.rs"
cp "$here/capture.lock" "$work/capture/Cargo.lock"
cat > "$work/capture/Cargo.toml" <<TOML
[package]
name = "capture"
version = "0.1.0"
edition = "2024"
publish = false

[workspace]

[dependencies]
reference = { path = "$work/tree/$crate", package = "$package" }
openarm_description = { path = "$work/tree/openarm_description" }
rand = "0.10"
serde_json = "1"
TOML

cargo run --locked --release --quiet --manifest-path "$work/capture/Cargo.toml" \
  -- "$work/tree/openarm_description" > "$work/distances.json"
mv "$work/distances.json" "$here/distances.json"
echo "wrote $here/distances.json"
