use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

use crate::names::{NameTag, Sha256};

/// The versions a run used, which a bundle records and the workflow
/// director compares with a stack: the sha256 of each contract document, the
/// peppy version of the stack (`unknown` for a build that is not a release),
/// and each node by the sha256 of its pinned manifest and the commit its
/// daemon built.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Lock {
    pub contracts: BTreeMap<NameTag, Sha256>,
    pub peppy: String,
    pub nodes: BTreeMap<NameTag, LockedNode>,
}

/// A node of a lock. A node from a local directory, or one that a run used
/// at two commits, has no commit.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LockedNode {
    pub sha256: Sha256,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub commit: Option<String>,
}

/// The peppy version of a build that is not a release.
pub const UNKNOWN_PEPPY: &str = "unknown";
