use super::types::{DependsOn, Execution, InterfaceKind, Interfaces, Manifest, NodeConfig};
use crate::{
    consts::normalize_tag,
    error::{ParsingError, Result},
    parsing::read_non_empty_file,
};
use std::collections::{HashMap, HashSet};
use std::path::Path;

/// Validates the manifest's link namespace and implements claims:
///
/// - Rejects duplicate `link_id`s across the combined
///   nodes/contracts/pairings/implements set. `link_id` is the shared
///   identity for all four: consumed topics/services/actions resolve their
///   producer by `link_id` alone, produced entries resolve their implements
///   slot the same way, and pairing slots are addressed by `link_id` in
///   `--pair`/`pairings:`, so a collision either silently overwrites a
///   resolution or binds the wrong slot.
/// - Rejects pairing link_ids that are not wire-safe segments: unlike
///   node/contract/implements link_ids (local resolution names), a pairing
///   slot link_id is stamped verbatim into the producer-side link_id segment
///   of every publish keyexpr.
/// - Rejects duplicate `(name, tag)` pairs in `manifest.implements`
///   (including tag-sanitization collisions like `v1` vs `v-1`): wire keys
///   embed only `contract/{name}/{tag}` plus the `_` sentinel link slot, so
///   two instances of one contract in one node would collide on the wire.
fn validate_manifest_links(manifest: &Manifest) -> Result<()> {
    let mut seen_link_ids: HashSet<&str> = HashSet::new();
    let implements = manifest.implements.iter().map(|e| e.link_id.as_str());
    let depends = manifest.depends_on.iter().flat_map(|d| {
        d.nodes
            .iter()
            .map(|n| n.link_id.as_str())
            .chain(d.contracts.iter().map(|c| c.link_id.as_str()))
            .chain(d.pairings.iter().map(|p| p.link_id.as_str()))
    });
    for link_id in depends.chain(implements) {
        if !seen_link_ids.insert(link_id) {
            return Err(ParsingError::DuplicateLinkId(link_id.to_owned()).into());
        }
    }

    if let Some(depends_on) = &manifest.depends_on {
        for pairing in &depends_on.pairings {
            if !is_wire_safe_link_id(&pairing.link_id) {
                return Err(ParsingError::PairingSentinelLinkId(pairing.link_id.clone()).into());
            }
        }
    }

    let mut seen_contracts: HashMap<(&str, String), &str> = HashMap::new();
    for entry in &manifest.implements {
        let key = (entry.name.as_str(), normalize_tag(&entry.tag));
        if let Some(prev_tag) = seen_contracts.insert(key, entry.tag.as_str()) {
            if prev_tag == entry.tag {
                return Err(ParsingError::DuplicateImplementsContract {
                    name: entry.name.as_str().to_owned(),
                    tag: entry.tag.clone(),
                }
                .into());
            }
            return Err(ParsingError::ImplementsTagSanitizationCollision {
                name: entry.name.as_str().to_owned(),
                tag_a: prev_tag.to_owned(),
                tag_b: entry.tag.clone(),
            }
            .into());
        }
    }
    Ok(())
}

/// Validates the link_id direction rules and duplicate keys of the
/// produced/consumed interface entries:
///
/// - A produced (emits/exposes) entry's `link_id` must name a
///   `manifest.implements` slot — naming a `depends_on` slot or nothing at
///   all gets a dedicated error each.
/// - A consumed entry's `link_id` must not name an implements slot (those
///   are produced, not consumed); resolution against `depends_on` stays in
///   `validate_dependency_specs`, which runs where dependencies resolve.
/// - Native entries are unique by `name` per section; contract-backed
///   entries are unique by `(link_id, name)` per section. A native and a
///   contract-backed entry may share a name (they are namespaced apart in
///   modules, schema keys, and wire keys).
fn validate_interfaces(manifest: &Manifest, interfaces: &Interfaces) -> Result<()> {
    const KINDS: [(InterfaceKind, &str); 3] = [
        (InterfaceKind::Topic, super::types::EmittedTopic::SECTION),
        (
            InterfaceKind::Service,
            super::types::ExposedService::SECTION,
        ),
        (InterfaceKind::Action, super::types::ExposedAction::SECTION),
    ];

    let implements_link_ids: HashSet<&str> = manifest
        .implements
        .iter()
        .map(|e| e.link_id.as_str())
        .collect();

    for (kind, section) in KINDS {
        validate_produced_section(
            section,
            interfaces.produced(kind),
            &implements_link_ids,
            manifest.depends_on.as_ref(),
        )?;
    }

    for (kind, _) in KINDS {
        for (link_id, _) in interfaces.consumed(kind) {
            if implements_link_ids.contains(link_id) {
                return Err(ParsingError::ConsumedItemReferencesImplementsLinkId {
                    link_id: link_id.to_owned(),
                }
                .into());
            }
        }
    }
    Ok(())
}

/// Which `depends_on` list (if any) declares `link_id`. Only consulted on
/// the error path, so a linear scan of the (tiny) lists beats prebuilding
/// a lookup map that valid configs never read.
fn depends_list_containing(depends_on: Option<&DependsOn>, link_id: &str) -> Option<&'static str> {
    let d = depends_on?;
    if d.nodes.iter().any(|n| n.link_id == link_id) {
        return Some("nodes");
    }
    if d.contracts.iter().any(|c| c.link_id == link_id) {
        return Some("contracts");
    }
    if d.pairings.iter().any(|p| p.link_id == link_id) {
        return Some("pairings");
    }
    None
}

fn validate_produced_section<'a>(
    section: &'static str,
    entries: impl Iterator<Item = (Option<&'a str>, &'a str)>,
    implements_link_ids: &HashSet<&str>,
    depends_on: Option<&DependsOn>,
) -> Result<()> {
    let mut seen_native: HashSet<&str> = HashSet::new();
    let mut seen_contract: HashSet<(&str, &str)> = HashSet::new();
    for (link_id, name) in entries {
        match link_id {
            Some(link_id) => {
                if !implements_link_ids.contains(link_id) {
                    if let Some(found_in) = depends_list_containing(depends_on, link_id) {
                        return Err(ParsingError::EmitsLinkIdNotImplements {
                            section: section.to_owned(),
                            link_id: link_id.to_owned(),
                            found_in: found_in.to_owned(),
                        }
                        .into());
                    }
                    return Err(ParsingError::UndeclaredEmitsLinkId {
                        section: section.to_owned(),
                        link_id: link_id.to_owned(),
                    }
                    .into());
                }
                if !seen_contract.insert((link_id, name)) {
                    return Err(ParsingError::DuplicateInterfaceEntry {
                        section: section.to_owned(),
                        key: format!("{link_id}:{name}"),
                    }
                    .into());
                }
            }
            None => {
                if !seen_native.insert(name) {
                    return Err(ParsingError::DuplicateInterfaceEntry {
                        section: section.to_owned(),
                        key: name.to_owned(),
                    }
                    .into());
                }
            }
        }
    }
    Ok(())
}

/// A pairing slot link_id travels the wire as a keyexpr segment, so it must
/// obey the segment rules from `pmi::wire::Segment` (which this crate cannot
/// name — pmi depends on us): no `/`, no `@`, and not one of the reserved
/// sentinels (`*`, `**`, and the default-link_id sentinel `_`). Emptiness and
/// all-punctuation values are already rejected by the field deserializer.
fn is_wire_safe_link_id(link_id: &str) -> bool {
    !link_id.contains('/')
        && !link_id.contains('@')
        && !matches!(link_id, "*" | "**")
        && link_id != crate::consts::DEFAULT_LINK_ID_SENTINEL
}

/// Validates execution constraints.
fn validate_execution(execution: &Execution) -> Result<()> {
    if let Some(cmds) = &execution.run_cmd
        && cmds.is_empty()
    {
        return Err(ParsingError::EmptyRunCmd.into());
    }

    // `run_cmd` and `container` are mutually exclusive; exactly one must be present.
    match (&execution.run_cmd, &execution.container) {
        (Some(_), Some(_)) => return Err(ParsingError::ProcessAndContainerConflict.into()),
        (None, None) => return Err(ParsingError::NoProcessOrContainer.into()),
        _ => {}
    }

    // Validate container mount paths (reject top-level system directories).
    if let Some(container) = &execution.container
        && let Err((path, blocked_list)) = container.validate()
    {
        return Err(ParsingError::InvalidMountPath(path, blocked_list).into());
    }

    // Validate ${parameters:...} references in mount paths.
    if let Some(container) = &execution.container
        && let Err((ref_path, reason)) = container.validate_parameter_refs(&execution.parameters)
    {
        return Err(ParsingError::InvalidMountPathParameterRef(ref_path, reason).into());
    }

    Ok(())
}

/// Parser responsible for extracting configuration from JSON5 documents
pub struct NodeConfigParser;

impl NodeConfigParser {
    pub fn from_path(file: impl AsRef<Path>) -> Result<NodeConfig> {
        let path = file.as_ref();
        let content = read_non_empty_file(path)?;
        Self::from_content(&content)
    }

    /// Takes a JSON5 content as parameter
    pub fn from_content(content: &str) -> Result<NodeConfig> {
        // Strict schema validation is handled by serde via #[serde(deny_unknown_fields)]
        let config: NodeConfig = crate::error::deserialize_json5_with_path(content)?;
        validate_execution(&config.execution)?;
        validate_manifest_links(&config.manifest)?;
        validate_interfaces(&config.manifest, &config.interfaces)?;
        Ok(config)
    }
}

/// Loads a `peppy.json5` at `path` and returns a fully resolved [`NodeConfig`].
///
/// Used by `peppylib`'s standalone mode so Rust and Python nodes can
/// `cargo run` / `python -m` directly from a node directory.
pub fn load_standalone_node_config(path: impl AsRef<Path>) -> Result<NodeConfig> {
    NodeConfigParser::from_path(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{error::Error, node::ContainerConfig};
    use tempfile::NamedTempFile;

    /// Test helper: borrows the `ContainerConfig` from a parsed config.
    fn container(config: &NodeConfig) -> &ContainerConfig {
        config
            .execution
            .container
            .as_ref()
            .expect("expected container")
    }

    #[test]
    fn test_parse_minimal_config() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "test_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                run_cmd: ["./target/release/test_node"],
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).unwrap();
        assert_eq!(config.manifest.name.as_str(), "test_node");
        assert_eq!(config.manifest.tag, "v1");
        assert_eq!(
            config.execution.run_cmd.as_ref().unwrap(),
            &vec!["./target/release/test_node"]
        );
        assert!(config.execution.parameters.is_empty());
    }

    #[test]
    fn test_parse_complex_config() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "camera_driver",
                tag: "v21",
            },
            interfaces: {
                topics: {
                    emits: [
                        { name: "/camera/image_raw" }
                    ]
                }
            },
            execution: {
                language: "rust",
                run_cmd: ["./target/release/camera_driver"],
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).unwrap();
        assert_eq!(config.manifest.name.as_str(), "camera_driver");
        assert_eq!(config.manifest.tag, "v21");
        assert_eq!(
            config.execution.language,
            crate::node::PeppygenLanguage::Rust
        );
        assert_eq!(
            config.execution.run_cmd.as_ref().unwrap(),
            &vec!["./target/release/camera_driver"]
        );
        assert!(config.interfaces.topics.is_some());
    }

    #[test]
    fn test_empty_file() {
        let tmp = NamedTempFile::new().unwrap();
        // Ensure file is empty
        std::fs::write(tmp.path(), b"").unwrap();
        let result = NodeConfigParser::from_path(tmp.path());
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            Error::Parsing(ParsingError::EmptyContent(_))
        ));
    }

    #[test]
    fn test_cannot_read_file() {
        let result = NodeConfigParser::from_path("/path/that/does/not/exist.json5");
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            Error::Parsing(ParsingError::CannotRead(_, _))
        ));
    }

    #[test]
    fn test_cannot_parse_json5() {
        let json5 = r#"{ manifest: [unclosed"#; // invalid JSON5
        let result = NodeConfigParser::from_content(json5);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            Error::Parsing(ParsingError::CannotParseConfig(_))
        ));
    }

    #[test]
    fn test_parse_container_config() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "container_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                container: {
                    def_file: "apptainer.def",
                },
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).unwrap();
        assert!(config.execution.run_cmd.is_none());
        assert_eq!(container(&config).def_file, "apptainer.def");
    }

    #[test]
    fn test_process_and_container_conflict() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "bad_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
                container: {
                    def_file: "apptainer.def",
                },
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(matches!(
            result.unwrap_err(),
            Error::Parsing(ParsingError::ProcessAndContainerConflict)
        ));
    }

    #[test]
    fn test_no_process_or_container() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "bare_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(matches!(
            result.unwrap_err(),
            Error::Parsing(ParsingError::NoProcessOrContainer)
        ));
    }

    #[test]
    fn test_duplicate_link_id_within_nodes_rejected() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "dup_node",
                tag: "v1",
                depends_on: {
                    nodes: [
                        { name: "alpha", tag: "v1", link_id: "shared" },
                        { name: "beta",  tag: "v1", link_id: "shared" },
                    ],
                },
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::DuplicateLinkId(id)) if id == "shared"
            ),
            "expected DuplicateLinkId(\"shared\"), got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_duplicate_link_id_across_nodes_and_contracts_rejected() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "dup_node",
                tag: "v1",
                depends_on: {
                    nodes: [
                        { name: "alpha", tag: "v1", link_id: "shared" },
                    ],
                    contracts: [
                        { name: "depth_camera", tag: "v1", link_id: "shared" },
                    ],
                },
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::DuplicateLinkId(id)) if id == "shared"
            ),
            "expected DuplicateLinkId(\"shared\") across nodes+contracts, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_unique_link_ids_parse_ok() {
        // Same (name, tag) repeated under distinct link_ids must parse.
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "openarm01_backbone",
                tag: "v1",
                depends_on: {
                    nodes: [
                        { name: "depth_camera", tag: "v1", link_id: "wrist_left_camera" },
                        { name: "depth_camera", tag: "v1", link_id: "wrist_right_camera" },
                        { name: "depth_camera", tag: "v1", link_id: "torso_camera" },
                    ],
                },
            },
            interfaces: {
                topics: {
                    consumes: [
                        { link_id: "wrist_left_camera",  name: "video_stream" },
                        { link_id: "wrist_right_camera", name: "video_stream" },
                        { link_id: "torso_camera",       name: "video_stream" },
                    ],
                },
            },
            execution: {
                language: "rust",
                build_cmd: ["cargo", "build", "--release"],
                run_cmd: ["./target/release/backbone"],
            },
        }"#;
        let config =
            NodeConfigParser::from_content(json5).expect("distinct link_ids should parse cleanly");
        let deps = config
            .manifest
            .depends_on
            .expect("depends_on should be set");
        assert_eq!(deps.nodes.len(), 3);
    }

    #[test]
    fn test_dependency_entry_with_from_any_rejected() {
        // `from_any` was removed from the schema: a dependency slot only ever
        // receives from producers the launcher binds to it. A manifest still
        // carrying the flag must fail loudly instead of silently parsing.
        for deps_block in [
            r#"nodes: [{ name: "alpha", tag: "v1", link_id: "a", from_any: true }]"#,
            r#"contracts: [{ name: "beta", tag: "v1", link_id: "b", from_any: true }]"#,
        ] {
            let json5 = format!(
                r#"{{
                peppy_schema: "node/v1",
                manifest: {{
                    name: "stale_node",
                    tag: "v1",
                    depends_on: {{ {deps_block} }},
                }},
                execution: {{
                    language: "rust",
                    run_cmd: ["./bin"],
                }},
            }}"#
            );
            let result = NodeConfigParser::from_content(&json5);
            assert!(
                result.is_err(),
                "`from_any` must be rejected as an unknown field in: {deps_block}"
            );
        }
    }

    #[test]
    fn test_pairing_dependency_parses_with_defaults() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "robot_arm",
                tag: "v1",
                depends_on: {
                    pairings: [
                        { name: "arm_link", tag: "v1", role: "arm", link_id: "controller", optional: true },
                    ],
                },
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).unwrap();
        let deps = config.manifest.depends_on.unwrap();
        assert_eq!(deps.pairings.len(), 1);
        let pairing = &deps.pairings[0];
        assert_eq!(pairing.name.as_str(), "arm_link");
        assert_eq!(pairing.role, "arm");
        assert_eq!(pairing.link_id, "controller");
        assert!(pairing.optional);
        assert!(pairing.sha256.is_none());
    }

    #[test]
    fn test_pairing_optional_defaults_to_false() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "arm_controller",
                tag: "v1",
                depends_on: {
                    pairings: [
                        { name: "arm_link", tag: "v1", role: "controller", link_id: "arm" },
                    ],
                },
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).unwrap();
        let deps = config.manifest.depends_on.unwrap();
        assert!(!deps.pairings[0].optional);
    }

    #[test]
    fn test_same_pairing_twice_under_distinct_link_ids_parses() {
        // The two-arm commander shape: one pairing contract, two slots.
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "two_arm_commander",
                tag: "v1",
                depends_on: {
                    pairings: [
                        { name: "arm_link", tag: "v1", role: "controller", link_id: "left_arm" },
                        { name: "arm_link", tag: "v1", role: "controller", link_id: "right_arm" },
                    ],
                },
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).unwrap();
        assert_eq!(config.manifest.depends_on.unwrap().pairings.len(), 2);
    }

    #[test]
    fn test_duplicate_link_id_across_contracts_and_pairings_rejected() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "dup_node",
                tag: "v1",
                depends_on: {
                    contracts: [
                        { name: "depth_camera", tag: "v1", link_id: "shared" },
                    ],
                    pairings: [
                        { name: "arm_link", tag: "v1", role: "arm", link_id: "shared" },
                    ],
                },
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::DuplicateLinkId(id)) if id == "shared"
            ),
            "expected DuplicateLinkId(\"shared\") across contracts+pairings, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_pairing_link_id_with_at_sign_rejected_as_wire_unsafe() {
        // Node/contract link_ids never travel the wire, but a pairing slot
        // link_id is stamped into publish keyexprs, so wire-unsafe characters
        // must be rejected at parse time.
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "bad_pairing_node",
                tag: "v1",
                depends_on: {
                    pairings: [
                        { name: "arm_link", tag: "v1", role: "arm", link_id: "ctrl@home" },
                    ],
                },
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::PairingSentinelLinkId(id)) if id == "ctrl@home"
            ),
            "expected PairingSentinelLinkId(\"ctrl@home\"), got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn is_wire_safe_link_id_rejects_each_reserved_shape() {
        // Direct coverage of every branch: separator, addressing marker,
        // reserved keyexpr sentinels, default-link_id sentinel.
        for unsafe_id in ["a/b", "ctrl@home", "*", "**", "_"] {
            assert!(
                !is_wire_safe_link_id(unsafe_id),
                "{unsafe_id:?} must be rejected as wire-unsafe"
            );
        }
        assert!(is_wire_safe_link_id("controller"));
    }

    #[test]
    fn test_pairing_link_id_keyexpr_wildcards_rejected() {
        // Like the `_` sentinel below, bare `*` / `**` are caught by the
        // field deserializer's alphanumeric rule before the wire-safety
        // check; the parse must fail one way or another.
        for wildcard in ["*", "**"] {
            let json5 = format!(
                r#"{{
                peppy_schema: "node/v1",
                manifest: {{
                    name: "bad_pairing_node",
                    tag: "v1",
                    depends_on: {{
                        pairings: [
                            {{ name: "arm_link", tag: "v1", role: "arm", link_id: "{wildcard}" }},
                        ],
                    }},
                }},
                execution: {{
                    language: "rust",
                    run_cmd: ["./bin"],
                }},
            }}"#
            );
            assert!(
                NodeConfigParser::from_content(&json5).is_err(),
                "pairing link_id {wildcard:?} must fail parse"
            );
        }
    }

    #[test]
    fn test_pairing_link_id_sentinel_rejected() {
        // The bare `_` sentinel never reaches the wire-safety check (the field
        // deserializer's alphanumeric rule rejects it first), but it must fail
        // parse one way or another.
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "bad_pairing_node",
                tag: "v1",
                depends_on: {
                    pairings: [
                        { name: "arm_link", tag: "v1", role: "arm", link_id: "_" },
                    ],
                },
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
            },
        }"#;
        assert!(NodeConfigParser::from_content(json5).is_err());
    }

    #[test]
    fn test_pairing_entry_with_unknown_field_rejected() {
        // Pairing entries are deny_unknown_fields; a stray key must not
        // silently pass.
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "bad_pairing_node",
                tag: "v1",
                depends_on: {
                    pairings: [
                        { name: "arm_link", tag: "v1", role: "arm", link_id: "controller", extra: true },
                    ],
                },
            },
            execution: {
                language: "rust",
                run_cmd: ["./bin"],
            },
        }"#;
        assert!(NodeConfigParser::from_content(json5).is_err());
    }

    #[test]
    fn test_empty_run_cmd() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "empty_cmd_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                run_cmd: [],
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(matches!(
            result.unwrap_err(),
            Error::Parsing(ParsingError::EmptyRunCmd)
        ));
    }

    /// Top-level system directories (e.g. `/tmp`) are blocked as mount sources
    /// because Lima 2.0+ rejects them as guest mount points and binding an
    /// entire system directory into a container is almost always a mistake.
    /// Users should mount a subdirectory instead (e.g. `/tmp/my_app`).
    #[test]
    fn test_container_config_rejects_system_path_mount() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "bad_mount_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                container: {
                    def_file: "apptainer.def",
                    mount_paths: ["/tmp:/tmp:rw"],
                },
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::InvalidMountPath(_, _))
            ),
            "expected InvalidMountPath error, got: {:?}",
            result.unwrap_err()
        );
    }

    /// macOS exposes `/private/tmp`, `/private/var`, etc. as aliases for
    /// `/tmp`, `/var`, etc. The validation strips the `/private` prefix so
    /// these paths are caught by the same blocked-mount-source check.
    #[test]
    fn test_container_config_rejects_private_system_path_mount() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "bad_mount_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                container: {
                    def_file: "apptainer.def",
                    mount_paths: ["/private/tmp:/tmp:rw"],
                },
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::InvalidMountPath(_, _))
            ),
            "expected InvalidMountPath error for /private/tmp, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_container_config_accepts_subdirectory_mount() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "good_mount_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                container: {
                    def_file: "apptainer.def",
                    mount_paths: ["/tmp/my_app_data:/tmp/my_app_data:rw"],
                },
            },
        }"#;
        let config =
            NodeConfigParser::from_content(json5).expect("subdirectory mount should be accepted");
        assert_eq!(
            container(&config).mount_paths.as_deref().unwrap(),
            &["/tmp/my_app_data:/tmp/my_app_data:rw"]
        );
    }

    #[test]
    fn test_container_config_accepts_no_mount_paths() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "no_mount_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                container: {
                    def_file: "apptainer.def",
                },
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).expect("no mount_paths should be valid");
        assert!(container(&config).mount_paths.is_none());
    }

    // NOTE: These parse-time tests assert the raw `${parameters:...}` template
    // strings. Actual substitution with concrete argument values happens at
    // runtime in `resolve_mount_path_parameters()` (core-node-internal), which
    // has its own test coverage.
    #[test]
    fn test_container_mount_path_with_parameter_ref() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "camera_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                parameters: {
                    device_path: "string",
                },
                container: {
                    def_file: "apptainer.def",
                    mount_paths: ["${parameters:device_path}:/dev/video0:rw"],
                },
            },
        }"#;
        let config = NodeConfigParser::from_content(json5)
            .expect("parameter ref in mount path should parse");
        assert_eq!(
            container(&config).mount_paths.as_deref().unwrap(),
            &["${parameters:device_path}:/dev/video0:rw"]
        );
    }

    #[test]
    fn test_container_mount_path_with_nested_parameter_ref() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "camera_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                parameters: {
                    video: {
                        device_path: "string",
                        frame_rate: "u16",
                    },
                },
                container: {
                    def_file: "apptainer.def",
                    mount_paths: ["${parameters:video.device_path}:/dev/video0:rw"],
                },
            },
        }"#;
        let config = NodeConfigParser::from_content(json5)
            .expect("nested parameter ref in mount path should parse");
        assert_eq!(
            container(&config).mount_paths.as_deref().unwrap(),
            &["${parameters:video.device_path}:/dev/video0:rw"]
        );
    }

    #[test]
    fn test_container_mount_path_rejects_unknown_parameter_ref() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "bad_ref_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                parameters: {
                    device_path: "string",
                },
                container: {
                    def_file: "apptainer.def",
                    mount_paths: ["${parameters:nonexistent}:/data:rw"],
                },
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::InvalidMountPathParameterRef(ref_path, _))
                    if ref_path == "nonexistent"
            ),
            "expected InvalidMountPathParameterRef error, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_container_mount_path_rejects_non_string_parameter_ref() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "bad_type_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                parameters: {
                    frame_rate: "u16",
                },
                container: {
                    def_file: "apptainer.def",
                    mount_paths: ["${parameters:frame_rate}:/data:rw"],
                },
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::InvalidMountPathParameterRef(ref_path, reason))
                    if ref_path == "frame_rate" && reason.contains("string")
            ),
            "expected InvalidMountPathParameterRef error about string type, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_container_mount_path_skips_blocked_check_for_parameter_ref() {
        // A mount path whose source is a parameter reference should NOT be rejected
        // at parse time, even though the resolved value might be a blocked path.
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "dynamic_mount_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                parameters: {
                    path: "string",
                },
                container: {
                    def_file: "apptainer.def",
                    mount_paths: ["${parameters:path}:/container/data:rw"],
                },
            },
        }"#;
        let config = NodeConfigParser::from_content(json5)
            .expect("parameter ref source should skip blocked-path check at parse time");
        assert_eq!(
            container(&config).mount_paths.as_deref().unwrap(),
            &["${parameters:path}:/container/data:rw"]
        );
    }

    #[test]
    fn test_container_config_extra_args_default_to_none() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "container_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                container: {
                    def_file: "apptainer.def",
                },
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).unwrap();
        assert!(container(&config).apptainer_build_extra_args.is_none());
        assert!(container(&config).apptainer_run_extra_args.is_none());
        assert!(container(&config).lima_shell_extra_args.is_none());
    }

    #[test]
    fn test_container_config_parses_extra_args() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "container_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                container: {
                    def_file: "apptainer.def",
                    apptainer_build_extra_args: ["--no-setgroups", "--force"],
                    apptainer_run_extra_args: ["--no-setgroups"],
                    lima_shell_extra_args: ["--timeout", "30"],
                },
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).unwrap();
        let c = container(&config);
        assert_eq!(
            c.apptainer_build_extra_args.as_deref().unwrap(),
            &["--no-setgroups", "--force"]
        );
        assert_eq!(
            c.apptainer_run_extra_args.as_deref().unwrap(),
            &["--no-setgroups"]
        );
        assert_eq!(
            c.lima_shell_extra_args.as_deref().unwrap(),
            &["--timeout", "30"]
        );
    }

    #[test]
    fn test_container_config_parses_empty_extra_args() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "container_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                container: {
                    def_file: "apptainer.def",
                    apptainer_build_extra_args: [],
                    apptainer_run_extra_args: [],
                    lima_shell_extra_args: [],
                },
            },
        }"#;
        let config = NodeConfigParser::from_content(json5).unwrap();
        let c = container(&config);
        assert_eq!(
            c.apptainer_build_extra_args.as_deref().unwrap(),
            &[] as &[String]
        );
        assert_eq!(
            c.apptainer_run_extra_args.as_deref().unwrap(),
            &[] as &[String]
        );
        assert_eq!(
            c.lima_shell_extra_args.as_deref().unwrap(),
            &[] as &[String]
        );
    }

    #[test]
    fn test_parse_config_execution_without_language_rejected() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "test_node",
                tag: "v1",
            },
            execution: {
                run_cmd: ["./bin"],
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(matches!(
            result.unwrap_err(),
            Error::Parsing(ParsingError::MissingExecutionLanguage)
        ));
    }

    #[test]
    fn test_node_error_message_includes_field_path() {
        // run_cmd should be an array, not a map
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "test_node",
                tag: "v1",
            },
            execution: {
                language: "rust",
                run_cmd: { wrong: "type" },
            },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        let Error::Parsing(ParsingError::CannotParseConfig(msg)) = result.unwrap_err() else {
            panic!("expected CannotParseConfig error");
        };
        assert!(
            msg.contains("execution.run_cmd"),
            "error should include field path, got: {msg}"
        );
    }

    #[test]
    fn load_standalone_returns_resolved_config() {
        let tmp = NamedTempFile::new().unwrap();
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: { language: "rust", run_cmd: ["./target/debug/my_node"] },
        }"#;
        std::fs::write(tmp.path(), json5).unwrap();

        let node_config =
            load_standalone_node_config(tmp.path()).expect("config should load cleanly");
        assert_eq!(node_config.manifest.name.as_str(), "my_node");
        assert_eq!(
            node_config.execution.run_cmd.as_deref(),
            Some(["./target/debug/my_node".to_string()].as_slice())
        );
    }

    /// Future-proofing test: when a new field is added to [`NodeConfig`], this
    /// exhaustive destructuring will fail to compile, forcing the author to
    /// think about how it should be handled.
    #[test]
    fn node_config_field_set_is_complete() {
        let tmp = NamedTempFile::new().unwrap();
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: { language: "rust", run_cmd: ["./target/debug/my_node"] },
        }"#;
        std::fs::write(tmp.path(), json5).unwrap();

        let merged = load_standalone_node_config(tmp.path()).unwrap();

        // Exhaustive destructuring — add-a-field will fail compilation here.
        let NodeConfig {
            peppy_schema,
            manifest,
            interfaces,
            execution,
        } = merged;

        assert_eq!(peppy_schema, crate::schema::PeppySchema::NodeV1);
        assert_eq!(manifest.name.as_str(), "my_node");
        assert_eq!(manifest.tag, "v1");
        assert_eq!(interfaces, crate::node::Interfaces::default());
        assert_eq!(
            execution.run_cmd.as_deref(),
            Some(["./target/debug/my_node".to_string()].as_slice())
        );
    }

    // ─── manifest.implements + produced-entry Tier A validation ─────────────

    /// Wraps an implements list + interfaces block in a minimal node config.
    fn node_with(implements: &str, interfaces: &str) -> String {
        format!(
            r#"{{
            peppy_schema: "node/v1",
            manifest: {{
                name: "camera_node",
                tag: "v1",
                implements: [{implements}],
            }},
            interfaces: {interfaces},
            execution: {{
                language: "rust",
                run_cmd: ["./bin"],
            }},
        }}"#
        )
    }

    #[test]
    fn test_implements_with_full_coverage_entries_parses() {
        let json5 = node_with(
            r#"{ name: "uvc_camera", tag: "v1", link_id: "cam" }"#,
            r#"{
                topics: { emits: [{ link_id: "cam", name: "video_stream" }] },
                services: { exposes: [{ link_id: "cam", name: "video_stream_info" }] },
            }"#,
        );
        let config = NodeConfigParser::from_content(&json5).expect("should parse");
        assert_eq!(config.manifest.implements.len(), 1);
        assert_eq!(config.manifest.implements[0].link_id, "cam");
    }

    #[test]
    fn test_old_conforms_to_manifest_rejected_as_unknown_field() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "old_node", tag: "v1" },
            interfaces: {
                conforms_to: [{ name: "uvc_camera", tag: "v1" }],
            },
            execution: { language: "rust", run_cmd: ["./bin"] },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        let Error::Parsing(ParsingError::CannotParseConfig(msg)) = result.unwrap_err() else {
            panic!("expected plain serde unknown-field rejection");
        };
        assert!(
            msg.contains("conforms_to"),
            "error should name the unknown field, got: {msg}"
        );
    }

    #[test]
    fn test_duplicate_implements_contract_rejected() {
        let json5 = node_with(
            r#"{ name: "uvc_camera", tag: "v1", link_id: "cam_a" },
               { name: "uvc_camera", tag: "v1", link_id: "cam_b" }"#,
            r#"{}"#,
        );
        let result = NodeConfigParser::from_content(&json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::DuplicateImplementsContract { name, tag })
                    if name == "uvc_camera" && tag == "v1"
            ),
            "expected DuplicateImplementsContract, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_implements_sanitized_tag_collision_rejected() {
        let json5 = node_with(
            r#"{ name: "uvc_camera", tag: "v-1", link_id: "cam_a" },
               { name: "uvc_camera", tag: "v_1", link_id: "cam_b" }"#,
            r#"{}"#,
        );
        let result = NodeConfigParser::from_content(&json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::ImplementsTagSanitizationCollision { name, .. })
                    if name == "uvc_camera"
            ),
            "expected ImplementsTagSanitizationCollision, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_implements_link_id_collides_with_depends_on_link_id() {
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "dup_node",
                tag: "v1",
                implements: [{ name: "uvc_camera", tag: "v1", link_id: "shared" }],
                depends_on: {
                    nodes: [{ name: "alpha", tag: "v1", link_id: "shared" }],
                },
            },
            execution: { language: "rust", run_cmd: ["./bin"] },
        }"#;
        let result = NodeConfigParser::from_content(json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::DuplicateLinkId(id)) if id == "shared"
            ),
            "expected DuplicateLinkId across implements+depends_on, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_emits_link_id_naming_depends_on_slot_rejected() {
        for (deps_block, found_in) in [
            (
                r#"nodes: [{ name: "alpha", tag: "v1", link_id: "dep" }]"#,
                "nodes",
            ),
            (
                r#"contracts: [{ name: "uvc_camera", tag: "v1", link_id: "dep" }]"#,
                "contracts",
            ),
            (
                r#"pairings: [{ name: "arm_link", tag: "v1", role: "arm", link_id: "dep" }]"#,
                "pairings",
            ),
        ] {
            let json5 = format!(
                r#"{{
                peppy_schema: "node/v1",
                manifest: {{
                    name: "wrong_direction",
                    tag: "v1",
                    depends_on: {{ {deps_block} }},
                }},
                interfaces: {{
                    topics: {{ emits: [{{ link_id: "dep", name: "video_stream" }}] }},
                }},
                execution: {{ language: "rust", run_cmd: ["./bin"] }},
            }}"#
            );
            let result = NodeConfigParser::from_content(&json5);
            assert!(
                matches!(
                    result.as_ref().unwrap_err(),
                    Error::Parsing(ParsingError::EmitsLinkIdNotImplements { link_id, found_in: f, .. })
                        if link_id == "dep" && f == found_in
                ),
                "expected EmitsLinkIdNotImplements({found_in}), got: {:?}",
                result.unwrap_err()
            );
        }
    }

    #[test]
    fn test_emits_link_id_matching_nothing_rejected() {
        let json5 = node_with(
            r#"{ name: "uvc_camera", tag: "v1", link_id: "cam" }"#,
            r#"{ topics: { emits: [{ link_id: "typo", name: "video_stream" }] } }"#,
        );
        let result = NodeConfigParser::from_content(&json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::UndeclaredEmitsLinkId { link_id, .. })
                    if link_id == "typo"
            ),
            "expected UndeclaredEmitsLinkId, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_consumed_item_referencing_implements_link_id_rejected() {
        let json5 = node_with(
            r#"{ name: "uvc_camera", tag: "v1", link_id: "cam" }"#,
            r#"{
                topics: {
                    emits: [{ link_id: "cam", name: "video_stream" }],
                    consumes: [{ link_id: "cam", name: "video_stream" }],
                },
            }"#,
        );
        let result = NodeConfigParser::from_content(&json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::ConsumedItemReferencesImplementsLinkId { link_id })
                    if link_id == "cam"
            ),
            "expected ConsumedItemReferencesImplementsLinkId, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_duplicate_native_emit_name_rejected() {
        let json5 = node_with(
            "",
            r#"{
                topics: { emits: [
                    { name: "stream", message_format: { x: "f64" } },
                    { name: "stream", message_format: { y: "f64" } },
                ] },
            }"#,
        );
        let result = NodeConfigParser::from_content(&json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::DuplicateInterfaceEntry { key, .. })
                    if key == "stream"
            ),
            "expected DuplicateInterfaceEntry, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_duplicate_contract_backed_entry_rejected() {
        let json5 = node_with(
            r#"{ name: "uvc_camera", tag: "v1", link_id: "cam" }"#,
            r#"{
                topics: { emits: [
                    { link_id: "cam", name: "video_stream" },
                    { link_id: "cam", name: "video_stream" },
                ] },
            }"#,
        );
        let result = NodeConfigParser::from_content(&json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::DuplicateInterfaceEntry { key, .. })
                    if key == "cam:video_stream"
            ),
            "expected DuplicateInterfaceEntry, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_native_and_contract_backed_same_name_coexist() {
        // Allowed on the producer: the two are namespaced apart in modules,
        // schema keys, and wire keys, and node-dep consumers resolve only the
        // native one (contract-backed interfaces are consumed via
        // depends_on.contracts).
        let json5 = node_with(
            r#"{ name: "uvc_camera", tag: "v1", link_id: "cam" }"#,
            r#"{
                topics: { emits: [
                    { link_id: "cam", name: "video_stream" },
                    { name: "video_stream", message_format: { x: "f64" } },
                ] },
            }"#,
        );
        NodeConfigParser::from_content(&json5)
            .expect("native + contract-backed same-name coexistence must parse");
    }

    #[test]
    fn test_contract_backed_entry_with_inline_shape_rejected() {
        let json5 = node_with(
            r#"{ name: "uvc_camera", tag: "v1", link_id: "cam" }"#,
            r#"{
                topics: { emits: [
                    { link_id: "cam", name: "video_stream", qos_profile: "sensor_data" },
                ] },
            }"#,
        );
        let result = NodeConfigParser::from_content(&json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::ContractBackedEntryWithInlineShape { field, .. })
                    if field == "qos_profile"
            ),
            "expected ContractBackedEntryWithInlineShape, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_empty_interface_name_rejected_via_structured_error() {
        let json5 = node_with(
            "",
            r#"{ topics: { emits: [{ message_format: { x: "f64" } }] } }"#,
        );
        let result = NodeConfigParser::from_content(&json5);
        assert!(
            matches!(
                result.as_ref().unwrap_err(),
                Error::Parsing(ParsingError::EmptyInterfaceName { section })
                    if section == "topics.emits"
            ),
            "expected EmptyInterfaceName, got: {:?}",
            result.unwrap_err()
        );
    }

    #[test]
    fn test_relay_shape_parses() {
        // Implementing AND depending on the same contract (name, tag) under
        // distinct link_ids is legal (relay shape).
        let json5 = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "relay",
                tag: "v1",
                implements: [{ name: "uvc_camera", tag: "v1", link_id: "cam_out" }],
                depends_on: {
                    contracts: [{ name: "uvc_camera", tag: "v1", link_id: "cam_in" }],
                },
            },
            interfaces: {
                topics: {
                    emits: [{ link_id: "cam_out", name: "video_stream" }],
                    consumes: [{ link_id: "cam_in", name: "video_stream" }],
                },
            },
            execution: { language: "rust", run_cmd: ["./bin"] },
        }"#;
        NodeConfigParser::from_content(json5).expect("relay shape must parse");
    }
}
