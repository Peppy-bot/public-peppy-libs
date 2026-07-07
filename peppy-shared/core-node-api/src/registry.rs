//! Machine-readable registry of every core-node wire method.
//!
//! This is the single in-code source of truth for the callable surface a
//! federated client sees over the zenoh wire: 19 request/reply **services**, 6
//! streaming **actions**, and 2 published **topics** (27 entries; `clock` is
//! both a service and a topic, so entries are keyed by `(name, kind)`).
//!
//! Each entry hands out, per Cap'n Proto payload, three things:
//!
//! * a display name ([`PayloadDescriptor::rust_type`]),
//! * a [`TypeId`](core::any::TypeId) of the codec struct
//!   ([`PayloadDescriptor::rust_type_id`]) — used by `peppylib`'s cross-crate
//!   sync test to prove the registry and the transport table stay in step, and
//! * a runtime Cap'n Proto reflection handle ([`PayloadDescriptor::introspect`])
//!   plus the originating `.capnp` file — consumed by the `platform-backend`
//!   AsyncAPI generator, which walks the type into JSON Schema.
//!
//! The registry describes the Cap'n Proto payloads *only*. The wire framing
//! around them (query/reply attachments, the goal-id envelope, the action
//! result-status byte) is documented by the generator, not here.
//!
//! Because the raw `.capnp` sources carry the doc comments that the compiled
//! schema nodes drop, they are embedded verbatim in [`SCHEMA_SOURCES`] and
//! served alongside the generated document.

use core::any::TypeId;

use capnp::introspect::{Introspect, Type};

use crate::names;

/// The three interaction styles a core-node method can have.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MethodKind {
    /// A zenoh queryable: the caller `get`s and the daemon replies.
    Service,
    /// Goal / cancel / result queryables plus a per-goal feedback pub/sub topic.
    Action,
    /// A one-way `put` on a pub/sub topic.
    Topic,
}

/// Which peer hosts the queryable/publisher for a method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Host {
    /// The core-node daemon, addressed as `node/{core_node_name}/core`.
    CoreNodeDaemon,
    /// Every spawned node instance, addressed as `node/{node_name}/{node_tag}`;
    /// the daemon is the caller. Only `clock_offset` is hosted here today.
    SpawnedNode,
}

/// Everything the registry knows about a single Cap'n Proto payload.
///
/// `PartialEq` is intentionally not derived: the struct holds function
/// pointers, whose comparison is a lint and is meaningless anyway. Tests
/// compare the individual fields.
#[derive(Debug, Clone, Copy)]
pub struct PayloadDescriptor {
    /// Human-facing name of the Rust codec struct, e.g. `"ClockRequest"`.
    pub rust_type: &'static str,
    /// `TypeId` of the codec struct (`core_node_api::encoding::{rust_type}`).
    /// Used by the `peppylib` transport sync test for exact type identity.
    pub rust_type_id: fn() -> TypeId,
    /// Runtime Cap'n Proto reflection handle for the payload's wire root.
    pub introspect: fn() -> Type,
    /// `.capnp` file that defines the payload's struct, e.g. `"clock.capnp"`.
    pub schema_file: &'static str,
}

/// The payloads of a method, grouped by interaction style.
#[derive(Debug, Clone, Copy)]
pub enum Payloads {
    /// Request/reply.
    Service {
        request: PayloadDescriptor,
        response: PayloadDescriptor,
    },
    /// Streaming: goal + its ack, per-goal feedback, and the terminal result.
    Action {
        goal: PayloadDescriptor,
        goal_response: PayloadDescriptor,
        feedback: PayloadDescriptor,
        result: PayloadDescriptor,
    },
    /// Fire-and-forget publish.
    Topic { message: PayloadDescriptor },
}

impl Payloads {
    /// All payload descriptors in a stable, style-defined order
    /// (request, response / goal, goal_response, feedback, result / message).
    pub fn descriptors(&self) -> Vec<&PayloadDescriptor> {
        match self {
            Payloads::Service { request, response } => vec![request, response],
            Payloads::Action {
                goal,
                goal_response,
                feedback,
                result,
            } => vec![goal, goal_response, feedback, result],
            Payloads::Topic { message } => vec![message],
        }
    }
}

/// A single core-node wire method.
#[derive(Debug, Clone, Copy)]
pub struct MethodDescriptor {
    /// The wire name, one of the [`crate::names`] constants.
    pub name: &'static str,
    /// The peer that hosts this method.
    pub host: Host,
    /// A one-line human summary, condensed from the codec's rustdoc.
    pub summary: &'static str,
    /// The Cap'n Proto payloads carried by this method.
    pub payloads: Payloads,
}

impl MethodDescriptor {
    /// The interaction style, derived from [`MethodDescriptor::payloads`].
    pub fn kind(&self) -> MethodKind {
        match self.payloads {
            Payloads::Service { .. } => MethodKind::Service,
            Payloads::Action { .. } => MethodKind::Action,
            Payloads::Topic { .. } => MethodKind::Topic,
        }
    }
}

/// Build a [`PayloadDescriptor`] from a codec-struct ident, its Cap'n Proto
/// `Owned` root, and the schema file. `rust_type` is the ident stringified;
/// `rust_type_id` keys on `crate::encoding::{ident}` (the only public path to
/// the codec struct, and the same type `peppylib` observes); `introspect`
/// coerces the generated `Introspect::introspect` associated fn to `fn() -> Type`.
macro_rules! pd {
    ($enc:ident, $owned:ty, $file:literal) => {
        PayloadDescriptor {
            rust_type: stringify!($enc),
            rust_type_id: || TypeId::of::<crate::encoding::$enc>(),
            introspect: <$owned as Introspect>::introspect,
            schema_file: $file,
        }
    };
}

/// A request/reply service entry.
macro_rules! service {
    ($name:expr, $host:expr, $summary:literal,
     $req:ident : $req_owned:ty, $resp:ident : $resp_owned:ty, $file:literal) => {
        MethodDescriptor {
            name: $name,
            host: $host,
            summary: $summary,
            payloads: Payloads::Service {
                request: pd!($req, $req_owned, $file),
                response: pd!($resp, $resp_owned, $file),
            },
        }
    };
}

/// A streaming action entry (all four payloads share one schema file).
macro_rules! action {
    ($name:expr, $summary:literal,
     $goal:ident : $goal_owned:ty,
     $goal_resp:ident : $goal_resp_owned:ty,
     $fb:ident : $fb_owned:ty,
     $res:ident : $res_owned:ty, $file:literal) => {
        MethodDescriptor {
            name: $name,
            host: Host::CoreNodeDaemon,
            summary: $summary,
            payloads: Payloads::Action {
                goal: pd!($goal, $goal_owned, $file),
                goal_response: pd!($goal_resp, $goal_resp_owned, $file),
                feedback: pd!($fb, $fb_owned, $file),
                result: pd!($res, $res_owned, $file),
            },
        }
    };
}

/// A published-topic entry.
macro_rules! topic {
    ($name:expr, $summary:literal, $msg:ident : $msg_owned:ty, $file:literal) => {
        MethodDescriptor {
            name: $name,
            host: Host::CoreNodeDaemon,
            summary: $summary,
            payloads: Payloads::Topic {
                message: pd!($msg, $msg_owned, $file),
            },
        }
    };
}

/// Every core-node wire method. `clock` appears twice (service + topic); the
/// set of `(name, kind)` pairs is pinned against the [`crate::names`] constants
/// by the tests below.
pub static METHODS: &[MethodDescriptor] = &[
    // ---- Services ---------------------------------------------------------
    service!(
        names::CLOCK,
        Host::CoreNodeDaemon,
        "Read the daemon's current wall-clock time snapshot.",
        ClockRequest: crate::clock_capnp::clock_request::Owned,
        ClockResponse: crate::clock_capnp::clock_response::Owned,
        "clock.capnp"
    ),
    service!(
        names::INFO,
        Host::CoreNodeDaemon,
        "Report daemon build/runtime info and its container inventory.",
        InfoRequest: crate::info_capnp::info_request::Owned,
        InfoResponse: crate::info_capnp::info_response::Owned,
        "info.capnp"
    ),
    service!(
        names::HEALTH,
        Host::CoreNodeDaemon,
        "Liveness probe: a well-formed reply is itself the health signal.",
        HealthRequest: crate::health_capnp::health_request::Owned,
        HealthResponse: crate::health_capnp::health_response::Owned,
        "health.capnp"
    ),
    service!(
        names::DATASTORE_STORE,
        Host::CoreNodeDaemon,
        "Store a value under a datastore key.",
        DatastoreStoreRequest: crate::datastore_capnp::datastore_store_request::Owned,
        DatastoreStoreResponse: crate::datastore_capnp::datastore_store_response::Owned,
        "datastore.capnp"
    ),
    service!(
        names::DATASTORE_GET,
        Host::CoreNodeDaemon,
        "Fetch a value by datastore key.",
        DatastoreGetRequest: crate::datastore_capnp::datastore_get_request::Owned,
        DatastoreGetResponse: crate::datastore_capnp::datastore_get_response::Owned,
        "datastore.capnp"
    ),
    service!(
        names::DATASTORE_LIST,
        Host::CoreNodeDaemon,
        "List datastore entries (optionally under a prefix).",
        DatastoreListRequest: crate::datastore_capnp::datastore_list_request::Owned,
        DatastoreListResponse: crate::datastore_capnp::datastore_list_response::Owned,
        "datastore.capnp"
    ),
    service!(
        names::DATASTORE_REMOVE,
        Host::CoreNodeDaemon,
        "Remove a datastore entry by key.",
        DatastoreRemoveRequest: crate::datastore_capnp::datastore_remove_request::Owned,
        DatastoreRemoveResponse: crate::datastore_capnp::datastore_remove_response::Owned,
        "datastore.capnp"
    ),
    service!(
        names::STACK_RESET,
        Host::CoreNodeDaemon,
        "Tear down the whole node stack back to an empty state.",
        NodeResetRequest: crate::node_capnp::node_reset_request::Owned,
        NodeResetResponse: crate::node_capnp::node_reset_response::Owned,
        "node.capnp"
    ),
    service!(
        names::STACK_LIST,
        Host::CoreNodeDaemon,
        "List every node in the stack with its lifecycle stage.",
        StackListRequest: crate::node_capnp::node_list_request::Owned,
        StackListResponse: crate::node_capnp::node_list_response::Owned,
        "node.capnp"
    ),
    service!(
        names::NODE_INIT,
        Host::CoreNodeDaemon,
        "Initialize a node config entry in the stack.",
        NodeInitRequest: crate::node_capnp::node_init_request::Owned,
        NodeInitResponse: crate::node_capnp::node_init_response::Owned,
        "node.capnp"
    ),
    service!(
        names::NODE_REMOVE,
        Host::CoreNodeDaemon,
        "Remove a node entry from the stack.",
        NodeRemoveRequest: crate::node_capnp::node_remove_request::Owned,
        NodeRemoveResponse: crate::node_capnp::node_remove_response::Owned,
        "node.capnp"
    ),
    service!(
        names::NODE_SYNC,
        Host::CoreNodeDaemon,
        "Reconcile a node's config against its resolved repo sources.",
        NodeSyncRequest: crate::node_capnp::node_sync_request::Owned,
        NodeSyncResponse: crate::node_capnp::node_sync_response::Owned,
        "node.capnp"
    ),
    service!(
        names::NODE_INFO,
        Host::CoreNodeDaemon,
        "Inspect a single node: config, stage, and tracked instances.",
        NodeInfoRequest: crate::node_capnp::node_info_request::Owned,
        NodeInfoResponse: crate::node_capnp::node_info_response::Owned,
        "node.capnp"
    ),
    service!(
        names::NODE_STOP,
        Host::CoreNodeDaemon,
        "Stop a running node instance.",
        NodeStopRequest: crate::node_capnp::node_stop_request::Owned,
        NodeStopResponse: crate::node_capnp::node_stop_response::Owned,
        "node.capnp"
    ),
    service!(
        names::REPO_ADD,
        Host::CoreNodeDaemon,
        "Register a repository source with the daemon.",
        RepoAddRequest: crate::repo_capnp::repo_add_request::Owned,
        RepoAddResponse: crate::repo_capnp::repo_add_response::Owned,
        "repo.capnp"
    ),
    service!(
        names::REPO_EXCLUDE,
        Host::CoreNodeDaemon,
        "Mark a repository as excluded from discovery.",
        RepoExcludeRequest: crate::repo_capnp::repo_exclude_request::Owned,
        RepoExcludeResponse: crate::repo_capnp::repo_exclude_response::Owned,
        "repo.capnp"
    ),
    service!(
        names::REPO_LIST,
        Host::CoreNodeDaemon,
        "List registered repositories and their discovered node entries.",
        RepoListRequest: crate::repo_capnp::repo_list_request::Owned,
        RepoListResponse: crate::repo_capnp::repo_list_response::Owned,
        "repo.capnp"
    ),
    service!(
        names::REPO_REMOVE,
        Host::CoreNodeDaemon,
        "Remove a repository source.",
        RepoRemoveRequest: crate::repo_capnp::repo_remove_request::Owned,
        RepoRemoveResponse: crate::repo_capnp::repo_remove_response::Owned,
        "repo.capnp"
    ),
    service!(
        names::CLOCK_OFFSET,
        Host::SpawnedNode,
        "Framework service every spawned node exposes; the daemon polls it \
         during `peppy stack benchmark` to normalize cross-host timestamps.",
        ClockOffsetRequest: crate::clock_capnp::clock_offset_request::Owned,
        ClockOffsetResponse: crate::clock_capnp::clock_offset_response::Owned,
        "clock.capnp"
    ),
    // ---- Actions ----------------------------------------------------------
    action!(
        names::STACK_LAUNCH_ACTION,
        "Launch a stack from a launcher manifest, streaming per-node progress.",
        LaunchGoal: crate::launch_capnp::launch_goal::Owned,
        LaunchGoalResponse: crate::launch_capnp::launch_goal_response::Owned,
        LaunchFeedback: crate::launch_capnp::launch_feedback::Owned,
        LaunchResult: crate::launch_capnp::launch_result::Owned,
        "launch.capnp"
    ),
    action!(
        names::STACK_BENCHMARK_ACTION,
        "Benchmark interface latencies across the running stack.",
        StackBenchmarkGoal: crate::benchmark_capnp::stack_benchmark_goal::Owned,
        StackBenchmarkGoalResponse: crate::benchmark_capnp::stack_benchmark_goal_response::Owned,
        StackBenchmarkFeedback: crate::benchmark_capnp::stack_benchmark_feedback::Owned,
        StackBenchmarkResult: crate::benchmark_capnp::stack_benchmark_result::Owned,
        "benchmark.capnp"
    ),
    action!(
        names::NODE_ADD_ACTION,
        "Add a node to the stack, streaming resolution and fetch progress.",
        NodeAddGoal: crate::node_capnp::node_add_goal::Owned,
        NodeAddGoalResponse: crate::node_capnp::node_add_goal_response::Owned,
        NodeAddFeedback: crate::node_capnp::node_add_feedback::Owned,
        NodeAddResult: crate::node_capnp::node_add_result::Owned,
        "node.capnp"
    ),
    action!(
        names::NODE_BUILD_ACTION,
        "Build a node's container image, streaming build log lines.",
        NodeBuildGoal: crate::node_capnp::node_build_goal::Owned,
        NodeBuildGoalResponse: crate::node_capnp::node_build_goal_response::Owned,
        NodeBuildFeedback: crate::node_capnp::node_build_feedback::Owned,
        NodeBuildResult: crate::node_capnp::node_build_result::Owned,
        "node.capnp"
    ),
    action!(
        names::NODE_RUN_ACTION,
        "Run a node instance, streaming startup progress.",
        NodeRunGoal: crate::node_capnp::node_run_goal::Owned,
        NodeRunGoalResponse: crate::node_capnp::node_run_goal_response::Owned,
        NodeRunFeedback: crate::node_capnp::node_run_feedback::Owned,
        NodeRunResult: crate::node_capnp::node_run_result::Owned,
        "node.capnp"
    ),
    action!(
        names::REPO_REFRESH_ACTION,
        "Rescan repositories, streaming each discovered/excluded item.",
        RepoRefreshGoal: crate::repo_capnp::repo_refresh_goal::Owned,
        RepoRefreshGoalResponse: crate::repo_capnp::repo_refresh_goal_response::Owned,
        RepoRefreshFeedback: crate::repo_capnp::repo_refresh_feedback::Owned,
        RepoRefreshResult: crate::repo_capnp::repo_refresh_result::Owned,
        "repo.capnp"
    ),
    // ---- Topics -----------------------------------------------------------
    topic!(
        names::CLOCK,
        "Periodic wall-clock snapshot published for time-sync consumers.",
        ClockTick: crate::clock_capnp::clock_tick::Owned,
        "clock.capnp"
    ),
    topic!(
        names::DAEMON_HEARTBEAT,
        "Daemon liveness beacon; payload is a constant ClockTick(0), only arrival matters.",
        ClockTick: crate::clock_capnp::clock_tick::Owned,
        "clock.capnp"
    ),
];

/// The raw `.capnp` sources, embedded so the compiled-away doc comments remain
/// downloadable. Keys are the bare file names (e.g. `"clock.capnp"`); the
/// [`PayloadDescriptor::schema_file`] fields index into this table. Kept in
/// sync with the on-disk `schemas/` directory by the tests below.
pub const SCHEMA_SOURCES: &[(&str, &str)] = &[
    (
        "benchmark.capnp",
        include_str!("../schemas/benchmark.capnp"),
    ),
    ("clock.capnp", include_str!("../schemas/clock.capnp")),
    (
        "datastore.capnp",
        include_str!("../schemas/datastore.capnp"),
    ),
    ("health.capnp", include_str!("../schemas/health.capnp")),
    ("info.capnp", include_str!("../schemas/info.capnp")),
    ("launch.capnp", include_str!("../schemas/launch.capnp")),
    ("node.capnp", include_str!("../schemas/node.capnp")),
    ("repo.capnp", include_str!("../schemas/repo.capnp")),
];

/// Version of the wire protocol described by this registry. Feeds the AsyncAPI
/// document's `info.version`. Bump the minor on additive methods/fields, the
/// major on breaking wire changes.
pub const WIRE_API_VERSION: &str = "0.1.0";

/// Version of this crate, for provenance only (not the wire version).
pub const CRATE_VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    /// Read a descriptor's Cap'n Proto display name, e.g. `"clock.capnp:ClockRequest"`.
    fn display_name(pd: &PayloadDescriptor) -> String {
        match (pd.introspect)().which() {
            capnp::introspect::TypeVariant::Struct(raw) => {
                let schema: capnp::schema::StructSchema = raw.into();
                schema
                    .get_proto()
                    .get_display_name()
                    .expect("display name is present")
                    .to_str()
                    .expect("display name is UTF-8")
                    .to_string()
            }
            _ => panic!("{} does not introspect to a struct", pd.rust_type),
        }
    }

    /// The `(name, kind)` set in `METHODS` must equal the enumerated wire-name
    /// constants: every `names::*` constant except `CORE_NODE_TAG`, with `clock`
    /// appearing exactly twice (once `Service`, once `Topic`). This is the first
    /// link in the drift-protection chain — adding or dropping a method, or
    /// mislabelling its kind, fails here.
    #[test]
    fn registry_names_match_the_wire_contract() {
        use crate::names::*;
        let mut expected: Vec<(&str, MethodKind)> = vec![
            // Services
            (CLOCK, MethodKind::Service),
            (INFO, MethodKind::Service),
            (HEALTH, MethodKind::Service),
            (DATASTORE_STORE, MethodKind::Service),
            (DATASTORE_GET, MethodKind::Service),
            (DATASTORE_LIST, MethodKind::Service),
            (DATASTORE_REMOVE, MethodKind::Service),
            (STACK_RESET, MethodKind::Service),
            (STACK_LIST, MethodKind::Service),
            (NODE_INIT, MethodKind::Service),
            (NODE_REMOVE, MethodKind::Service),
            (NODE_SYNC, MethodKind::Service),
            (NODE_INFO, MethodKind::Service),
            (NODE_STOP, MethodKind::Service),
            (REPO_ADD, MethodKind::Service),
            (REPO_EXCLUDE, MethodKind::Service),
            (REPO_LIST, MethodKind::Service),
            (REPO_REMOVE, MethodKind::Service),
            (CLOCK_OFFSET, MethodKind::Service),
            // Actions
            (STACK_LAUNCH_ACTION, MethodKind::Action),
            (STACK_BENCHMARK_ACTION, MethodKind::Action),
            (NODE_ADD_ACTION, MethodKind::Action),
            (NODE_BUILD_ACTION, MethodKind::Action),
            (NODE_RUN_ACTION, MethodKind::Action),
            (REPO_REFRESH_ACTION, MethodKind::Action),
            // Topics
            (CLOCK, MethodKind::Topic),
            (DAEMON_HEARTBEAT, MethodKind::Topic),
        ];
        expected.sort();

        let mut actual: Vec<(&str, MethodKind)> =
            METHODS.iter().map(|m| (m.name, m.kind())).collect();
        actual.sort();

        assert_eq!(actual, expected);
        assert_eq!(METHODS.len(), 27);
    }

    /// Every payload descriptor must introspect to a Cap'n Proto struct whose
    /// display name is prefixed with the schema file the descriptor claims. The
    /// `.src_prefix("schemas")` in `build.rs` guarantees the display name is
    /// `"{file}.capnp:{TypePath}"`, so this ties each registry root to the right
    /// `.capnp` file even where the Rust type name and capnp struct name differ
    /// (e.g. `StackListRequest` -> `node.capnp:NodeListRequest`).
    #[test]
    fn payload_descriptors_resolve_and_point_at_their_schema_file() {
        for m in METHODS {
            for pd in m.payloads.descriptors() {
                let display = display_name(pd);
                let prefix = format!("{}:", pd.schema_file);
                assert!(
                    display.starts_with(&prefix),
                    "{} ({}) introspects to {:?}, expected prefix {:?}",
                    pd.rust_type,
                    m.name,
                    display,
                    prefix,
                );
            }
        }
    }

    /// `SCHEMA_SOURCES` must key exactly the on-disk `schemas/*.capnp` files and
    /// cover every `schema_file` referenced by a descriptor, with non-empty
    /// contents.
    #[test]
    fn schema_sources_cover_the_schemas_dir() {
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/schemas");
        let on_disk: BTreeSet<String> = std::fs::read_dir(dir)
            .expect("schemas dir is readable")
            .map(|e| {
                e.expect("dir entry")
                    .file_name()
                    .into_string()
                    .expect("UTF-8 name")
            })
            .filter(|n| n.ends_with(".capnp"))
            .collect();

        let keys: BTreeSet<String> = SCHEMA_SOURCES.iter().map(|(k, _)| k.to_string()).collect();
        assert_eq!(
            keys, on_disk,
            "SCHEMA_SOURCES keys must equal schemas/*.capnp"
        );

        for (name, source) in SCHEMA_SOURCES {
            assert!(!source.is_empty(), "{name} source is empty");
        }

        for m in METHODS {
            for pd in m.payloads.descriptors() {
                assert!(
                    keys.contains(pd.schema_file),
                    "{} references schema {} which is not in SCHEMA_SOURCES",
                    pd.rust_type,
                    pd.schema_file,
                );
            }
        }
    }

    /// Spot-check that, for at least one payload per schema file, the hand-written
    /// codec round-trips *and* the registry's introspected root points at exactly
    /// the same capnp struct the codec name implies. These six are the empty,
    /// trivially-constructible codecs (one per non-derived file plus the two
    /// unions' files), so a codec silently repointing at another root shows up
    /// here as a display-name mismatch.
    #[test]
    fn spot_check_codec_roundtrip_and_registry_root() {
        // Each is an empty unit-struct codec, so the bare type name is also a
        // value. `encode()` borrows, then `decode()` reconstructs from the bytes.
        macro_rules! roundtrip {
            ($ty:ident) => {{
                let value = crate::encoding::$ty;
                let payload = value.encode().expect("encode");
                let round = crate::encoding::$ty::decode(payload.as_ref()).expect("decode");
                assert_eq!(round, value, concat!(stringify!($ty), " round-trip"));
            }};
        }
        roundtrip!(HealthRequest);
        roundtrip!(InfoRequest);
        roundtrip!(RepoListRequest);
        roundtrip!(NodeResetRequest);
        roundtrip!(ClockOffsetRequest);
        roundtrip!(RepoRefreshGoal);

        // Registry root points at exactly the named capnp struct (file:Type).
        let expected: &[(&str, MethodKind, &str)] = &[
            (
                names::HEALTH,
                MethodKind::Service,
                "health.capnp:HealthRequest",
            ),
            (names::INFO, MethodKind::Service, "info.capnp:InfoRequest"),
            (
                names::REPO_LIST,
                MethodKind::Service,
                "repo.capnp:RepoListRequest",
            ),
            (
                names::STACK_RESET,
                MethodKind::Service,
                "node.capnp:NodeResetRequest",
            ),
            (
                names::CLOCK_OFFSET,
                MethodKind::Service,
                "clock.capnp:ClockOffsetRequest",
            ),
            (
                names::REPO_REFRESH_ACTION,
                MethodKind::Action,
                "repo.capnp:RepoRefreshGoal",
            ),
        ];
        for (name, kind, want) in expected {
            let method = METHODS
                .iter()
                .find(|m| m.name == *name && m.kind() == *kind)
                .unwrap_or_else(|| panic!("no {name} of kind {kind:?}"));
            let pd = match &method.payloads {
                Payloads::Service { request, .. } => request,
                Payloads::Action { goal, .. } => goal,
                Payloads::Topic { message } => message,
            };
            assert_eq!(&display_name(pd), want, "{name} root display name");
        }
    }

    /// `TypeId` handles are stable and distinct per codec struct (sanity check
    /// that the `pd!` macro wired distinct types, not the same one twice).
    #[test]
    fn type_ids_are_distinct_per_payload() {
        let clock = METHODS
            .iter()
            .find(|m| m.name == names::CLOCK && m.kind() == MethodKind::Service)
            .unwrap();
        if let Payloads::Service { request, response } = &clock.payloads {
            assert_ne!((request.rust_type_id)(), (response.rust_type_id)());
        } else {
            panic!("clock service payloads");
        }
    }
}
