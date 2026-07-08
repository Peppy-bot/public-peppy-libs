//! Machine-readable registry of every core-node wire method.
//!
//! This is the single declaration point for the callable surface a federated
//! client sees over the zenoh wire: request/reply **services**, streaming
//! **actions**, and published **topics** (`clock` is both a service and a
//! topic, so entries are keyed by `(name, kind)`). Every method is declared
//! exactly once, in the [`methods!`] invocation below, which emits:
//!
//! * the [`ServiceId`] / [`ActionId`] / [`TopicId`] enums — the handles every
//!   consumer that must make a per-method decision (daemon registration,
//!   peppylib wrapper coverage, wire-name pinning, doc tagging) matches on
//!   **exhaustively**, so forgetting a new method is a compile error there;
//! * the [`METHODS`] descriptor slice, in declaration order, which the
//!   `platform-backend` AsyncAPI generator walks.
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
    /// The wire name, as declared in the [`methods!`] invocation and pinned by
    /// the exhaustive wire-pin tests below.
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

/// The one declaration point for every core-node wire method.
///
/// Each `services` entry emits a [`ServiceId`] variant, each `actions` entry an
/// [`ActionId`] variant, each `topics` entry a [`TopicId`] variant (per-kind
/// enums, so `clock` can be both a service and a topic), and every entry emits
/// its [`MethodDescriptor`] in [`METHODS`] — in declaration order (services,
/// then actions, then topics), which `descriptor()` relies on for indexing and
/// the AsyncAPI generator relies on for stable document ordering.
///
/// Per entry: `name` is the wire string (the compatibility contract, pinned by
/// the wire-pin tests below); `summary` is the one-line human description fed
/// to the AsyncAPI docs and re-emitted as the variant's first rustdoc line;
/// doc comments on an entry become extended rustdoc on the variant. Services
/// carry a `host`; actions and topics are always daemon-hosted today. Payload
/// lines pair the codec struct in [`crate::encoding`] with its Cap'n Proto
/// `Owned` root (see [`pd!`]).
///
/// The enums deliberately do **not** get `Display`/`From<..> for &str` impls:
/// `.name()` at the wire boundary is the only sanctioned way back to a string,
/// so stringly-typed plumbing cannot quietly reappear. They are also not
/// `#[non_exhaustive]` — downstream crates matching exhaustively (daemon
/// registration, peppylib coverage) and thus failing to compile when a method
/// is added is the entire point of this registry.
macro_rules! methods {
    (
        services {
            $( $(#[$smeta:meta])* $svar:ident {
                name: $sname:literal,
                host: $shost:ident,
                summary: $ssummary:literal,
                request: $sreq:ident as $sreq_owned:ty,
                response: $sresp:ident as $sresp_owned:ty,
                schema: $sschema:literal $(,)?
            } )+
        }
        actions {
            $( $(#[$ameta:meta])* $avar:ident {
                name: $aname:literal,
                summary: $asummary:literal,
                goal: $agoal:ident as $agoal_owned:ty,
                goal_response: $agoal_resp:ident as $agoal_resp_owned:ty,
                feedback: $afb:ident as $afb_owned:ty,
                result: $ares:ident as $ares_owned:ty,
                schema: $aschema:literal $(,)?
            } )+
        }
        topics {
            $( $(#[$tmeta:meta])* $tvar:ident {
                name: $tname:literal,
                summary: $tsummary:literal,
                message: $tmsg:ident as $tmsg_owned:ty,
                schema: $tschema:literal $(,)?
            } )+
        }
    ) => {
        /// Every core-node **service** (request/reply queryable), one variant
        /// per method. Generated by [`methods!`]; match exhaustively (no
        /// wildcard arm) wherever a per-service decision is made, so a new
        /// service is a compile error there until it is handled.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub enum ServiceId {
            $( #[doc = $ssummary] #[doc = ""] $(#[$smeta])* $svar, )+
        }

        impl ServiceId {
            /// Every service, in registry (declaration) order.
            pub const ALL: &'static [Self] = &[ $( Self::$svar, )+ ];

            /// The wire name. The only sanctioned enum-to-string step, for use
            /// at the wire boundary.
            pub const fn name(self) -> &'static str {
                match self { $( Self::$svar => $sname, )+ }
            }

            /// The peer that hosts this service's queryable.
            pub const fn host(self) -> Host {
                match self { $( Self::$svar => Host::$shost, )+ }
            }

            /// This service's entry in [`METHODS`].
            pub fn descriptor(self) -> &'static MethodDescriptor {
                // Services lead METHODS in declaration order.
                &METHODS[self as usize]
            }
        }

        /// Every core-node **action** (streaming goal/feedback/result), one
        /// variant per method. Generated by [`methods!`]; match exhaustively
        /// (no wildcard arm) wherever a per-action decision is made.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub enum ActionId {
            $( #[doc = $asummary] #[doc = ""] $(#[$ameta])* $avar, )+
        }

        impl ActionId {
            /// Every action, in registry (declaration) order.
            pub const ALL: &'static [Self] = &[ $( Self::$avar, )+ ];

            /// The wire name. The only sanctioned enum-to-string step, for use
            /// at the wire boundary.
            pub const fn name(self) -> &'static str {
                match self { $( Self::$avar => $aname, )+ }
            }

            /// This action's entry in [`METHODS`].
            pub fn descriptor(self) -> &'static MethodDescriptor {
                // Actions follow the services in METHODS.
                &METHODS[ServiceId::ALL.len() + self as usize]
            }
        }

        /// Every core-node **topic** (one-way publish), one variant per
        /// method. Generated by [`methods!`]; match exhaustively (no wildcard
        /// arm) wherever a per-topic decision is made.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub enum TopicId {
            $( #[doc = $tsummary] #[doc = ""] $(#[$tmeta])* $tvar, )+
        }

        impl TopicId {
            /// Every topic, in registry (declaration) order.
            pub const ALL: &'static [Self] = &[ $( Self::$tvar, )+ ];

            /// The wire name. The only sanctioned enum-to-string step, for use
            /// at the wire boundary.
            pub const fn name(self) -> &'static str {
                match self { $( Self::$tvar => $tname, )+ }
            }

            /// This topic's entry in [`METHODS`].
            pub fn descriptor(self) -> &'static MethodDescriptor {
                // Topics close METHODS, after services and actions.
                &METHODS[ServiceId::ALL.len() + ActionId::ALL.len() + self as usize]
            }
        }

        /// Every core-node wire method, in declaration order (services, then
        /// actions, then topics — the order the `descriptor()` index math and
        /// the generated AsyncAPI document depend on). `clock` appears twice
        /// (service + topic); `(name, kind)` pairs are unique, enforced by the
        /// tests below.
        pub static METHODS: &[MethodDescriptor] = &[
            $( MethodDescriptor {
                name: $sname,
                host: Host::$shost,
                summary: $ssummary,
                payloads: Payloads::Service {
                    request: pd!($sreq, $sreq_owned, $sschema),
                    response: pd!($sresp, $sresp_owned, $sschema),
                },
            }, )+
            $( MethodDescriptor {
                name: $aname,
                host: Host::CoreNodeDaemon,
                summary: $asummary,
                payloads: Payloads::Action {
                    goal: pd!($agoal, $agoal_owned, $aschema),
                    goal_response: pd!($agoal_resp, $agoal_resp_owned, $aschema),
                    feedback: pd!($afb, $afb_owned, $aschema),
                    result: pd!($ares, $ares_owned, $aschema),
                },
            }, )+
            $( MethodDescriptor {
                name: $tname,
                host: Host::CoreNodeDaemon,
                summary: $tsummary,
                payloads: Payloads::Topic {
                    message: pd!($tmsg, $tmsg_owned, $tschema),
                },
            }, )+
        ];
    };
}

methods! {
    services {
        Clock {
            name: "clock",
            host: CoreNodeDaemon,
            summary: "Read the daemon's current wall-clock time snapshot.",
            request: ClockRequest as crate::clock_capnp::clock_request::Owned,
            response: ClockResponse as crate::clock_capnp::clock_response::Owned,
            schema: "clock.capnp",
        }
        Info {
            name: "info",
            host: CoreNodeDaemon,
            summary: "Report daemon build/runtime info and its container inventory.",
            request: InfoRequest as crate::info_capnp::info_request::Owned,
            response: InfoResponse as crate::info_capnp::info_response::Owned,
            schema: "info.capnp",
        }
        /// Liveness service the core node exposes for an external prober (the
        /// platform backend polls it over the federated zenoh link). Distinct
        /// from the per-node echo `node_health` in peppylib, which the
        /// daemon's watchdog uses to check spawned nodes.
        Health {
            name: "health",
            host: CoreNodeDaemon,
            summary: "Liveness probe: a well-formed reply is itself the health signal.",
            request: HealthRequest as crate::health_capnp::health_request::Owned,
            response: HealthResponse as crate::health_capnp::health_response::Owned,
            schema: "health.capnp",
        }
        DatastoreStore {
            name: "datastore_store",
            host: CoreNodeDaemon,
            summary: "Store a value under a datastore key.",
            request: DatastoreStoreRequest as crate::datastore_capnp::datastore_store_request::Owned,
            response: DatastoreStoreResponse as crate::datastore_capnp::datastore_store_response::Owned,
            schema: "datastore.capnp",
        }
        DatastoreGet {
            name: "datastore_get",
            host: CoreNodeDaemon,
            summary: "Fetch a value by datastore key.",
            request: DatastoreGetRequest as crate::datastore_capnp::datastore_get_request::Owned,
            response: DatastoreGetResponse as crate::datastore_capnp::datastore_get_response::Owned,
            schema: "datastore.capnp",
        }
        DatastoreList {
            name: "datastore_list",
            host: CoreNodeDaemon,
            summary: "List datastore entries (optionally under a prefix).",
            request: DatastoreListRequest as crate::datastore_capnp::datastore_list_request::Owned,
            response: DatastoreListResponse as crate::datastore_capnp::datastore_list_response::Owned,
            schema: "datastore.capnp",
        }
        DatastoreRemove {
            name: "datastore_remove",
            host: CoreNodeDaemon,
            summary: "Remove a datastore entry by key.",
            request: DatastoreRemoveRequest as crate::datastore_capnp::datastore_remove_request::Owned,
            response: DatastoreRemoveResponse as crate::datastore_capnp::datastore_remove_response::Owned,
            schema: "datastore.capnp",
        }
        StackReset {
            name: "stack_reset",
            host: CoreNodeDaemon,
            summary: "Tear down the whole node stack back to an empty state.",
            request: NodeResetRequest as crate::node_capnp::node_reset_request::Owned,
            response: NodeResetResponse as crate::node_capnp::node_reset_response::Owned,
            schema: "node.capnp",
        }
        StackList {
            name: "stack_list",
            host: CoreNodeDaemon,
            summary: "List every node in the stack with its lifecycle stage.",
            request: StackListRequest as crate::node_capnp::node_list_request::Owned,
            response: StackListResponse as crate::node_capnp::node_list_response::Owned,
            schema: "node.capnp",
        }
        NodeInit {
            name: "node_init",
            host: CoreNodeDaemon,
            summary: "Initialize a node config entry in the stack.",
            request: NodeInitRequest as crate::node_capnp::node_init_request::Owned,
            response: NodeInitResponse as crate::node_capnp::node_init_response::Owned,
            schema: "node.capnp",
        }
        NodeRemove {
            name: "node_remove",
            host: CoreNodeDaemon,
            summary: "Remove a node entry from the stack.",
            request: NodeRemoveRequest as crate::node_capnp::node_remove_request::Owned,
            response: NodeRemoveResponse as crate::node_capnp::node_remove_response::Owned,
            schema: "node.capnp",
        }
        NodeSync {
            name: "node_sync",
            host: CoreNodeDaemon,
            summary: "Reconcile a node's config against its resolved repo sources.",
            request: NodeSyncRequest as crate::node_capnp::node_sync_request::Owned,
            response: NodeSyncResponse as crate::node_capnp::node_sync_response::Owned,
            schema: "node.capnp",
        }
        NodeInfo {
            name: "node_info",
            host: CoreNodeDaemon,
            summary: "Inspect a single node: config, stage, and tracked instances.",
            request: NodeInfoRequest as crate::node_capnp::node_info_request::Owned,
            response: NodeInfoResponse as crate::node_capnp::node_info_response::Owned,
            schema: "node.capnp",
        }
        NodeStop {
            name: "node_stop",
            host: CoreNodeDaemon,
            summary: "Stop a running node instance.",
            request: NodeStopRequest as crate::node_capnp::node_stop_request::Owned,
            response: NodeStopResponse as crate::node_capnp::node_stop_response::Owned,
            schema: "node.capnp",
        }
        RepoAdd {
            name: "repo_add",
            host: CoreNodeDaemon,
            summary: "Register a repository source with the daemon.",
            request: RepoAddRequest as crate::repo_capnp::repo_add_request::Owned,
            response: RepoAddResponse as crate::repo_capnp::repo_add_response::Owned,
            schema: "repo.capnp",
        }
        RepoExclude {
            name: "repo_exclude",
            host: CoreNodeDaemon,
            summary: "Mark a repository as excluded from discovery.",
            request: RepoExcludeRequest as crate::repo_capnp::repo_exclude_request::Owned,
            response: RepoExcludeResponse as crate::repo_capnp::repo_exclude_response::Owned,
            schema: "repo.capnp",
        }
        RepoList {
            name: "repo_list",
            host: CoreNodeDaemon,
            summary: "List registered repositories and their discovered node entries.",
            request: RepoListRequest as crate::repo_capnp::repo_list_request::Owned,
            response: RepoListResponse as crate::repo_capnp::repo_list_response::Owned,
            schema: "repo.capnp",
        }
        RepoRemove {
            name: "repo_remove",
            host: CoreNodeDaemon,
            summary: "Remove a repository source.",
            request: RepoRemoveRequest as crate::repo_capnp::repo_remove_request::Owned,
            response: RepoRemoveResponse as crate::repo_capnp::repo_remove_response::Owned,
            schema: "repo.capnp",
        }
        /// Framework service every *spawned node* (not the daemon) exposes.
        /// The daemon polls it during `peppy stack benchmark` to measure each
        /// producer's clock offset and normalize cross-host timestamps.
        ClockOffset {
            name: "clock_offset",
            host: SpawnedNode,
            summary: "Framework service every spawned node exposes; the daemon polls it \
                      during `peppy stack benchmark` to normalize cross-host timestamps.",
            request: ClockOffsetRequest as crate::clock_capnp::clock_offset_request::Owned,
            response: ClockOffsetResponse as crate::clock_capnp::clock_offset_response::Owned,
            schema: "clock.capnp",
        }
    }
    actions {
        StackLaunch {
            name: "stack_launch",
            summary: "Launch a stack from a launcher manifest, streaming per-node progress.",
            goal: LaunchGoal as crate::launch_capnp::launch_goal::Owned,
            goal_response: LaunchGoalResponse as crate::launch_capnp::launch_goal_response::Owned,
            feedback: LaunchFeedback as crate::launch_capnp::launch_feedback::Owned,
            result: LaunchResult as crate::launch_capnp::launch_result::Owned,
            schema: "launch.capnp",
        }
        StackBenchmark {
            name: "stack_benchmark",
            summary: "Benchmark interface latencies across the running stack.",
            goal: StackBenchmarkGoal as crate::benchmark_capnp::stack_benchmark_goal::Owned,
            goal_response: StackBenchmarkGoalResponse as crate::benchmark_capnp::stack_benchmark_goal_response::Owned,
            feedback: StackBenchmarkFeedback as crate::benchmark_capnp::stack_benchmark_feedback::Owned,
            result: StackBenchmarkResult as crate::benchmark_capnp::stack_benchmark_result::Owned,
            schema: "benchmark.capnp",
        }
        NodeAdd {
            name: "node_add",
            summary: "Add a node to the stack, streaming resolution and fetch progress.",
            goal: NodeAddGoal as crate::node_capnp::node_add_goal::Owned,
            goal_response: NodeAddGoalResponse as crate::node_capnp::node_add_goal_response::Owned,
            feedback: NodeAddFeedback as crate::node_capnp::node_add_feedback::Owned,
            result: NodeAddResult as crate::node_capnp::node_add_result::Owned,
            schema: "node.capnp",
        }
        NodeBuild {
            name: "node_build",
            summary: "Build a node's container image, streaming build log lines.",
            goal: NodeBuildGoal as crate::node_capnp::node_build_goal::Owned,
            goal_response: NodeBuildGoalResponse as crate::node_capnp::node_build_goal_response::Owned,
            feedback: NodeBuildFeedback as crate::node_capnp::node_build_feedback::Owned,
            result: NodeBuildResult as crate::node_capnp::node_build_result::Owned,
            schema: "node.capnp",
        }
        NodeRun {
            name: "node_run",
            summary: "Run a node instance, streaming startup progress.",
            goal: NodeRunGoal as crate::node_capnp::node_run_goal::Owned,
            goal_response: NodeRunGoalResponse as crate::node_capnp::node_run_goal_response::Owned,
            feedback: NodeRunFeedback as crate::node_capnp::node_run_feedback::Owned,
            result: NodeRunResult as crate::node_capnp::node_run_result::Owned,
            schema: "node.capnp",
        }
        RepoRefresh {
            name: "repo_refresh",
            summary: "Rescan repositories, streaming each discovered/excluded item.",
            goal: RepoRefreshGoal as crate::repo_capnp::repo_refresh_goal::Owned,
            goal_response: RepoRefreshGoalResponse as crate::repo_capnp::repo_refresh_goal_response::Owned,
            feedback: RepoRefreshFeedback as crate::repo_capnp::repo_refresh_feedback::Owned,
            result: RepoRefreshResult as crate::repo_capnp::repo_refresh_result::Owned,
            schema: "repo.capnp",
        }
    }
    topics {
        Clock {
            name: "clock",
            summary: "Periodic wall-clock snapshot published for time-sync consumers.",
            message: ClockTick as crate::clock_capnp::clock_tick::Owned,
            schema: "clock.capnp",
        }
        /// Topic the daemon publishes a periodic liveness beat on. Each
        /// spawned node subscribes via its daemon-liveness watchdog and shuts
        /// itself down if the beat goes silent past the configured grace
        /// period (uncatchable-death safety net). Keyed by the core-node name
        /// (derived deterministically per machine unless the operator
        /// overrides it via `core_node_name` in
        /// `~/.peppy/conf/peppy_config.json5` or `--core-node-name`), so a
        /// restarted daemon resumes on the same key and nodes survive the
        /// restart.
        DaemonHeartbeat {
            name: "daemon_heartbeat",
            summary: "Daemon liveness beacon; payload is a constant ClockTick(0), only arrival matters.",
            message: ClockTick as crate::clock_capnp::clock_tick::Owned,
            schema: "clock.capnp",
        }
    }
}

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

    /// Pinned copy of every service wire string. Exhaustive on purpose:
    /// adding a service without pinning its wire string here is a compile
    /// error, so this second copy of the strings cannot be forgotten — its
    /// only failure mode is a deliberate rename, which is exactly what
    /// [`wire_names_are_pinned`] must force a human to look at (the strings
    /// are the compatibility contract between the daemon and every client;
    /// publish and subscribe sides must agree byte-for-byte).
    fn pinned_service_name(id: ServiceId) -> &'static str {
        match id {
            ServiceId::Clock => "clock",
            ServiceId::Info => "info",
            ServiceId::Health => "health",
            ServiceId::DatastoreStore => "datastore_store",
            ServiceId::DatastoreGet => "datastore_get",
            ServiceId::DatastoreList => "datastore_list",
            ServiceId::DatastoreRemove => "datastore_remove",
            ServiceId::StackReset => "stack_reset",
            ServiceId::StackList => "stack_list",
            ServiceId::NodeInit => "node_init",
            ServiceId::NodeRemove => "node_remove",
            ServiceId::NodeSync => "node_sync",
            ServiceId::NodeInfo => "node_info",
            ServiceId::NodeStop => "node_stop",
            ServiceId::RepoAdd => "repo_add",
            ServiceId::RepoExclude => "repo_exclude",
            ServiceId::RepoList => "repo_list",
            ServiceId::RepoRemove => "repo_remove",
            ServiceId::ClockOffset => "clock_offset",
        }
    }

    /// See [`pinned_service_name`].
    fn pinned_action_name(id: ActionId) -> &'static str {
        match id {
            ActionId::StackLaunch => "stack_launch",
            ActionId::StackBenchmark => "stack_benchmark",
            ActionId::NodeAdd => "node_add",
            ActionId::NodeBuild => "node_build",
            ActionId::NodeRun => "node_run",
            ActionId::RepoRefresh => "repo_refresh",
        }
    }

    /// See [`pinned_service_name`].
    fn pinned_topic_name(id: TopicId) -> &'static str {
        match id {
            TopicId::Clock => "clock",
            TopicId::DaemonHeartbeat => "daemon_heartbeat",
        }
    }

    /// An accidental variant rename (or a typo in a `name:` field) must not
    /// silently change the wire; see [`pinned_service_name`].
    #[test]
    fn wire_names_are_pinned() {
        for &id in ServiceId::ALL {
            assert_eq!(id.name(), pinned_service_name(id));
        }
        for &id in ActionId::ALL {
            assert_eq!(id.name(), pinned_action_name(id));
        }
        for &id in TopicId::ALL {
            assert_eq!(id.name(), pinned_topic_name(id));
        }
    }

    /// `(name, kind)` is the registry key (`clock` is deliberately both a
    /// service and a topic); two methods of one kind sharing a wire string
    /// would shadow each other on the wire.
    #[test]
    fn method_name_kind_pairs_are_unique() {
        let pairs: BTreeSet<(&str, MethodKind)> =
            METHODS.iter().map(|m| (m.name, m.kind())).collect();
        assert_eq!(pairs.len(), METHODS.len(), "duplicate (name, kind) pair");
    }

    /// `descriptor()` indexes `METHODS` on the declaration-order invariant
    /// (services, then actions, then topics); prove every id lands on its own
    /// entry with the kind and host it claims.
    #[test]
    fn ids_index_their_own_descriptors() {
        assert_eq!(
            METHODS.len(),
            ServiceId::ALL.len() + ActionId::ALL.len() + TopicId::ALL.len(),
        );
        for &id in ServiceId::ALL {
            let m = id.descriptor();
            assert_eq!((m.name, m.kind()), (id.name(), MethodKind::Service));
            assert_eq!(m.host, id.host());
        }
        for &id in ActionId::ALL {
            let m = id.descriptor();
            assert_eq!((m.name, m.kind()), (id.name(), MethodKind::Action));
        }
        for &id in TopicId::ALL {
            let m = id.descriptor();
            assert_eq!((m.name, m.kind()), (id.name(), MethodKind::Topic));
        }
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
        let expected: &[(&MethodDescriptor, &str)] = &[
            (ServiceId::Health.descriptor(), "health.capnp:HealthRequest"),
            (ServiceId::Info.descriptor(), "info.capnp:InfoRequest"),
            (
                ServiceId::RepoList.descriptor(),
                "repo.capnp:RepoListRequest",
            ),
            (
                ServiceId::StackReset.descriptor(),
                "node.capnp:NodeResetRequest",
            ),
            (
                ServiceId::ClockOffset.descriptor(),
                "clock.capnp:ClockOffsetRequest",
            ),
            (
                ActionId::RepoRefresh.descriptor(),
                "repo.capnp:RepoRefreshGoal",
            ),
        ];
        for (method, want) in expected {
            let pd = match &method.payloads {
                Payloads::Service { request, .. } => request,
                Payloads::Action { goal, .. } => goal,
                Payloads::Topic { message } => message,
            };
            assert_eq!(&display_name(pd), want, "{} root display name", method.name);
        }
    }

    /// `TypeId` handles are stable and distinct per codec struct (sanity check
    /// that the `pd!` macro wired distinct types, not the same one twice).
    #[test]
    fn type_ids_are_distinct_per_payload() {
        let clock = ServiceId::Clock.descriptor();
        if let Payloads::Service { request, response } = &clock.payloads {
            assert_ne!((request.rust_type_id)(), (response.rust_type_id)());
        } else {
            panic!("clock service payloads");
        }
    }
}
