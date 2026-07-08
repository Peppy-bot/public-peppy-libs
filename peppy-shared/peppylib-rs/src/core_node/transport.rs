//! Transport shims that bridge the capnp wire types in
//! [`core_node_api::encoding`] to the peppylib messenger.
//!
//! `core-node-api` holds the pure wire types (no peppylib dep). This
//! module exposes `poll_*` / `send_*` free functions over those types.
//!
//! Each `poll_*` / `send_*` is a one-line macro invocation — the actual
//! routing lives in [`poll_core_node_service`] and [`send_core_node_goal`].
//! Wrapper lines are keyed by [`ServiceId`] / [`ActionId`]: when a method is
//! added to the registry, the exhaustive coverage matches in the tests below
//! stop compiling until this file decides whether it gets a wrapper line, a
//! hand-written wrapper, or a documented exclusion.

use std::time::Duration;

use config::node::QoSProfile;
use core_node_api::Payload;
use core_node_api::encoding::*;
use core_node_api::names;
use core_node_api::{ActionId, ServiceId};

use crate::error::Result;
use crate::messaging::{ActionGoalHandle, SenderTarget, ServiceTarget};
use crate::{ActionMessenger, MessengerHandle, ServiceMessenger};

/// Routing parameters for a single service poll. Bundled into a struct so
/// [`poll_core_node_service`] doesn't need a `clippy::too_many_arguments`
/// escape hatch — the helper otherwise reaches 9 positional args.
///
/// Control-plane calls always discover (no producer pin at the messenger):
/// the daemon's services listen under a random per-boot instance_id no
/// caller can know up front, and the daemon stays addressed through
/// `to_target` (its identity rides in the service root), so discovery
/// resolves exactly that daemon's endpoint. The exception is `node_stop`,
/// whose listener may be hosted by a per-instance node: user node names
/// are not unique across daemons, so its service root cannot pin the
/// route by itself and the discovery is additionally scoped to the core
/// node hosting the target instance (`ServiceTarget::CoreNode`).
struct ServiceRoute<'a> {
    messenger: &'a MessengerHandle,
    bound_core_node: &'a str,
    as_instance_id: &'a str,
    /// Target of the service. For daemon-hosted core_node services this is
    /// `SenderTarget::node(to_core_node, CORE_NODE_TAG)`. For per-instance
    /// services (e.g. `node_stop`) it carries the target node's name+tag.
    to_target: SenderTarget,
    service_name: &'a str,
    /// Producer scope of the discovery. `ServiceTarget::Any` for daemon
    /// services (the service root already pins the route); `node_stop`
    /// narrows it to the target's core node, see the struct doc.
    target: ServiceTarget<'a>,
}

/// Routing parameters for a single goal send. Same rationale as
/// [`ServiceRoute`].
struct GoalRoute<'a> {
    messenger: &'a MessengerHandle,
    as_core_node: &'a str,
    as_instance_id: &'a str,
    action_name: &'a str,
    /// Core node whose daemon hosts the action; used only to build the
    /// `SenderTarget` service root. `None` falls back to the caller's own
    /// core_node.
    to_core_node: Option<&'a str>,
}

async fn poll_core_node_service<Response>(
    route: ServiceRoute<'_>,
    request_payload: Payload,
    decode_response: fn(&[u8]) -> core_node_api::Result<Response>,
    response_timeout: impl Into<Option<Duration>> + Send,
) -> Result<Response> {
    let response = ServiceMessenger::poll(
        route.messenger,
        route.bound_core_node,
        route.as_instance_id,
        route.to_target,
        route.service_name,
        route.target,
        request_payload,
        response_timeout,
    )
    .await?;
    decode_response(response.payload_bytes().as_ref()).map_err(Into::into)
}

async fn send_core_node_goal(
    route: GoalRoute<'_>,
    goal_payload: Payload,
    goal_timeout: Duration,
) -> Result<ActionGoalHandle> {
    let to_core = route.to_core_node.unwrap_or(route.as_core_node);
    ActionMessenger::send_goal(
        route.messenger,
        route.as_core_node,
        route.as_instance_id,
        SenderTarget::node(to_core, names::CORE_NODE_TAG)?,
        route.action_name,
        None,
        goal_payload,
        QoSProfile::default(),
        goal_timeout,
    )
    .await
}

/// Defines a `poll_*` wrapper that encodes `$req`, polls the service named
/// `$service` on the target core node, and decodes the response into `$resp`.
macro_rules! poll_service {
    ($vis:vis $name:ident, $req:ty, $resp:ty, $service:expr) => {
        $vis async fn $name(
            request: &$req,
            messenger: &MessengerHandle,
            bound_core_node: &str,
            as_instance_id: &str,
            to_core_node: &str,
            response_timeout: impl Into<Option<Duration>> + Send,
        ) -> Result<$resp> {
            poll_core_node_service(
                ServiceRoute {
                    messenger,
                    bound_core_node,
                    as_instance_id,
                    to_target: SenderTarget::node(to_core_node, names::CORE_NODE_TAG)?,
                    service_name: $service.name(),
                    target: ServiceTarget::Any,
                },
                request.encode()?,
                <$resp>::decode,
                response_timeout,
            )
            .await
        }
    };
}

/// Defines a `send_*` wrapper that encodes `$goal` and sends it as the action
/// named `$action` to the target core node.
macro_rules! send_goal {
    ($vis:vis $name:ident, $goal:ty, $action:expr) => {
        $vis async fn $name(
            goal: &$goal,
            messenger: &MessengerHandle,
            as_core_node: &str,
            as_instance_id: &str,
            to_core_node: Option<&str>,
            goal_timeout: Duration,
        ) -> Result<ActionGoalHandle> {
            send_core_node_goal(
                GoalRoute {
                    messenger,
                    as_core_node,
                    as_instance_id,
                    action_name: $action.name(),
                    to_core_node,
                },
                goal.encode()?,
                goal_timeout,
            )
            .await
        }
    };
}

/// Wraps the per-service / per-goal wrapper lines so a single list is the
/// source of truth for both the transport wrappers and a `#[cfg(test)]`
/// TypeId table. Each `service` line expands to a `poll_service!` invocation
/// (byte-identical to a hand-written one) and contributes a
/// `(ServiceId, req TypeId, resp TypeId)` row; each `goal` line expands to a
/// `send_goal!` invocation and contributes an `(ActionId, goal TypeId)` row.
/// The table is consumed by the tests below to prove the transport surface
/// and [`core_node_api::registry`] never drift apart. Add a service/action by
/// appending one line here — the table follows automatically.
macro_rules! core_node_transport {
    (
        services: [ $( $svc_vis:vis $svc_fn:ident, $req:ty, $resp:ty, $service:expr ; )* ]
        goals:    [ $( $goal_vis:vis $goal_fn:ident, $goal:ty, $action:expr ; )* ]
    ) => {
        $( poll_service!($svc_vis $svc_fn, $req, $resp, $service); )*
        $( send_goal!($goal_vis $goal_fn, $goal, $action); )*

        /// Emitted from the same lines that generate the transport wrappers, so
        /// it cannot fall out of sync with them. `node_stop` is hand-written
        /// (bespoke routing) and contributes its row via the test module's
        /// `hand_written_rows`.
        #[cfg(test)]
        pub(crate) mod transport_table {
            // Bring in the id enums and the `core_node_api::encoding::*` glob
            // from the parent module so the captured `$service`/`$req`/`$goal`
            // metavariables resolve here (macro-emitted paths resolve in the
            // module the tokens land in, not the invocation site).
            use super::*;

            /// `(service, request TypeId, response TypeId)`.
            pub(crate) static SERVICES:
                &[(ServiceId, fn() -> ::std::any::TypeId, fn() -> ::std::any::TypeId)] = &[
                $( ($service, || ::std::any::TypeId::of::<$req>(), || ::std::any::TypeId::of::<$resp>()), )*
            ];
            /// `(action, goal TypeId)`.
            pub(crate) static GOALS: &[(ActionId, fn() -> ::std::any::TypeId)] = &[
                $( ($action, || ::std::any::TypeId::of::<$goal>()), )*
            ];
        }
    };
}

core_node_transport! {
    services: [
        pub poll_clock, ClockRequest, ClockResponse, ServiceId::Clock;
        pub poll_info, InfoRequest, InfoResponse, ServiceId::Info;
        pub poll_stack_list, StackListRequest, StackListResponse, ServiceId::StackList;
        pub poll_datastore_store, DatastoreStoreRequest, DatastoreStoreResponse, ServiceId::DatastoreStore;
        pub poll_datastore_get, DatastoreGetRequest, DatastoreGetResponse, ServiceId::DatastoreGet;
        pub poll_datastore_list, DatastoreListRequest, DatastoreListResponse, ServiceId::DatastoreList;
        pub poll_datastore_remove, DatastoreRemoveRequest, DatastoreRemoveResponse, ServiceId::DatastoreRemove;
        pub poll_node_reset, NodeResetRequest, NodeResetResponse, ServiceId::StackReset;
        pub poll_node_init, NodeInitRequest, NodeInitResponse, ServiceId::NodeInit;
        pub poll_node_remove, NodeRemoveRequest, NodeRemoveResponse, ServiceId::NodeRemove;
        pub poll_node_sync, NodeSyncRequest, NodeSyncResponse, ServiceId::NodeSync;
        pub poll_node_info, NodeInfoRequest, NodeInfoResponse, ServiceId::NodeInfo;
        pub poll_repo_list, RepoListRequest, RepoListResponse, ServiceId::RepoList;
        pub poll_repo_add, RepoAddRequest, RepoAddResponse, ServiceId::RepoAdd;
        pub poll_repo_exclude, RepoExcludeRequest, RepoExcludeResponse, ServiceId::RepoExclude;
        pub poll_repo_remove, RepoRemoveRequest, RepoRemoveResponse, ServiceId::RepoRemove;
    ]
    goals: [
        pub send_launch, LaunchGoal, ActionId::StackLaunch;
        pub send_node_add, NodeAddGoal, ActionId::NodeAdd;
        pub send_node_run, NodeRunGoal, ActionId::NodeRun;
        pub send_node_build, NodeBuildGoal, ActionId::NodeBuild;
        pub send_repo_refresh, RepoRefreshGoal, ActionId::RepoRefresh;
        pub send_stack_benchmark, StackBenchmarkGoal, ActionId::StackBenchmark;
    ]
}

/// `node_stop` is the only service whose listener may be hosted by a
/// per-instance node rather than the daemon, so it routes by an explicit
/// `to_target` (name + tag) instead of defaulting to the daemon's core_node
/// identity. Hand-written for that reason.
///
/// User node names are not unique across daemons, so the `to_target`
/// service root alone cannot pin the route: on a multi-daemon network a
/// wildcard discovery could be won by a same-named listener on a foreign
/// core node, which would answer "unknown instance" while the right reply
/// is dropped. The discovery is therefore scoped to `scope_core_node` —
/// the core node hosting the instance to stop — which pins the route for
/// both daemon-hosted and per-instance listeners.
///
/// The two core-node parameters play distinct roles: `bound_core_node` is
/// the caller's identity (the core node it is bound to, riding in the
/// request's sender address), while `scope_core_node` is the discovery
/// scope. They coincide when stopping through the local daemon and differ
/// when targeting a remote one.
pub async fn poll_node_stop(
    request: &NodeStopRequest,
    messenger: &MessengerHandle,
    bound_core_node: &str,
    as_instance_id: &str,
    to_target: SenderTarget,
    scope_core_node: &str,
    response_timeout: impl Into<Option<Duration>> + Send,
) -> Result<NodeStopResponse> {
    poll_core_node_service(
        ServiceRoute {
            messenger,
            bound_core_node,
            as_instance_id,
            to_target,
            service_name: ServiceId::NodeStop.name(),
            target: ServiceTarget::CoreNode(scope_core_node),
        },
        request.encode()?,
        NodeStopResponse::decode,
        response_timeout,
    )
    .await
}

#[cfg(test)]
mod tests {
    use std::any::TypeId;

    use core_node_api::registry::Payloads;

    use super::*;

    /// How `peppylib` covers a registry service. Returned by the exhaustive
    /// [`service_coverage`] / [`action_coverage`] matches, so a new method
    /// cannot be added to the registry without deciding its coverage here —
    /// forgetting is a compile error, not a silently missing wrapper.
    enum Coverage {
        /// Has a `core_node_transport!` wrapper line (checked against the
        /// macro-emitted table).
        Wrapped,
        /// Hand-written wrapper outside the macro (bespoke routing) — its row
        /// is contributed by [`hand_written_rows`] and checked the same way.
        HandWritten,
        /// Deliberately no `peppylib` wrapper; records who calls it instead.
        External(&'static str),
    }

    /// Exhaustive on purpose: no wildcard arm, so every new service forces an
    /// explicit coverage decision in this file.
    fn service_coverage(id: ServiceId) -> Coverage {
        match id {
            ServiceId::Health => {
                Coverage::External("polled by the platform backend over the federated zenoh link")
            }
            ServiceId::ClockOffset => Coverage::External(
                "hosted by spawned nodes, polled via a raw ServiceMessenger in the daemon benchmark",
            ),
            // Bespoke to_target / scope_core_node routing, see poll_node_stop.
            ServiceId::NodeStop => Coverage::HandWritten,
            ServiceId::Clock => Coverage::Wrapped,
            ServiceId::Info => Coverage::Wrapped,
            ServiceId::DatastoreStore => Coverage::Wrapped,
            ServiceId::DatastoreGet => Coverage::Wrapped,
            ServiceId::DatastoreList => Coverage::Wrapped,
            ServiceId::DatastoreRemove => Coverage::Wrapped,
            ServiceId::StackReset => Coverage::Wrapped,
            ServiceId::StackList => Coverage::Wrapped,
            ServiceId::NodeInit => Coverage::Wrapped,
            ServiceId::NodeRemove => Coverage::Wrapped,
            ServiceId::NodeSync => Coverage::Wrapped,
            ServiceId::NodeInfo => Coverage::Wrapped,
            ServiceId::RepoAdd => Coverage::Wrapped,
            ServiceId::RepoExclude => Coverage::Wrapped,
            ServiceId::RepoList => Coverage::Wrapped,
            ServiceId::RepoRemove => Coverage::Wrapped,
        }
    }

    /// Exhaustive for the same reason as [`service_coverage`]. Every action
    /// has a `send_goal!` wrapper today.
    fn action_coverage(id: ActionId) -> Coverage {
        match id {
            ActionId::StackLaunch => Coverage::Wrapped,
            ActionId::StackBenchmark => Coverage::Wrapped,
            ActionId::NodeAdd => Coverage::Wrapped,
            ActionId::NodeBuild => Coverage::Wrapped,
            ActionId::NodeRun => Coverage::Wrapped,
            ActionId::RepoRefresh => Coverage::Wrapped,
        }
    }

    /// One row of the service drift table: `(id, request TypeId, response TypeId)`.
    type ServiceRow = (ServiceId, fn() -> TypeId, fn() -> TypeId);

    /// Rows for the `Coverage::HandWritten` wrappers, whose bespoke routing
    /// keeps them out of the `core_node_transport!` list.
    fn hand_written_rows() -> Vec<ServiceRow> {
        vec![(
            ServiceId::NodeStop,
            || TypeId::of::<NodeStopRequest>(),
            || TypeId::of::<NodeStopResponse>(),
        )]
    }

    /// A wrapper row's request/response types must be identical — by `TypeId`,
    /// i.e. exact type identity through the shared `core-node-api` path
    /// dependency — to the registry descriptor's types.
    fn assert_row_matches_registry(id: ServiceId, row: &ServiceRow) {
        let Payloads::Service { request, response } = &id.descriptor().payloads else {
            unreachable!("ServiceId descriptors are Payloads::Service")
        };
        assert_eq!(
            (request.rust_type_id)(),
            (row.1)(),
            "{}: request type mismatch between registry and transport",
            id.name()
        );
        assert_eq!(
            (response.rust_type_id)(),
            (row.2)(),
            "{}: response type mismatch between registry and transport",
            id.name()
        );
    }

    /// Every registry service is covered exactly as [`service_coverage`]
    /// claims: `Wrapped` ids have exactly one macro-table row with matching
    /// payload types, `HandWritten` ids have their row in
    /// [`hand_written_rows`] (and none in the macro table), and `External`
    /// ids appear in neither.
    #[test]
    fn registry_services_match_transport_table() {
        let hand_written = hand_written_rows();
        for &id in ServiceId::ALL {
            let table_rows: Vec<&ServiceRow> = transport_table::SERVICES
                .iter()
                .filter(|(rid, _, _)| *rid == id)
                .collect();
            match service_coverage(id) {
                Coverage::Wrapped => {
                    assert_eq!(
                        table_rows.len(),
                        1,
                        "{}: expected exactly one core_node_transport! line",
                        id.name()
                    );
                    assert_row_matches_registry(id, table_rows[0]);
                }
                Coverage::HandWritten => {
                    assert!(
                        table_rows.is_empty(),
                        "{}: is Coverage::HandWritten but also has a \
                         core_node_transport! line",
                        id.name()
                    );
                    let row = hand_written
                        .iter()
                        .find(|(rid, _, _)| *rid == id)
                        .unwrap_or_else(|| panic!("{}: no hand_written_rows entry", id.name()));
                    assert_row_matches_registry(id, row);
                }
                Coverage::External(caller) => {
                    assert!(
                        table_rows.is_empty() && !hand_written.iter().any(|(rid, _, _)| *rid == id),
                        "{}: documented as external-only ({caller}) but has a \
                         wrapper row; update service_coverage",
                        id.name()
                    );
                }
            }
        }
    }

    /// Every registry action's goal type must match its `send_goal!` wrapper's
    /// goal type. (Feedback/result/goal-response are not carried by the
    /// transport table — `send_goal!` only encodes the goal — so they are
    /// verified in-crate by `core-node-api`'s own registry tests.)
    #[test]
    fn registry_actions_match_transport_table() {
        for &id in ActionId::ALL {
            match action_coverage(id) {
                Coverage::Wrapped => {}
                Coverage::HandWritten | Coverage::External(_) => unreachable!(
                    "no hand-written/external action transport exists today; \
                     teach this test about it before using those variants"
                ),
            }
            let Payloads::Action { goal, .. } = &id.descriptor().payloads else {
                unreachable!("ActionId descriptors are Payloads::Action")
            };
            let rows: Vec<_> = transport_table::GOALS
                .iter()
                .filter(|(rid, _)| *rid == id)
                .collect();
            assert_eq!(
                rows.len(),
                1,
                "{}: expected exactly one send_goal! line",
                id.name()
            );
            assert_eq!(
                (goal.rust_type_id)(),
                (rows[0].1)(),
                "{}: goal type mismatch between registry and transport",
                id.name()
            );
        }
    }
}
