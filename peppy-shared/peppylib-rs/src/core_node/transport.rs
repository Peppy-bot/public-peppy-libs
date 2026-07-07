//! Transport shims that bridge the capnp wire types in
//! [`core_node_api::encoding`] to the peppylib messenger.
//!
//! `core-node-api` holds the pure wire types (no peppylib dep). This
//! module exposes `poll_*` / `send_*` free functions over those types.
//!
//! Each `poll_*` / `send_*` is a one-line macro invocation — the actual
//! routing lives in [`poll_core_node_service`] and [`send_core_node_goal`].
//! Add a new service by appending one `poll_service!` / `send_goal!` line
//! at the bottom of this file.

use std::time::Duration;

use config::node::QoSProfile;
use core_node_api::Payload;
use core_node_api::encoding::*;
use core_node_api::names;

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
                    service_name: $service,
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
                    action_name: $action,
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
/// `(name, req TypeId, resp TypeId)` row; each `goal` line expands to a
/// `send_goal!` invocation and contributes a `(name, goal TypeId)` row. The
/// table is consumed by the tests below to prove the transport surface and
/// [`core_node_api::registry`] never drift apart. Add a service/action by
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
        /// (bespoke routing) and is added to the service set by the test.
        #[cfg(test)]
        pub(crate) mod transport_table {
            // Bring in `names` and the `core_node_api::encoding::*` glob from
            // the parent module so the captured `$service`/`$req`/`$goal`
            // metavariables resolve here (macro-emitted paths resolve in the
            // module the tokens land in, not the invocation site).
            use super::*;

            /// `(service name, request TypeId, response TypeId)`.
            pub(crate) static SERVICES:
                &[(&str, fn() -> ::std::any::TypeId, fn() -> ::std::any::TypeId)] = &[
                $( ($service, || ::std::any::TypeId::of::<$req>(), || ::std::any::TypeId::of::<$resp>()), )*
            ];
            /// `(action name, goal TypeId)`.
            pub(crate) static GOALS: &[(&str, fn() -> ::std::any::TypeId)] = &[
                $( ($action, || ::std::any::TypeId::of::<$goal>()), )*
            ];
        }
    };
}

core_node_transport! {
    services: [
        pub poll_clock, ClockRequest, ClockResponse, names::CLOCK;
        pub poll_info, InfoRequest, InfoResponse, names::INFO;
        pub poll_stack_list, StackListRequest, StackListResponse, names::STACK_LIST;
        pub poll_datastore_store, DatastoreStoreRequest, DatastoreStoreResponse, names::DATASTORE_STORE;
        pub poll_datastore_get, DatastoreGetRequest, DatastoreGetResponse, names::DATASTORE_GET;
        pub poll_datastore_list, DatastoreListRequest, DatastoreListResponse, names::DATASTORE_LIST;
        pub poll_datastore_remove, DatastoreRemoveRequest, DatastoreRemoveResponse, names::DATASTORE_REMOVE;
        pub poll_node_reset, NodeResetRequest, NodeResetResponse, names::STACK_RESET;
        pub poll_node_init, NodeInitRequest, NodeInitResponse, names::NODE_INIT;
        pub poll_node_remove, NodeRemoveRequest, NodeRemoveResponse, names::NODE_REMOVE;
        pub poll_node_sync, NodeSyncRequest, NodeSyncResponse, names::NODE_SYNC;
        pub poll_node_info, NodeInfoRequest, NodeInfoResponse, names::NODE_INFO;
        pub poll_repo_list, RepoListRequest, RepoListResponse, names::REPO_LIST;
        pub poll_repo_add, RepoAddRequest, RepoAddResponse, names::REPO_ADD;
        pub poll_repo_exclude, RepoExcludeRequest, RepoExcludeResponse, names::REPO_EXCLUDE;
        pub poll_repo_remove, RepoRemoveRequest, RepoRemoveResponse, names::REPO_REMOVE;
    ]
    goals: [
        pub send_launch, LaunchGoal, names::STACK_LAUNCH_ACTION;
        pub send_node_add, NodeAddGoal, names::NODE_ADD_ACTION;
        pub send_node_run, NodeRunGoal, names::NODE_RUN_ACTION;
        pub send_node_build, NodeBuildGoal, names::NODE_BUILD_ACTION;
        pub send_repo_refresh, RepoRefreshGoal, names::REPO_REFRESH_ACTION;
        pub send_stack_benchmark, StackBenchmarkGoal, names::STACK_BENCHMARK_ACTION;
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
            service_name: names::NODE_STOP,
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

    use core_node_api::names;
    use core_node_api::registry::{METHODS, MethodKind, Payloads};

    use super::*;

    /// Registry `Service`/`Action` entries that intentionally have no `peppylib`
    /// transport wrapper: `HEALTH` is polled by the platform backend over the
    /// federated link, and `clock_offset` is polled via a raw `ServiceMessenger`
    /// in the daemon benchmark (it is hosted by spawned nodes, not the daemon).
    const EXCLUDED_SERVICES: &[&str] = &[names::HEALTH, names::CLOCK_OFFSET];

    /// One row of the service drift table: `(name, request TypeId, response TypeId)`.
    type ServiceRow = (&'static str, fn() -> TypeId, fn() -> TypeId);

    /// The hand-written `poll_node_stop` wrapper contributes its row here, since
    /// its bespoke routing keeps it out of the `core_node_transport!` list.
    fn node_stop_row() -> ServiceRow {
        (
            names::NODE_STOP,
            || TypeId::of::<NodeStopRequest>(),
            || TypeId::of::<NodeStopResponse>(),
        )
    }

    fn service_rows() -> Vec<ServiceRow> {
        transport_table::SERVICES
            .iter()
            .copied()
            .chain(std::iter::once(node_stop_row()))
            .collect()
    }

    /// Every registry `Service` (bar the documented exclusions) must have a
    /// transport wrapper whose request/response types are identical — by
    /// `TypeId`, i.e. exact type identity through the shared `core-node-api`
    /// path dependency — to the registry's descriptor types.
    #[test]
    fn registry_services_match_transport_table() {
        let rows = service_rows();
        for m in METHODS {
            if m.kind() != MethodKind::Service || EXCLUDED_SERVICES.contains(&m.name) {
                continue;
            }
            let Payloads::Service { request, response } = &m.payloads else {
                unreachable!("kind() said Service")
            };
            let row = rows
                .iter()
                .find(|(name, _, _)| *name == m.name)
                .unwrap_or_else(|| {
                    panic!(
                        "registry service {:?} has no transport wrapper (add a \
                         core_node_transport! line or add it to EXCLUDED_SERVICES)",
                        m.name
                    )
                });
            assert_eq!(
                (request.rust_type_id)(),
                (row.1)(),
                "{}: request type mismatch between registry and transport",
                m.name
            );
            assert_eq!(
                (response.rust_type_id)(),
                (row.2)(),
                "{}: response type mismatch between registry and transport",
                m.name
            );
        }
    }

    /// Every registry `Action`'s goal type must match the `send_goal!` wrapper's
    /// goal type. (Feedback/result/goal-response are not carried by the
    /// transport table — `send_goal!` only encodes the goal — so they are
    /// verified in-crate by `core-node-api`'s own registry tests.)
    #[test]
    fn registry_actions_match_transport_table() {
        for m in METHODS {
            if m.kind() != MethodKind::Action {
                continue;
            }
            let Payloads::Action { goal, .. } = &m.payloads else {
                unreachable!("kind() said Action")
            };
            let row = transport_table::GOALS
                .iter()
                .find(|(name, _)| *name == m.name)
                .unwrap_or_else(|| panic!("registry action {:?} has no send_goal wrapper", m.name));
            assert_eq!(
                (goal.rust_type_id)(),
                (row.1)(),
                "{}: goal type mismatch between registry and transport",
                m.name
            );
        }
    }

    /// The transport table must carry no rows the registry does not know about,
    /// so a wrapper for a removed/renamed method is caught too.
    #[test]
    fn transport_table_has_no_rows_absent_from_registry() {
        for (name, _, _) in service_rows() {
            assert!(
                METHODS
                    .iter()
                    .any(|m| m.name == name && m.kind() == MethodKind::Service),
                "transport service {name:?} is not a registry Service",
            );
        }
        for (name, _) in transport_table::GOALS {
            assert!(
                METHODS
                    .iter()
                    .any(|m| m.name == *name && m.kind() == MethodKind::Action),
                "transport goal {name:?} is not a registry Action",
            );
        }
    }
}
