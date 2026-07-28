//! Encoding types for the NodeRun action (streaming version with feedback).

use std::collections::BTreeMap;
use std::path::PathBuf;

use capnp::message::Builder;
use config::runtime::{NodeInstancePlan, ProducerRef};

use crate::node_capnp;
use crate::{NonEmptyPayload, Payload, Result};

use super::builder::FeedbackStream;
use crate::encoding::{
    capnp_list_len, decode_message, encode_message, encode_message_non_empty, optional_text,
    read_text_list, required_text, write_text_list,
};

/// One peer reference carried by [`NodeRunGoal::requested_pairs`] /
/// [`NodeRunGoal::covered_pairs`]: the peer instance and, optionally, the
/// pinned complementary slot on it. `Display` renders the CLI/launcher
/// target grammar (`<peer_instance>` or `<peer_instance>/<peer_link_id>`);
/// instance ids and link_ids are `/`-free names, so the rendering is
/// unambiguous.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PairTarget {
    /// The peer instance, addressed the way the whole mesh addresses
    /// instances. Its `core_node` is always populated, including for a
    /// same-daemon pair, so a daemon never has to infer "this must be local"
    /// from an absent field. Set by whoever plans the pair: the coordinator of
    /// a federated launch, or the receiving daemon's own name for a
    /// `node run --pair`.
    pub peer: ProducerRef,
    /// The complementary slot on the peer, when the request pins one.
    /// `None` is unpinned: exactly one available complementary slot must
    /// exist on the peer and the daemon resolves it.
    pub peer_link_id: Option<String>,
}

impl PairTarget {
    pub fn new(peer_instance_id: impl Into<String>, peer_core_node: impl Into<String>) -> Self {
        Self {
            peer: ProducerRef::new(peer_core_node, peer_instance_id),
            peer_link_id: None,
        }
    }

    pub fn pinned(
        peer_instance_id: impl Into<String>,
        peer_link_id: impl Into<String>,
        peer_core_node: impl Into<String>,
    ) -> Self {
        Self {
            peer: ProducerRef::new(peer_core_node, peer_instance_id),
            peer_link_id: Some(peer_link_id.into()),
        }
    }
}

impl std::fmt::Display for PairTarget {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.peer_link_id {
            Some(link) => write!(f, "{}/{}", self.peer.instance_id, link),
            None => f.write_str(&self.peer.instance_id),
        }
    }
}

/// One resolved observer request carried by [`NodeRunGoal::planned_observations`],
/// keyed in the goal by the starting node's own observer-slot link_id: the
/// source instance the slot taps and the source-side participant slot the
/// source publishes the observed role under. Unlike a [`PairTarget`] the source
/// slot is always resolved (the planner fills it).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationTarget {
    /// The source instance, addressed for the same self-describing-placement
    /// reason as [`PairTarget::peer`]. A remote source subscribes identically
    /// to a local one; what differs is that its lifecycle transitions arrive
    /// as notifications from its own daemon rather than from local lifecycle
    /// events.
    pub source: ProducerRef,
    pub source_link_id: String,
}

impl ObservationTarget {
    pub fn new(
        source_instance_id: impl Into<String>,
        source_link_id: impl Into<String>,
        source_core_node: impl Into<String>,
    ) -> Self {
        Self {
            source: ProducerRef::new(source_core_node, source_instance_id),
            source_link_id: source_link_id.into(),
        }
    }
}

/// Goal message for the NodeRun action.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeRunGoal {
    /// What to start, NOT how it is wired into the mesh. See
    /// [`NodeInstancePlan`]: the receiving daemon owns the runtime identity of
    /// every node it spawns, so the messaging endpoint, `bound_core_node`, and
    /// the resolved framework values are added by the daemon, on every path
    /// including `peppy node run`. One assembly site, one invariant.
    pub instance_plan: NodeInstancePlan,
    /// SHA256 of the manifest the planner validated this instance against.
    /// The spawning daemon refuses if its own re-resolved manifest hashes
    /// differently, so a cache that moved between a federated preflight and
    /// this dispatch fails loudly rather than starting a node against a plan
    /// that was never checked for it.
    ///
    /// `None` on the in-process launch path, where planner and spawner are the
    /// same daemon reading the same entity under the same lock and there is no
    /// window to close.
    pub manifest_sha256: Option<String>,
    pub node_name: String,
    pub tag: String,
    pub env_vars: Vec<(String, String)>,
    pub timeout_secs: u64,
    /// Pairing requests from `--pair <link_id>@<peer_instance>[/<peer_link_id>]`
    /// or a launch plan, keyed by the starting node's own slot link_id.
    /// Commands to the daemon, not resolved config: the daemon validates and
    /// reserves each pair BEFORE spawning and delivers it live after the
    /// instance commits to Running.
    pub requested_pairs: BTreeMap<String, PairTarget>,
    /// Pairing slot link_ids deliberately left unpaired via `--defer-pair` /
    /// the launcher's `defer_pairings:`. Together with `requested_pairs` and
    /// `covered_pairs` these must cover every required pairing slot of the
    /// manifest, or the daemon rejects the run.
    pub deferred_pairs: Vec<String>,
    /// Pairing slots of this instance that a LATER-starting instance of the
    /// same `stack launch` will claim through its own `requested_pairs`
    /// entry, keyed by this instance's slot link_id; each value names that
    /// future peer. A launch-mechanism marker, not user intent: the slot
    /// boots unpaired and needs no action, unlike a `deferred_pairs` entry
    /// which records a deliberate opt-out. Never set by the CLI.
    pub covered_pairs: BTreeMap<String, PairTarget>,
    /// Observer requests from `--link <observer_link>@<source>[/<source_link>]`
    /// or a launch plan, keyed by the starting node's own observer-slot
    /// link_id. Commands to the daemon, not resolved config: the daemon
    /// registers each with its observation coordinator BEFORE the instance
    /// commits to Running, so the source pin is delivered the moment both are
    /// up and re-delivered whenever the source restarts.
    pub planned_observations: BTreeMap<String, ObservationTarget>,
    /// See [`crate::encoding::NodeAddGoal::launch_id`].
    pub launch_id: Option<String>,
    /// Core nodes that must hear about this instance's lifecycle but that the
    /// spawning daemon cannot work out for itself: the daemons whose observers
    /// tap it. A source is deliberately unaware of its observers, so only the
    /// planner can name them. Pairing recipients are NOT listed here; a pair
    /// names both endpoints, so the daemon derives those from its own registry.
    pub lifecycle_watchers: Vec<String>,
}

impl NodeRunGoal {
    pub fn new(
        instance_plan: NodeInstancePlan,
        node_name: impl Into<String>,
        tag: impl Into<String>,
        timeout_secs: u64,
    ) -> Self {
        Self {
            instance_plan,
            manifest_sha256: None,
            node_name: node_name.into(),
            tag: tag.into(),
            env_vars: Vec::new(),
            timeout_secs,
            requested_pairs: BTreeMap::new(),
            deferred_pairs: Vec::new(),
            covered_pairs: BTreeMap::new(),
            planned_observations: BTreeMap::new(),
            launch_id: None,
            lifecycle_watchers: Vec::new(),
        }
    }

    /// Pins the manifest the planner validated against. Set by a coordinator
    /// dispatching to a peer; left unset on the in-process launch path.
    pub fn with_manifest_sha256(mut self, manifest_sha256: impl Into<String>) -> Self {
        self.manifest_sha256 = Some(manifest_sha256.into());
        self
    }

    pub fn with_env_vars(mut self, env_vars: Vec<(String, String)>) -> Self {
        self.env_vars = env_vars;
        self
    }

    pub fn with_requested_pairs(mut self, requested_pairs: BTreeMap<String, PairTarget>) -> Self {
        self.requested_pairs = requested_pairs;
        self
    }

    pub fn with_deferred_pairs(mut self, deferred_pairs: Vec<String>) -> Self {
        self.deferred_pairs = deferred_pairs;
        self
    }

    pub fn with_covered_pairs(mut self, covered_pairs: BTreeMap<String, PairTarget>) -> Self {
        self.covered_pairs = covered_pairs;
        self
    }

    pub fn with_planned_observations(
        mut self,
        planned_observations: BTreeMap<String, ObservationTarget>,
    ) -> Self {
        self.planned_observations = planned_observations;
        self
    }

    /// Marks this goal as one step of a federated launch's dispatch, so the
    /// receiving daemon accepts it while reserved for that launch.
    pub fn with_launch_id(mut self, launch_id: impl Into<String>) -> Self {
        self.launch_id = Some(launch_id.into());
        self
    }

    /// Names the daemons whose observers tap this instance, so the daemon that
    /// spawns it can report its lifecycle to them.
    pub fn with_lifecycle_watchers(mut self, lifecycle_watchers: Vec<String>) -> Self {
        self.lifecycle_watchers = lifecycle_watchers;
        self
    }

    /// Builds a goal for in-process execution that bypasses the action-loop
    /// gate (see `services::stack::launch::start_node_directly`). The
    /// `timeout_secs` field feeds the gate's busy-reporting and is unread on
    /// this path, so it is zero by construction.
    pub fn for_internal_execution(
        instance_plan: NodeInstancePlan,
        node_name: impl Into<String>,
        tag: impl Into<String>,
    ) -> Self {
        Self::new(instance_plan, node_name, tag, 0)
    }

    pub fn encode(&self) -> Result<Payload> {
        let instance_plan_json5 = serde_json5::to_string(&self.instance_plan)
            .map_err(|e| crate::Error::Encoding(format!("NodeRunGoal.instance_plan: {e}")))?;
        let mut builder = Builder::new_default();
        {
            let mut goal = builder.init_root::<node_capnp::node_run_goal::Builder>();
            goal.set_instance_plan_json5(&instance_plan_json5);
            goal.set_manifest_sha256(self.manifest_sha256.as_deref().unwrap_or(""));
            goal.set_launch_id(self.launch_id.as_deref().unwrap_or(""));

            let watcher_count = capnp_list_len(
                self.lifecycle_watchers.len(),
                "NodeRunGoal.lifecycle_watchers",
            )?;
            write_text_list(
                goal.reborrow().init_lifecycle_watchers(watcher_count),
                &self.lifecycle_watchers,
            );
            goal.set_node_name(&self.node_name);
            goal.set_tag(&self.tag);

            let env_var_count = capnp_list_len(self.env_vars.len(), "NodeRunGoal.env_vars")?;
            let mut env_vars = goal.reborrow().init_env_vars(env_var_count);
            for (idx, (key, value)) in self.env_vars.iter().enumerate() {
                let mut env_var = env_vars.reborrow().get(idx as u32);
                env_var.set_key(key);
                env_var.set_value(value);
            }

            goal.reborrow().set_timeout_secs(self.timeout_secs);

            let pair_count =
                capnp_list_len(self.requested_pairs.len(), "NodeRunGoal.requested_pairs")?;
            fill_pair_requests(
                goal.reborrow().init_requested_pairs(pair_count),
                &self.requested_pairs,
            );

            let deferred_count =
                capnp_list_len(self.deferred_pairs.len(), "NodeRunGoal.deferred_pairs")?;
            write_text_list(
                goal.reborrow().init_deferred_pairs(deferred_count),
                &self.deferred_pairs,
            );

            let covered_count =
                capnp_list_len(self.covered_pairs.len(), "NodeRunGoal.covered_pairs")?;
            fill_pair_requests(
                goal.reborrow().init_covered_pairs(covered_count),
                &self.covered_pairs,
            );

            let observation_count = capnp_list_len(
                self.planned_observations.len(),
                "NodeRunGoal.planned_observations",
            )?;
            fill_observation_requests(
                goal.reborrow().init_planned_observations(observation_count),
                &self.planned_observations,
            );
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let goal = reader.get_root::<node_capnp::node_run_goal::Reader>()?;

        let env_vars_reader = goal.get_env_vars()?;
        let mut env_vars = Vec::with_capacity(env_vars_reader.len() as usize);
        for idx in 0..env_vars_reader.len() {
            let env_var = env_vars_reader.get(idx);
            env_vars.push((
                env_var.get_key()?.to_str()?.to_owned(),
                env_var.get_value()?.to_str()?.to_owned(),
            ));
        }

        // Cap'n Proto defaults an absent field to the empty string, so a goal
        // from a peppy that still ships an assembled runtime config decodes
        // here with no plan at all. Left unchecked that would spawn a node
        // under a defaulted identity. Refuse, and name the fix.
        let instance_plan_json5 = goal.get_instance_plan_json5()?.to_str()?;
        if instance_plan_json5.is_empty() {
            return Err(crate::Error::Decoding(
                "NodeRunGoal.instance_plan is empty: this goal came from a peppy that still \
                 assembles runtime configs caller-side. Upgrade the caller to the same version \
                 as this daemon."
                    .to_owned(),
            ));
        }
        let instance_plan: NodeInstancePlan = serde_json5::from_str(instance_plan_json5)
            .map_err(|e| crate::Error::Decoding(format!("NodeRunGoal.instance_plan: {e}")))?;

        Ok(Self {
            instance_plan,
            manifest_sha256: optional_text(goal.get_manifest_sha256()?.to_str()?),
            launch_id: optional_text(goal.get_launch_id()?.to_str()?),
            lifecycle_watchers: read_text_list(goal.get_lifecycle_watchers()?)?,
            node_name: goal.get_node_name()?.to_str()?.to_owned(),
            tag: goal.get_tag()?.to_str()?.to_owned(),
            env_vars,
            timeout_secs: goal.get_timeout_secs(),
            requested_pairs: read_pair_requests(goal.get_requested_pairs()?)?,
            deferred_pairs: read_text_list(goal.get_deferred_pairs()?)?,
            covered_pairs: read_pair_requests(goal.get_covered_pairs()?)?,
            planned_observations: read_observation_requests(goal.get_planned_observations()?)?,
        })
    }
}

/// Writes a `link_id -> PairTarget` map into an initialized
/// `List(PairRequest)` builder ([`NodeRunGoal::requested_pairs`] and
/// [`NodeRunGoal::covered_pairs`] share the wire shape). An unpinned
/// `peer_link_id` is encoded as the empty string.
fn fill_pair_requests(
    mut list: capnp::struct_list::Builder<'_, node_capnp::pair_request::Owned>,
    pairs: &BTreeMap<String, PairTarget>,
) {
    for (idx, (link_id, target)) in pairs.iter().enumerate() {
        let mut pair = list.reborrow().get(idx as u32);
        pair.set_link_id(link_id);
        pair.set_peer_link_id(target.peer_link_id.as_deref().unwrap_or(""));
        write_instance_address(pair.init_peer(), &target.peer);
    }
}

/// Inverse of [`fill_pair_requests`]: an empty `peerLinkId` decodes to
/// `None` (unpinned).
fn read_pair_requests(
    list: capnp::struct_list::Reader<'_, node_capnp::pair_request::Owned>,
) -> Result<BTreeMap<String, PairTarget>> {
    let mut pairs = BTreeMap::new();
    for idx in 0..list.len() {
        let pair = list.get(idx);
        pairs.insert(
            pair.get_link_id()?.to_str()?.to_owned(),
            PairTarget {
                peer: read_instance_address(pair.get_peer()?, "NodeRunGoal pair request")?,
                peer_link_id: optional_text(pair.get_peer_link_id()?.to_str()?),
            },
        );
    }
    Ok(pairs)
}

/// Writes a [`ProducerRef`] into an initialized `InstanceAddress` builder.
fn write_instance_address(
    mut address: node_capnp::instance_address::Builder<'_>,
    producer: &ProducerRef,
) {
    address.set_core_node(&producer.core_node);
    address.set_instance_id(&producer.instance_id);
}

/// Inverse of [`write_instance_address`].
///
/// Both halves are required. Every wire message a peer acts on is
/// self-describing about placement, so an absent core node is a bug in the
/// sender rather than a shorthand for "local" — refusing here is what stops a
/// daemon from ever having to guess. `context` names the message, since the
/// address itself cannot say which one carried it.
fn read_instance_address(
    address: node_capnp::instance_address::Reader<'_>,
    context: &str,
) -> Result<ProducerRef> {
    let core_node = address.get_core_node()?.to_str()?;
    if core_node.is_empty() {
        return Err(crate::Error::Decoding(format!(
            "{context} carries an empty core_node: placement must be explicit, \
             even when the target is on the receiving daemon"
        )));
    }
    Ok(ProducerRef::new(
        core_node,
        required_text(
            address.get_instance_id()?.to_str()?,
            &format!("{context} instance_id"),
        )?,
    ))
}

/// Writes an `observer_link_id -> ObservationTarget` map into an initialized
/// `List(ObservationRequest)` builder ([`NodeRunGoal::planned_observations`]).
fn fill_observation_requests(
    mut list: capnp::struct_list::Builder<'_, node_capnp::observation_request::Owned>,
    observations: &BTreeMap<String, ObservationTarget>,
) {
    for (idx, (observer_link_id, target)) in observations.iter().enumerate() {
        let mut observation = list.reborrow().get(idx as u32);
        observation.set_observer_link_id(observer_link_id);
        observation.set_source_link_id(&target.source_link_id);
        write_instance_address(observation.init_source(), &target.source);
    }
}

/// Inverse of [`fill_observation_requests`].
fn read_observation_requests(
    list: capnp::struct_list::Reader<'_, node_capnp::observation_request::Owned>,
) -> Result<BTreeMap<String, ObservationTarget>> {
    let mut observations = BTreeMap::new();
    for idx in 0..list.len() {
        let observation = list.get(idx);
        observations.insert(
            observation.get_observer_link_id()?.to_str()?.to_owned(),
            ObservationTarget {
                source: read_instance_address(
                    observation.get_source()?,
                    "NodeRunGoal observation request",
                )?,
                source_link_id: observation.get_source_link_id()?.to_str()?.to_owned(),
            },
        );
    }
    Ok(observations)
}

/// Response to the NodeRun goal request.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeRunGoalResponse {
    pub accepted: bool,
    pub log_path: PathBuf,
    pub rejection_reason: Option<String>,
}

impl NodeRunGoalResponse {
    pub fn accepted(log_path: impl Into<PathBuf>) -> Self {
        Self {
            accepted: true,
            log_path: log_path.into(),
            rejection_reason: None,
        }
    }

    pub fn rejected(reason: impl Into<String>) -> Self {
        Self {
            accepted: false,
            log_path: PathBuf::new(),
            rejection_reason: Some(reason.into()),
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut response = builder.init_root::<node_capnp::node_run_goal_response::Builder>();
            response.set_accepted(self.accepted);
            response.set_log_path(self.log_path.to_string_lossy().as_ref());
            if let Some(ref reason) = self.rejection_reason {
                response.set_rejection_reason(reason);
            }
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let response = reader.get_root::<node_capnp::node_run_goal_response::Reader>()?;
        Ok(Self {
            accepted: response.get_accepted(),
            log_path: PathBuf::from(response.get_log_path()?.to_str()?),
            rejection_reason: optional_text(response.get_rejection_reason()?.to_str()?),
        })
    }
}

/// Feedback message for the NodeRun action.
/// Represents a single line of output from the run_cmd process.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeRunFeedback {
    pub stream: FeedbackStream,
    /// The line of output
    pub line: String,
}

impl NodeRunFeedback {
    pub fn from_stream(stream: FeedbackStream, line: impl Into<String>) -> Self {
        Self {
            stream,
            line: line.into(),
        }
    }

    pub fn stdout(line: impl Into<String>) -> Self {
        Self::from_stream(FeedbackStream::Stdout, line)
    }

    pub fn stderr(line: impl Into<String>) -> Self {
        Self::from_stream(FeedbackStream::Stderr, line)
    }

    pub fn warning(line: impl Into<String>) -> Self {
        Self::from_stream(FeedbackStream::Warning, line)
    }

    pub fn is_stdout(&self) -> bool {
        self.stream == FeedbackStream::Stdout
    }

    pub fn is_stderr(&self) -> bool {
        self.stream == FeedbackStream::Stderr
    }

    pub fn is_warning(&self) -> bool {
        self.stream == FeedbackStream::Warning
    }

    pub fn encode(&self) -> Result<NonEmptyPayload> {
        let mut builder = Builder::new_default();
        {
            let mut feedback = builder.init_root::<node_capnp::node_run_feedback::Builder>();
            feedback.set_stream(self.stream.to_capnp());
            feedback.set_line(&self.line);
        }
        encode_message_non_empty(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let feedback = reader.get_root::<node_capnp::node_run_feedback::Reader>()?;
        Ok(Self {
            stream: FeedbackStream::from_capnp(feedback.get_stream()?),
            line: feedback.get_line()?.to_str()?.to_owned(),
        })
    }
}

/// Result message for the NodeRun action.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeRunResult {
    pub success: bool,
    pub error_message: Option<String>,
    /// Process ID of the started node (None if not available or failed).
    pub pid: Option<u32>,
}

impl NodeRunResult {
    pub fn new(success: bool, error_message: Option<String>, pid: Option<u32>) -> Self {
        Self {
            success,
            error_message,
            pid,
        }
    }

    pub fn success(pid: u32) -> Self {
        Self::new(true, None, Some(pid))
    }

    pub fn failure(error_message: impl Into<String>) -> Self {
        Self::new(false, Some(error_message.into()), None)
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut result = builder.init_root::<node_capnp::node_run_result::Builder>();
            result.set_success(self.success);
            if let Some(ref error_message) = self.error_message {
                result.set_error_message(error_message);
            }
            result.set_pid(self.pid.unwrap_or(0));
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let result = reader.get_root::<node_capnp::node_run_result::Reader>()?;
        let error_message = optional_text(result.get_error_message()?.to_str()?);
        let pid_value = result.get_pid();
        let pid = if pid_value == 0 {
            None
        } else {
            Some(pid_value)
        };
        Ok(Self {
            success: result.get_success(),
            error_message,
            pid,
        })
    }
}

impl crate::encoding::Wire for NodeRunGoal {
    type Root = crate::node_capnp::node_run_goal::Owned;
}

impl crate::encoding::Wire for NodeRunGoalResponse {
    type Root = crate::node_capnp::node_run_goal_response::Owned;
}

impl crate::encoding::Wire for NodeRunFeedback {
    type Root = crate::node_capnp::node_run_feedback::Owned;
}

impl crate::encoding::Wire for NodeRunResult {
    type Root = crate::node_capnp::node_run_result::Owned;
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- NodeRunGoal ---

    fn plan(instance_id: &str) -> NodeInstancePlan {
        NodeInstancePlan::new(config::runtime::Name::new(instance_id).expect("valid name"))
    }

    #[test]
    fn node_run_goal_new_has_empty_env_vars() {
        let goal = NodeRunGoal::new(plan("inst_1"), "node", "tag", 30);
        assert_eq!(goal.instance_plan.instance_id.as_str(), "inst_1");
        assert_eq!(goal.node_name, "node");
        assert_eq!(goal.tag, "tag");
        assert!(goal.env_vars.is_empty());
        assert_eq!(goal.timeout_secs, 30);
        assert_eq!(goal.manifest_sha256, None);
    }

    #[test]
    fn node_run_goal_roundtrip_empty_env_vars() {
        let goal = NodeRunGoal::new(plan("inst_1"), "node", "tag", 30);
        let encoded = goal.encode().expect("encode");
        let decoded = NodeRunGoal::decode(&encoded).expect("decode");
        assert_eq!(decoded, goal);
        assert!(decoded.env_vars.is_empty());
    }

    #[test]
    fn node_run_goal_roundtrip_instance_plan_fields() {
        let goal = NodeRunGoal::new(
            NodeInstancePlan {
                use_sim_time: Some(true),
                slot_bindings: BTreeMap::from([(
                    "camera".to_owned(),
                    config::runtime::BoundProducers::try_from(vec![
                        config::runtime::ProducerRef::new("cn-robot-7", "wrist_cam_inst"),
                    ])
                    .expect("one producer is a valid set"),
                )]),
                ..plan("planner_inst")
            },
            "deliberative_planner",
            "v1",
            30,
        )
        .with_manifest_sha256("a".repeat(64));

        let encoded = goal.encode().expect("encode");
        let decoded = NodeRunGoal::decode(&encoded).expect("decode");
        assert_eq!(decoded, goal);
        assert_eq!(decoded.instance_plan.use_sim_time, Some(true));
        assert_eq!(
            decoded.instance_plan.slot_bindings["camera"].as_slice()[0].core_node,
            "cn-robot-7"
        );
        assert_eq!(decoded.manifest_sha256, Some("a".repeat(64)));
    }

    /// A cross-daemon producer binding is carried the same way a local one is:
    /// the `ProducerRef` already names its core node, so nothing at the point
    /// of use records which machine the producer sits on.
    #[test]
    fn node_run_goal_carries_producers_on_other_core_nodes() {
        let goal = NodeRunGoal::new(
            NodeInstancePlan {
                slot_bindings: BTreeMap::from([(
                    "scene".to_owned(),
                    config::runtime::BoundProducers::try_from(vec![
                        config::runtime::ProducerRef::new("cn-robot-7", "wrist_cam_inst"),
                    ])
                    .expect("one producer is a valid set"),
                )]),
                ..plan("planner_inst")
            },
            "deliberative_planner",
            "v1",
            0,
        );
        let decoded = NodeRunGoal::decode(&goal.encode().expect("encode")).expect("decode");
        assert_eq!(decoded, goal);
    }

    #[test]
    fn node_run_goal_roundtrip_pairs() {
        let goal = NodeRunGoal::new(plan("inst_1"), "node", "tag", 30)
            .with_requested_pairs(
                [
                    ("arm".to_owned(), PairTarget::new("arm_1", "cn-local")),
                    (
                        "gripper".to_owned(),
                        PairTarget::pinned("grip_1", "controller", "cn-local"),
                    ),
                    // The cross-daemon case: same shape, different core node.
                    (
                        "deliberation".to_owned(),
                        PairTarget::pinned("planner_inst", "deliberator", "cn-atlas-h100"),
                    ),
                ]
                .into_iter()
                .collect(),
            )
            .with_deferred_pairs(vec!["spare".to_owned()])
            .with_covered_pairs(
                [(
                    "left".to_owned(),
                    PairTarget::pinned("cmd_1", "left_arm", "cn-local"),
                )]
                .into_iter()
                .collect(),
            )
            .with_planned_observations(
                [(
                    "observed_arm".to_owned(),
                    ObservationTarget::new("arm_1", "controller", "cn-local"),
                )]
                .into_iter()
                .collect(),
            );
        let encoded = goal.encode().expect("encode");
        let decoded = NodeRunGoal::decode(&encoded).expect("decode");
        assert_eq!(decoded, goal);
        // An unpinned target's empty peerLinkId decodes back to None.
        assert_eq!(decoded.requested_pairs["arm"].peer_link_id, None);
        assert_eq!(
            decoded.requested_pairs["gripper"].peer_link_id.as_deref(),
            Some("controller")
        );
        assert_eq!(
            decoded.requested_pairs["deliberation"].peer.core_node,
            "cn-atlas-h100"
        );
        assert_eq!(decoded.deferred_pairs, vec!["spare".to_owned()]);
        assert_eq!(
            decoded.covered_pairs["left"],
            PairTarget::pinned("cmd_1", "left_arm", "cn-local")
        );
        assert_eq!(
            decoded.planned_observations["observed_arm"],
            ObservationTarget::new("arm_1", "controller", "cn-local")
        );
    }

    /// Placement is never inferred from an absent field, so a pair or
    /// observation whose core node did not survive encoding is refused rather
    /// than quietly read as "local".
    #[test]
    fn node_run_goal_decode_rejects_pair_without_a_core_node() {
        let goal = NodeRunGoal::new(plan("inst_1"), "node", "tag", 0).with_requested_pairs(
            [("arm".to_owned(), PairTarget::new("arm_1", ""))]
                .into_iter()
                .collect(),
        );
        let encoded = goal.encode().expect("encode");
        let error = NodeRunGoal::decode(&encoded).expect_err("empty core node must fail");
        assert!(
            error.to_string().contains("empty core_node"),
            "got: {error}"
        );
    }

    #[test]
    fn node_run_goal_decode_rejects_observation_without_a_core_node() {
        let goal = NodeRunGoal::new(plan("inst_1"), "node", "tag", 0).with_planned_observations(
            [(
                "observed".to_owned(),
                ObservationTarget::new("arm_1", "controller", ""),
            )]
            .into_iter()
            .collect(),
        );
        let encoded = goal.encode().expect("encode");
        let error = NodeRunGoal::decode(&encoded).expect_err("empty core node must fail");
        assert!(
            error.to_string().contains("empty core_node"),
            "got: {error}"
        );
    }

    /// The break with pre-federation callers is enforced here, not by the
    /// codec: Cap'n Proto defaults an absent field to the empty string, so a
    /// goal that still ships an assembled runtime config would otherwise
    /// decode into a defaulted plan and spawn a node under the wrong identity.
    #[test]
    fn node_run_goal_decode_rejects_a_goal_carrying_no_instance_plan() {
        let mut builder = Builder::new_default();
        {
            let mut goal = builder.init_root::<node_capnp::node_run_goal::Builder>();
            goal.set_node_name("node");
            goal.set_tag("tag");
        }
        let encoded = crate::encoding::encode_message(&builder).expect("encode");
        let error = NodeRunGoal::decode(&encoded).expect_err("a goal with no plan must fail");
        assert!(
            error.to_string().contains("instance_plan is empty"),
            "got: {error}"
        );
        assert!(
            error.to_string().contains("assembles runtime configs"),
            "the message must name the version gap, got: {error}"
        );
    }

    /// Pins tenet 9 at the type level: a daemon owns the runtime identity of
    /// every node it spawns, so nothing a requester sends may name a messaging
    /// endpoint or a daemon. This holds for `peppy node run` exactly as it does
    /// for a federated dispatch, which is what makes the invariant checkable in
    /// one place instead of two.
    #[test]
    fn an_instance_plan_names_no_endpoint_and_no_daemon() {
        let serialized = serde_json5::to_string(&NodeInstancePlan {
            use_sim_time: Some(false),
            ..plan("inst_1")
        })
        .expect("serialize");

        for forbidden in [
            "messaging_host",
            "messaging_port",
            "bound_core_node",
            "discovery",
            "lifecycle",
        ] {
            assert!(
                !serialized.contains(forbidden),
                "an instance plan must not carry `{forbidden}`, got: {serialized}"
            );
        }
    }

    /// `Display` renders the CLI/launcher target grammar, the format fed to
    /// the shared pairing validator. Placement is deliberately absent from it:
    /// a target names an instance, and where that instance runs is declared
    /// once on the instance, never repeated at the point of use.
    #[test]
    fn pair_target_display_matches_target_grammar() {
        assert_eq!(PairTarget::new("arm_1", "cn-local").to_string(), "arm_1");
        assert_eq!(
            PairTarget::pinned("cmd_1", "left_arm", "cn-atlas").to_string(),
            "cmd_1/left_arm"
        );
    }

    #[test]
    fn node_run_goal_roundtrip_populated_env_vars() {
        let goal = NodeRunGoal::new(plan("inst_1"), "node", "tag", 42).with_env_vars(vec![
            ("KEY1".to_owned(), "VAL1".to_owned()),
            ("KEY2".to_owned(), "VAL2".to_owned()),
        ]);
        let encoded = goal.encode().expect("encode");
        let decoded = NodeRunGoal::decode(&encoded).expect("decode");
        assert_eq!(decoded, goal);
        assert_eq!(
            decoded.env_vars,
            vec![
                ("KEY1".to_owned(), "VAL1".to_owned()),
                ("KEY2".to_owned(), "VAL2".to_owned()),
            ]
        );
    }

    #[test]
    fn node_run_goal_for_internal_execution_has_zero_timeout() {
        let goal = NodeRunGoal::for_internal_execution(plan("inst_1"), "node", "tag");
        assert_eq!(goal.timeout_secs, 0);
        assert!(goal.env_vars.is_empty());
        // The in-process path has no preflight/dispatch window to close, so it
        // pins no manifest.
        assert_eq!(goal.manifest_sha256, None);
        let encoded = goal.encode().expect("encode");
        let decoded = NodeRunGoal::decode(&encoded).expect("decode");
        assert_eq!(decoded, goal);
    }

    /// A dispatched start names the launch that reserved the machine; a
    /// user-typed one names nothing. The receiving daemon tells them apart by
    /// exactly this field, so it has to survive the wire.
    #[test]
    fn node_run_goal_round_trips_the_launch_it_belongs_to() {
        let dispatched = NodeRunGoal::new(plan("inst_1"), "node", "tag", 30)
            .with_launch_id("launch-abc123")
            .with_lifecycle_watchers(vec!["cn-atlas".to_owned()]);
        let decoded = NodeRunGoal::decode(&dispatched.encode().expect("encode")).expect("decode");
        assert_eq!(decoded.launch_id.as_deref(), Some("launch-abc123"));
        assert_eq!(decoded.lifecycle_watchers, ["cn-atlas"]);
        assert_eq!(decoded, dispatched);

        let typed = NodeRunGoal::new(plan("inst_1"), "node", "tag", 30);
        let decoded = NodeRunGoal::decode(&typed.encode().expect("encode")).expect("decode");
        assert_eq!(
            decoded.launch_id, None,
            "a goal nobody dispatched must not claim a launch"
        );
        assert!(decoded.lifecycle_watchers.is_empty());
    }

    #[test]
    fn node_run_goal_decode_rejects_malformed_bytes() {
        assert!(NodeRunGoal::decode(&[0xde, 0xad, 0xbe, 0xef]).is_err());
    }

    // --- NodeRunGoalResponse ---

    #[test]
    fn node_run_goal_response_accepted_roundtrip() {
        let response = NodeRunGoalResponse::accepted("/var/log/run.log");
        assert!(response.accepted);
        assert_eq!(response.log_path, PathBuf::from("/var/log/run.log"));
        assert_eq!(response.rejection_reason, None);
        let encoded = response.encode().expect("encode");
        let decoded = NodeRunGoalResponse::decode(&encoded).expect("decode");
        assert_eq!(decoded, response);
    }

    #[test]
    fn node_run_goal_response_rejected_roundtrip() {
        let response = NodeRunGoalResponse::rejected("busy");
        assert!(!response.accepted);
        assert_eq!(response.log_path, PathBuf::new());
        assert_eq!(response.rejection_reason, Some("busy".to_owned()));
        let encoded = response.encode().expect("encode");
        let decoded = NodeRunGoalResponse::decode(&encoded).expect("decode");
        assert_eq!(decoded, response);
    }

    #[test]
    fn node_run_goal_response_decode_rejects_malformed_bytes() {
        assert!(NodeRunGoalResponse::decode(&[0xde, 0xad, 0xbe, 0xef]).is_err());
    }

    // --- NodeRunFeedback ---

    #[test]
    fn node_run_feedback_from_stream_roundtrip() {
        let feedback = NodeRunFeedback::from_stream(FeedbackStream::Stdout, "line");
        assert_eq!(feedback.stream, FeedbackStream::Stdout);
        assert_eq!(feedback.line, "line");
        let encoded = feedback.encode().expect("encode");
        let decoded = NodeRunFeedback::decode(&encoded.into_inner()).expect("decode");
        assert_eq!(decoded, feedback);
    }

    #[test]
    fn node_run_feedback_stdout_predicates() {
        let feedback = NodeRunFeedback::stdout("out");
        assert!(feedback.is_stdout());
        assert!(!feedback.is_stderr());
        assert!(!feedback.is_warning());
        let encoded = feedback.encode().expect("encode");
        let decoded = NodeRunFeedback::decode(&encoded.into_inner()).expect("decode");
        assert_eq!(decoded, feedback);
        assert!(decoded.is_stdout());
    }

    #[test]
    fn node_run_feedback_stderr_predicates() {
        let feedback = NodeRunFeedback::stderr("err");
        assert!(!feedback.is_stdout());
        assert!(feedback.is_stderr());
        assert!(!feedback.is_warning());
        let encoded = feedback.encode().expect("encode");
        let decoded = NodeRunFeedback::decode(&encoded.into_inner()).expect("decode");
        assert_eq!(decoded, feedback);
        assert!(decoded.is_stderr());
    }

    #[test]
    fn node_run_feedback_warning_predicates() {
        let feedback = NodeRunFeedback::warning("warn");
        assert!(!feedback.is_stdout());
        assert!(!feedback.is_stderr());
        assert!(feedback.is_warning());
        let encoded = feedback.encode().expect("encode");
        let decoded = NodeRunFeedback::decode(&encoded.into_inner()).expect("decode");
        assert_eq!(decoded, feedback);
        assert!(decoded.is_warning());
    }

    #[test]
    fn node_run_feedback_decode_rejects_malformed_bytes() {
        assert!(NodeRunFeedback::decode(&[0xde, 0xad, 0xbe, 0xef]).is_err());
    }

    // --- NodeRunResult ---

    #[test]
    fn node_run_result_new_roundtrip() {
        let result = NodeRunResult::new(true, Some("warn".to_owned()), Some(7));
        assert!(result.success);
        assert_eq!(result.error_message, Some("warn".to_owned()));
        assert_eq!(result.pid, Some(7));
        let encoded = result.encode().expect("encode");
        let decoded = NodeRunResult::decode(&encoded).expect("decode");
        assert_eq!(decoded, result);
    }

    #[test]
    fn node_run_result_success_roundtrip() {
        let result = NodeRunResult::success(1234);
        assert!(result.success);
        assert_eq!(result.error_message, None);
        assert_eq!(result.pid, Some(1234));
        let encoded = result.encode().expect("encode");
        let decoded = NodeRunResult::decode(&encoded).expect("decode");
        assert_eq!(decoded, result);
    }

    #[test]
    fn node_run_result_failure_roundtrip() {
        let result = NodeRunResult::failure("boom");
        assert!(!result.success);
        assert_eq!(result.error_message, Some("boom".to_owned()));
        assert_eq!(result.pid, None);
        let encoded = result.encode().expect("encode");
        let decoded = NodeRunResult::decode(&encoded).expect("decode");
        assert_eq!(decoded, result);
    }

    #[test]
    fn node_run_result_decode_rejects_malformed_bytes() {
        assert!(NodeRunResult::decode(&[0xde, 0xad, 0xbe, 0xef]).is_err());
    }
}
