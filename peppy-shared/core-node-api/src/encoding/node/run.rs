//! Encoding types for the NodeRun action (streaming version with feedback).

use std::collections::BTreeMap;
use std::path::PathBuf;

use capnp::message::Builder;

use crate::node_capnp;
use crate::{NonEmptyPayload, Payload, Result};

use super::builder::FeedbackStream;
use crate::encoding::{
    capnp_list_len, decode_message, encode_message, encode_message_non_empty, optional_text,
};

/// One peer reference carried by [`NodeRunGoal::requested_pairs`] /
/// [`NodeRunGoal::covered_pairs`]: the peer instance and, optionally, the
/// pinned complementary slot on it. `Display` renders the CLI/launcher
/// target grammar (`<peer_instance>` or `<peer_instance>/<peer_link_id>`);
/// instance ids and link_ids are `/`-free names, so the rendering is
/// unambiguous.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PairTarget {
    pub peer_instance_id: String,
    /// The complementary slot on the peer, when the request pins one.
    /// `None` is unpinned: exactly one available complementary slot must
    /// exist on the peer and the daemon resolves it.
    pub peer_link_id: Option<String>,
}

impl PairTarget {
    pub fn new(peer_instance_id: impl Into<String>) -> Self {
        Self {
            peer_instance_id: peer_instance_id.into(),
            peer_link_id: None,
        }
    }

    pub fn pinned(peer_instance_id: impl Into<String>, peer_link_id: impl Into<String>) -> Self {
        Self {
            peer_instance_id: peer_instance_id.into(),
            peer_link_id: Some(peer_link_id.into()),
        }
    }
}

impl std::fmt::Display for PairTarget {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.peer_link_id {
            Some(link) => write!(f, "{}/{}", self.peer_instance_id, link),
            None => f.write_str(&self.peer_instance_id),
        }
    }
}

/// Goal message for the NodeRun action.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeRunGoal {
    pub runtime_config_json5: String,
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
}

impl NodeRunGoal {
    pub fn new(
        runtime_config_json5: impl Into<String>,
        node_name: impl Into<String>,
        tag: impl Into<String>,
        timeout_secs: u64,
    ) -> Self {
        Self {
            runtime_config_json5: runtime_config_json5.into(),
            node_name: node_name.into(),
            tag: tag.into(),
            env_vars: Vec::new(),
            timeout_secs,
            requested_pairs: BTreeMap::new(),
            deferred_pairs: Vec::new(),
            covered_pairs: BTreeMap::new(),
        }
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

    /// Builds a goal for in-process execution that bypasses the action-loop
    /// gate (see `services::stack::launch::start_node_directly`). The
    /// `timeout_secs` field feeds the gate's busy-reporting and is unread on
    /// this path, so it is zero by construction.
    pub fn for_internal_execution(
        runtime_config_json5: impl Into<String>,
        node_name: impl Into<String>,
        tag: impl Into<String>,
    ) -> Self {
        Self::new(runtime_config_json5, node_name, tag, 0)
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut goal = builder.init_root::<node_capnp::node_run_goal::Builder>();
            goal.set_runtime_config_json5(&self.runtime_config_json5);
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
            let mut deferred = goal.reborrow().init_deferred_pairs(deferred_count);
            for (idx, link_id) in self.deferred_pairs.iter().enumerate() {
                deferred.set(idx as u32, link_id.as_str());
            }

            let covered_count =
                capnp_list_len(self.covered_pairs.len(), "NodeRunGoal.covered_pairs")?;
            fill_pair_requests(
                goal.reborrow().init_covered_pairs(covered_count),
                &self.covered_pairs,
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

        let deferred_reader = goal.get_deferred_pairs()?;
        let mut deferred_pairs = Vec::with_capacity(deferred_reader.len() as usize);
        for idx in 0..deferred_reader.len() {
            deferred_pairs.push(deferred_reader.get(idx)?.to_str()?.to_owned());
        }

        Ok(Self {
            runtime_config_json5: goal.get_runtime_config_json5()?.to_str()?.to_owned(),
            node_name: goal.get_node_name()?.to_str()?.to_owned(),
            tag: goal.get_tag()?.to_str()?.to_owned(),
            env_vars,
            timeout_secs: goal.get_timeout_secs(),
            requested_pairs: read_pair_requests(goal.get_requested_pairs()?)?,
            deferred_pairs,
            covered_pairs: read_pair_requests(goal.get_covered_pairs()?)?,
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
        pair.set_peer_instance_id(&target.peer_instance_id);
        pair.set_peer_link_id(target.peer_link_id.as_deref().unwrap_or(""));
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
                peer_instance_id: pair.get_peer_instance_id()?.to_str()?.to_owned(),
                peer_link_id: optional_text(pair.get_peer_link_id()?.to_str()?),
            },
        );
    }
    Ok(pairs)
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

#[cfg(test)]
mod tests {
    use super::*;

    // --- NodeRunGoal ---

    #[test]
    fn node_run_goal_new_has_empty_env_vars() {
        let goal = NodeRunGoal::new("config", "node", "tag", 30);
        assert_eq!(goal.runtime_config_json5, "config");
        assert_eq!(goal.node_name, "node");
        assert_eq!(goal.tag, "tag");
        assert!(goal.env_vars.is_empty());
        assert_eq!(goal.timeout_secs, 30);
    }

    #[test]
    fn node_run_goal_roundtrip_empty_env_vars() {
        let goal = NodeRunGoal::new("config", "node", "tag", 30);
        let encoded = goal.encode().expect("encode");
        let decoded = NodeRunGoal::decode(&encoded).expect("decode");
        assert_eq!(decoded, goal);
        assert!(decoded.env_vars.is_empty());
    }

    #[test]
    fn node_run_goal_roundtrip_pairs() {
        let goal = NodeRunGoal::new("config", "node", "tag", 30)
            .with_requested_pairs(
                [
                    ("arm".to_owned(), PairTarget::new("arm_1")),
                    (
                        "gripper".to_owned(),
                        PairTarget::pinned("grip_1", "controller"),
                    ),
                ]
                .into_iter()
                .collect(),
            )
            .with_deferred_pairs(vec!["spare".to_owned()])
            .with_covered_pairs(
                [("left".to_owned(), PairTarget::pinned("cmd_1", "left_arm"))]
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
        assert_eq!(decoded.deferred_pairs, vec!["spare".to_owned()]);
        assert_eq!(
            decoded.covered_pairs["left"],
            PairTarget::pinned("cmd_1", "left_arm")
        );
    }

    /// `Display` renders the CLI/launcher target grammar, the format fed to
    /// the shared pairing validator.
    #[test]
    fn pair_target_display_matches_target_grammar() {
        assert_eq!(PairTarget::new("arm_1").to_string(), "arm_1");
        assert_eq!(
            PairTarget::pinned("cmd_1", "left_arm").to_string(),
            "cmd_1/left_arm"
        );
    }

    #[test]
    fn node_run_goal_roundtrip_populated_env_vars() {
        let goal = NodeRunGoal::new("config", "node", "tag", 42).with_env_vars(vec![
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
        let goal = NodeRunGoal::for_internal_execution("config", "node", "tag");
        assert_eq!(goal.timeout_secs, 0);
        assert!(goal.env_vars.is_empty());
        let encoded = goal.encode().expect("encode");
        let decoded = NodeRunGoal::decode(&encoded).expect("decode");
        assert_eq!(decoded, goal);
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
