//! Encoding types for the daemon-to-daemon federated-launch messages.
//!
//! See `schemas/federation.capnp` for why the reservation exchange and the
//! relationship notifications have deliberately different guarantees: the
//! former is coordinator-driven and must be exact, the latter is best-effort
//! and idempotent.

use capnp::message::Builder;
use config::runtime::ProducerRef;

use crate::federation_capnp;
use crate::{Payload, Result};

use crate::encoding::{
    capnp_list_len, decode_message, encode_message, optional_text, read_text_list, required_text,
    write_text_list,
};

/// Reserves one participant for one launch, carrying the pins for every
/// deployment placed on it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParticipantReserveRequest {
    pub launch_id: String,
    /// The coordinator driving the launch. The participant watches this core
    /// node's presence for as long as it holds the reservation.
    pub coordinator_core_node: String,
    /// JSON5-encoded `DeploymentPins`, one per deployment placed on this
    /// participant: the root node's pin plus the pin of every transitive
    /// node dependency and every contract and pairing document in its
    /// closure. Opaque here: the pin model lives in peppy, whose serde
    /// decoding is the validation, and this crate has no business
    /// re-deriving it.
    pub deployment_pins_json5: Vec<String>,
}

impl ParticipantReserveRequest {
    pub fn new(launch_id: impl Into<String>, coordinator_core_node: impl Into<String>) -> Self {
        Self {
            launch_id: launch_id.into(),
            coordinator_core_node: coordinator_core_node.into(),
            deployment_pins_json5: Vec::new(),
        }
    }

    pub fn with_deployment_pins(mut self, pins: Vec<String>) -> Self {
        self.deployment_pins_json5 = pins;
        self
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut request =
                builder.init_root::<federation_capnp::participant_reserve_request::Builder>();
            request.set_launch_id(&self.launch_id);
            request.set_coordinator_core_node(&self.coordinator_core_node);
            let count = capnp_list_len(
                self.deployment_pins_json5.len(),
                "ParticipantReserveRequest.deployment_pins_json5",
            )?;
            write_text_list(
                request.reborrow().init_deployment_pins_json5(count),
                &self.deployment_pins_json5,
            );
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let request = reader.get_root::<federation_capnp::participant_reserve_request::Reader>()?;

        Ok(Self {
            launch_id: required_text(request.get_launch_id()?.to_str()?, "launch_id")?,
            coordinator_core_node: required_text(
                request.get_coordinator_core_node()?.to_str()?,
                "coordinator_core_node",
            )?,
            deployment_pins_json5: read_text_list(request.get_deployment_pins_json5()?)?,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParticipantReserveResponse {
    pub accepted: bool,
    pub rejection_reason: Option<String>,
    /// The participant's peppy version, so a mixed-version federation is
    /// refused before any stack is touched. Same string the info service
    /// reports: one source of truth for "what version is that daemon".
    pub peppy_version: String,
    /// The participant's root entity instance id, folded into the
    /// coordinator's global instance-id uniqueness check.
    pub root_instance_id: String,
}

impl ParticipantReserveResponse {
    pub fn accepted(peppy_version: impl Into<String>, root_instance_id: impl Into<String>) -> Self {
        Self {
            accepted: true,
            rejection_reason: None,
            peppy_version: peppy_version.into(),
            root_instance_id: root_instance_id.into(),
        }
    }

    /// A refusal still reports the version, so a coordinator can tell "busy"
    /// apart from "too old" without a second round trip.
    pub fn rejected(reason: impl Into<String>, peppy_version: impl Into<String>) -> Self {
        Self {
            accepted: false,
            rejection_reason: Some(reason.into()),
            peppy_version: peppy_version.into(),
            root_instance_id: String::new(),
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut response =
                builder.init_root::<federation_capnp::participant_reserve_response::Builder>();
            response.set_accepted(self.accepted);
            response.set_rejection_reason(self.rejection_reason.as_deref().unwrap_or(""));
            response.set_peppy_version(&self.peppy_version);
            response.set_root_instance_id(&self.root_instance_id);
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let response =
            reader.get_root::<federation_capnp::participant_reserve_response::Reader>()?;

        Ok(Self {
            accepted: response.get_accepted(),
            rejection_reason: optional_text(response.get_rejection_reason()?.to_str()?),
            peppy_version: response.get_peppy_version()?.to_str()?.to_owned(),
            root_instance_id: response.get_root_instance_id()?.to_str()?.to_owned(),
        })
    }
}

/// Commits a reserved participant to replacing its stack slice, and hands it
/// the container bind sources that slice will need.
///
/// The destructive half of the exchange, deliberately split from the
/// reservation: reserving happens before the coordinator knows whether every
/// participant will accept, so folding a teardown into it would replace a
/// stack on one machine for a launch another is about to refuse.
///
/// [`Self::mount_sources`] rides along because this is the one moment the
/// participant's slice is empty; see the schema.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParticipantSliceBeginRequest {
    pub launch_id: String,
    pub mount_sources: Vec<String>,
}

impl ParticipantSliceBeginRequest {
    pub fn new(launch_id: impl Into<String>, mount_sources: Vec<String>) -> Self {
        Self {
            launch_id: launch_id.into(),
            mount_sources,
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut request =
                builder.init_root::<federation_capnp::participant_slice_begin_request::Builder>();
            request.set_launch_id(&self.launch_id);
            let count = capnp_list_len(
                self.mount_sources.len(),
                "ParticipantSliceBeginRequest.mount_sources",
            )?;
            write_text_list(
                request.reborrow().init_mount_sources(count),
                &self.mount_sources,
            );
        }
        encode_message(&builder)
    }

    /// An empty launch id is refused: the exchange acts on exactly one launch,
    /// and a defaulted id names none. An empty mount source is refused one
    /// level down, for the same reason: it names no host path, and the
    /// participant would resolve it against its own working directory.
    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let request =
            reader.get_root::<federation_capnp::participant_slice_begin_request::Reader>()?;
        Ok(Self {
            launch_id: required_text(request.get_launch_id()?.to_str()?, "launch_id")?,
            mount_sources: required_text_list(
                read_text_list(request.get_mount_sources()?)?,
                "mount_sources",
            )?,
        })
    }
}

/// Refuses an empty entry in a list whose members each name something: a
/// defaulted string in such a list is a hole, not a value.
fn required_text_list(values: Vec<String>, field: &str) -> Result<Vec<String>> {
    for (idx, value) in values.iter().enumerate() {
        required_text(value, &format!("{field}[{idx}]"))?;
    }
    Ok(values)
}

/// The reply to `participant_slice_begin`: the verdict, plus the bind sources
/// the participant had to create to honour it.
///
/// [`Self::auto_created_mount_sources`] is what the coordinator turns back into
/// the warning an operator would have seen had the instance run on their own
/// machine; see the schema.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParticipantSliceBeginResponse {
    pub ok: bool,
    pub rejection_reason: Option<String>,
    pub auto_created_mount_sources: Vec<String>,
}

impl ParticipantSliceBeginResponse {
    pub fn ok(auto_created_mount_sources: Vec<String>) -> Self {
        Self {
            ok: true,
            rejection_reason: None,
            auto_created_mount_sources,
        }
    }

    pub fn refused(reason: impl Into<String>) -> Self {
        Self {
            ok: false,
            rejection_reason: Some(reason.into()),
            auto_created_mount_sources: Vec::new(),
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut response =
                builder.init_root::<federation_capnp::participant_slice_begin_response::Builder>();
            response.set_ok(self.ok);
            response.set_rejection_reason(self.rejection_reason.as_deref().unwrap_or(""));
            let count = capnp_list_len(
                self.auto_created_mount_sources.len(),
                "ParticipantSliceBeginResponse.auto_created_mount_sources",
            )?;
            write_text_list(
                response.reborrow().init_auto_created_mount_sources(count),
                &self.auto_created_mount_sources,
            );
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let response =
            reader.get_root::<federation_capnp::participant_slice_begin_response::Reader>()?;
        Ok(Self {
            ok: response.get_ok(),
            rejection_reason: optional_text(response.get_rejection_reason()?.to_str()?),
            auto_created_mount_sources: required_text_list(
                read_text_list(response.get_auto_created_mount_sources()?)?,
                "auto_created_mount_sources",
            )?,
        })
    }
}

/// The reply to every federation exchange whose answer is "did you do it, and
/// if not, why": `pair_commit` and `participant_release`. One codec rather than
/// two, because the two differ only in which verb the bool reports and that
/// verb is already the service name.
///
/// [`Self::rejection_reason`] is load-bearing on refusal — see the schema.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FederationVerdict {
    pub ok: bool,
    pub rejection_reason: Option<String>,
}

impl FederationVerdict {
    pub fn ok() -> Self {
        Self {
            ok: true,
            rejection_reason: None,
        }
    }

    pub fn refused(reason: impl Into<String>) -> Self {
        Self {
            ok: false,
            rejection_reason: Some(reason.into()),
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut verdict = builder.init_root::<federation_capnp::federation_verdict::Builder>();
            verdict.set_ok(self.ok);
            verdict.set_rejection_reason(self.rejection_reason.as_deref().unwrap_or(""));
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let verdict = reader.get_root::<federation_capnp::federation_verdict::Reader>()?;
        Ok(Self {
            ok: verdict.get_ok(),
            rejection_reason: optional_text(verdict.get_rejection_reason()?.to_str()?),
        })
    }
}

/// Writes a [`ProducerRef`] into an initialized `InstanceAddress` builder.
///
/// The federation twin of `node::run`'s helper of the same name: the two
/// schemas declare the struct separately because each `.capnp` is compiled on
/// its own, but both decode to one [`ProducerRef`].
fn write_instance_address(
    mut address: federation_capnp::instance_address::Builder<'_>,
    producer: &ProducerRef,
) {
    address.set_core_node(&producer.core_node);
    address.set_instance_id(&producer.instance_id);
}

/// Inverse of [`write_instance_address`]. Both halves are required: an address
/// missing either one names no instance in particular.
fn read_instance_address(
    address: federation_capnp::instance_address::Reader<'_>,
    field: &str,
) -> Result<ProducerRef> {
    Ok(ProducerRef::new(
        required_text(
            address.get_core_node()?.to_str()?,
            &format!("{field}.core_node"),
        )?,
        required_text(
            address.get_instance_id()?.to_str()?,
            &format!("{field}.instance_id"),
        )?,
    ))
}

/// Asks a peer daemon to record its half of a cross-daemon pair and deliver the
/// pin to its own endpoint.
///
/// The field names are relative to the RECEIVER: `local_*` is the endpoint on
/// the daemon being asked, `peer_*` is the one on the daemon asking. Both
/// addresses carry their core node, so the receiver checks that `local` really
/// names it rather than assuming so; see the schema.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PairCommitRequest {
    pub pairing_name: String,
    pub pairing_tag: String,
    pub local: ProducerRef,
    pub local_link_id: String,
    pub local_role: String,
    pub peer: ProducerRef,
    pub peer_link_id: String,
    pub peer_role: String,
}

impl PairCommitRequest {
    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut request = builder.init_root::<federation_capnp::pair_commit_request::Builder>();
            request.set_pairing_name(&self.pairing_name);
            request.set_pairing_tag(&self.pairing_tag);
            request.set_local_link_id(&self.local_link_id);
            request.set_local_role(&self.local_role);
            request.set_peer_link_id(&self.peer_link_id);
            request.set_peer_role(&self.peer_role);
            write_instance_address(request.reborrow().init_local(), &self.local);
            write_instance_address(request.init_peer(), &self.peer);
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let request = reader.get_root::<federation_capnp::pair_commit_request::Reader>()?;
        Ok(Self {
            pairing_name: required_text(request.get_pairing_name()?.to_str()?, "pairing_name")?,
            pairing_tag: required_text(request.get_pairing_tag()?.to_str()?, "pairing_tag")?,
            local: read_instance_address(request.get_local()?, "local")?,
            local_link_id: required_text(request.get_local_link_id()?.to_str()?, "local_link_id")?,
            local_role: required_text(request.get_local_role()?.to_str()?, "local_role")?,
            peer: read_instance_address(request.get_peer()?, "peer")?,
            peer_link_id: required_text(request.get_peer_link_id()?.to_str()?, "peer_link_id")?,
            peer_role: required_text(request.get_peer_role()?.to_str()?, "peer_role")?,
        })
    }
}

/// Releases a reservation. Idempotent: releasing one that is not held
/// succeeds, because a coordinator unwinding a failed preflight cannot always
/// know which participants actually acked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParticipantReleaseRequest {
    pub launch_id: String,
}

impl ParticipantReleaseRequest {
    pub fn new(launch_id: impl Into<String>) -> Self {
        Self {
            launch_id: launch_id.into(),
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            builder
                .init_root::<federation_capnp::launch_scoped_request::Builder>()
                .set_launch_id(&self.launch_id);
        }
        encode_message(&builder)
    }

    /// An empty launch id is refused: the exchange acts on exactly one launch,
    /// and a defaulted id names none.
    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let request = reader.get_root::<federation_capnp::launch_scoped_request::Reader>()?;
        Ok(Self {
            launch_id: required_text(request.get_launch_id()?.to_str()?, "launch_id")?,
        })
    }
}

/// What happened to an instance, as reported by the daemon that owns it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RelationshipEvent {
    /// Reached Running under a fresh incarnation. Observing daemons advance
    /// their incarnation counter for this source and redeliver its pin.
    ReachedRunning,
    /// Stopped or died. A daemon holding a pair with it dissolves that pair.
    Stopped,
}

/// Best-effort, idempotent notification from the daemon that owns an instance
/// to a daemon holding a relationship with it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RelationshipNotification {
    /// The instance whose lifecycle moved, and the daemon that owns it. Two
    /// daemons can host same-named instances, so the pair is the identity.
    pub instance: ProducerRef,
    pub event: RelationshipEvent,
}

impl RelationshipNotification {
    pub fn new(
        instance_id: impl Into<String>,
        core_node: impl Into<String>,
        event: RelationshipEvent,
    ) -> Self {
        Self {
            instance: ProducerRef::new(core_node, instance_id),
            event,
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut notification =
                builder.init_root::<federation_capnp::relationship_notification::Builder>();
            let mut event = notification.reborrow().init_event();
            match self.event {
                RelationshipEvent::ReachedRunning => event.set_reached_running(()),
                RelationshipEvent::Stopped => event.set_stopped(()),
            }
            write_instance_address(notification.init_instance(), &self.instance);
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        use federation_capnp::relationship_notification::event::Which;

        let reader = decode_message(data)?;
        let notification =
            reader.get_root::<federation_capnp::relationship_notification::Reader>()?;

        let event = match notification.get_event().which()? {
            Which::ReachedRunning(()) => RelationshipEvent::ReachedRunning,
            Which::Stopped(()) => RelationshipEvent::Stopped,
        };

        Ok(Self {
            instance: read_instance_address(notification.get_instance()?, "instance")?,
            event,
        })
    }
}

/// Carries no fields: the notification is best-effort and the receiver simply
/// converges on what it is told, so a well-formed reply is itself the ack —
/// the same contract [`HealthRequest`](crate::encoding::HealthRequest) uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RelationshipNotificationAck;

impl RelationshipNotificationAck {
    pub fn new() -> Self {
        Self
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            builder.init_root::<federation_capnp::relationship_notification_ack::Builder>();
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        // Validate the framing decodes as the ack; the struct is empty, so
        // there is nothing else to read back.
        reader.get_root::<federation_capnp::relationship_notification_ack::Reader>()?;
        Ok(Self)
    }
}

impl crate::encoding::Wire for ParticipantReserveRequest {
    type Root = crate::federation_capnp::participant_reserve_request::Owned;
}

impl crate::encoding::Wire for ParticipantReserveResponse {
    type Root = crate::federation_capnp::participant_reserve_response::Owned;
}

impl crate::encoding::Wire for ParticipantSliceBeginRequest {
    type Root = crate::federation_capnp::participant_slice_begin_request::Owned;
}

impl crate::encoding::Wire for ParticipantSliceBeginResponse {
    type Root = crate::federation_capnp::participant_slice_begin_response::Owned;
}

impl crate::encoding::Wire for ParticipantReleaseRequest {
    type Root = crate::federation_capnp::launch_scoped_request::Owned;
}

impl crate::encoding::Wire for PairCommitRequest {
    type Root = crate::federation_capnp::pair_commit_request::Owned;
}

impl crate::encoding::Wire for FederationVerdict {
    type Root = crate::federation_capnp::federation_verdict::Owned;
}

impl crate::encoding::Wire for RelationshipNotification {
    type Root = crate::federation_capnp::relationship_notification::Owned;
}

impl crate::encoding::Wire for RelationshipNotificationAck {
    type Root = crate::federation_capnp::relationship_notification_ack::Owned;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reserve_request_round_trips() {
        let request = ParticipantReserveRequest::new("launch-abc123", "cn-robot-7")
            .with_deployment_pins(vec![
                r#"{root:{kind:"node",name:"deliberative_planner",tag:"v1"},closure:[]}"#
                    .to_owned(),
                r#"{root:{kind:"node",name:"episode_recorder",tag:"v1"},closure:[]}"#.to_owned(),
            ]);
        let payload = request.encode().expect("encode");
        assert_eq!(
            ParticipantReserveRequest::decode(payload.as_ref()).expect("decode"),
            request
        );
    }

    #[test]
    fn reserve_request_round_trips_with_no_deployments() {
        let request = ParticipantReserveRequest::new("launch-abc123", "cn-robot-7");
        let payload = request.encode().expect("encode");
        let decoded = ParticipantReserveRequest::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, request);
        assert!(decoded.deployment_pins_json5.is_empty());
    }

    #[test]
    fn reserve_request_decode_rejects_missing_identity() {
        for (launch_id, coordinator, expected) in [
            ("", "cn-robot-7", "launch_id"),
            ("launch-abc123", "", "coordinator_core_node"),
        ] {
            let request = ParticipantReserveRequest::new(launch_id, coordinator);
            let payload = request.encode().expect("encode");
            let error = ParticipantReserveRequest::decode(payload.as_ref())
                .expect_err("missing identity must fail");
            assert!(error.to_string().contains(expected), "got: {error}");
        }
    }

    #[test]
    fn reserve_response_round_trips_acceptance() {
        let response =
            ParticipantReserveResponse::accepted("v0.20.0-3-g8c7cbaa7", "core_node_gen_1");
        let payload = response.encode().expect("encode");
        let decoded = ParticipantReserveResponse::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, response);
        assert!(decoded.accepted);
        assert_eq!(decoded.root_instance_id, "core_node_gen_1");
    }

    /// A refusal still reports the version so "busy" and "too old" are
    /// distinguishable without a second round trip.
    #[test]
    fn reserve_response_round_trips_refusal_with_version() {
        let response = ParticipantReserveResponse::rejected(
            "already reserved for launch `launch-other`",
            "v0.19.0",
        );
        let payload = response.encode().expect("encode");
        let decoded = ParticipantReserveResponse::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, response);
        assert!(!decoded.accepted);
        assert_eq!(decoded.peppy_version, "v0.19.0");
        assert!(decoded.root_instance_id.is_empty());
    }

    #[test]
    fn release_round_trips() {
        let request = ParticipantReleaseRequest::new("launch-abc123");
        let payload = request.encode().expect("encode");
        assert_eq!(
            ParticipantReleaseRequest::decode(payload.as_ref()).expect("decode"),
            request
        );
    }

    #[test]
    fn slice_begin_round_trips_with_mount_sources() {
        let request = ParticipantSliceBeginRequest::new(
            "launch-abc123",
            vec![
                "/tmp/video_reconstruction".to_owned(),
                "/data/episodes".to_owned(),
            ],
        );
        let payload = request.encode().expect("encode");
        let decoded = ParticipantSliceBeginRequest::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, request);
        assert_eq!(decoded.mount_sources.len(), 2);
        assert_eq!(decoded.mount_sources[1], "/data/episodes");
    }

    /// A slice with no container node binds nothing, which is not the same
    /// shape of message as one that failed to say what it binds.
    #[test]
    fn slice_begin_round_trips_with_no_mount_sources() {
        let request = ParticipantSliceBeginRequest::new("launch-abc123", Vec::new());
        let payload = request.encode().expect("encode");
        let decoded = ParticipantSliceBeginRequest::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, request);
        assert!(decoded.mount_sources.is_empty());
    }

    /// A defaulted entry names no host path. Accepting one would have the
    /// participant create a directory for the empty string.
    #[test]
    fn slice_begin_decode_rejects_an_empty_mount_source() {
        let request = ParticipantSliceBeginRequest::new(
            "launch-abc123",
            vec![String::new(), "/ok".to_owned()],
        );
        let payload = request.encode().expect("encode");
        let error = ParticipantSliceBeginRequest::decode(payload.as_ref())
            .expect_err("an empty mount source must fail");
        assert!(
            error.to_string().contains("mount_sources[0]"),
            "got: {error}"
        );
    }

    #[test]
    fn slice_begin_response_round_trips_both_outcomes() {
        for response in [
            ParticipantSliceBeginResponse::ok(vec!["/tmp/video_reconstruction".to_owned()]),
            ParticipantSliceBeginResponse::ok(Vec::new()),
            ParticipantSliceBeginResponse::refused("reserved for launch `launch-other`"),
        ] {
            let payload = response.encode().expect("encode");
            assert_eq!(
                ParticipantSliceBeginResponse::decode(payload.as_ref()).expect("decode"),
                response
            );
        }
    }

    #[test]
    fn verdict_round_trips_both_outcomes() {
        for verdict in [
            FederationVerdict::ok(),
            FederationVerdict::refused("reserved for launch `launch-other`"),
        ] {
            let payload = verdict.encode().expect("encode");
            assert_eq!(
                FederationVerdict::decode(payload.as_ref()).expect("decode"),
                verdict
            );
        }
    }

    /// The destructive step must never act on a defaulted launch id: that is
    /// how a machine would get its stack replaced on behalf of nobody.
    #[test]
    fn slice_begin_decode_rejects_empty_launch_id() {
        let payload = ParticipantSliceBeginRequest::new("", Vec::new())
            .encode()
            .expect("encode");
        let error = ParticipantSliceBeginRequest::decode(payload.as_ref())
            .expect_err("empty launch id must fail");
        assert!(error.to_string().contains("launch_id"), "got: {error}");
    }

    fn pair_commit() -> PairCommitRequest {
        PairCommitRequest {
            pairing_name: "task_delegation".to_owned(),
            pairing_tag: "v1".to_owned(),
            local: ProducerRef::new("cn-robot-7", "reflex_inst"),
            local_link_id: "delegation".to_owned(),
            local_role: "executor".to_owned(),
            peer: ProducerRef::new("cn-atlas", "planner_inst"),
            peer_link_id: "delegation".to_owned(),
            peer_role: "planner".to_owned(),
        }
    }

    /// Both endpoints name their core node, so a request is readable without
    /// knowing which machine opened it.
    #[test]
    fn pair_commit_round_trips() {
        let request = pair_commit();
        let payload = request.encode().expect("encode");
        let decoded = PairCommitRequest::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, request);
        assert_eq!(decoded.local.core_node, "cn-robot-7");
        assert_eq!(decoded.peer.core_node, "cn-atlas");
    }

    /// Every field addresses a specific slot on a specific machine. A defaulted
    /// one would pair the wrong thing silently, so each is refused at decode
    /// rather than filled in.
    #[test]
    fn pair_commit_decode_rejects_any_empty_address_field() {
        type Blank = fn(&mut PairCommitRequest);
        let fields: [(&str, Blank); 10] = [
            ("pairing_name", |r| r.pairing_name.clear()),
            ("pairing_tag", |r| r.pairing_tag.clear()),
            ("local.core_node", |r| r.local.core_node.clear()),
            ("local.instance_id", |r| r.local.instance_id.clear()),
            ("local_link_id", |r| r.local_link_id.clear()),
            ("local_role", |r| r.local_role.clear()),
            ("peer.core_node", |r| r.peer.core_node.clear()),
            ("peer.instance_id", |r| r.peer.instance_id.clear()),
            ("peer_link_id", |r| r.peer_link_id.clear()),
            ("peer_role", |r| r.peer_role.clear()),
        ];
        for (field, blank) in fields {
            let mut request = pair_commit();
            blank(&mut request);
            let payload = request.encode().expect("encode");
            let error = PairCommitRequest::decode(payload.as_ref())
                .err()
                .unwrap_or_else(|| panic!("a blank `{field}` must be refused"));
            assert!(error.to_string().contains(field), "got: {error}");
        }
    }

    #[test]
    fn release_request_decode_rejects_empty_launch_id() {
        let payload = ParticipantReleaseRequest::new("").encode().expect("encode");
        let error = ParticipantReleaseRequest::decode(payload.as_ref())
            .expect_err("empty launch id must fail");
        assert!(error.to_string().contains("launch_id"), "got: {error}");
    }

    #[test]
    fn relationship_notification_round_trips_every_event() {
        for event in [
            RelationshipEvent::ReachedRunning,
            RelationshipEvent::Stopped,
        ] {
            let notification = RelationshipNotification::new("reflex_inst", "cn-robot-7", event);
            let payload = notification.encode().expect("encode");
            let decoded = RelationshipNotification::decode(payload.as_ref()).expect("decode");
            assert_eq!(decoded, notification);
            assert_eq!(decoded.event, event);
        }
    }

    /// A notification names the instance AND the daemon that owns it: two
    /// daemons can host same-named instances, so the pair is the identity.
    #[test]
    fn relationship_notification_decode_rejects_a_partial_address() {
        for (instance_id, core_node, expected) in [
            ("", "cn-robot-7", "instance_id"),
            ("reflex_inst", "", "core_node"),
        ] {
            let notification =
                RelationshipNotification::new(instance_id, core_node, RelationshipEvent::Stopped);
            let payload = notification.encode().expect("encode");
            let error = RelationshipNotification::decode(payload.as_ref())
                .expect_err("partial address must fail");
            assert!(error.to_string().contains(expected), "got: {error}");
        }
    }

    #[test]
    fn relationship_ack_round_trips() {
        let ack = RelationshipNotificationAck::new();
        let payload = ack.encode().expect("encode");
        assert_eq!(
            RelationshipNotificationAck::decode(payload.as_ref()).expect("decode"),
            ack
        );
    }

    #[test]
    fn decode_rejects_malformed_bytes() {
        assert!(ParticipantReserveRequest::decode(b"not capnp").is_err());
        assert!(ParticipantReserveResponse::decode(b"not capnp").is_err());
        assert!(ParticipantReleaseRequest::decode(b"not capnp").is_err());
        assert!(FederationVerdict::decode(b"not capnp").is_err());
        assert!(RelationshipNotification::decode(b"not capnp").is_err());
        assert!(RelationshipNotificationAck::decode(b"not capnp").is_err());
    }
}
