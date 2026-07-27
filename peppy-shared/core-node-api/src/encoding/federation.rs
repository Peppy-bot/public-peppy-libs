//! Encoding types for the daemon-to-daemon federated-launch messages.
//!
//! See `schemas/federation.capnp` for why the reservation exchange and the
//! relationship notifications have deliberately different guarantees: the
//! former is coordinator-driven and must be exact, the latter is best-effort
//! and idempotent.

use capnp::message::Builder;

use crate::federation_capnp;
use crate::{Payload, Result};

use crate::encoding::{capnp_list_len, decode_message, encode_message, optional_text};

/// Reserves one participant for one launch, and asks it to resolve the
/// manifests for its own slice while it is at it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParticipantReserveRequest {
    pub launch_id: String,
    /// The coordinator driving the launch. The participant watches this core
    /// node's presence for as long as it holds the reservation.
    pub coordinator_core_node: String,
    /// JSON5-encoded launcher `DeploymentSource`, one per deployment placed on
    /// this participant. Opaque here: the launcher document model lives in
    /// peppy, and this crate has no business re-deriving it.
    pub deployment_sources_json5: Vec<String>,
}

impl ParticipantReserveRequest {
    pub fn new(launch_id: impl Into<String>, coordinator_core_node: impl Into<String>) -> Self {
        Self {
            launch_id: launch_id.into(),
            coordinator_core_node: coordinator_core_node.into(),
            deployment_sources_json5: Vec::new(),
        }
    }

    pub fn with_deployment_sources(mut self, sources: Vec<String>) -> Self {
        self.deployment_sources_json5 = sources;
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
                self.deployment_sources_json5.len(),
                "ParticipantReserveRequest.deployment_sources_json5",
            )?;
            let mut sources = request.reborrow().init_deployment_sources_json5(count);
            for (idx, source) in self.deployment_sources_json5.iter().enumerate() {
                sources.set(idx as u32, source.as_str());
            }
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let request = reader.get_root::<federation_capnp::participant_reserve_request::Reader>()?;

        let launch_id = non_empty(request.get_launch_id()?.to_str()?, "launch_id")?;
        let coordinator_core_node = non_empty(
            request.get_coordinator_core_node()?.to_str()?,
            "coordinator_core_node",
        )?;

        let sources_reader = request.get_deployment_sources_json5()?;
        let mut deployment_sources_json5 = Vec::with_capacity(sources_reader.len() as usize);
        for idx in 0..sources_reader.len() {
            deployment_sources_json5.push(sources_reader.get(idx)?.to_str()?.to_owned());
        }

        Ok(Self {
            launch_id,
            coordinator_core_node,
            deployment_sources_json5,
        })
    }
}

/// A launch is only reconstructible if every message that carries its identity
/// actually carries it, so an empty one is refused rather than defaulted.
fn non_empty(value: &str, field: &str) -> Result<String> {
    if value.is_empty() {
        return Err(crate::Error::Decoding(format!(
            "federation message field `{field}` is empty"
        )));
    }
    Ok(value.to_owned())
}

/// One manifest as a participant resolved it, aligned by index with the
/// request's `deployment_sources_json5`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedManifest {
    pub config_json5: String,
    /// SHA256 of the manifest, echoed back on the instance plan the
    /// coordinator later dispatches so the participant can refuse a cache that
    /// moved between preflight and start.
    pub config_sha256: String,
}

impl ResolvedManifest {
    pub fn new(config_json5: impl Into<String>, config_sha256: impl Into<String>) -> Self {
        Self {
            config_json5: config_json5.into(),
            config_sha256: config_sha256.into(),
        }
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
    pub manifests: Vec<ResolvedManifest>,
}

impl ParticipantReserveResponse {
    pub fn accepted(
        peppy_version: impl Into<String>,
        root_instance_id: impl Into<String>,
        manifests: Vec<ResolvedManifest>,
    ) -> Self {
        Self {
            accepted: true,
            rejection_reason: None,
            peppy_version: peppy_version.into(),
            root_instance_id: root_instance_id.into(),
            manifests,
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
            manifests: Vec::new(),
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

            let count = capnp_list_len(
                self.manifests.len(),
                "ParticipantReserveResponse.manifests",
            )?;
            let mut manifests = response.reborrow().init_manifests(count);
            for (idx, manifest) in self.manifests.iter().enumerate() {
                let mut entry = manifests.reborrow().get(idx as u32);
                entry.set_config_json5(&manifest.config_json5);
                entry.set_config_sha256(&manifest.config_sha256);
            }
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let response =
            reader.get_root::<federation_capnp::participant_reserve_response::Reader>()?;

        let manifests_reader = response.get_manifests()?;
        let mut manifests = Vec::with_capacity(manifests_reader.len() as usize);
        for idx in 0..manifests_reader.len() {
            let entry = manifests_reader.get(idx);
            manifests.push(ResolvedManifest {
                config_json5: entry.get_config_json5()?.to_str()?.to_owned(),
                config_sha256: entry.get_config_sha256()?.to_str()?.to_owned(),
            });
        }

        Ok(Self {
            accepted: response.get_accepted(),
            rejection_reason: optional_text(response.get_rejection_reason()?.to_str()?),
            peppy_version: response.get_peppy_version()?.to_str()?.to_owned(),
            root_instance_id: response.get_root_instance_id()?.to_str()?.to_owned(),
            manifests,
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
            let mut request =
                builder.init_root::<federation_capnp::participant_release_request::Builder>();
            request.set_launch_id(&self.launch_id);
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let request = reader.get_root::<federation_capnp::participant_release_request::Reader>()?;
        Ok(Self {
            launch_id: non_empty(request.get_launch_id()?.to_str()?, "launch_id")?,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParticipantReleaseResponse {
    /// False only when the reservation is held for a DIFFERENT launch, which
    /// the caller has no standing to release.
    pub released: bool,
    pub rejection_reason: Option<String>,
}

impl ParticipantReleaseResponse {
    pub fn released() -> Self {
        Self {
            released: true,
            rejection_reason: None,
        }
    }

    pub fn refused(reason: impl Into<String>) -> Self {
        Self {
            released: false,
            rejection_reason: Some(reason.into()),
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut response =
                builder.init_root::<federation_capnp::participant_release_response::Builder>();
            response.set_released(self.released);
            response.set_rejection_reason(self.rejection_reason.as_deref().unwrap_or(""));
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let response =
            reader.get_root::<federation_capnp::participant_release_response::Reader>()?;
        Ok(Self {
            released: response.get_released(),
            rejection_reason: optional_text(response.get_rejection_reason()?.to_str()?),
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
    pub instance_id: String,
    pub core_node: String,
    pub event: RelationshipEvent,
}

impl RelationshipNotification {
    pub fn new(
        instance_id: impl Into<String>,
        core_node: impl Into<String>,
        event: RelationshipEvent,
    ) -> Self {
        Self {
            instance_id: instance_id.into(),
            core_node: core_node.into(),
            event,
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut notification =
                builder.init_root::<federation_capnp::relationship_notification::Builder>();
            notification.set_instance_id(&self.instance_id);
            notification.set_core_node(&self.core_node);
            let mut event = notification.reborrow().init_event();
            match self.event {
                RelationshipEvent::ReachedRunning => event.set_reached_running(()),
                RelationshipEvent::Stopped => event.set_stopped(()),
            }
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
            instance_id: non_empty(notification.get_instance_id()?.to_str()?, "instance_id")?,
            core_node: non_empty(notification.get_core_node()?.to_str()?, "core_node")?,
            event,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RelationshipNotificationAck {
    pub received: bool,
}

impl RelationshipNotificationAck {
    pub fn received() -> Self {
        Self { received: true }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut ack =
                builder.init_root::<federation_capnp::relationship_notification_ack::Builder>();
            ack.set_received(self.received);
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let ack = reader.get_root::<federation_capnp::relationship_notification_ack::Reader>()?;
        Ok(Self {
            received: ack.get_received(),
        })
    }
}

impl crate::encoding::Wire for ParticipantReserveRequest {
    type Root = crate::federation_capnp::participant_reserve_request::Owned;
}

impl crate::encoding::Wire for ParticipantReserveResponse {
    type Root = crate::federation_capnp::participant_reserve_response::Owned;
}

impl crate::encoding::Wire for ParticipantReleaseRequest {
    type Root = crate::federation_capnp::participant_release_request::Owned;
}

impl crate::encoding::Wire for ParticipantReleaseResponse {
    type Root = crate::federation_capnp::participant_release_response::Owned;
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
            .with_deployment_sources(vec![
                r#"{name:"deliberative_planner",tag:"v1"}"#.to_owned(),
                r#"{name:"episode_recorder",tag:"v1"}"#.to_owned(),
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
        assert!(decoded.deployment_sources_json5.is_empty());
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
    fn reserve_response_round_trips_acceptance_with_manifests() {
        let response = ParticipantReserveResponse::accepted(
            "v0.20.0-3-g8c7cbaa7",
            "core_node_gen_1",
            vec![
                ResolvedManifest::new(r#"{peppy_schema:"node/v1"}"#, "a".repeat(64)),
                ResolvedManifest::new(r#"{peppy_schema:"node/v1"}"#, "b".repeat(64)),
            ],
        );
        let payload = response.encode().expect("encode");
        let decoded = ParticipantReserveResponse::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, response);
        assert!(decoded.accepted);
        assert_eq!(decoded.manifests.len(), 2);
        assert_eq!(decoded.manifests[1].config_sha256, "b".repeat(64));
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
        assert!(decoded.manifests.is_empty());
    }

    #[test]
    fn release_round_trips() {
        let request = ParticipantReleaseRequest::new("launch-abc123");
        let payload = request.encode().expect("encode");
        assert_eq!(
            ParticipantReleaseRequest::decode(payload.as_ref()).expect("decode"),
            request
        );

        for response in [
            ParticipantReleaseResponse::released(),
            ParticipantReleaseResponse::refused("reserved for launch `launch-other`"),
        ] {
            let payload = response.encode().expect("encode");
            assert_eq!(
                ParticipantReleaseResponse::decode(payload.as_ref()).expect("decode"),
                response
            );
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
            let notification = RelationshipNotification::new(
                instance_id,
                core_node,
                RelationshipEvent::Stopped,
            );
            let payload = notification.encode().expect("encode");
            let error = RelationshipNotification::decode(payload.as_ref())
                .expect_err("partial address must fail");
            assert!(error.to_string().contains(expected), "got: {error}");
        }
    }

    #[test]
    fn relationship_ack_round_trips() {
        let ack = RelationshipNotificationAck::received();
        let payload = ack.encode().expect("encode");
        let decoded = RelationshipNotificationAck::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, ack);
        assert!(decoded.received);
    }

    #[test]
    fn decode_rejects_malformed_bytes() {
        assert!(ParticipantReserveRequest::decode(b"not capnp").is_err());
        assert!(ParticipantReserveResponse::decode(b"not capnp").is_err());
        assert!(ParticipantReleaseRequest::decode(b"not capnp").is_err());
        assert!(ParticipantReleaseResponse::decode(b"not capnp").is_err());
        assert!(RelationshipNotification::decode(b"not capnp").is_err());
        assert!(RelationshipNotificationAck::decode(b"not capnp").is_err());
    }
}
