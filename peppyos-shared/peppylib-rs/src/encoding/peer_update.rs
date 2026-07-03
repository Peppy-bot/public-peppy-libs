//! Cap'n Proto codec for the framework `peer_update` service (pairing-slot
//! delivery). See `schemas/peer_update.capnp` for the wire contract.

use crate::error::{Error, Result};
use crate::messaging::{PeerPin, ProducerRef};
use crate::peer_update_capnp;
use crate::types::Payload;

/// Absolute pairing-slot state pushed by the daemon. `pin: Some` pairs the
/// slot to that peer; `None` clears it. Field-for-field mirror of the capnp
/// `PeerUpdateRequest` with the `paired`/peer-fields flattening folded into
/// `Option<PeerPin>`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PeerUpdateRequest {
    pub link_id: String,
    pub sequence: u64,
    pub pin: Option<PeerPin>,
}

impl PeerUpdateRequest {
    pub fn encode(&self) -> Result<Payload> {
        let mut builder = ::capnp::message::Builder::new_default();
        {
            let mut root = builder.init_root::<peer_update_capnp::peer_update_request::Builder>();
            root.set_link_id(&self.link_id);
            root.set_sequence(self.sequence);
            match &self.pin {
                Some(pin) => {
                    root.set_paired(true);
                    root.set_peer_core_node(&pin.producer.core_node);
                    root.set_peer_instance_id(&pin.producer.instance_id);
                    root.set_peer_link_id(&pin.peer_link_id);
                }
                None => {
                    root.set_paired(false);
                    root.set_peer_core_node("");
                    root.set_peer_instance_id("");
                    root.set_peer_link_id("");
                }
            }
        }
        super::encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = super::decode_message(data)?;
        let root = reader
            .get_root::<peer_update_capnp::peer_update_request::Reader>()
            .map_err(|e| Error::Deserialization(e.to_string()))?;
        let link_id = read_text(root.get_link_id(), "linkId")?;
        let sequence = root.get_sequence();
        let pin = if root.get_paired() {
            Some(PeerPin {
                producer: ProducerRef::new(
                    read_text(root.get_peer_core_node(), "peerCoreNode")?,
                    read_text(root.get_peer_instance_id(), "peerInstanceId")?,
                ),
                peer_link_id: read_text(root.get_peer_link_id(), "peerLinkId")?,
            })
        } else {
            None
        };
        Ok(Self {
            link_id,
            sequence,
            pin,
        })
    }
}

/// Node-side reply to a [`PeerUpdateRequest`]. `accepted = false` with
/// `stale_sequence = true` means the request's sequence was strictly older
/// than the slot's current one (a delayed retry) — the daemon treats that as
/// already-superseded, not as a failure to revert.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PeerUpdateResponse {
    pub accepted: bool,
    pub stale_sequence: bool,
    pub message: String,
}

impl PeerUpdateResponse {
    pub fn accepted() -> Self {
        Self {
            accepted: true,
            stale_sequence: false,
            message: String::new(),
        }
    }

    pub fn stale() -> Self {
        Self {
            accepted: false,
            stale_sequence: true,
            message: "stale sequence".to_string(),
        }
    }

    pub fn rejected(message: impl Into<String>) -> Self {
        Self {
            accepted: false,
            stale_sequence: false,
            message: message.into(),
        }
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = ::capnp::message::Builder::new_default();
        {
            let mut root = builder.init_root::<peer_update_capnp::peer_update_response::Builder>();
            root.set_accepted(self.accepted);
            root.set_stale_sequence(self.stale_sequence);
            root.set_message(&self.message);
        }
        super::encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = super::decode_message(data)?;
        let root = reader
            .get_root::<peer_update_capnp::peer_update_response::Reader>()
            .map_err(|e| Error::Deserialization(e.to_string()))?;
        Ok(Self {
            accepted: root.get_accepted(),
            stale_sequence: root.get_stale_sequence(),
            message: read_text(root.get_message(), "message")?,
        })
    }
}

fn read_text(field: ::capnp::Result<::capnp::text::Reader<'_>>, name: &str) -> Result<String> {
    field
        .map_err(|e| Error::Deserialization(format!("peer_update field `{name}`: {e}")))?
        .to_str()
        .map(str::to_owned)
        .map_err(|e| Error::Deserialization(format!("peer_update field `{name}` not UTF-8: {e}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_round_trips_paired_and_unpaired() {
        let paired = PeerUpdateRequest {
            link_id: "arm".to_string(),
            sequence: 42,
            pin: Some(PeerPin {
                producer: ProducerRef::new("core_a", "arm_1"),
                peer_link_id: "controller".to_string(),
            }),
        };
        let decoded = PeerUpdateRequest::decode(&paired.encode().unwrap().into_inner()).unwrap();
        assert_eq!(decoded, paired);

        let unpaired = PeerUpdateRequest {
            link_id: "arm".to_string(),
            sequence: 43,
            pin: None,
        };
        let decoded = PeerUpdateRequest::decode(&unpaired.encode().unwrap().into_inner()).unwrap();
        assert_eq!(decoded, unpaired);
    }

    #[test]
    fn response_round_trips_all_shapes() {
        for response in [
            PeerUpdateResponse::accepted(),
            PeerUpdateResponse::stale(),
            PeerUpdateResponse::rejected("unknown pairing slot 'arm'"),
        ] {
            let decoded =
                PeerUpdateResponse::decode(&response.encode().unwrap().into_inner()).unwrap();
            assert_eq!(decoded, response);
        }
    }
}
