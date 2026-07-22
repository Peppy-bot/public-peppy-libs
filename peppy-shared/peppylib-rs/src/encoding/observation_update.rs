//! Cap'n Proto codec for the framework `observation_update` service
//! (observer-slot delivery). See `schemas/observation_update.capnp` for the wire
//! contract.

use crate::error::{Error, Result};
use crate::messaging::{ObservationPin, ProducerRef};
use crate::observation_update_capnp;
use crate::types::Payload;

/// Absolute observer-slot state pushed by the daemon. `source: Some` pins the
/// slot to that producer source; `None` is the pre-resolution boot state.
/// Field-for-field mirror of the capnp `ObservationUpdateRequest` with the
/// `hasSource`/source-fields flattening folded into `Option<ObservationPin>`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationUpdateRequest {
    pub link_id: String,
    pub sequence: u64,
    pub source: Option<ObservationPin>,
    pub source_generation: u64,
    pub source_live: bool,
}

impl ObservationUpdateRequest {
    pub fn encode(&self) -> Result<Payload> {
        let mut builder = ::capnp::message::Builder::new_default();
        {
            let mut root = builder
                .init_root::<observation_update_capnp::observation_update_request::Builder>();
            root.set_link_id(&self.link_id);
            root.set_sequence(self.sequence);
            root.set_source_generation(self.source_generation);
            root.set_source_live(self.source_live);
            match &self.source {
                Some(source) => {
                    root.set_has_source(true);
                    root.set_source_core_node(&source.producer.core_node);
                    root.set_source_instance_id(&source.producer.instance_id);
                    root.set_source_link_id(&source.source_link_id);
                }
                None => {
                    root.set_has_source(false);
                    root.set_source_core_node("");
                    root.set_source_instance_id("");
                    root.set_source_link_id("");
                }
            }
        }
        super::encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = super::decode_message(data)?;
        let root = reader
            .get_root::<observation_update_capnp::observation_update_request::Reader>()
            .map_err(|e| Error::Deserialization(e.to_string()))?;
        let link_id = super::read_text(root.get_link_id(), "observation_update", "linkId")?;
        let sequence = root.get_sequence();
        let source_generation = root.get_source_generation();
        let source_live = root.get_source_live();
        let source = if root.get_has_source() {
            Some(ObservationPin {
                producer: ProducerRef::new(
                    super::read_text(
                        root.get_source_core_node(),
                        "observation_update",
                        "sourceCoreNode",
                    )?,
                    super::read_text(
                        root.get_source_instance_id(),
                        "observation_update",
                        "sourceInstanceId",
                    )?,
                ),
                source_link_id: super::read_text(
                    root.get_source_link_id(),
                    "observation_update",
                    "sourceLinkId",
                )?,
            })
        } else {
            None
        };
        Ok(Self {
            link_id,
            sequence,
            source,
            source_generation,
            source_live,
        })
    }
}

/// Node-side reply to an [`ObservationUpdateRequest`]. `accepted = false` with
/// `stale_sequence = true` means the request's sequence was strictly older than
/// the slot's current one (a delayed retry) — the daemon treats that as
/// already-superseded, not as a failure to revert.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationUpdateResponse {
    pub accepted: bool,
    pub stale_sequence: bool,
    pub message: String,
}

impl ObservationUpdateResponse {
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
            let mut root = builder
                .init_root::<observation_update_capnp::observation_update_response::Builder>();
            root.set_accepted(self.accepted);
            root.set_stale_sequence(self.stale_sequence);
            root.set_message(&self.message);
        }
        super::encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = super::decode_message(data)?;
        let root = reader
            .get_root::<observation_update_capnp::observation_update_response::Reader>()
            .map_err(|e| Error::Deserialization(e.to_string()))?;
        Ok(Self {
            accepted: root.get_accepted(),
            stale_sequence: root.get_stale_sequence(),
            message: super::read_text(root.get_message(), "observation_update", "message")?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pin() -> ObservationPin {
        ObservationPin {
            producer: ProducerRef::new("core_a", "arm_1"),
            source_link_id: "commander".to_string(),
        }
    }

    #[test]
    fn request_round_trips_resolved_and_unresolved() {
        let resolved = ObservationUpdateRequest {
            link_id: "observed_arm".to_string(),
            sequence: 42,
            source: Some(pin()),
            source_generation: 7,
            source_live: true,
        };
        let decoded =
            ObservationUpdateRequest::decode(&resolved.encode().unwrap().into_inner()).unwrap();
        assert_eq!(decoded, resolved);

        let unresolved = ObservationUpdateRequest {
            link_id: "observed_arm".to_string(),
            sequence: 43,
            source: None,
            source_generation: 0,
            source_live: false,
        };
        let decoded =
            ObservationUpdateRequest::decode(&unresolved.encode().unwrap().into_inner()).unwrap();
        assert_eq!(decoded, unresolved);
    }

    #[test]
    fn response_round_trips_all_shapes() {
        for response in [
            ObservationUpdateResponse::accepted(),
            ObservationUpdateResponse::stale(),
            ObservationUpdateResponse::rejected("unknown observer slot 'observed_arm'"),
        ] {
            let decoded =
                ObservationUpdateResponse::decode(&response.encode().unwrap().into_inner())
                    .unwrap();
            assert_eq!(decoded, response);
        }
    }
}
