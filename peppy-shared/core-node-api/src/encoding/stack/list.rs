use capnp::message::Builder;

use crate::node_capnp;
use crate::{Payload, Result};

use crate::encoding::{decode_message, encode_message, required_text};

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct StackListRequest;

impl StackListRequest {
    pub fn new() -> Self {
        Self
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        builder.init_root::<node_capnp::stack_list_request::Builder>();
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        let reader = decode_message(data)?;
        let request = reader.get_root::<node_capnp::stack_list_request::Reader>()?;
        let size = request.total_size()?;
        if size.word_count != 0 || size.cap_count != 0 {
            return Err(crate::Error::Decoding(format!(
                "StackListRequest must be an empty struct, got {} words and {} capabilities",
                size.word_count, size.cap_count
            )));
        }
        Ok(Self)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StackListResponse {
    pub graph_json: String,
    /// Presence identity of the serving daemon: its core-node name and
    /// daemon-generation instance id, matching its core-node presence token.
    pub core_node: String,
    pub instance_id: String,
    /// Hostname of the machine the serving daemon runs on.
    pub host_name: String,
    /// The launch this daemon's slice belongs to: its id and the coordinator
    /// that drove it. `None` when the stack was not started by a federated
    /// launch.
    ///
    /// Carrying it here is what makes a slice self-describing, and therefore
    /// what lets a coordinator REDISCOVER its participants instead of
    /// remembering them. `stack list` already fans out to every live core
    /// node, so rediscovery is that same fan-out filtered by launch id: a
    /// restarted coordinator finds its own launch again, and `stack reset`
    /// works from any machine in the federation.
    pub launch: Option<LaunchIdentity>,
}

/// Which launch a slice belongs to, and who drove it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LaunchIdentity {
    pub launch_id: String,
    pub coordinator_core_node: String,
}

impl LaunchIdentity {
    pub fn new(launch_id: impl Into<String>, coordinator_core_node: impl Into<String>) -> Self {
        Self {
            launch_id: launch_id.into(),
            coordinator_core_node: coordinator_core_node.into(),
        }
    }
}

impl StackListResponse {
    pub fn new(
        graph_json: impl Into<String>,
        core_node: impl Into<String>,
        instance_id: impl Into<String>,
        host_name: impl Into<String>,
    ) -> Self {
        Self {
            graph_json: graph_json.into(),
            core_node: core_node.into(),
            instance_id: instance_id.into(),
            host_name: host_name.into(),
            launch: None,
        }
    }

    /// Attaches the launch this slice belongs to. A daemon whose stack came
    /// from a federated launch always sets this.
    pub fn with_launch(mut self, launch: LaunchIdentity) -> Self {
        self.launch = Some(launch);
        self
    }

    pub fn encode(&self) -> Result<Payload> {
        let mut builder = Builder::new_default();
        {
            let mut response = builder.init_root::<node_capnp::stack_list_response::Builder>();
            response.set_graph_json(&self.graph_json);
            response.set_core_node(&self.core_node);
            response.set_instance_id(&self.instance_id);
            response.set_host_name(&self.host_name);
            let mut launch = response.reborrow().init_launch();
            match &self.launch {
                Some(identity) => {
                    let mut wire = launch.init_identity();
                    wire.set_launch_id(&identity.launch_id);
                    wire.set_coordinator_core_node(&identity.coordinator_core_node);
                }
                None => launch.set_standalone(()),
            }
        }
        encode_message(&builder)
    }

    pub fn decode(data: &[u8]) -> Result<Self> {
        use node_capnp::stack_list_response::launch::Which;

        let reader = decode_message(data)?;
        let response = reader.get_root::<node_capnp::stack_list_response::Reader>()?;

        // The union makes "a slice is either part of a launch or it is not" a
        // wire fact, so there is no half-formed identity left to police here.
        let launch = match response.get_launch().which()? {
            Which::Standalone(()) => None,
            Which::Identity(identity) => {
                let identity = identity?;
                Some(LaunchIdentity::new(
                    required_text(identity.get_launch_id()?.to_str()?, "launch.launch_id")?,
                    required_text(
                        identity.get_coordinator_core_node()?.to_str()?,
                        "launch.coordinator_core_node",
                    )?,
                ))
            }
        };

        Ok(Self {
            graph_json: response.get_graph_json()?.to_str()?.to_owned(),
            core_node: response.get_core_node()?.to_str()?.to_owned(),
            instance_id: response.get_instance_id()?.to_str()?.to_owned(),
            host_name: response.get_host_name()?.to_str()?.to_owned(),
            launch,
        })
    }
}

impl crate::encoding::Wire for StackListRequest {
    type Root = crate::node_capnp::stack_list_request::Owned;
}

impl crate::encoding::Wire for StackListResponse {
    type Root = crate::node_capnp::stack_list_response::Owned;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_round_trips() {
        let request = StackListRequest::new();
        let payload = request.encode().expect("encode");
        let decoded = StackListRequest::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, request);
    }

    #[test]
    fn request_decode_rejects_legacy_nonempty_struct() {
        // The old schema encoded a removed boolean in a one-data-word root
        // struct. Build both possible framed messages directly so this
        // regression fixture needs no legacy generated API.
        for legacy_flag in [false, true] {
            let mut payload = Vec::new();
            payload.extend_from_slice(&0_u32.to_le_bytes()); // one segment
            payload.extend_from_slice(&2_u32.to_le_bytes()); // two words
            payload.extend_from_slice(&(1_u64 << 32).to_le_bytes()); // struct: data=1, ptrs=0
            payload.extend_from_slice(&u64::from(legacy_flag).to_le_bytes());

            let error = StackListRequest::decode(&payload)
                .expect_err("legacy request shape must not decode as the new empty request");
            assert!(
                error.to_string().contains("must be an empty struct"),
                "got: {error}"
            );
        }
    }

    #[test]
    fn request_decode_rejects_malformed() {
        assert!(StackListRequest::decode(b"not capnp").is_err());
    }

    #[test]
    fn response_round_trips_graph_json_and_daemon_identity() {
        let response = StackListResponse::new(
            r#"{"nodes":["a","b"],"edges":[["a","b"]]}"#,
            "core_a",
            "generation_1",
            "robo-a",
        );
        let payload = response.encode().expect("encode");
        let decoded = StackListResponse::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, response);
        assert_eq!(
            decoded.graph_json,
            r#"{"nodes":["a","b"],"edges":[["a","b"]]}"#
        );
        assert_eq!(decoded.core_node, "core_a");
        assert_eq!(decoded.instance_id, "generation_1");
        assert_eq!(decoded.host_name, "robo-a");
        assert_eq!(decoded.launch, None, "a local stack belongs to no launch");
    }

    /// A slice is self-describing: it names the launch it belongs to and the
    /// coordinator that drove it, which is what lets a restarted coordinator
    /// rediscover its participants rather than lose them.
    #[test]
    fn response_round_trips_launch_identity() {
        let response = StackListResponse::new("{}", "cn-atlas-h100", "generation_1", "atlas")
            .with_launch(LaunchIdentity::new("launch-abc123", "cn-robot-7"));
        let payload = response.encode().expect("encode");
        let decoded = StackListResponse::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, response);
        let launch = decoded.launch.expect("launch identity must survive");
        assert_eq!(launch.launch_id, "launch-abc123");
        assert_eq!(launch.coordinator_core_node, "cn-robot-7");
    }

    /// The union carries "part of a launch or not" in the discriminant, so the
    /// two halves can no longer be sent apart. What remains decodable-but-wrong
    /// is an identity arm with a blank half, and that is still refused — a
    /// launch nobody can name is no better than no launch at all.
    #[test]
    fn response_decode_rejects_a_blank_half_of_the_identity() {
        for (launch_id, coordinator, expected) in [
            ("launch-abc123", "", "coordinator_core_node"),
            ("", "cn-robot-7", "launch_id"),
        ] {
            let response = StackListResponse::new("{}", "cn-atlas", "gen_1", "atlas")
                .with_launch(LaunchIdentity::new(launch_id, coordinator));
            let payload = response.encode().expect("encode");
            let error =
                StackListResponse::decode(payload.as_ref()).expect_err("blank half must fail");
            assert!(error.to_string().contains(expected), "got: {error}");
        }
    }

    /// A standalone slice must decode as one even though `LaunchIdentity` is a
    /// pointer field: the discriminant, not an empty string, is what says so.
    #[test]
    fn standalone_is_the_discriminant_not_an_empty_identity() {
        let payload = StackListResponse::new("{}", "cn-solo", "gen_1", "solo")
            .encode()
            .expect("encode");
        let decoded = StackListResponse::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded.launch, None);
    }

    #[test]
    fn response_round_trips_empty_fields() {
        let response = StackListResponse::new("", "", "", "");
        let payload = response.encode().expect("encode");
        let decoded = StackListResponse::decode(payload.as_ref()).expect("decode");
        assert_eq!(decoded, response);
        assert!(decoded.graph_json.is_empty());
        assert!(decoded.core_node.is_empty());
        assert!(decoded.instance_id.is_empty());
        assert!(decoded.host_name.is_empty());
    }

    #[test]
    fn response_decode_rejects_malformed() {
        assert!(StackListResponse::decode(b"not capnp").is_err());
    }
}
