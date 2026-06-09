use std::time::Duration;

use peppylib::config::QoSProfile;
use peppylib::messaging::{ConsumerFilter, SenderTarget};
use peppylib::{MessengerHandle, Payload};
use serde_json::Value;

use crate::config::DaemonState;

const SIM_CTRL_TIMEOUT: Duration = Duration::from_secs(5);
const SIM_CTRL_REQ_SUFFIX: &str = "_req";
const SIM_CTRL_RES_SUFFIX: &str = "_res";

pub async fn call_sim(
    daemon: &DaemonState,
    sim_node: &str,
    service: &str,
    payload: Value,
) -> std::result::Result<Value, String> {
    let req_topic = format!("sim_ctrl_{service}{SIM_CTRL_REQ_SUFFIX}");
    let res_topic = format!("sim_ctrl_{service}{SIM_CTRL_RES_SUFFIX}");

    let handle = MessengerHandle::from_host_port("localhost", daemon.messaging_port)
        .await
        .map_err(|e| format!("connect: {e}"))?;

    // SenderTarget identities: the sim-side producer is a node target;
    // "sim_bridge" is the placeholder for this side's publish identity.
    let sim_target = SenderTarget::node(sim_node, "v1")
        .map_err(|e| format!("invalid sim_node target '{sim_node}': {e}"))?;
    let bridge_target = SenderTarget::node("sim_bridge", "v1")
        .map_err(|e| format!("invalid sim_bridge target: {e}"))?;

    let mut sub = peppylib::TopicMessenger::subscribe(
        &handle,
        &daemon.core_node_name,
        &format!("sim_bridge_{service}_res_sub"),
        Some(sim_target),
        false,
        &res_topic,
        None,
        &ConsumerFilter::Any,
        QoSProfile::Standard,
    )
    .await
    .map_err(|e| format!("subscribe {res_topic}: {e}"))?;

    let body = serde_json::to_vec(&payload).map_err(|e| format!("serialize: {e}"))?;
    peppylib::TopicMessenger::emit(
        &handle,
        &daemon.core_node_name,
        &format!("sim_bridge_{service}_req_pub"),
        bridge_target,
        &req_topic,
        QoSProfile::Standard,
        Payload::from(body),
    )
    .await
    .map_err(|e| format!("emit {req_topic}: {e}"))?;

    let msg = tokio::time::timeout(SIM_CTRL_TIMEOUT, sub.on_next_message())
        .await
        .map_err(|_| format!("timeout waiting for {res_topic}"))?
        .ok_or_else(|| format!("subscription closed for {res_topic}"))?;

    serde_json::from_slice::<Value>(msg.payload().as_ref())
        .map_err(|e| format!("deserialize response: {e}"))
}

/// Requires a multi-threaded Tokio runtime (`flavor = "multi_thread"`). Panics on current_thread.
pub fn call_sim_sync(
    daemon: &DaemonState,
    sim_node: &str,
    service: &str,
    payload: Value,
) -> std::result::Result<Value, String> {
    tokio::task::block_in_place(|| {
        tokio::runtime::Handle::current().block_on(call_sim(daemon, sim_node, service, payload))
    })
}
