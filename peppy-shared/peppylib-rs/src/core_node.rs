pub mod transport;

/// Subscribes to a periodic core-node-published topic (e.g. `clock`,
/// `daemon_heartbeat`) on `node_runner`'s bound core node, keyed the same way
/// the daemon publishes it. The publisher is scoped by `from_target` alone —
/// a daemon's node name IS its core_node name, so the target's name segment
/// already pins which daemon's stream this matches. `SensorData` QoS: for
/// these streams a slow subscriber should get newer messages dropped rather
/// than back-pressure the publisher.
pub(crate) async fn subscribe_core_topic(
    node_runner: &crate::runtime::NodeRunner,
    topic: &str,
) -> crate::error::Result<crate::messaging::Subscription> {
    let processor = node_runner.processor();
    let core_node = processor.bound_core_node();
    crate::messaging::TopicMessenger::subscribe_target_scoped(
        node_runner.messenger(),
        core_node,
        processor.bound_instance_id(),
        crate::messaging::SenderTarget::node(core_node, core_node_api::names::CORE_NODE_TAG)?,
        topic,
        config::node::QoSProfile::SensorData,
    )
    .await
}
