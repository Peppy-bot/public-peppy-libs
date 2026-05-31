use std::sync::Arc;

use peppylib::config::QoSProfile;
use peppylib::messaging::{ConsumerFilter, SenderTarget};
use peppylib::runtime::CancellationToken;
use serde::Deserialize;

use crate::config::DaemonState;
use super::{BACKOFF_INIT, BACKOFF_MAX};

pub async fn run_sim_to_os<Runner, Msg, EmitFn>(
    runner: Arc<Runner>,
    token: CancellationToken,
    daemon: DaemonState,
    sim_node: Arc<str>,
    topic: Arc<str>,
    emit_fn: EmitFn,
) where
    Runner: Send + Sync + 'static,
    Msg: for<'de> Deserialize<'de> + Send + 'static,
    EmitFn: Fn(Arc<Runner>, Msg) -> super::BoxFuture<Result<(), String>> + Send + 'static,
{
    let mut backoff = BACKOFF_INIT;

    'retry: loop {
        let handle = tokio::select! {
            _ = token.cancelled() => break,
            result = peppylib::MessengerHandle::from_host_port("localhost", daemon.messaging_port) => {
                match result {
                    Ok(h) => h,
                    Err(e) => {
                        tracing::warn!("sim_to_os({topic}): connect — {e}, retry in {backoff:?}");
                        tokio::select! {
                            _ = token.cancelled() => break 'retry,
                            _ = tokio::time::sleep(backoff) => {}
                        }
                        backoff = (backoff * 2).min(BACKOFF_MAX);
                        continue 'retry;
                    }
                }
            }
        };

        let instance_id = format!("sim_bridge_{topic}");
        // v0.10: subscribe takes SenderTarget for the producer-side identity.
        // sim_node names a conforming node (its tag is fixed at v1 across
        // the openarm01 deployment).
        let sim_target = match SenderTarget::node(&*sim_node, "v1") {
            Ok(t) => t,
            Err(e) => {
                tracing::error!("sim_to_os({topic}): invalid sim_node target '{sim_node}': {e}");
                break 'retry;
            }
        };
        let mut sub = tokio::select! {
            _ = token.cancelled() => break,
            result = peppylib::TopicMessenger::subscribe(
                &handle,
                &daemon.core_node_name,
                &instance_id,
                Some(sim_target),
                false,
                &*topic,
                None,
                &ConsumerFilter::Any,
                QoSProfile::SensorData,
            ) => {
                match result {
                    Ok(s) => s,
                    Err(e) => {
                        tracing::warn!("sim_to_os({topic}): subscribe — {e}, retry in {backoff:?}");
                        tokio::select! {
                            _ = token.cancelled() => break 'retry,
                            _ = tokio::time::sleep(backoff) => {}
                        }
                        backoff = (backoff * 2).min(BACKOFF_MAX);
                        continue 'retry;
                    }
                }
            }
        };

        tracing::info!("sim_to_os: subscribed to {sim_node}/{topic}");
        backoff = BACKOFF_INIT;

        loop {
            tokio::select! {
                _ = token.cancelled() => break 'retry,
                msg = sub.on_next_message() => match msg {
                    Some(msg) => {
                        match serde_json::from_slice::<Msg>(msg.payload().as_ref()) {
                            Ok(m) => {
                                if let Err(e) = emit_fn(runner.clone(), m).await {
                                    tracing::warn!("sim_to_os({topic}): emit — {e}");
                                }
                            }
                            Err(e) => tracing::warn!("sim_to_os({topic}): deserialize — {e}"),
                        }
                    }
                    None => {
                        tracing::info!("sim_to_os({topic}): subscription closed — re-subscribing");
                        break;
                    }
                },
            }
        }
    }
}
