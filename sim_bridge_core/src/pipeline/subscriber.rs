use std::sync::Arc;

use peppylib::config::QoSProfile;
use peppylib::runtime::CancellationToken;
use serde::Serialize;

use crate::config::DaemonState;
use super::{BACKOFF_INIT, BACKOFF_MAX};

pub async fn run_os_to_sim<Runner, Msg, RecvFn>(
    runner: Arc<Runner>,
    token: CancellationToken,
    daemon: DaemonState,
    topic: Arc<str>,
    recv_fn: RecvFn,
) where
    Runner: Send + Sync + 'static,
    Msg: Serialize + Send + 'static,
    RecvFn: Fn(Arc<Runner>) -> super::BoxFuture<std::result::Result<(String, Msg), String>> + Send + 'static,
{
    let mut backoff = BACKOFF_INIT;

    'retry: loop {
        let handle = tokio::select! {
            _ = token.cancelled() => break,
            result = peppylib::MessengerHandle::from_host_port("localhost", daemon.messaging_port) => {
                match result {
                    Ok(h) => h,
                    Err(e) => {
                        tracing::warn!("os_to_sim({topic}): connect — {e}, retry in {backoff:?}");
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

        tracing::info!("os_to_sim({topic}): connected");
        backoff = BACKOFF_INIT;

        loop {
            tokio::select! {
                _ = token.cancelled() => break 'retry,
                result = recv_fn(runner.clone()) => {
                    let (_sender, msg) = match result {
                        Ok(m) => m,
                        Err(e) => {
                            tracing::warn!("os_to_sim({topic}): receive — {e}, reconnecting");
                            break;
                        }
                    };

                    let payload = match serde_json::to_vec(&msg) {
                        Ok(b) => b,
                        Err(e) => {
                            tracing::warn!("os_to_sim({topic}): serialize — {e}");
                            continue;
                        }
                    };

                    if let Err(e) = peppylib::TopicMessenger::emit(
                        &handle,
                        &daemon.core_node_name,
                        &format!("sim_bridge_{topic}_pub"),
                        "sim_bridge",
                        &*topic,
                        QoSProfile::Standard,
                        peppylib::Payload::from(payload),
                    )
                    .await
                    {
                        tracing::warn!("os_to_sim({topic}): peppylib emit — {e}, reconnecting");
                        break;
                    }
                },
            }
        }
    }
}
