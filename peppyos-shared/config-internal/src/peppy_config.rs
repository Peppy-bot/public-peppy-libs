//! Messaging-topology and lifecycle defaults shared with the runtime config.
//!
//! The full `peppy_config.json5` document loader (parsing, validation, and
//! in-place completion) is daemon-only and not part of this library. Only the
//! default buffer/grace constants the runtime config types build on, and the
//! [`PeerConfig`] tuning struct, survive here.

use serde::{Deserialize, Serialize};

/// Default subscriber channel buffer for the `Standard` QoS tier (number of
/// in-flight messages). Mirrors the historical hardcoded value.
pub const DEFAULT_STANDARD_BUFFER_SIZE: usize = 128;
/// Default subscriber channel buffer for the `HighThroughput` QoS tier (e.g.
/// sensor-data streams). Mirrors the historical hardcoded value.
pub const DEFAULT_HIGH_THROUGHPUT_BUFFER_SIZE: usize = 1024;

/// Default daemon-liveness grace period, in seconds (180 = 3 minutes). A
/// spawned node that sees no daemon heartbeat for this long shuts itself down
/// to avoid lingering as an orphan after an uncatchable daemon death.
pub const DEFAULT_DAEMON_GRACE_SECS: u64 = 180;
/// Minimum accepted grace period, in seconds. Must comfortably exceed the
/// heartbeat interval and the router-watchdog restart window so a brief daemon
/// blip never trips a node's watchdog.
pub const MIN_DAEMON_GRACE_SECS: u64 = 30;
/// Cadence, in seconds, of the daemon-liveness heartbeat each spawned node's
/// watchdog listens for (published by the daemon; see
/// `core_node::services::clock::publish_daemon_heartbeat`). Defined next to
/// `MIN_DAEMON_GRACE_SECS` so the invariant between them is enforced where
/// both values live.
pub const DAEMON_HEARTBEAT_INTERVAL_SECS: u64 = 5;
// Compile-time guard on the watchdog's false-trip margin: even several missed
// beats must fit inside the smallest accepted grace period.
const _: () = assert!(MIN_DAEMON_GRACE_SECS >= 3 * DAEMON_HEARTBEAT_INTERVAL_SECS);

/// Default cooperative-shutdown grace period, in seconds. How long the daemon
/// (on a clean ctrl+C / `systemctl stop`) and `peppy node stop` wait for a node
/// to run its cleanup hooks before force-killing its process group. 5s gives a
/// robot node room to park actuators and release hardware before it is killed.
pub const DEFAULT_SHUTDOWN_GRACE_SECS: u64 = 5;

/// Worst-case time a node runtime needs to tear down its asyncio event-loop
/// thread after its shutdown hooks finish, before the OS process can exit. A
/// background task may be executing native code (pycapnp serialization, a pyo3
/// future) that must be joined rather than killed mid-call, so this is a real
/// floor the daemon must allow for. Read by `peppylib-py` to bound the loop-join
/// and by the daemon to size its force-kill deadline above the node's real exit
/// cost. Nodes with no asyncio loop (sync-setup Python, Rust) simply finish well
/// inside it.
pub const EVENT_LOOP_JOIN_BUDGET_SECS: u64 = 5;

/// Peer-mode tuning knobs. Buffer sizes are the per-QoS subscriber channel
/// capacities used when nodes peer directly (no router relay to absorb bursts).
///
/// `#[serde(default)]` fills any field a partial `peer` block omits from
/// [`PeerConfig::default`], so every per-field default flows from the single
/// `Default` impl below rather than parallel `default = "fn"` helpers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct PeerConfig {
    pub standard_buffer_size: usize,
    pub high_throughput_buffer_size: usize,
}

impl Default for PeerConfig {
    fn default() -> Self {
        Self {
            standard_buffer_size: DEFAULT_STANDARD_BUFFER_SIZE,
            high_throughput_buffer_size: DEFAULT_HIGH_THROUGHPUT_BUFFER_SIZE,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn peer_config_defaults_match_constants() {
        let peer = PeerConfig::default();
        assert_eq!(peer.standard_buffer_size, DEFAULT_STANDARD_BUFFER_SIZE);
        assert_eq!(
            peer.high_throughput_buffer_size,
            DEFAULT_HIGH_THROUGHPUT_BUFFER_SIZE
        );
    }
}
