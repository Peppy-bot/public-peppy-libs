//! Python bindings for the pairing runtime surface: [`PyPeerInfo`] (identity
//! of the peer paired on a slot), [`PyPeerSlot`] (observe a slot's pin state),
//! and [`PyPeerSubscription`] (receive the paired peer's publishes).

use super::iface::PyProducerRef;
use super::topics::PyTopicMessage;
use peppylib::messaging::PeerInfo;
use peppylib::runtime::{PeerSlot, PeerSubscription};
use pyo3::prelude::*;
use std::sync::Arc;
use tokio::sync::Mutex;

/// Identity of the peer paired on a pairing slot: the peer instance's full
/// `(core_node, instance_id)` wire address plus the link_id of the peer's own
/// complementary slot. Returned by `PeerSlot.paired()` / `wait_paired()`.
#[pyclass(name = "PeerInfo", frozen, eq, skip_from_py_object)]
#[derive(Clone, PartialEq, Eq)]
pub struct PyPeerInfo {
    pub(crate) inner: PeerInfo,
}

#[pymethods]
impl PyPeerInfo {
    /// The peer instance's full wire address.
    #[getter]
    fn producer(&self) -> PyProducerRef {
        PyProducerRef::from(self.inner.producer.clone())
    }

    /// The link_id of the peer's complementary pairing slot.
    #[getter]
    fn peer_link_id(&self) -> &str {
        &self.inner.peer_link_id
    }

    fn __repr__(&self) -> String {
        format!(
            "PeerInfo(producer=ProducerRef({:?}, {:?}), peer_link_id={:?})",
            self.inner.producer.core_node, self.inner.producer.instance_id, self.inner.peer_link_id
        )
    }
}

impl From<PeerInfo> for PyPeerInfo {
    fn from(inner: PeerInfo) -> Self {
        Self { inner }
    }
}

/// Handle onto one pairing slot's live pin state, obtained via
/// `node_runner.peer(link_id)`. `paired()` reads the current peer (or `None`
/// while unpaired); `wait_paired()` awaits one.
#[pyclass(name = "PeerSlot")]
pub struct PyPeerSlot {
    pub(crate) inner: PeerSlot,
}

#[pymethods]
impl PyPeerSlot {
    /// The currently paired peer, or `None` while the slot is unpaired.
    fn paired(&self) -> Option<PyPeerInfo> {
        self.inner.paired().map(PyPeerInfo::from)
    }

    /// Wait until the slot is paired and return the peer's identity. Returns
    /// immediately when already paired.
    fn wait_paired<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let mut slot = self.inner.clone();
        crate::py_future::future_into_py(py, async move {
            slot.wait_paired()
                .await
                .map(PyPeerInfo::from)
                .map_err(super::to_py_err)
        })
    }
}

/// Stream of the paired peer's publishes on one topic of a pairing slot,
/// vended by `node_runner.subscribe_peer(...)`. Yields nothing while the slot
/// is unpaired; delivery follows the slot's live pin (pair, re-pin, clear).
#[pyclass(name = "PeerSubscription")]
pub struct PyPeerSubscription {
    pub(crate) inner: Arc<Mutex<PeerSubscription>>,
}

#[pymethods]
impl PyPeerSubscription {
    /// Wait for and receive the next message from the currently paired peer.
    /// Returns `None` when the runtime is torn down.
    fn on_next_message<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = Arc::clone(&self.inner);
        crate::py_future::future_into_py(py, async move {
            let mut subscription = inner.lock().await;
            match subscription.on_next_message().await {
                Some(message) => Ok(Some(PyTopicMessage::from(message))),
                None => Ok(None),
            }
        })
    }
}
