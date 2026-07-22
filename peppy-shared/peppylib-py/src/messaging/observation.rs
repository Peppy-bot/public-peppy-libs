//! Python bindings for the observer runtime surface: [`PyObservedSource`] (the
//! resolved source of an observer slot), [`PyObservationSlot`] (observe a slot's
//! resolved source), and [`PyObservedSubscription`] (receive the observed
//! source's publishes on one topic, yielded as `(producer, message)`).

use super::target::PyProducerRef;
use super::topics::PyTopicMessage;
use peppylib::messaging::ObservedSource;
use peppylib::runtime::{ObservationSlot, ObservedTopicSubscription};
use pyo3::prelude::*;
use std::sync::Arc;
use tokio::sync::Mutex;

/// The resolved source of an observer slot: the observed instance's full
/// `(core_node, instance_id)` wire address plus the producer-side link_id of the
/// observed pairing slot. Returned by `ObservationSlot.source()`. Purely local
/// configuration state; there is no health-derived helper, because a third
/// node's health is not knowable here.
#[pyclass(name = "ObservedSource", frozen, eq, skip_from_py_object)]
#[derive(Clone, PartialEq, Eq)]
pub struct PyObservedSource {
    pub(crate) inner: ObservedSource,
}

#[pymethods]
impl PyObservedSource {
    /// The observed source instance's full wire address.
    #[getter]
    fn producer(&self) -> PyProducerRef {
        PyProducerRef::from(self.inner.producer.clone())
    }

    /// The producer-side link_id of the observed pairing slot.
    #[getter]
    fn source_link_id(&self) -> &str {
        &self.inner.source_link_id
    }

    fn __repr__(&self) -> String {
        format!(
            "ObservedSource(producer=ProducerRef({:?}, {:?}), source_link_id={:?})",
            self.inner.producer.core_node,
            self.inner.producer.instance_id,
            self.inner.source_link_id
        )
    }
}

impl From<ObservedSource> for PyObservedSource {
    fn from(inner: ObservedSource) -> Self {
        Self { inner }
    }
}

/// Handle onto one observer slot's live observation state, obtained via
/// `node_runner.observation_slot(link_id)`. `source()` reads the resolved source
/// (or `None` before the daemon has delivered it).
#[pyclass(name = "ObservationSlot")]
pub struct PyObservationSlot {
    pub(crate) inner: ObservationSlot,
}

#[pymethods]
impl PyObservationSlot {
    /// The resolved source of this observer slot, or `None` before the daemon
    /// has delivered it.
    fn source(&self) -> Option<PyObservedSource> {
        self.inner.source().map(PyObservedSource::from)
    }
}

/// Stream of an observed source's publishes on one topic, vended by
/// `node_runner.subscribe_observed(...)`. Each `on_next_message()` yields a
/// `(producer, message)` tuple, or `None` when the runtime is torn down.
/// Delivery is a live stream, not a mailbox, and follows the source instance's
/// lifecycle independently of its peer relationship.
#[pyclass(name = "ObservedSubscription")]
pub struct PyObservedSubscription {
    pub(crate) inner: Arc<Mutex<ObservedTopicSubscription>>,
}

#[pymethods]
impl PyObservedSubscription {
    /// Wait for and receive the next `(producer, message)` from the currently
    /// observed source incarnation. Returns `None` when the runtime is torn
    /// down. Messages buffered under a superseded source incarnation are dropped
    /// before they surface here.
    fn on_next_message<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = Arc::clone(&self.inner);
        crate::py_future::future_into_py(py, async move {
            let mut subscription = inner.lock().await;
            match subscription.next().await {
                Some((producer, message)) => Ok(Some((
                    PyProducerRef::from(producer),
                    PyTopicMessage::from(message),
                ))),
                None => Ok(None),
            }
        })
    }
}
