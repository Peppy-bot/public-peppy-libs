//! Python bindings for the observer runtime surface: [`PyObservedSource`] (one
//! observed pairing of an observer slot), [`PyObservationSlot`] (read a
//! `cardinality: "one"` slot's source), [`PyObservationSlotSet`] (read a
//! multi-member slot's whole set), and [`PyObservedSubscription`] (receive the
//! observed sources' publishes on one topic, yielded as `(producer, message)`).

use super::target::PyProducerRef;
use super::topics::PyTopicMessage;
use peppylib::messaging::ObservedSource;
use peppylib::runtime::{ObservationSlot, ObservationSlotSet, ObservedTopicSubscription};
use pyo3::prelude::*;
use std::sync::Arc;
use tokio::sync::Mutex;

/// One pairing an observer slot observes: the observed instance's full
/// `(core_node, instance_id)` wire address plus the producer-side link_id of the
/// observed pairing slot. Returned by `ObservationSlot.source()` and
/// `ObservationSlotSet.sources()`. Purely local configuration state; there is no
/// health-derived helper, because a third node's health is not knowable here.
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

/// Handle onto a `cardinality: "one"` observer slot's live observation state,
/// obtained via `node_runner.observation_slot(link_id)`. `source()` reads the
/// observed source (or `None` before the daemon has delivered it). Multi-member
/// slots are read through [`PyObservationSlotSet`] instead.
#[pyclass(name = "ObservationSlot")]
pub struct PyObservationSlot {
    pub(crate) inner: ObservationSlot,
}

#[pymethods]
impl PyObservationSlot {
    /// The observed source of this slot, or `None` before the daemon has
    /// delivered it.
    ///
    /// Raises `PanicException` if the slot holds more than one member, which a
    /// `cardinality: "one"` slot cannot have: reading a multi-member slot
    /// through this accessor is stale codegen, so regenerate the node's
    /// bindings and read it through `ObservationSlotSet.sources()`.
    fn source(&self) -> Option<PyObservedSource> {
        self.inner.source().map(PyObservedSource::from)
    }
}

/// Handle onto a multi-member observer slot's live observation state (a
/// `one_or_more` or `zero_or_more` slot), obtained via
/// `node_runner.observation_slot_set(link_id)`. The set is live: the daemon
/// replaces it whole whenever the plan's observed pairings change, so an empty
/// list is legal at any instant.
#[pyclass(name = "ObservationSlotSet")]
pub struct PyObservationSlotSet {
    pub(crate) inner: ObservationSlotSet,
}

#[pymethods]
impl PyObservationSlotSet {
    /// Every pairing this slot currently observes, in plan order: the order the
    /// launcher's array or the `--link` occurrences wrote, so member N here is
    /// the deployment's Nth entry for this slot.
    fn sources(&self) -> Vec<PyObservedSource> {
        self.inner
            .sources()
            .into_iter()
            .map(PyObservedSource::from)
            .collect()
    }

    fn __repr__(&self) -> String {
        let sources = self
            .sources()
            .iter()
            .map(PyObservedSource::__repr__)
            .collect::<Vec<_>>();
        format!("ObservationSlotSet(sources=[{}])", sources.join(", "))
    }
}

/// Stream of the observed sources' publishes on one topic, fanned in across the
/// slot's whole member set and vended by `node_runner.subscribe_observed(...)`.
/// Each `on_next_message()` yields a `(producer, message)` tuple, or `None` when
/// the runtime is torn down. Delivery is a live stream, not a mailbox, and
/// follows each source instance's lifecycle independently of its peer
/// relationship.
#[pyclass(name = "ObservedSubscription")]
pub struct PyObservedSubscription {
    pub(crate) inner: Arc<Mutex<ObservedTopicSubscription>>,
}

#[pymethods]
impl PyObservedSubscription {
    /// Wait for and receive the next `(producer, message)` from any currently
    /// observed source incarnation. Every cardinality fans in the same way and
    /// every message is tagged with the member that published it. Returns `None`
    /// when the runtime is torn down. Messages buffered under a superseded
    /// source incarnation, or under a member the slot has since dropped, are
    /// dropped before they surface here.
    fn on_next_message<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = Arc::clone(&self.inner);
        crate::py_future::future_into_py(py, async move {
            let mut subscription = inner.lock().await;
            match subscription.on_next_message().await {
                Some((producer, message)) => Ok(Some((
                    PyProducerRef::from(producer),
                    PyTopicMessage::from(message),
                ))),
                None => Ok(None),
            }
        })
    }
}
