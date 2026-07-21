use super::target::{PyProducerRef, PySenderTarget};
use super::{PyMessengerHandle, future_into_py_unit, to_py_err};
use crate::config::PyQoSProfile;
use peppylib::messaging::{BoundSetSubscription, Subscription, TopicMessenger, TopicPublisher};
use peppylib::types::{Message, Payload};
use pyo3::prelude::*;
use pyo3::types::PyBytes;
use std::sync::Arc;
use tokio::sync::Mutex;

/// Python wrapper for TopicMessage
#[pyclass(name = "TopicMessage", skip_from_py_object)]
#[derive(Clone)]
pub struct PyTopicMessage {
    // Held as a `Payload` (refcounted `Bytes`) rather than `Vec<u8>` so cloning a
    // `PyTopicMessage` is a refcount bump, and the wire bytes are copied into a
    // Python buffer once, in the getter, instead of also being copied eagerly at
    // construction.
    pub(crate) payload: Payload,
    pub(crate) instance_id: String,
    pub(crate) core_node: String,
    pub(crate) link_id: String,
}

#[pymethods]
impl PyTopicMessage {
    #[getter]
    fn payload<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.payload.as_ref())
    }

    #[getter]
    fn instance_id(&self) -> &str {
        &self.instance_id
    }

    #[getter]
    fn core_node(&self) -> &str {
        &self.core_node
    }

    /// The producer's bound link_id, parsed from the inbound topic keyexpr.
    /// On a pairing subscription this is the peer's own slot link_id, which is
    /// what the Rust forwarding path re-checks against the slot's pin; exposing
    /// it here lets a Python node make the same check. Empty for messages that
    /// arrived via a non-topic path, where the keyexpr encodes no link_id.
    #[getter]
    fn link_id(&self) -> &str {
        &self.link_id
    }

    /// The publisher's full `(core_node, instance_id)` identity as a structured
    /// [`PyProducerRef`]. This is what generated consumed-topic callbacks return
    /// alongside the message; consumers key per-slot state on it.
    #[getter]
    fn producer(&self) -> PyProducerRef {
        PyProducerRef::new(self.core_node.clone(), self.instance_id.clone())
    }
}

impl From<Message> for PyTopicMessage {
    fn from(msg: Message) -> Self {
        Self {
            payload: msg.payload(),
            instance_id: msg.instance_id().to_string(),
            core_node: msg.core_node().to_string(),
            link_id: msg.link_id().to_string(),
        }
    }
}

/// Python wrapper for Subscription
#[pyclass(name = "Subscription")]
pub struct PySubscription {
    inner: Arc<Mutex<Subscription>>,
}

#[pymethods]
impl PySubscription {
    /// Wait for and receive the next message.
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

/// Python wrapper for a dep slot's merged bound-set subscription: one
/// producer-pinned wire subscription per bound producer, merged behind a
/// single `on_next_message` that yields `(producer, message)` tuples. See
/// [`BoundSetSubscription`] for the merge semantics (per-producer order,
/// fair polling, drain-before-shutdown, empty-set pending).
#[pyclass(name = "BoundSetSubscription")]
pub struct PyBoundSetSubscription {
    inner: Arc<Mutex<BoundSetSubscription>>,
}

#[pymethods]
impl PyBoundSetSubscription {
    /// Wait for the next message from any bound producer. Returns a
    /// `(ProducerRef, TopicMessage)` tuple, or `None` once the node is
    /// shutting down and no queued message remains.
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

/// Python wrapper for TopicMessenger
#[pyclass(name = "TopicMessenger")]
pub struct PyTopicMessenger;

#[pymethods]
impl PyTopicMessenger {
    /// Subscribe to a topic from one producer. Pass
    /// `SenderTarget.node(name, tag)` or `SenderTarget.interface(name, tag)`
    /// to match the publisher's target. `from_producer` is a full
    /// [`ProducerRef`](peppylib::messaging::ProducerRef) identity pinned
    /// on the wire — only that producer's publishes reach the
    /// subscription. Generated consumed topics never splice this: they go
    /// through [`subscribe_bound_set`](Self::subscribe_bound_set), which
    /// covers the slot's complete bound set for every cardinality.
    #[staticmethod]
    #[pyo3(signature = (messenger, as_core_node, as_instance_id, from_target, to_topic, from_producer, qos))]
    #[allow(clippy::too_many_arguments)]
    fn subscribe<'py>(
        py: Python<'py>,
        messenger: &PyMessengerHandle,
        as_core_node: String,
        as_instance_id: String,
        from_target: PySenderTarget,
        to_topic: String,
        from_producer: PyProducerRef,
        qos: PyQoSProfile,
    ) -> PyResult<Bound<'py, PyAny>> {
        let handle = messenger.inner.clone();
        let from_target = from_target.into_inner();
        crate::py_future::future_into_py(py, async move {
            let subscription = TopicMessenger::subscribe(
                &handle,
                &as_core_node,
                &as_instance_id,
                from_target,
                &to_topic,
                &from_producer.into_inner(),
                qos.into(),
            )
            .await
            .map_err(to_py_err)?;

            Ok(PySubscription {
                inner: Arc::new(Mutex::new(subscription)),
            })
        })
    }

    /// Subscribe to a topic across a dep slot's complete bound producer
    /// set: one producer-pinned wire subscription per member of
    /// `bound_producers`, merged behind one [`PyBoundSetSubscription`]
    /// yielding `(producer, message)` tuples. An empty set opens zero wire
    /// subscriptions and yields nothing until `shutdown` fires (the
    /// `zero_or_more` empty-slot case). Generated code splices
    /// `node_runner.bound_producers(link_id)` and the node's cancellation
    /// token here.
    #[staticmethod]
    #[pyo3(signature = (messenger, as_core_node, as_instance_id, from_target, to_topic, bound_producers, qos, shutdown))]
    #[allow(clippy::too_many_arguments)]
    fn subscribe_bound_set<'py>(
        py: Python<'py>,
        messenger: &PyMessengerHandle,
        as_core_node: String,
        as_instance_id: String,
        from_target: PySenderTarget,
        to_topic: String,
        bound_producers: Vec<PyProducerRef>,
        qos: PyQoSProfile,
        shutdown: &crate::runtime::PyCancellationToken,
    ) -> PyResult<Bound<'py, PyAny>> {
        let handle = messenger.inner.clone();
        let from_target = from_target.into_inner();
        let producers: Vec<peppylib::messaging::ProducerRef> = bound_producers
            .into_iter()
            .map(PyProducerRef::into_inner)
            .collect();
        let shutdown = shutdown.inner_token();
        crate::py_future::future_into_py(py, async move {
            let subscription = TopicMessenger::subscribe_bound_set(
                &handle,
                &as_core_node,
                &as_instance_id,
                from_target,
                &to_topic,
                &producers,
                qos.into(),
                shutdown,
            )
            .await
            .map_err(to_py_err)?;

            Ok(PyBoundSetSubscription {
                inner: Arc::new(Mutex::new(subscription)),
            })
        })
    }

    /// Declare a reusable publisher for a topic and return a [`PyTopicPublisher`].
    ///
    /// This is the only topic-publish path. The central messenger lock is taken
    /// ONCE here, at declaration; every subsequent `publisher.publish(...)` is
    /// lock-free. Declare once, then publish per message (a camera streaming
    /// frames, a sensor at rate).
    ///
    /// `link_id` binds the publisher under a concrete producer-side link_id
    /// wire segment (pairing slot publishers pass their own slot link_id);
    /// `None` falls back to the reserved default `_` segment.
    #[staticmethod]
    #[pyo3(signature = (messenger, as_core_node, as_instance_id, as_target, as_topic_name, qos, link_id=None))]
    #[allow(clippy::too_many_arguments)]
    fn declare_publisher<'py>(
        py: Python<'py>,
        messenger: &PyMessengerHandle,
        as_core_node: String,
        as_instance_id: String,
        as_target: PySenderTarget,
        as_topic_name: String,
        qos: PyQoSProfile,
        link_id: Option<String>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let handle = messenger.inner.clone();
        let as_target = as_target.into_inner();
        crate::py_future::future_into_py(py, async move {
            let publisher = TopicMessenger::declare_publisher(
                &handle,
                &as_core_node,
                &as_instance_id,
                as_target,
                link_id.as_deref(),
                &as_topic_name,
                qos.into(),
            )
            .await
            .map_err(to_py_err)?;
            Ok(PyTopicPublisher { inner: publisher })
        })
    }
}

/// Python wrapper for a lock-free per-topic publisher, vended by
/// [`PyTopicMessenger::declare_publisher`]. Holds the topic binding so each
/// `publish` skips the central messenger lock; clone-cheap (an `Arc` bump).
#[pyclass(name = "TopicPublisher")]
pub struct PyTopicPublisher {
    inner: TopicPublisher,
}

#[pymethods]
impl PyTopicPublisher {
    /// Publish a payload on the declared topic. Lock-free on the hot path.
    fn publish<'py>(&self, py: Python<'py>, payload: Vec<u8>) -> PyResult<Bound<'py, PyAny>> {
        let publisher = self.inner.clone();
        future_into_py_unit(py, async move {
            publisher
                .publish(Payload::from(payload))
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }
}
