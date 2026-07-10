use peppylib::messaging::{
    ConsumerFilter, InterfaceIdentifier, NodeIdentifier, PairingIdentifier, ProducerRef,
    SenderTarget, SenderTargetError,
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

fn sender_target_error_to_py(err: SenderTargetError) -> PyErr {
    PyValueError::new_err(err.to_string())
}

/// Python wrapper for [`SenderTarget`]. Mirrors the Rust API: construct via
/// the `node(name, tag)` / `interface(name, tag)` static methods. Each
/// emission addresses either a node or an interface — never both. The wire
/// format embeds an `interface` / `node` discriminator so the two namespaces
/// cannot collide.
///
/// Subscribers that should match any publisher pass `None` for `from_target`
/// in `subscribe()` rather than constructing a wildcard `SenderTarget`.
#[pyclass(name = "SenderTarget", frozen, eq, hash, from_py_object)]
#[derive(Clone, PartialEq, Eq, Hash)]
pub struct PySenderTarget {
    pub(crate) inner: SenderTarget,
}

#[pymethods]
impl PySenderTarget {
    /// Build a node-shaped target. `name` is the node's `manifest.name`,
    /// `tag` is the node's `manifest.tag`. Raises `ValueError` if either
    /// segment fails validation (empty, contains `/`, or collides with a
    /// reserved sentinel).
    #[staticmethod]
    fn node(name: &str, tag: &str) -> PyResult<Self> {
        NodeIdentifier::new(name, tag)
            .map(|inner| Self {
                inner: SenderTarget::Node(inner),
            })
            .map_err(sender_target_error_to_py)
    }

    /// Build an interface-shaped target. Used for topics / services / actions
    /// pulled in via `interfaces.conforms_to`. Raises `ValueError` if either
    /// segment fails validation.
    #[staticmethod]
    fn interface(name: &str, tag: &str) -> PyResult<Self> {
        InterfaceIdentifier::new(name, tag)
            .map(|inner| Self {
                inner: SenderTarget::Interface(inner),
            })
            .map_err(sender_target_error_to_py)
    }

    /// Build a pairing-shaped target. Used for topics exchanged over a
    /// `depends_on.pairings` slot. Raises `ValueError` if either segment
    /// fails validation.
    #[staticmethod]
    fn pairing(name: &str, tag: &str) -> PyResult<Self> {
        PairingIdentifier::new(name, tag)
            .map(|inner| Self {
                inner: SenderTarget::Pairing(inner),
            })
            .map_err(sender_target_error_to_py)
    }

    #[getter]
    fn is_node(&self) -> bool {
        self.inner.is_node()
    }

    #[getter]
    fn is_interface(&self) -> bool {
        self.inner.is_interface()
    }

    #[getter]
    fn is_pairing(&self) -> bool {
        self.inner.is_pairing()
    }

    #[getter]
    fn name(&self) -> &str {
        self.inner.name()
    }

    #[getter]
    fn tag(&self) -> &str {
        self.inner.tag()
    }

    fn __repr__(&self) -> String {
        match &self.inner {
            SenderTarget::Node(_) => {
                format!(
                    "SenderTarget.node({:?}, {:?})",
                    self.inner.name(),
                    self.inner.tag()
                )
            }
            SenderTarget::Interface(_) => {
                format!(
                    "SenderTarget.interface({:?}, {:?})",
                    self.inner.name(),
                    self.inner.tag()
                )
            }
            SenderTarget::Pairing(_) => {
                format!(
                    "SenderTarget.pairing({:?}, {:?})",
                    self.inner.name(),
                    self.inner.tag()
                )
            }
        }
    }
}

impl PySenderTarget {
    pub(crate) fn into_inner(self) -> SenderTarget {
        self.inner
    }
}

/// Python wrapper for [`ProducerRef`] — the publisher's full
/// `(core_node, instance_id)` wire identity. Returned alongside every consumed
/// message, and accepted by every producer-targeting call site (topic
/// subscribe, service poll, action send_goal) so a consumer can pass the
/// identity it received straight back. `instance_id` alone is only unique
/// within one stack, so the pair is what distinguishes producers across the
/// whole mesh; a `from_any` consumer keys per-producer state on it.
/// `frozen, eq, hash` make it usable directly as a `dict` key, mirroring the
/// Rust `HashMap<ProducerRef, _>` idiom. `from_py_object` lets it be extracted
/// as a call argument.
#[pyclass(name = "ProducerRef", frozen, eq, hash, from_py_object)]
#[derive(Clone, PartialEq, Eq, Hash)]
pub struct PyProducerRef {
    pub(crate) inner: ProducerRef,
}

#[pymethods]
impl PyProducerRef {
    #[new]
    pub(crate) fn new(core_node: String, instance_id: String) -> Self {
        Self {
            inner: ProducerRef::new(core_node, instance_id),
        }
    }

    #[getter]
    fn core_node(&self) -> &str {
        &self.inner.core_node
    }

    #[getter]
    fn instance_id(&self) -> &str {
        &self.inner.instance_id
    }

    fn __repr__(&self) -> String {
        producer_repr(&self.inner)
    }
}

/// The one `ProducerRef` rendering, shared by [`PyProducerRef`] and
/// [`PyConsumerFilter`] reprs so both round-trip through the constructor
/// syntax.
fn producer_repr(producer: &ProducerRef) -> String {
    format!(
        "ProducerRef({:?}, {:?})",
        producer.core_node, producer.instance_id
    )
}

impl PyProducerRef {
    pub(crate) fn into_inner(self) -> ProducerRef {
        self.inner
    }
}

impl From<ProducerRef> for PyProducerRef {
    fn from(inner: ProducerRef) -> Self {
        Self { inner }
    }
}

/// Python wrapper for the per-slot
/// [`ConsumerFilter`](peppylib::messaging::ConsumerFilter): which
/// producers a consumer slot receives from / calls into. Generated code
/// obtains it from `NodeRunner.consumer_filter(link_id)` (the
/// daemon-resolved binding for that slot) and passes it straight to topic
/// `subscribe`, service `poll` / `is_reachable`, and action `send_goal` /
/// `is_reachable`. The filter argument is required at every such entry
/// point; there is no implicit default. Every filter shape is also
/// constructible directly:
/// [`any()`](Self::any) is the pure wildcard for standalone fixtures and
/// tests, [`pin()`](Self::pin) pins one producer (e.g. a `ProducerRef`
/// received alongside a consumed message), [`only_from()`](Self::only_from)
/// restricts to an explicit producer set, and [`silent()`](Self::silent)
/// is the deliberately-unbound shape.
#[pyclass(name = "ConsumerFilter", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyConsumerFilter {
    pub(crate) inner: ConsumerFilter,
}

#[pymethods]
impl PyConsumerFilter {
    /// Pure-wildcard filter: matches any producer, exactly like a slot
    /// with no daemon-resolved binding (standalone mode). The explicit
    /// opt-in for standalone fixtures and tests; daemon-launched nodes
    /// always read the real filter via `NodeRunner.consumer_filter(link_id)`.
    #[staticmethod]
    fn any() -> Self {
        Self {
            inner: ConsumerFilter::Any,
        }
    }

    /// Pin a single producer by its full `(core_node, instance_id)`
    /// identity — e.g. to pass a `ProducerRef` received alongside a
    /// consumed message straight back into a call site. Daemon-resolved
    /// pinned slots arrive in this shape via `consumer_filter(link_id)`.
    #[staticmethod]
    fn pin(producer: PyProducerRef) -> Self {
        Self {
            inner: ConsumerFilter::Pin(producer.into_inner()),
        }
    }

    /// Restrict to an explicit producer set: receive from / discover
    /// among all of them and only them. Mirrors a bound multi-producer
    /// `from_any` slot. Duplicate producers are canonicalized away (the
    /// first occurrence keeps its position) — daemon-resolved bound sets
    /// are duplicate-free by construction, and this direct constructor
    /// upholds the same invariant so a repeated entry can never fan out
    /// duplicate pinned subscriptions.
    #[staticmethod]
    fn only_from(producers: Vec<PyProducerRef>) -> Self {
        let mut unique: Vec<ProducerRef> = Vec::with_capacity(producers.len());
        for producer in producers {
            let producer = producer.into_inner();
            if !unique.contains(&producer) {
                unique.push(producer);
            }
        }
        Self {
            inner: ConsumerFilter::OnlyFrom(unique),
        }
    }

    /// The deliberately-unbound shape: no subscription, calls fail before
    /// any wire work. Mirrors an unbound `from_any` slot.
    #[staticmethod]
    fn silent() -> Self {
        Self {
            inner: ConsumerFilter::Silent,
        }
    }

    /// Every producer this slot is explicitly bound to (empty for silent
    /// and wildcard filters). Mirrors the Rust `bound_producers()`; lets
    /// node code enumerate the bound set to key per-producer state.
    #[getter]
    fn bound_producers(&self) -> Vec<PyProducerRef> {
        self.inner
            .bound_producers()
            .iter()
            .cloned()
            .map(PyProducerRef::from)
            .collect()
    }

    fn __repr__(&self) -> String {
        match &self.inner {
            ConsumerFilter::Pin(producer) => {
                format!("ConsumerFilter.pin({})", producer_repr(producer))
            }
            ConsumerFilter::OnlyFrom(producers) => {
                let refs: Vec<String> = producers.iter().map(producer_repr).collect();
                format!("ConsumerFilter.only_from([{}])", refs.join(", "))
            }
            ConsumerFilter::Silent => "ConsumerFilter.silent()".to_string(),
            ConsumerFilter::Any => "ConsumerFilter.any()".to_string(),
        }
    }
}

impl PyConsumerFilter {
    pub(crate) fn into_inner(self) -> ConsumerFilter {
        self.inner
    }
}

impl From<ConsumerFilter> for PyConsumerFilter {
    fn from(inner: ConsumerFilter) -> Self {
        Self { inner }
    }
}
