//! Consumer-side per-slot filter applied by the messaging layer to decide
//! which producer messages reach which `depends_on` slot. The daemon-side
//! binding validator (the `daemon-config` crate in peppy) pre-resolves
//! each consumer instance's launcher / CLI binding map into per-slot
//! producer lists — each stamped with the producer's full
//! `(core_node, instance_id)` wire address; at startup, the runtime
//! [`crate::runtime::Processor`] reads each declared `link_id` and caches
//! a [`ConsumerFilter`] for the subscribe / poll / send_goal call sites.
//!
//! Every producer reference below the validator is a [`ProducerRef`]: the
//! wire addresses producers by the pair (instance_id alone is only unique
//! within one stack), so a half-address is unrepresentable here by
//! construction.
//!
//! A slot receives messages ONLY from the producers bound to it — there is
//! no wildcard subscription. The filter's cardinality selects the wire
//! strategy:
//! - empty — the slot is silent: no wire subscription is opened at all.
//! - one producer — wire-layer pin of both `from_core_node` and
//!   `from_instance_id`; no in-process filtering and no discovery.
//! - several producers — wire wildcards plus an in-process acceptance set
//!   that admits exactly the bound producers.

pub use config::runtime::ProducerRef;

/// The producers explicitly bound to one consumer slot, in binding order
/// and duplicate-free. Built by [`crate::runtime::Processor`] from the
/// daemon-supplied `slot_bindings`; an unbound slot yields
/// [`ConsumerFilter::silent`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConsumerFilter {
    producers: Vec<ProducerRef>,
}

impl ConsumerFilter {
    /// Filter of a slot bound to no producer: subscriptions stay silent and
    /// service / action calls fail as unpinned. Also the fallback for a
    /// `link_id` the daemon shipped no binding entry for.
    pub const fn silent() -> Self {
        Self {
            producers: Vec::new(),
        }
    }

    pub fn new(producers: Vec<ProducerRef>) -> Self {
        Self { producers }
    }

    pub fn producers(&self) -> &[ProducerRef] {
        &self.producers
    }

    /// `true` when the slot is bound to no producer.
    pub fn is_silent(&self) -> bool {
        self.producers.is_empty()
    }

    /// Service / action call sites (and the pinned-subscription fast path)
    /// address a single fully-pinned producer. Returns `Some(producer)`
    /// when the slot is bound to exactly one producer; `None` for silent
    /// and multi-producer slots.
    pub fn pinned_target(&self) -> Option<&ProducerRef> {
        match self.producers.as_slice() {
            [producer] => Some(producer),
            _ => None,
        }
    }
}

impl From<&[ProducerRef]> for ConsumerFilter {
    fn from(producers: &[ProducerRef]) -> Self {
        Self::new(producers.to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Core_node used by these fixtures: bindings are stack-scoped, so
    /// every producer in one consumer's binding map shares the launching
    /// daemon's core_node.
    const CORE: &str = "core_a";

    fn pref(instance_id: &str) -> ProducerRef {
        ProducerRef::new(CORE, instance_id)
    }

    #[test]
    fn silent_filter_has_no_producers_and_no_pin() {
        let filter = ConsumerFilter::silent();
        assert!(filter.is_silent());
        assert!(filter.producers().is_empty());
        assert_eq!(filter.pinned_target(), None);
    }

    #[test]
    fn single_producer_filter_pins_that_producer() {
        let filter = ConsumerFilter::new(vec![pref("cam1")]);
        assert!(!filter.is_silent());
        assert_eq!(filter.pinned_target(), Some(&pref("cam1")));
    }

    #[test]
    fn multi_producer_filter_keeps_order_and_never_pins() {
        let filter = ConsumerFilter::new(vec![pref("cam1"), pref("cam2")]);
        assert!(!filter.is_silent());
        assert_eq!(filter.producers(), &[pref("cam1"), pref("cam2")]);
        assert_eq!(filter.pinned_target(), None);
    }

    /// Producer identity is the full pair: two producers sharing an
    /// instance_id on different core_nodes are distinct entries, so the
    /// filter never collapses them into a single pin.
    #[test]
    fn same_instance_id_on_different_core_nodes_stays_multi() {
        let filter = ConsumerFilter::new(vec![pref("cam1"), ProducerRef::new("core_b", "cam1")]);
        assert_eq!(filter.pinned_target(), None);
        assert_eq!(filter.producers().len(), 2);
    }
}
