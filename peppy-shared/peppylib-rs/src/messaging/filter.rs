//! Consumer-side per-slot filter applied by the messaging layer to decide
//! which producer messages reach which `depends_on` slot. The daemon-side
//! binding validator (the `daemon-config` crate in peppy) pre-resolves
//! each consumer instance's launcher / CLI binding map — keyed by the
//! slot's `link_id` — into per-slot [`config::runtime::SlotBinding`]
//! entries, each stamped with the producer's full
//! `(core_node, instance_id)` wire address; at startup, the runtime
//! [`crate::runtime::Processor`] reads each declared `link_id` and
//! synthesizes a [`ConsumerFilter`] for the subscribe / poll / send_goal
//! call.
//!
//! Every producer reference below the validator is a [`ProducerRef`]: the
//! wire addresses producers by the pair (instance_id alone is only unique
//! within one stack), so a half-address is unrepresentable here by
//! construction.
//!
//! The mapping from [`SlotBinding`] is a pure per-slot function — no slot
//! ever looks at its siblings' bindings:
//! - Pinned slots and from_any slots bound to exactly one producer pin
//!   that producer on the wire ([`ConsumerFilter::Pin`]).
//! - from_any slots bound to N ≥ 2 producers receive from all N and only
//!   those N, realized as N producer-pinned wire subscriptions
//!   ([`ConsumerFilter::OnlyFrom`]) — never a wildcard plus an in-process
//!   filter, so nothing outside the bound set traverses the wire.
//! - from_any slots deliberately left unbound are silent
//!   ([`ConsumerFilter::Silent`]): no wire subscription exists and
//!   service / action calls fail before any wire work.
//! - Slots with no binding entry at all resolve to the pure wildcard
//!   ([`ConsumerFilter::Any`]) — the standalone contract: the daemon
//!   materializes an entry for every declared slot, so a missing entry
//!   means the node runs without a daemon (or in a test fixture).

use config::runtime::SlotBinding;
use std::collections::BTreeMap;

pub use config::runtime::ProducerRef;

use super::ServiceTarget;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConsumerFilter {
    /// Wire-layer pin of both `from_core_node` and `from_instance_id` to a
    /// single producer: subscribe / poll / send_goal address exactly this
    /// producer's `(core_node, instance_id)`. No in-process filtering and
    /// no discovery required. Used for pinned slots and from_any slots
    /// bound to exactly one producer.
    Pin(ProducerRef),
    /// One producer-pinned wire subscription per listed producer: the slot
    /// receives from all of them and only them; nothing else traverses the
    /// wire. Service / action calls discover among the listed producers
    /// only. Built from `FromAnyBound` with two or more producers (a
    /// single producer collapses to [`ConsumerFilter::Pin`]); non-empty
    /// and duplicate-free by construction — each listed producer fans out
    /// one pinned wire subscription, so a repeated entry would subscribe
    /// (and deliver) twice.
    OnlyFrom(Vec<ProducerRef>),
    /// Deliberately unbound from_any slot: no wire subscription is created
    /// (zero wire traffic) and service / action calls fail before any wire
    /// work. Valid — an unbound slot means "this consumer runs without
    /// that input".
    Silent,
    /// Pure wildcard at the wire layer. Standalone mode (no daemon
    /// bindings) and test fixtures only — a daemon-launched node always
    /// carries a binding entry for every declared slot.
    Any,
}

impl ConsumerFilter {
    /// The [`ServiceTarget`] scope for a service / action call on this
    /// slot: a pinned slot addresses its producer directly, a multi-bound
    /// slot restricts discovery to the bound set, a silent slot refuses
    /// the call before any wire work, and the standalone wildcard keeps
    /// open discovery.
    pub fn call_target(&self) -> ServiceTarget<'_> {
        match self {
            ConsumerFilter::Pin(producer) => ServiceTarget::Producer(producer),
            // `OnlyFrom` is non-empty by construction from the binding
            // resolver, but nothing stops direct construction (e.g. the
            // Python `only_from([])`); normalize here so downstream call
            // sites never see an empty bound set.
            ConsumerFilter::OnlyFrom(producers) if producers.is_empty() => ServiceTarget::Unbound,
            ConsumerFilter::OnlyFrom(producers) => ServiceTarget::OneOf(producers),
            ConsumerFilter::Silent => ServiceTarget::Unbound,
            ConsumerFilter::Any => ServiceTarget::Any,
        }
    }

    /// Every producer this slot is explicitly bound to: the pinned
    /// producer, or the `OnlyFrom` set. Empty for [`ConsumerFilter::Silent`]
    /// (bound to nothing) and [`ConsumerFilter::Any`] (a wildcard names no
    /// producers). Node code can match a received message's
    /// `(core_node, instance_id)` against this set to tell bound producers
    /// apart.
    pub fn bound_producers(&self) -> &[ProducerRef] {
        match self {
            ConsumerFilter::Pin(producer) => std::slice::from_ref(producer),
            ConsumerFilter::OnlyFrom(producers) => producers,
            ConsumerFilter::Silent | ConsumerFilter::Any => &[],
        }
    }
}

/// Compute the [`ConsumerFilter`] for `link_id` from the daemon-supplied
/// per-slot bindings. A pure per-slot map — sibling slots never influence
/// each other:
/// - `Pinned` → [`ConsumerFilter::Pin`].
/// - `FromAnyBound` with one producer → [`ConsumerFilter::Pin`]: the call
///   sites pin the wire and skip discovery, exactly like an explicitly
///   pinned slot.
/// - `FromAnyBound` with two or more producers → [`ConsumerFilter::OnlyFrom`].
/// - `FromAnyUnbound` → [`ConsumerFilter::Silent`]. An empty bound set
///   needs no arm here: [`config::runtime::BoundProducers`] is non-empty
///   by construction, so an empty binding can only arrive as
///   `FromAnyUnbound`.
///
/// A `link_id` missing from `slot_bindings` resolves to
/// [`ConsumerFilter::Any`]. This is the standalone contract: the daemon
/// materializes an entry for every declared slot, so a missing entry
/// means no daemon resolved bindings at all and the slot keeps the open
/// wildcard. Do not map a missing entry to `Silent` — that would mute
/// every standalone node.
pub fn resolve_consumer_filter(
    link_id: &str,
    slot_bindings: &BTreeMap<String, SlotBinding>,
) -> ConsumerFilter {
    match slot_bindings.get(link_id) {
        None => ConsumerFilter::Any,
        Some(SlotBinding::Pinned { producer }) => ConsumerFilter::Pin(producer.clone()),
        Some(SlotBinding::FromAnyBound { producers }) => match producers.split_first() {
            (single, []) => ConsumerFilter::Pin(single.clone()),
            _ => ConsumerFilter::OnlyFrom(producers.to_vec()),
        },
        Some(SlotBinding::FromAnyUnbound) => ConsumerFilter::Silent,
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

    fn slot_map(entries: Vec<(&str, SlotBinding)>) -> BTreeMap<String, SlotBinding> {
        entries
            .into_iter()
            .map(|(k, v)| (k.to_string(), v))
            .collect()
    }

    #[test]
    fn pinned_slot_resolves_to_pin() {
        let bindings = slot_map(vec![(
            "main",
            SlotBinding::Pinned {
                producer: pref("cam1"),
            },
        )]);
        let filter = resolve_consumer_filter("main", &bindings);
        assert_eq!(filter, ConsumerFilter::Pin(pref("cam1")));
    }

    /// The single-producer collapse yields a full `ProducerRef`, which is
    /// what lets call sites skip discovery for from_any-bound-to-one
    /// slots exactly like explicitly pinned ones.
    #[test]
    fn from_any_bound_to_single_producer_collapses_to_pin() {
        let bindings = slot_map(vec![("extra", SlotBinding::from_any(vec![pref("cam1")]))]);
        let filter = resolve_consumer_filter("extra", &bindings);
        assert_eq!(filter, ConsumerFilter::Pin(pref("cam1")));
    }

    #[test]
    fn from_any_bound_to_multiple_producers_resolves_to_only_from() {
        let bindings = slot_map(vec![(
            "extra",
            SlotBinding::from_any(vec![pref("cam1"), pref("cam2")]),
        )]);
        let filter = resolve_consumer_filter("extra", &bindings);
        assert_eq!(
            filter,
            ConsumerFilter::OnlyFrom(vec![pref("cam1"), pref("cam2")])
        );
    }

    /// Resolution is a pure per-slot map: a sibling slot pinned to one of
    /// this slot's bound producers does NOT subtract it from the bound
    /// set. Both slots receive that producer's emits independently.
    #[test]
    fn multi_bound_set_ignores_sibling_pinned_claims() {
        let bindings = slot_map(vec![
            (
                "wrist_left",
                SlotBinding::Pinned {
                    producer: pref("cam1"),
                },
            ),
            (
                "extra",
                SlotBinding::from_any(vec![pref("cam1"), pref("cam2")]),
            ),
        ]);
        let filter = resolve_consumer_filter("extra", &bindings);
        assert_eq!(
            filter,
            ConsumerFilter::OnlyFrom(vec![pref("cam1"), pref("cam2")])
        );
    }

    /// An unbound from_any slot is deliberately silent: no wildcard
    /// fallback, no wire subscription, regardless of what sibling slots
    /// bind.
    #[test]
    fn from_any_unbound_is_silent() {
        let bindings = slot_map(vec![
            ("extra", SlotBinding::FromAnyUnbound),
            (
                "wrist_left",
                SlotBinding::Pinned {
                    producer: pref("cam1"),
                },
            ),
        ]);
        let filter = resolve_consumer_filter("extra", &bindings);
        assert_eq!(filter, ConsumerFilter::Silent);
        assert!(filter.bound_producers().is_empty());
    }

    /// An empty explicit binding resolves through [`SlotBinding::from_any`]
    /// to `FromAnyUnbound`, so "bound to nothing" reaches this layer as the
    /// silent slot state, never as an empty bound set (which the type makes
    /// unrepresentable).
    #[test]
    fn from_any_resolved_from_empty_set_is_silent() {
        let bindings = slot_map(vec![("extra", SlotBinding::from_any(vec![]))]);
        let filter = resolve_consumer_filter("extra", &bindings);
        assert_eq!(filter, ConsumerFilter::Silent);
    }

    /// Standalone contract: no slot binding entry for `link_id` → open
    /// wildcard. The daemon materializes an entry for every declared
    /// slot, so this only happens with no daemon at all — mapping it to
    /// `Silent` would mute every standalone node.
    #[test]
    fn missing_slot_binding_falls_back_to_any() {
        let bindings: BTreeMap<String, SlotBinding> = BTreeMap::new();
        let filter = resolve_consumer_filter("nope", &bindings);
        assert_eq!(filter, ConsumerFilter::Any);
    }

    #[test]
    fn call_target_maps_variants_onto_service_scopes() {
        let pin = ConsumerFilter::Pin(pref("cam1"));
        assert!(matches!(
            pin.call_target(),
            ServiceTarget::Producer(p) if *p == pref("cam1")
        ));

        let only = ConsumerFilter::OnlyFrom(vec![pref("cam1"), pref("cam2")]);
        assert!(matches!(
            only.call_target(),
            ServiceTarget::OneOf(set) if set == [pref("cam1"), pref("cam2")]
        ));

        assert!(matches!(
            ConsumerFilter::Silent.call_target(),
            ServiceTarget::Unbound
        ));
        assert!(matches!(
            ConsumerFilter::Any.call_target(),
            ServiceTarget::Any
        ));
    }

    #[test]
    fn bound_producers_exposes_the_explicit_set() {
        assert_eq!(
            ConsumerFilter::Pin(pref("cam1")).bound_producers(),
            &[pref("cam1")]
        );
        assert_eq!(
            ConsumerFilter::OnlyFrom(vec![pref("cam1"), pref("cam2")]).bound_producers(),
            &[pref("cam1"), pref("cam2")]
        );
        assert!(ConsumerFilter::Silent.bound_producers().is_empty());
        assert!(ConsumerFilter::Any.bound_producers().is_empty());
    }
}
