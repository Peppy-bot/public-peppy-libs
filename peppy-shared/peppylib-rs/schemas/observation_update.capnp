@0xd94b2f8a61c7e350;

# Observation-slot delivery for the framework `observation_update` service.
#
# The daemon pushes ABSOLUTE observation state (never deltas) for one observer
# slot. `sequence` orders deliveries so a retried request can never roll a slot
# back: the node rejects strictly-smaller sequences (`staleSequence = true`) and
# treats an equal sequence as an idempotent retry.
#
# `sourceGeneration` is a separate, semantic counter. It advances only when the
# observed source's incarnation changes (never on the source's own peer
# transitions), and is the sole discriminator between an old and a new
# incarnation of the same source instance, whose publishes are byte-identical on
# the wire. `sourceLive` reports whether the source instance is currently in a
# non-terminal state. `hasSource` is false (and the source fields ride empty)
# only before the daemon has resolved the slot's source.

# The node replies with the shared `SlotUpdateResponse` (see slot_update.capnp).
struct ObservationUpdateRequest {
    linkId @0 :Text;
    # The receiving node's own observer-slot link_id being updated.
    sequence @1 :UInt64;
    hasSource @2 :Bool;
    sourceCoreNode @3 :Text;
    sourceInstanceId @4 :Text;
    sourceLinkId @5 :Text;
    # The producer-side link_id of the observed pairing slot (its wire segment).
    sourceGeneration @6 :UInt64;
    sourceLive @7 :Bool;
}
