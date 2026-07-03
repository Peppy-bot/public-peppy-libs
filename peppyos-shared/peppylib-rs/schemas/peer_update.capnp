@0xc4e8a17d92b5f3d1;

# Pairing-slot delivery for the framework `peer_update` service.
#
# The daemon pushes ABSOLUTE slot state (never deltas): `paired = true`
# carries the peer's full wire triple; `paired = false` clears the slot and
# the peer fields ride empty. `sequence` orders deliveries so a retried
# request can never roll a slot back — the node rejects strictly-smaller
# sequences (`staleSequence = true`) and treats an equal sequence as an
# idempotent retry.

struct PeerUpdateRequest {
    linkId @0 :Text;
    # The receiving node's own pairing-slot link_id being updated.
    sequence @1 :UInt64;
    paired @2 :Bool;
    peerCoreNode @3 :Text;
    peerInstanceId @4 :Text;
    peerLinkId @5 :Text;
    # The link_id of the peer's complementary slot (its producer-side wire
    # segment). Empty when `paired = false`.
}

struct PeerUpdateResponse {
    accepted @0 :Bool;
    staleSequence @1 :Bool;
    message @2 :Text;
    # Human-readable rejection reason when `accepted = false` (unknown slot,
    # stale sequence). Empty on success.
}
