@0xa5c9659cafd96bfc;

# Daemon-to-daemon messages for a federated launch.
#
# Two kinds live here, and the distinction matters:
#
#   * The RESERVATION exchange, which the coordinator drives during preflight.
#     It is the serialization point for the whole launch: the coordinator holds
#     a reservation on every participant before anything is torn down, so two
#     coordinators racing the same machines are refused before either has
#     replaced a stack. Daemons never negotiate among themselves.
#
#   * RELATIONSHIP NOTIFICATIONS, which flow at runtime between the daemon that
#     owns an instance and daemons that hold a relationship with it. These are
#     best-effort and idempotent by construction: the owning daemon stays
#     authoritative and the notification only reports what already happened, so
#     a dropped one degrades to staleness rather than to disagreement.
#
# There is deliberately no "which daemons hold a slice of my launch" message.
# The stack-list response already carries `(launchId, coordinatorCoreNode)`, so
# rediscovery is the presence fan-out `stack list` already performs, filtered by
# launch id.

# Reserves one participant for one launch.
struct ParticipantReserveRequest {
    # Identity of the launch being reserved for.
    launchId @0 :Text;
    # The coordinator driving it. The participant watches this core node's
    # presence for as long as it holds the reservation, and releases it if the
    # coordinator disappears, so a coordinator that dies mid-launch cannot wedge
    # a machine until its next daemon restart.
    coordinatorCoreNode @1 :Text;
    # JSON5-encoded launcher `DeploymentSource`, one per deployment placed on
    # this participant. The participant resolves its own manifests rather than
    # being handed the coordinator's: the coordinator then needs no reachability
    # to sources it does not use, and the manifest the coordinator validates is
    # provably the one the participant will spawn from, because it is the same
    # daemon and the same cache.
    #
    # A `local:` source never appears here. Its path names a tree on the
    # coordinator's filesystem, so the plan refuses it for any deployment placed
    # off-coordinator.
    deploymentSourcesJson5 @2 :List(Text);
}

# One manifest as a participant resolved it, aligned by index with the
# request's `deploymentSourcesJson5`.
struct ResolvedManifest {
    # JSON5-serialized NodeConfig.
    configJson5 @0 :Text;
    # SHA256 of the manifest. The coordinator echoes this back on the instance
    # plan it later dispatches, and the participant refuses the start if its
    # re-resolved manifest no longer hashes the same. That closes the window
    # between preflight and dispatch in which a cache could move.
    configSha256 @1 :Text;
}

struct ParticipantReserveResponse {
    accepted @0 :Bool;
    # Populated when `accepted` is false: already reserved for another launch,
    # or a launch already running.
    rejectionReason @1 :Text;
    # The participant's peppy version, compared against the coordinator's own
    # so a mixed-version federation is refused before any stack is touched.
    # Same string the info service reports, so there is one source of truth for
    # "what version is that daemon".
    peppyVersion @2 :Text;
    # The participant's root entity instance id, folded into the coordinator's
    # global instance-id uniqueness check.
    rootInstanceId @3 :Text;
    # One entry per requested deployment source, in request order.
    manifests @4 :List(ResolvedManifest);
}

# Tells a reserved participant to replace its stack slice: tear down whatever
# it is running, clear it, and record that the slice now belongs to this
# launch.
#
# Separate from the reservation on purpose. Reserving is NON-DESTRUCTIVE and
# happens before the coordinator knows whether every participant will accept,
# so folding the teardown into it would replace a stack on machine A for a
# launch that machine B is about to refuse. This is the commit point: the
# coordinator sends it only once every participant is reserved.
struct ParticipantSliceBeginRequest {
    # Must match the reservation this participant holds. A participant that is
    # reserved for a different launch refuses, which is what stops a stale
    # coordinator from wiping a machine out from under the launch that owns it.
    launchId @0 :Text;
}

struct ParticipantSliceBeginResponse {
    began @0 :Bool;
    rejectionReason @1 :Text;
}

# Releases a reservation the coordinator obtained but no longer needs, either
# because a later participant refused or because the launch finished.
# Idempotent: releasing a reservation that is not held succeeds.
struct ParticipantReleaseRequest {
    launchId @0 :Text;
}

struct ParticipantReleaseResponse {
    # False only when the reservation is held for a DIFFERENT launch, which the
    # caller has no standing to release.
    released @0 :Bool;
    rejectionReason @1 :Text;
}

# Asks a peer daemon to record its half of a cross-daemon pair and deliver the
# pin to its own endpoint.
#
# Pairing is symmetric, but a registry is not shared: each daemon records the
# pairs its own instances are in, and only a daemon can deliver a pin to a node
# it hosts. A same-daemon pair is committed and delivered in one place; a
# cross-daemon one needs the second half committed where the second node lives,
# and this is that request.
#
# Sent by the daemon starting the LATER endpoint, which is the side that has
# just validated the pair. The receiving side does not re-derive the pairing
# rules: it cannot read the sender's manifests, and the launch coordinator
# already checked both against each other before anything started.
struct PairCommitRequest {
    pairingName @0 :Text;
    pairingTag @1 :Text;
    # The endpoint that lives on the RECEIVING daemon.
    localInstanceId @2 :Text;
    localLinkId @3 :Text;
    localRole @4 :Text;
    # The endpoint on the SENDING daemon, which the receiver pins its node to.
    peerCoreNode @5 :Text;
    peerInstanceId @6 :Text;
    peerLinkId @7 :Text;
    peerRole @8 :Text;
}

struct PairCommitResponse {
    committed @0 :Bool;
    # Populated when `committed` is false. The sender reverts its own half, so
    # a pair is never left established on one machine and absent on the other.
    rejectionReason @1 :Text;
}

# Best-effort notification from the daemon that owns an instance to a daemon
# holding a relationship with it. Idempotent: the receiver converges on the
# reported state, so a duplicate changes nothing and a lost one leaves the
# receiver stale rather than wrong.
struct RelationshipNotification {
    # The instance whose lifecycle moved, and the daemon that owns it.
    instanceId @0 :Text;
    coreNode @1 :Text;
    event :union {
        # The instance reached Running under a fresh incarnation. Observing
        # daemons advance their incarnation counter for this source and
        # redeliver its pin, which is what makes an observer drop and redeclare
        # its subscription across a source restart.
        reachedRunning @2 :Void;
        # The instance stopped or died. A daemon holding a pair with it
        # dissolves that pair locally. Dissolution stays authoritative on the
        # daemon that owns the dead instance; this only propagates it.
        #
        # An unreachable daemon cannot send this, which is exactly why a node
        # whose correctness depends on freshness owns a staleness watchdog
        # rather than trusting the framework to notice a partition.
        stopped @3 :Void;
    }
}

struct RelationshipNotificationAck {
    received @0 :Bool;
}
