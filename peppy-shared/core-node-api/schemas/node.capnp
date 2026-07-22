@0xa8f5c2b9e4d3f1aa;

# Node message structures for core-node services

# Feedback stream tag carried by NodeAdd/NodeBuild/NodeRun feedback messages.
enum FeedbackStream {
    stdout @0;
    stderr @1;
    # Out-of-band warning emitted by the daemon itself.
    warning @2;
}

# Node List service
struct NodeListRequest {}

struct NodeListResponse {
    # JSON-serialized graph representation (SerializedNodeGraph)
    graphJson @0 :Text;
    # Hostname of the daemon serving this stack
    hostName @1 :Text;
    # Presence identity of the serving daemon: its core-node name and
    # daemon-generation instance id, matching its core-node presence token.
    coreNode @2 :Text;
    instanceId @3 :Text;
}

# Node Add Action (streaming version with feedback)
struct NodeAddGoal {
    # Git commit hash of the node being added
    gitHash @0 :Text;
    # Source of the node (filesystem path, git repository, HTTP URL, or
    # a `name:tag` lookup in the repo cache).
    source :union {
        # Filesystem path to the node directory
        fs @1 :Text;
        # Git repository source
        git @2 :NodeAddGitSource;
        # HTTP URL source
        http @3 :Text;
        # Reference a node by `name:tag` — the daemon looks it up in
        # `~/.peppy/cache/nodes.json5` and resolves transitive
        # dependencies as an atomic batch.
        repoNode @8 :NodeAddRepoNodeSource;
    }
    # Optional SHA256 checksum for HTTP sources
    httpSha256 @6 :Text;
    # Environment variables to apply when executing build_cmd (e.g. PATH)
    envVars @4 :List(EnvVar);
    # Timeout in seconds for the add operation (used to report remaining time when busy)
    timeoutSecs @5 :UInt64;
    # When true, cancel any in-progress add action and start a new one
    force @7 :Bool;
}

struct NodeAddRepoNodeSource {
    # Node name as it appears in `nodes.json5`
    name @0 :Text;
    # Node tag as it appears in `nodes.json5`
    tag @1 :Text;
}

struct EnvVar {
    key @0 :Text;
    value @1 :Text;
}

struct NodeAddGitSource {
    # URL of the git repository
    repoUrl @0 :Text;
    # Path within the repository to the node
    repoPath @1 :Text;
    # Optional git ref (tag/branch/commit) to checkout before reading repoPath
    repoRef @2 :Text;
}

struct NodeAddGoalResponse {
    # Whether the goal was accepted
    accepted @0 :Bool;
    # Path to the log file (empty if rejected)
    logPath @1 :Text;
    # Rejection reason (empty if accepted)
    rejectionReason @2 :Text;
}

struct NodeAddFeedback {
    stream @0 :FeedbackStream;
    # The line of output
    line @1 :Text;
}

struct NodeAddResult {
    # Whether the node was added successfully
    success @0 :Bool;
    # Error message if failed
    errorMessage @1 :Text;
    # Path to the log file containing stdout/stderr output
    logPath @2 :Text;
    # Name of the added node (empty on failure)
    nodeName @3 :Text;
    # Tag of the added node (empty on failure)
    nodeTag @4 :Text;
}

# Node Build Action — drives the build of a previously-added node
struct NodeBuildGoal {
    nodeName @0 :Text;
    nodeTag @1 :Text;
    envVars @2 :List(EnvVar);
    timeoutSecs @3 :UInt64;
    force @4 :Bool;
}

struct NodeBuildGoalResponse {
    accepted @0 :Bool;
    logPath @1 :Text;
    rejectionReason @2 :Text;
}

struct NodeBuildFeedback {
    stream @0 :FeedbackStream;
    line @1 :Text;
}

struct NodeBuildResult {
    success @0 :Bool;
    errorMessage @1 :Text;
    # Path to the resulting .sif/archive in storage (empty on failure)
    artifactPath @2 :Text;
    logPath @3 :Text;
}

# Node Init service
struct NodeInitRequest {
    # Root directory where the node will be created
    nodeRootDir @0 :Text;
    # Name of the node (used for directory and package name)
    nodeName @1 :Text;
    # Git commit hash of the node being initialized
    gitHash @2 :Text;
    # Toolchain to use for the node ("cargo" or "uv")
    toolchain @3 :Text;
    # Whether to initialize the node with container support
    withContainer @4 :Bool;
}

struct NodeInitResponse {
    # Whether the init was successful
    success @0 :Bool;
    # Error message if failed
    errorMessage @1 :Text;
}

# Node Sync service
struct NodeSyncRequest {
    # Root directory of the node/workspace
    nodeRootDir @0 :Text;
    # Git commit hash of the node being synced
    gitHash @1 :Text;
    # When true, dependencies missing from the persistent node stack are
    # looked up in `~/.peppy/cache/nodes.json5` and materialized through
    # the existing FS / git / HTTP repository cache before peppygen
    # generation proceeds. Resolution still prefers the node stack; the
    # repository cache is consulted only as a fallback.
    includeRepositories @2 :Bool;
}

struct NodeSyncResponse {
    # Whether the sync was successful
    success @0 :Bool;
    # Error message if failed
    errorMessage @1 :Text;
    # `name:tag` of every dependency the daemon resolved through the
    # persistent node stack. Always populated on success; empty on failure.
    resolvedFromStack @2 :List(Text);
    # Every dependency the daemon resolved by fetching from the repository
    # cache. Only populated when the request set `includeRepositories`.
    resolvedFromRepositories @3 :List(RepoResolvedEntry);
}

struct RepoResolvedEntry {
    name @0 :Text;
    tag @1 :Text;
    # "fs" | "git" | "url"
    sourceKind @2 :Text;
}

struct NodeRunGoal {
    # Runtime configuration in JSON5 format (PEPPY_RUNTIME_CONFIG)
    runtimeConfigJson5 @0 :Text;
    # Name of the node to run
    nodeName @1 :Text;
    # Tag of the node to run
    tag @2 :Text;
    # Environment variables to apply when executing run_cmd (e.g. PATH)
    envVars @3 :List(EnvVar);
    # Timeout in seconds for the run operation (used to report remaining time when busy)
    timeoutSecs @4 :UInt64;
    # Pairing requests from `--pair <link_id>@<peer_instance>[/<peer_link_id>]`
    # or a launch plan: commands to the daemon, not resolved config. The
    # daemon validates and reserves each pair BEFORE spawning and delivers it
    # live after the instance commits to Running.
    requestedPairs @5 :List(PairRequest);
    # Pairing slot link_ids deliberately left unpaired via `--defer-pair` /
    # the launcher's `defer_pairings:`. Together with requestedPairs and
    # coveredPairs these must cover every required pairing slot of the
    # manifest or the daemon rejects the run.
    deferredPairs @6 :List(Text);
    # Pairing slots of this instance that a LATER-starting instance of the
    # same `stack launch` will claim through its own requestedPairs entry;
    # each entry names that future peer. A launch-mechanism marker, not user
    # intent: the slot boots unpaired and needs no action, unlike a
    # deferredPairs entry which records a deliberate opt-out. Never set by
    # the CLI.
    coveredPairs @7 :List(PairRequest);
    # Observer requests from `--link <observer_link>@<source_instance>[/<source_link>]`
    # or a launch plan: the observer slots of this instance and the source each
    # taps. Like requestedPairs these are commands to the daemon, not resolved
    # config: the daemon registers each observation BEFORE the instance commits
    # to Running so it delivers the source pin the moment both are up (and
    # again whenever the source restarts). The source's core_node is always
    # this daemon's, so it is not carried here.
    plannedObservations @8 :List(ObservationRequest);
}

struct PairRequest {
    # The starting node's own pairing-slot link_id.
    linkId @0 :Text;
    # The peer instance this slot pairs with.
    peerInstanceId @1 :Text;
    # The complementary slot on the peer, when the request pins one. Empty
    # means unpinned: exactly one available complementary slot must exist on
    # the peer and the daemon resolves it.
    peerLinkId @2 :Text;
}

struct ObservationRequest {
    # The starting node's own observer-slot link_id.
    observerLinkId @0 :Text;
    # The source instance whose role topics this slot observes.
    sourceInstanceId @1 :Text;
    # The source-side participant slot link_id: the segment the source
    # publishes its observed role topics under, and the third element of the
    # observer's fully-pinned subscription. Always resolved by the planner (the
    # CLI preflight or the launcher), never empty.
    sourceLinkId @2 :Text;
}

struct NodeRunGoalResponse {
    # Whether the goal was accepted
    accepted @0 :Bool;
    # Path to the log file (empty if rejected)
    logPath @1 :Text;
    # Rejection reason (empty if accepted)
    rejectionReason @2 :Text;
}

struct NodeRunFeedback {
    stream @0 :FeedbackStream;
    # The line of output
    line @1 :Text;
}

struct NodeRunResult {
    # Whether the run was successful
    success @0 :Bool;
    # Error message if failed (optional)
    errorMessage @1 :Text;
    # Process ID of the running node (0 if not available or failed)
    pid @2 :UInt32;
}

# Node Stop service
struct NodeStopRequest {
    # Instance ID of the node to stop
    instanceId @0 :Text;
}

struct NodeStopResponse {
    # Whether the stop was successful
    success @0 :Bool;
    # Error message if failed (optional)
    errorMessage @1 :Text;
    # Whether the node had to be force-killed (it did not exit within the
    # cooperative shutdown grace period). False when it exited gracefully.
    forceKilled @2 :Bool;
}

# Node Remove service
struct NodeRemoveRequest {
    # Name of the node to remove
    nodeName @0 :Text;
    # If set, will attempt to stop all the instances associated with this node first before removing the node
    stopInstances @1 :Bool;
    # Tag of the node to remove
    tag @2 :Text;
}

struct NodeRemoveResponse {
    # Whether the removal was successful
    success @0 :Bool;
    # Error message if failed (optional)
    errorMessage @1 :Text;
}

# Node Reset service
struct NodeResetRequest {
}

struct NodeResetResponse {
    # Whether the reset was successful
    success @0 :Bool;
    # Error message if failed (optional)
    errorMessage @1 :Text;
}

# Node Info service
struct NodeInfoRequest {
    # Name of the node to look up in the stack
    nodeName @0 :Text;
    # Tag of the node to look up in the stack
    nodeTag @1 :Text;
}

struct NodeInstanceInfo {
    # Instance identifier
    instanceId @0 :Text;
    # Per-instance state: "starting" or "running"
    state @1 :Text;
    # JSON-encoded `BTreeMap<String, Vec<ProducerRef>>` (link_id → bound
    # producer list) mirroring
    # `RuntimeConfig.node_instance.slot_bindings`. Empty string when the
    # node has no `depends_on` slots. Surfacing this lets the launcher /
    # CLI cross-check newly-staged binding plans against what running
    # consumers have already claimed.
    slotBindingsJson @2 :Text;
    # Liveness from the daemon's most recent `node_health` probe for this
    # instance: true when it last answered within the probe timeout, false
    # otherwise. Defaults to `true` so a message from a producer that predates
    # this field is not read as spuriously unhealthy on version skew.
    healthy @3 :Bool = true;
    # JSON-encoded `BTreeMap<String, SerializedPairingSlot>` mirroring
    # `SerializedInstance.pairing_slots` (the live pairing-slot state per
    # `depends_on.pairings` entry). Empty string when the node declares no
    # pairings.
    pairingSlotsJson @4 :Text;
}

# Node info lookup result.
#
# The response is a union so that "no such node in the stack" is a
# first-class successful outcome rather than a protocol-level error. This
# lets the `peppy node add` preflight check, `peppy node info`, and any
# other caller disambiguate "not found" from a real fault without having
# to sniff the error-string payload of an `InvalidServiceRequest`.
struct NodeInfoResponse {
    union {
        # The node is not in the stack. Carries no payload — the caller
        # already knows which `(name, tag)` it asked about.
        notInStack @0 :Void;
        # The node is in the stack. All of the node metadata is grouped
        # under this arm; callers must match on the union before reading.
        found :group {
            # JSON5-serialized NodeConfig as stored in the node stack
            configJson5 @1 :Text;
            # SHA256 of the entire NodeConfig file
            configSha256 @2 :Text;
            # Lifecycle stage of the in-stack entity ("Added"/"Building"/"Ready"/"Root").
            stage @3 :Text;
            # All tracked instances of this entity, including in-flight `Starting` ones.
            instances @4 :List(NodeInstanceInfo);
            # Path to the most-recent add/build log file for this entity.
            # Empty string when no add log has been produced yet.
            addLogPath @5 :Text;
            # Per-instance run log paths, aligned with `instances` (same order).
            runLogPaths @6 :List(Text);
        }
    }
}
