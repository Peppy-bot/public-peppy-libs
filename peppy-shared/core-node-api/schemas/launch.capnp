@0xa8f5c2b9e4d3f1a9;

# Launch action message structures for core-node services

struct EnvVar {
    key @0 :Text;
    value @1 :Text;
}

struct LaunchGoal {
    # Environment variables to apply when executing build_cmd and run_cmd (e.g. PATH)
    envVars @0 :List(EnvVar);
    # Idle timeout in seconds for the node add phase (resets on git/http progress or sub-process output)
    nodeAddIdleTimeoutSecs @1 :UInt64;
    # Idle timeout in seconds for the node run-startup phase (resets on subprocess output until the node signals ready)
    nodeRunIdleTimeoutSecs @2 :UInt64;
    # Absolute max timeout in seconds for the entire launch; 0 = unset, no overall deadline (idle timeouts still apply)
    maxTimeoutSecs @3 :UInt64;
    # Idle timeout in seconds for the node build phase (resets on build_cmd output)
    nodeBuildIdleTimeoutSecs @4 :UInt64;
    # Where the launcher file comes from. `fs` carries an absolute path; `repository` carries
    # a launcher name to look up in `~/.peppy/cache/launchers.json5`.
    launcherOrigin :union {
        fs @5 :Text;
        repository @6 :Text;
    }
    # Identity of this federated launch, minted by the coordinator that
    # receives this goal. Every participant records it alongside its slice,
    # so the global stack is reconstructible by query from any machine and a
    # restarted coordinator finds its own launch again.
    #
    # Never empty. A goal without one comes from a peppy that predates
    # federated launch, and is refused at decode rather than silently running
    # every instance on the coordinator: Cap'n Proto defaults an absent field
    # to the empty string, so nothing but this check makes the break loud.
    launchId @7 :Text;
    # The `core_nodes` placeholders declared by the launcher, wired to
    # concrete core nodes by `stack launch --place`. Empty for a launcher
    # that declares no core node links; the coordinator cross-checks this map
    # against the parsed document and refuses a mismatch.
    coreNodeLinks @8 :List(CoreNodeLink);
}

# One `--place <core-node-link>@<core-node>` wiring: the placeholder the
# launcher author declared on the left, the concrete machine on the right.
struct CoreNodeLink {
    # The placeholder as it appears in the launcher's `core_nodes` list.
    linkId @0 :Text;
    # The core node it is wired to. Never the literal `self`: the CLI
    # resolves `self` to the coordinator's own name before dispatch, so a
    # daemon only ever sees concrete names.
    coreNode @1 :Text;
}

struct LaunchGoalResponse {
    # Whether the goal was accepted
    accepted @0 :Bool;
    # Path to the log file for this launch action
    logPath @1 :Text;
    # Rejection reason if not accepted (empty if accepted)
    rejectionReason @2 :Text;
}

enum LaunchFeedbackStep {
    launcherStep @0;
    addingNode @1;
    runningNode @2;
    buildingNode @3;
}

struct LaunchFeedback {
    # The stream type: "stdout" or "stderr"
    stream @0 :Text;
    # The line of output
    line @1 :Text;
    # The step in the launch process this feedback is from
    step @2 :LaunchFeedbackStep;
}

struct NodeAddLog {
    # Node label in "name:tag" format
    nodeLabel @0 :Text;
    # Path to the node add log file
    logPath @1 :Text;
    # Whether the add operation failed
    failed @2 :Bool;
}

struct NodeRunLog {
    # Instance ID
    instanceId @0 :Text;
    # Node label in "name:tag" format
    nodeLabel @1 :Text;
    # Path to the node run log file
    logPath @2 :Text;
    # Whether the run operation failed
    failed @3 :Bool;
}

struct NodeBuildLog {
    # Node label in "name:tag" format
    nodeLabel @0 :Text;
    # Path to the node build log file
    logPath @1 :Text;
    # Whether the build operation failed
    failed @2 :Bool;
}

struct LaunchResult {
    # Whether the launch was successful
    success @0 :Bool;
    # Path to the log file for this launch action
    logPath @1 :Text;
    # Error message if failed (empty if successful)
    errorMessage @2 :Text;
    # Per-node add log entries
    nodeAddLogs @3 :List(NodeAddLog);
    # Per-node run log entries
    nodeRunLogs @4 :List(NodeRunLog);
    # Per-node build log entries
    nodeBuildLogs @5 :List(NodeBuildLog);
}
