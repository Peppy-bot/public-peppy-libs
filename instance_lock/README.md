# instance_lock

One running instance per device. A node claims its device at startup and holds
the claim for its whole run:

```rust
let claim = instance_lock::claim("openarm_arm_0", node_runner.processor().bound_instance_id())?;
```

The claim is an exclusive advisory lock on `/tmp/peppy_instance_locks/openarm_arm_0.lock`,
taken in one syscall. Of two starts that race for one device exactly one wins,
and the loser's refusal names the holder and the command that stops it:

```
instance lock /tmp/peppy_instance_locks/openarm_arm_0.lock is held by instance
`left_arm_inst`; stop it with `peppy node stop left_arm_inst`
```

The kernel releases the lock when the holding process exits, by any route
including SIGKILL and the OOM killer, so a crashed instance leaves its device
free for the next start. Dropping the returned `InstanceLock` releases the claim,
which is how a node orders the release after the hardware it drives is safe: it
holds the claim in a shutdown hook that runs after the hook that disables the
motors.

A node that runs in a container reaches the directory the other instances use by
binding it, with a `mount_paths` entry in the node manifest:

```json5
container: {
  def_file: "apptainer.def",
  mount_paths: ["/tmp/peppy_instance_locks"],
},
```

The directory must already exist, and a claim naming one that does not is refused naming that entry.
