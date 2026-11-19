# workflow/v1

A workflow is a JSON5 document that says what robots do. It names roles and the contracts of
their skills, never a robot, a world or an object of a world: a string argument is a
description that perception resolves.

## Top-level fields

`workflow_schema`, `manifest`, `task`, `success` and `actions` are required; the other
fields are optional.

- `workflow_schema`: `"workflow/v1"`, the first key.
- `manifest`: `{ name, revision }`.
- `task`: `{ objective }`, the goal in prose.
- `contracts`: alias to `"name:tag"`, such as `{ postures: "postures:v1" }`. Steps name
  `<alias>.<member>`.
- `roles`: role to `{}` or `{ model }`: the model id a robot of the role reports.
- `locations`: the names of the locations the workflow uses, each a place of the world that
  the skills of a role know through `locations:v1`. A location of a world that a workflow
  uses follows the name rule of Rules, and a teach of `locations:v1` refuses a name outside
  it.
- `tables`: table to `{ <enum value>: <row>, ... }`, one row per value of the enum that keys
  it; a row is a bool, a number, a string or a list of them, or every row of the table is
  `{ $location: "<name>" }`, a table of locations, which goes only to a field with the mark
  `location` and which no expression reads.
- `observations`: name to `{ member: "<alias>.<service>", role, args?, instance? }` (a
  service called on demand) or `{ topic: "<alias>.<topic>", role, max_age_s, instance? }`
  (the latest message; older than `max_age_s` fails the step that reads it). A read of a
  topic before its first message waits for it, up to `max_age_s` past the start of the run,
  and fails as stale after.
- `assumptions`: name to `{ says, check?, kind? }`. `check` is an expression over
  observations and tables; without one, a person confirms it. `kind: "reach"` says the role
  reaches its work zone.
- `success`, `minimum`: expressions over observations and tables. A run completes when every
  lane completed and `success` holds, and ends degraded when `minimum` holds.
- `initialization`: role to one step, run before the lanes.
- `actions`: role to a lane `{ tree, safe_state? }`, or group name to a lane group
  `{ roles: [...], params: { <role>: { <param>: <value> } }, tree, safe_state? }`. A
  parameter value is a literal, `{ $observation: "<name>" }`, `{ $location: "<name>" }` or
  `{ $signal: "<name>" }`.
  `safe_state` is a list of `{ id, timeout_s, call: { member, instance?, args? } }`, run
  when the lane suspends, fails or is aborted.

## Steps

Every step has an `id`, unique within its role, and one kind key. Any step can have
`timeout_s` (its bound, at least 1 ns and at most 86400 s), `retry` (new attempts after a
retryable failure, with no wait between them) and `invariant` (an expression over topic
observations, checked at each poll). These fields go in the step, beside its kind key, and
never inside the object of the kind key.

- `call`: `{ id, timeout_s?, call: { member: "<alias>.<member>", instance?, args?, as?,
  done_when? } }`. `as` names the result; `done_when`, false after a result, is a failed
  attempt, and may read the result's own name. A skill that reports a failure inside a
  completed result fails the step only through `done_when`.
- `sequence`: `{ id, sequence: [<steps>] }`, in order.
- `parallel`: `{ id, parallel: [<steps>], policy: "all" | "any" }`.
- `if`: `{ id, if: <expression>, then: <step>, else?: <step> }`.
- `match`: `{ id, match: <enum expression>, cases: { <value>: <step> }, otherwise?: <step> }`.
  The cases and `otherwise` cover every value of the enum.
- `for_each`: `{ id, for_each: { list: <expression>, as: <name> }, max_items, body: <step> }`.
- `repeat_until`: `{ id, repeat_until: <expression>, fail_when?, max_iterations, breaker?:
  { max_failures }, body: <step> }`.
- `wait_until`: `{ id, wait_until: <expression>, fail_when?, poll_s?, timeout_s }`.
- `signal`: `{ id, signal: "<signal>" }`, or `{ $param: "<param>" }` in a lane group.
- `wait_for`: `{ id, wait_for: ["<signal>", ...], as?, timeout_s }`; the result's `signal`
  field names the signal that came.

Each call step has its own `timeout_s`, or an enclosing step has one; each loop has its
bound; each wait has `timeout_s`.

## Arguments

The `args` of a call give each field of the request of its member that the contract does
not mark `$optional`, also a field that the member does not read in some cases, such as a
target beside an item id. An argument is a literal, an object or a list of arguments, or
a reference:

- `{ $ref: "<name>.<path>" }`: a named result before it on every path, an observation, or
  a loop element. A name that each case of a `match` defines is defined after the `match`
  only when it has an `otherwise` that defines it too. A call reads each observation that
  its arguments name once, as it sends its request. A field that holds an item id takes only
  a `$ref` to an item id that a call or an observation of the same node instance gives.
- `{ $table: "<table>", key: <$ref or $param> }`: the row the key selects.
- `{ $location: "<name>" }`: a location that `locations` declares, admitted only in a field
  with the mark `location`. Such a field holds only a `$location`, directly, through a table
  row or through a parameter, never a literal and never a `$ref`.
- `{ $param: "<param>" }`: the value a lane group gives the role.

## Points, poses and twists

A point, a pose and a twist name their frame and their time, as the geometry types of
contracts-hub give them: a record `{ frame, timestamp, position }`, `{ frame, timestamp,
position, orientation }` or `{ frame, timestamp, linear_velocity, angular_velocity }`, or a
record of many values that gives one `frame` and one `timestamp`. `frame` is a frame as
`transforms` names frames, and `timestamp` a time on the clock of `transforms` as RFC 3339
text. A command may leave its `frame` out, for its robot frame, or for world in a
simulation, and its `timestamp` out, for the latest time. A record whose values hold at
every time, such as the result of a plan, holds no `timestamp`. A field that takes such a
record gets it whole:

- A literal record is in the robot frame of the robot of its role, the root link of its
  URDF, which it names, such as `base_link`, or leaves `frame` out: with no frame, it goes
  with none, which the skill reads as its robot frame, or as world in a simulation. Or it
  names a frame that the tree of the robot holds above its robot frame: `odom`, `map`, or
  `world` where a node publishes it into the tree. A literal record gives each of its fields
  as a literal, such as `{ frame: "map", position: [1.2, 0.4, 0.0] }`, a target in map at
  the latest time with no orientation. A literal record in an empty frame is refused, and so
  is one in another link of the robot, and one that names a frame for a robot whose skills
  give no URDF through `robot_identity:v1`, whose links the director cannot tell. A point, a
  pose or a twist in another link of the robot comes from a report of a skill of the robot by
  `$ref`, which brings its frame and the time it holds at.
- A `$ref` to a point, a pose or a twist of a result or an observation brings its frame and
  its timestamp, and the skill places it where its frame stood at that time: a point that a
  skill gave in the robot frame before a move names the same place after it. A record that
  leaves its timestamp out, as a report does for values that hold at every time, or that
  holds none, brings its frame alone, and the request names the latest time.
- A `$ref` to a point, a pose or a twist with no frame of its own, in a record that gives one
  `frame`, such as an item of `grasp_points` of `limb_states` or of `points` of
  `scan_items`, brings the frame and the timestamp of that record (a twist beside a
  `child_frame` brings that frame), and so does a loop element of such a list of a result.
  One in a record that holds a position or an orientation of its own is posed relative to
  that record, such as a shape of a collision object, posed relative to its object, or a
  vertex of a mesh, relative to its shape, and is refused; so is one in a record that gives
  no frame, and a loop element of a list of an observation, which its loop reads from one
  message: a `$ref` to an item of that list by its index brings the frame and the timestamp
  of the message.
- A point goes to a field that takes a pose whose orientation may be left out, with no
  orientation, which the member reads as none, and a pose goes to a field that takes a point
  with its position alone.
- A point, a pose or a twist of another role goes to a call only in `world`, a frame that the
  trees of both robots hold above their robot frames where a node publishes it: a run fails
  the call with `rejected` in another frame. No node publishes `world` into a robot's tree,
  so a call takes the points, poses and twists of the reports of its own robot. A whole
  record of another role that holds points, poses or twists is refused: give it field by
  field, with a `$ref` of its own to each point, pose or twist.
- The frame, the timestamp and the values of a record that names its frame are literals, or
  the record comes whole by one `$ref`. A request that names its frame at its own level, such
  as the request of `check_positions`, takes its values by `$ref` to one record that gives one
  `frame`, such as `points: { $ref: "scanned.points" }`, or to one point, pose or twist that
  names its own, such as `points: [{ $ref: "seen.item_point" }]`, and takes that frame and
  the timestamp of that record with them; values of another role reach it in `world` alone.
  A `frame` or a `timestamp` given beside such values is refused unless it is the `$ref` of
  theirs, and so is a `timestamp` beside values of a record that holds none, values of two
  records, a literal value beside them, and a list whose values each name a frame of their
  own.

## Expressions

A subset of CEL that gives typed values: literals, names, `.field`, `[index]`, `!`, `&&`,
`||`, `==`, `!=`, `<`, `<=`, `>`, `>=`, `in`, `size()`, `<list>.all(x, ...)` and
`<list>.exists(x, ...)`. An enum value is a string literal, such as `'book'`, and must be a
value of the enum it is compared with. An expression reads observations, tables, results
named before it on every path, loop elements and parameters, never an optional field.

## Rules

- A name (a role, an alias, an observation, a table, a location, a signal, an `as` or a loop
  element) and a step id are 1 to 64 lowercase letters, digits and `_`, and start with a
  letter; a name is no word of CEL. Each `as` and each loop element of a role names one
  thing, apart from every alias, observation, table, location and signal.
- No two aliases, observations, tables, locations or signals share a name.
- Each role has one lane in `actions`, and each lane, initialization and observation names a
  role of `roles`.
- The `args` of an observation hold no `$ref`: a run reads an observation at any instant.
- A `$observation` parameter names an observation of the role it is given to.
- Each call, safe-state call and observation names its member through an alias of
  `contracts`.
- Each role of a lane group gives the same parameters.
- No `as` and no loop element of a lane group is named like a parameter of the group.
- A `$signal` parameter goes only to a `signal` step.
- An expression reads no `$signal` or `$location` parameter, and no parameter whose value
  holds an object.
- No call and no observation names a member of `locations:v1`: the applier of worlds teaches
  the locations of a world, and the director lists them before a run.
- `repeat_until` and its `fail_when` read no result of the body.
- Signals pass between the lanes of `actions`: an initialization emits and awaits none.
- A lane commands only its role. A lane that calls a member with the mark
  `reaches_for_item` checks first that the robot reaches: with a `kind: "reach"` assumption
  whose check reads an observation of its role, or with a reach check named before the
  reaching call on every path. A reach check is a call of a member whose result has a field
  with the mark `checks_reach`, with `as` and a `done_when` that is that field of its
  result, alone or as a term of `&&`, such as `done_when: "near.all_workable"`. Each `$ref`
  of an item id, a point or a pose of the reaching call reads the same result or observation
  as the `$ref` of a point of a reach check before it, so the robot reaches only an item
  whose point a check read. A loop element reads what the list of its loop reads.
- No observation reads a member that moves the robot.
- Each awaited signal has a `signal` step in another lane, and no waits hold each other: a
  wait reached on every path needs a signal that some lane emits before, or beside, the waits
  that hold that lane.

## Variation

A workflow handles variation with a bounded `for_each` over what a scan found, an enum of a
classifying skill, a table with a row per value, and an `unknown` fallback.
