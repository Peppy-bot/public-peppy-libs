# workflow/v1

A workflow is a JSON5 document that says what robots do. It names roles and the contracts of
their skills, never a robot, a world or an object of a world: a string argument is a
description that perception resolves.

## Top-level fields

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
  it; a row is a literal or `{ $location: "<name>" }`.
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
observations, checked at each poll).

- `call`: `{ id, call: { member: "<alias>.<member>", instance?, args?, as?, done_when? } }`.
  `as` names the result; `done_when`, false after a result, is a failed attempt, and may
  read the result's own name. A skill that reports a failure inside a completed result fails
  the step only through `done_when`.
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

Each call has its own `timeout_s` or one of an enclosing step; each loop has its bound; each
wait has `timeout_s`.

## Arguments

An argument is a literal, an object or a list of arguments, or a reference:

- `{ $ref: "<name>.<path>" }`: a named result before it on every path, an observation, or
  a loop element. A name that each case of a `match` defines is defined after the `match`
  only when it has an `otherwise` that defines it too. A field that holds a position, a pose
  or an item id takes only a `$ref` to a position, a pose or an item id that a call or an
  observation of the same role gives; an item id comes from a call or an observation of the
  same node instance.
- `{ $table: "<table>", key: <$ref or $param> }`: the row the key selects.
- `{ $location: "<name>" }`: a location that `locations` declares, admitted only in a field
  with the mark `location`. Such a field holds only a `$location`, directly, through a table
  row or through a parameter, never a literal and never a `$ref`.
- `{ $param: "<param>" }`: the value a lane group gives the role.

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
- A lane commands only its role. A lane that calls a member that reaches for an item has a
  `kind: "reach"` assumption whose check reads an observation of its role.
- No observation reads a member that moves the robot.
- Each awaited signal has a `signal` step in another lane, and no waits hold each other: a
  wait reached on every path needs a signal that some lane emits before, or beside, the waits
  that hold that lane.

## Variation

A workflow handles variation with a bounded `for_each` over what a scan found, an enum of a
classifying skill, a table with a row per value, and an `unknown` fallback.
