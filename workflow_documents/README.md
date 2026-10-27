# workflow_documents

The documents of the workflow side: workflows, their runs and their bundles,
the expressions of a workflow, and the JSON texts that the workflow director
gives with the events of a run and its end.

```rust
use workflow_documents::{Bundle, RunDocument, RunEvent, Workflow, WorkflowDraft};

let workflow = Workflow::parse(text)?;
let next = WorkflowDraft::parse(draft_text)?.revised(revision)?;
let run = RunDocument::parse(&run_text)?;

let files = Bundle::write(&workflow, world_files, lock)?;
let bundle = Bundle::open(files)?;

let event: RunEvent = serde_json::from_str(&feedback)?;
let result = event.run_result();
```

A `Workflow` (`workflow/v1`) says what robots do: its roles and the contracts
of their skills, its tables, observations and assumptions, the expressions
`success` and `minimum`, a step of initialization per role and one lane of
steps per role, with the `safe_state` that the lane runs when it suspends,
fails or is aborted. Its `locations` name the places of the world that the
skills of its roles know through `locations:v1`. A `{ $location }` argument
gives one of them to a field with the mark `location`. A lane group gives one
tree to several roles, each with its own parameters, and reads as one lane
per role. `Workflow::parse` holds the structural rules that need no contract:
closed step kinds and fields, one namespace, unique step ids and names, a
bound on every call, loop and wait, each `$ref` defined before it on every
path, and each awaited signal emitted by another lane with no waits that hold
each other. Every refusal names its place: a field path, or `<role>.<id>` of
a step. `WorkflowDraft` reads a workflow whose revision the chat gives it,
with `revised`.

An expression is a subset of CEL (`expression`): literals, names, fields,
indexes, `!`, `&&`, `||`, comparisons, `in`, `size()`, `all()` and
`exists()`, parsed with no macro, typed against what it reads, and evaluated
over values. A form outside the subset is refused at its place, counted in
characters.

A `RunDocument` (`run/v1`) binds the roles of one workflow revision to robots,
by name or `"auto"`, says whether a person attends the run, which assumptions
the caller confirms, how incidents are monitored, and whether a difference
from the lock of a bundle refuses the run. `robots_named_twice` refuses each
role that names the robot of another role, for a run and for the check of a
binding. `run_lease_name` writes, and `run_of_lease_name` reads, the name of
the leases that the director takes for a run. A `Bundle`
(`workflow_bundle/v1`) carries a workflow out of the chat with its world
files, the digest of each file and the `Lock` of the versions it ran with: the
peppy version, the sha256 of each contract and of each node, and each node's
commit when it has one.

The results (`result`) are the JSON texts the director gives: the `RunResult`
of a run that ended, the `Suspension` of a suspended run, each
`IncidentRecord`, and the `RunStatus` of a run. Each writes every field but an
empty optional one, and its read refuses any other missing field. A `RunEvent`
is one event of a run, of an `EventKind`, as the feedback of `run`, `resume`
and `abort` and the topic `run_events` give it, and reads the result, the
suspension or the assumptions that wait from its details.

The marks (`marks`) are the marks of fields and members that the director
reads: `robot_frame`, `item_id`, `reaches_for_item` and `location`.
`LANGUAGE` is the language reference that a model that writes workflows
reads.

Each document carries the root key `workflow_schema` and takes the root key
rule, the manifest and the digest from `documents_core`. The crate gives the
JSON5 parse of `documents_core` as `parse`, which reads a text within the
limits of a document.
