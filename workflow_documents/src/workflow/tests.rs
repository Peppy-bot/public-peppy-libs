use std::collections::BTreeSet;

use serde_json::{Value, json};

use super::*;
use crate::names::Name;

/// A workflow that holds every rule: one arm clears a table into a crate.
fn clear_table() -> Value {
    json!({
        "workflow_schema": "workflow/v1",
        "manifest": { "name": "clear_table", "revision": 1 },
        "task": { "objective": "Put each block in the crate" },
        "contracts": {
            "postures": "postures:v1",
            "perception": "item_perception:v1",
            "manipulation": "item_manipulation:v1",
            "space": "workspace:v1",
            "limbs": "limb_state:v1",
        },
        "roles": { "clearer": {} },
        "tables": { "speed": { "slow": 3, "fast": 1 } },
        "observations": {
            "crate": { "member": "perception.identify_item", "role": "clearer", "args": { "description": "crate", "timeout_s": 0 } },
            "pose": { "topic": "limbs.joint_states", "role": "clearer", "max_age_s": 0.5 },
        },
        "assumptions": {
            "crate_reachable": { "kind": "reach", "says": "The crate is in reach", "check": "crate.item_id != ''" },
            "table_clear": { "says": "Nothing but blocks is on the table" },
        },
        "success": "crate.item_id != ''",
        "initialization": {
            "clearer": { "id": "ready", "timeout_s": 30, "call": { "member": "postures.move_to_ready", "args": { "duration_s": 3 } } },
        },
        "actions": {
            "clearer": {
                "safe_state": [{ "id": "home", "timeout_s": 10, "call": { "member": "postures.move_to_home", "args": { "duration_s": 3 } } }],
                "tree": { "id": "clear", "timeout_s": 900, "repeat_until": "crate.item_id == ''", "max_iterations": 3, "body": {
                    "id": "pass", "sequence": [
                        { "id": "scan", "call": { "member": "perception.scan_items", "args": { "timeout_s": 0 }, "as": "found" } },
                        { "id": "each", "for_each": { "list": "found.item_ids", "as": "item" }, "max_items": 20, "body": {
                            "id": "grab", "retry": 1, "call": {
                                "member": "manipulation.grab_item",
                                "args": { "item_id": { "$ref": "item" }, "position": { "$ref": "crate.position" }, "speed": { "$table": "speed", "key": { "$ref": "found.mode" } } },
                                "as": "grab", "done_when": "grab.success",
                            },
                        } },
                    ],
                } },
            },
        },
    })
}

fn parse(value: &Value) -> Result<Workflow, Refusals> {
    Workflow::parse(&serde_json::to_string(value).unwrap())
}

fn reasons(value: &Value) -> Vec<String> {
    parse(value)
        .expect_err("refused")
        .iter()
        .map(Refusal::to_string)
        .collect()
}

fn refuses(value: &Value, part: &str) {
    let found = reasons(value);
    assert!(
        found.iter().any(|reason| reason.contains(part)),
        "no refusal holds {part:?}: {found:#?}"
    );
}

fn name(text: &str) -> Name {
    Name::parse(text).unwrap()
}

/// The workflow with the field at `pointer` (a JSON pointer) set to `value`.
fn with(pointer: &str, value: Value) -> Value {
    let mut workflow = clear_table();
    let (parent, key) = pointer.rsplit_once('/').expect("a pointer has a parent");
    match workflow
        .pointer_mut(parent)
        .expect("the pointer names a field")
    {
        Value::Object(fields) => {
            fields.insert(key.to_owned(), value);
        }
        Value::Array(items) => items[key.parse::<usize>().expect("an index")] = value,
        other => panic!("{parent} is {other}"),
    }
    workflow
}

#[test]
fn a_workflow_that_holds_every_rule_parses_into_its_lanes() {
    let workflow = parse(&clear_table()).unwrap();
    let lane = &workflow.lanes[&name("clearer")];
    assert_eq!(lane.origin, LaneOrigin::Role);
    assert_eq!(lane.safe_state[0].id.as_str(), "home");
    assert!(workflow.assumptions[&name("crate_reachable")].reach);
    assert!(workflow.assumptions[&name("table_clear")].check.is_none());
    let StepKind::RepeatUntil { max_iterations, .. } = &lane.tree.kind else {
        panic!("{:?}", lane.tree.kind)
    };
    assert_eq!(max_iterations.get(), 3);
    assert_eq!(workflow.digest(), workflow.document().digest());
}

#[test]
fn a_step_has_one_kind_and_only_its_fields() {
    refuses(
        &with("/actions/clearer/tree/body/sequence/0/sequence", json!([])),
        "one kind key",
    );
    refuses(
        &with("/actions/clearer/tree/body/sequence/0/colour", json!("red")),
        "unknown field `colour`",
    );
    refuses(
        &with(
            "/actions/clearer/tree/body/sequence/0/call/as",
            json!("Found"),
        ),
        "clearer.scan",
    );
}

#[test]
fn each_call_has_a_bound_and_each_wait_its_own() {
    let mut workflow = clear_table();
    workflow["actions"]["clearer"]["tree"]
        .as_object_mut()
        .unwrap()
        .remove("timeout_s");
    refuses(&workflow, "clearer.scan: a call has its own `timeout_s`");
    let wait = json!({ "id": "wait", "wait_until": "crate.item_id != ''" });
    refuses(
        &with("/actions/clearer/tree/body/sequence/0", wait),
        "clearer.wait: a wait has its own `timeout_s`",
    );
    let safe = json!([{ "id": "home", "call": { "member": "postures.move_to_home" } }]);
    refuses(
        &with("/actions/clearer/safe_state", safe),
        "a call of a `safe_state` has its own `timeout_s`",
    );
    let instant =
        json!([{ "id": "home", "timeout_s": 0, "call": { "member": "postures.move_to_home" } }]);
    refuses(
        &with("/actions/clearer/safe_state", instant),
        "0 s is no bound",
    );
    let no_call = json!([{ "id": "home", "timeout_s": 5 }]);
    refuses(
        &with("/actions/clearer/safe_state", no_call),
        "has no field `call`",
    );
}

#[test]
fn a_ref_reads_only_names_defined_before_it_on_every_path() {
    let early = json!({ "item_id": { "$ref": "grab.item_id" } });
    refuses(
        &with(
            "/actions/clearer/tree/body/sequence/1/body/call/args",
            early,
        ),
        "`$ref` reads `grab`",
    );
    let branch = json!([
        { "id": "maybe", "if": "crate.item_id != ''", "then": { "id": "scan", "call": { "member": "perception.scan_items", "as": "found" } } },
        { "id": "each", "for_each": { "list": "found.item_ids", "as": "item" }, "max_items": 2, "body": { "id": "home2", "call": { "member": "postures.move_to_home" } } },
    ]);
    refuses(
        &with("/actions/clearer/tree/body/sequence", branch),
        "`for_each.list` reads `found`",
    );
}

#[test]
fn ids_and_local_names_are_unique_per_role_and_apart_from_global_names() {
    refuses(
        &with("/actions/clearer/tree/body/sequence/0/id", json!("ready")),
        "two steps with the id `ready`",
    );
    refuses(
        &with(
            "/actions/clearer/tree/body/sequence/0/call/as",
            json!("crate"),
        ),
        "`crate` names a local value",
    );
    let mut located = with("/locations", json!(["bin"]));
    located["actions"]["clearer"]["tree"]["body"]["sequence"][0]["call"]["as"] = json!("bin");
    refuses(
        &located,
        "`bin` names a local value and an alias, observation, table, location or signal",
    );
    let mut workflow = clear_table();
    workflow["tables"]["crate"] = json!({ "a": 1 });
    refuses(&workflow, "`crate` names an observation and a table");
    let mut literal_key = clear_table();
    literal_key["actions"]["clearer"]["tree"]["body"]["sequence"][1]["body"]["call"]["args"]["speed"]
        ["key"] = json!("slow");
    refuses(
        &literal_key,
        "the key of the table `speed` is a `$ref` or a `$param`",
    );
    let mut reference = clear_table();
    reference["tables"]["speed"]["$ref"] = json!(2);
    refuses(
        &reference,
        "tables.speed.$ref: names a row with a value of the enum",
    );
}

#[test]
fn every_role_has_one_lane_and_every_alias_its_contract() {
    let mut workflow = clear_table();
    workflow["roles"]["helper"] = json!({});
    refuses(&workflow, "the role `helper` has no lane");
    refuses(
        &with("/initialization/clearer/call/member", json!("arm.move")),
        "`arm.move` names the alias `arm`",
    );
    let mut stray = clear_table();
    stray["actions"]["visitor"] = stray["actions"]["clearer"].clone();
    refuses(&stray, "`visitor` is no role of `roles`");
}

#[test]
fn whole_run_expressions_read_only_observations_and_tables() {
    refuses(
        &with("/success", json!("found.item_ids == []")),
        "`success` reads `found`",
    );
    refuses(
        &with(
            "/actions/clearer/tree/invariant",
            json!("crate.item_id != ''"),
        ),
        "it reads only topic observations",
    );
    refuses(&with("/success", json!("crate.item_id + 1")), "`+`");
}

#[test]
fn the_arguments_of_an_observation_are_literals() {
    refuses(
        &with(
            "/observations/crate/args/description",
            json!({ "$ref": "pose.positions" }),
        ),
        "observations.crate.args: `$ref` reads `pose`; an observation is read at any instant of a run, so its arguments are literals",
    );
}

fn two_arms() -> Value {
    json!({
        "workflow_schema": "workflow/v1",
        "manifest": { "name": "pack", "revision": 1 },
        "task": { "objective": "Fill two boxes" },
        "contracts": { "postures": "postures:v1", "packing": "packing:v1" },
        "roles": { "arm_a": {}, "arm_b": {}, "courier": {} },
        "observations": {
            "state_a": { "member": "packing.station_state", "role": "arm_a" },
            "state_b": { "member": "packing.station_state", "role": "arm_b" },
        },
        "success": "state_a.full && state_b.full",
        "actions": {
            "pickers": {
                "roles": ["arm_a", "arm_b"],
                "params": {
                    "arm_a": { "state": { "$observation": "state_a" }, "full": { "$signal": "full_a" }, "count": 6 },
                    "arm_b": { "state": { "$observation": "state_b" }, "full": { "$signal": "full_b" }, "count": 4 },
                },
                "tree": { "id": "fill", "timeout_s": 600, "sequence": [
                    { "id": "loop", "repeat_until": "state.tapes >= count", "max_iterations": 12, "body": {
                        "id": "pick", "call": { "member": "packing.pick", "args": { "station": { "$ref": "state.station" } } },
                    } },
                    { "id": "done", "signal": { "$param": "full" } },
                ] },
            },
            "courier": { "tree": { "id": "serve", "timeout_s": 600, "sequence": [
                { "id": "next", "wait_for": ["full_a", "full_b"], "as": "which", "timeout_s": 300 },
                { "id": "close", "call": { "member": "packing.close", "args": { "station": { "$ref": "which.signal" } } } },
            ] } },
        },
    })
}

#[test]
fn a_lane_group_gives_each_role_its_own_lane_with_its_parameters() {
    let workflow = parse(&two_arms()).unwrap();
    let a = &workflow.lanes[&name("arm_a")];
    let b = &workflow.lanes[&name("arm_b")];
    assert_eq!(a.origin, LaneOrigin::Group(name("pickers")));
    let StepKind::Sequence(steps) = &b.tree.kind else {
        panic!()
    };
    let StepKind::RepeatUntil { until, body, .. } = &steps[0].kind else {
        panic!()
    };
    assert_eq!(until.source(), "(state_b.tapes >= 4)");
    let StepKind::Call(call) = &body.kind else {
        panic!()
    };
    assert_eq!(
        call.args["station"],
        Arg::Ref(crate::expression::RefPath::parse("state_b.station").unwrap())
    );
    assert_eq!(steps[1].kind, StepKind::Signal(name("full_b")));
    let StepKind::Sequence(steps) = &a.tree.kind else {
        panic!()
    };
    assert_eq!(steps[1].kind, StepKind::Signal(name("full_a")));
}

#[test]
fn a_parameter_exists_for_every_role_and_a_signal_goes_only_to_signal_steps() {
    let mut workflow = two_arms();
    workflow["actions"]["pickers"]["params"]["arm_b"]
        .as_object_mut()
        .unwrap()
        .remove("count");
    refuses(&workflow, "gives other parameters");
    let mut signal_in_args = two_arms();
    signal_in_args["actions"]["pickers"]["tree"]["sequence"][0]["body"]["call"]["args"]["station"] =
        json!({ "$param": "full" });
    refuses(&signal_in_args, "holds a signal");
    let mut outside = clear_table();
    outside["actions"]["clearer"]["tree"]["body"]["sequence"][0]["call"]["args"]["x"] =
        json!({ "$param": "p" });
    refuses(&outside, "only in the tree of a lane group");
    let mut hidden = two_arms();
    hidden["actions"]["pickers"]["tree"]["sequence"][0]["body"]["call"]["as"] = json!("count");
    refuses(&hidden, "`count` is a parameter of the lane group");
}

#[test]
fn an_observation_parameter_names_an_observation_of_its_own_role() {
    let mut other_role = two_arms();
    other_role["actions"]["pickers"]["params"]["arm_a"]["state"] =
        json!({ "$observation": "state_b" });
    refuses(
        &other_role,
        "actions.pickers.params.arm_a.state: `state_b` is an observation of the role `arm_b`",
    );
    let mut unknown = two_arms();
    unknown["actions"]["pickers"]["params"]["arm_b"]["state"] =
        json!({ "$observation": "state_c" });
    refuses(
        &unknown,
        "actions.pickers.params.arm_b.state: `state_c` is no observation of the workflow",
    );
}

/// A parameter that does not parse is refused at its own place, whatever its kind.
#[test]
fn a_malformed_parameter_is_refused_at_its_role_and_name() {
    for kind in ["$observation", "$location", "$signal"] {
        let mut workflow = two_arms();
        workflow["actions"]["pickers"]["params"]["arm_a"]["state"] = json!({ kind: "Not a name" });
        refuses(&workflow, "actions.pickers.params.arm_a.state");
    }
}

#[test]
fn a_signal_has_an_emitter_in_another_lane_and_no_lanes_block_each_other() {
    let mut workflow = two_arms();
    workflow["actions"]["courier"]["tree"]["sequence"][0]["wait_for"] = json!(["full_c"]);
    refuses(&workflow, "no other lane has a `signal` step for `full_c`");
    let deadlock = json!({
        "workflow_schema": "workflow/v1",
        "manifest": { "name": "deadlock", "revision": 1 },
        "task": { "objective": "Two lanes that wait for each other" },
        "contracts": { "postures": "postures:v1" },
        "roles": { "a": {}, "b": {} },
        "success": "true",
        "actions": {
            "a": { "tree": { "id": "a1", "sequence": [
                { "id": "wait", "wait_for": ["from_b"], "timeout_s": 5 },
                { "id": "tell", "signal": "from_a" },
            ] } },
            "b": { "tree": { "id": "b1", "sequence": [
                { "id": "wait", "wait_for": ["from_a"], "timeout_s": 5 },
                { "id": "tell", "signal": "from_b" },
            ] } },
        },
    });
    refuses(&deadlock, "the waits `a.wait`, `b.wait` each wait");
    refuses(&deadlock, "none of them can end");
}

/// The lanes `a` and `b` with the sequences `a` and `b`, and a lane `c`
/// that waits for the signal `other`.
fn two_lanes(a: Value, b: Value) -> Value {
    json!({
        "workflow_schema": "workflow/v1",
        "manifest": { "name": "relay", "revision": 1 },
        "task": { "objective": "Two lanes that pass signals" },
        "contracts": { "postures": "postures:v1" },
        "roles": { "a": {}, "b": {}, "c": {} },
        "success": "true",
        "actions": {
            "a": { "tree": { "id": "a1", "sequence": a } },
            "b": { "tree": { "id": "b1", "sequence": b } },
            "c": { "tree": { "id": "c1", "sequence": [
                { "id": "hear", "wait_for": ["other"], "timeout_s": 5 },
                { "id": "back", "signal": "from_c" },
            ] } },
        },
    })
}

fn wait(id: &str, signal: &str) -> Value {
    json!({ "id": id, "wait_for": [signal], "timeout_s": 5 })
}

fn emit(id: &str, signal: &str) -> Value {
    json!({ "id": id, "signal": signal })
}

/// The refusals of `value`, none when it parses.
fn refusal_texts(value: &Value) -> Vec<String> {
    parse(value)
        .err()
        .map(|refusals| refusals.iter().map(Refusal::to_string).collect())
        .unwrap_or_default()
}

fn blocks(value: &Value) -> bool {
    refusal_texts(value)
        .iter()
        .any(|reason| reason.contains("none of them can end"))
}

#[test]
fn a_lane_that_emits_before_it_waits_blocks_no_lane() {
    let relay = two_lanes(
        json!([
            emit("tell", "from_a"),
            wait("hear", "from_b"),
            emit("other", "other")
        ]),
        json!([emit("tell", "from_b"), wait("hear", "from_a")]),
    );
    assert!(!blocks(&relay), "{:#?}", refusal_texts(&relay));
}

#[test]
fn a_signal_that_no_waiting_lane_needs_unblocks_no_lane() {
    // `a` emits `other` first, which only `c` waits for; `a` and `b` still
    // each wait for the other.
    let held = two_lanes(
        json!([
            emit("first", "other"),
            wait("hear", "from_b"),
            emit("tell", "from_a")
        ]),
        json!([wait("hear", "from_a"), emit("tell", "from_b")]),
    );
    assert!(blocks(&held), "{:#?}", refusal_texts(&held));
}

#[test]
fn a_signal_that_one_branch_of_an_if_emits_can_end_a_wait() {
    let maybe = two_lanes(
        json!([
            { "id": "maybe", "if": "true", "then": emit("tell", "from_a") },
            wait("hear", "from_b"),
            emit("other", "other"),
        ]),
        json!([wait("hear", "from_a"), emit("tell", "from_b")]),
    );
    assert!(!blocks(&maybe), "{:#?}", refusal_texts(&maybe));
}

#[test]
fn a_signal_beside_a_wait_in_a_parallel_can_end_the_wait_of_another_lane() {
    let beside = two_lanes(
        json!([
            { "id": "both", "policy": "all", "parallel": [wait("hear", "from_b"), emit("tell", "from_a")] },
            emit("other", "other"),
        ]),
        json!([wait("hear", "from_a"), emit("tell", "from_b")]),
    );
    assert!(!blocks(&beside), "{:#?}", refusal_texts(&beside));
}

#[test]
fn a_draft_takes_its_revision_from_the_chat() {
    let mut draft = clear_table();
    draft["manifest"] = json!({ "name": "clear_table" });
    let draft = WorkflowDraft::parse(&serde_json::to_string(&draft).unwrap()).unwrap();
    assert_eq!(draft.name(), "clear_table");
    let workflow = draft.revised(Revision::new(4).unwrap()).unwrap();
    assert_eq!(workflow.manifest.revision.get(), 4);
    let revised = serde_json::to_string(&clear_table()).unwrap();
    assert!(
        WorkflowDraft::parse(&revised)
            .unwrap_err()
            .to_string()
            .contains("given by the chat")
    );
}

#[test]
fn another_kind_is_refused_before_the_fields() {
    let mut workflow = clear_table();
    workflow["workflow_schema"] = json!("run/v1");
    refuses(&workflow, "this parser reads \"workflow/v1\"");
}

#[test]
fn a_signal_passes_between_the_lanes_of_the_actions_alone() {
    let mut early = clear_table();
    early["initialization"]["clearer"] = json!({ "id": "ready", "signal": "ready" });
    refuses(
        &early,
        "signals pass between the lanes of `actions`, not in an initialization",
    );
}

#[test]
fn a_wait_for_names_at_least_one_signal() {
    let mut empty = clear_table();
    empty["actions"]["clearer"]["tree"]["body"]["sequence"][0] =
        json!({ "id": "hear", "wait_for": [], "timeout_s": 5 });
    refuses(&empty, "`wait_for` names at least one signal");
}

#[test]
fn each_role_has_one_lane_keyed_by_a_role_of_the_workflow() {
    let mut twice = two_arms();
    twice["actions"]["arm_a"] = json!({ "tree": { "id": "rest", "signal": "full_a" } });
    refuses(
        &twice,
        "the role `arm_a` has two lanes; give each role one lane",
    );
    let mut spare = clear_table();
    spare["roles"]["spare"] = json!({});
    refuses(
        &spare,
        "the role `spare` has no lane; give it one, or remove the role",
    );
    let mut ghost = clear_table();
    ghost["actions"]["ghost"] = json!({ "tree": { "id": "rest", "signal": "nobody" } });
    refuses(
        &ghost,
        "`ghost` is no role of `roles`; a lane is keyed by its role",
    );
}

#[test]
fn a_lane_group_gives_parameters_to_its_own_roles_alone() {
    let mut stranger = two_arms();
    stranger["actions"]["pickers"]["params"]["courier"] = json!({ "state": { "$observation": "state_a" }, "full": { "$signal": "full_a" }, "count": 1 });
    refuses(
        &stranger,
        "gives parameters to `courier`, which is no role of the group",
    );
}

#[test]
fn a_table_argument_names_a_table_of_the_workflow() {
    let mut missing = clear_table();
    missing["actions"]["clearer"]["tree"]["body"]["sequence"][1]["body"]["call"]["args"]["speed"]
        ["$table"] = json!("pace");
    refuses(
        &missing,
        "`$table` names `pace`, which `tables` does not give",
    );
}

#[test]
fn an_observation_reads_for_a_role_of_the_workflow() {
    let mut ghost = clear_table();
    ghost["observations"]["crate"]["role"] = json!("ghost");
    refuses(&ghost, "`ghost` is no role of `roles`");
}

/// A `parallel` with the policy `all` defines after it what each of its
/// children defines, and one with `any` what was defined before it; a
/// `match` defines after it only what each case and its `otherwise` define.
#[test]
fn a_name_is_defined_after_a_parallel_or_a_match_only_where_every_path_defines_it() {
    let scan = json!({ "id": "scan", "call": { "member": "perception.scan_items", "args": { "timeout_s": 0 }, "as": "found" } });
    let rest = json!({ "id": "rest", "call": { "member": "postures.move_to_home" } });
    let each = json!({ "id": "each", "for_each": { "list": "found.item_ids", "as": "item" }, "max_items": 2,
        "body": { "id": "home2", "call": { "member": "postures.move_to_home" } } });
    let reads_found = |value: &Value| {
        refusal_texts(value)
            .iter()
            .any(|reason| reason.contains("`for_each.list` reads `found`"))
    };
    let parallel = |policy: &str| {
        with(
            "/actions/clearer/tree/body/sequence",
            json!([{ "id": "both", "policy": policy, "parallel": [scan.clone(), rest.clone()] }, each.clone()]),
        )
    };
    assert!(!reads_found(&parallel("all")));
    assert!(reads_found(&parallel("any")));
    let matched = with(
        "/actions/clearer/tree/body/sequence",
        json!([
            { "id": "look", "call": { "member": "perception.scan_items", "args": { "timeout_s": 0 }, "as": "seen" } },
            { "id": "which", "match": "seen.mode", "cases": { "slow": scan.clone() }, "otherwise": rest.clone() },
            each.clone(),
        ]),
    );
    assert!(reads_found(&matched));
}

#[test]
fn a_local_name_names_one_value_in_the_trees_of_its_role() {
    refuses(
        &with(
            "/actions/clearer/tree/body/sequence/1/body/call/as",
            json!("found"),
        ),
        "`found` names two values in the trees of the role `clearer`",
    );
}

#[test]
fn a_param_names_a_parameter_that_each_role_of_the_group_gives() {
    let mut unknown = two_arms();
    unknown["actions"]["pickers"]["tree"]["sequence"][0]["body"]["call"]["args"]["speed"] =
        json!({ "$param": "speed" });
    refuses(&unknown, "the role `arm_a` gives no parameter `speed`");
}

#[test]
fn an_initialization_is_keyed_by_a_role_of_the_workflow() {
    let mut ghost = clear_table();
    ghost["initialization"]["ghost"] = ghost["initialization"]["clearer"].clone();
    refuses(
        &ghost,
        "initialization.ghost: `ghost` is no role of `roles`",
    );
}

#[test]
fn a_call_of_a_safe_state_takes_its_member_its_instance_and_its_args_alone() {
    let named = json!([{ "id": "home", "timeout_s": 10, "call": { "member": "postures.move_to_home", "as": "homed" } }]);
    refuses(
        &with("/actions/clearer/safe_state", named),
        "a call of a `safe_state` takes `member`, `instance` and `args` only",
    );
    let retried = json!([{ "id": "home", "timeout_s": 10, "retry": 1, "call": { "member": "postures.move_to_home" } }]);
    refuses(
        &with("/actions/clearer/safe_state", retried),
        "has an unknown field `retry`",
    );
}

/// A lane that emits a signal itself waits in vain: the signal comes from
/// another lane.
#[test]
fn a_signal_that_only_the_waiting_lane_emits_is_refused() {
    let alone = two_lanes(
        json!([emit("e", "own"), wait("w", "own")]),
        json!([emit("tell", "other")]),
    );
    assert!(
        refusal_texts(&alone)
            .iter()
            .any(|reason| reason.contains("no other lane has a `signal` step for `own`")),
        "{:?}",
        refusal_texts(&alone)
    );
}

/// A call gives a location by `{ $location }` of a declared name. An undeclared name, a location named twice and a
/// location that shares a name with a table are refused.
#[test]
fn a_location_is_declared_once_and_goes_to_an_argument_by_its_name() {
    let mut workflow = with("/locations", json!(["bin"]));
    workflow["actions"]["clearer"]["tree"]["body"]["sequence"][1]["body"]["call"]["args"]["target"] =
        json!({ "$location": "bin" });
    let parsed = parse(&workflow).unwrap();
    assert_eq!(parsed.locations, BTreeSet::from([name("bin")]));
    let mut undeclared = workflow.clone();
    undeclared["locations"] = json!(["shelf"]);
    refuses(
        &undeclared,
        "`$location` names `bin`, which `locations` does not declare",
    );
    refuses(
        &with("/locations", json!(["bin", "bin"])),
        "names `bin` twice",
    );
    refuses(&with("/locations", json!(["speed"])), "share one namespace");
}

/// No call and no observation names a member of locations:v1: the applier of worlds teaches locations and the
/// director lists them.
#[test]
fn no_call_or_observation_names_a_member_of_locations_v1() {
    let mut workflow = with("/contracts/places", json!("locations:v1"));
    workflow["initialization"]["clearer"]["call"]["member"] = json!("places.list");
    refuses(&workflow, "calls `places.list` of locations:v1");
    let mut observed = with("/contracts/places", json!("locations:v1"));
    observed["observations"]["known"] = json!({ "member": "places.list", "role": "clearer" });
    refuses(&observed, "calls `places.list` of locations:v1");
}

/// A table row and a lane-group parameter hold a `{ $location }` of a declared location.
#[test]
fn a_table_row_and_a_lane_group_parameter_hold_a_location() {
    let mut workflow = with("/locations", json!(["bin", "shelf"]));
    workflow["tables"]["place_for"] =
        json!({ "book": { "$location": "shelf" }, "toy": { "$location": "bin" } });
    assert!(parse(&workflow).is_ok(), "{:?}", reasons(&workflow));
    workflow["tables"]["place_for"]["toy"] = json!({ "$location": "box" });
    refuses(&workflow, "tables.place_for.toy: `$location` names `box`");
    let mut group = two_arms();
    group["locations"] = json!(["station_a", "station_b"]);
    group["actions"]["pickers"]["params"]["arm_a"]["place"] = json!({ "$location": "station_a" });
    group["actions"]["pickers"]["params"]["arm_b"]["place"] = json!({ "$location": "station_b" });
    group["actions"]["pickers"]["tree"]["sequence"][0]["body"]["call"]["args"]["station"] =
        json!({ "$param": "place" });
    assert!(parse(&group).is_ok(), "{:?}", reasons(&group));
    group["locations"] = json!(["station_a"]);
    refuses(&group, "arm_b.pick: `$location` names `station_b`");
}
