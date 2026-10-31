//! Properties on drawn expressions, run documents and bundles: an expression
//! survives its text and evaluates as cel's interpreter does, a run document
//! survives its text, a bundle opens as it was written while any corrupted
//! file is refused, and the digest of a bundle stays with its trials and
//! changes with any other field.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use hegel::TestCase;
use hegel::generators::{self as gs, Generator, PrintableGenerator};
use serde_json::json;
use workflow_documents::bundle::{FILES_DIR, TRIALS_DIR, WORKFLOW_PATH};
use workflow_documents::expression::{CompareOp, Expr, Literal, Value};
use workflow_documents::run::{Attendance, Incidents, RoleBinding, VersionPolicy, WorkflowRef};
use workflow_documents::work_cell::{
    ParameterKind, RoleValues, TestedRange, WatchParameter, WorkCellParameter,
};
use workflow_documents::{
    Bundle, BundleFile, Expression, Lock, LockedNode, Name, NameTag, Rate, Revision, RobotName,
    RunDocument, Sha256, Thresholds, Validation, WorkCell, Workflow,
};

fn int_literal(tc: &TestCase) -> Expr {
    Expr::Literal(Literal::Int(
        tc.draw(gs::integers::<i64>().min_value(-50).max_value(50)),
    ))
}

/// A drawn expression of ints and bools that gives a bool.
fn bool_expr(tc: &TestCase, depth: u8) -> Expr {
    let leaf = depth == 0 || tc.draw(gs::integers::<u8>().min_value(0).max_value(3)) == 0;
    if leaf {
        return match tc.draw(gs::integers::<u8>().min_value(0).max_value(1)) {
            0 => Expr::Literal(Literal::Bool(tc.draw(gs::booleans()))),
            _ => {
                let op = tc.draw(gs::sampled_from(&[
                    CompareOp::Eq,
                    CompareOp::Ne,
                    CompareOp::Lt,
                    CompareOp::Le,
                    CompareOp::Gt,
                    CompareOp::Ge,
                ]));
                Expr::Compare(op, Box::new(int_literal(tc)), Box::new(int_literal(tc)))
            }
        };
    }
    let a = Box::new(bool_expr(tc, depth - 1));
    match tc.draw(gs::integers::<u8>().min_value(0).max_value(3)) {
        0 => Expr::Not(a),
        1 => Expr::And(a, Box::new(bool_expr(tc, depth - 1))),
        2 => Expr::Or(a, Box::new(bool_expr(tc, depth - 1))),
        _ => {
            let items = (0..tc.draw(gs::integers::<usize>().min_value(1).max_value(3)))
                .map(|_| int_literal(tc))
                .collect();
            Expr::In(Box::new(int_literal(tc)), Box::new(Expr::List(items)))
        }
    }
}

#[hegel::test]
fn an_expression_survives_its_text(tc: TestCase) {
    let expr = bool_expr(&tc, 4);
    let text = expr.to_string();
    let parsed = Expression::parse(&text).unwrap_or_else(|error| panic!("{error}\n{text}"));
    assert_eq!(parsed.expr(), &expr, "{text}");
}

#[hegel::test]
fn an_expression_evaluates_as_cel_does(tc: TestCase) {
    let expr = bool_expr(&tc, 4);
    let text = expr.to_string();
    let ours = expr.evaluate(&BTreeMap::<Name, Value>::new()).unwrap();
    let env = cel::Env::stdlib();
    let program = env.compile(&text).unwrap();
    let theirs = program.execute(&cel::Context::with_env(Arc::new(env)));
    let theirs = match theirs {
        Ok(cel::Value::Bool(b)) => Value::Bool(b),
        other => panic!("{text}: cel gives {other:?}"),
    };
    assert_eq!(ours, theirs, "{text}");
}

/// Robot names over an alphabet that cannot spell the reserved `self`.
fn robot_names() -> impl PrintableGenerator<RobotName> {
    gs::text()
        .alphabet("ab_-1")
        .min_size(1)
        .max_size(4)
        .map(|text| RobotName::parse(&text).expect("the alphabet is a robot name's"))
}

/// Measured values of a work cell: finite, within a few metres.
fn measured() -> impl PrintableGenerator<f64> {
    gs::floats::<f64>().min_value(-5.0).max_value(5.0)
}

/// A work cell block of up to two parameters, each with one value or a value
/// per role of `roles`.
fn work_cell(tc: &TestCase, roles: &[Name]) -> WorkCell {
    (0..tc.draw(gs::integers::<usize>().min_value(0).max_value(2)))
        .map(|index| {
            let values = match tc.draw(gs::booleans()) {
                true => RoleValues::Single(tc.draw(measured())),
                false => RoleValues::PerRole(
                    roles
                        .iter()
                        .map(|role| (role.clone(), tc.draw(measured())))
                        .collect(),
                ),
            };
            (Name::parse(&format!("parameter_{index}")).unwrap(), values)
        })
        .collect()
}

#[hegel::test]
fn a_run_document_survives_its_text(tc: TestCase) {
    let roles: BTreeMap<Name, RoleBinding> = (0..tc
        .draw(gs::integers::<usize>().min_value(1).max_value(3)))
        .map(|index| {
            let binding = match tc.draw(gs::booleans()) {
                true => RoleBinding::Auto,
                false => RoleBinding::Robot(tc.draw(robot_names())),
            };
            (Name::parse(&format!("role_{index}")).unwrap(), binding)
        })
        .collect();
    let role_names: Vec<Name> = roles.keys().cloned().collect();
    let document = RunDocument {
        workflow: WorkflowRef {
            name: "clear_table".into(),
            revision: Revision::new(tc.draw(gs::integers::<u32>().min_value(1).max_value(9)))
                .unwrap(),
        },
        roles,
        attendance: tc.draw(gs::sampled_from(&[
            Attendance::Attended,
            Attendance::Unattended,
        ])),
        confirmed: BTreeSet::new(),
        incidents: Incidents::None,
        versions: tc.draw(gs::sampled_from(&[
            VersionPolicy::Refuse,
            VersionPolicy::Warn,
        ])),
        work_cell: work_cell(&tc, &role_names),
        watch: match tc.draw(gs::booleans()) {
            true => BTreeMap::new(),
            false => BTreeMap::from([(
                Name::parse("loose_items").unwrap(),
                WatchParameter {
                    unit: "items".into(),
                    observation: Expression::parse("loose.count").unwrap(),
                },
            )]),
        },
    };
    let text = document.document().to_json5();
    assert_eq!(RunDocument::parse(&text).unwrap(), document, "{text}");
}

fn a_workflow() -> Workflow {
    let text = json!({
        "workflow_schema": "workflow/v1",
        "manifest": { "name": "home", "revision": 2 },
        "task": { "objective": "Go home" },
        "contracts": { "postures": "postures:v1" },
        "roles": { "arm": {} },
        "success": "true",
        "actions": { "arm": { "tree": { "id": "home", "timeout_s": 30, "call": { "member": "postures.move_to_home" } } } },
    });
    Workflow::parse(&text.to_string()).unwrap()
}

fn a_lock() -> Lock {
    Lock {
        contracts: BTreeMap::from([(
            NameTag::parse("postures:v1").unwrap(),
            Sha256::parse(&"a".repeat(64)).unwrap(),
        )]),
        peppy: "v0.35.1".into(),
        nodes: BTreeMap::from([(
            NameTag::parse("workflow_director:v1").unwrap(),
            LockedNode {
                sha256: Sha256::parse(&"b".repeat(64)).unwrap(),
                commit: Some("5c02e1d".into()),
            },
        )]),
    }
}

#[hegel::test]
fn a_bundle_opens_as_written_and_refuses_any_corrupted_file(tc: TestCase) {
    let count = tc.draw(gs::integers::<usize>().min_value(0).max_value(3));
    let files: Vec<BundleFile> = (0..count)
        .map(|index| BundleFile {
            path: format!("{FILES_DIR}world_{index}.json5"),
            content: format!(
                "{{ n: {}, name: \"w{index}\" }}",
                tc.draw(gs::integers::<i32>().min_value(0).max_value(9))
            ),
        })
        .collect();
    let workflow = a_workflow();
    let written = Bundle::write(&workflow, files.clone(), a_lock()).unwrap();
    let bundle = Bundle::open(written.clone()).unwrap();
    assert_eq!(bundle.workflow, workflow);
    assert_eq!(bundle.document.lock, a_lock());
    assert_eq!(bundle.files.len(), count);
    let victim = tc.draw(
        gs::integers::<usize>()
            .min_value(0)
            .max_value(written.len() - 1),
    );
    let mut corrupted = written.clone();
    corrupted[victim].content = corrupted[victim].content.replacen('}', ", extra: 1 }", 1);
    assert!(Bundle::open(corrupted).is_err(), "{}", written[victim].path);
    let mut unlisted = written.clone();
    unlisted.push(BundleFile {
        path: format!("{FILES_DIR}stray.json5"),
        content: "{}".into(),
    });
    assert!(Bundle::open(unlisted).is_err());
    let without_workflow: Vec<BundleFile> = written
        .into_iter()
        .filter(|file| file.path != WORKFLOW_PATH)
        .collect();
    assert!(Bundle::open(without_workflow).is_err());
}

fn rates() -> impl PrintableGenerator<Rate> {
    gs::floats::<f64>()
        .min_value(0.0)
        .max_value(1.0)
        .map(|share| Rate::new(share).expect("a share from 0 to 1"))
}

/// The validation of a signed-off sweep of `a_workflow`: drawn thresholds,
/// and a drawn range for one control parameter.
#[hegel::composite]
fn validations(tc: &TestCase) -> Validation {
    let thresholds = Thresholds {
        completed_at_least: tc.draw(rates()),
        unsafe_events_at_most: tc.draw(gs::integers::<u32>().min_value(0).max_value(3)),
        false_successes_at_most: tc.draw(rates()),
        false_assumption_checks_at_most: tc.draw(gs::integers::<u32>().min_value(0).max_value(3)),
    };
    let speed = Name::parse("belt_speed_mps").unwrap();
    let min = tc.draw(measured());
    let max = min + tc.draw(gs::floats::<f64>().min_value(0.0).max_value(1.0));
    Validation::new(
        thresholds,
        BTreeMap::from([(
            speed.clone(),
            WorkCellParameter {
                unit: "m/s".into(),
                kind: ParameterKind::Control,
            },
        )]),
        BTreeMap::from([(speed, RoleValues::Single(TestedRange { min, max }))]),
    )
    .unwrap()
}

/// `files` with the bundle document rewritten by `change`.
fn with_bundle_document(
    files: &[BundleFile],
    change: impl Fn(&mut serde_json::Value),
) -> Vec<BundleFile> {
    files
        .iter()
        .map(|file| {
            if file.path != "bundle.json5" {
                return file.clone();
            }
            let mut document = workflow_documents::parse(&file.content).unwrap();
            change(&mut document);
            BundleFile {
                path: file.path.clone(),
                content: document.to_string(),
            }
        })
        .collect()
}

#[hegel::test]
fn the_digest_of_a_bundle_stays_with_its_trials_and_changes_with_any_other_field(tc: TestCase) {
    let validation = tc.draw(validations());
    let written =
        Bundle::write_signed_off(&a_workflow(), Vec::new(), a_lock(), validation.clone()).unwrap();
    let digest = Bundle::open(written.clone()).unwrap().document.digest();
    let reports: Vec<String> = (0..tc.draw(gs::integers::<usize>().min_value(1).max_value(3)))
        .map(|index| format!("{TRIALS_DIR}trial_{index}.json5"))
        .collect();
    let mut with_trials = with_bundle_document(&written, |document| {
        document["trials"] = json!(reports);
    });
    with_trials.extend(reports.iter().map(|path| BundleFile {
        path: path.clone(),
        content: "{ runs: 20 }".into(),
    }));
    let opened = Bundle::open(with_trials).unwrap();
    assert_eq!(opened.document.trials, reports);
    assert_eq!(opened.document.digest(), digest);
    let field = tc.draw(gs::sampled_from(&["thresholds", "envelope", "lock"]));
    let changed = with_bundle_document(&written, |document| match field {
        "thresholds" => {
            let at_most = &mut document["thresholds"]["unsafe_events_at_most"];
            *at_most = json!(at_most.as_u64().unwrap() + 1);
        }
        "envelope" => {
            let max = &mut document["envelope"]["belt_speed_mps"][1];
            *max = json!(max.as_f64().unwrap() + 1.0);
        }
        _ => document["lock"]["peppy"] = json!("v0.35.2"),
    });
    let opened = Bundle::open(changed).unwrap();
    assert_ne!(opened.document.digest(), digest, "{field}");
}
