//! The fields of the reference outputs that copy their scenarios.
//!
//! A field of a kind's outputs is its path with the list positions left out and `[]` for each
//! list it lies in, so `plans.0.path` and `plans.1.path` are the field `plans[].path`. A field
//! copies its scenario when each of its values that is a list of two or more records (lists
//! or objects) equals a whole list of records of the output's scenario, with each number of
//! both in single precision, the precision in which some references read their inputs. A
//! field whose values equal their input in some cases and not in others is a result.

use crate::{json_file, json_names, parse, read_bytes, reference_directory, scenario_directory};
use serde_json::Value;
use std::collections::BTreeMap;

/// A field of a kind's outputs: the kind, and the field's path with the list positions left
/// out and `[]` for each list it lies in.
type Field = (String, String);

/// How many values of a field are lists of records, and how many of them are copies.
#[derive(Default)]
struct Tally {
    values: usize,
    copies: usize,
}

/// The lists of records of the reference outputs of each kind, tallied by field.
#[derive(Default)]
struct Fields(BTreeMap<Field, Tally>);

impl Fields {
    /// Tallies each list of records of `output`, the reference output of `scenario`, a
    /// scenario of `kind`.
    fn add(&mut self, kind: &str, output: &Value, scenario: &Value) {
        let given = ScenarioLists::of(scenario);
        for (path, list) in record_lists(output) {
            let tally = self.0.entry((kind.to_owned(), path)).or_default();
            tally.values += 1;
            if given.holds(list) {
                tally.copies += 1;
            }
        }
    }

    /// The fields each of whose values is a copy, by kind and path, leaving out a field that
    /// lies inside another of them.
    fn copied(&self) -> Vec<Field> {
        let all: Vec<&Field> = self
            .0
            .iter()
            .filter(|(_, tally)| tally.copies == tally.values)
            .map(|(field, _)| field)
            .collect();
        all.iter()
            .filter(|(kind, path)| {
                !all.iter()
                    .any(|(outer_kind, outer)| outer_kind == kind && lies_inside(path, outer))
            })
            .map(|&field| field.clone())
            .collect()
    }
}

/// Whether the field `path` lies inside the field `outer`: a member of it, or a list in it.
fn lies_inside(path: &str, outer: &str) -> bool {
    path.strip_prefix(outer)
        .is_some_and(|rest| rest.starts_with('.') || rest.starts_with('['))
}

/// The lists of records of a scenario, each number in single precision.
struct ScenarioLists(Vec<Value>);

impl ScenarioLists {
    fn of(scenario: &Value) -> Self {
        Self(
            record_lists(scenario)
                .into_iter()
                .map(|(_, list)| in_single_precision(list))
                .collect(),
        )
    }

    /// Whether `list` equals one of the lists, with each number of both in single precision.
    fn holds(&self, list: &[Value]) -> bool {
        self.0.contains(&in_single_precision(list))
    }
}

/// Each list of two or more records in `value`, with the path of its field.
fn record_lists(value: &Value) -> Vec<(String, &[Value])> {
    let mut lists = Vec::new();
    collect_record_lists(value, "", &mut lists);
    lists
}

fn collect_record_lists<'a>(value: &'a Value, path: &str, lists: &mut Vec<(String, &'a [Value])>) {
    match value {
        Value::Array(items) => {
            if is_record_list(items) {
                lists.push((path.to_owned(), items.as_slice()));
            }
            let inner = format!("{path}[]");
            for item in items {
                collect_record_lists(item, &inner, lists);
            }
        }
        Value::Object(members) => {
            for (key, member) in members {
                let inner = if path.is_empty() {
                    key.clone()
                } else {
                    format!("{path}.{key}")
                };
                collect_record_lists(member, &inner, lists);
            }
        }
        Value::Null | Value::Bool(_) | Value::Number(_) | Value::String(_) => {}
    }
}

fn is_record_list(items: &[Value]) -> bool {
    items.len() >= 2 && items.iter().all(|item| item.is_array() || item.is_object())
}

/// `list` with each number in single precision, so that a copy, a copy in single precision
/// and an integer copy of a float compare equal.
fn in_single_precision(list: &[Value]) -> Value {
    Value::Array(list.iter().map(number_in_single_precision).collect())
}

/// `value` with each number as the single precision number nearest it, or as itself where
/// single precision holds no finite number near it.
fn number_in_single_precision(value: &Value) -> Value {
    match value {
        Value::Number(number) => {
            let x = number.as_f64().expect("a JSON number reads as a double");
            let single = x as f32;
            Value::from(if single.is_finite() {
                f64::from(single)
            } else {
                x
            })
        }
        Value::Array(items) => Value::Array(items.iter().map(number_in_single_precision).collect()),
        Value::Object(members) => Value::Object(
            members
                .iter()
                .map(|(key, member)| (key.clone(), number_in_single_precision(member)))
                .collect(),
        ),
        Value::Null | Value::Bool(_) | Value::String(_) => value.clone(),
    }
}

/// The `kind` that `scenario` names.
fn kind_of(name: &str, scenario: &Value) -> String {
    scenario
        .get("kind")
        .and_then(Value::as_str)
        .unwrap_or_else(|| panic!("scenarios/{name}.json names its kind"))
        .to_owned()
}

/// The fields of the reference outputs, every kind's, that copy their scenarios.
fn copied_fields_of_the_harness() -> Vec<Field> {
    let scenarios = scenario_directory();
    let outputs = reference_directory();
    let mut fields = Fields::default();
    for name in json_names(&outputs) {
        let scenario_path = json_file(&scenarios, &name);
        let output_path = json_file(&outputs, &name);
        let scenario: Value = parse(&scenario_path, &read_bytes(&scenario_path));
        let output: Value = parse(&output_path, &read_bytes(&output_path));
        fields.add(&kind_of(&name, &scenario), &output, &scenario);
    }
    fields.copied()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    /// The copied fields of the outputs `cases`, each a kind, an output and its scenario.
    fn copied(cases: &[(&str, Value, Value)]) -> Vec<Field> {
        let mut fields = Fields::default();
        for (kind, output, scenario) in cases {
            fields.add(kind, output, scenario);
        }
        fields.copied()
    }

    fn field(kind: &str, path: &str) -> Field {
        (kind.to_owned(), path.to_owned())
    }

    #[test]
    fn a_field_that_copies_its_scenario_in_every_output_is_named() {
        let scenario = json!({"kind": "k", "costs": [[0, 1], [2, 3]]});
        let output = json!({"costs": [[0, 1], [2, 3]], "plan": [[0.5, 1.5], [2.5, 3.5]]});
        assert_eq!(
            copied(&[("k", output, scenario)]),
            vec![field("k", "costs")]
        );
    }

    #[test]
    fn a_copy_in_single_precision_is_named() {
        let scenario = json!({"kind": "k", "clouds": [[0.1, 0.2], [0.3, 0.4]]});
        let rounded: Vec<Vec<f64>> = [[0.1, 0.2], [0.3, 0.4]]
            .iter()
            .map(|point| point.iter().map(|&x| f64::from(x as f32)).collect())
            .collect();
        let output = json!({"clouds": rounded});
        assert_eq!(
            copied(&[("k", output, scenario)]),
            vec![field("k", "clouds")]
        );
    }

    #[test]
    fn an_integer_copy_of_a_float_is_named() {
        let scenario = json!({"kind": "k", "footprint": [[1.0, 0.0], [0.0, 1.0]]});
        let output = json!({"footprint": [[1, 0], [0, 1]]});
        assert_eq!(
            copied(&[("k", output, scenario)]),
            vec![field("k", "footprint")]
        );
    }

    #[test]
    fn a_field_equal_to_its_input_in_some_outputs_and_not_in_others_is_a_result() {
        let empty = json!({"kind": "k", "costs": [[0, 0], [0, 0]]});
        let walled = json!({"kind": "k", "costs": [[254, 0], [0, 0]]});
        let cases = [
            ("k", json!({"inflated": [[0, 0], [0, 0]]}), empty),
            ("k", json!({"inflated": [[254, 253], [253, 128]]}), walled),
        ];
        assert_eq!(copied(&cases), Vec::<Field>::new());
    }

    #[test]
    fn a_list_of_copied_lists_names_the_inner_field() {
        let scenario = json!({"kind": "k", "grids": [{"costs": [[0, 1], [2, 3]]}, {"costs": [[4, 5], [6, 7]]}]});
        let output = json!({"given": [[[0, 1], [2, 3]], [[4, 5], [6, 7]]]});
        assert_eq!(
            copied(&[("k", output, scenario)]),
            vec![field("k", "given[]")]
        );
    }

    #[test]
    fn numbers_beyond_single_precision_compare_as_themselves() {
        let scenario = json!({"kind": "k", "big": [[1e300, 0], [0, 0]]});
        let output = json!({"big": [[2e300, 0], [0, 0]]});
        assert_eq!(copied(&[("k", output, scenario)]), Vec::<Field>::new());
    }

    #[test]
    fn the_values_of_a_field_are_judged_across_the_list_positions() {
        let scenario = json!({"kind": "k", "start": [[0, 0], [1, 1]]});
        let output = json!({"plans": [
            {"path": [[0, 0], [1, 1]]},
            {"path": [[0, 0], [0.5, 0.5], [1, 1]]}
        ]});
        assert_eq!(copied(&[("k", output, scenario)]), Vec::<Field>::new());
    }

    #[test]
    fn a_list_of_one_record_or_of_numbers_is_not_judged() {
        let scenario = json!({"kind": "k", "goal": [[1, 2]], "speed": [0.1, 0.2, 0.3]});
        let output = json!({"goal": [[1, 2]], "speed": [0.1, 0.2, 0.3]});
        assert_eq!(copied(&[("k", output, scenario)]), Vec::<Field>::new());
    }

    #[test]
    fn a_field_inside_a_copied_field_is_left_out() {
        let scenario = json!({"kind": "k", "worlds": [{"boxes": [[1, 2], [3, 4]]}, {"boxes": [[5, 6], [7, 8]]}]});
        let output = scenario.clone();
        assert_eq!(
            copied(&[("k", output, scenario)]),
            vec![field("k", "worlds")]
        );
    }

    #[test]
    fn the_fields_of_two_kinds_are_judged_apart() {
        let grid = json!([[0, 1], [2, 3]]);
        let cases = [
            (
                "a",
                json!({"costs": grid.clone()}),
                json!({"kind": "a", "costs": grid.clone()}),
            ),
            (
                "b",
                json!({"costs": [[9, 9], [9, 9]]}),
                json!({"kind": "b", "costs": grid}),
            ),
        ];
        assert_eq!(copied(&cases), vec![field("a", "costs")]);
    }

    #[test]
    fn no_reference_output_copies_its_scenario() {
        let copied = copied_fields_of_the_harness();
        assert!(
            copied.is_empty(),
            "these fields of the reference outputs copy their scenarios, which the readers \
             read: drop each from the kind's reference program and take it from the scenario \
             in its reader:\n{}",
            copied
                .iter()
                .map(|(kind, path)| format!("  {kind}: {path}"))
                .collect::<Vec<_>>()
                .join("\n")
        );
    }
}
