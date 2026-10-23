//! Properties of the value tree, the digest and the root key rule on drawn
//! documents: every spelling of one document parses to its tree and digests
//! the same, a key twice is refused by name, and a typed body survives the
//! document.

use documents_core::{Document, Error, Kind, parse};
use hegel::TestCase;
use hegel::generators::{self as gs, Generator, PrintableGenerator};
use serde_json::{Map, Number, Value};

const WORLD: Kind = Kind::new(ROOT_KEY, "world", 1);

/// The root key of `WORLD` and its value, as a document spells them.
const ROOT_KEY: &str = "simulation_schema";
const TAG: &str = "world/v1";

/// The largest magnitude of a number in a document: 2^53.
const MAX_EXACT_INTEGER: i64 = 1 << 53;

/// The keys of a drawn object: short texts over a few letters, a digit and
/// `_`, so the JSON5 writer leaves most unquoted and some need quotes.
fn keys() -> impl PrintableGenerator<String> {
    gs::text().alphabet("ab_1").min_size(1).max_size(3)
}

/// A number a document holds: an integer or a finite float, within 2^53.
#[hegel::composite]
fn numbers(tc: &TestCase) -> Value {
    if tc.draw(gs::booleans()) {
        let integer = tc.draw(
            gs::integers::<i64>()
                .min_value(-MAX_EXACT_INTEGER)
                .max_value(MAX_EXACT_INTEGER),
        );
        return Value::Number(Number::from(integer));
    }
    let bound = MAX_EXACT_INTEGER as f64;
    let float = tc.draw(gs::floats::<f64>().min_value(-bound).max_value(bound));
    Value::Number(Number::from_f64(float).expect("a finite float"))
}

/// A tree a document holds: null, booleans, numbers and texts at the leaves;
/// arrays and objects above them, each key once.
fn trees() -> impl PrintableGenerator<Value> {
    gs::recursive(
        hegel::one_of!(
            gs::just(Value::Null),
            gs::booleans().map(Value::Bool),
            numbers(),
            gs::text().max_size(6).map(Value::String)
        ),
        |subtrees| {
            gs::tuples2(
                gs::booleans(),
                gs::vecs(gs::tuples2(keys(), subtrees)).max_size(4),
            )
            .map(|(array, pairs)| {
                if array {
                    return Value::Array(pairs.into_iter().map(|(_, value)| value).collect());
                }
                Value::Object(pairs.into_iter().collect())
            })
        },
    )
}

/// `tree` with the keys of each object in a drawn order.
fn shuffled(tc: &TestCase, tree: &Value) -> Value {
    match tree {
        Value::Array(items) => Value::Array(items.iter().map(|item| shuffled(tc, item)).collect()),
        Value::Object(fields) => {
            let order = tc.draw(gs::permutations(fields.keys().cloned().collect::<Vec<_>>()));
            Value::Object(
                order
                    .into_iter()
                    .map(|key| (key.clone(), shuffled(tc, &fields[&key])))
                    .collect(),
            )
        }
        leaf => leaf.clone(),
    }
}

#[hegel::test]
fn every_spelling_of_a_document_parses_to_its_tree_and_digests_the_same(tc: TestCase) {
    let mut fields = Map::from_iter([(ROOT_KEY.to_owned(), Value::String(TAG.to_owned()))]);
    fields.insert("body".to_owned(), tc.draw(trees()));
    let tree = Value::Object(fields);
    let shuffled = shuffled(&tc, &tree);
    let spellings = [
        json5::to_string(&tree).unwrap(),
        serde_json::to_string(&tree).unwrap(),
        serde_json::to_string_pretty(&shuffled).unwrap(),
        json5::to_string(&shuffled).unwrap(),
    ];
    let expected = parse(&spellings[0]).unwrap();
    let digest = Document::parse(&spellings[0], WORLD).unwrap().digest();
    for spelling in &spellings {
        assert_eq!(parse(spelling).unwrap(), expected, "{spelling}");
        assert_eq!(
            Document::parse(spelling, WORLD).unwrap().digest(),
            digest,
            "{spelling}"
        );
    }
}

#[hegel::test]
fn a_key_twice_in_an_object_is_refused_by_its_name(tc: TestCase) {
    let fields = tc.draw(gs::btree_maps(keys(), trees()).min_size(1).max_size(4));
    let key = tc.draw(gs::sampled_from(fields.keys().cloned().collect::<Vec<_>>()));
    let written = serde_json::to_string(&fields).unwrap();
    let with_duplicate = format!(
        "{},{}:0}}",
        &written[..written.len() - 1],
        serde_json::to_string(&key).unwrap()
    );
    assert_eq!(
        parse(&with_duplicate),
        Err(Error::DuplicateKey {
            path: String::new(),
            key
        })
    );
}

#[hegel::test]
fn a_body_survives_the_document_of_its_kind(tc: TestCase) {
    let drawn: Map<String, Value> = tc
        .draw(gs::btree_maps(keys(), trees()).max_size(4))
        .into_iter()
        .filter(|(key, _)| key != ROOT_KEY)
        .collect();
    let body: Map<String, Value> =
        serde_json::from_value(parse(&serde_json::to_string(&drawn).unwrap()).unwrap()).unwrap();
    let document = Document::of(WORLD, &body).unwrap();
    assert_eq!(document.read::<Map<String, Value>>().unwrap(), body);
    let reparsed = Document::parse(&document.to_json5(), WORLD).unwrap();
    assert_eq!(reparsed, document);
    assert_eq!(reparsed.digest(), document.digest());
    assert!(
        document
            .to_json5()
            .starts_with("{\n  simulation_schema: \"world/v1\""),
        "{}",
        document.to_json5()
    );
}

#[hegel::test]
fn another_kind_is_refused_whatever_the_fields_hold(tc: TestCase) {
    let body = tc.draw(gs::btree_maps(keys(), trees()).max_size(3));
    let (name, version) = tc.draw(gs::tuples2(
        gs::sampled_from(&["world", "sweep", "workflow"]),
        gs::integers::<u32>().min_value(0).max_value(3),
    ));
    let other = Kind::new("simulation_schema", name, version);
    if other == WORLD {
        return;
    }
    let document = Document::of(other, &body).unwrap();
    assert_eq!(
        Document::parse(&document.to_json5(), WORLD),
        Err(Error::WrongKind {
            root_key: "simulation_schema",
            found: format!("\"{name}/v{version}\""),
            expected: "world/v1".into()
        })
    );
}
