use std::cmp::Ordering;
use std::collections::BTreeMap;

use super::{CompareOp, Expr, Literal, Type};
use crate::names::Name;

/// A value that an expression reads or gives.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Bool(bool),
    Int(i128),
    Double(f64),
    String(String),
    List(Vec<Value>),
    Object(BTreeMap<String, Value>),
}

/// Why an expression that typed gives no value.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum EvalError {
    #[error("`{0}` has no value")]
    Unknown(Name),
    #[error("index {index} is beyond the {len} items of `{list}`")]
    IndexOutOfRange {
        list: String,
        index: i128,
        len: usize,
    },
    #[error("`{0}` gives no value of the type its expression needs")]
    Mismatch(String),
}

/// The values of the names an expression reads.
pub trait Scope {
    fn value(&self, name: &Name) -> Option<Value>;
}

impl Scope for BTreeMap<Name, Value> {
    fn value(&self, name: &Name) -> Option<Value> {
        self.get(name).cloned()
    }
}

/// A scope with one more name: the element of an `all` or an `exists`.
struct With<'a> {
    outer: &'a dyn Scope,
    name: &'a Name,
    value: Value,
}

impl Scope for With<'_> {
    fn value(&self, name: &Name) -> Option<Value> {
        if name == self.name {
            return Some(self.value.clone());
        }
        self.outer.value(name)
    }
}

impl Value {
    /// Reads the canonical JSON of a message, as the codec gives it, by its
    /// type: an integer is a JSON number or the decimal text the codec gives
    /// a 64-bit integer; an object keeps the fields of its type; bytes and a
    /// time stay their text.
    pub fn read(json: &serde_json::Value, ty: &Type) -> Result<Value, String> {
        use serde_json::Value as Json;
        let mismatch = || format!("{json} is not {ty}");
        match (ty, json) {
            (Type::Bool, Json::Bool(b)) => Ok(Value::Bool(*b)),
            (Type::Int, Json::Number(n)) => n
                .as_i64()
                .map(i128::from)
                .or_else(|| n.as_u64().map(i128::from))
                .map(Value::Int)
                .ok_or_else(mismatch),
            (Type::Int, Json::String(text)) => {
                text.parse::<i128>().map(Value::Int).map_err(|_| mismatch())
            }
            (Type::Double, Json::Number(n)) => n.as_f64().map(Value::Double).ok_or_else(mismatch),
            (Type::String | Type::Enum(_) | Type::Bytes | Type::Time, Json::String(text)) => {
                Ok(Value::String(text.clone()))
            }
            (Type::List(item), Json::Array(items)) => items
                .iter()
                .map(|value| Value::read(value, item))
                .collect::<Result<_, _>>()
                .map(Value::List),
            (Type::Object(fields), Json::Object(object)) => fields
                .iter()
                .filter_map(|(name, field)| match object.get(name) {
                    Some(value) => {
                        Some(Value::read(value, &field.ty).map(|value| (name.clone(), value)))
                    }
                    None if field.optional => None,
                    None => Some(Err(format!("{json} has no field `{name}`"))),
                })
                .collect::<Result<_, _>>()
                .map(Value::Object),
            (Type::Table { keys, row }, Json::Object(object)) => keys
                .iter()
                .map(|key| {
                    let value = object
                        .get(key)
                        .ok_or_else(|| format!("{json} has no row `{key}`"))?;
                    Value::read(value, row).map(|value| (key.clone(), value))
                })
                .collect::<Result<_, _>>()
                .map(Value::Object),
            _ => Err(mismatch()),
        }
    }
}

impl Expr {
    /// The value of the expression in `scope`. An expression that typed
    /// fails only on an index beyond its list or a value its scope lacks.
    pub fn evaluate(&self, scope: &dyn Scope) -> Result<Value, EvalError> {
        match self {
            Expr::Literal(literal) => Ok(literal_value(literal)),
            Expr::List(items) => items
                .iter()
                .map(|item| item.evaluate(scope))
                .collect::<Result<_, _>>()
                .map(Value::List),
            Expr::Ident(name) => scope
                .value(name)
                .ok_or_else(|| EvalError::Unknown(name.clone())),
            Expr::Select(operand, field) => match operand.evaluate(scope)? {
                Value::Object(mut fields) => fields
                    .remove(field)
                    .ok_or_else(|| EvalError::Mismatch(self.to_string())),
                _ => Err(EvalError::Mismatch(self.to_string())),
            },
            Expr::Index(operand, index) => match (operand.evaluate(scope)?, index.evaluate(scope)?)
            {
                (Value::List(items), Value::Int(i)) => {
                    let len = items.len();
                    usize::try_from(i)
                        .ok()
                        .and_then(|i| items.into_iter().nth(i))
                        .ok_or(EvalError::IndexOutOfRange {
                            list: operand.to_string(),
                            index: i,
                            len,
                        })
                }
                (Value::Object(mut rows), Value::String(key)) => rows
                    .remove(&key)
                    .ok_or_else(|| EvalError::Mismatch(self.to_string())),
                _ => Err(EvalError::Mismatch(self.to_string())),
            },
            Expr::Not(operand) => Ok(Value::Bool(!truth(operand, scope)?)),
            Expr::And(a, b) => both(a, b, scope, false),
            Expr::Or(a, b) => both(a, b, scope, true),
            Expr::Compare(op, a, b) => {
                let (left, right) = (a.evaluate(scope)?, b.evaluate(scope)?);
                let order = compare(&left, &right);
                Ok(Value::Bool(match op {
                    CompareOp::Eq => order == Some(Ordering::Equal),
                    CompareOp::Ne => order != Some(Ordering::Equal),
                    CompareOp::Lt => order == Some(Ordering::Less),
                    CompareOp::Le => matches!(order, Some(Ordering::Less | Ordering::Equal)),
                    CompareOp::Gt => order == Some(Ordering::Greater),
                    CompareOp::Ge => matches!(order, Some(Ordering::Greater | Ordering::Equal)),
                }))
            }
            Expr::In(item, list) => {
                let item = item.evaluate(scope)?;
                match list.evaluate(scope)? {
                    Value::List(items) => {
                        Ok(Value::Bool(items.iter().any(|candidate| {
                            compare(&item, candidate) == Some(Ordering::Equal)
                        })))
                    }
                    _ => Err(EvalError::Mismatch(self.to_string())),
                }
            }
            Expr::Size(operand) => match operand.evaluate(scope)? {
                Value::List(items) => Ok(Value::Int(items.len() as i128)),
                Value::String(text) => Ok(Value::Int(text.chars().count() as i128)),
                _ => Err(EvalError::Mismatch(self.to_string())),
            },
            Expr::All {
                range,
                var,
                predicate,
            } => quantified(range, var, predicate, scope, true),
            Expr::Exists {
                range,
                var,
                predicate,
            } => quantified(range, var, predicate, scope, false),
        }
    }
}

fn literal_value(literal: &Literal) -> Value {
    match literal {
        Literal::Bool(b) => Value::Bool(*b),
        Literal::Int(i) => Value::Int(i128::from(*i)),
        Literal::UInt(u) => Value::Int(i128::from(*u)),
        Literal::Double(d) => Value::Double(*d),
        Literal::String(s) => Value::String(s.clone()),
    }
}

fn truth(expr: &Expr, scope: &dyn Scope) -> Result<bool, EvalError> {
    match expr.evaluate(scope)? {
        Value::Bool(b) => Ok(b),
        _ => Err(EvalError::Mismatch(expr.to_string())),
    }
}

/// `&&` (`decisive` false) or `||` (`decisive` true) as CEL gives it: a side
/// that gives the decisive value decides, whatever the other side gives, an
/// error included.
fn both(a: &Expr, b: &Expr, scope: &dyn Scope, decisive: bool) -> Result<Value, EvalError> {
    let left = truth(a, scope);
    if left == Ok(decisive) {
        return Ok(Value::Bool(decisive));
    }
    let right = truth(b, scope);
    if right == Ok(decisive) {
        return Ok(Value::Bool(decisive));
    }
    left?;
    right?;
    Ok(Value::Bool(!decisive))
}

fn quantified(
    range: &Expr,
    var: &Name,
    predicate: &Expr,
    scope: &dyn Scope,
    all: bool,
) -> Result<Value, EvalError> {
    let Value::List(items) = range.evaluate(scope)? else {
        return Err(EvalError::Mismatch(range.to_string()));
    };
    let mut error = None;
    for value in items {
        let element = With {
            outer: scope,
            name: var,
            value,
        };
        match truth(predicate, &element) {
            Ok(holds) if holds != all => return Ok(Value::Bool(!all)),
            Ok(_) => {}
            Err(e) => error = error.or(Some(e)),
        }
    }
    match error {
        Some(e) => Err(e),
        None => Ok(Value::Bool(all)),
    }
}

/// The order of two values: numbers by their mathematical value, texts and
/// bools by their own order; `None` for values that do not compare.
fn compare(a: &Value, b: &Value) -> Option<Ordering> {
    match (a, b) {
        (Value::Int(x), Value::Int(y)) => Some(x.cmp(y)),
        (Value::Double(x), Value::Double(y)) => x.partial_cmp(y),
        (Value::Int(x), Value::Double(y)) => compare_int_double(*x, *y),
        (Value::Double(x), Value::Int(y)) => compare_int_double(*y, *x).map(Ordering::reverse),
        (Value::String(x), Value::String(y)) => Some(x.cmp(y)),
        (Value::Bool(x), Value::Bool(y)) => Some(x.cmp(y)),
        _ => None,
    }
}

fn compare_int_double(int: i128, double: f64) -> Option<Ordering> {
    if double.is_nan() {
        return None;
    }
    if double.fract() == 0.0 && double.abs() < 1.7e38 {
        return Some(int.cmp(&(double as i128)));
    }
    (int as f64).partial_cmp(&double)
}

#[cfg(test)]
mod tests {
    use super::super::{Expression, FieldType};
    use super::*;
    use serde_json::json;

    fn name(text: &str) -> Name {
        Name::parse(text).unwrap()
    }

    fn evaluate(source: &str, scope: &BTreeMap<Name, Value>) -> Result<Value, EvalError> {
        Expression::parse(source).unwrap().expr().evaluate(scope)
    }

    #[test]
    fn values_compare_by_their_mathematical_value() {
        let scope = BTreeMap::from([(name("n"), Value::Int(3))]);
        assert_eq!(evaluate("n == 3.0", &scope), Ok(Value::Bool(true)));
        assert_eq!(evaluate("n < 3.5 && n > 2u", &scope), Ok(Value::Bool(true)));
        assert_eq!(evaluate("'b' > 'a'", &scope), Ok(Value::Bool(true)));
    }

    #[test]
    fn and_and_or_absorb_an_error_of_the_other_side_as_cel_does() {
        let scope = BTreeMap::from([(name("items"), Value::List(vec![]))]);
        assert_eq!(
            evaluate("items[0] == 1 && false", &scope),
            Ok(Value::Bool(false))
        );
        assert_eq!(
            evaluate("true || items[0] == 1", &scope),
            Ok(Value::Bool(true))
        );
        assert!(matches!(
            evaluate("items[0] == 1 && true", &scope),
            Err(EvalError::IndexOutOfRange {
                index: 0,
                len: 0,
                ..
            })
        ));
    }

    #[test]
    fn all_and_exists_bind_their_element() {
        let scope = BTreeMap::from([(name("xs"), Value::List(vec![Value::Int(1), Value::Int(4)]))]);
        assert_eq!(evaluate("xs.all(x, x > 0)", &scope), Ok(Value::Bool(true)));
        assert_eq!(
            evaluate("xs.exists(x, x > 3)", &scope),
            Ok(Value::Bool(true))
        );
        assert_eq!(evaluate("xs.all(x, x > 3)", &scope), Ok(Value::Bool(false)));
        assert_eq!(
            evaluate("size(xs) == 2 && 4 in xs", &scope),
            Ok(Value::Bool(true))
        );
    }

    #[test]
    fn a_message_reads_by_its_type_with_64_bit_integers_as_text() {
        let ty = Type::Object(BTreeMap::from([
            (
                "count".to_owned(),
                FieldType {
                    ty: Type::Int,
                    optional: false,
                },
            ),
            (
                "note".to_owned(),
                FieldType {
                    ty: Type::String,
                    optional: true,
                },
            ),
        ]));
        assert_eq!(
            Value::read(&json!({"count": "18446744073709551615"}), &ty),
            Ok(Value::Object(BTreeMap::from([(
                "count".to_owned(),
                Value::Int(u64::MAX as i128)
            )])))
        );
        assert!(Value::read(&json!({"note": "x"}), &ty).is_err());
    }
}
