use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use super::{CompareOp, Expr, Literal};
use crate::names::Name;

/// The type of a value that an expression reads or gives. The workflow
/// director builds the types of observations and results from the message
/// formats of their contracts.
#[derive(Clone, Debug, PartialEq)]
pub enum Type {
    Bool,
    Int,
    Double,
    String,
    /// A string of a closed set of values (`$enum` of a contract).
    Enum(BTreeSet<String>),
    List(Box<Type>),
    Object(BTreeMap<String, FieldType>),
    /// A table of a workflow: one row per value of its key enum.
    Table {
        keys: BTreeSet<String>,
        row: Box<Type>,
    },
    Bytes,
    Time,
}

/// The type of a field of an object, and whether a message can leave it out.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldType {
    pub ty: Type,
    pub optional: bool,
}

impl fmt::Display for Type {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Type::Bool => f.write_str("bool"),
            Type::Int => f.write_str("int"),
            Type::Double => f.write_str("double"),
            Type::String => f.write_str("string"),
            Type::Enum(values) => {
                let values: Vec<String> = values.iter().map(|v| format!("'{v}'")).collect();
                write!(f, "one of {}", values.join(", "))
            }
            Type::List(item) => write!(f, "list of {item}"),
            Type::Object(_) => f.write_str("object"),
            Type::Table { .. } => f.write_str("table"),
            Type::Bytes => f.write_str("bytes"),
            Type::Time => f.write_str("time"),
        }
    }
}

impl Type {
    fn is_number(&self) -> bool {
        matches!(self, Type::Int | Type::Double)
    }

    fn is_text(&self) -> bool {
        matches!(self, Type::String | Type::Enum(_))
    }
}

/// The names an expression may read, each with its type.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TypeScope {
    names: BTreeMap<Name, Type>,
}

impl TypeScope {
    pub fn new(names: BTreeMap<Name, Type>) -> TypeScope {
        TypeScope { names }
    }

    pub fn get(&self, name: &Name) -> Option<&Type> {
        self.names.get(name)
    }

    fn with(&self, name: &Name, ty: Type) -> TypeScope {
        let mut names = self.names.clone();
        names.insert(name.clone(), ty);
        TypeScope { names }
    }
}

impl Expr {
    /// The type of the expression in `scope`, or why it has none: a name
    /// outside the scope, a field the type does not have, an optional field,
    /// an operation on types that do not take it, or a string literal that
    /// is not a value of the enum it is compared with.
    pub fn infer(&self, scope: &TypeScope) -> Result<Type, String> {
        match self {
            Expr::Literal(literal) => Ok(literal_type(literal)),
            Expr::List(items) => {
                let mut types = items.iter().map(|item| item.infer(scope));
                let first = types.next().ok_or_else(|| {
                    "an empty list has no type; compare `size(...)` with 0".to_owned()
                })??;
                for ty in types {
                    if !comparable(&first, &ty?) {
                        return Err(format!("`{self}` holds items of different types"));
                    }
                }
                Ok(Type::List(Box::new(first)))
            }
            Expr::Ident(name) => scope
                .get(name)
                .cloned()
                .ok_or_else(|| format!("`{name}` is no name this expression can read")),
            Expr::Select(operand, field) => match operand.infer(scope)? {
                Type::Object(fields) => match fields.get(field) {
                    Some(FieldType { optional: true, .. }) => Err(format!(
                        "`{self}` reads the optional field `{field}`, which a message can leave out"
                    )),
                    Some(FieldType { ty, .. }) => Ok(ty.clone()),
                    None => Err(format!("`{operand}` has no field `{field}`")),
                },
                other => Err(format!("`{operand}` is {other}, which has no fields")),
            },
            Expr::Index(operand, index) => match (operand.infer(scope)?, index.infer(scope)?) {
                (Type::List(item), Type::Int) => Ok(*item),
                (Type::Table { keys, row }, key) => {
                    if let Expr::Literal(Literal::String(value)) = index.as_ref() {
                        if !keys.contains(value) {
                            return Err(format!("the table `{operand}` has no row '{value}'"));
                        }
                        return Ok(*row);
                    }
                    match key {
                        Type::Enum(values) if values == keys => Ok(*row),
                        other => Err(format!(
                            "the table `{operand}` takes a key of its enum, not {other}"
                        )),
                    }
                }
                (other, _) => Err(format!("`{operand}` is {other}, which takes no index")),
            },
            Expr::Not(operand) => boolean(operand, scope).map(|()| Type::Bool),
            Expr::And(a, b) | Expr::Or(a, b) => {
                boolean(a, scope)?;
                boolean(b, scope)?;
                Ok(Type::Bool)
            }
            Expr::Compare(op, a, b) => {
                let (left, right) = (a.infer(scope)?, b.infer(scope)?);
                enum_literals(&left, b)?;
                enum_literals(&right, a)?;
                let ordered = !matches!(op, CompareOp::Eq | CompareOp::Ne);
                let fits = match ordered {
                    true => {
                        (left.is_number() && right.is_number())
                            || (left == Type::String && right == Type::String)
                    }
                    false => comparable(&left, &right),
                };
                if !fits {
                    return Err(format!("`{self}` compares {left} with {right}"));
                }
                Ok(Type::Bool)
            }
            Expr::In(item, list) => match list.infer(scope)? {
                Type::List(of) => {
                    let ty = item.infer(scope)?;
                    if let Expr::List(items) = list.as_ref() {
                        items
                            .iter()
                            .try_for_each(|literal| enum_literals(&ty, literal))?;
                    }
                    enum_literals(&of, item)?;
                    if !comparable(&ty, &of) {
                        return Err(format!("`{self}` looks for {ty} in a list of {of}"));
                    }
                    Ok(Type::Bool)
                }
                other => Err(format!("`{list}` is {other}, not a list")),
            },
            Expr::Size(operand) => match operand.infer(scope)? {
                Type::List(_) | Type::String | Type::Enum(_) => Ok(Type::Int),
                other => Err(format!("`{operand}` is {other}, which has no size")),
            },
            Expr::All {
                range,
                var,
                predicate,
            }
            | Expr::Exists {
                range,
                var,
                predicate,
            } => match range.infer(scope)? {
                Type::List(item) => {
                    if scope.get(var).is_some() {
                        return Err(format!(
                            "the element `{var}` has the name of another value; give it its own"
                        ));
                    }
                    boolean(predicate, &scope.with(var, *item)).map(|()| Type::Bool)
                }
                other => Err(format!("`{range}` is {other}, not a list")),
            },
        }
    }
}

fn literal_type(literal: &Literal) -> Type {
    match literal {
        Literal::Bool(_) => Type::Bool,
        Literal::Int(_) | Literal::UInt(_) => Type::Int,
        Literal::Double(_) => Type::Double,
        Literal::String(_) => Type::String,
    }
}

fn boolean(expr: &Expr, scope: &TypeScope) -> Result<(), String> {
    match expr.infer(scope)? {
        Type::Bool => Ok(()),
        other => Err(format!("`{expr}` is {other}, not a bool")),
    }
}

/// Whether two types compare for equality: equal types, two numbers, or two
/// texts (an enum and a string).
fn comparable(a: &Type, b: &Type) -> bool {
    match (a, b) {
        (Type::Time | Type::Bytes, _) | (_, Type::Time | Type::Bytes) => false,
        (Type::List(_) | Type::Object(_) | Type::Table { .. }, _) => false,
        _ => a == b || (a.is_number() && b.is_number()) || (a.is_text() && b.is_text()),
    }
}

/// Refuses a string literal `other` compared with an enum that does not hold
/// it.
fn enum_literals(ty: &Type, other: &Expr) -> Result<(), String> {
    let (Type::Enum(values), Expr::Literal(Literal::String(value))) = (ty, other) else {
        return Ok(());
    };
    if values.contains(value) {
        return Ok(());
    }
    Err(format!(
        "'{value}' is not a value of {}",
        Type::Enum(values.clone())
    ))
}

#[cfg(test)]
mod tests {
    use super::super::Expression;
    use super::*;

    fn name(text: &str) -> Name {
        Name::parse(text).unwrap()
    }

    fn scope() -> TypeScope {
        let kinds: BTreeSet<String> = ["book", "toy", "unknown"].map(String::from).into();
        let sorted = Type::Object(BTreeMap::from([
            (
                "item_kind".to_owned(),
                FieldType {
                    ty: Type::Enum(kinds.clone()),
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
        let found = Type::Object(BTreeMap::from([
            (
                "item_ids".to_owned(),
                FieldType {
                    ty: Type::List(Box::new(Type::String)),
                    optional: false,
                },
            ),
            (
                "kinds".to_owned(),
                FieldType {
                    ty: Type::List(Box::new(Type::Enum(kinds.clone()))),
                    optional: false,
                },
            ),
        ]));
        TypeScope::new(BTreeMap::from([
            (name("sorted"), sorted),
            (name("found"), found),
            (name("count"), Type::Int),
            (
                name("shelf"),
                Type::Table {
                    keys: kinds,
                    row: Box::new(Type::Double),
                },
            ),
        ]))
    }

    fn infer(source: &str) -> Result<Type, String> {
        Expression::parse(source).unwrap().expr().infer(&scope())
    }

    #[test]
    fn well_typed_expressions_give_their_types() {
        assert_eq!(infer("sorted.item_kind == 'book'"), Ok(Type::Bool));
        assert_eq!(
            infer("count > 1.5 && size(found.item_ids) <= 20"),
            Ok(Type::Bool)
        );
        assert_eq!(infer("found.item_ids.all(id, id != '')"), Ok(Type::Bool));
        assert_eq!(infer("shelf['toy'] > 0.0"), Ok(Type::Bool));
        assert_eq!(infer("shelf[sorted.item_kind]"), Ok(Type::Double));
        assert_eq!(infer("sorted.item_kind in ['book', 'toy']"), Ok(Type::Bool));
    }

    #[test]
    fn a_literal_outside_the_enum_is_refused() {
        assert!(
            infer("sorted.item_kind == 'boook'")
                .unwrap_err()
                .contains("'boook'")
        );
        assert!(
            infer("sorted.item_kind in ['book', 'cup']")
                .unwrap_err()
                .contains("'cup'")
        );
        assert!(infer("shelf['cup']").unwrap_err().contains("no row"));
        assert!(infer("'cup' in found.kinds").unwrap_err().contains("'cup'"));
        assert_eq!(infer("'toy' in found.kinds"), Ok(Type::Bool));
    }

    #[test]
    fn untyped_reads_are_refused_with_their_cause() {
        assert!(infer("missing > 1").unwrap_err().contains("no name"));
        assert!(infer("sorted.note == ''").unwrap_err().contains("optional"));
        assert!(infer("sorted.nope").unwrap_err().contains("no field"));
        assert!(infer("count && true").unwrap_err().contains("not a bool"));
        assert!(infer("found == found").unwrap_err().contains("compares"));
        assert!(infer("count < 'a'").unwrap_err().contains("compares"));
        assert!(
            infer("found.item_ids.all(count, count)")
                .unwrap_err()
                .contains("own")
        );
    }
}
