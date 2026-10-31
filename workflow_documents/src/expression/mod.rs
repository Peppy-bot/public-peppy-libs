//! The expressions of a workflow: a subset of CEL that gives typed values and
//! never structure. An expression holds literals, names, fields, indexes, `!`,
//! `&&`, `||`, comparisons, `in`, `size()`, `all()` and `exists()`; the parser
//! refuses every other form, naming it and its place in the text.

mod evaluate;
mod parse;
mod types;

use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::BTreeSet;
use std::fmt;

use crate::names::Name;

pub use evaluate::{EvalError, Scope, Value};
pub use types::{FieldType, Type, TypeScope};

/// A comparison of two values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompareOp {
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
}

impl CompareOp {
    fn symbol(self) -> &'static str {
        match self {
            CompareOp::Eq => "==",
            CompareOp::Ne => "!=",
            CompareOp::Lt => "<",
            CompareOp::Le => "<=",
            CompareOp::Gt => ">",
            CompareOp::Ge => ">=",
        }
    }
}

/// A literal of an expression.
#[derive(Clone, Debug, PartialEq)]
pub enum Literal {
    Bool(bool),
    Int(i64),
    UInt(u64),
    Double(f64),
    String(String),
}

/// An expression of the subset.
#[derive(Clone, Debug, PartialEq)]
pub enum Expr {
    Literal(Literal),
    List(Vec<Expr>),
    Ident(Name),
    Select(Box<Expr>, String),
    Index(Box<Expr>, Box<Expr>),
    Not(Box<Expr>),
    And(Box<Expr>, Box<Expr>),
    Or(Box<Expr>, Box<Expr>),
    Compare(CompareOp, Box<Expr>, Box<Expr>),
    In(Box<Expr>, Box<Expr>),
    Size(Box<Expr>),
    All {
        range: Box<Expr>,
        var: Name,
        predicate: Box<Expr>,
    },
    Exists {
        range: Box<Expr>,
        var: Name,
        predicate: Box<Expr>,
    },
}

impl Expr {
    /// The names the expression reads from its scope: each identifier that
    /// no enclosing `all` or `exists` binds.
    pub fn roots(&self) -> BTreeSet<Name> {
        let mut found = BTreeSet::new();
        self.collect_roots(&mut Vec::new(), &mut found);
        found
    }

    fn collect_roots(&self, bound: &mut Vec<Name>, found: &mut BTreeSet<Name>) {
        match self {
            Expr::Literal(_) => {}
            Expr::Ident(name) => {
                if !bound.contains(name) {
                    found.insert(name.clone());
                }
            }
            Expr::List(items) => items
                .iter()
                .for_each(|item| item.collect_roots(bound, found)),
            Expr::Select(operand, _) | Expr::Not(operand) | Expr::Size(operand) => {
                operand.collect_roots(bound, found)
            }
            Expr::Index(a, b)
            | Expr::And(a, b)
            | Expr::Or(a, b)
            | Expr::Compare(_, a, b)
            | Expr::In(a, b) => {
                a.collect_roots(bound, found);
                b.collect_roots(bound, found);
            }
            Expr::All {
                range,
                var,
                predicate,
            }
            | Expr::Exists {
                range,
                var,
                predicate,
            } => {
                range.collect_roots(bound, found);
                bound.push(var.clone());
                predicate.collect_roots(bound, found);
                bound.pop();
            }
        }
    }

    /// The expression with each free identifier that `with` gives a
    /// replacement for replaced by it: how a lane group puts the value of a
    /// parameter in the trees of each of its roles.
    pub fn substituted(&self, with: &dyn Fn(&Name) -> Option<Expr>) -> Expr {
        self.substitute(with, &mut Vec::new())
    }

    fn substitute(&self, with: &dyn Fn(&Name) -> Option<Expr>, bound: &mut Vec<Name>) -> Expr {
        fn sub(e: &Expr, with: &dyn Fn(&Name) -> Option<Expr>, bound: &mut Vec<Name>) -> Box<Expr> {
            Box::new(e.substitute(with, bound))
        }
        match self {
            Expr::Literal(_) => self.clone(),
            Expr::Ident(name) if !bound.contains(name) => {
                with(name).unwrap_or_else(|| self.clone())
            }
            Expr::Ident(_) => self.clone(),
            Expr::List(items) => Expr::List(
                items
                    .iter()
                    .map(|item| item.substitute(with, bound))
                    .collect(),
            ),
            Expr::Select(operand, field) => Expr::Select(sub(operand, with, bound), field.clone()),
            Expr::Not(operand) => Expr::Not(sub(operand, with, bound)),
            Expr::Size(operand) => Expr::Size(sub(operand, with, bound)),
            Expr::Index(a, b) => Expr::Index(sub(a, with, bound), sub(b, with, bound)),
            Expr::And(a, b) => Expr::And(sub(a, with, bound), sub(b, with, bound)),
            Expr::Or(a, b) => Expr::Or(sub(a, with, bound), sub(b, with, bound)),
            Expr::Compare(op, a, b) => Expr::Compare(*op, sub(a, with, bound), sub(b, with, bound)),
            Expr::In(a, b) => Expr::In(sub(a, with, bound), sub(b, with, bound)),
            Expr::All {
                range,
                var,
                predicate,
            } => {
                let range = sub(range, with, bound);
                bound.push(var.clone());
                let predicate = sub(predicate, with, bound);
                bound.pop();
                Expr::All {
                    range,
                    var: var.clone(),
                    predicate,
                }
            }
            Expr::Exists {
                range,
                var,
                predicate,
            } => {
                let range = sub(range, with, bound);
                bound.push(var.clone());
                let predicate = sub(predicate, with, bound);
                bound.pop();
                Expr::Exists {
                    range,
                    var: var.clone(),
                    predicate,
                }
            }
        }
    }
}

impl fmt::Display for Literal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Literal::Bool(b) => write!(f, "{b}"),
            Literal::Int(i) => write!(f, "{i}"),
            Literal::UInt(u) => write!(f, "{u}u"),
            // Debug writes a point or an exponent, so the text reads as a double.
            Literal::Double(d) => write!(f, "{d:?}"),
            Literal::String(s) => {
                f.write_str("'")?;
                for c in s.chars() {
                    match c {
                        '\\' => f.write_str("\\\\")?,
                        '\'' => f.write_str("\\'")?,
                        '\n' => f.write_str("\\n")?,
                        '\r' => f.write_str("\\r")?,
                        '\t' => f.write_str("\\t")?,
                        c if c.is_control() => write!(f, "\\u{:04x}", u32::from(c))?,
                        c => write!(f, "{c}")?,
                    }
                }
                f.write_str("'")
            }
        }
    }
}

/// The expression as CEL text that parses back to the same expression: every
/// operation in parentheses.
impl fmt::Display for Expr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Expr::Literal(literal) => write!(f, "{literal}"),
            Expr::List(items) => {
                let items: Vec<String> = items.iter().map(Expr::to_string).collect();
                write!(f, "[{}]", items.join(", "))
            }
            Expr::Ident(name) => write!(f, "{name}"),
            Expr::Select(operand, field) => write!(f, "{operand}.{field}"),
            Expr::Index(operand, index) => write!(f, "{operand}[{index}]"),
            Expr::Not(operand) => write!(f, "!({operand})"),
            Expr::And(a, b) => write!(f, "({a} && {b})"),
            Expr::Or(a, b) => write!(f, "({a} || {b})"),
            Expr::Compare(op, a, b) => write!(f, "({a} {} {b})", op.symbol()),
            Expr::In(a, b) => write!(f, "({a} in {b})"),
            Expr::Size(operand) => write!(f, "size({operand})"),
            Expr::All {
                range,
                var,
                predicate,
            } => write!(f, "{range}.all({var}, {predicate})"),
            Expr::Exists {
                range,
                var,
                predicate,
            } => write!(f, "{range}.exists({var}, {predicate})"),
        }
    }
}

/// An expression of a workflow with the text it was written as.
#[derive(Clone, Debug, PartialEq)]
pub struct Expression {
    source: String,
    expr: Expr,
}

impl Expression {
    pub fn parse(source: &str) -> Result<Expression, String> {
        Ok(Expression {
            source: source.to_owned(),
            expr: parse::parse(source)?,
        })
    }

    pub fn of(expr: Expr) -> Expression {
        Expression {
            source: expr.to_string(),
            expr,
        }
    }

    pub fn expr(&self) -> &Expr {
        &self.expr
    }

    pub fn source(&self) -> &str {
        &self.source
    }

    pub fn roots(&self) -> BTreeSet<Name> {
        self.expr.roots()
    }

    /// The expression with its free identifiers replaced (see
    /// [`Expr::substituted`]); unchanged text when nothing is replaced.
    pub fn substituted(&self, with: &dyn Fn(&Name) -> Option<Expr>) -> Expression {
        let expr = self.expr.substituted(with);
        if expr == self.expr {
            return self.clone();
        }
        Expression::of(expr)
    }
}

/// An expression writes the text it was written as.
impl Serialize for Expression {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&self.source)
    }
}

impl<'de> Deserialize<'de> for Expression {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Expression, D::Error> {
        let source = String::deserialize(deserializer)?;
        Expression::parse(&source).map_err(serde::de::Error::custom)
    }
}

/// One step of a path into a value: a field or an index.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PathStep {
    Field(String),
    Index(u64),
}

/// A path that a `$ref` reads: a name, then fields and indexes, such as
/// `found.item_ids[0]`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RefPath {
    pub root: Name,
    pub steps: Vec<PathStep>,
}

impl RefPath {
    pub fn parse(text: &str) -> Result<RefPath, String> {
        let not_a_path =
            || format!("`{text}` is no path: write a name, then `.field` and `[index]` steps");
        let mut steps = Vec::new();
        let mut expr = parse::parse(text).map_err(|_| not_a_path())?;
        loop {
            match expr {
                Expr::Ident(root) => {
                    steps.reverse();
                    return Ok(RefPath { root, steps });
                }
                Expr::Select(operand, field) => {
                    steps.push(PathStep::Field(field));
                    expr = *operand;
                }
                Expr::Index(operand, index) => {
                    let index = match *index {
                        Expr::Literal(Literal::Int(i)) if i >= 0 => i as u64,
                        Expr::Literal(Literal::UInt(u)) => u,
                        _ => return Err(not_a_path()),
                    };
                    steps.push(PathStep::Index(index));
                    expr = *operand;
                }
                _ => return Err(not_a_path()),
            }
        }
    }

    /// The path as an expression, so it can be typed and read as one.
    fn to_expr(&self) -> Expr {
        self.steps
            .iter()
            .fold(Expr::Ident(self.root.clone()), |expr, step| match step {
                PathStep::Field(field) => Expr::Select(Box::new(expr), field.clone()),
                PathStep::Index(index) => Expr::Index(
                    Box::new(expr),
                    Box::new(Expr::Literal(Literal::Int(*index as i64))),
                ),
            })
    }
}

impl fmt::Display for RefPath {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.to_expr())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_path_parses_into_its_root_and_steps_and_prints_back() {
        let path = RefPath::parse("found.item_ids[2].x").unwrap();
        assert_eq!(path.root.as_str(), "found");
        assert_eq!(
            path.steps,
            vec![
                PathStep::Field("item_ids".into()),
                PathStep::Index(2),
                PathStep::Field("x".into())
            ]
        );
        assert_eq!(path.to_string(), "found.item_ids[2].x");
        assert!(RefPath::parse("a == b").is_err());
        assert!(RefPath::parse("a[b]").is_err());
    }

    /// A literal prints as text that parses back to it: a double as a double
    /// whatever its size, and a string with each control character escaped.
    #[test]
    fn a_literal_prints_as_text_that_parses_back_to_it() {
        for source in [
            "x == 1e15",
            "x == 1e300",
            "x == 0.5",
            "x == 2.0",
            r"x == 'a\nb\tc\u0001d\\e\'f'",
        ] {
            let expression = Expression::parse(source).unwrap();
            let printed = expression.expr().to_string();
            let again = Expression::parse(&printed).unwrap();
            assert_eq!(
                again.expr(),
                expression.expr(),
                "{source} printed {printed}"
            );
        }
    }

    /// A refusal names the place of the refused part in characters, as the
    /// writer of the expression counts them.
    #[test]
    fn a_refusal_names_the_character_where_the_refused_part_starts() {
        let at = |source: &str| {
            let refusal = Expression::parse(source).unwrap_err();
            let (_, rest) = refusal.split_once(" at character ").expect(&refusal);
            rest.split(':').next().unwrap().to_owned()
        };
        assert_eq!(at("'é' == x && y + 1 > 0"), at("'e' == x && y + 1 > 0"));
    }

    #[test]
    fn a_negation_past_the_range_of_an_integer_is_refused() {
        assert!(Expression::parse("-(-9223372036854775808)").is_err());
        assert!(Expression::parse("-(-9223372036854775807) == 9223372036854775807").is_ok());
    }

    #[test]
    fn the_roots_leave_out_the_names_a_comprehension_binds() {
        let expression = Expression::parse("found.ids.all(x, x in allowed) && n > 0").unwrap();
        let roots: Vec<String> = expression.roots().iter().map(|n| n.to_string()).collect();
        assert_eq!(roots, vec!["allowed", "found", "n"]);
    }

    #[test]
    fn a_substitution_replaces_free_names_only() {
        let expression = Expression::parse("state.count >= 6 && items.all(state, state)").unwrap();
        let state = Name::parse("state").unwrap();
        let replaced = expression.substituted(&|name| {
            (name == &state).then(|| Expr::Ident(Name::parse("state_a").unwrap()))
        });
        assert_eq!(
            replaced.source(),
            "((state_a.count >= 6) && items.all(state, state))"
        );
    }
}
