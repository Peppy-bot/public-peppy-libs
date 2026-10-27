use cel::common::ast::{Expr as CelExpr, IdedExpr, LiteralValue, SourceInfo, operators};

use super::{CompareOp, Expr, Literal};
use crate::names::Name;

/// The deepest nesting an expression of a workflow takes.
const MAX_RECURSION_DEPTH: u16 = 64;

/// What an expression of a workflow holds, as a refusal says it.
const SUBSET: &str = "an expression of a workflow holds literals, names, fields, indexes, `!`, `&&`, `||`, comparisons, `in`, `size()`, `all()` and `exists()`";

/// Parses CEL text into the subset, with no macro expanded and no optional
/// syntax: a form outside the subset is refused with its place in the text.
pub(super) fn parse(source: &str) -> Result<Expr, String> {
    let (expr, info) = cel::Env::default()
        .with_optional_support(false)
        .parser()
        .max_recursion_depth(MAX_RECURSION_DEPTH)
        .parse_with_source_info(source)
        .map_err(|errors| format!("the expression `{source}` does not parse: {errors}"))?;
    Converter { source, info }.convert(&expr)
}

struct Converter<'a> {
    source: &'a str,
    info: SourceInfo,
}

impl Converter<'_> {
    fn refuse(&self, at: &IdedExpr, what: &str) -> String {
        // The parser places an expression by its first byte; the user reads
        // characters.
        let place = self
            .info
            .offset_for(at.id)
            .map(|(start, _)| {
                let start = start as usize;
                let before = self
                    .source
                    .get(..start)
                    .map_or(start, |text| text.chars().count());
                format!(" at character {}", before + 1)
            })
            .unwrap_or_default();
        format!("the expression `{}`{place}: {what}; {SUBSET}", self.source)
    }

    fn boxed(&self, expr: &IdedExpr) -> Result<Box<Expr>, String> {
        self.convert(expr).map(Box::new)
    }

    fn convert(&self, ided: &IdedExpr) -> Result<Expr, String> {
        match &ided.expr {
            CelExpr::Literal(literal) => self.literal(ided, literal),
            CelExpr::Ident(name) => Name::parse(name)
                .map(Expr::Ident)
                .map_err(|_| self.refuse(ided, &format!("`{name}` is no name of a workflow"))),
            CelExpr::Select(select) if select.test => {
                Err(self.refuse(ided, "`has()` is not in the subset"))
            }
            CelExpr::Select(select) => Ok(Expr::Select(
                self.boxed(&select.operand)?,
                select.field.clone(),
            )),
            CelExpr::List(list) => list
                .elements
                .iter()
                .map(|element| self.convert(element))
                .collect::<Result<_, _>>()
                .map(Expr::List),
            CelExpr::Call(call) => self.call(ided, call),
            CelExpr::Map(_) | CelExpr::Struct(_) => {
                Err(self.refuse(ided, "maps and messages are not in the subset"))
            }
            CelExpr::Comprehension(_) | CelExpr::Unspecified => {
                Err(self.refuse(ided, "this form is not in the subset"))
            }
        }
    }

    fn literal(&self, at: &IdedExpr, literal: &LiteralValue) -> Result<Expr, String> {
        let literal = match literal {
            LiteralValue::Boolean(b) => Literal::Bool(*b.inner()),
            LiteralValue::Int(i) => Literal::Int(*i.inner()),
            LiteralValue::UInt(u) => Literal::UInt(*u.inner()),
            LiteralValue::Double(d) => Literal::Double(*d.inner()),
            LiteralValue::String(s) => Literal::String(s.inner().to_owned()),
            LiteralValue::Bytes(_) => return Err(self.refuse(at, "bytes are not in the subset")),
            LiteralValue::Null => return Err(self.refuse(at, "null is not in the subset")),
        };
        Ok(Expr::Literal(literal))
    }

    fn call(&self, at: &IdedExpr, call: &cel::common::ast::CallExpr) -> Result<Expr, String> {
        let name = call.func_name.as_str();
        let args = &call.args;
        let compare = |op| -> Result<Expr, String> {
            Ok(Expr::Compare(
                op,
                self.boxed(&args[0])?,
                self.boxed(&args[1])?,
            ))
        };
        match (name, call.target.as_deref(), args.len()) {
            (operators::LOGICAL_AND, None, 2) => {
                Ok(Expr::And(self.boxed(&args[0])?, self.boxed(&args[1])?))
            }
            (operators::LOGICAL_OR, None, 2) => {
                Ok(Expr::Or(self.boxed(&args[0])?, self.boxed(&args[1])?))
            }
            (operators::LOGICAL_NOT, None, 1) => Ok(Expr::Not(self.boxed(&args[0])?)),
            (operators::EQUALS, None, 2) => compare(CompareOp::Eq),
            (operators::NOT_EQUALS, None, 2) => compare(CompareOp::Ne),
            (operators::LESS, None, 2) => compare(CompareOp::Lt),
            (operators::LESS_EQUALS, None, 2) => compare(CompareOp::Le),
            (operators::GREATER, None, 2) => compare(CompareOp::Gt),
            (operators::GREATER_EQUALS, None, 2) => compare(CompareOp::Ge),
            (operators::IN, None, 2) => Ok(Expr::In(self.boxed(&args[0])?, self.boxed(&args[1])?)),
            (operators::INDEX, None, 2) => {
                Ok(Expr::Index(self.boxed(&args[0])?, self.boxed(&args[1])?))
            }
            (operators::NEGATE, None, 1) => match self.convert(&args[0])? {
                Expr::Literal(Literal::Int(i)) => match i.checked_neg() {
                    Some(negated) => Ok(Expr::Literal(Literal::Int(negated))),
                    None => {
                        Err(self.refuse(at, "the negated number is out of the range of an integer"))
                    }
                },
                Expr::Literal(Literal::Double(d)) => Ok(Expr::Literal(Literal::Double(-d))),
                Expr::Literal(Literal::UInt(_)) => Err(self.refuse(
                    at,
                    "`-` negates no unsigned integer; write the value as an integer, without `u`",
                )),
                _ => Err(self.refuse(at, "`-` negates only an integer or a double literal")),
            },
            ("size", None, 1) => Ok(Expr::Size(self.boxed(&args[0])?)),
            ("size", Some(target), 0) => Ok(Expr::Size(self.boxed(target)?)),
            (operators::ALL | operators::EXISTS, Some(target), 2) => {
                let CelExpr::Ident(var) = &args[0].expr else {
                    return Err(self.refuse(
                        at,
                        "the first argument of `all` or `exists` names the element",
                    ));
                };
                let var = Name::parse(var)
                    .map_err(|_| self.refuse(at, &format!("`{var}` is no name of a workflow")))?;
                let range = self.boxed(target)?;
                let predicate = self.boxed(&args[1])?;
                Ok(match name {
                    operators::ALL => Expr::All {
                        range,
                        var,
                        predicate,
                    },
                    _ => Expr::Exists {
                        range,
                        var,
                        predicate,
                    },
                })
            }
            _ => Err(self.refuse(at, &format!("`{}` is not in the subset", shown(name)))),
        }
    }
}

/// The name of a call as a person writes it.
fn shown(name: &str) -> &str {
    match name {
        operators::ADD => "+",
        operators::SUBSTRACT => "-",
        operators::MULTIPLY => "*",
        operators::DIVIDE => "/",
        operators::MODULO => "%",
        operators::CONDITIONAL => "?:",
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_subset_parses_and_prints_back_to_the_same_tree() {
        for source in [
            "books.count == 0",
            "found && crate.reachable",
            "!(a < 1) || b >= -2.5",
            "sorted.item_kind == 'book'",
            "x in ['a', 'b']",
            "size(found.ids) <= 20 && found.ids.size() > 0",
            "items.all(i, i.ok) && items.exists(i, i.count > 1u)",
            "rows[0].value != 'x'",
        ] {
            let expr = parse(source).unwrap();
            assert_eq!(parse(&expr.to_string()).unwrap(), expr, "{source}");
        }
    }

    #[test]
    fn a_negative_literal_is_a_literal() {
        assert_eq!(parse("-1").unwrap(), Expr::Literal(Literal::Int(-1)));
        assert_eq!(parse("--2").unwrap(), Expr::Literal(Literal::Int(2)));
    }

    #[test]
    fn forms_outside_the_subset_are_refused_by_name() {
        for (source, named) in [
            ("a + 1", "`+`"),
            ("a ? b : c", "`?:`"),
            ("has(a.b)", "`has`"),
            ("a.map(x, x)", "`map`"),
            ("{'a': 1}", "maps"),
            ("null", "null"),
            ("b'x'", "bytes"),
            ("-a", "negates only"),
            ("-(1u)", "negates no unsigned integer"),
            ("Upper", "no name of a workflow"),
            ("f(a)", "`f`"),
        ] {
            let refusal = parse(source).unwrap_err();
            assert!(refusal.contains(named), "{source}: {refusal}");
        }
        assert!(parse("a.?b").is_err());
        assert!(parse("a +").unwrap_err().contains("does not parse"));
    }
}
