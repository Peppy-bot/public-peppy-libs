use serde_json::Value;
use std::collections::BTreeMap;
use std::num::NonZeroU32;

use super::object::{Fields, read_value};
use crate::expression::{Expr, Expression, Literal, RefPath};
use crate::names::{Bound, InstanceName, MemberRef, Name, StepId};
use crate::refusal::{At, Refusal};

/// One element of a lane.
#[derive(Clone, Debug, PartialEq)]
pub struct Step {
    pub id: StepId,
    /// The step's `timeout_s`: its bound on the run clock, none for a step
    /// without one. The parser refuses a `wait_until` or a `wait_for` with
    /// none.
    pub timeout: Option<Bound>,
    /// New attempts after a retryable failure.
    pub retry: u32,
    /// Evaluated at each poll tick of the step; reads topic observations.
    pub invariant: Option<Expression>,
    pub kind: StepKind,
}

/// How `parallel` ends: at the success of every child, or of the first.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Policy {
    All,
    Any,
}

/// The closed set of step kinds.
#[derive(Clone, Debug, PartialEq)]
pub enum StepKind {
    Call(Call),
    Sequence(Vec<Step>),
    Parallel {
        policy: Policy,
        steps: Vec<Step>,
    },
    If {
        cond: Expression,
        then: Box<Step>,
        otherwise: Option<Box<Step>>,
    },
    Match {
        on: Expression,
        cases: BTreeMap<String, Step>,
        otherwise: Option<Box<Step>>,
    },
    ForEach {
        list: Expression,
        element: Name,
        max_items: NonZeroU32,
        body: Box<Step>,
    },
    RepeatUntil {
        until: Expression,
        fail_when: Option<Expression>,
        max_iterations: NonZeroU32,
        /// The `max_failures` of the breaker: failed passes that open it.
        breaker: Option<NonZeroU32>,
        body: Box<Step>,
    },
    WaitUntil {
        until: Expression,
        fail_when: Option<Expression>,
        poll: Option<Bound>,
    },
    Signal(Name),
    WaitFor {
        signals: Vec<Name>,
        result: Option<Name>,
    },
}

/// A call of a member on the robot of the role.
#[derive(Clone, Debug, PartialEq)]
pub struct Call {
    pub member: MemberRef,
    /// The instance within the robot, when it has several of the contract.
    pub instance: Option<InstanceName>,
    pub args: BTreeMap<String, Arg>,
    /// The name of the result.
    pub result: Option<Name>,
    /// False after a result is a failed attempt.
    pub done_when: Option<Expression>,
}

/// A call of a `safe_state`: a member with its arguments and its own bound.
#[derive(Clone, Debug, PartialEq)]
pub struct SafeCall {
    pub id: StepId,
    pub timeout: Bound,
    pub member: MemberRef,
    pub instance: Option<InstanceName>,
    pub args: BTreeMap<String, Arg>,
}

/// An argument of a call or an observation.
#[derive(Clone, Debug, PartialEq)]
pub enum Arg {
    Literal(Value),
    Object(BTreeMap<String, Arg>),
    List(Vec<Arg>),
    /// `{ $ref: "name.path" }`: a named result, an observation or a loop
    /// element.
    Ref(RefPath),
    /// `{ $table: "table", key: <arg> }`: the row of the table that the key
    /// selects.
    Table {
        table: Name,
        key: Box<Arg>,
    },
    /// `{ $location: "name" }`: a declared location, which goes to a field
    /// with the mark `location`.
    Location(Name),
}

/// The value a lane group gives a parameter for one role.
#[derive(Clone, Debug, PartialEq)]
pub enum Param {
    Literal(Value),
    Observation(Name),
    Signal(Name),
    Location(Name),
}

/// The step kinds, each named by its key.
const KIND_KEYS: [&str; 10] = [
    "call",
    "sequence",
    "parallel",
    "if",
    "match",
    "for_each",
    "repeat_until",
    "wait_until",
    "signal",
    "wait_for",
];

/// Reads the steps of one role: its role, the parameters a lane group gives
/// it, and the refusals found so far.
pub(super) struct StepReader<'a> {
    pub(super) role: &'a Name,
    pub(super) params: Option<&'a BTreeMap<Name, Param>>,
    pub(super) refusals: &'a mut Vec<Refusal>,
}

impl StepReader<'_> {
    fn refuse(&mut self, refusal: Refusal) {
        self.refusals.push(refusal);
    }

    /// Refuses each field of `fields` that no rule read, at `at`: whether
    /// one was.
    fn refuse_unknown(&mut self, at: &At, fields: &Fields) -> bool {
        let unknown = fields.unknown();
        let refused = !unknown.is_empty();
        unknown
            .into_iter()
            .for_each(|refusal| self.refuse(Refusal::new(at.clone(), refusal.reason)));
        refused
    }

    fn at_step(&self, id: &StepId) -> At {
        At::step(self.role, id)
    }

    /// The step at `path`, or `None` once its refusals are recorded.
    pub(super) fn step(&mut self, value: &Value, path: &str) -> Option<Step> {
        let fields = match Fields::of(value, path) {
            Ok(fields) => fields,
            Err(refusal) => {
                self.refuse(refusal);
                return None;
            }
        };
        let id: StepId = match fields.read_required("id") {
            Ok(id) => id,
            Err(refusal) => {
                self.refuse(refusal);
                return None;
            }
        };
        let at = self.at_step(&id);
        let kinds: Vec<&str> = KIND_KEYS
            .iter()
            .copied()
            .filter(|key| fields.has(key))
            .collect();
        let [kind] = kinds[..] else {
            self.refuse(Refusal::new(
                at,
                format!(
                    "a step has one kind key of {}; this one has {}",
                    KIND_KEYS.join(", "),
                    if kinds.is_empty() {
                        "none".to_owned()
                    } else {
                        kinds.join(" and ")
                    }
                ),
            ));
            return None;
        };
        let timeout = self.field(&at, fields.read::<Bound>("timeout_s"))?;
        let retry = self.field(&at, fields.read::<u32>("retry"))?.unwrap_or(0);
        let invariant = self.expression(&at, fields.get("invariant"), "invariant")?;
        let kind = self.kind(kind, &fields, &at, &id)?;
        let needs_timeout = matches!(kind, StepKind::WaitUntil { .. } | StepKind::WaitFor { .. });
        if needs_timeout && timeout.is_none() {
            self.refuse(Refusal::new(at.clone(), "a wait has its own `timeout_s`"));
        }
        let refused = self.refuse_unknown(&at, &fields);
        (!refused).then_some(Step {
            id,
            timeout,
            retry,
            invariant,
            kind,
        })
    }

    fn kind(&mut self, kind: &str, fields: &Fields, at: &At, id: &StepId) -> Option<StepKind> {
        let child = |reader: &mut Self, key: &str| -> Option<Box<Step>> {
            let value = reader.field(at, fields.require(key))?;
            reader.step(value, &fields.at(key)).map(Box::new)
        };
        let optional_child = |reader: &mut Self, key: &str| -> Option<Option<Box<Step>>> {
            match fields.get(key) {
                None => Some(None),
                Some(value) => reader
                    .step(value, &fields.at(key))
                    .map(|step| Some(Box::new(step))),
            }
        };
        match kind {
            "call" => self
                .call(fields.get("call")?, &fields.at("call"), at)
                .map(StepKind::Call),
            "sequence" => self
                .steps(fields.get("sequence")?, &fields.at("sequence"), at)
                .map(StepKind::Sequence),
            "parallel" => {
                let steps = self.steps(fields.get("parallel")?, &fields.at("parallel"), at)?;
                let policy = self.field(at, fields.read_required::<Policy>("policy"))?;
                Some(StepKind::Parallel { policy, steps })
            }
            "if" => {
                let cond = self.expression(at, fields.get("if"), "if")??;
                let then = child(self, "then")?;
                let otherwise = optional_child(self, "else")?;
                Some(StepKind::If {
                    cond,
                    then,
                    otherwise,
                })
            }
            "match" => {
                let on = self.expression(at, fields.get("match"), "match")??;
                let cases_value = self.field(at, fields.require("cases"))?;
                let Value::Object(cases_map) = cases_value else {
                    self.refuse(Refusal::new(
                        at.clone(),
                        "`cases` maps each value to a step",
                    ));
                    return None;
                };
                let cases: Option<BTreeMap<String, Step>> = cases_map
                    .iter()
                    .map(|(value, step)| {
                        self.step(step, &format!("{}.{value}", fields.at("cases")))
                            .map(|step| (value.clone(), step))
                    })
                    .collect::<Vec<_>>()
                    .into_iter()
                    .collect();
                let otherwise = optional_child(self, "otherwise")?;
                Some(StepKind::Match {
                    on,
                    cases: cases?,
                    otherwise,
                })
            }
            "for_each" => {
                let spec = self.field(
                    at,
                    Fields::of(fields.get("for_each")?, &fields.at("for_each")),
                )?;
                let list = self.expression(at, spec.get("list"), "for_each.list")??;
                let element = self.field(at, spec.read_required::<Name>("as"))?;
                self.no_param_named(at, Some(&element));
                self.refuse_unknown(at, &spec);
                let max_items = self.field(at, fields.read_required::<NonZeroU32>("max_items"))?;
                let body = child(self, "body")?;
                Some(StepKind::ForEach {
                    list,
                    element,
                    max_items,
                    body,
                })
            }
            "repeat_until" => {
                let until = self.expression(at, fields.get("repeat_until"), "repeat_until")??;
                let fail_when = self.expression(at, fields.get("fail_when"), "fail_when")?;
                let max_iterations =
                    self.field(at, fields.read_required::<NonZeroU32>("max_iterations"))?;
                let breaker = match fields.get("breaker") {
                    None => None,
                    Some(value) => {
                        let spec = self.field(at, Fields::of(value, &fields.at("breaker")))?;
                        let max =
                            self.field(at, spec.read_required::<NonZeroU32>("max_failures"))?;
                        self.refuse_unknown(at, &spec);
                        Some(max)
                    }
                };
                let body = child(self, "body")?;
                Some(StepKind::RepeatUntil {
                    until,
                    fail_when,
                    max_iterations,
                    breaker,
                    body,
                })
            }
            "wait_until" => {
                let until = self.expression(at, fields.get("wait_until"), "wait_until")??;
                let fail_when = self.expression(at, fields.get("fail_when"), "fail_when")?;
                let poll = self.field(at, fields.read::<Bound>("poll_s"))?;
                Some(StepKind::WaitUntil {
                    until,
                    fail_when,
                    poll,
                })
            }
            "signal" => self.signal(fields.get("signal")?, at).map(StepKind::Signal),
            "wait_for" => {
                let signals = self.field(at, fields.read_required::<Vec<Name>>("wait_for"))?;
                if signals.is_empty() {
                    self.refuse(Refusal::new(
                        at.clone(),
                        "`wait_for` names at least one signal",
                    ));
                    return None;
                }
                let result = self.field(at, fields.read::<Name>("as"))?;
                self.no_param_named(at, result.as_ref());
                Some(StepKind::WaitFor { signals, result })
            }
            _ => unreachable!("{id}: the kind keys are {KIND_KEYS:?}"),
        }
    }

    fn steps(&mut self, value: &Value, path: &str, at: &At) -> Option<Vec<Step>> {
        let Value::Array(items) = value else {
            self.refuse(Refusal::new(
                at.clone(),
                format!("`{path}` is a list of steps"),
            ));
            return None;
        };
        if items.is_empty() {
            self.refuse(Refusal::new(
                at.clone(),
                format!("`{path}` holds at least one step"),
            ));
            return None;
        }
        let steps: Vec<Option<Step>> = items
            .iter()
            .enumerate()
            .map(|(index, item)| self.step(item, &format!("{path}[{index}]")))
            .collect();
        steps.into_iter().collect()
    }

    fn signal(&mut self, value: &Value, at: &At) -> Option<Name> {
        if let Some(param) = value.get("$param") {
            let name: Name = self.field(at, read_value(param, "signal.$param"))?;
            return match self.param(&name, at)? {
                Param::Signal(signal) => Some(signal.clone()),
                _ => {
                    self.refuse(Refusal::new(
                        at.clone(),
                        format!(
                            "the parameter `{name}` of a `signal` step holds a `{{ $signal }}`"
                        ),
                    ));
                    None
                }
            };
        }
        self.field(at, read_value(value, "signal"))
    }

    pub(super) fn call(&mut self, value: &Value, path: &str, at: &At) -> Option<Call> {
        let fields = self.field(at, Fields::of(value, path))?;
        let member = self.field(at, fields.read_required::<MemberRef>("member"))?;
        let instance = self.field(at, fields.read::<InstanceName>("instance"))?;
        let args = self.args(fields.get("args"), &fields.at("args"), at)?;
        let result = self.field(at, fields.read::<Name>("as"))?;
        self.no_param_named(at, result.as_ref());
        let done_when = self.expression(at, fields.get("done_when"), "call.done_when")?;
        let refused = self.refuse_unknown(at, &fields);
        (!refused).then_some(Call {
            member,
            instance,
            args,
            result,
            done_when,
        })
    }

    pub(super) fn args(
        &mut self,
        value: Option<&Value>,
        path: &str,
        at: &At,
    ) -> Option<BTreeMap<String, Arg>> {
        let Some(value) = value else {
            return Some(BTreeMap::new());
        };
        match self.arg(value, path, at)? {
            Arg::Object(args) => Some(args),
            _ => {
                self.refuse(Refusal::new(
                    at.clone(),
                    format!("`{path}` is an object of arguments"),
                ));
                None
            }
        }
    }

    fn arg(&mut self, value: &Value, path: &str, at: &At) -> Option<Arg> {
        match value {
            Value::Null => {
                self.refuse(Refusal::new(
                    at.clone(),
                    format!("`{path}` is null; leave the field out"),
                ));
                None
            }
            Value::Array(items) => {
                let items: Vec<Option<Arg>> = items
                    .iter()
                    .enumerate()
                    .map(|(index, item)| self.arg(item, &format!("{path}[{index}]"), at))
                    .collect();
                items.into_iter().collect::<Option<_>>().map(Arg::List)
            }
            Value::Object(map) if map.keys().any(|key| key.starts_with('$')) => {
                self.reference(value, path, at)
            }
            Value::Object(map) => {
                let fields: Vec<Option<(String, Arg)>> = map
                    .iter()
                    .map(|(key, item)| {
                        self.arg(item, &format!("{path}.{key}"), at)
                            .map(|arg| (key.clone(), arg))
                    })
                    .collect();
                fields.into_iter().collect::<Option<_>>().map(Arg::Object)
            }
            scalar => Some(Arg::Literal(scalar.clone())),
        }
    }

    fn reference(&mut self, value: &Value, path: &str, at: &At) -> Option<Arg> {
        let fields = self.field(at, Fields::of(value, path))?;
        let arg = if let Some(path_value) = fields.get("$ref") {
            let text: String = self.field(at, read_value(path_value, &fields.at("$ref")))?;
            let path = match RefPath::parse(&text) {
                Ok(path) => path,
                Err(reason) => {
                    self.refuse(Refusal::new(at.clone(), reason));
                    return None;
                }
            };
            self.resolved_ref(path, at)?
        } else if let Some(table) = fields.get("$table") {
            let table: Name = self.field(at, read_value(table, &fields.at("$table")))?;
            let key_value = self.field(at, fields.require("key"))?;
            // The document writes the key as a reference; a `$param` gives
            // the row of each role of a lane group.
            let written = key_value.as_object().is_some_and(|key| {
                key.len() == 1 && (key.contains_key("$ref") || key.contains_key("$param"))
            });
            if !written {
                self.refuse(Refusal::new(
                    at.clone(),
                    format!("the key of the table `{table}` is a `$ref` or a `$param`"),
                ));
                return None;
            }
            let key = self.arg(key_value, &fields.at("key"), at)?;
            if !matches!(key, Arg::Ref(_) | Arg::Literal(Value::String(_))) {
                self.refuse(Refusal::new(
                    at.clone(),
                    format!("the key of the table `{table}` is a `$ref` or a `$param`"),
                ));
                return None;
            }
            Arg::Table {
                table,
                key: Box::new(key),
            }
        } else if let Some(location) = fields.get("$location") {
            let name: Name = self.field(at, read_value(location, &fields.at("$location")))?;
            Arg::Location(name)
        } else if let Some(param) = fields.get("$param") {
            let name: Name = self.field(at, read_value(param, &fields.at("$param")))?;
            match self.param(&name, at)? {
                Param::Literal(value) => Arg::Literal(value.clone()),
                Param::Observation(observation) => Arg::Ref(RefPath {
                    root: observation.clone(),
                    steps: Vec::new(),
                }),
                Param::Location(location) => Arg::Location(location.clone()),
                Param::Signal(_) => {
                    self.refuse(Refusal::new(
                        at.clone(),
                        format!("the parameter `{name}` holds a signal, which goes only to a `signal` step"),
                    ));
                    return None;
                }
            }
        } else {
            self.refuse(Refusal::new(
                at.clone(),
                format!(
                    "`{path}` is a reference: `{{ $ref }}`, `{{ $table, key }}`, `{{ $location }}` or `{{ $param }}`"
                ),
            ));
            return None;
        };
        let refused = self.refuse_unknown(at, &fields);
        (!refused).then_some(arg)
    }

    /// A `$ref` whose root may be a parameter: an observation parameter
    /// names the observation, and a literal parameter stands for its value
    /// when the path has no step.
    fn resolved_ref(&mut self, path: RefPath, at: &At) -> Option<Arg> {
        let Some(param) = self.params.and_then(|params| params.get(&path.root)) else {
            return Some(Arg::Ref(path));
        };
        match param {
            Param::Observation(observation) => Some(Arg::Ref(RefPath {
                root: observation.clone(),
                steps: path.steps,
            })),
            Param::Literal(value) if path.steps.is_empty() => Some(Arg::Literal(value.clone())),
            _ => {
                self.refuse(Refusal::new(
                    at.clone(),
                    format!(
                        "`{path}` reads into the parameter `{}`, which holds no observation",
                        path.root
                    ),
                ));
                None
            }
        }
    }

    fn param(&mut self, name: &Name, at: &At) -> Option<&Param> {
        let Some(params) = self.params else {
            self.refuse(Refusal::new(
                at.clone(),
                format!("`$param: {name}` is used only in the tree of a lane group"),
            ));
            return None;
        };
        let found = params.get(name);
        if found.is_none() {
            self.refuse(Refusal::new(
                at.clone(),
                format!("the role `{}` gives no parameter `{name}`", self.role),
            ));
        }
        found
    }

    /// The expression of `value`, with each parameter of the role put in;
    /// `Some(None)` when there is none.
    pub(super) fn expression(
        &mut self,
        at: &At,
        value: Option<&Value>,
        field: &str,
    ) -> Option<Option<Expression>> {
        let Some(value) = value else {
            return Some(None);
        };
        let Value::String(source) = value else {
            self.refuse(Refusal::new(
                at.clone(),
                format!("`{field}` is an expression, written as a string"),
            ));
            return None;
        };
        let expression = match Expression::parse(source) {
            Ok(expression) => expression,
            Err(reason) => {
                self.refuse(Refusal::new(at.clone(), format!("`{field}`: {reason}")));
                return None;
            }
        };
        let Some(params) = self.params else {
            return Some(Some(expression));
        };
        let mut refused = None;
        let substituted = expression.substituted(&|name| match params.get(name)? {
            Param::Observation(observation) => Some(Expr::Ident(observation.clone())),
            Param::Literal(value) => literal_expr(value),
            Param::Signal(_) | Param::Location(_) => None,
        });
        for root in substituted.roots() {
            match params.get(&root) {
                Some(Param::Signal(_)) => {
                    refused = Some(format!(
                        "`{field}` reads the parameter `{root}`, which holds a signal"
                    ))
                }
                Some(Param::Location(_)) => {
                    refused = Some(format!(
                        "`{field}` reads the parameter `{root}`, which holds a location"
                    ))
                }
                Some(Param::Literal(_)) => {
                    refused = Some(format!(
                        "`{field}` reads the parameter `{root}`, whose value is no literal of an expression"
                    ))
                }
                _ => {}
            }
        }
        if let Some(reason) = refused {
            self.refuse(Refusal::new(at.clone(), reason));
            return None;
        }
        Some(Some(substituted))
    }

    /// Refuses a local value named like a parameter of the lane group: the
    /// locals and the parameters of a lane share one namespace.
    fn no_param_named(&mut self, at: &At, local: Option<&Name>) {
        let Some(local) = local.filter(|local| {
            self.params
                .is_some_and(|params| params.contains_key(*local))
        }) else {
            return;
        };
        self.refuse(Refusal::new(
            at.clone(),
            format!("`{local}` is a parameter of the lane group; give this value another name"),
        ));
    }

    /// The value of a field read with serde, or `None` once its refusal is
    /// recorded at `at`.
    fn field<T>(&mut self, at: &At, read: Result<T, Refusal>) -> Option<T> {
        match read {
            Ok(value) => Some(value),
            Err(refusal) => {
                let reason = match &refusal.at {
                    At::Field(path) if !path.is_empty() => format!("`{path}` {}", refusal.reason),
                    _ => refusal.reason,
                };
                self.refuse(Refusal::new(at.clone(), reason));
                None
            }
        }
    }
}

/// A JSON literal as an expression literal: a bool, a number, a string, or a
/// list of those.
fn literal_expr(value: &Value) -> Option<Expr> {
    match value {
        Value::Bool(b) => Some(Expr::Literal(Literal::Bool(*b))),
        Value::Number(n) => Some(Expr::Literal(match (n.as_i64(), n.as_f64()) {
            (Some(i), _) => Literal::Int(i),
            (None, Some(d)) => Literal::Double(d),
            (None, None) => return None,
        })),
        Value::String(s) => Some(Expr::Literal(Literal::String(s.clone()))),
        Value::Array(items) => items
            .iter()
            .map(literal_expr)
            .collect::<Option<_>>()
            .map(Expr::List),
        Value::Null | Value::Object(_) => None,
    }
}
