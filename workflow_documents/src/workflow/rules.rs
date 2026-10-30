//! The rules of a workflow that need no contract: the roles and their lanes,
//! one namespace for the global names, unique step ids and local names per
//! role, the names each argument and expression may read, a bound on each
//! call, and signals that some other lane emits with no blocking cycle.

use std::collections::{BTreeMap, BTreeSet};

use super::{Arg, ObservationSource, Step, StepKind, Workflow};
use crate::expression::Expression;
use crate::names::{MemberRef, Name, StepId};
use crate::refusal::{At, Refusal};
use serde_json::Value;

/// The refusals of `workflow`.
pub(super) fn check(workflow: &Workflow) -> Vec<Refusal> {
    let mut refusals = Vec::new();
    roles_and_lanes(workflow, &mut refusals);
    aliases(workflow, &mut refusals);
    let signals = signal_names(workflow);
    namespace(workflow, &signals, &mut refusals);
    locations(workflow, &mut refusals);
    let globals = Globals::of(workflow);
    expressions_of_the_workflow(workflow, &globals, &mut refusals);
    let mut lanes_of_signals = SignalUse::default();
    for (role, lane) in &workflow.lanes {
        let mut walker = Walker {
            role,
            globals: &globals,
            refusals: &mut refusals,
            ids: BTreeSet::new(),
            locals: BTreeSet::new(),
            in_initialization: false,
        };
        let defined = match workflow.initialization.get(role) {
            Some(tree) => {
                walker.in_initialization = true;
                let defined = walker.walk(tree, BTreeSet::new(), false);
                walker.in_initialization = false;
                defined
            }
            None => BTreeSet::new(),
        };
        walker.walk(&lane.tree, defined, false);
        lanes_of_signals.add_lane(role, &lane.tree);
        for call in &lane.safe_state {
            walker.id(&call.id);
            walker.args(&call.args, &BTreeSet::new(), &At::step(role, &call.id));
        }
    }
    lanes_of_signals.check(&mut refusals);
    refusals
}

fn roles_and_lanes(workflow: &Workflow, refusals: &mut Vec<Refusal>) {
    for role in workflow.lanes.keys() {
        if !workflow.roles.contains_key(role) {
            refusals.push(Refusal::new(
                At::field(format!("actions.{role}")),
                format!("`{role}` is no role of `roles`; a lane is keyed by its role, a lane group names `roles`"),
            ));
        }
    }
    for role in workflow.roles.keys() {
        if !workflow.lanes.contains_key(role) {
            refusals.push(Refusal::new(
                At::field("actions"),
                format!("the role `{role}` has no lane; give it one, or remove the role"),
            ));
        }
    }
    for role in workflow.initialization.keys() {
        if !workflow.roles.contains_key(role) {
            refusals.push(Refusal::new(
                At::field(format!("initialization.{role}")),
                format!("`{role}` is no role of `roles`"),
            ));
        }
    }
    for (name, observation) in &workflow.observations {
        if !workflow.roles.contains_key(&observation.role) {
            refusals.push(Refusal::new(
                At::field(format!("observations.{name}.role")),
                format!("`{}` is no role of `roles`", observation.role),
            ));
        }
    }
}

/// Each member and topic a workflow names, with where it names it.
fn named_members(workflow: &Workflow) -> Vec<(At, &MemberRef)> {
    let mut found: Vec<(At, &MemberRef)> = workflow
        .observations
        .iter()
        .map(|(name, observation)| {
            (
                At::field(format!("observations.{name}")),
                observation.named(),
            )
        })
        .collect();
    let trees = workflow
        .initialization
        .iter()
        .chain(workflow.lanes.iter().map(|(role, lane)| (role, &lane.tree)));
    for (role, tree) in trees {
        tree.visit(&mut |step| {
            if let StepKind::Call(call) = &step.kind {
                found.push((At::step(role, &step.id), &call.member));
            }
        });
    }
    for (role, lane) in &workflow.lanes {
        for call in &lane.safe_state {
            found.push((At::step(role, &call.id), &call.member));
        }
    }
    found
}

fn aliases(workflow: &Workflow, refusals: &mut Vec<Refusal>) {
    for (at, member) in named_members(workflow) {
        if !workflow.contracts.contains_key(&member.alias) {
            refusals.push(Refusal::new(
                at,
                format!(
                    "`{member}` names the alias `{}`, which `contracts` does not give",
                    member.alias
                ),
            ));
        }
    }
}

/// Each signal that a `signal` or a `wait_for` step names.
fn signal_names(workflow: &Workflow) -> BTreeSet<Name> {
    let mut signals = BTreeSet::new();
    for lane in workflow.lanes.values() {
        lane.tree.visit(&mut |step| match &step.kind {
            StepKind::Signal(signal) => {
                signals.insert(signal.clone());
            }
            StepKind::WaitFor { signals: names, .. } => signals.extend(names.iter().cloned()),
            _ => {}
        });
    }
    signals
}

/// No two aliases, observations, tables, locations or signals share a name.
fn namespace(workflow: &Workflow, signals: &BTreeSet<Name>, refusals: &mut Vec<Refusal>) {
    // Each kind with its article.
    let kinds: [(&str, Vec<&Name>); 5] = [
        ("an alias", workflow.contracts.keys().collect()),
        ("an observation", workflow.observations.keys().collect()),
        ("a table", workflow.tables.keys().collect()),
        ("a location", workflow.locations.iter().collect()),
        ("a signal", signals.iter().collect()),
    ];
    let mut seen: BTreeMap<&Name, &str> = BTreeMap::new();
    for (kind, names) in &kinds {
        for name in names {
            if let Some(earlier) = seen.insert(name, kind) {
                refusals.push(Refusal::new(
                    At::Document,
                    format!("`{name}` names {earlier} and {kind}; aliases, observations, tables, locations and signals share one namespace"),
                ));
            }
        }
    }
}

/// The contract of the locations of a skill, which the applier of worlds
/// teaches and the director lists: no call or observation names a member of
/// it.
const LOCATIONS_CONTRACT: (&str, &str) = ("locations", "v1");

/// Each `$location` of a call, an observation or a table row names a
/// declared location, and no call or observation names a member of
/// locations:v1.
fn locations(workflow: &Workflow, refusals: &mut Vec<Refusal>) {
    let declared = |location: &Name, at: At, refusals: &mut Vec<Refusal>| {
        if !workflow.locations.contains(location) {
            refusals.push(Refusal::new(
                at,
                format!("`$location` names `{location}`, which `locations` does not declare"),
            ));
        }
    };
    for (role, tree) in &workflow.initialization {
        tree.visit(&mut |step| {
            if let StepKind::Call(call) = &step.kind {
                for location in arg_locations(&call.args) {
                    declared(&location, At::step(role, &step.id), refusals);
                }
            }
        });
    }
    for (role, lane) in &workflow.lanes {
        lane.tree.visit(&mut |step| {
            if let StepKind::Call(call) = &step.kind {
                for location in arg_locations(&call.args) {
                    declared(&location, At::step(role, &step.id), refusals);
                }
            }
        });
        for call in &lane.safe_state {
            for location in arg_locations(&call.args) {
                declared(&location, At::step(role, &call.id), refusals);
            }
        }
    }
    for (name, observation) in &workflow.observations {
        if let ObservationSource::Member { args, .. } = &observation.source {
            for location in arg_locations(args) {
                declared(
                    &location,
                    At::field(format!("observations.{name}")),
                    refusals,
                );
            }
        }
    }
    for (table, rows) in &workflow.tables {
        for (key, row) in rows {
            let Some(Value::String(location)) = row.get("$location") else {
                continue;
            };
            match Name::parse(location) {
                Ok(location) => declared(
                    &location,
                    At::field(format!("tables.{table}.{key}")),
                    refusals,
                ),
                Err(reason) => refusals.push(Refusal::new(
                    At::field(format!("tables.{table}.{key}")),
                    format!("`$location`: {reason}"),
                )),
            }
        }
    }
    let (name, tag) = LOCATIONS_CONTRACT;
    for (at, member) in named_members(workflow) {
        let names_locations = workflow
            .contracts
            .get(&member.alias)
            .is_some_and(|contract| contract.name.as_str() == name && contract.tag() == tag);
        if names_locations {
            refusals.push(Refusal::new(
                at,
                format!(
                    "calls `{}.{}` of locations:v1, which only the applier of worlds and the director call",
                    member.alias, member.member
                ),
            ));
        }
    }
}

/// The global names of a workflow, by what each may be read in.
struct Globals {
    observations: BTreeSet<Name>,
    topics: BTreeSet<Name>,
    tables: BTreeSet<Name>,
    all: BTreeSet<Name>,
}

impl Globals {
    fn of(workflow: &Workflow) -> Globals {
        let observations: BTreeSet<Name> = workflow.observations.keys().cloned().collect();
        let topics = workflow
            .observations
            .iter()
            .filter(|(_, observation)| {
                matches!(observation.source, ObservationSource::Topic { .. })
            })
            .map(|(name, _)| name.clone())
            .collect();
        let tables: BTreeSet<Name> = workflow.tables.keys().cloned().collect();
        let all = observations
            .iter()
            .chain(&tables)
            .chain(workflow.contracts.keys())
            .chain(&workflow.locations)
            .chain(&signal_names(workflow))
            .cloned()
            .collect();
        Globals {
            observations,
            topics,
            tables,
            all,
        }
    }
}

/// `success`, `minimum` and the assumption checks read only observations and
/// tables; the arguments of an observation hold no `$ref`.
fn expressions_of_the_workflow(
    workflow: &Workflow,
    globals: &Globals,
    refusals: &mut Vec<Refusal>,
) {
    let allowed: BTreeSet<Name> = globals
        .observations
        .union(&globals.tables)
        .cloned()
        .collect();
    let mut read_only = |expression: &Expression, at: At, field: &str| {
        reads_only(
            expression,
            &allowed,
            at,
            field,
            "observations and tables",
            refusals,
        )
    };
    read_only(&workflow.success, At::field("success"), "success");
    if let Some(minimum) = &workflow.minimum {
        read_only(minimum, At::field("minimum"), "minimum");
    }
    for (name, assumption) in &workflow.assumptions {
        if let Some(check) = &assumption.check {
            read_only(
                check,
                At::field(format!("assumptions.{name}.check")),
                "check",
            );
        }
    }
    for (name, observation) in &workflow.observations {
        if let ObservationSource::Member { args, .. } = &observation.source
            && let Some(path) = arg_refs(args).first()
        {
            refusals.push(Refusal::new(
                At::field(format!("observations.{name}.args")),
                format!("`$ref` reads `{path}`; an observation is read at any instant of a run, so its arguments are literals or locations"),
            ));
        }
    }
}

fn reads_only(
    expression: &Expression,
    allowed: &BTreeSet<Name>,
    at: At,
    field: &str,
    what: &str,
    refusals: &mut Vec<Refusal>,
) {
    for root in expression.roots() {
        if !allowed.contains(&root) {
            refusals.push(Refusal::new(
                at.clone(),
                format!("`{field}` reads `{root}`; it reads only {what}"),
            ));
        }
    }
}

/// Each argument of `args`, in order, before the arguments it holds.
fn each_arg<'a>(args: &'a BTreeMap<String, Arg>, visit: &mut impl FnMut(&'a Arg)) {
    fn walk<'a, F: FnMut(&'a Arg)>(arg: &'a Arg, visit: &mut F) {
        visit(arg);
        match arg {
            Arg::Literal(_) | Arg::Ref(_) | Arg::Location(_) => {}
            Arg::Object(fields) => {
                for field in fields.values() {
                    walk(field, visit);
                }
            }
            Arg::List(items) => {
                for item in items {
                    walk(item, visit);
                }
            }
            Arg::Table { key, .. } => walk(key, visit),
        }
    }
    for arg in args.values() {
        walk(arg, visit);
    }
}

/// The roots of every `$ref` of the arguments, and of every `$table` key.
fn arg_refs(args: &BTreeMap<String, Arg>) -> Vec<Name> {
    let mut found = Vec::new();
    each_arg(args, &mut |arg| {
        if let Arg::Ref(path) = arg {
            found.push(path.root.clone());
        }
    });
    found
}

/// The locations that the `$location` arguments name.
fn arg_locations(args: &BTreeMap<String, Arg>) -> Vec<Name> {
    let mut found = Vec::new();
    each_arg(args, &mut |arg| {
        if let Arg::Location(location) = arg {
            found.push(location.clone());
        }
    });
    found
}

/// The tables that the `$table` arguments name.
fn arg_tables(args: &BTreeMap<String, Arg>) -> Vec<Name> {
    let mut found = Vec::new();
    each_arg(args, &mut |arg| {
        if let Arg::Table { table, .. } = arg {
            found.push(table.clone());
        }
    });
    found
}

/// Walks the trees of one role: unique ids and local names, the names each
/// read may see, and a bound on each call.
struct Walker<'a> {
    role: &'a Name,
    globals: &'a Globals,
    refusals: &'a mut Vec<Refusal>,
    ids: BTreeSet<StepId>,
    locals: BTreeSet<Name>,
    in_initialization: bool,
}

impl Walker<'_> {
    fn at(&self, id: &StepId) -> At {
        At::step(self.role, id)
    }

    fn id(&mut self, id: &StepId) {
        if !self.ids.insert(id.clone()) {
            self.refusals.push(Refusal::new(
                self.at(id),
                format!("the role `{}` has two steps with the id `{id}`", self.role),
            ));
        }
    }

    fn local(&mut self, name: &Name, at: &At) {
        if self.globals.all.contains(name) {
            self.refusals.push(Refusal::new(
                at.clone(),
                format!("`{name}` names a local value and an alias, observation, table, location or signal; give it its own name"),
            ));
        }
        if !self.locals.insert(name.clone()) {
            self.refusals.push(Refusal::new(
                at.clone(),
                format!(
                    "`{name}` names two values in the trees of the role `{}`",
                    self.role
                ),
            ));
        }
    }

    fn readable(&self, defined: &BTreeSet<Name>) -> BTreeSet<Name> {
        defined
            .iter()
            .chain(&self.globals.observations)
            .chain(&self.globals.tables)
            .cloned()
            .collect()
    }

    fn expression(
        &mut self,
        expression: &Expression,
        defined: &BTreeSet<Name>,
        at: &At,
        field: &str,
    ) {
        let readable = self.readable(defined);
        reads_only(
            expression,
            &readable,
            at.clone(),
            field,
            "observations, tables, results named before it on every path, loop elements and parameters",
            self.refusals,
        );
    }

    fn args(&mut self, args: &BTreeMap<String, Arg>, defined: &BTreeSet<Name>, at: &At) {
        let readable: BTreeSet<Name> = defined.union(&self.globals.observations).cloned().collect();
        for root in arg_refs(args) {
            if !readable.contains(&root) {
                self.refusals.push(Refusal::new(
                    at.clone(),
                    format!("`$ref` reads `{root}`, which is no observation, loop element or result named before it on every path"),
                ));
            }
        }
        for table in arg_tables(args) {
            if !self.globals.tables.contains(&table) {
                self.refusals.push(Refusal::new(
                    at.clone(),
                    format!("`$table` names `{table}`, which `tables` does not give"),
                ));
            }
        }
    }

    /// Walks `step` with the names `defined` before it, and gives the names
    /// defined after it on every path.
    fn walk(&mut self, step: &Step, defined: BTreeSet<Name>, bounded: bool) -> BTreeSet<Name> {
        let at = self.at(&step.id);
        self.id(&step.id);
        let bounded = bounded || step.timeout.is_some();
        if let Some(invariant) = &step.invariant {
            reads_only(
                invariant,
                &self.globals.topics,
                at.clone(),
                "invariant",
                "topic observations",
                self.refusals,
            );
        }
        match &step.kind {
            StepKind::Call(call) => {
                if !bounded {
                    self.refusals.push(Refusal::new(
                        at.clone(),
                        "a call has its own `timeout_s` or one of an enclosing step",
                    ));
                }
                self.args(&call.args, &defined, &at);
                let mut after = defined.clone();
                if let Some(result) = &call.result {
                    self.local(result, &at);
                    after.insert(result.clone());
                }
                if let Some(done_when) = &call.done_when {
                    self.expression(done_when, &after, &at, "done_when");
                }
                after
            }
            StepKind::Sequence(steps) => steps
                .iter()
                .fold(defined, |names, child| self.walk(child, names, bounded)),
            StepKind::Parallel { policy, steps } => {
                let outputs: Vec<BTreeSet<Name>> = steps
                    .iter()
                    .map(|child| self.walk(child, defined.clone(), bounded))
                    .collect();
                match policy {
                    super::Policy::All => outputs.into_iter().flatten().collect(),
                    super::Policy::Any => defined,
                }
            }
            StepKind::If {
                cond,
                then,
                otherwise,
            } => {
                self.expression(cond, &defined, &at, "if");
                let then = self.walk(then, defined.clone(), bounded);
                let otherwise = match otherwise {
                    Some(step) => self.walk(step, defined.clone(), bounded),
                    None => defined,
                };
                then.intersection(&otherwise).cloned().collect()
            }
            StepKind::Match {
                on,
                cases,
                otherwise,
            } => {
                self.expression(on, &defined, &at, "match");
                let mut branches: Vec<BTreeSet<Name>> = cases
                    .values()
                    .map(|child| self.walk(child, defined.clone(), bounded))
                    .collect();
                branches.push(match otherwise {
                    Some(step) => self.walk(step, defined.clone(), bounded),
                    None => defined.clone(),
                });
                branches
                    .into_iter()
                    .reduce(|a, b| a.intersection(&b).cloned().collect())
                    .unwrap_or(defined)
            }
            StepKind::ForEach {
                list,
                element,
                body,
                ..
            } => {
                self.expression(list, &defined, &at, "for_each.list");
                self.local(element, &at);
                let mut inside = defined.clone();
                inside.insert(element.clone());
                self.walk(body, inside, bounded);
                defined
            }
            StepKind::RepeatUntil {
                until,
                fail_when,
                body,
                ..
            } => {
                self.expression(until, &defined, &at, "repeat_until");
                if let Some(fail_when) = fail_when {
                    self.expression(fail_when, &defined, &at, "fail_when");
                }
                self.walk(body, defined.clone(), bounded);
                defined
            }
            StepKind::WaitUntil {
                until, fail_when, ..
            } => {
                self.expression(until, &defined, &at, "wait_until");
                if let Some(fail_when) = fail_when {
                    self.expression(fail_when, &defined, &at, "fail_when");
                }
                defined
            }
            StepKind::Signal(_) => {
                self.signal_in_initialization(&at);
                defined
            }
            StepKind::WaitFor { result, .. } => {
                self.signal_in_initialization(&at);
                let mut after = defined;
                if let Some(result) = result {
                    self.local(result, &at);
                    after.insert(result.clone());
                }
                after
            }
        }
    }

    fn signal_in_initialization(&mut self, at: &At) {
        if self.in_initialization {
            self.refusals.push(Refusal::new(
                at.clone(),
                "signals pass between the lanes of `actions`, not in an initialization",
            ));
        }
    }
}

/// The signals of the lanes: the emits and the waits of each lane, each at
/// its place in the tree of its lane, in tree order.
#[derive(Default)]
struct SignalUse {
    emits: BTreeMap<Name, Vec<Emit>>,
    waits: BTreeMap<Name, Vec<Wait>>,
}

struct Emit {
    signal: Name,
    place: Place,
}

struct Wait {
    id: StepId,
    signals: Vec<Name>,
    place: Place,
}

/// Where a step stands in the tree of its lane: the child it is of each
/// step that encloses it, and whether the lane reaches it on every path,
/// through sequences, `all` parallels and `repeat_until` bodies.
#[derive(Clone)]
struct Place {
    path: Vec<Child>,
    every_path: bool,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Child {
    /// The child at an index of a sequence, which runs after the ones
    /// before it.
    InOrder(usize),
    /// A child of a parallel step, a branch of an `if` or a `match`, or the
    /// body of a loop, which no sibling runs before.
    Beside(usize),
}

impl Place {
    fn root() -> Place {
        Place {
            path: Vec::new(),
            every_path: true,
        }
    }

    fn child(&self, child: Child, every_path: bool) -> Place {
        let mut path = self.path.clone();
        path.push(child);
        Place {
            path,
            every_path: self.every_path && every_path,
        }
    }

    /// Whether this place runs after `other`: at the first step that
    /// encloses both, it is a later child of a sequence.
    fn after(&self, other: &Place) -> bool {
        self.path
            .iter()
            .zip(&other.path)
            .find(|(mine, theirs)| mine != theirs)
            .is_some_and(|pair| {
                matches!(pair, (Child::InOrder(mine), Child::InOrder(theirs)) if mine > theirs)
            })
    }
}

impl SignalUse {
    fn add_lane(&mut self, lane: &Name, tree: &Step) {
        self.walk(lane, tree, &Place::root());
    }

    fn walk(&mut self, lane: &Name, step: &Step, place: &Place) {
        match &step.kind {
            StepKind::Call(_) | StepKind::WaitUntil { .. } => {}
            StepKind::Signal(signal) => self.emits.entry(lane.clone()).or_default().push(Emit {
                signal: signal.clone(),
                place: place.clone(),
            }),
            StepKind::WaitFor { signals, .. } => {
                self.waits.entry(lane.clone()).or_default().push(Wait {
                    id: step.id.clone(),
                    signals: signals.clone(),
                    place: place.clone(),
                })
            }
            StepKind::Sequence(steps) => {
                for (index, child) in steps.iter().enumerate() {
                    self.walk(lane, child, &place.child(Child::InOrder(index), true));
                }
            }
            StepKind::Parallel { policy, steps } => {
                let every_child = matches!(policy, super::Policy::All);
                for (index, child) in steps.iter().enumerate() {
                    self.walk(lane, child, &place.child(Child::Beside(index), every_child));
                }
            }
            StepKind::If {
                then, otherwise, ..
            } => {
                let branches = std::iter::once(&**then).chain(otherwise.as_deref());
                for (index, branch) in branches.enumerate() {
                    self.walk(lane, branch, &place.child(Child::Beside(index), false));
                }
            }
            StepKind::Match {
                cases, otherwise, ..
            } => {
                let branches = cases.values().chain(otherwise.as_deref());
                for (index, branch) in branches.enumerate() {
                    self.walk(lane, branch, &place.child(Child::Beside(index), false));
                }
            }
            StepKind::ForEach { body, .. } => {
                self.walk(lane, body, &place.child(Child::Beside(0), false))
            }
            StepKind::RepeatUntil { body, .. } => {
                self.walk(lane, body, &place.child(Child::Beside(0), true))
            }
        }
    }

    /// The lanes that have a `signal` step for each signal.
    fn emitters(&self) -> BTreeMap<&Name, BTreeSet<&Name>> {
        let mut emitters: BTreeMap<&Name, BTreeSet<&Name>> = BTreeMap::new();
        for (lane, emits) in &self.emits {
            for emit in emits {
                emitters.entry(&emit.signal).or_default().insert(lane);
            }
        }
        emitters
    }

    /// Refuses a signal that no other lane emits and the waits that no
    /// signal can end.
    fn check(&self, refusals: &mut Vec<Refusal>) {
        let emitters = self.emitters();
        for (lane, waits) in &self.waits {
            for wait in waits {
                for signal in &wait.signals {
                    let others = emitters
                        .get(signal)
                        .is_some_and(|lanes| lanes.iter().any(|emitter| *emitter != lane));
                    if !others {
                        refusals.push(Refusal::new(
                            At::step(lane, &wait.id),
                            format!("no other lane has a `signal` step for `{signal}`"),
                        ));
                    }
                }
            }
        }
        let blocking = self.blocking_waits();
        if !blocking.is_empty() {
            let names: Vec<String> = blocking
                .iter()
                .map(|(lane, wait)| format!("`{lane}.{}`", wait.id))
                .collect();
            refusals.push(Refusal::new(
                At::field("actions"),
                format!(
                    "the waits {} each wait for signals that their lanes emit only after one of these waits: none of them can end; emit each signal before the wait that holds it back, or wait for it in a `parallel` beside the step that emits it",
                    names.join(", ")
                ),
            ));
        }
    }

    /// The waits that no signal can end: the largest set of waits, each
    /// one its lane reaches on every path, whose every signal comes only from
    /// a `signal` step that runs after a wait of the set in its lane.
    fn blocking_waits(&self) -> Vec<(&Name, &Wait)> {
        let mut blocking: Vec<(&Name, &Wait)> = self
            .waits
            .iter()
            .flat_map(|(lane, waits)| {
                waits
                    .iter()
                    .filter(|wait| wait.place.every_path)
                    .map(move |wait| (lane, wait))
            })
            .collect();
        loop {
            let can_come = |signal: &Name| {
                self.emits.iter().any(|(lane, emits)| {
                    emits.iter().any(|emit| {
                        emit.signal == *signal
                            && !blocking
                                .iter()
                                .any(|(held, wait)| *held == lane && emit.place.after(&wait.place))
                    })
                })
            };
            let still: Vec<(&Name, &Wait)> = blocking
                .iter()
                .copied()
                .filter(|(_, wait)| !wait.signals.iter().any(can_come))
                .collect();
            if still.len() == blocking.len() {
                return still;
            }
            blocking = still;
        }
    }
}

impl Step {
    /// Calls `visit` on this step and each step under it, in tree order.
    pub fn visit<'a>(&'a self, visit: &mut dyn FnMut(&'a Step)) {
        visit(self);
        match &self.kind {
            StepKind::Call(_)
            | StepKind::WaitUntil { .. }
            | StepKind::Signal(_)
            | StepKind::WaitFor { .. } => {}
            StepKind::Sequence(steps) | StepKind::Parallel { steps, .. } => {
                steps.iter().for_each(|step| step.visit(visit))
            }
            StepKind::If {
                then, otherwise, ..
            } => {
                then.visit(visit);
                if let Some(step) = otherwise {
                    step.visit(visit);
                }
            }
            StepKind::Match {
                cases, otherwise, ..
            } => {
                cases.values().for_each(|step| step.visit(visit));
                if let Some(step) = otherwise {
                    step.visit(visit);
                }
            }
            StepKind::ForEach { body, .. } | StepKind::RepeatUntil { body, .. } => {
                body.visit(visit)
            }
        }
    }
}
