use serde::{Deserialize, Serialize};
use std::fmt;

use crate::names::{Name, StepId};

/// Where a refusal points: the document as a whole, a field by its path from
/// the root, or a step by its role and id.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum At {
    Document,
    Field(String),
    Step { role: Name, id: StepId },
}

impl At {
    pub fn field(path: impl Into<String>) -> At {
        At::Field(path.into())
    }

    pub fn step(role: &Name, id: &StepId) -> At {
        At::Step {
            role: role.clone(),
            id: id.clone(),
        }
    }
}

/// `<role>.<id>` for a step, the dotted path for a field, and nothing for the
/// document: the `at` that the workflow director gives each error of a check.
impl fmt::Display for At {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            At::Document => Ok(()),
            At::Field(path) => f.write_str(path),
            At::Step { role, id } => write!(f, "{role}.{id}"),
        }
    }
}

/// One rule that a document breaks, with where it breaks it.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Refusal {
    pub at: At,
    pub reason: String,
}

impl Refusal {
    pub fn new(at: At, reason: impl Into<String>) -> Refusal {
        Refusal {
            at,
            reason: reason.into(),
        }
    }

    /// The `{ at, message }` form of a refusal that a check answers with.
    pub fn located(&self) -> Located {
        Located {
            at: self.at.to_string(),
            message: self.reason.clone(),
        }
    }
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.at {
            At::Document => f.write_str(&self.reason),
            _ => write!(f, "{}: {}", self.at, self.reason),
        }
    }
}

/// A refusal as text: where, and why.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Located {
    pub at: String,
    pub message: String,
}

/// `at: message`, or the message alone where it names no place.
impl fmt::Display for Located {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.at.is_empty() {
            true => f.write_str(&self.message),
            false => write!(f, "{}: {}", self.at, self.message),
        }
    }
}

/// The rules a document breaks, at least one. A workflow gives them in the
/// order of their places, each once; the other documents in the order their
/// parser found them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Refusals(Vec<Refusal>);

impl Refusals {
    pub fn one(refusal: Refusal) -> Refusals {
        Refusals(vec![refusal])
    }

    /// These refusals of the document in the file `file`, each at its place
    /// in that file.
    pub fn in_file(self, file: &str) -> Refusals {
        let placed = self
            .0
            .into_iter()
            .map(|refusal| {
                let at = match &refusal.at {
                    At::Document => At::field(file),
                    at => At::field(format!("{file}: {at}")),
                };
                Refusal { at, ..refusal }
            })
            .collect();
        Refusals(placed)
    }

    /// The refusals of `found`, or `ok` when there is none.
    pub fn or_ok<T>(found: Vec<Refusal>, ok: T) -> Result<T, Refusals> {
        if found.is_empty() {
            return Ok(ok);
        }
        Err(Refusals(found))
    }

    pub fn iter(&self) -> impl Iterator<Item = &Refusal> {
        self.0.iter()
    }
}

impl From<documents_core::Error> for Refusals {
    fn from(error: documents_core::Error) -> Refusals {
        Refusals::one(Refusal::new(At::Document, error.to_string()))
    }
}

impl fmt::Display for Refusals {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let lines: Vec<String> = self.0.iter().map(Refusal::to_string).collect();
        f.write_str(&lines.join("\n"))
    }
}

impl std::error::Error for Refusals {}
