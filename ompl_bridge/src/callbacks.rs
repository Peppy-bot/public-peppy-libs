//! The caller's checks, constraint and goal sampling as OMPL calls them, from C++, and as
//! `plan` calls them: an error or a panic in one, in its message or in its drop, is kept
//! here, never crosses into C++ or out of `plan`, and ends the plan.

use std::any::Any;
use std::cell::Cell;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, PoisonError};

use crate::ffi::bridge;
use crate::{
    CheckError, Checks, Constraint, GoalSample, GoalSampling, Motion, PLANNING_THREADS,
    SampledGoals, Space,
};

thread_local! {
    /// Whether this thread runs a check, the constraint or the goal sampling of a plan.
    static IN_CALLBACK: Cell<bool> = const { Cell::new(false) };
}

/// Whether the calling thread runs a check, the constraint or the goal sampling of a plan.
pub(crate) fn in_callback() -> bool {
    IN_CALLBACK.get()
}

pub(crate) struct Callbacks<'a> {
    checks: &'a dyn Checks,
    constraint: Option<&'a dyn Constraint>,
    sampled: Option<Sampled<'a>>,
    /// The caller's flag that ends the plan.
    stop: Option<&'a AtomicBool>,
    failed: AtomicBool,
    /// The first failure: what the error or the panic said.
    failure: Mutex<Option<String>>,
}

// C++ shares one `&Callbacks` between the threads of a plan: its attempts' and those on
// which PRM and PRM* search their roadmap.
const _: () = {
    const fn shared_between_threads<T: Sync>() {}
    shared_between_threads::<Callbacks<'static>>();
};

/// The sampling of an attempt, with the attempt's number.
type AttemptSampling<'a> = (u32, Box<dyn GoalSampling + 'a>);

/// A plan's sampled goals: the caller's sampling, the space its goals lie in, and a slot for
/// the sampling of each attempt that plans at once, once the attempt has asked for a goal.
/// The attempts of a batch are at most `PLANNING_THREADS` in a row, so each takes a slot of its
/// own, and an attempt of a later batch replaces the sampling of one that has ended.
pub(crate) struct Sampled<'a> {
    goals: &'a dyn SampledGoals,
    space: &'a Space,
    slots: [Mutex<Option<AttemptSampling<'a>>>; PLANNING_THREADS.get() as usize],
}

impl<'a> Sampled<'a> {
    pub(crate) fn new(goals: &'a dyn SampledGoals, space: &'a Space) -> Self {
        Self {
            goals,
            space,
            slots: std::array::from_fn(|_| Mutex::new(None)),
        }
    }
}

impl<'a> Callbacks<'a> {
    pub(crate) fn new(
        checks: &'a dyn Checks,
        constraint: Option<&'a dyn Constraint>,
        sampled: Option<Sampled<'a>>,
        stop: Option<&'a AtomicBool>,
    ) -> Self {
        Self {
            checks,
            constraint,
            sampled,
            stop,
            failed: AtomicBool::new(false),
            failure: Mutex::new(None),
        }
    }

    /// What the first failing call said, once one has failed.
    pub(crate) fn failure(&self) -> Option<String> {
        self.failure
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .clone()
    }

    pub(crate) fn failed(&self) -> bool {
        self.failed.load(Ordering::SeqCst)
    }

    /// Whether the caller has set its flag that ends the plan.
    pub(crate) fn stopped(&self) -> bool {
        self.stop.is_some_and(|stop| stop.load(Ordering::SeqCst))
    }

    pub(crate) fn state_valid(&self, state: &[f64]) -> bool {
        self.guarded(false, || self.checks.state(state))
    }

    /// Whether the motion is valid by the planner's check, or by the simplifier's when
    /// `simplifying`; when it is not, `last_valid` takes the fraction of the way along it
    /// to its last valid state.
    pub(crate) fn motion_valid(
        &self,
        from: &[f64],
        to: &[f64],
        simplifying: bool,
        last_valid: &mut f64,
    ) -> bool {
        let motion = self.guarded(Motion::Invalid { last_valid: 0.0 }, || {
            let motion = if simplifying {
                self.checks.simplifier_motion(from, to)?
            } else {
                self.checks.motion(from, to)?
            };
            match motion {
                Motion::Invalid { last_valid } if !(0.0..=1.0).contains(&last_valid) => Err(
                    format!("a motion's last valid fraction lies in [0, 1], not {last_valid}")
                        .into(),
                ),
                motion => Ok(motion),
            }
        });
        match motion {
            Motion::Valid => true,
            Motion::Invalid {
                last_valid: fraction,
            } => {
                *last_valid = fraction;
                false
            }
        }
    }

    pub(crate) fn constraint_function(&self, state: &[f64], out: &mut [f64]) {
        let constraint = self
            .constraint
            .expect("a constrained plan has a constraint");
        self.guarded((), || constraint.function(state, out));
    }

    pub(crate) fn constraint_jacobian(&self, state: &[f64], out: &mut [f64]) {
        let constraint = self
            .constraint
            .expect("a constrained plan has a constraint");
        self.guarded((), || constraint.jacobian(state, out));
    }

    /// One sample of attempt `attempt`'s goal sampling, which writes the goal it finds into
    /// `goal`; the sampling of an attempt starts at the attempt's first sample. A goal
    /// outside the space fails the plan.
    pub(crate) fn sample_goal(&self, attempt: u32, goal: &mut [f64]) -> bridge::GoalSample {
        let sampled = self
            .sampled
            .as_ref()
            .expect("a plan of sampled goals has its sampling");
        let found = self.guarded(GoalSample::Ended, || {
            let mut slot = sampled.slots[attempt as usize % sampled.slots.len()]
                .lock()
                .unwrap_or_else(PoisonError::into_inner);
            if slot.as_ref().is_none_or(|(of, _)| *of != attempt) {
                *slot = Some((attempt, sampled.goals.sampling(attempt)));
            }
            let (_, sampling) = slot
                .as_mut()
                .expect("the slot holds the attempt's sampling");
            let found = sampling.sample(goal)?;
            let outside = (found == GoalSample::Found)
                .then(|| sampled.space.outside("a sampled goal", goal))
                .flatten();
            match outside {
                Some(outside) => {
                    Err(format!("{outside}; a goal lies within the space's bounds").into())
                }
                None => Ok(found),
            }
        });
        match found {
            GoalSample::Found => bridge::GoalSample::Found,
            GoalSample::Missed => bridge::GoalSample::Missed,
            GoalSample::Ended => bridge::GoalSample::Ended,
        }
    }

    /// Drops the samplings of the attempts, whose drop is the caller's too: a panic in one
    /// fails the plan.
    pub(crate) fn end_samplings(&self) {
        let Some(sampled) = &self.sampled else {
            return;
        };
        for slot in &sampled.slots {
            let ended = caught(|| {
                drop(slot.lock().unwrap_or_else(PoisonError::into_inner).take());
                Ok(())
            });
            if let Err(failure) = ended {
                self.fail(failure);
            }
        }
    }

    /// `call`'s answer, or `fallback` once any call has failed: this one by its error
    /// or its panic, or one before it.
    fn guarded<T>(&self, fallback: T, call: impl FnOnce() -> Result<T, CheckError>) -> T {
        if self.failed() {
            return fallback;
        }
        match caught(call) {
            Ok(answer) => answer,
            Err(failure) => {
                self.fail(failure);
                fallback
            }
        }
    }

    fn fail(&self, failure: String) {
        let mut first = self
            .failure
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        first.get_or_insert(failure);
        self.failed.store(true, Ordering::SeqCst);
    }
}

/// `call`'s answer, or what its error or its panic said, with the calling thread marked
/// as running a callback of a plan while it runs. The error's message and its drop run
/// inside the catch, and the drop of a panic's payload runs inside a catch of its own.
pub(crate) fn caught<T>(call: impl FnOnce() -> Result<T, CheckError>) -> Result<T, String> {
    let was_in_callback = IN_CALLBACK.replace(true);
    let outcome = catch_unwind(AssertUnwindSafe(|| {
        call().map_err(|error| error.to_string())
    }));
    IN_CALLBACK.set(was_in_callback);
    outcome.unwrap_or_else(|payload| Err(panic_message(payload)))
}

/// What a panic said. Drops its payload; a panic in that drop is caught, and its own
/// payload is forgotten unread.
fn panic_message(payload: Box<dyn Any + Send>) -> String {
    let message = payload
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "a panic with no message".to_string());
    if let Err(payload_of_drop) = catch_unwind(AssertUnwindSafe(move || drop(payload))) {
        std::mem::forget(payload_of_drop);
    }
    format!("panicked: {message}")
}
