//! A single-flight move slot: a handler claims it when it admits a goal, so
//! the part the slot guards runs one goal at a time and a second goal is
//! refused while the first runs. The claim gives the [`SlotRelease`], the one
//! way to free the slot, which the goal carries from its admission and then
//! the move that runs it, so every end of the goal frees the slot, an end
//! before the goal reaches the loop that runs it included.
//!
//! A stop that finds the slot claimed and no goal of it in its loop came
//! between the admission of the goal and the loop's take of it. The stop
//! marks the slot ([`MoveSlot::stop_unstarted`]), and the loop ends the goal
//! cancelled for the stop's reason when it takes the goal
//! ([`MoveSlot::take_stop`]).

use std::sync::{Arc, Mutex};

use crate::sync::lock_ignoring_poison;

/// Who holds a move slot.
#[derive(Debug, Default, PartialEq)]
enum Holder {
    /// No goal holds the slot.
    #[default]
    Free,
    /// An admitted goal holds the slot.
    Goal,
    /// An admitted goal holds the slot, and a stop came, for this reason,
    /// before the loop that runs the goal took it.
    StoppedGoal(String),
}

/// A move slot, shared by the handlers that claim it and the loop that runs
/// the claimed goal.
#[derive(Clone, Default)]
pub struct MoveSlot(Arc<Mutex<Holder>>);

impl MoveSlot {
    /// Run `change` on who holds the slot. Every holder is a whole state, so a
    /// lock that a panic poisoned holds one too.
    fn with_holder<T>(&self, change: impl FnOnce(&mut Holder) -> T) -> T {
        change(&mut lock_ignoring_poison(&self.0))
    }

    /// Claim the slot for an admitted goal, which carries the release the
    /// claim gives: `None` while another goal holds it.
    #[must_use]
    pub fn claim(&self) -> Option<SlotRelease> {
        self.with_holder(|holder| {
            if *holder != Holder::Free {
                return None;
            }
            *holder = Holder::Goal;
            Some(SlotRelease(self.clone()))
        })
    }

    /// Whether a goal holds the slot.
    pub fn is_claimed(&self) -> bool {
        self.with_holder(|holder| *holder != Holder::Free)
    }

    /// Free the slot. The mark of a stop goes with the goal that held it.
    fn release(&self) {
        self.with_holder(|holder| *holder = Holder::Free);
    }

    /// Mark the goal that holds the slot as stopped for `reason`, which names
    /// the stop: for a stop that ended no goal of this slot, so the goal is
    /// admitted and its loop has not taken it. A free slot gets no
    /// mark, and a goal that an earlier stop marked keeps that stop's reason.
    /// Whether this stop marked a goal.
    pub fn stop_unstarted(&self, reason: &str) -> bool {
        self.with_holder(|holder| {
            let unstarted = *holder == Holder::Goal;
            if unstarted {
                *holder = Holder::StoppedGoal(reason.to_string());
            }
            unstarted
        })
    }

    /// The reason of the stop that came before the loop took the goal
    /// that holds the slot, or `None` when no stop came. The mark is taken:
    /// the goal still holds the slot, for its caller to end it, which drops
    /// its release.
    #[must_use]
    pub fn take_stop(&self) -> Option<String> {
        self.with_holder(|holder| match std::mem::take(holder) {
            Holder::StoppedGoal(reason) => {
                *holder = Holder::Goal;
                Some(reason)
            }
            unmarked => {
                *holder = unmarked;
                None
            }
        })
    }
}

/// The claim of a move slot, which frees the slot on drop. A goal, or the move
/// that runs it, holds the one its claim gave for its lifetime, so no end of it
/// (success, failure, cancel, or a move that never starts) leaves the slot
/// claimed.
#[must_use = "dropping a SlotRelease frees its move slot"]
pub struct SlotRelease(MoveSlot);

impl Drop for SlotRelease {
    fn drop(&mut self) {
        self.0.release();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Claims the free `slot`, as the admission of a goal does.
    fn claimed(slot: &MoveSlot) -> SlotRelease {
        slot.claim().expect("a free slot is claimed")
    }

    #[test]
    fn a_slot_is_claimed_once_until_its_release_drops() {
        let slot = MoveSlot::default();
        let release = claimed(&slot);
        assert!(slot.is_claimed());
        assert!(
            slot.claim().is_none(),
            "a claimed slot refuses a second claim"
        );
        assert!(slot.is_claimed(), "the refused claim frees nothing");
        drop(release);
        assert!(
            !slot.is_claimed(),
            "the release frees the slot on drop, so no move terminal can leak it"
        );
        let _again = claimed(&slot);
    }

    #[test]
    fn a_stop_marks_the_goal_that_holds_the_slot_and_the_mark_is_taken_once() {
        let slot = MoveSlot::default();
        let _release = claimed(&slot);
        assert_eq!(slot.take_stop(), None, "no stop came");
        assert!(
            slot.stop_unstarted("stopped: the operator asked"),
            "the stop marks the goal"
        );
        assert!(slot.is_claimed(), "the stopped goal still holds the slot");
        assert!(
            slot.claim().is_none(),
            "a stopped goal refuses a second claim"
        );
        assert_eq!(
            slot.take_stop(),
            Some("stopped: the operator asked".to_string())
        );
        assert_eq!(slot.take_stop(), None, "the mark is taken once");
        assert!(
            slot.is_claimed(),
            "the goal holds the slot until its release"
        );
    }

    #[test]
    fn a_stop_marks_no_free_slot_and_keeps_the_reason_of_the_first_stop() {
        let slot = MoveSlot::default();
        assert!(
            !slot.stop_unstarted("stopped: first"),
            "a free slot holds no goal to mark"
        );
        assert!(!slot.is_claimed(), "a stop claims nothing");
        let _release = claimed(&slot);
        assert_eq!(slot.take_stop(), None, "the stop came before the goal");

        assert!(slot.stop_unstarted("stopped: first"));
        assert!(
            !slot.stop_unstarted("stopped: second"),
            "a goal an earlier stop marked is not marked again"
        );
        assert_eq!(slot.take_stop(), Some("stopped: first".to_string()));
    }

    #[test]
    fn the_release_of_a_stopped_goal_leaves_no_mark_for_the_next_goal() {
        let slot = MoveSlot::default();
        let release = claimed(&slot);
        assert!(slot.stop_unstarted("stopped"));
        drop(release);
        assert!(!slot.is_claimed());
        let _next = claimed(&slot);
        assert_eq!(slot.take_stop(), None, "the next goal is not stopped");
    }

    #[test]
    fn a_slot_whose_lock_a_panic_poisoned_still_claims_and_releases() {
        let slot = MoveSlot::default();
        let poisoner = slot.clone();
        let panicked = std::thread::spawn(move || {
            let _held = poisoner.0.lock().expect("a fresh lock");
            panic!("a holder panics with the lock held");
        })
        .join();
        assert!(panicked.is_err(), "the holder panicked");
        assert!(slot.0.is_poisoned());
        let release = claimed(&slot);
        drop(release);
        assert!(!slot.is_claimed());
    }
}
