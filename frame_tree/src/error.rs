//! Why a lookup has no answer. Each variant is a failure kind of a lookup: an unknown
//! frame, no chain between the frames, a transform with no value at the time, and a broken
//! tree.

use std::fmt;

use crate::cache::HeldStamps;
use crate::frame::{FrameName, LookupTime, Publisher};

#[derive(Clone, PartialEq, Debug, thiserror::Error)]
pub enum LookupError {
    /// No transform the tree took names the frame, as tf2's lookup refuses a frame it holds
    /// no number for (buffer_core.cpp:148-153). `held` is every frame the tree's transforms
    /// name, sorted, so the error says what to look up; tf2's names none.
    #[error("no transform of the tree names the frame {frame}: {}", held_frames_text(.held))]
    UnknownFrame {
        frame: FrameName,
        held: Vec<FrameName>,
    },
    /// tf2's connectivity error (buffer_core.cpp:1047-1058): the chains of the two frames
    /// share no frame, whether no transform between them came or, at the latest time, a
    /// moving one holds no stamp since the clock went back.
    #[error(
        "no chain of transforms joins {source_frame} to {target_frame}: they are in two \
         trees; check that each transform between them is published, and after the clock \
         went back, that each moving one has sent a stamp since"
    )]
    NoChain {
        target_frame: FrameName,
        source_frame: FrameName,
    },
    /// tf2's extrapolation errors (cache.cpp:71-111): the transform has no value at
    /// `requested`. Its parent is the parent of its newest sample, or of its last one
    /// before a clear.
    #[error(
        "the transform {transform} has no value at {requested}: {}; look up a time it \
         holds, or the latest time",
        held_text(.held)
    )]
    Extrapolation {
        transform: TransformName,
        requested: LookupTime,
        held: HeldStamps,
    },
    /// The lookup needs a transform that breaks the tree.
    #[error(transparent)]
    BrokenTree(#[from] BrokenTree),
}

fn held_frames_text(held: &[FrameName]) -> String {
    if held.is_empty() {
        return "the tree holds no frame yet, since no publisher's transform has reached it"
            .to_owned();
    }
    let names: Vec<&str> = held.iter().map(FrameName::as_str).collect();
    format!("look up one of the frames it holds: {}", names.join(", "))
}

/// A transform by its frames.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct TransformName {
    pub parent: FrameName,
    pub child: FrameName,
}

impl fmt::Display for TransformName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} -> {}", self.parent, self.child)
    }
}

/// Transforms that break the tree.
#[derive(Clone, PartialEq, Debug, thiserror::Error)]
pub enum BrokenTree {
    /// Two publishers hold a transform of `child`. tf2 keeps every sample of either and
    /// names the last publisher (buffer_core.cpp:293).
    #[error(
        "two publishers, {first} and {second}, publish the transform of {child}: each \
         transform of the tree has one publisher, so stop the one that does not own it"
    )]
    TwoPublishers {
        child: FrameName,
        first: Publisher,
        second: Publisher,
    },
    /// The transforms of a cycle, each frame's transform to its parent, in the order a walk
    /// up the tree meets them. tf2 finds a cycle only when a walk passes its depth limit
    /// (buffer_core.cpp:398-407).
    #[error(
        "the transforms {} form a cycle: the chain of parents of each frame ends at a root, so \
         fix the publisher of one of them",
        cycle_text(.transforms)
    )]
    Cycle { transforms: Vec<TransformName> },
}

fn held_text(held: &HeldStamps) -> String {
    match held {
        HeldStamps::Empty => "it holds no sample since the clock went back".into(),
        HeldStamps::One(stamp) => format!("it holds one sample, at {stamp}"),
        HeldStamps::Range { oldest, newest } => {
            format!("it holds samples from {oldest} to {newest}")
        }
    }
}

fn cycle_text(transforms: &[TransformName]) -> String {
    transforms
        .iter()
        .map(TransformName::to_string)
        .collect::<Vec<_>>()
        .join(", ")
}
