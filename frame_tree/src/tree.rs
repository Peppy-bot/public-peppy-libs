// Copyright 2008, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This file translates into Rust tf2's BufferCore (geometry2 0.45.10): the storing of a
// transform, the lookup of a transform, the walks of the tree and the latest common time
// of tf2/src/buffer_core.cpp. Peppy Inc changed it: each transform records its publisher
// and its start, a publisher's static set replaces its last one, the tree names the
// transforms that two publishers hold or that close a cycle, and the caller gives the
// cache window.

//! The tree: every transform the publishers sent, kept per child frame and per publisher,
//! and the lookup of one frame in another.

use std::collections::{HashMap, HashSet};
use std::time::Duration;

use crate::arithmetic::{self, Quaternion, Vector};
use crate::cache::{HeldStamps, NoValue, Sample, TimeCache};
use crate::error::{BrokenTree, LookupError, TransformName};
use crate::frame::{FrameName, LookupTime, Publisher, Stamp, Start};
use crate::transforms::{
    FrameTransform, MessageError, MovingMessage, Parsed, Placement, StaticMessage, TransformError,
    TransformErrorReason,
};

/// The place of the source frame in the target frame, and the time it holds at: the
/// lookup's own time, the latest time common to the chain, or none where every transform
/// the lookup composes is static and holds at every time.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Found {
    pub placement: Placement,
    pub time: Option<Stamp>,
}

/// A cache window of zero, which `FrameTree::new` refuses.
#[derive(Clone, Copy, PartialEq, Eq, Debug, thiserror::Error)]
#[error("the cache window is 0 s: give a window above 0 s, as the transforms contract asks")]
pub struct ZeroWindow;

/// The transform of one child frame from one publisher: a static transform, the samples of
/// a moving one, or a moving one whose samples a clear dropped, with the parent of its last
/// sample. tf2 holds a `StaticCache` or a `TimeCache` per child frame
/// (buffer_core.cpp:310-319), and a cleared `TimeCache` holds no sample
/// (cache.cpp:293-296).
#[derive(Clone, Debug)]
enum History {
    Static {
        parent: FrameName,
        placement: Placement,
    },
    Moving(TimeCache),
    Empty {
        parent: FrameName,
    },
}

#[derive(Clone, Debug)]
struct Entry {
    publisher: Publisher,
    history: History,
}

/// Why a walk did not read a transform: no value at the time, with the newest stamp the
/// transform holds, which tf2 weighs when both walks fail (buffer_core.cpp:426-445), or a
/// broken tree.
enum Failure {
    NoValue {
        newest: Option<Stamp>,
        error: LookupError,
    },
    Broken(BrokenTree),
}

impl Failure {
    fn into_error(self) -> LookupError {
        match self {
            Failure::NoValue { error, .. } => error,
            Failure::Broken(broken) => broken.into(),
        }
    }
}

/// The newest stamp of a transform and the parent of its newest sample, as
/// `getLatestTimeAndParent` gives them (cache.cpp:303-311, static_cache.cpp:77-80): no
/// stamp for a static transform.
struct Newest<'a> {
    stamp: Option<Stamp>,
    parent: &'a FrameName,
}

/// The frames of a robot as one tree, from the transforms its publishers send: each
/// transform kept per child frame and per [`Publisher`], a moving transform's samples
/// within the cache window of the child frame's newest stamp, a static transform for every
/// time.
#[derive(Clone, Debug)]
pub struct FrameTree {
    window_nanos: u64,
    /// Every frame a valid transform named, whether or not the tree took its sample, as
    /// tf2 numbers both frames of a valid transform before its cache takes it
    /// (buffer_core.cpp:273, 291), and forgets none.
    known: HashSet<FrameName>,
    /// The transforms of each child frame, one entry per publisher, in the order the
    /// publishers first sent one. No list is empty.
    transforms: HashMap<FrameName, Vec<Entry>>,
}

impl FrameTree {
    /// An empty tree that keeps each moving transform's samples within `window` of its
    /// child frame's newest stamp, as tf2's `cache_time` does (buffer_core.hpp:106). The
    /// caller gives the window, above zero.
    pub fn new(window: Duration) -> Result<Self, ZeroWindow> {
        if window.is_zero() {
            return Err(ZeroWindow);
        }
        Ok(Self {
            window_nanos: u64::try_from(window.as_nanos()).unwrap_or(u64::MAX),
            known: HashSet::new(),
            transforms: HashMap::new(),
        })
    }

    /// Takes a message of `transforms` from `publisher`, and gives each transform of it the
    /// tree refused, in the message's order: an invalid one, or one whose stamp is older
    /// than its child frame's newest by more than the window. A message refused whole
    /// changes nothing.
    pub fn take_moving(
        &mut self,
        publisher: &Publisher,
        message: &MovingMessage,
    ) -> Result<Vec<TransformError>, MessageError> {
        let (
            stamp,
            start,
            Parsed {
                transforms,
                mut refused,
            },
        ) = message.parse()?;
        for transform in &transforms {
            let Err(reason) = self.insert_moving(publisher, stamp, start, transform) else {
                continue;
            };
            let child = transform.child().as_str();
            refused.push(TransformError {
                index: message
                    .child_frames
                    .iter()
                    .position(|named| named == child)
                    .expect("a parsed transform names a child frame of its message"),
                child: child.to_owned(),
                reason,
            });
        }
        refused.sort_by_key(|error| error.index);
        Ok(refused)
    }

    /// Takes a message of `static_transforms` from `publisher` as its whole static set, and
    /// gives each invalid transform of it, in the message's order. A message refused whole
    /// changes nothing.
    pub fn take_static(
        &mut self,
        publisher: &Publisher,
        message: &StaticMessage,
    ) -> Result<Vec<TransformError>, MessageError> {
        let Parsed {
            transforms,
            refused,
        } = message.parse()?;
        self.replace_static(publisher, &transforms);
        Ok(refused)
    }

    /// Takes one moving transform, as tf2's `insertData` takes a sample
    /// (cache.cpp:246-291) with the window counted back from the newest stamp of the child
    /// frame from any publisher. A start that differs from the one held from `publisher`
    /// drops `publisher`'s samples of the child frame first, so no lookup interpolates
    /// across a start. Then the samples past the window go, a moving transform left with
    /// none goes, and a sample older than the window is refused (cache.cpp:256-264).
    /// Otherwise the sample goes in among `publisher`'s samples; a transform that
    /// `publisher` sent static, or that a clear emptied, becomes a moving one, as tf2
    /// replaces a static cache with a time cache (buffer_core.cpp:276-285). Last, the other
    /// publishers' moving transforms of the child frame that hold no stamp within the
    /// window of its newest go.
    fn insert_moving(
        &mut self,
        publisher: &Publisher,
        stamp: Stamp,
        start: Start,
        transform: &FrameTransform,
    ) -> Result<(), TransformErrorReason> {
        self.register(transform);
        let window = self.window_nanos;
        let entries = self.entries_of(transform.child());
        entries.retain(|entry| {
            entry.publisher != *publisher
                || !matches!(&entry.history, History::Moving(cache) if cache.start() != start)
        });
        if let Some(latest) = newest_moving(entries) {
            entries.retain_mut(|entry| match &mut entry.history {
                History::Moving(cache) => cache.prune(latest),
                History::Static { .. } | History::Empty { .. } => true,
            });
            if stamp.as_nanos() < latest.as_nanos().saturating_sub(window) {
                return Err(TransformErrorReason::BeforeWindow {
                    stamp,
                    newest: latest,
                });
            }
        }
        let sample = Sample {
            stamp,
            parent: transform.parent().clone(),
            placement: transform.placement(),
        };
        let fresh = |sample| History::Moving(TimeCache::new(sample, window, start));
        upsert(
            entries,
            publisher,
            sample,
            fresh,
            |history, sample| match history {
                History::Moving(cache) => cache.insert(sample),
                History::Static { .. } | History::Empty { .. } => *history = fresh(sample),
            },
        );
        drop_unheld(entries, window);
        Ok(())
    }

    /// Drops the samples of every moving transform, as tf2's `clear` does
    /// (buffer_core.cpp:171-183): a consumer clears its tree when its clock goes back, as the
    /// transforms contract asks. Each transform keeps its frames and the parent of its last
    /// sample and holds no sample until its next one. tf2 clears its static transforms too
    /// (static_cache.cpp:63); a static transform holds at every time, so the clear keeps it.
    pub fn clear_moving(&mut self) {
        for entry in self.transforms.values_mut().flatten() {
            let History::Moving(cache) = &entry.history else {
                continue;
            };
            let parent = cache.newest().parent.clone();
            entry.history = History::Empty { parent };
        }
    }

    /// Takes `transforms` as every static transform of `publisher`: each transform of it
    /// holds at every time, replacing the transform `publisher` held of its child frame,
    /// static or moving, as tf2 replaces a frame's cache with a static one
    /// (buffer_core.cpp:276-285); and each static transform of `publisher` of a child frame
    /// the set does not name is dropped. A message of `static_transforms` carries its
    /// publisher's whole static set, so the set it omits a transform from is the set
    /// without it; tf2 drops no static transform.
    fn replace_static(&mut self, publisher: &Publisher, transforms: &[FrameTransform]) {
        let children: HashSet<&FrameName> = transforms.iter().map(FrameTransform::child).collect();
        for (child, entries) in &mut self.transforms {
            if !children.contains(child) {
                entries.retain(|entry| {
                    entry.publisher != *publisher
                        || !matches!(entry.history, History::Static { .. })
                });
            }
        }
        self.transforms.retain(|_, entries| !entries.is_empty());
        for transform in transforms {
            self.register(transform);
            let history = History::Static {
                parent: transform.parent().clone(),
                placement: transform.placement(),
            };
            let entries = self.entries_of(transform.child());
            upsert(
                entries,
                publisher,
                history,
                |history| history,
                |held, history| {
                    *held = history;
                },
            );
        }
    }

    fn register(&mut self, transform: &FrameTransform) {
        for frame in [transform.parent(), transform.child()] {
            if !self.known.contains(frame) {
                self.known.insert(frame.clone());
            }
        }
    }

    /// The entries of `child`, a new empty list where it has none.
    fn entries_of(&mut self, child: &FrameName) -> &mut Vec<Entry> {
        if !self.transforms.contains_key(child) {
            self.transforms.insert(child.clone(), Vec::new());
        }
        self.transforms
            .get_mut(child)
            .expect("the list of the child frame is there")
    }

    /// The place of `source` in `target` at `time`, as tf2's `lookupTransform` gives it
    /// (buffer_core.cpp:822-878): the identity for one frame, known or not; else each
    /// frame known, the target's checked first, and the transforms composed along both
    /// frames' chains of parents up to where they meet, each moving transform at the time,
    /// by `walkToTopParent` and `TransformAccum` (buffer_core.cpp:330-596). At the latest
    /// time, the time is the latest common to the chain, by `getLatestCommonTime`
    /// (buffer_core.cpp:1112-1261).
    pub fn lookup(
        &self,
        target: &FrameName,
        source: &FrameName,
        time: LookupTime,
    ) -> Result<Found, LookupError> {
        if target == source {
            let time = match time {
                LookupTime::At(stamp) => Some(stamp),
                LookupTime::Latest => self.newest_stamp(target),
            };
            return Ok(Found {
                placement: Placement::IDENTITY,
                time,
            });
        }
        for frame in [target, source] {
            if !self.known.contains(frame) {
                return Err(LookupError::UnknownFrame {
                    frame: frame.clone(),
                    held: self.held_frames(),
                });
            }
        }
        let at = match time {
            LookupTime::At(stamp) => LookupTime::At(stamp),
            LookupTime::Latest => self
                .latest_common_time(target, source)?
                .map_or(LookupTime::Latest, LookupTime::At),
        };
        let placement = self.walk(target, source, at)?;
        Ok(Found {
            placement,
            time: match at {
                LookupTime::At(stamp) => Some(stamp),
                LookupTime::Latest => None,
            },
        })
    }

    /// Every frame the tree's transforms name, sorted by name.
    fn held_frames(&self) -> Vec<FrameName> {
        let mut held: Vec<FrameName> = self.known.iter().cloned().collect();
        held.sort();
        held
    }

    /// The newest stamp of the moving transform of `frame`, as tf2 gives the latest time of
    /// a lookup of a frame in itself (buffer_core.cpp:834-844): none for a static or a
    /// cleared transform, a frame without a transform, or a transform of two publishers.
    fn newest_stamp(&self, frame: &FrameName) -> Option<Stamp> {
        match self.history_of(frame)? {
            Ok(History::Moving(cache)) => Some(cache.newest().stamp),
            _ => None,
        }
    }

    /// The transform of `child` the tree reads: the one held, a static transform or a
    /// moving one, or an emptied one where none is held; broken where two publishers hold
    /// one. None for a frame without a transform, a root.
    fn history_of(&self, child: &FrameName) -> Option<Result<&History, BrokenTree>> {
        let entries = self.transforms.get(child)?;
        let mut held = entries
            .iter()
            .filter(|entry| !matches!(entry.history, History::Empty { .. }));
        Some(match (held.next(), held.next()) {
            (Some(first), Some(second)) => Err(BrokenTree::TwoPublishers {
                child: child.clone(),
                first: first.publisher.clone(),
                second: second.publisher.clone(),
            }),
            (Some(only), None) => Ok(&only.history),
            (None, _) => Ok(&entries.first()?.history),
        })
    }

    /// The transform of `child` at `at`, its parent and its placement, as `TransformAccum`
    /// gathers it (buffer_core.cpp:536-545); none for a frame without a transform, a root.
    fn transform_at(
        &self,
        child: &FrameName,
        at: LookupTime,
    ) -> Option<Result<(&FrameName, Placement), Failure>> {
        let history = match self.history_of(child)? {
            Ok(history) => history,
            Err(broken) => return Some(Err(Failure::Broken(broken))),
        };
        Some(match history {
            History::Static { parent, placement } => Ok((parent, *placement)),
            History::Moving(cache) => {
                cache
                    .sample(at)
                    .map_err(|NoValue { requested, held }| Failure::NoValue {
                        newest: Some(cache.newest().stamp),
                        error: LookupError::Extrapolation {
                            transform: TransformName {
                                parent: cache.newest().parent.clone(),
                                child: child.clone(),
                            },
                            requested: LookupTime::At(requested),
                            held,
                        },
                    })
            }
            History::Empty { parent } => Err(Failure::NoValue {
                newest: None,
                error: LookupError::Extrapolation {
                    transform: TransformName {
                        parent: parent.clone(),
                        child: child.clone(),
                    },
                    requested: at,
                    held: HeldStamps::Empty,
                },
            }),
        })
    }

    /// The newest of the transform of `child`. None for a frame without a transform, and for
    /// a moving transform that holds no stamp, which gives its child no parent at the latest
    /// time, as tf2's empty cache gives none (cache.cpp:303-307, buffer_core.cpp:1147-1150).
    fn latest_of(&self, child: &FrameName) -> Option<Result<Newest<'_>, BrokenTree>> {
        match self.history_of(child)? {
            Err(broken) => Some(Err(broken)),
            Ok(History::Empty { .. }) => None,
            Ok(History::Static { parent, .. }) => Some(Ok(Newest {
                stamp: None,
                parent,
            })),
            Ok(History::Moving(cache)) => Some(Ok(Newest {
                stamp: Some(cache.newest().stamp),
                parent: &cache.newest().parent,
            })),
        }
    }

    /// `walkToTopParent` with `TransformAccum` (buffer_core.cpp:330-596): the source's chain
    /// read up to its root, or to the first transform it cannot read; the target's chain
    /// read up to where the source's stopped; the two composed. A walk stops at a broken
    /// tree as at a transform with no value, so a lookup fails on it only where it needs it,
    /// and the target's failure goes through `preferred` against the source's.
    fn walk(
        &self,
        target: &FrameName,
        source: &FrameName,
        at: LookupTime,
    ) -> Result<Placement, LookupError> {
        let mut accumulated = Accumulated::default();

        let mut frame = source;
        let mut visited = Vec::new();
        let mut stopped = None;
        let top_parent = loop {
            let (parent, placement) = match self.transform_at(frame, at) {
                None => break frame,
                Some(Err(failure)) => {
                    stopped = Some(failure);
                    break frame;
                }
                Some(Ok(transform)) => transform,
            };
            if frame == target {
                return Ok(accumulated.source.placement());
            }
            accumulated.source.up(placement);
            frame = step_up(&mut visited, frame, parent)?;
        };

        let mut frame = target;
        let mut visited = Vec::new();
        while frame != top_parent {
            let (parent, placement) = match self.transform_at(frame, at) {
                None => break,
                Some(Err(failure)) => return Err(preferred(stopped, failure)),
                Some(Ok(transform)) => transform,
            };
            if frame == source {
                return Ok(accumulated.target.inverse());
            }
            accumulated.target.up(placement);
            frame = match step_up(&mut visited, frame, parent) {
                Ok(parent) => parent,
                Err(cycle) => return Err(preferred(stopped, Failure::Broken(cycle))),
            };
        }
        if frame != top_parent {
            return Err(stopped.map_or_else(
                || LookupError::NoChain {
                    target_frame: target.clone(),
                    source_frame: source.clone(),
                },
                Failure::into_error,
            ));
        }
        Ok(accumulated.full_path())
    }

    /// `getLatestCommonTime` (buffer_core.cpp:1112-1261): the earliest of the newest stamps
    /// of the moving transforms on both chains up to their nearest common ancestor; none
    /// when every transform on them is static. The source's chain stops at a broken tree,
    /// and a lookup fails on it only where the target's chain does not meet the source's
    /// below it; the target's failure goes through `preferred` against the source's.
    fn latest_common_time(
        &self,
        target: &FrameName,
        source: &FrameName,
    ) -> Result<Option<Stamp>, LookupError> {
        let mut source_chain: Vec<Newest<'_>> = Vec::new();
        let mut common = None;
        let mut frame = source;
        let mut visited = Vec::new();
        let mut stopped = None;
        while let Some(latest) = self.latest_of(frame) {
            let newest = match latest {
                Ok(newest) => newest,
                Err(broken) => {
                    stopped = Some(Failure::Broken(broken));
                    break;
                }
            };
            common = earlier(common, newest.stamp);
            let parent = newest.parent;
            source_chain.push(newest);
            frame = step_up(&mut visited, frame, parent)?;
            if frame == target {
                return Ok(common);
            }
        }

        let mut common = None;
        let mut frame = target;
        let mut visited = Vec::new();
        let mut common_parent = None;
        while let Some(latest) = self.latest_of(frame) {
            let Newest { stamp, parent } = match latest {
                Ok(newest) => newest,
                Err(broken) => return Err(preferred(stopped, Failure::Broken(broken))),
            };
            common = earlier(common, stamp);
            if source_chain.iter().any(|held| held.parent == parent) {
                common_parent = Some(parent);
                break;
            }
            frame = match step_up(&mut visited, frame, parent) {
                Ok(parent) => parent,
                Err(cycle) => return Err(preferred(stopped, Failure::Broken(cycle))),
            };
            if frame == source {
                return Ok(common);
            }
        }
        let Some(common_parent) = common_parent else {
            return Err(stopped.map_or_else(
                || LookupError::NoChain {
                    target_frame: target.clone(),
                    source_frame: source.clone(),
                },
                Failure::into_error,
            ));
        };
        for held in &source_chain {
            common = earlier(common, held.stamp);
            if held.parent == common_parent {
                break;
            }
        }
        Ok(common)
    }
}

/// Gives `publisher`'s entry among `entries` the history `value` makes: `update` changes
/// the one it holds, else `fresh` makes a new entry.
fn upsert<T>(
    entries: &mut Vec<Entry>,
    publisher: &Publisher,
    value: T,
    fresh: impl FnOnce(T) -> History,
    update: impl FnOnce(&mut History, T),
) {
    match entries
        .iter_mut()
        .find(|entry| entry.publisher == *publisher)
    {
        Some(entry) => update(&mut entry.history, value),
        None => entries.push(Entry {
            publisher: publisher.clone(),
            history: fresh(value),
        }),
    }
}

/// The newest stamp of the moving transforms among `entries`.
fn newest_moving(entries: &[Entry]) -> Option<Stamp> {
    entries
        .iter()
        .filter_map(|entry| match &entry.history {
            History::Moving(cache) => Some(cache.newest().stamp),
            History::Static { .. } | History::Empty { .. } => None,
        })
        .max()
}

/// The step of a walk from `frame` to its `parent`, refused where the parent is a frame the
/// walk passed: the transforms from that frame up to `frame` and back to it form a cycle.
/// tf2 walks on until its depth limit (buffer_core.cpp:398-407, 469-478, 1169-1177,
/// 1223-1231).
fn step_up<'a>(
    visited: &mut Vec<&'a FrameName>,
    frame: &'a FrameName,
    parent: &'a FrameName,
) -> Result<&'a FrameName, BrokenTree> {
    if let Some(start) = visited.iter().position(|passed| *passed == parent) {
        let ring: Vec<&FrameName> = visited[start..].iter().copied().chain([frame]).collect();
        let transforms = (0..ring.len())
            .map(|i| TransformName {
                parent: ring[(i + 1) % ring.len()].clone(),
                child: ring[i].clone(),
            })
            .collect();
        return Err(BrokenTree::Cycle { transforms });
    }
    visited.push(frame);
    Ok(parent)
}

/// Keeps the entries of one child frame that are held: each static transform, until its
/// publisher's next static set leaves it out, and each moving transform whose newest stamp
/// lies within `window` of the newest stamp of the child frame's moving transforms. An
/// emptied transform holds no stamp.
fn drop_unheld(entries: &mut Vec<Entry>, window: u64) {
    let Some(newest) = newest_moving(entries) else {
        return;
    };
    entries.retain(|entry| match &entry.history {
        History::Static { .. } => true,
        History::Moving(cache) => {
            cache.newest().stamp.as_nanos().saturating_add(window) >= newest.as_nanos()
        }
        History::Empty { .. } => false,
    });
}

/// The error of a lookup whose target's chain failed at `target_side`, after the source's
/// stopped at `source_side`, as the walks meet them: the source's first, but where both are
/// missing values, the one whose transform holds the newer samples, the source's on a tie,
/// as tf2 weighs them (buffer_core.cpp:426-445).
fn preferred(source_side: Option<Failure>, target_side: Failure) -> LookupError {
    match (source_side, target_side) {
        (None, target_side) => target_side.into_error(),
        (
            Some(Failure::NoValue {
                newest: source_newest,
                error: source_error,
            }),
            Failure::NoValue {
                newest: target_newest,
                error: target_error,
            },
        ) => {
            if source_newest >= target_newest {
                source_error
            } else {
                target_error
            }
        }
        (Some(source_side), _) => source_side.into_error(),
    }
}

/// The earlier of two stamps, none standing for a static transform, which tf2 leaves out
/// of the latest common time (buffer_core.cpp:1152-1154).
fn earlier(common: Option<Stamp>, stamp: Option<Stamp>) -> Option<Stamp> {
    match (common, stamp) {
        (Some(common), Some(stamp)) => Some(common.min(stamp)),
        (common, stamp) => common.or(stamp),
    }
}

/// The transforms a walk composed from a frame up its chain, as `TransformAccum` holds
/// them for each chain (buffer_core.cpp:524-534).
struct Chain {
    rotation: Quaternion,
    position: Vector,
}

impl Default for Chain {
    fn default() -> Self {
        Self {
            rotation: arithmetic::IDENTITY,
            position: [0.0; 3],
        }
    }
}

impl Chain {
    /// `accum` (buffer_core.cpp:547-556): the chain carried up one transform.
    fn up(&mut self, transform: Placement) {
        self.position = arithmetic::sum(
            arithmetic::rotate(transform.orientation(), self.position),
            transform.position(),
        );
        self.rotation = arithmetic::product(transform.orientation(), self.rotation);
    }

    /// The chain as composed, `finalize(TargetParentOfSource)` of the source's chain
    /// (buffer_core.cpp:563-566).
    fn placement(&self) -> Placement {
        Placement::new(self.position, self.rotation)
    }

    /// The chain's inverse, `inv_target_quat` and `inv_target_vec`, which is
    /// `finalize(SourceParentOfTarget)` of the target's chain (buffer_core.cpp:567-574).
    fn inverse(&self) -> Placement {
        let rotation = arithmetic::inverse(self.rotation);
        Placement::new(
            arithmetic::rotate(rotation, arithmetic::negated(self.position)),
            rotation,
        )
    }
}

/// Both chains of a walk.
#[derive(Default)]
struct Accumulated {
    source: Chain,
    target: Chain,
}

impl Accumulated {
    /// `finalize(FullPath)` (buffer_core.cpp:575-583).
    fn full_path(&self) -> Placement {
        let inverse = self.target.inverse();
        Placement::new(
            arithmetic::sum(
                arithmetic::rotate(inverse.orientation(), self.source.position),
                inverse.position(),
            ),
            arithmetic::product(inverse.orientation(), self.source.rotation),
        )
    }
}
