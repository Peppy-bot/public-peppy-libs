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
// This file translates into Rust tf2's TimeCache and StaticCache (geometry2 0.45.10):
// tf2/src/cache.cpp, tf2/src/static_cache.cpp and tf2/include/tf2/time_cache.hpp.

//! The samples of one child frame's moving transform from one publisher within the tree's
//! window, newest first, as tf2's `TimeCache` keeps them in a list sorted newest first
//! (cache.cpp:246-291), and when the transform last started over.

use std::collections::VecDeque;

use crate::arithmetic;
use crate::frame::{FrameName, LookupTime, Stamp, Start};
use crate::transforms::Placement;

/// One sample of a moving transform: its parent and its placement at its stamp.
#[derive(Clone, Debug)]
pub(crate) struct Sample {
    pub stamp: Stamp,
    pub parent: FrameName,
    pub placement: Placement,
}

/// The stamps a moving transform holds: tf2 tells one sample apart from a range
/// (cache.cpp:134-165). A transform whose samples a clear dropped holds none, as tf2's
/// cleared cache holds none (cache.cpp:122-126, 293-296).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HeldStamps {
    Empty,
    One(Stamp),
    Range { oldest: Stamp, newest: Stamp },
}

/// A time a moving transform has no value at: before its oldest sample, past its newest, or
/// other than the stamp of its only sample.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct NoValue {
    pub requested: Stamp,
    pub held: HeldStamps,
}

/// The samples of a moving transform, newest first, never empty: tf2 allocates a frame's
/// cache as it inserts the frame's first sample (buffer_core.cpp:273-293). `start` is when
/// the transform last started over, as its publisher gave it.
#[derive(Clone, Debug)]
pub(crate) struct TimeCache {
    samples: VecDeque<Sample>,
    window_nanos: u64,
    start: Start,
}

/// The samples a time falls on or between (`findClosest`, cache.cpp:114-181).
enum Closest<'a> {
    One(&'a Sample),
    Two {
        older: &'a Sample,
        newer: &'a Sample,
    },
}

impl TimeCache {
    pub fn new(first: Sample, window_nanos: u64, start: Start) -> Self {
        Self {
            samples: VecDeque::from([first]),
            window_nanos,
            start,
        }
    }

    pub fn newest(&self) -> &Sample {
        self.samples.front().expect("a time cache holds a sample")
    }

    fn oldest(&self) -> &Sample {
        self.samples.back().expect("a time cache holds a sample")
    }

    /// `findClosest` (cache.cpp:114-181). tf2 marks the latest time with the time point
    /// zero (cache.cpp:128-132); a stamp of zero is a time like any other here.
    fn closest(&self, at: LookupTime) -> Result<Closest<'_>, NoValue> {
        let LookupTime::At(time) = at else {
            return Ok(Closest::One(self.newest()));
        };
        if self.samples.len() == 1 {
            let only = self.newest();
            return if only.stamp == time {
                Ok(Closest::One(only))
            } else {
                Err(NoValue {
                    requested: time,
                    held: HeldStamps::One(only.stamp),
                })
            };
        }
        let (newest, oldest) = (self.newest(), self.oldest());
        if time == newest.stamp {
            return Ok(Closest::One(newest));
        }
        if time == oldest.stamp {
            return Ok(Closest::One(oldest));
        }
        if time > newest.stamp || time < oldest.stamp {
            return Err(NoValue {
                requested: time,
                held: HeldStamps::Range {
                    oldest: oldest.stamp,
                    newest: newest.stamp,
                },
            });
        }
        let older = self
            .samples
            .iter()
            .position(|sample| sample.stamp <= time)
            .expect("a time within the samples has a sample at or before it");
        Ok(Closest::Two {
            older: &self.samples[older],
            newer: &self.samples[older - 1],
        })
    }

    /// `getData` (cache.cpp:207-230): the parent and the placement at `at`, interpolated
    /// between the two samples around it when both have one parent, else the older's.
    pub fn sample(&self, at: LookupTime) -> Result<(&FrameName, Placement), NoValue> {
        match self.closest(at)? {
            Closest::One(sample) => Ok((&sample.parent, sample.placement)),
            Closest::Two { older, newer } if older.parent == newer.parent => {
                let LookupTime::At(time) = at else {
                    unreachable!("the latest time falls on the newest sample")
                };
                Ok((&older.parent, interpolate(older, newer, time)))
            }
            Closest::Two { older, .. } => Ok((&older.parent, older.placement)),
        }
    }

    /// When the transform last started over, as its publisher gave it.
    pub fn start(&self) -> Start {
        self.start
    }

    /// Inserts `sample` in order among the samples, the step of `insertData` after its
    /// prune and its refusal (cache.cpp:266-288); a sample of a held stamp replaces it,
    /// where tf2 keeps both (cache.cpp:274-288).
    pub fn insert(&mut self, sample: Sample) {
        let position = self
            .samples
            .iter()
            .position(|held| held.stamp <= sample.stamp)
            .unwrap_or(self.samples.len());
        match self.samples.get_mut(position) {
            Some(held) if held.stamp == sample.stamp => *held = sample,
            _ => self.samples.insert(position, sample),
        }
    }

    /// `pruneList` (cache.cpp:336-343): drops each oldest sample whose stamp plus the
    /// window is before `latest`, the newest stamp held of the child frame from any
    /// publisher. Gives whether a sample is left; a cache left with none is its owner's to
    /// drop.
    #[must_use]
    pub fn prune(&mut self, latest: Stamp) -> bool {
        while self.samples.back().is_some_and(|oldest| {
            oldest.stamp.as_nanos().saturating_add(self.window_nanos) < latest.as_nanos()
        }) {
            self.samples.pop_back();
        }
        !self.samples.is_empty()
    }
}

/// `interpolate` (cache.cpp:183-205): the position blended linearly and the orientation by
/// tf2's slerp, by the fraction of the time between the two stamps. Its zero-distance
/// branch, two samples of one stamp, is not reached from `getData`: the newer sample's
/// stamp is after the time and the older's at or before it.
fn interpolate(older: &Sample, newer: &Sample, time: Stamp) -> Placement {
    // Each count is below 2^63 nanoseconds, so it converts to f64 as tf2's int64 count does.
    let ratio = (time.as_nanos() - older.stamp.as_nanos()) as f64
        / (newer.stamp.as_nanos() - older.stamp.as_nanos()) as f64;
    Placement::new(
        arithmetic::interpolated(
            older.placement.position(),
            newer.placement.position(),
            ratio,
        ),
        arithmetic::slerp(
            older.placement.orientation(),
            newer.placement.orientation(),
            ratio,
        ),
    )
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, SystemTime};

    use super::*;

    const WINDOW: u64 = 10;

    fn frame(name: &str) -> FrameName {
        FrameName::parse(name).unwrap()
    }

    fn sample(stamp: u64, x: f64) -> Sample {
        sample_of(stamp, "p", x)
    }

    fn sample_of(stamp: u64, parent: &str, x: f64) -> Sample {
        Sample {
            stamp: Stamp::from_nanos(stamp),
            parent: frame(parent),
            placement: Placement::new([x, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        }
    }

    fn start(seconds: u64) -> Start {
        Start::from_time(SystemTime::UNIX_EPOCH + Duration::from_secs(seconds)).unwrap()
    }

    /// The start, in s after the Unix epoch, of every sample of a test but a start over's.
    const START: u64 = 0;

    /// Prunes against the cache's newest and inserts `sample`, as the tree does for a
    /// child frame of one publisher.
    fn take(cache: &mut TimeCache, sample: Sample) {
        let latest = cache.newest().stamp;
        assert!(cache.prune(latest));
        cache.insert(sample);
    }

    fn cache(samples: &[Sample]) -> TimeCache {
        let mut cache = TimeCache::new(samples[0].clone(), WINDOW, start(START));
        for sample in &samples[1..] {
            take(&mut cache, sample.clone());
        }
        cache
    }

    fn stamps(cache: &TimeCache) -> Vec<u64> {
        cache.samples.iter().map(|s| s.stamp.as_nanos()).collect()
    }

    fn x_at(cache: &TimeCache, time: u64) -> Result<f64, NoValue> {
        cache
            .sample(LookupTime::At(Stamp::from_nanos(time)))
            .map(|(_, placement)| placement.position()[0])
    }

    #[test]
    fn one_sample_holds_at_its_stamp_alone() {
        let cache = cache(&[sample(5, 1.0)]);
        assert_eq!(x_at(&cache, 5), Ok(1.0));
        assert_eq!(
            cache.sample(LookupTime::Latest).unwrap().1.position()[0],
            1.0
        );
        let held = HeldStamps::One(Stamp::from_nanos(5));
        for time in [4, 6] {
            assert_eq!(
                x_at(&cache, time),
                Err(NoValue {
                    requested: Stamp::from_nanos(time),
                    held
                })
            );
        }
    }

    #[test]
    fn a_time_between_two_samples_interpolates_and_none_is_held_outside() {
        let cache = cache(&[sample(2, 0.0), sample(6, 4.0)]);
        assert_eq!(x_at(&cache, 3), Ok(1.0));
        assert_eq!(x_at(&cache, 2), Ok(0.0));
        assert_eq!(x_at(&cache, 6), Ok(4.0));
        let held = HeldStamps::Range {
            oldest: Stamp::from_nanos(2),
            newest: Stamp::from_nanos(6),
        };
        assert_eq!(x_at(&cache, 1).unwrap_err().held, held);
        assert_eq!(x_at(&cache, 7).unwrap_err().held, held);
    }

    #[test]
    fn a_parent_change_between_two_samples_gives_the_older() {
        let cache = cache(&[sample_of(2, "a", 0.0), sample_of(6, "b", 4.0)]);
        let (parent, placement) = cache.sample(LookupTime::At(Stamp::from_nanos(4))).unwrap();
        assert_eq!(*parent, frame("a"));
        assert_eq!(placement.position()[0], 0.0);
    }

    #[test]
    fn samples_insert_in_order_and_one_of_a_held_stamp_replaces_it() {
        let mut cache = cache(&[sample(4, 4.0), sample(2, 2.0), sample(8, 8.0)]);
        assert_eq!(stamps(&cache), [8, 4, 2]);
        take(&mut cache, sample(4, 5.0));
        let xs: Vec<f64> = cache
            .samples
            .iter()
            .map(|s| s.placement.position()[0])
            .collect();
        assert_eq!(xs, [8.0, 5.0, 2.0]);
    }

    #[test]
    fn a_prune_drops_the_samples_past_the_window_of_the_latest_stamp() {
        let mut cache = cache(&[sample(0, 0.0), sample(10, 1.0), sample(20, 2.0)]);
        // 0 + 10 is before 20; 10 + 10 is not.
        assert!(cache.prune(Stamp::from_nanos(20)));
        assert_eq!(stamps(&cache), [20, 10]);
        // Against another publisher's newer stamp the cache can be left with none.
        assert!(!cache.prune(Stamp::from_nanos(31)));
    }

    #[test]
    fn the_start_is_the_one_the_cache_was_made_with() {
        let cache = TimeCache::new(sample(1, 0.0), WINDOW, start(9));
        assert_eq!(cache.start(), start(9));
    }
}
