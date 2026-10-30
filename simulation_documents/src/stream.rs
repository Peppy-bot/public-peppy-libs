//! The streams that a seed gives the samplers of a world pattern.
//!
//! Each sampler draws from its own stream: the seed and the path of the
//! sampler in the concrete world, such as `objects.stock_2.pose`, give it. So
//! an added entry changes no other value, and the same pattern and seed give
//! the same values. A stream is xoshiro256** from the sha256 of the seed and
//! the path, written here so that no update of a dependency changes a draw.

use serde::{Deserialize, Serialize};
use sha2::{Digest as _, Sha256};
use std::fmt;

use crate::SitePath;

/// The seed of an expansion.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Seed(pub u32);

impl fmt::Display for Seed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// What a stream hashes first, so that no other sha256 of the same bytes
/// gives its state.
const DOMAIN: &[u8] = b"simulation_documents world pattern stream v1\0";

/// The values one sampler draws, in order.
#[derive(Clone, Debug)]
pub struct Stream {
    state: [u64; 4],
}

impl Stream {
    /// The stream of the sampler at `path` for `seed`.
    pub fn new(seed: Seed, path: &SitePath) -> Stream {
        let mut hasher = Sha256::new();
        hasher.update(DOMAIN);
        hasher.update(seed.0.to_le_bytes());
        hasher.update(path.to_string().as_bytes());
        let digest: [u8; 32] = hasher.finalize().into();
        let (words, _) = digest.as_chunks::<8>();
        let mut state = [0u64; 4];
        for (word, bytes) in state.iter_mut().zip(words) {
            *word = u64::from_le_bytes(*bytes);
        }
        // xoshiro256** has no state of all zeros; sha256 gives it with a
        // chance of 2^-256.
        if state == [0; 4] {
            state[0] = 1;
        }
        Stream { state }
    }

    /// The next 64 bits of the stream.
    pub fn next_u64(&mut self) -> u64 {
        let [s0, s1, s2, s3] = &mut self.state;
        let result = s1.wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = *s1 << 17;
        *s2 ^= *s0;
        *s3 ^= *s1;
        *s1 ^= *s2;
        *s0 ^= *s3;
        *s2 ^= t;
        *s3 = s3.rotate_left(45);
        result
    }

    /// A uniform number from 0, included, to 1, excluded, with 53 bits.
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// A uniform number from `min` to `max`; `min` when the two are equal.
    pub fn uniform(&mut self, min: f64, max: f64) -> f64 {
        let drawn = min + (max - min) * self.unit();
        drawn.clamp(min, max)
    }

    /// A uniform integer from 0 to `count - 1`, by rejection, so that each
    /// has the same chance. `count` is at least 1.
    pub fn below(&mut self, count: u64) -> u64 {
        assert!(count > 0, "a draw below 0 has no value");
        let zone = u64::MAX - (u64::MAX - count + 1) % count;
        loop {
            let drawn = self.next_u64();
            if drawn <= zone {
                return drawn % count;
            }
        }
    }

    /// A uniform integer from `min` to `max`, both included.
    pub fn integer(&mut self, min: u64, max: u64) -> u64 {
        assert!(min <= max, "a range ends at or above its start");
        match (max - min).checked_add(1) {
            Some(count) => min + self.below(count),
            None => self.next_u64(),
        }
    }

    /// The index of one of `len` values, with equal chances.
    pub fn index(&mut self, len: usize) -> usize {
        self.below(len as u64) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sampler::Segment;

    fn path(text: &str) -> SitePath {
        SitePath::new(
            text.split('.')
                .map(|key| Segment::Key(key.to_owned()))
                .collect(),
        )
    }

    /// The generator is xoshiro256**: from the state 1, 2, 3, 4 it gives the
    /// values of its reference implementation.
    #[test]
    fn the_generator_gives_the_values_of_xoshiro256_star_star() {
        let mut stream = Stream {
            state: [1, 2, 3, 4],
        };
        let first: Vec<u64> = (0..3).map(|_| stream.next_u64()).collect();
        assert_eq!(first, [11_520, 0, 1_509_978_240]);
    }

    /// A draw never changes: the same pattern and seed give the same world in
    /// every build.
    #[test]
    fn a_stream_gives_the_values_it_always_gave() {
        let mut stream = Stream::new(Seed(1), &path("objects.stock.count"));
        let first: Vec<u64> = (0..3).map(|_| stream.next_u64()).collect();
        assert_eq!(
            first,
            [
                2_652_278_581_755_479_919,
                11_364_788_394_673_127_564,
                2_438_604_551_530_442_657
            ]
        );
    }

    #[test]
    fn each_seed_and_path_gives_its_own_stream() {
        let draw = |seed: u32, at: &str| Stream::new(Seed(seed), &path(at)).next_u64();
        assert_eq!(draw(7, "scene"), draw(7, "scene"));
        assert_ne!(draw(7, "scene"), draw(8, "scene"));
        assert_ne!(draw(7, "scene"), draw(7, "objects.crate.mass"));
    }

    #[test]
    fn draws_stay_within_their_bounds() {
        let mut stream = Stream::new(Seed(3), &path("objects.crate.mass"));
        for _ in 0..1000 {
            let number = stream.uniform(0.5, 2.0);
            assert!((0.5..=2.0).contains(&number), "{number}");
            let integer = stream.integer(3, 6);
            assert!((3..=6).contains(&integer), "{integer}");
            assert!(stream.index(4) < 4);
        }
        assert_eq!(stream.uniform(1.5, 1.5), 1.5);
        assert_eq!(stream.integer(9, 9), 9);
        let mut seen = [false; 4];
        for _ in 0..200 {
            seen[stream.integer(3, 6) as usize - 3] = true;
        }
        assert_eq!(seen, [true; 4], "each integer of the range comes up");
    }
}
