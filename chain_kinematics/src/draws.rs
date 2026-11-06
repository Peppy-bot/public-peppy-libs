//! Seeded draws: SplitMix64, so a seed gives the same numbers on every machine and in
//! every release.

use crate::Limit;
use crate::ik::{RandomSource, draw_within};

/// A stream of draws from one seed.
pub struct Draws(u64);

impl Draws {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    /// The next 64 bits of the stream.
    pub fn next_bits(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A number in `[0, 1)`, from the top 53 bits of the next draw.
    pub fn unit(&mut self) -> f64 {
        (self.next_bits() >> 11) as f64 / (1_u64 << 53) as f64
    }

    /// A number in `[low, high)`.
    pub fn between(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.unit()
    }

    /// A configuration inside `limits`, each joint drawn in turn.
    pub fn within<const N: usize>(&mut self, limits: &[Limit; N]) -> [f64; N] {
        draw_within(self, limits)
    }
}

impl RandomSource for Draws {
    fn between(&mut self, low: f64, high: f64) -> f64 {
        Self::between(self, low, high)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // As a random source, the draws give the numbers they give of their own, in the order
    // they give them: a configuration draws each joint in turn.
    #[test]
    fn a_random_source_draws_the_numbers_of_the_draws() {
        let limits = [
            Limit { lo: -1.0, hi: 2.0 },
            Limit { lo: 0.5, hi: 0.75 },
            Limit { lo: -3.0, hi: -2.0 },
        ];
        let (mut source, mut own) = (Draws::new(7), Draws::new(7));
        assert_eq!(
            RandomSource::between(&mut source, -1.0, 1.0),
            own.between(-1.0, 1.0)
        );
        assert_eq!(
            source.within(&limits),
            limits.map(|limit| own.between(limit.lo, limit.hi))
        );
    }
}
