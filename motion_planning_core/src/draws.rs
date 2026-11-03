//! Seeded draws: SplitMix64, so a seed gives the same numbers on every machine and in
//! every release.

pub(crate) struct Draws(u64);

impl Draws {
    pub(crate) fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A number in `[0, 1)`, from the top 53 bits of the next draw.
    pub(crate) fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1_u64 << 53) as f64
    }

    /// A number in `[low, high)`.
    pub(crate) fn between(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.unit()
    }
}

/// One seed from `seed` and `stamp`: the bits of both mixed, never zero.
pub(crate) fn mixed_seed(seed: u32, stamp: u64) -> std::num::NonZeroU32 {
    let mut draws = Draws::new(u64::from(seed) ^ stamp.rotate_left(32));
    let mixed = (draws.next() >> 32) as u32;
    std::num::NonZeroU32::new(mixed).unwrap_or(std::num::NonZeroU32::MIN)
}
