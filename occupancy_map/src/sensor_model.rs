//! OctoMap's sensor model, as MoveIt's occupancy map keeps it (octomap
//! 1.10's defaults): each update adds the log-odds of a hit or a miss to a
//! voxel's log-odds in single precision, clamped between two bounds, and a
//! voxel at or above even odds is occupied.

/// The probability a hit says the voxel is occupied.
const PROB_HIT: f64 = 0.7;
/// The probability a miss says the voxel is occupied.
const PROB_MISS: f64 = 0.4;
/// The lowest probability a voxel holds.
const CLAMP_MIN: f64 = 0.1192;
/// The highest probability a voxel holds.
const CLAMP_MAX: f64 = 0.971;

/// OctoMap's `logodds`: the log of the odds of `probability`, taken in double
/// and kept in single.
fn log_odds(probability: f64) -> f32 {
    (probability / (1.0 - probability)).ln() as f32
}

/// The log-odds a hit, a miss and the cut-out robot add, and the bounds a
/// voxel's log-odds stays between.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SensorModel {
    pub hit: f32,
    pub miss: f32,
    /// What a voxel inside the robot adds: enough to take any log-odds to
    /// the lower bound, as MoveIt's updater sets it.
    pub cut_out: f32,
    /// What a voxel a collision object fills adds: enough to take any log-odds to the
    /// upper bound.
    pub filled: f32,
    min: f32,
    max: f32,
}

impl SensorModel {
    pub fn octomap() -> Self {
        let (min, max) = (log_odds(CLAMP_MIN), log_odds(CLAMP_MAX));
        Self {
            hit: log_odds(PROB_HIT),
            miss: log_odds(PROB_MISS),
            cut_out: min - max,
            filled: max - min,
            min,
            max,
        }
    }

    /// A voxel's log-odds after `update`, from `current`, none for a voxel no
    /// ray has reached: OctoMap's `updateNode`, which leaves a voxel at a
    /// bound that the update pushes against, and otherwise adds the update
    /// to its log-odds, 0 for a new voxel, and clamps the sum.
    pub fn updated(&self, current: Option<f32>, update: f32) -> f32 {
        if let Some(value) = current
            && ((update >= 0.0 && value >= self.max) || (update <= 0.0 && value <= self.min))
        {
            return value;
        }
        let value = current.unwrap_or(0.0) + update;
        if value < self.min {
            self.min
        } else if value > self.max {
            self.max
        } else {
            value
        }
    }
}

/// Whether a voxel of log-odds `value` is occupied: at or above even odds,
/// log-odds 0.
pub(crate) fn occupied(value: f32) -> bool {
    value >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    // Hits raise a voxel to the upper bound and misses lower it to the lower
    // one, each step OctoMap's; the cut-out takes any voxel to the lower
    // bound at once.
    #[test]
    fn updates_walk_a_voxel_between_its_bounds() {
        let model = SensorModel::octomap();
        assert!((model.hit - 0.847_297_9).abs() < 1e-6);
        assert!((model.miss + 0.405_465_1).abs() < 1e-6);
        let mut value = None;
        for _ in 0..10 {
            value = Some(model.updated(value, model.hit));
        }
        assert_eq!(value, Some(model.max));
        assert!(occupied(model.max));
        for _ in 0..20 {
            value = Some(model.updated(value, model.miss));
        }
        assert_eq!(value, Some(model.min));
        assert!(!occupied(model.min));
        assert_eq!(model.updated(Some(model.max), model.cut_out), model.min);
        assert_eq!(model.updated(None, model.cut_out), model.min);
        assert_eq!(model.updated(None, model.hit), model.hit);
    }
}
