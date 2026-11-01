//! A world of box obstacles and its checks, which the plans of the tests plan through.

use ompl_bridge::{CheckError, Checks, Motion};

/// An axis-aligned box, open: a state on its surface is free.
pub struct Obstacle {
    pub min: Vec<f64>,
    pub max: Vec<f64>,
}

impl Obstacle {
    fn holds(&self, state: &[f64]) -> bool {
        state
            .iter()
            .zip(self.min.iter().zip(&self.max))
            .all(|(x, (lo, hi))| lo < x && x < hi)
    }
}

/// Boxes, and the step a motion is walked in.
pub struct World {
    pub obstacles: Vec<Obstacle>,
    pub motion_step: f64,
}

impl World {
    pub fn free(&self, state: &[f64]) -> bool {
        !self.obstacles.iter().any(|o| o.holds(state))
    }

    /// The motion from `from` to `to` walked in steps of at most the motion step,
    /// each state judged free or not.
    pub fn walk(&self, from: &[f64], to: &[f64]) -> Motion {
        let steps = (distance(from, to) / self.motion_step).ceil().max(1.0) as usize;
        let mut last_valid = 0.0;
        for k in 1..=steps {
            let t = k as f64 / steps as f64;
            let state: Vec<f64> = from.iter().zip(to).map(|(a, b)| a + (b - a) * t).collect();
            if !self.free(&state) {
                return Motion::Invalid { last_valid };
            }
            last_valid = t;
        }
        Motion::Valid
    }
}

impl Checks for World {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        Ok(self.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Ok(self.walk(from, to))
    }
}

/// The Euclidean distance between two states, summed as OMPL's real vector space
/// sums it.
pub fn distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

/// The length of a path, state to state.
pub fn length(path: &[Vec<f64>]) -> f64 {
    path.windows(2).map(|w| distance(&w[0], &w[1])).sum()
}
