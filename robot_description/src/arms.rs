//! The arms of a robot of `K` arms: an arm by its place among its description's arms
//! ([`RobotDescription::arms`](crate::RobotDescription::arms)), and a value for each arm,
//! held in place, so a control loop reads and writes one with no allocation.

use std::ops::{Index, IndexMut};

/// An arm of a robot, by its place among its description's arms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ArmId(usize);

impl ArmId {
    pub const fn new(place: usize) -> Self {
        Self(place)
    }

    /// The arm's place among its description's arms.
    pub const fn place(self) -> usize {
        self.0
    }
}

/// A value for each of the `K` arms of a robot, in the order of its description's arms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PerArm<T, const K: usize>([T; K]);

impl<T, const K: usize> PerArm<T, K> {
    /// The values of the arms, in the order of the description's arms.
    pub const fn new(values: [T; K]) -> Self {
        Self(values)
    }

    /// A value for each arm, from the arm.
    pub fn from_fn(mut value: impl FnMut(ArmId) -> T) -> Self {
        Self(std::array::from_fn(|place| value(ArmId(place))))
    }

    /// The robot's arms, in the order of its description's arms.
    pub fn ids() -> impl Iterator<Item = ArmId> {
        (0..K).map(ArmId)
    }

    pub fn iter(&self) -> std::slice::Iter<'_, T> {
        self.0.iter()
    }

    /// Each value mapped, with its arm.
    pub fn map<U>(self, mut f: impl FnMut(ArmId, T) -> U) -> PerArm<U, K> {
        let mut place = 0;
        PerArm(self.0.map(|value| {
            let mapped = f(ArmId(place), value);
            place += 1;
            mapped
        }))
    }

    /// The values, in the order of the description's arms.
    pub fn into_array(self) -> [T; K] {
        self.0
    }
}

impl<A, B, const K: usize> PerArm<(A, B), K> {
    /// The first and the second of each arm's pair.
    pub fn unzip(self) -> (PerArm<A, K>, PerArm<B, K>) {
        let pairs = self.0;
        let mut firsts: [Option<A>; K] = std::array::from_fn(|_| None);
        let mut seconds: [Option<B>; K] = std::array::from_fn(|_| None);
        for (place, (a, b)) in pairs.into_iter().enumerate() {
            firsts[place] = Some(a);
            seconds[place] = Some(b);
        }
        (
            PerArm(firsts.map(|a| a.expect("a value for each arm"))),
            PerArm(seconds.map(|b| b.expect("a value for each arm"))),
        )
    }
}

impl<T, const K: usize> PerArm<Option<T>, K> {
    /// The values, where every arm has one.
    pub fn transpose(self) -> Option<PerArm<T, K>> {
        if self.0.iter().any(Option::is_none) {
            return None;
        }
        Some(PerArm(
            self.0.map(|value| value.expect("every arm has one")),
        ))
    }
}

impl<T, E, const K: usize> PerArm<Result<T, E>, K> {
    /// The values, or the first arm's error.
    pub fn transpose(self) -> Result<PerArm<T, K>, E> {
        let mut values: [Option<T>; K] = std::array::from_fn(|_| None);
        for (place, value) in self.0.into_iter().enumerate() {
            values[place] = Some(value?);
        }
        Ok(PerArm(
            values.map(|value| value.expect("a value for each arm")),
        ))
    }
}

impl<T, const K: usize> Index<ArmId> for PerArm<T, K> {
    type Output = T;

    fn index(&self, arm: ArmId) -> &T {
        &self.0[arm.0]
    }
}

impl<T, const K: usize> IndexMut<ArmId> for PerArm<T, K> {
    fn index_mut(&mut self, arm: ArmId) -> &mut T {
        &mut self.0[arm.0]
    }
}

impl<T, const K: usize> IntoIterator for PerArm<T, K> {
    type Item = T;
    type IntoIter = std::array::IntoIter<T, K>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

impl<'a, T, const K: usize> IntoIterator for &'a PerArm<T, K> {
    type Item = &'a T;
    type IntoIter = std::slice::Iter<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEFT: ArmId = ArmId::new(0);
    const RIGHT: ArmId = ArmId::new(1);

    // Each arm's value stands at the arm's place, and a map, a zip and a transpose keep it
    // there.
    #[test]
    fn each_value_stands_at_its_arms_place() {
        let mut values = PerArm::new([1, 2]);
        values[RIGHT] = 5;
        assert_eq!((values[LEFT], values[RIGHT]), (1, 5));
        assert_eq!(PerArm::<(), 2>::ids().collect::<Vec<_>>(), [LEFT, RIGHT]);
        assert_eq!(
            values
                .map(|arm, value| (arm.place(), value * 2))
                .into_array(),
            [(0, 2), (1, 10)]
        );
        assert_eq!(PerArm::from_fn(ArmId::place), PerArm::new([0, 1]));
        let (firsts, seconds) = PerArm::new([(1, 'a'), (2, 'b')]).unzip();
        assert_eq!(
            (firsts, seconds),
            (PerArm::new([1, 2]), PerArm::new(['a', 'b']))
        );
        assert_eq!(
            PerArm::new([Some(1), Some(2)]).transpose(),
            Some(PerArm::new([1, 2]))
        );
        assert_eq!(PerArm::new([Some(1), None]).transpose(), None);
        assert_eq!(
            PerArm::new([Err::<i32, _>("first"), Err("second")]).transpose(),
            Err("first")
        );
        assert_eq!(
            PerArm::new([Ok::<_, ()>(1), Ok(2)]).transpose(),
            Ok(PerArm::new([1, 2]))
        );
        assert_eq!((&values).into_iter().copied().sum::<i32>(), 6);
    }
}
