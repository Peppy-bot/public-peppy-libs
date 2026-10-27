//! The two arms of the bimanual robot and a value held once per arm.

use std::fmt;
use std::ops::{Index, IndexMut};

/// An arm side of the bimanual robot, selecting the `openarm_left_*` or
/// `openarm_right_*` chain in the bundled URDF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Deserialize, serde::Serialize),
    serde(rename_all = "lowercase")
)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    /// Both sides, left then right: the order of every per-arm table on the
    /// wire, and the order [`ArmPair::from_fn`] builds a pair in.
    pub const BOTH: [Side; 2] = [Side::Left, Side::Right];

    /// The side's lowercase name: the word the URDF names its chain by, and
    /// how the side is written in logs and messages.
    fn name(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
        }
    }

    /// The side's segment of the bundled URDF's link and joint names.
    pub(crate) fn urdf_prefix(self) -> &'static str {
        self.name()
    }

    /// This side's arm name on the wire: the `arm_name` limb_motion goals
    /// carry and limb_state's `arm_names` reports.
    pub fn arm_name(self) -> &'static str {
        match self {
            Self::Left => "left_arm",
            Self::Right => "right_arm",
        }
    }

    /// This side's gripper name on the wire, in the same role as
    /// [`Self::arm_name`].
    pub fn gripper_name(self) -> &'static str {
        match self {
            Self::Left => "left_gripper",
            Self::Right => "right_gripper",
        }
    }

    /// Parse a wire `arm_name` into the side whose [`Self::arm_name`] it is.
    pub fn from_arm_name(arm_name: &str) -> Result<Self, UnknownArmName> {
        Side::BOTH
            .into_iter()
            .find(|side| side.arm_name() == arm_name)
            .ok_or(UnknownArmName)
    }

    /// Parse a wire `gripper_name` into the side whose [`Self::gripper_name`]
    /// it is.
    pub fn from_gripper_name(gripper_name: &str) -> Result<Self, UnknownGripperName> {
        Side::BOTH
            .into_iter()
            .find(|side| side.gripper_name() == gripper_name)
            .ok_or(UnknownGripperName)
    }
}

impl fmt::Display for Side {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// An `arm_name` no side of this robot carries; its message names the arms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnknownArmName;

impl fmt::Display for UnknownArmName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(r#"unknown arm_name: this robot's arms are "left_arm" and "right_arm""#)
    }
}

impl std::error::Error for UnknownArmName {}

/// A `gripper_name` no side of this robot carries; its message names the
/// grippers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnknownGripperName;

impl fmt::Display for UnknownGripperName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(
            r#"unknown gripper_name: this robot's grippers are "left_gripper" and "right_gripper""#,
        )
    }
}

impl std::error::Error for UnknownGripperName {}

/// A `left`/`right` pair of the same type, one value per arm, read and
/// written by [`Side`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ArmPair<T> {
    pub left: T,
    pub right: T,
}

impl<T> ArmPair<T> {
    pub const fn new(left: T, right: T) -> Self {
        Self { left, right }
    }

    /// The values, in [`Side::BOTH`] order.
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        [&self.left, &self.right].into_iter()
    }

    /// The pair holding `f(side)` for each side, in [`Side::BOTH`] order: `f`
    /// runs for the left side first.
    pub fn from_fn(mut f: impl FnMut(Side) -> T) -> Self {
        let left = f(Side::Left);
        let right = f(Side::Right);
        Self { left, right }
    }

    /// The pair holding `f(side, value)` for each side's value, in
    /// [`Side::BOTH`] order.
    pub fn map<U>(self, mut f: impl FnMut(Side, T) -> U) -> ArmPair<U> {
        let left = f(Side::Left, self.left);
        let right = f(Side::Right, self.right);
        ArmPair { left, right }
    }

    /// The pair holding each side's value of `self` and of `other` together.
    pub fn zip<U>(self, other: ArmPair<U>) -> ArmPair<(T, U)> {
        ArmPair::new((self.left, other.left), (self.right, other.right))
    }
}

impl<T> IntoIterator for ArmPair<T> {
    type Item = (Side, T);
    type IntoIter = std::array::IntoIter<(Side, T), 2>;

    /// Each side with its value, in [`Side::BOTH`] order.
    fn into_iter(self) -> Self::IntoIter {
        [(Side::Left, self.left), (Side::Right, self.right)].into_iter()
    }
}

impl<A, B> ArmPair<(A, B)> {
    /// The pair of first values and the pair of second values.
    pub fn unzip(self) -> (ArmPair<A>, ArmPair<B>) {
        let ArmPair {
            left: (left_a, left_b),
            right: (right_a, right_b),
        } = self;
        (ArmPair::new(left_a, right_a), ArmPair::new(left_b, right_b))
    }
}

impl<T> Index<Side> for ArmPair<T> {
    type Output = T;

    fn index(&self, side: Side) -> &T {
        match side {
            Side::Left => &self.left,
            Side::Right => &self.right,
        }
    }
}

impl<T> IndexMut<Side> for ArmPair<T> {
    fn index_mut(&mut self, side: Side) -> &mut T {
        match side {
            Side::Left => &mut self.left,
            Side::Right => &mut self.right,
        }
    }
}

impl<T, E> ArmPair<Result<T, E>> {
    /// Both values, or the first error in [`Side::BOTH`] order.
    pub fn transpose(self) -> Result<ArmPair<T>, E> {
        Ok(ArmPair::new(self.left?, self.right?))
    }
}

impl<T> ArmPair<Option<T>> {
    /// Both values, or `None` when either side holds none.
    pub fn transpose(self) -> Option<ArmPair<T>> {
        Some(ArmPair::new(self.left?, self.right?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_side_names_itself_its_arm_and_its_gripper() {
        assert_eq!(Side::Left.to_string(), "left");
        assert_eq!(Side::Right.to_string(), "right");
        assert_eq!(Side::BOTH.map(Side::arm_name), ["left_arm", "right_arm"]);
        assert_eq!(
            Side::BOTH.map(Side::gripper_name),
            ["left_gripper", "right_gripper"]
        );
    }

    #[test]
    fn wire_names_parse_back_to_their_side_and_foreign_names_refuse() {
        for side in Side::BOTH {
            assert_eq!(Side::from_arm_name(side.arm_name()), Ok(side));
            assert_eq!(Side::from_gripper_name(side.gripper_name()), Ok(side));
            assert!(
                UnknownArmName
                    .to_string()
                    .contains(&format!("\"{}\"", side.arm_name()))
            );
            assert!(
                UnknownGripperName
                    .to_string()
                    .contains(&format!("\"{}\"", side.gripper_name()))
            );
        }
        assert_eq!(Side::from_arm_name("left_gripper"), Err(UnknownArmName));
        assert_eq!(Side::from_gripper_name("left_arm"), Err(UnknownGripperName));
        assert_eq!(Side::from_arm_name("LEFT_ARM"), Err(UnknownArmName));
        assert_eq!(Side::from_arm_name(""), Err(UnknownArmName));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn a_side_deserializes_from_its_lowercase_name_only() {
        let side = |json: &str| serde_json::from_str::<Side>(json);
        assert_eq!(side(r#""left""#).unwrap(), Side::Left);
        assert_eq!(side(r#""right""#).unwrap(), Side::Right);
        let refused = side(r#""Left""#).unwrap_err().to_string();
        assert_eq!(
            refused,
            "unknown variant `Left`, expected `left` or `right` at line 1 column 6"
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn a_side_serializes_to_its_lowercase_name() {
        let json = |side: Side| serde_json::to_string(&side).unwrap();
        assert_eq!(json(Side::Left), r#""left""#);
        assert_eq!(json(Side::Right), r#""right""#);
    }

    #[test]
    fn each_side_reads_and_writes_its_own_value() {
        let mut pair = ArmPair::new("left", "right");
        assert_eq!(pair[Side::Left], "left");
        assert_eq!(pair[Side::Right], "right");
        pair[Side::Left] = "new left";
        assert_eq!(pair, ArmPair::new("new left", "right"));
        pair[Side::Right] = "new right";
        assert_eq!(pair, ArmPair::new("new left", "new right"));
    }

    #[test]
    fn iter_walks_the_values_left_first() {
        assert_eq!(ArmPair::new(1, 2).iter().collect::<Vec<_>>(), [&1, &2]);
    }

    #[test]
    fn into_iter_hands_each_value_its_own_side_left_first() {
        assert_eq!(
            ArmPair::new(1, 2).into_iter().collect::<Vec<_>>(),
            [(Side::Left, 1), (Side::Right, 2)]
        );
    }

    #[test]
    fn from_fn_builds_each_side_from_its_own_side_left_first() {
        let mut calls = Vec::new();
        let pair = ArmPair::from_fn(|side| {
            calls.push(side);
            side
        });
        assert_eq!(pair, ArmPair::new(Side::Left, Side::Right));
        assert_eq!(calls, Side::BOTH);
    }

    #[test]
    fn map_hands_each_value_its_own_side_left_first() {
        let mut calls = Vec::new();
        let pair = ArmPair::new(1, 2).map(|side, value| {
            calls.push(side);
            format!("{side} {value}")
        });
        assert_eq!(
            pair,
            ArmPair::new("left 1".to_owned(), "right 2".to_owned())
        );
        assert_eq!(calls, Side::BOTH);
    }

    #[test]
    fn zip_and_unzip_keep_each_value_on_its_side() {
        let zipped = ArmPair::new(1, 2).zip(ArmPair::new("a", "b"));
        assert_eq!(zipped, ArmPair::new((1, "a"), (2, "b")));
        assert_eq!(zipped.unzip(), (ArmPair::new(1, 2), ArmPair::new("a", "b")));
    }

    #[test]
    fn transpose_gives_both_values_or_the_first_error() {
        let pair = |left: Result<i32, &'static str>, right| ArmPair::new(left, right);
        assert_eq!(pair(Ok(1), Ok(2)).transpose(), Ok(ArmPair::new(1, 2)));
        assert_eq!(pair(Ok(1), Err("right")).transpose(), Err("right"));
        assert_eq!(pair(Err("left"), Ok(2)).transpose(), Err("left"));
        assert_eq!(pair(Err("left"), Err("right")).transpose(), Err("left"));
    }

    #[test]
    fn transpose_gives_both_values_or_none() {
        assert_eq!(
            ArmPair::new(Some(1), Some(2)).transpose(),
            Some(ArmPair::new(1, 2))
        );
        assert_eq!(ArmPair::new(Some(1), None).transpose(), None);
        assert_eq!(ArmPair::new(None, Some(2)).transpose(), None);
    }
}
