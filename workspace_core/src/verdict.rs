//! What a robot makes of one point: whether an arm reaches it and whether its
//! perception camera sees it.

/// Whether an arm reaches a target.
#[derive(Clone, Debug, PartialEq)]
pub enum Reach {
    /// The arm named reaches it.
    Reached { arm: String },
    /// No arm reaches it. `by` is how far short of it the closest arm
    /// stops, in metres; within the reach tolerance when the arms come close
    /// enough but none can point its gripper a grasp direction there.
    Short { by: f64 },
}

impl Reach {
    pub fn reached(&self) -> bool {
        matches!(self, Self::Reached { .. })
    }

    /// The arm that reaches the target, or `""` when none does.
    pub fn arm(&self) -> &str {
        match self {
            Self::Reached { arm } => arm,
            Self::Short { .. } => "",
        }
    }

    /// How far short of the target the closest arm stops, in metres: 0 when
    /// an arm reaches it.
    pub fn short_by(&self) -> f64 {
        match self {
            Self::Reached { .. } => 0.0,
            Self::Short { by } => *by,
        }
    }
}

/// What a robot's perception camera makes of a point.
#[derive(Clone, Debug, PartialEq)]
pub enum View {
    /// No camera is asked: the robot has no perception camera, or the
    /// answer cannot read its camera's field of view.
    NoCamera,
    /// The camera sees the point.
    Seen,
    /// The point is outside the camera's field of view.
    OutsideField,
    /// The point is nearer or farther than the depths the camera measures.
    OutOfDepth,
    /// Something stands between the camera and the point: the text names it.
    /// Only an answer that knows the world around the robot can say this.
    HiddenBy(String),
}

impl View {
    /// Whether this leaves the point workable: seen, or not asked.
    pub fn passes(&self) -> bool {
        matches!(self, Self::NoCamera | Self::Seen)
    }

    /// Whether the camera sees the point.
    pub fn seen(&self) -> bool {
        matches!(self, Self::Seen)
    }

    /// The view's name in an answer: `no_camera`, `seen`, `outside_field`,
    /// `out_of_depth` or `hidden`.
    pub fn name(&self) -> &'static str {
        match self {
            Self::NoCamera => "no_camera",
            Self::Seen => "seen",
            Self::OutsideField => "outside_field",
            Self::OutOfDepth => "out_of_depth",
            Self::HiddenBy(_) => "hidden",
        }
    }

    /// What hides the point, or `""` when nothing is said to.
    pub fn hidden_by(&self) -> &str {
        match self {
            Self::HiddenBy(what) => what,
            _ => "",
        }
    }
}

/// One point of a surface under the grid, in the robot frame (`x` ahead of
/// the robot, `y` to its left, metres), and how the robot fares with it:
/// whether an arm reaches the target above it, and whether the perception
/// camera sees it.
#[derive(Clone, Debug, PartialEq)]
pub struct GridPoint {
    pub x: f64,
    pub y: f64,
    pub reach: Reach,
    pub view: View,
}

impl GridPoint {
    /// Whether the robot can work the point ([`workable`]).
    pub fn workable(&self) -> bool {
        workable(&self.reach, &self.view)
    }
}

/// Whether a robot can work a point or an object whose reach is `reach` and
/// whose view is `view`: an arm reaches it and the perception camera sees
/// it, or is not asked.
pub fn workable(reach: &Reach, view: &View) -> bool {
    reach.reached() && view.passes()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(reach: Reach, view: View) -> GridPoint {
        GridPoint {
            x: 0.3,
            y: 0.0,
            reach,
            view,
        }
    }

    fn reached() -> Reach {
        Reach::Reached {
            arm: "right_arm".into(),
        }
    }

    #[test]
    fn a_point_is_workable_when_an_arm_reaches_it_and_the_camera_sees_it_or_is_not_asked() {
        assert!(point(reached(), View::Seen).workable());
        assert!(point(reached(), View::NoCamera).workable());
        assert!(!point(reached(), View::OutsideField).workable());
        assert!(!point(reached(), View::OutOfDepth).workable());
        assert!(!point(reached(), View::HiddenBy("object a (object/cube)".into())).workable());
        assert!(!point(Reach::Short { by: 0.2 }, View::Seen).workable());
    }

    #[test]
    fn a_reach_and_a_view_name_their_parts_as_an_answer_carries_them() {
        assert_eq!((reached().arm(), reached().short_by()), ("right_arm", 0.0));
        let short = Reach::Short { by: 0.6 };
        assert_eq!((short.arm(), short.short_by()), ("", 0.6));
        let names: Vec<_> = [
            View::NoCamera,
            View::Seen,
            View::OutsideField,
            View::OutOfDepth,
            View::HiddenBy("the floor".into()),
        ]
        .iter()
        .map(View::name)
        .collect();
        assert_eq!(
            names,
            [
                "no_camera",
                "seen",
                "outside_field",
                "out_of_depth",
                "hidden"
            ]
        );
        assert_eq!(View::HiddenBy("the floor".into()).hidden_by(), "the floor");
        assert_eq!(View::Seen.hidden_by(), "");
        assert!(View::Seen.seen() && !View::NoCamera.seen());
    }
}
