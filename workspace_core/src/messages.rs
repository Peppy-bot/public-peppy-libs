//! The one-line text of each verdict, so that an answer on the robot and one
//! in a simulation say the same thing in the same words. A camera is named
//! by the name it runs under on the robot (`chest` reads "the chest
//! camera").

use crate::fit::{Fit, Rectangle};
use crate::grasp::GraspDirection;
use crate::grid::REACH_TOLERANCE;
use crate::verdict::{Reach, View};

/// The perception camera of a robot as the message of a surface names it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceCamera<'a> {
    /// The name it runs under on the robot.
    pub name: &'a str,
    /// How far it stands above the surface's top, in metres: below 0 when
    /// the top is above it.
    pub above_top: f64,
}

/// The message of one point or object: whether an arm reaches it and whether
/// `camera`, the robot's perception camera, sees it.
pub fn point_message(reach: &Reach, view: &View, camera: Option<&str>) -> String {
    let camera = camera_phrase(camera);
    match (reach, view.passes()) {
        (Reach::Reached { arm }, true) => match view {
            View::Seen => format!("Workable: {arm} reaches it and {camera} sees it."),
            _ => format!("Workable: {arm} reaches it; {NO_CAMERA}."),
        },
        (Reach::Reached { arm }, false) => {
            format!(
                "Not workable: {arm} reaches it, but {}.",
                view_failure(view, &camera)
            )
        }
        (Reach::Short { by }, true) => match view {
            View::Seen => format!("Not workable: {}; {camera} sees it.", shortfall(*by)),
            _ => format!("Not workable: {}; {NO_CAMERA}.", shortfall(*by)),
        },
        (Reach::Short { by }, false) => format!(
            "Not workable: {}, and {}.",
            shortfall(*by),
            view_failure(view, &camera)
        ),
    }
}

/// The message of a surface the robot's fit to is `fit`, its perception
/// camera `camera`. Where to put objects on a workable surface is the
/// caller's to add, in the frame it answers in
/// ([`robot_frame_placement`] for the robot frame).
pub fn surface_message(fit: &Fit, camera: Option<SurfaceCamera<'_>>) -> String {
    match fit {
        Fit::Workable { area, .. } => format!("Workable: {area:.3} m²."),
        Fit::NotMeasured => {
            "Not measured: no point of its top lies under the grid in front of the robot."
                .to_owned()
        }
        Fit::NotReachable => format!(
            "Not reachable: no arm reaches any point of it with its gripper pointing {}.",
            directions()
        ),
        Fit::NotVisible { reach, view } => {
            let phrase = camera_phrase(camera.map(|camera| camera.name));
            if let Some(camera) = camera.filter(|camera| camera.above_top <= 0.0) {
                return format!(
                    "Not visible: the surface is {:.2} m above {phrase}, which cannot see it.",
                    -camera.above_top
                );
            }
            let reached = x_span(reach);
            match view {
                Some(view) => format!(
                    "Not visible: the arms reach {reached} of it and {phrase} sees {}, so no point is both.",
                    x_span(view)
                ),
                None => format!(
                    "Not visible: the arms reach {reached} of it, but {phrase} sees none of it."
                ),
            }
        }
        Fit::TooLittleRoom { area, min } => format!(
            "Too little room: {area:.3} m² is workable, under the {min:.3} m² that holds a few objects."
        ),
    }
}

/// What an answer that checks several points or objects counts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Counted {
    Points,
    Objects,
}

impl Counted {
    fn noun(self, count: usize) -> &'static str {
        match (self, count) {
            (Self::Points, 1) => "point",
            (Self::Points, _) => "points",
            (Self::Objects, 1) => "object",
            (Self::Objects, _) => "objects",
        }
    }
}

/// How many of the `count` points or objects an answer checks the robot
/// can work: `workable` of them.
pub fn workable_count_message(workable: usize, count: usize, counted: Counted) -> String {
    let noun = counted.noun(count);
    match (workable, count) {
        (_, 1) if workable == 1 => format!("The {noun} is workable."),
        (0, 1) => format!("The {noun} is not workable."),
        (all, _) if all == count => format!("All {count} {noun} are workable."),
        (0, _) => format!("None of the {count} {noun} is workable."),
        (1, _) => format!("1 of the {count} {noun} is workable."),
        (some, _) => format!("{some} of the {count} {noun} are workable."),
    }
}

/// Where to put objects on a workable surface, said in the robot frame.
pub fn robot_frame_placement(rectangle: &Rectangle) -> String {
    format!(
        "Put items {} and {} in the robot frame.",
        between("x", rectangle.x),
        between("y", rectangle.y)
    )
}

/// The clause of a point no camera is asked about ([`View::NoCamera`]): the
/// answer says why once, for every point.
const NO_CAMERA: &str = "its view is not checked";

fn camera_phrase(camera: Option<&str>) -> String {
    match camera {
        Some(name) => format!("the {name} camera"),
        None => "the perception camera".to_owned(),
    }
}

/// How far short of a point the closest arm stops.
fn shortfall(by: f64) -> String {
    if by > REACH_TOLERANCE {
        format!("it is out of reach by {by:.2} m")
    } else {
        format!(
            "no arm reaches it with its gripper pointing {}",
            directions()
        )
    }
}

/// The grasp directions, as a message lists them: `down or forward`.
fn directions() -> String {
    GraspDirection::ALL
        .iter()
        .map(|direction| direction.name())
        .collect::<Vec<_>>()
        .join(" or ")
}

/// Why `view` keeps the point from being workable.
fn view_failure(view: &View, camera: &str) -> String {
    match view {
        View::OutsideField => format!("it is outside the field of view of {camera}"),
        View::OutOfDepth => format!("it is outside the depth range of {camera}"),
        View::HiddenBy(what) => format!("{what} hides it from {camera}"),
        View::NoCamera | View::Seen => unreachable!("a view that passes keeps nothing from it"),
    }
}

fn x_span(rectangle: &Rectangle) -> String {
    match rectangle.x {
        [low, high] if low == high => format!("x {low:.2} m"),
        [low, high] => format!("x {low:.2} to {high:.2} m"),
    }
}

fn between(axis: &str, [low, high]: [f64; 2]) -> String {
    if low == high {
        format!("at {axis} {low:.2} m")
    } else {
        format!("between {axis} {low:.2} and {high:.2} m")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reached() -> Reach {
        Reach::Reached {
            arm: "right_arm".into(),
        }
    }

    #[test]
    fn each_verdict_of_a_point_has_one_fixed_message() {
        let chest = Some("chest");
        let short = Reach::Short { by: 0.6 };
        let cases = [
            (
                reached(),
                View::Seen,
                "Workable: right_arm reaches it and the chest camera sees it.",
            ),
            (
                reached(),
                View::NoCamera,
                "Workable: right_arm reaches it; its view is not checked.",
            ),
            (
                reached(),
                View::OutsideField,
                "Not workable: right_arm reaches it, but it is outside the field of view of the chest camera.",
            ),
            (
                reached(),
                View::OutOfDepth,
                "Not workable: right_arm reaches it, but it is outside the depth range of the chest camera.",
            ),
            (
                reached(),
                View::HiddenBy("object obj_1 (object/cube)".into()),
                "Not workable: right_arm reaches it, but object obj_1 (object/cube) hides it from the chest camera.",
            ),
            (
                short.clone(),
                View::Seen,
                "Not workable: it is out of reach by 0.60 m; the chest camera sees it.",
            ),
            (
                short.clone(),
                View::NoCamera,
                "Not workable: it is out of reach by 0.60 m; its view is not checked.",
            ),
            (
                short,
                View::OutsideField,
                "Not workable: it is out of reach by 0.60 m, and it is outside the field of view of the chest camera.",
            ),
            (
                Reach::Short { by: 0.004 },
                View::Seen,
                "Not workable: no arm reaches it with its gripper pointing down or forward; the chest camera sees it.",
            ),
        ];
        for (reach, view, expected) in cases {
            assert_eq!(point_message(&reach, &view, chest), expected);
        }
    }

    #[test]
    fn each_verdict_of_a_surface_has_one_fixed_message() {
        let chest = |above_top| SurfaceCamera {
            name: "chest",
            above_top,
        };
        let reach = Rectangle {
            x: [0.2, 0.32],
            y: [-0.3, 0.3],
        };
        let rectangle = Rectangle {
            x: [0.22, 0.36],
            y: [-0.2, 0.2],
        };
        let cases = [
            (
                Fit::Workable {
                    area: 0.052,
                    rectangle,
                },
                chest(0.34),
                "Workable: 0.052 m².",
            ),
            (
                Fit::NotMeasured,
                chest(0.34),
                "Not measured: no point of its top lies under the grid in front of the robot.",
            ),
            (
                Fit::NotReachable,
                chest(0.34),
                "Not reachable: no arm reaches any point of it with its gripper pointing down or forward.",
            ),
            (
                Fit::NotVisible { reach, view: None },
                chest(-0.17),
                "Not visible: the surface is 0.17 m above the chest camera, which cannot see it.",
            ),
            (
                Fit::NotVisible { reach, view: None },
                chest(0.1),
                "Not visible: the arms reach x 0.20 to 0.32 m of it, but the chest camera sees none of it.",
            ),
            (
                Fit::NotVisible {
                    reach,
                    view: Some(Rectangle {
                        x: [0.4, 0.64],
                        y: [-0.3, 0.3],
                    }),
                },
                chest(0.1),
                "Not visible: the arms reach x 0.20 to 0.32 m of it and the chest camera sees x 0.40 to 0.64 m, so no point is both.",
            ),
            (
                Fit::TooLittleRoom {
                    area: 0.02,
                    min: 0.03,
                },
                chest(0.34),
                "Too little room: 0.020 m² is workable, under the 0.030 m² that holds a few objects.",
            ),
        ];
        for (fit, camera, expected) in cases {
            assert_eq!(surface_message(&fit, Some(camera)), expected);
        }
        assert_eq!(
            surface_message(&Fit::NotReachable, None),
            surface_message(&Fit::NotReachable, Some(chest(0.3)))
        );
    }

    #[test]
    fn the_count_of_workable_points_or_objects_reads_as_a_sentence() {
        let cases = [
            (1, 1, Counted::Points, "The point is workable."),
            (0, 1, Counted::Objects, "The object is not workable."),
            (3, 3, Counted::Points, "All 3 points are workable."),
            (0, 2, Counted::Objects, "None of the 2 objects is workable."),
            (1, 4, Counted::Points, "1 of the 4 points is workable."),
            (2, 5, Counted::Objects, "2 of the 5 objects are workable."),
        ];
        for (workable, count, counted, expected) in cases {
            assert_eq!(workable_count_message(workable, count, counted), expected);
        }
    }

    #[test]
    fn a_placement_in_the_robot_frame_names_both_spans_or_the_one_value_of_a_line() {
        let rectangle = Rectangle {
            x: [0.22, 0.36],
            y: [-0.2, 0.2],
        };
        assert_eq!(
            robot_frame_placement(&rectangle),
            "Put items between x 0.22 and 0.36 m and between y -0.20 and 0.20 m in the robot frame."
        );
        let line = Rectangle {
            x: [0.3, 0.3],
            y: [-0.1, 0.1],
        };
        assert_eq!(
            robot_frame_placement(&line),
            "Put items at x 0.30 m and between y -0.10 and 0.10 m in the robot frame."
        );
    }
}
