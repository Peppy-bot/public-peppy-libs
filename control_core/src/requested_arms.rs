//! The arms a request to a robot node names: their places among the robot's arms, in
//! the request's order, the place of an arm or a gripper it names by name, the limb a
//! node's parameter names by its place, and the waypoints every arm of a joint path passes
//! through. A refusal names what is wrong in the words of the request's field.

/// The place in `robot_arms` of each arm a request's `arm_names` name, in their order: one
/// or more of the robot's arm names, none named twice. A refusal names what is wrong in the
/// words of the field.
pub fn places_in_order(arm_names: &[&str], robot_arms: &[&str]) -> Result<Vec<usize>, String> {
    if arm_names.is_empty() {
        return Err(format!(
            "arms holds no arm: name one or more of {}",
            quoted(robot_arms).join(" and ")
        ));
    }
    let places = arm_names
        .iter()
        .map(|name| place_of_arm("arms", name, robot_arms))
        .collect::<Result<Vec<usize>, String>>()?;
    match (0..places.len()).find(|&k| places[..k].contains(&places[k])) {
        Some(twice) => Err(format!(
            "arms names \"{}\" twice: name each arm once",
            arm_names[twice]
        )),
        None => Ok(places),
    }
}

/// The number of waypoints every arm holds, given each arm's name and count in a request's
/// order: refused unless the first holds one or more and every other as many.
///
/// # Panics
///
/// When `counts` holds no arm.
pub fn common_waypoint_count(counts: &[(&str, usize)]) -> Result<usize, String> {
    let (first, count) = counts[0];
    if count == 0 {
        return Err(format!(
            "{first} holds no waypoint: a joint path passes through one or more"
        ));
    }
    match counts.iter().find(|(_, held)| *held != count) {
        Some((arm, held)) => Err(format!(
            "{arm} holds {held} waypoints, where {first} holds {count}: every arm passes \
             through as many"
        )),
        None => Ok(count),
    }
}

/// The place in `robot_arms` of the arm a request's `field` names by `name`, or the
/// refusal that names the robot's arms.
pub fn place_of_arm(field: &str, name: &str, robot_arms: &[&str]) -> Result<usize, String> {
    place_of("arm", field, name, robot_arms)
}

/// The place in `robot_grippers` of the gripper a request's `field` names by `name`, or the
/// refusal that names the robot's grippers.
pub fn place_of_gripper(field: &str, name: &str, robot_grippers: &[&str]) -> Result<usize, String> {
    place_of("gripper", field, name, robot_grippers)
}

/// The place in `limbs`, the robot's limbs of the kind `limb`, of the one `field` names by
/// `name`, or the refusal that names them.
fn place_of(limb: &str, field: &str, name: &str, limbs: &[&str]) -> Result<usize, String> {
    limbs.iter().position(|of| *of == name).ok_or_else(|| {
        format!(
            "{field} names \"{name}\", no {limb} of this robot: {}",
            its(limb, &quoted(limbs))
        )
    })
}

/// The limb at `place` among `limbs`, the robot's limbs of the kind `limb` in place order,
/// that a node's parameter `field` names by its place, or the refusal that names each
/// place with the name `name` gives its limb.
pub fn limb_at<'a, T>(
    limb: &str,
    field: &str,
    place: u8,
    limbs: &'a [T],
    name: impl Fn(&T) -> &str,
) -> Result<&'a T, String> {
    limbs.get(usize::from(place)).ok_or_else(|| {
        let places: Vec<String> = limbs
            .iter()
            .enumerate()
            .map(|(at, of)| format!("{at} (\"{}\")", name(of)))
            .collect();
        format!(
            "{field} names {place}, no {limb} of this robot: {}",
            its(limb, &places)
        )
    })
}

/// A robot's limbs of the kind `limb`, each as `listed` writes it, as a refusal names
/// them: "its arms are a and b", "its arm is a", or "it has no arm".
fn its(limb: &str, listed: &[String]) -> String {
    match listed {
        [] => format!("it has no {limb}"),
        [one] => format!("its {limb} is {one}"),
        _ => format!("its {limb}s are {}", listed.join(" and ")),
    }
}

/// Limb names, quoted, as a refusal lists them.
fn quoted(names: &[&str]) -> Vec<String> {
    names.iter().map(|name| format!("\"{name}\"")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The arms of a robot of two.
    const BOTH: [&str; 2] = ["left_arm", "right_arm"];

    #[test]
    fn a_robot_of_one_arm_refuses_another_in_the_singular() {
        assert_eq!(
            places_in_order(&["elbow"], &["arm"]).unwrap_err(),
            "arms names \"elbow\", no arm of this robot: its arm is \"arm\""
        );
        assert_eq!(places_in_order(&["arm"], &["arm"]).unwrap(), [0]);
    }

    /// A request's arms are one or more of the robot's arms, each once, in the request's
    /// order; any other list is refused naming what is wrong in the words of the field.
    #[test]
    fn a_request_takes_each_arm_of_the_robot_at_most_once() {
        let refused = |names: &[&str]| places_in_order(names, &BOTH).expect_err("refused");
        assert_eq!(
            refused(&[]),
            "arms holds no arm: name one or more of \"left_arm\" and \"right_arm\""
        );
        assert_eq!(
            refused(&["left_arm", "head"]),
            "arms names \"head\", no arm of this robot: its arms are \"left_arm\" and \
             \"right_arm\""
        );
        assert_eq!(
            refused(&["right_arm", "left_arm", "right_arm"]),
            "arms names \"right_arm\" twice: name each arm once"
        );
        assert_eq!(places_in_order(&["right_arm"], &BOTH), Ok(vec![1]));
        assert_eq!(
            places_in_order(&["right_arm", "left_arm"], &BOTH),
            Ok(vec![1, 0])
        );
    }

    /// A gripper's name is looked up and refused in the words of a gripper.
    #[test]
    fn a_name_of_no_gripper_is_refused_naming_the_grippers() {
        let grippers = ["left_gripper", "right_gripper"];
        assert_eq!(
            place_of_gripper("gripper_name", "right_gripper", &grippers),
            Ok(1)
        );
        assert_eq!(
            place_of_gripper("gripper_name", "left", &grippers),
            Err(
                "gripper_name names \"left\", no gripper of this robot: its grippers are \
                 \"left_gripper\" and \"right_gripper\""
                    .to_string()
            )
        );
        assert_eq!(
            place_of_gripper("gripper_name", "left", &["gripper"]),
            Err(
                "gripper_name names \"left\", no gripper of this robot: its gripper is \"gripper\""
                    .to_string()
            )
        );
    }

    /// A limb a parameter names by its place is the robot's limb there; a place past the
    /// robot's limbs is refused naming each place with its limb, in the words of the limb.
    #[test]
    fn a_place_of_no_limb_is_refused_naming_each_place() {
        /// An arm's name.
        fn name<'a>(arm: &'a (&'static str, usize)) -> &'a str {
            arm.0
        }
        let arms = [("left_arm", 7), ("right_arm", 7)];
        assert_eq!(limb_at("arm", "arm_id", 1, &arms, name), Ok(&arms[1]));
        assert_eq!(
            limb_at("arm", "arm_id", 2, &arms, name),
            Err(
                "arm_id names 2, no arm of this robot: its arms are 0 (\"left_arm\") and 1 \
                 (\"right_arm\")"
                    .to_string()
            )
        );
        assert_eq!(
            limb_at("gripper", "gripper_id", 1, &arms[..1], name),
            Err(
                "gripper_id names 1, no gripper of this robot: its gripper is 0 (\"left_arm\")"
                    .to_string()
            )
        );
        assert_eq!(
            limb_at("arm", "arm_id", 0, &arms[..0], name),
            Err("arm_id names 0, no arm of this robot: it has no arm".to_string())
        );
    }

    /// Every arm of a request passes through as many waypoints, one or more; the refusal
    /// names the arm that differs from the first.
    #[test]
    fn every_arm_holds_as_many_waypoints() {
        assert_eq!(
            common_waypoint_count(&[("left_arm", 3), ("right_arm", 3)]),
            Ok(3)
        );
        assert_eq!(
            common_waypoint_count(&[("left_arm", 0), ("right_arm", 0)]),
            Err("left_arm holds no waypoint: a joint path passes through one or more".to_string())
        );
        assert_eq!(
            common_waypoint_count(&[("left_arm", 2), ("right_arm", 0)]),
            Err(
                "right_arm holds 0 waypoints, where left_arm holds 2: every arm passes through \
                 as many"
                    .to_string()
            )
        );
        assert_eq!(common_waypoint_count(&[("right_arm", 1)]), Ok(1));
    }
}
