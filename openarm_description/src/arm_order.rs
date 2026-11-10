//! The arms a request names, in its order: the order its joint arrays hold the arms'
//! joints in, [`ARM_DOF`] for each arm.

use crate::{ARM_DOF, ArmPair, Side};

/// The arms a request names, in its order, each once.
#[derive(Clone, Debug)]
pub struct ArmOrder(Vec<Side>);

impl ArmOrder {
    /// Parse the arm names of a request's `arms`, in their order: one or more
    /// of this robot's arm names, none named twice. A refusal names what is
    /// wrong in the words of the field.
    pub fn parse(arm_names: &[&str]) -> Result<Self, String> {
        let places = places_in_order(arm_names, &Side::BOTH.map(Side::arm_name))?;
        Ok(Self(
            places.into_iter().map(|place| Side::BOTH[place]).collect(),
        ))
    }

    /// The arms, in the request's order.
    pub fn sides(&self) -> &[Side] {
        &self.0
    }

    /// How many joint positions an array in this order holds.
    fn joint_count(&self) -> usize {
        self.0.len() * ARM_DOF
    }

    /// The number of waypoints every arm of this order holds, given each
    /// arm's `counts` in this order: refused unless the first holds one or
    /// more and every other as many.
    ///
    /// # Panics
    ///
    /// When `counts` does not hold one count for each arm of this order.
    pub fn waypoint_count(&self, counts: &[usize]) -> Result<usize, String> {
        assert_eq!(counts.len(), self.0.len(), "one count for each arm");
        let named: Vec<(&str, usize)> = self
            .0
            .iter()
            .map(|side| side.arm_name())
            .zip(counts.iter().copied())
            .collect();
        common_waypoint_count(&named)
    }

    /// Each arm of this order with its joints in `array`, an array in this
    /// order.
    ///
    /// # Panics
    ///
    /// When `array` does not hold [`Self::joint_count`] joints.
    fn split<'a>(&'a self, array: &'a [f64]) -> impl Iterator<Item = (Side, [f64; ARM_DOF])> + 'a {
        assert_eq!(array.len(), self.joint_count(), "an array in this order");
        self.0
            .iter()
            .zip(array.chunks_exact(ARM_DOF))
            .map(|(&side, joints)| {
                (
                    side,
                    joints.try_into().expect("an arm's share of the array"),
                )
            })
    }

    /// The joints of `side`'s arm in `array`, an array in this order: none
    /// for an arm this order does not name.
    pub fn joints_of(&self, array: &[f64], side: Side) -> Option<[f64; ARM_DOF]> {
        self.split(array)
            .find_map(|(named, joints)| (named == side).then_some(joints))
    }

    /// `held` with each arm of this order at its joints in `array`.
    pub fn placed(&self, array: &[f64], held: &ArmPair<[f64; ARM_DOF]>) -> ArmPair<[f64; ARM_DOF]> {
        ArmPair::from_fn(|side| self.joints_of(array, side).unwrap_or(held[side]))
    }
}

/// The place in `robot_arms` of each arm a request's `arm_names` name, in their order: one
/// or more of the robot's arm names, none named twice. A refusal names what is wrong in the
/// words of the field.
pub fn places_in_order(arm_names: &[&str], robot_arms: &[&str]) -> Result<Vec<usize>, String> {
    if arm_names.is_empty() {
        return Err(format!(
            "arms holds no arm: name one or more of {}",
            listed(robot_arms)
        ));
    }
    let places = arm_names
        .iter()
        .map(|name| {
            robot_arms
                .iter()
                .position(|arm| arm == name)
                .ok_or_else(|| no_such_arm("arms", name, robot_arms))
        })
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

/// The refusal of `name`, an arm of `field` that a robot of the arms `robot_arms` does not
/// have.
pub fn no_such_arm(field: &str, name: &str, robot_arms: &[&str]) -> String {
    let (its, are) = match robot_arms {
        [_] => ("its arm", "is"),
        _ => ("its arms", "are"),
    };
    format!(
        "{field} names \"{name}\", no arm of this robot: {its} {are} {}",
        listed(robot_arms)
    )
}

/// A robot's arm names, quoted, as a refusal lists them.
fn listed(robot_arms: &[&str]) -> String {
    robot_arms
        .iter()
        .map(|name| format!("\"{name}\""))
        .collect::<Vec<_>>()
        .join(" and ")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Where the tests hold the arms, and where an array takes them.
    const HELD: ArmPair<[f64; ARM_DOF]> = ArmPair {
        left: [0.0, -0.8, 0.0, 1.2, 0.0, 0.0, 0.0],
        right: [0.0, 0.8, 0.0, 1.2, 0.0, 0.0, 0.0],
    };
    const TO: ArmPair<[f64; ARM_DOF]> = ArmPair {
        left: [0.0, -1.2, 0.0, 1.2, 0.0, 0.0, 0.0],
        right: [0.0, 0.8, 0.0, 1.3, 0.0, 0.0, 0.0],
    };

    fn order(arm_names: &[&str]) -> ArmOrder {
        ArmOrder::parse(arm_names).expect("the arms of this robot, each once")
    }

    #[test]
    fn a_robot_of_one_arm_refuses_another_in_the_singular() {
        assert_eq!(
            places_in_order(&["elbow"], &["arm"]).unwrap_err(),
            "arms names \"elbow\", no arm of this robot: its arm is \"arm\""
        );
        assert_eq!(places_in_order(&["arm"], &["arm"]).unwrap(), [0]);
    }

    /// The array in `order` of the arms at `joints`.
    fn array_of(order: &ArmOrder, joints: &ArmPair<[f64; ARM_DOF]>) -> Vec<f64> {
        order
            .sides()
            .iter()
            .flat_map(|side| joints[*side])
            .collect()
    }

    /// A request's arms are one or more of this robot's arms, each once; any
    /// other list is refused naming what is wrong in the words of the field.
    #[test]
    fn a_request_takes_each_arm_of_the_robot_at_most_once() {
        let refused = |names: &[&str]| ArmOrder::parse(names).expect_err("refused");
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
        assert_eq!(order(&["right_arm"]).sides(), [Side::Right]);
        assert_eq!(
            order(&["right_arm", "left_arm"]).sides(),
            [Side::Right, Side::Left]
        );
    }

    /// Every arm of a request passes through as many waypoints, one or more;
    /// the refusal names the arm that differs from the first.
    #[test]
    fn every_arm_holds_as_many_waypoints() {
        let both = order(&["left_arm", "right_arm"]);
        assert_eq!(both.waypoint_count(&[3, 3]), Ok(3));
        assert_eq!(
            both.waypoint_count(&[0, 0]),
            Err("left_arm holds no waypoint: a joint path passes through one or more".to_string())
        );
        assert_eq!(
            both.waypoint_count(&[2, 0]),
            Err(
                "right_arm holds 0 waypoints, where left_arm holds 2: every arm passes through \
                 as many"
                    .to_string()
            )
        );
        assert_eq!(order(&["right_arm"]).waypoint_count(&[1]), Ok(1));
    }

    /// An array in an order holds the named arms' joints in that order: named
    /// right first, its first seven joints are the right arm's, and the arms
    /// it does not name keep where they are held.
    #[test]
    fn an_array_holds_each_named_arm_in_the_order_of_the_requests_arms() {
        let right_first = order(&["right_arm", "left_arm"]);
        let array = array_of(&right_first, &TO);
        assert_eq!(array[..ARM_DOF], TO.right);
        assert_eq!(array[ARM_DOF..], TO.left);
        assert_eq!(right_first.joint_count(), 2 * ARM_DOF);
        assert_eq!(right_first.placed(&array, &HELD), TO);

        let left = order(&["left_arm"]);
        let array = array_of(&left, &TO);
        assert_eq!(array, TO.left);
        assert_eq!(
            left.placed(&array, &HELD),
            ArmPair::new(TO.left, HELD.right)
        );
        assert_eq!(left.joints_of(&array, Side::Right), None);
    }
}
