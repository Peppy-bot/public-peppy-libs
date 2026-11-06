//! The arms a plan moves together: one or more arms of `N` joints each, a state of the
//! plan holding their joints one arm after another, `D` in all, the bodies of every arm,
//! how far any point of them moves, and the robot's self model where the robot has one.

use chain_kinematics::Limit;

use crate::arm::{Arm, Body, SelfModel};

/// A body of an arm of the group placed at a state: the index of its arm in the group,
/// the name of its link or of its rider, and the body.
pub struct GroupBody<'a> {
    pub arm: usize,
    pub name: &'a str,
    pub body: Body<'a>,
}

/// The arms a plan moves together, in the order a state holds their joints.
pub struct Arms<'a, const N: usize, const D: usize> {
    arms: Vec<Arm<'a, N>>,
    self_model: Option<&'a dyn SelfModel<D>>,
}

impl<'a, const N: usize, const D: usize> Arms<'a, N, D> {
    /// The group of `arms`, the first's joints first in a state, checked against
    /// `self_model` when the robot has one.
    ///
    /// # Panics
    ///
    /// When `arms` does not hold `D / N` arms.
    pub fn new(arms: Vec<Arm<'a, N>>, self_model: Option<&'a dyn SelfModel<D>>) -> Self {
        const {
            assert!(
                N > 0 && D >= N && D.is_multiple_of(N),
                "a state holds one or more whole arms"
            )
        };
        assert_eq!(
            arms.len() * N,
            D,
            "a group of {} arms of {N} joints holds {D} joints",
            arms.len()
        );
        Self { arms, self_model }
    }

    /// The joints of arm `k` in the state `q`.
    pub fn slice(q: &[f64; D], k: usize) -> [f64; N] {
        std::array::from_fn(|i| q[k * N + i])
    }

    /// The state that holds each arm's joints of `parts`, in the group's order.
    ///
    /// # Panics
    ///
    /// When `parts` does not hold `D / N` arms.
    pub fn join(parts: &[[f64; N]]) -> [f64; D] {
        assert_eq!(parts.len() * N, D, "one part for each arm of the group");
        std::array::from_fn(|i| parts[i / N][i % N])
    }

    pub fn arms(&self) -> &[Arm<'a, N>] {
        &self.arms
    }

    pub fn self_model(&self) -> Option<&dyn SelfModel<D>> {
        self.self_model
    }

    /// Every body of every arm, as `Arm::bodies` places it, with the arm at its joints of
    /// `q`, the arms in the group's order.
    pub fn bodies(&self, q: &[f64; D]) -> Vec<GroupBody<'a>> {
        self.arms
            .iter()
            .enumerate()
            .flat_map(|(k, arm)| {
                arm.bodies(&Self::slice(q, k))
                    .into_iter()
                    .map(move |(name, body)| GroupBody { arm: k, name, body })
            })
            .collect()
    }

    /// Whether a link of an arm carries an object.
    pub fn carries_an_object(&self) -> bool {
        self.arms.iter().any(Arm::carries_an_object)
    }

    /// The limits of each joint of the state.
    pub fn limits(&self) -> [Limit; D] {
        let limits: Vec<[Limit; N]> = self
            .arms
            .iter()
            .map(|arm| arm.geometry().chain().limits())
            .collect();
        std::array::from_fn(|i| limits[i / N][i % N])
    }

    /// The URDF name of joint `i` of the state.
    pub fn joint_name(&self, i: usize) -> &str {
        self.arms[i / N].geometry().chain().joint_name(i % N)
    }

    /// A bound on how far any point of a body moves on the straight joint motion from
    /// `from` to `to`: the largest of the arms' bounds on their joints, since a point rides
    /// one arm.
    pub fn displacement(&self, from: &[f64; D], to: &[f64; D]) -> f64 {
        self.largest(|k, arm| arm.displacement(&Self::slice(from, k), &Self::slice(to, k)))
    }

    /// The farthest any point of a body moves per unit of the state's L2 distance, along
    /// any joint motion: the largest of the arms' levers, since an arm's joints move at
    /// most that distance.
    pub fn lever(&self) -> f64 {
        self.largest(|_, arm| arm.lever())
    }

    /// The farthest any point of a body moves per unit of a joint path's length along a
    /// stretch on which each joint's share of the path's unit tangent is at most
    /// `tangent_share`: the largest of the arms' `Arm::lever_along` on their shares.
    pub fn lever_along(&self, tangent_share: &[f64; D]) -> f64 {
        self.largest(|k, arm| arm.lever_along(&Self::slice(tangent_share, k)))
    }

    /// The largest of `bound` over the arms, by their index: the one arm's own for a group
    /// of one.
    fn largest(&self, bound: impl Fn(usize, &Arm<'a, N>) -> f64) -> f64 {
        self.arms
            .iter()
            .enumerate()
            .map(|(k, arm)| bound(k, arm))
            .reduce(f64::max)
            .expect("a group holds an arm")
    }
}

#[cfg(test)]
mod tests {
    use chain_kinematics::{ChainSpec, JointSelection};
    use collision_geometry::chain::ChainGeometry;
    use hegel::TestCase;
    use hegel::generators as gs;

    use super::*;

    /// An arm of two revolute joints, about z and then about y 30 cm out, at `y` meters
    /// along y, whose links hold a box and a ball, its names prefixed by `prefix`.
    fn arm_at(prefix: &str, y: f64) -> ChainGeometry<2> {
        let urdf = format!(
            r#"<robot name="{prefix}arm">
                <link name="base"/>
                <link name="{prefix}link_1">
                  <collision><origin xyz="0.15 0 0"/><geometry><box size="0.3 0.04 0.04"/></geometry></collision>
                </link>
                <link name="{prefix}link_2">
                  <collision><origin xyz="0.1 0 0"/><geometry><sphere radius="0.04"/></geometry></collision>
                </link>
                <joint name="{prefix}joint_1" type="revolute">
                  <origin xyz="0 {y} 0"/>
                  <parent link="base"/><child link="{prefix}link_1"/><axis xyz="0 0 1"/>
                  <limit lower="-3" upper="3" effort="1" velocity="1"/>
                </joint>
                <joint name="{prefix}joint_2" type="revolute">
                  <origin xyz="0.3 0 0"/>
                  <parent link="{prefix}link_1"/><child link="{prefix}link_2"/><axis xyz="0 1 0"/>
                  <limit lower="-2" upper="2" effort="1" velocity="1"/>
                </joint>
              </robot>"#
        );
        ChainGeometry::from_urdf(
            &urdf,
            "",
            &ChainSpec {
                base_link: None,
                tip_link: &format!("{prefix}link_2"),
                joints: JointSelection::PathOrder,
            },
        )
        .expect("an arm of a box and a ball")
    }

    // A state holds each arm's joints one arm after another, and comes back from them.
    #[test]
    fn a_state_holds_each_arm_in_the_groups_order() {
        let q = [0.1, 0.2, 0.3, 0.4];
        assert_eq!(Arms::<2, 4>::slice(&q, 0), [0.1, 0.2]);
        assert_eq!(Arms::<2, 4>::slice(&q, 1), [0.3, 0.4]);
        assert_eq!(Arms::<2, 4>::join(&[[0.1, 0.2], [0.3, 0.4]]), q);
    }

    // A group holds as many arms as its state holds whole arms.
    #[test]
    #[should_panic(expected = "a group of 1 arms of 2 joints holds 4 joints")]
    fn a_group_short_of_its_arms_is_refused() {
        let left = arm_at("left_", 0.3);
        Arms::<2, 4>::new(vec![Arm::new("left", &left)], None);
    }

    // On drawn straight motions of two arms, no vertex of a piece of either arm moves
    // farther than the group's displacement to a drawn state along the motion, and each
    // arm's bodies stand where the arm alone places them at its joints of the state.
    #[hegel::test(test_cases = 200)]
    fn the_groups_displacement_bounds_every_point_of_every_arm(tc: TestCase) {
        let (left, right) = (arm_at("left_", 0.3), arm_at("right_", -0.3));
        let arms = Arms::<2, 4>::new(
            vec![Arm::new("left", &left), Arm::new("right", &right)],
            None,
        );
        let limits = arms.limits();
        let draw = |tc: &TestCase| -> [f64; 4] {
            std::array::from_fn(|i| {
                tc.draw(gs::floats().min_value(limits[i].lo).max_value(limits[i].hi))
            })
        };
        let (from, end) = (draw(&tc), draw(&tc));
        let share: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
        let to: [f64; 4] = std::array::from_fn(|i| from[i] + (end[i] - from[i]) * share);
        let vertices = |q: &[f64; 4]| -> Vec<nalgebra::Point3<f64>> {
            arms.bodies(q)
                .into_iter()
                .flat_map(|body| match body.body {
                    Body::Piece(piece) => piece.vertices().collect::<Vec<_>>(),
                    Body::Shape(_) => panic!("the arms carry no object"),
                })
                .collect()
        };
        let bound = arms.displacement(&from, &end);
        let moved = vertices(&from)
            .iter()
            .zip(&vertices(&to))
            .map(|(a, b)| (b - a).norm())
            .fold(0.0, f64::max);
        assert!(
            moved <= bound + 1e-12,
            "a vertex moves {moved} m, past {bound} m"
        );
        for (k, arm) in arms.arms().iter().enumerate() {
            let alone: Vec<&str> = arm
                .bodies(&Arms::<2, 4>::slice(&from, k))
                .into_iter()
                .map(|(name, _)| name)
                .collect();
            let in_the_group: Vec<&str> = arms
                .bodies(&from)
                .into_iter()
                .filter(|body| body.arm == k)
                .map(|body| body.name)
                .collect();
            assert_eq!(in_the_group, alone);
        }
    }
}
