//! The differential base a description gives ([`RobotDescription::differential_base`]):
//! the two wheels a robot drives on, and the outline of its parts on the floor
//! ([`RobotDescription::outline`]), each read from its tree of links and joints in the frame
//! of its root link, the robot frame, with every joint at zero.

use chain_kinematics::nalgebra::{Isometry3, UnitQuaternion, Vector3};
use chain_kinematics::{JointKind, pose_of_origin};
use urdf_rs::Geometry;

use crate::RobotDescription;

/// How far two lengths (m) or two unit vectors of the URDF may differ and count as the
/// same: what writing a decimal leaves of one value.
const SAME: f64 = 1e-9;

/// The axis each wheel turns about, so a positive turn rolls the base forward.
const WHEEL_AXIS: [f64; 3] = [0.0, 1.0, 0.0];

/// The two wheels of a differential base, as its description gives them.
#[derive(Clone, Debug, PartialEq)]
pub struct DifferentialBase {
    /// The wheel further along the robot frame's +y.
    pub left: Wheel,
    pub right: Wheel,
    /// The distance between the two wheel joints' origins (m), along y.
    pub separation_m: f64,
}

/// A wheel of a differential base.
#[derive(Clone, Debug, PartialEq)]
pub struct Wheel {
    /// Its continuous joint.
    pub joint: String,
    /// The radius of its link's one collision cylinder (m).
    pub radius_m: f64,
    /// The length of that cylinder (m): the width of its tire.
    pub width_m: f64,
}

/// Why a description gives no differential base.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum NoDifferentialBase {
    #[error(
        "its wheels are the continuous joints on a link that fixed joints alone join to its root link {root} that no group of its SRDF holds, two of them, and it has {found:?}"
    )]
    WheelCount { root: String, found: Vec<String> },
    #[error(
        "wheel joint {joint} turns about {axis:?} in the root link's frame: a positive turn of a wheel rolls the base forward, about +y"
    )]
    WheelAxis { joint: String, axis: [f64; 3] },
    #[error(
        "wheel joints {left} and {right} stand on no one axle: give their origins the same x and z"
    )]
    NoAxle { left: String, right: String },
    #[error(
        "wheel joints {left} and {right} stand at the same y: stand them apart along y, the left wheel along +y"
    )]
    NoTrack { left: String, right: String },
    #[error(
        "the axle of wheel joints {left} and {right} has its midpoint at {midpoint:?} in the root link's frame: stand that midpoint at x 0 and y 0, the point the base turns about"
    )]
    AxleOffOrigin {
        left: String,
        right: String,
        midpoint: [f64; 2],
    },
    #[error(
        "wheel link {link}: a wheel's collision geometry is one cylinder about its joint's axis, of a radius and a length above 0"
    )]
    WheelShape { link: String },
    #[error(
        "wheel joint {joint}: the collision cylinder of its link stands {offset_m} m off the joint's axis, so it orbits as the wheel turns: center the cylinder on the axis"
    )]
    WheelOffAxis { joint: String, offset_m: f64 },
    #[error(
        "wheel joint {joint}: its link carries the links {children:?}, which turn with the wheel: hang them on a link that does not turn"
    )]
    WheelCarries {
        joint: String,
        children: Vec<String>,
    },
}

/// The rectangle on the floor plane of the root link's frame that holds every collision
/// element of a robot, every joint at zero: how far it reaches ahead of the root link's
/// origin along +x (`front`) and behind it (`rear`, at most `front`), and along +y
/// (`left`) and -y (`right`, at most `left`), each in meters as a coordinate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Outline {
    pub front: f64,
    pub rear: f64,
    pub left: f64,
    pub right: f64,
}

impl Outline {
    /// The rectangle's corners, counterclockwise from the front right one.
    pub fn corners(&self) -> [[f64; 2]; 4] {
        let Self {
            front,
            rear,
            left,
            right,
        } = *self;
        [[front, right], [front, left], [rear, left], [rear, right]]
    }

    /// The rectangle that holds this one and `other`.
    fn holding(self, other: Outline) -> Outline {
        Outline {
            front: self.front.max(other.front),
            rear: self.rear.min(other.rear),
            left: self.left.max(other.left),
            right: self.right.min(other.right),
        }
    }
}

/// Why a description gives no outline.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum NoOutline {
    #[error(
        "link {link}: the outline holds boxes, cylinders and spheres, and one of its collision elements is none of them"
    )]
    Shape { link: String },
    #[error("no link has collision geometry, so the robot has no outline")]
    NoGeometry,
}

impl RobotDescription {
    /// The differential base this description gives: its two wheels, the continuous
    /// joints on a link that fixed joints alone join to its root link that no group of its
    /// SRDF holds, so a continuous joint below a joint that moves, and an arm's on the base
    /// such as Kinova Gen3's `joint_1`, are no wheels. Each turns about +y in the root
    /// link's frame, so a positive turn of either rolls the base forward, the left one
    /// further along +y, both on one axle (the same x and z in the root link's frame) whose
    /// midpoint stands at the root link's origin in x and y, the point the base turns
    /// about. Each wheel's link holds one collision element, a cylinder about
    /// the joint's axis and centered on it, whose radius and length lie above 0, and
    /// carries no other link, since a link it carries turns with it. A description that
    /// gives other wheels is refused, naming the joints or the link.
    pub fn differential_base(&self) -> Result<DifferentialBase, NoDifferentialBase> {
        let [left, right] = self.wheel_joints()?;
        Ok(DifferentialBase {
            left: self.wheel(&left)?,
            right: self.wheel(&right)?,
            separation_m: left.position[1] - right.position[1],
        })
    }

    /// The outline of the robot on the floor: the rectangle that holds every collision
    /// element of every link, each a box, a cylinder or a sphere, placed in the root
    /// link's frame with every joint at zero. A description with an element of another
    /// shape, or with no collision element, gives none.
    pub fn outline(&self) -> Result<Outline, NoOutline> {
        let mut parts = Vec::new();
        for (link, frame) in self.link_frames_at_zero() {
            for collision in &link.collision {
                let placed = frame * pose_of_origin(&collision.origin);
                let part =
                    floor_reach(&collision.geometry, &placed).ok_or_else(|| NoOutline::Shape {
                        link: link.name.clone(),
                    })?;
                parts.push(part);
            }
        }
        parts
            .into_iter()
            .reduce(Outline::holding)
            .ok_or(NoOutline::NoGeometry)
    }

    /// The frame of the link `link` (an index of the tree) in the root link's, where fixed
    /// joints alone join the two; none where a joint between them moves.
    fn fixed_to_root(&self, link: usize) -> Option<Isometry3<f64>> {
        let path = self.tree.path_to(link);
        let joints = path.iter().map(|&joint| self.tree.joint(joint).placement);
        joints
            .clone()
            .all(|placement| !placement.kind.is_movable())
            .then(|| {
                joints.fold(Isometry3::identity(), |above, placement| {
                    above * placement.origin
                })
            })
    }

    /// The continuous joint `joint` (an index of the tree) in the root link's frame, where
    /// fixed joints alone join its parent link to the root link.
    fn placed(&self, joint: usize) -> Option<PlacedJoint<'_>> {
        let tree_joint = self.tree.joint(joint);
        let JointKind::Continuous { axis } = tree_joint.placement.kind else {
            return None;
        };
        let frame = self.fixed_to_root(self.tree.parent_of(joint))? * tree_joint.placement.origin;
        let position = frame.translation.vector;
        let turned = frame.rotation * axis.into_inner();
        Some(PlacedJoint {
            joint,
            name: &tree_joint.name,
            position: [position.x, position.y, position.z],
            axis: [turned.x, turned.y, turned.z],
            own_axis: axis.into_inner(),
        })
    }

    /// The wheel joints, [left, right]: the two continuous joints on a link that fixed
    /// joints alone join to the root link that no group of the SRDF holds.
    fn wheel_joints(&self) -> Result<[PlacedJoint<'_>; 2], NoDifferentialBase> {
        let on_the_base: Vec<PlacedJoint<'_>> = (0..self.tree.joints().len())
            .filter_map(|joint| self.placed(joint))
            .filter(|placed| !self.grouped_joints.contains(placed.name))
            .collect();
        let [a, b] = <[PlacedJoint<'_>; 2]>::try_from(on_the_base).map_err(|found| {
            NoDifferentialBase::WheelCount {
                root: self.root_link().to_owned(),
                found: found.iter().map(|wheel| wheel.name.to_owned()).collect(),
            }
        })?;
        if let Some(turned) = [&a, &b]
            .into_iter()
            .find(|wheel| !same(wheel.axis, WHEEL_AXIS))
        {
            return Err(NoDifferentialBase::WheelAxis {
                joint: turned.name.to_owned(),
                axis: turned.axis,
            });
        }
        let (left, right) = if a.position[1] > b.position[1] {
            (a, b)
        } else {
            (b, a)
        };
        let names = || (left.name.to_owned(), right.name.to_owned());
        let [lx, ly, lz] = left.position;
        let [rx, ry, rz] = right.position;
        if !same([lx, 0.0, lz], [rx, 0.0, rz]) {
            let (left, right) = names();
            return Err(NoDifferentialBase::NoAxle { left, right });
        }
        let track = ly - ry;
        if track.is_nan() || track <= SAME {
            let (left, right) = names();
            return Err(NoDifferentialBase::NoTrack { left, right });
        }
        let midpoint = [lx, (ly + ry) / 2.0];
        if !midpoint.iter().all(|coordinate| coordinate.abs() <= SAME) {
            let (left, right) = names();
            return Err(NoDifferentialBase::AxleOffOrigin {
                left,
                right,
                midpoint,
            });
        }
        Ok([left, right])
    }

    /// The wheel on the joint `placed`: its radius and its width, its link's one collision
    /// element, a cylinder about the joint's axis and centered on it, on a link that carries
    /// no other link, each refused naming the link or the joint.
    fn wheel(&self, placed: &PlacedJoint<'_>) -> Result<Wheel, NoDifferentialBase> {
        let link = &self.robot().links[self.tree.joint(placed.joint).child];
        let shape = || NoDifferentialBase::WheelShape {
            link: link.name.clone(),
        };
        let [collision] = link.collision.as_slice() else {
            return Err(shape());
        };
        let Geometry::Cylinder { radius, length } = collision.geometry else {
            return Err(shape());
        };
        let cylinder = pose_of_origin(&collision.origin);
        let along = (cylinder.rotation * Vector3::z()).dot(&placed.own_axis);
        if !(radius > 0.0 && length > 0.0 && (along.abs() - 1.0).abs() <= SAME) {
            return Err(shape());
        }
        let center = cylinder.translation.vector;
        let offset_m = (center - placed.own_axis * center.dot(&placed.own_axis)).norm();
        if offset_m > SAME {
            return Err(NoDifferentialBase::WheelOffAxis {
                joint: placed.name.to_owned(),
                offset_m,
            });
        }
        let carried = self.tree.children_of(self.tree.joint(placed.joint).child);
        if !carried.is_empty() {
            return Err(NoDifferentialBase::WheelCarries {
                joint: placed.name.to_owned(),
                children: carried
                    .iter()
                    .map(|&joint| self.tree.link(self.tree.joint(joint).child).name.clone())
                    .collect(),
            });
        }
        Ok(Wheel {
            joint: placed.name.to_owned(),
            radius_m: radius,
            width_m: length,
        })
    }
}

/// A wheel's continuous joint (an index of the tree), with its name, its origin and its
/// axis in the root link's frame, and its axis in its own frame.
struct PlacedJoint<'d> {
    joint: usize,
    name: &'d str,
    /// Its origin (m).
    position: [f64; 3],
    axis: [f64; 3],
    own_axis: Vector3<f64>,
}

fn same(a: [f64; 3], b: [f64; 3]) -> bool {
    a.iter().zip(b).all(|(a, b)| (a - b).abs() <= SAME)
}

/// How far a collision element of `geometry`, placed by `placed` in the root link's frame,
/// reaches on the floor: the rectangle along the frame's x and y that holds it, for a box, a
/// cylinder and a sphere; none for another shape.
fn floor_reach(geometry: &Geometry, placed: &Isometry3<f64>) -> Option<Outline> {
    let rotation: &UnitQuaternion<f64> = &placed.rotation;
    let [x, y, z] = [Vector3::x(), Vector3::y(), Vector3::z()].map(|unit| rotation * unit);
    let [reach_x, reach_y] = match *geometry {
        Geometry::Sphere { radius } => [radius, radius],
        Geometry::Box { ref size } => {
            let half = size.0.map(|side| side / 2.0);
            let along = |i: usize| {
                [x, y, z]
                    .iter()
                    .zip(half)
                    .map(|(column, half)| column[i].abs() * half)
                    .sum::<f64>()
            };
            [along(0), along(1)]
        }
        Geometry::Cylinder { radius, length } => {
            // The axis's own reach along i, and the disc's: its radius times the part of
            // the axis across i, which the two other components give.
            let along = |i: usize| {
                let across = (0..3)
                    .filter(|&j| j != i)
                    .map(|j| z[j] * z[j])
                    .sum::<f64>()
                    .sqrt();
                z[i].abs() * length / 2.0 + radius * across
            };
            [along(0), along(1)]
        }
        _ => return None,
    };
    let center = placed.translation.vector;
    Some(Outline {
        front: center.x + reach_x,
        rear: center.x - reach_x,
        left: center.y + reach_y,
        right: center.y - reach_y,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A base's URDF: `parts` inside the robot `base`, whose SRDF gives nothing.
    fn base(parts: &str) -> RobotDescription {
        RobotDescription::parse(
            &format!(r#"<robot name="base">{parts}</robot>"#),
            r#"<robot name="base"/>"#,
        )
        .unwrap_or_else(|error| panic!("{error}: {parts}"))
    }

    /// A wheel `name` on a continuous joint at `xyz` about `axis`, its link a cylinder
    /// about the joint's axis of `radius`.
    fn wheel_part(name: &str, xyz: &str, axis: &str, radius: f64) -> String {
        format!(
            r#"<link name="{name}_link"><collision><origin rpy="1.5707963267948966 0 0" xyz="0 0 0"/>
                 <geometry><cylinder length="0.04" radius="{radius}"/></geometry></collision></link>
               <joint name="{name}" type="continuous"><origin rpy="0 0 0" xyz="{xyz}"/>
                 <parent link="base_link"/><child link="{name}_link"/><axis xyz="{axis}"/></joint>"#
        )
    }

    const CHASSIS: &str = r#"<link name="base_link"><collision><origin rpy="0 0 0" xyz="0 0 0.2"/>
          <geometry><box size="0.4 0.3 0.1"/></geometry></collision></link>"#;

    fn refusal(parts: &str) -> String {
        base(parts)
            .differential_base()
            .expect_err("a description that gives no differential base")
            .to_string()
    }

    /// `parts` hung from the root link base_footprint by `joint`, the joint base_joint to
    /// base_link, 0.05 m up and turned by `rpy`.
    fn on_footprint(joint: &str, rpy: &str, parts: &str) -> String {
        format!(
            r#"<link name="base_footprint"/>
               <joint name="base_joint" type="{joint}"><origin rpy="{rpy}" xyz="0 0 0.05"/>
                 <parent link="base_footprint"/><child link="base_link"/><axis xyz="0 0 1"/>
                 <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>{parts}"#
        )
    }

    // The bundled botshare base: wheels 0.1 m in radius on tires 0.046 m wide, on base_link,
    // which a fixed joint holds 0.098 m above the root link, base_footprint, and 0.4 m apart.
    #[test]
    fn the_bundled_base_gives_its_wheels() {
        let base = crate::model("botshare_base")
            .expect("the bundled base")
            .description()
            .differential_base()
            .expect("a differential base");
        assert_eq!(
            base,
            DifferentialBase {
                left: Wheel {
                    joint: "left_wheel_joint".into(),
                    radius_m: 0.1,
                    width_m: 0.046,
                },
                right: Wheel {
                    joint: "right_wheel_joint".into(),
                    radius_m: 0.1,
                    width_m: 0.046,
                },
                separation_m: 0.4,
            }
        );
    }

    // Wheels on a link that a fixed joint holds above the root link are read in the root
    // link's frame: the fixed joint's turn turns their axes there.
    #[test]
    fn wheels_on_a_link_fixed_to_the_root_are_read_in_the_root_links_frame() {
        let wheels = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.05);
        let lifted = base(&on_footprint("fixed", "0 0 0", &wheels))
            .differential_base()
            .expect("a differential base");
        assert_eq!(
            (lifted.left.joint.as_str(), lifted.right.joint.as_str()),
            ("left", "right")
        );
        assert!((lifted.separation_m - 0.4).abs() <= SAME, "{lifted:?}");
        let turned = base(&on_footprint("fixed", "0 0 3.141592653589793", &wheels))
            .differential_base()
            .expect_err("wheels turned backward in the root link's frame");
        assert!(
            matches!(&turned, NoDifferentialBase::WheelAxis { axis, .. } if same(*axis, [0.0, -1.0, 0.0])),
            "{turned}"
        );
    }

    // A continuous joint below a joint that moves is no wheel of the base.
    #[test]
    fn wheels_below_a_moving_joint_are_refused() {
        let wheels = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.05);
        assert_eq!(
            refusal(&on_footprint("revolute", "0 0 0", &wheels)),
            "its wheels are the continuous joints on a link that fixed joints alone join to its root link base_footprint that no group of its SRDF holds, two of them, and it has []"
        );
    }

    // The wheels are the continuous joints on the base, the links that fixed joints alone join
    // to the root link: an arm's continuous wrist below its revolute shoulder is no wheel, and
    // a third continuous joint on the base, such as a spinning lidar's, is refused with the
    // three.
    #[test]
    fn a_continuous_joint_below_a_moving_joint_is_no_wheel_and_one_on_the_base_is() {
        let wheels = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.05);
        let arm = r#"<link name="upper_arm"/>
               <joint name="shoulder" type="revolute"><origin rpy="0 0 0" xyz="0.1 0 0.3"/>
                 <parent link="base_link"/><child link="upper_arm"/><axis xyz="0 0 1"/>
                 <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
               <link name="hand"/>
               <joint name="wrist_roll" type="continuous"><origin rpy="0 0 0" xyz="0 0 0.2"/>
                 <parent link="upper_arm"/><child link="hand"/><axis xyz="0 0 1"/></joint>"#;
        let with_arm = base(&(wheels.clone() + arm))
            .differential_base()
            .expect("the arm's wrist is no wheel");
        assert_eq!(
            (with_arm.left.joint.as_str(), with_arm.right.joint.as_str()),
            ("left", "right")
        );
        let lidar = r#"<link name="lidar"/>
               <joint name="lidar_spin" type="continuous"><origin rpy="0 0 0" xyz="0 0 0.3"/>
                 <parent link="base_link"/><child link="lidar"/><axis xyz="0 0 1"/></joint>"#;
        assert_eq!(
            refusal(&(wheels + lidar)),
            "its wheels are the continuous joints on a link that fixed joints alone join to its root link base_link that no group of its SRDF holds, two of them, and it has [\"left\", \"right\", \"lidar_spin\"]"
        );
    }

    // A continuous joint on the base that a group of the SRDF holds is an arm's, as Kinova
    // Gen3's joint_1 on its base link is, and no wheel; with no group holding it, it counts.
    #[test]
    fn a_continuous_joint_on_the_base_that_a_group_holds_is_no_wheel() {
        let parts = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.05)
            + r#"<link name="shoulder"/>
                 <joint name="joint_1" type="continuous"><origin rpy="0 0 0" xyz="0.1 0 0.3"/>
                   <parent link="base_link"/><child link="shoulder"/><axis xyz="0 0 1"/></joint>"#;
        let with_arm = RobotDescription::parse(
            &format!(r#"<robot name="base">{parts}</robot>"#),
            r#"<robot name="base"><group name="arm"><joint name="joint_1"/></group></robot>"#,
        )
        .expect("a base with an arm")
        .differential_base()
        .expect("joint_1 is the arm's");
        assert_eq!(
            (with_arm.left.joint.as_str(), with_arm.right.joint.as_str()),
            ("left", "right")
        );
        assert_eq!(
            refusal(&parts),
            "its wheels are the continuous joints on a link that fixed joints alone join to its root link base_link that no group of its SRDF holds, two of them, and it has [\"left\", \"right\", \"joint_1\"]"
        );
    }

    #[test]
    fn a_model_with_arms_gives_no_differential_base() {
        let arm = crate::model("so101")
            .expect("the bundled SO-101")
            .description()
            .differential_base()
            .expect_err("an arm");
        assert!(
            matches!(arm, NoDifferentialBase::WheelCount { .. }),
            "{arm}"
        );
    }

    // The left wheel is the one further along +y, whichever the URDF lists first,
    // and each wheel keeps its own radius and the width of its own tire.
    #[test]
    fn the_left_wheel_is_the_one_along_y_and_each_keeps_its_radius() {
        let parts = CHASSIS.to_owned()
            + &wheel_part("starboard", "0 -0.2 0.05", "0 1 0", 0.06)
            + &wheel_part("port", "0 0.2 0.05", "0 1 0", 0.05);
        let base = base(&parts)
            .differential_base()
            .expect("a differential base");
        assert_eq!(
            (
                base.left.joint.as_str(),
                base.left.radius_m,
                base.left.width_m
            ),
            ("port", 0.05, 0.04)
        );
        assert_eq!(
            (
                base.right.joint.as_str(),
                base.right.radius_m,
                base.right.width_m
            ),
            ("starboard", 0.06, 0.04)
        );
        assert!((base.separation_m - 0.4).abs() <= SAME, "{base:?}");
    }

    // A wheel joint turned by its origin to +y counts by its axis in the root's frame.
    #[test]
    fn a_wheel_axis_is_read_in_the_root_links_frame() {
        let turned = |name: &str, xyz: &str| {
            format!(
                r#"<link name="{name}_link"><collision><origin rpy="0 0 0" xyz="0 0 0"/>
                     <geometry><cylinder length="0.04" radius="0.05"/></geometry></collision></link>
                   <joint name="{name}" type="continuous"><origin rpy="-1.5707963267948966 0 0" xyz="{xyz}"/>
                     <parent link="base_link"/><child link="{name}_link"/><axis xyz="0 0 1"/></joint>"#
            )
        };
        let parts =
            CHASSIS.to_owned() + &turned("left", "0 0.2 0.05") + &turned("right", "0 -0.2 0.05");
        assert!(base(&parts).differential_base().is_ok());
    }

    #[test]
    fn a_base_without_two_wheels_on_one_axle_is_refused_naming_the_joints() {
        let one = CHASSIS.to_owned() + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05);
        assert_eq!(
            refusal(&one),
            "its wheels are the continuous joints on a link that fixed joints alone join to its root link base_link that no group of its SRDF holds, two of them, and it has [\"left\"]"
        );
        let backward = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 -1 0", 0.05);
        assert_eq!(
            refusal(&backward),
            "wheel joint right turns about [0.0, -1.0, 0.0] in the root link's frame: a positive turn of a wheel rolls the base forward, about +y"
        );
        let staggered = CHASSIS.to_owned()
            + &wheel_part("left", "0.1 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.05);
        assert_eq!(
            refusal(&staggered),
            "wheel joints left and right stand on no one axle: give their origins the same x and z"
        );
        let square = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + r#"<link name="right_link"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
                 <joint name="right" type="continuous"><origin rpy="0 0 0" xyz="0 -0.2 0.05"/>
                   <parent link="base_link"/><child link="right_link"/><axis xyz="0 1 0"/></joint>"#;
        assert_eq!(
            refusal(&square),
            "wheel link right_link: a wheel's collision geometry is one cylinder about its joint's axis, of a radius and a length above 0"
        );
    }

    // A wheel's cylinder off its joint's axis orbits as the wheel turns, and a link that a
    // wheel carries turns with it: each is refused, naming the joint and what to change.
    #[test]
    fn a_wheel_off_its_axis_or_carrying_a_link_is_refused_naming_its_joint() {
        let off_axis = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.05).replacen(
                r#"<origin rpy="1.5707963267948966 0 0" xyz="0 0 0"/>"#,
                r#"<origin rpy="1.5707963267948966 0 0" xyz="0.01 0 0"/>"#,
                1,
            );
        let refused = refusal(&off_axis);
        assert!(
            refused.starts_with("wheel joint right: the collision cylinder of its link stands 0.01")
                && refused.ends_with(
                    "m off the joint's axis, so it orbits as the wheel turns: center the cylinder on the axis"
                ),
            "{refused}"
        );
        let carrying = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.05)
            + r#"<link name="hub_cap"/>
                 <joint name="hub_cap_joint" type="fixed"><origin rpy="0 0 0" xyz="0 -0.03 0"/>
                   <parent link="left_link"/><child link="hub_cap"/></joint>"#;
        assert_eq!(
            refusal(&carrying),
            "wheel joint left: its link carries the links [\"hub_cap\"], which turn with the wheel: hang them on a link that does not turn"
        );
    }

    // Two wheels at one y give no track, an axle whose midpoint stands off the root
    // link's origin turns the base about another point, and a wheel of no radius rolls
    // nowhere, as one of no width stands on nothing: each is refused, naming what to change.
    #[test]
    fn a_base_whose_axle_gives_no_track_or_stands_off_the_origin_is_refused() {
        let one_side = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 0.2 0.05", "0 1 0", 0.05);
        assert!(
            refusal(&one_side).ends_with(
                "stand at the same y: stand them apart along y, the left wheel along +y"
            ),
            "{}",
            refusal(&one_side)
        );
        let ahead = CHASSIS.to_owned()
            + &wheel_part("left", "0.1 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0.1 -0.2 0.05", "0 1 0", 0.05);
        assert_eq!(
            refusal(&ahead),
            "the axle of wheel joints left and right has its midpoint at [0.1, 0.0] in the root link's frame: stand that midpoint at x 0 and y 0, the point the base turns about"
        );
        let aside = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.5 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.25 0.05", "0 1 0", 0.05);
        assert!(
            refusal(&aside).contains("midpoint at [0.0, 0.125]"),
            "{}",
            refusal(&aside)
        );
        let flat = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.0);
        let thin = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.05)
                .replace(r#"length="0.04""#, r#"length="0""#);
        assert_eq!(
            refusal(&thin),
            "wheel link right_link: a wheel's collision geometry is one cylinder about its joint's axis, of a radius and a length above 0"
        );
        assert_eq!(
            refusal(&flat),
            "wheel link right_link: a wheel's collision geometry is one cylinder about its joint's axis, of a radius and a length above 0"
        );
    }

    fn assert_corners_near(outline: Outline, expected: [[f64; 2]; 4]) {
        let corners = outline.corners();
        assert!(
            corners
                .iter()
                .flatten()
                .zip(expected.iter().flatten())
                .all(|(a, b)| (a - b).abs() <= SAME),
            "{corners:?} != {expected:?}"
        );
    }

    // The bundled botshare base's outline is its chassis's 0.78 m by 0.58 m, which holds
    // the wheels, the casters and the lidar.
    #[test]
    fn the_bundled_base_gives_its_outline() {
        let outline = crate::model("botshare_base")
            .expect("the bundled base")
            .description()
            .outline()
            .expect("an outline");
        assert_corners_near(
            outline,
            [[0.39, -0.29], [0.39, 0.29], [-0.39, 0.29], [-0.39, -0.29]],
        );
    }

    // The outline holds every part placed with every joint at zero: the wheels' cylinders,
    // turned to stand on edge, reach 0.02 m to each side past their joints, beyond the
    // chassis.
    #[test]
    fn the_outline_holds_every_part_at_zero() {
        let parts = CHASSIS.to_owned()
            + &wheel_part("left", "0 0.2 0.05", "0 1 0", 0.05)
            + &wheel_part("right", "0 -0.2 0.05", "0 1 0", 0.05);
        let outline = base(&parts).outline().expect("an outline");
        assert_corners_near(
            outline,
            [[0.2, -0.22], [0.2, 0.22], [-0.2, 0.22], [-0.2, -0.22]],
        );
        let lifted = base(&on_footprint("revolute", "0 0 1.5707963267948966", CHASSIS))
            .outline()
            .expect("an outline");
        assert_corners_near(
            lifted,
            [[0.15, -0.2], [0.15, 0.2], [-0.15, 0.2], [-0.15, -0.2]],
        );
    }

    // A part of another shape than a box, a cylinder or a sphere gives no outline, naming
    // its link, and a robot with no collision element has none.
    #[test]
    fn a_mesh_or_no_collision_element_gives_no_outline() {
        let parts = CHASSIS.to_owned()
            + r#"<link name="bumper"><collision><geometry><mesh filename="bumper.stl"/></geometry></collision></link>
                 <joint name="bumper_joint" type="fixed"><origin rpy="0 0 0" xyz="0.2 0 0.1"/>
                   <parent link="base_link"/><child link="bumper"/></joint>"#;
        assert_eq!(
            base(&parts).outline().expect_err("a mesh").to_string(),
            "link bumper: the outline holds boxes, cylinders and spheres, and one of its collision elements is none of them"
        );
        assert_eq!(
            base(r#"<link name="base_link"/>"#).outline(),
            Err(NoOutline::NoGeometry)
        );
    }
}
