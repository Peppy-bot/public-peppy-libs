//! The model's transforms: where each link of the robot stands in its parent link, from
//! the joints of its URDF, as a robot's publishers send them on transforms. The
//! transforms of the fixed joints hold for the life of the robot; the transform of a
//! joint that moves follows its position.

use std::collections::HashMap;

use chain_kinematics::nalgebra::Isometry3;
use frame_tree::{FrameTransform, TransformErrorReason};

use crate::RobotDescription;

/// Where a child link stands in its parent link: a point `p` of the child link's frame
/// is `pose * p` of the parent link's frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkTransform<'a> {
    pub parent: &'a str,
    pub child: &'a str,
    pub pose: Isometry3<f64>,
}

/// A link transform that the transforms contract does not take: the joint's two links,
/// and why.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[error("the joint from {parent} to {child}: {reason}")]
pub struct LinkTransformError {
    pub parent: String,
    pub child: String,
    pub reason: TransformErrorReason,
}

impl LinkTransform<'_> {
    /// This transform as the transforms contract carries it, checked as
    /// [`FrameTransform::parse`] checks one.
    pub fn frame_transform(&self) -> Result<FrameTransform, LinkTransformError> {
        let origin = self.pose.translation.vector;
        let rotation = self.pose.rotation;
        FrameTransform::parse(
            self.parent,
            self.child,
            [origin.x, origin.y, origin.z],
            [rotation.i, rotation.j, rotation.k, rotation.w],
        )
        .map_err(|reason| LinkTransformError {
            parent: self.parent.to_owned(),
            child: self.child.to_owned(),
            reason,
        })
    }
}

impl RobotDescription {
    /// The transform of every fixed joint of the URDF, in the URDF's order of joints.
    pub fn fixed_transforms(&self) -> Vec<LinkTransform<'_>> {
        self.tree
            .joints()
            .iter()
            .enumerate()
            .filter(|(_, joint)| !joint.placement.kind.is_movable())
            .map(|(index, joint)| self.link_transform(index, joint.placement.origin))
            .collect()
    }

    /// The transform of every joint of the URDF that moves and has a position, in the
    /// URDF's order of joints, with `positions` by joint name in the joint's unit,
    /// radians or meters. A joint that follows another, as its URDF `<mimic>` gives it,
    /// stands where its mimic puts it when `positions` holds the followed joint, and
    /// at its own position otherwise; a joint with neither has no transform.
    pub fn transforms_at(&self, positions: &HashMap<&str, f64>) -> Vec<LinkTransform<'_>> {
        self.tree
            .joints()
            .iter()
            .enumerate()
            .filter(|(_, joint)| joint.placement.kind.is_movable())
            .filter_map(|(index, joint)| {
                let position = joint
                    .mimic
                    .as_ref()
                    .and_then(|mimic| {
                        let leader = positions.get(mimic.joint())?;
                        Some(mimic.position_from(*leader))
                    })
                    .or_else(|| positions.get(joint.name.as_str()).copied())?;
                Some(self.link_transform(index, joint.pose_at(position)))
            })
            .collect()
    }

    /// The transform of the joint `joint`, of the tree, at `pose`.
    fn link_transform(&self, joint: usize, pose: Isometry3<f64>) -> LinkTransform<'_> {
        LinkTransform {
            parent: &self.tree.link(self.tree.parent_of(joint)).name,
            child: &self.tree.link(self.tree.joint(joint).child).name,
            pose,
        }
    }
}
