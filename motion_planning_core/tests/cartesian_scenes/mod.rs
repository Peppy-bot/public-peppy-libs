//! The cases of the ROS reference harness's Cartesian paths scenarios as this crate computes
//! them, in the scene the arm planning scenarios' module (`arm_scenes`) builds of a case's
//! parts. The example `cartesian_scenes` and the parity test share them, so the paths each
//! scenario carries are the ones the test computes.
#![allow(dead_code)]

use chain_kinematics::Chain;
use motion_planning_core::{
    Arm, CartesianFailure, CartesianRequest, Collisions, FollowedFrame, Frame, cartesian_path,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use ros_reference::cartesian_paths::{Offset, Ours, PathRequest, Paths};

use crate::arm_scenes::{JOINTS, SceneParts, Standing, in_the_scene, start_fault_name, state};

/// The frame of `link` of `chain`, whose tip link is `tip_link`: the grasp point at the tip
/// link, and else the child link of a joint of the chain.
fn frame_of(chain: &Chain<JOINTS>, tip_link: &str, link: &str) -> Frame {
    if link == tip_link {
        return Frame::GraspPoint;
    }
    let posed = chain.at(&[0.0; JOINTS]);
    let segment = (0..JOINTS)
        .find(|&segment| posed.link_name(segment) == link)
        .unwrap_or_else(|| panic!("{link} is the child link of no joint of the chain"));
    Frame::Link {
        segment,
        offset: Isometry3::identity(),
    }
}

/// The pose of `frame`, one [`frame_of`] gives, of `chain` at `q`.
fn frame_pose(chain: &Chain<JOINTS>, frame: Frame, q: &[f64; JOINTS]) -> Isometry3<f64> {
    match frame {
        Frame::GraspPoint => chain.world_ee_pose(q),
        Frame::Link { segment, offset } => chain.at(q).link_pose_world(segment) * offset,
        Frame::Part { .. } => panic!("a Cartesian path follows no part a link carries"),
    }
}

/// The pose `offset` names from `from`.
fn moved_by(from: &Isometry3<f64>, offset: &Offset) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(from.translation.vector + Vector3::from(offset.translation)),
        from.rotation * UnitQuaternion::from_scaled_axis(Vector3::from(offset.rotation)),
    )
}

/// The poses the lines of `request` run through, by this crate's forward kinematics of
/// `chain`, whose tip link is `tip_link`: its link's pose at the start, then each line's end.
pub fn corners<O>(
    chain: &Chain<JOINTS>,
    tip_link: &str,
    request: &PathRequest<O>,
) -> Vec<Isometry3<f64>> {
    let start: [f64; JOINTS] = state(&request.start);
    let from = frame_pose(chain, frame_of(chain, tip_link, &request.link), &start);
    std::iter::once(from)
        .chain(request.offsets.iter().map(|offset| moved_by(&from, offset)))
        .collect()
}

/// This crate's answer to `request`, a case of `paths`, by `arms`, the scenario's one arm,
/// among the parts of `standing` the arm does not move.
pub fn path_answer<O>(
    arms: &[Arm<JOINTS>],
    standing: &mut Standing,
    paths: &Paths,
    request: &PathRequest<O>,
) -> Ours {
    let chain = arms[0].geometry().chain();
    let tip_link = paths.chains[0].tip_link.as_str();
    let frame = FollowedFrame::of(chain, frame_of(chain, tip_link, &request.link))
        .unwrap_or_else(|| panic!("a Cartesian path follows {}", request.link));
    let poses = corners(chain, tip_link, request);
    let start: [f64; JOINTS] = state(&request.start);
    let world = SceneParts {
        voxel_size: paths.voxel_size,
        map_half_side: paths.map_half_side,
        boxes: &request.boxes,
        objects: &request.objects,
        attached: &request.attached,
        allowed: &request.allowed,
    };
    in_the_scene(
        arms,
        standing,
        &paths.chains,
        &world,
        &start,
        |group, met| {
            let answer = cartesian_path(
                group,
                &CartesianRequest {
                    start,
                    start_tolerance: [0.0; JOINTS],
                    frame,
                    waypoints: &poses[1..],
                    collisions: if request.avoid_collisions {
                        Collisions::Checked {
                            scene: met,
                            min_clearance_m: paths.min_clearance,
                        }
                    } else {
                        Collisions::Unchecked
                    },
                    seed: u64::from(request.seed),
                    stop: None,
                },
            );
            match answer {
                Ok(path) => Ours::Path {
                    fraction: path.fraction,
                    waypoints: path.waypoints.iter().map(|q| q.to_vec()).collect(),
                },
                Err(CartesianFailure::Start(fault)) => Ours::Failure {
                    failure: start_fault_name(&fault).to_string(),
                },
                Err(failure @ (CartesianFailure::Request(_) | CartesianFailure::Stopped)) => {
                    panic!("the case's request fails: {failure}")
                }
            }
        },
    )
}
