//  Player - One Hell of a Robot Server
//  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
//                      gerkey@usc.edu    kaspers@robotics.usc.edu
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//  version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
//
//  You should have received a copy of the GNU General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//  Copyright (c) 2008, Willow Garage, Inc.
//  All rights reserved.
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//  version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
//
//  You should have received a copy of the GNU General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// This file translates into Rust Nav2 1.5.2's pose arithmetic of AMCL:
// nav2_amcl/src/pf/pf_vector.c and nav2_amcl/include/nav2_amcl/angleutils.hpp (Copyright (C)
// 2000 Brian Gerkey & Kasper Stoy) and `getOdomPose` of nav2_amcl/src/amcl_node.cpp (Copyright
// (c) 2008, Willow Garage, Inc.).
//
// Rust translation by Peppy, 2026-11-13.
//
// Copyright 2026 Peppy Inc
//
// This file is part of mapping_core.
//
// mapping_core is free software: you can redistribute it and/or modify it under the terms of
// the GNU General Public License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// mapping_core is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// mapping_core, in COPYING. If not, see <https://www.gnu.org/licenses/>.

//! The pose arithmetic of AMCL: `pf_vector_sub` and `pf_vector_coord_add` of nav2_amcl
//! src/pf/pf_vector.c, `normalize` and `angle_diff` of include/nav2_amcl/angleutils.hpp,
//! and the heading that AMCL reads of the odometry (src/amcl_node.cpp `getOdomPose`), which
//! passes through tf2's rotation matrix, as `frame_tree::Transform` holds it.

use crate::geometry::Pose2;
use frame_tree::Transform;
use frame_tree::arithmetic::{quaternion_of_yaw, yaw_of_quaternion};
use std::f64::consts::PI;

/// `pf_vector_sub(a, b)`.
pub(crate) fn sub(a: Pose2, b: Pose2) -> Pose2 {
    Pose2::new(a.x - b.x, a.y - b.y, a.yaw - b.yaw)
}

/// `pf_vector_coord_add(a, b)`: the pose `a` of the frame of `b`, in the frame that `b`
/// lies in, its heading normalized.
pub(crate) fn coord_add(a: Pose2, b: Pose2) -> Pose2 {
    let x = b.x + a.x * b.yaw.cos() - a.y * b.yaw.sin();
    let y = b.y + a.x * b.yaw.sin() + a.y * b.yaw.cos();
    Pose2::new(x, y, normalize(b.yaw + a.yaw))
}

/// `angleutils::normalize`: the angle in [-pi, pi], as `atan2` of its sine and cosine.
pub(crate) fn normalize(angle: f64) -> f64 {
    angle.sin().atan2(angle.cos())
}

/// `angleutils::angle_diff(a, b)`: the turn from `b` to `a`, within pi.
pub(crate) fn angle_diff(a: f64, b: f64) -> f64 {
    let d1 = normalize(a) - normalize(b);
    let d2 = if d1 > 0.0 {
        -(2.0 * PI - d1.abs())
    } else {
        2.0 * PI - d1.abs()
    };
    if d1.abs() < d2.abs() { d1 } else { d2 }
}

/// The heading that `getOdomPose` reads of the base at `yaw` in the odometry frame: the
/// quaternion `setRPY(0, 0, yaw)` that the odometry's transform carries becomes the basis
/// of a `tf2::Transform` when tf2 transforms the base's pose into the odometry frame, and
/// the pose's quaternion is that basis's, whose `getYaw` AMCL reads.
pub(crate) fn odometry_yaw(yaw: f64) -> f64 {
    let basis = Transform::new([0.0; 3], quaternion_of_yaw(yaw))
        .expect("the quaternion of a finite heading makes a transform");
    yaw_of_quaternion(basis.orientation())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_turn_between_two_angles_lies_within_pi() {
        assert!((angle_diff(3.0, -3.0) - (6.0 - 2.0 * PI)).abs() < 1e-14);
        assert!((angle_diff(-3.0, 3.0) + (6.0 - 2.0 * PI)).abs() < 1e-14);
        assert!((angle_diff(0.5, 0.25) - 0.25).abs() < 1e-14);
    }

    #[test]
    fn the_odometrys_heading_comes_back_within_a_rounding_on_either_branch_of_the_matrix() {
        for yaw in [0.0, 0.3, -1.9, 2.2, 3.1, -3.1] {
            assert!((odometry_yaw(yaw) - yaw).abs() < 1e-14, "{yaw}");
        }
    }

    #[test]
    fn a_pose_at_the_origin_of_a_frame_is_the_frames_pose_its_heading_normalized() {
        let frame = Pose2::new(1.0, -2.0, 4.0);
        let moved = coord_add(Pose2::default(), frame);
        assert_eq!((moved.x, moved.y), (1.0, -2.0));
        assert!((moved.yaw - (4.0 - 2.0 * PI)).abs() < 1e-14);
    }
}
