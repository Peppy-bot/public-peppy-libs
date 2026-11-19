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
// This file translates into Rust Nav2 1.5.2's motion models of AMCL:
// nav2_amcl/src/motion_model/differential_motion_model.cpp and omni_motion_model.cpp (Copyright
// (C) 2000 Brian Gerkey & Kasper Stoy).
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

//! AMCL's motion models (nav2_amcl src/motion_model), which `robot_model_type` names:
//! `DifferentialMotionModel::odometryUpdate`, which follows sample_motion_model_odometry of
//! Probabilistic Robotics, and `OmniMotionModel::odometryUpdate`, for a base that also
//! moves sideways. Each moves every particle by the odometry's motion since the filter's
//! last update, with Gaussian noise drawn on the filter's generator.
//!
//! The differential model reads the motion as a turn, a move and a turn, the first turn 0
//! for a move under 1 cm, each drawn with noise: of a turn, from the turn (`alpha1`) and
//! from the move (`alpha2`); of the move, from the move (`alpha3`) and from the turns
//! (`alpha4`). A turn's noise grows with its distance from 0 or from pi, whichever is
//! nearer, so a move backward turns as little as one forward. The omnidirectional model
//! reads the motion as a move along its bearing from the particle's heading, a turn and a
//! sideways move, each drawn with noise: of the move, from the move (`alpha3`) and from the
//! turn (`alpha4`); of the turn, from the turn (`alpha1`) and from the move (`alpha2`); of
//! the sideways move, from the turn (`alpha4`) and from the move (`alpha5`).

use super::drand48::Drand48;
use super::particle_filter::Sample;
use super::pose_arithmetic::{angle_diff, sub};
use crate::geometry::Pose2;
use serde::Deserialize;
use std::f64::consts::PI;

/// The shortest move (m) that the differential model reads with a first turn.
const SHORTEST_MOVE_WITH_A_TURN: f64 = 0.01;

/// AMCL's motion model, by the plugin name that `robot_model_type` gives it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum RobotModelType {
    #[serde(rename = "nav2_amcl::DifferentialMotionModel")]
    Differential,
    #[serde(rename = "nav2_amcl::OmniMotionModel")]
    Omnidirectional,
}

/// A motion model with its variance coefficients `alpha1` to `alpha5`, each 0 or more.
/// The differential model reads the first four, the omnidirectional model all five.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MotionModel {
    pub robot_model_type: RobotModelType,
    pub alphas: [f64; 5],
}

/// `std::min` of two magnitudes: the second when it lies below the first.
fn smaller(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

impl MotionModel {
    /// `odometryUpdate`: each particle of `samples` moved by the motion `delta` that took
    /// the odometry to `pose`, with the noise that `generator` draws, three draws of
    /// `pf_ran_gaussian` a particle.
    pub fn odometry_update(
        &self,
        samples: &mut [Sample],
        generator: &mut Drand48,
        pose: Pose2,
        delta: Pose2,
    ) {
        match self.robot_model_type {
            RobotModelType::Differential => self.differential(samples, generator, pose, delta),
            RobotModelType::Omnidirectional => {
                self.omnidirectional(samples, generator, pose, delta)
            }
        }
    }

    /// `DifferentialMotionModel::odometryUpdate`.
    fn differential(
        &self,
        samples: &mut [Sample],
        generator: &mut Drand48,
        pose: Pose2,
        delta: Pose2,
    ) {
        let [alpha1, alpha2, alpha3, alpha4, _] = self.alphas;
        let old_pose = sub(pose, delta);
        let delta_rot1 =
            if (delta.y * delta.y + delta.x * delta.x).sqrt() < SHORTEST_MOVE_WITH_A_TURN {
                0.0
            } else {
                angle_diff(delta.y.atan2(delta.x), old_pose.yaw)
            };
        let delta_trans = (delta.x * delta.x + delta.y * delta.y).sqrt();
        let delta_rot2 = angle_diff(delta.yaw, delta_rot1);
        let turn_noise =
            |turn: f64| smaller(angle_diff(turn, 0.0).abs(), angle_diff(turn, PI).abs());
        let delta_rot1_noise = turn_noise(delta_rot1);
        let delta_rot2_noise = turn_noise(delta_rot2);
        for sample in samples {
            let delta_rot1_hat = angle_diff(
                delta_rot1,
                generator.gaussian(
                    (alpha1 * delta_rot1_noise * delta_rot1_noise
                        + alpha2 * delta_trans * delta_trans)
                        .sqrt(),
                ),
            );
            let delta_trans_hat = delta_trans
                - generator.gaussian(
                    (alpha3 * delta_trans * delta_trans
                        + alpha4 * delta_rot1_noise * delta_rot1_noise
                        + alpha4 * delta_rot2_noise * delta_rot2_noise)
                        .sqrt(),
                );
            let delta_rot2_hat = angle_diff(
                delta_rot2,
                generator.gaussian(
                    (alpha1 * delta_rot2_noise * delta_rot2_noise
                        + alpha2 * delta_trans * delta_trans)
                        .sqrt(),
                ),
            );
            let pose = &mut sample.pose;
            pose.x += delta_trans_hat * (pose.yaw + delta_rot1_hat).cos();
            pose.y += delta_trans_hat * (pose.yaw + delta_rot1_hat).sin();
            pose.yaw += delta_rot1_hat + delta_rot2_hat;
        }
    }

    /// `OmniMotionModel::odometryUpdate`.
    fn omnidirectional(
        &self,
        samples: &mut [Sample],
        generator: &mut Drand48,
        pose: Pose2,
        delta: Pose2,
    ) {
        let [alpha1, alpha2, alpha3, alpha4, alpha5] = self.alphas;
        let old_pose = sub(pose, delta);
        let delta_trans = (delta.x * delta.x + delta.y * delta.y).sqrt();
        let delta_rot = delta.yaw;
        let trans_hat_stddev =
            (alpha3 * (delta_trans * delta_trans) + alpha4 * (delta_rot * delta_rot)).sqrt();
        let rot_hat_stddev =
            (alpha1 * (delta_rot * delta_rot) + alpha2 * (delta_trans * delta_trans)).sqrt();
        let strafe_hat_stddev =
            (alpha4 * (delta_rot * delta_rot) + alpha5 * (delta_trans * delta_trans)).sqrt();
        for sample in samples {
            let delta_bearing = angle_diff(delta.y.atan2(delta.x), old_pose.yaw) + sample.pose.yaw;
            let cs_bearing = delta_bearing.cos();
            let sn_bearing = delta_bearing.sin();
            let delta_trans_hat = delta_trans + generator.gaussian(trans_hat_stddev);
            let delta_rot_hat = delta_rot + generator.gaussian(rot_hat_stddev);
            let delta_strafe_hat = 0.0 + generator.gaussian(strafe_hat_stddev);
            let pose = &mut sample.pose;
            pose.x += delta_trans_hat * cs_bearing + delta_strafe_hat * sn_bearing;
            pose.y += delta_trans_hat * sn_bearing - delta_strafe_hat * cs_bearing;
            pose.yaw += delta_rot_hat;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn noiseless(robot_model_type: RobotModelType) -> MotionModel {
        MotionModel {
            robot_model_type,
            alphas: [0.0; 5],
        }
    }

    fn moved(model: MotionModel, particle: Pose2, pose: Pose2, delta: Pose2) -> Pose2 {
        let mut samples = [Sample {
            pose: particle,
            weight: 1.0,
        }];
        model.odometry_update(&mut samples, &mut Drand48::seeded(0), pose, delta);
        samples[0].pose
    }

    fn assert_near(pose: Pose2, x: f64, y: f64, yaw: f64) {
        assert!(
            (pose.x - x).abs() < 1e-12 && (pose.y - y).abs() < 1e-12,
            "{pose:?}"
        );
        assert!((pose.yaw - yaw).abs() < 1e-12, "{pose:?}");
    }

    #[test]
    fn with_no_noise_a_particle_moves_by_the_odometrys_motion_in_its_own_frame() {
        // The odometry moved 1 m ahead from heading 0 and turned a quarter turn left.
        let pose = Pose2::new(1.0, 0.0, FRAC_PI_2);
        let delta = Pose2::new(1.0, 0.0, FRAC_PI_2);
        let particle = Pose2::new(1.0, 2.0, FRAC_PI_2);
        for robot_model_type in [
            RobotModelType::Differential,
            RobotModelType::Omnidirectional,
        ] {
            let moved = moved(noiseless(robot_model_type), particle, pose, delta);
            assert_near(moved, 1.0, 3.0, PI);
        }
    }

    #[test]
    fn with_no_noise_the_omnidirectional_model_moves_a_particle_sideways_as_the_odometry() {
        // The odometry, at heading 0, moved 1 m to its left without turning.
        let pose = Pose2::new(0.0, 1.0, 0.0);
        let delta = Pose2::new(0.0, 1.0, 0.0);
        let particle = Pose2::new(1.0, 2.0, FRAC_PI_2);
        let moved = moved(
            noiseless(RobotModelType::Omnidirectional),
            particle,
            pose,
            delta,
        );
        assert_near(moved, 0.0, 2.0, FRAC_PI_2);
    }

    #[test]
    fn robot_model_type_reads_amcls_plugin_names() {
        let read = |name: &str| serde_json::from_value::<RobotModelType>(serde_json::json!(name));
        assert_eq!(
            read("nav2_amcl::DifferentialMotionModel").expect("a model"),
            RobotModelType::Differential
        );
        assert_eq!(
            read("nav2_amcl::OmniMotionModel").expect("a model"),
            RobotModelType::Omnidirectional
        );
        assert!(read("nav2_amcl::AckermannMotionModel").is_err());
    }
}
