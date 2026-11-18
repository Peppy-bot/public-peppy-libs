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

//! AMCL's parameters that global localization reads, by the names of Nav2 1.5.2's AMCL
//! node (src/amcl_node.cpp `initParameters`), for its likelihood field model and its
//! motion models.

use super::likelihood_field::LikelihoodField;
use super::motion_model::{MotionModel, RobotModelType};
use super::particle_filter::FilterParameters;
use thiserror::Error;

/// AMCL's parameters, by the names of its node. Each is required.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AmclParameters {
    /// The variance coefficients of the motion model's noise, each 0 or more: of a turn,
    /// from the turn (`alpha1`) and from the move (`alpha2`); of a move, from the move
    /// (`alpha3`) and from the turns (`alpha4`); of a sideways move of the omnidirectional
    /// model, from the turn (`alpha4`) and from the move (`alpha5`).
    pub alpha1: f64,
    pub alpha2: f64,
    pub alpha3: f64,
    pub alpha4: f64,
    pub alpha5: f64,
    /// How far (m) from an occupied cell the likelihood field reaches, above 0.
    pub laser_likelihood_max_dist: f64,
    /// The range limits (m) within which AMCL reads a scan: each above 0 tightens the
    /// scan's own.
    pub laser_max_range: f64,
    pub laser_min_range: f64,
    /// How many beams of a scan weigh a particle, from 2.
    pub max_beams: u32,
    /// The particles that global localization spreads, and the most that resampling
    /// keeps, from 1.
    pub max_particles: u32,
    /// The fewest particles that resampling keeps, at most `max_particles`.
    pub min_particles: u32,
    /// KLD sampling's bound on the error of the particles' distribution, above 0, and its
    /// normal quantile.
    pub pf_err: f64,
    pub pf_z: f64,
    /// How many filter updates go between two resamplings, from 1.
    pub resample_interval: u32,
    /// The motion model that moves the particles.
    pub robot_model_type: RobotModelType,
    /// The standard deviation (m) of a beam's end about the nearest occupied cell, above 0.
    pub sigma_hit: f64,
    /// How far the odometry turns (rad) or moves (m), 0 or more, before the filter updates.
    pub update_min_a: f64,
    pub update_min_d: f64,
    /// The weights, each 0 or more, of a hit and of a random reading in a beam's
    /// likelihood.
    pub z_hit: f64,
    pub z_rand: f64,
}

/// Why parameters make no global localization.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum AmclParameterError {
    #[error("{name} is {value}, and every parameter of AMCL is finite")]
    NotFinite { name: &'static str, value: f64 },
    #[error("{name} is {value}, and it must be above 0")]
    NotPositive { name: &'static str, value: f64 },
    #[error("{name} is {value}, and it must be 0 or more")]
    Negative { name: &'static str, value: f64 },
    #[error("max_beams is {0}, and the likelihood field weighs a particle by 2 beams or more")]
    TooFewBeams(u32),
    #[error("max_particles is 0, and global localization spreads 1 particle or more")]
    NoParticles,
    #[error(
        "min_particles is {min}, above max_particles {max}: give a min_particles of \
         max_particles or fewer"
    )]
    MinAboveMax { min: u32, max: u32 },
    #[error("resample_interval is 0, and it must be 1 or more")]
    NoResampleInterval,
}

/// The parameters checked, as the parts of AMCL take them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CheckedParameters {
    pub filter: FilterParameters,
    pub motion: MotionModel,
    pub laser: LikelihoodField,
    pub laser_likelihood_max_dist: f64,
    pub update_min_a: f64,
    pub update_min_d: f64,
    pub resample_interval: u32,
}

impl AmclParameters {
    pub(crate) fn checked(&self) -> Result<CheckedParameters, AmclParameterError> {
        let reals = [
            ("alpha1", self.alpha1),
            ("alpha2", self.alpha2),
            ("alpha3", self.alpha3),
            ("alpha4", self.alpha4),
            ("alpha5", self.alpha5),
            ("laser_likelihood_max_dist", self.laser_likelihood_max_dist),
            ("laser_max_range", self.laser_max_range),
            ("laser_min_range", self.laser_min_range),
            ("pf_err", self.pf_err),
            ("pf_z", self.pf_z),
            ("sigma_hit", self.sigma_hit),
            ("update_min_a", self.update_min_a),
            ("update_min_d", self.update_min_d),
            ("z_hit", self.z_hit),
            ("z_rand", self.z_rand),
        ];
        if let Some((name, value)) = reals.into_iter().find(|(_, value)| !value.is_finite()) {
            return Err(AmclParameterError::NotFinite { name, value });
        }
        let positive = [
            ("laser_likelihood_max_dist", self.laser_likelihood_max_dist),
            ("pf_err", self.pf_err),
            ("sigma_hit", self.sigma_hit),
        ];
        if let Some((name, value)) = positive.into_iter().find(|(_, value)| *value <= 0.0) {
            return Err(AmclParameterError::NotPositive { name, value });
        }
        let at_least_zero = [
            ("alpha1", self.alpha1),
            ("alpha2", self.alpha2),
            ("alpha3", self.alpha3),
            ("alpha4", self.alpha4),
            ("alpha5", self.alpha5),
            ("update_min_a", self.update_min_a),
            ("update_min_d", self.update_min_d),
            ("z_hit", self.z_hit),
            ("z_rand", self.z_rand),
        ];
        if let Some((name, value)) = at_least_zero.into_iter().find(|(_, value)| *value < 0.0) {
            return Err(AmclParameterError::Negative { name, value });
        }
        if self.max_beams < 2 {
            return Err(AmclParameterError::TooFewBeams(self.max_beams));
        }
        if self.max_particles == 0 {
            return Err(AmclParameterError::NoParticles);
        }
        if self.min_particles > self.max_particles {
            return Err(AmclParameterError::MinAboveMax {
                min: self.min_particles,
                max: self.max_particles,
            });
        }
        if self.resample_interval == 0 {
            return Err(AmclParameterError::NoResampleInterval);
        }
        Ok(CheckedParameters {
            filter: FilterParameters {
                min_samples: self.min_particles as usize,
                max_samples: self.max_particles as usize,
                pop_err: self.pf_err,
                pop_z: self.pf_z,
            },
            motion: MotionModel {
                robot_model_type: self.robot_model_type,
                alphas: [
                    self.alpha1,
                    self.alpha2,
                    self.alpha3,
                    self.alpha4,
                    self.alpha5,
                ],
            },
            laser: LikelihoodField {
                z_hit: self.z_hit,
                z_rand: self.z_rand,
                sigma_hit: self.sigma_hit,
                max_beams: self.max_beams as usize,
                laser_min_range: self.laser_min_range,
                laser_max_range: self.laser_max_range,
            },
            laser_likelihood_max_dist: self.laser_likelihood_max_dist,
            update_min_a: self.update_min_a,
            update_min_d: self.update_min_d,
            resample_interval: self.resample_interval,
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// The parameters of Nav2's nav2_params.yaml, with recovery rates of 0.
    pub(crate) fn nav2_parameters() -> AmclParameters {
        AmclParameters {
            alpha1: 0.2,
            alpha2: 0.2,
            alpha3: 0.2,
            alpha4: 0.2,
            alpha5: 0.2,
            laser_likelihood_max_dist: 2.0,
            laser_max_range: 100.0,
            laser_min_range: -1.0,
            max_beams: 60,
            max_particles: 2000,
            min_particles: 500,
            pf_err: 0.05,
            pf_z: 0.99,
            resample_interval: 1,
            robot_model_type: RobotModelType::Differential,
            sigma_hit: 0.2,
            update_min_a: 0.2,
            update_min_d: 0.25,
            z_hit: 0.5,
            z_rand: 0.5,
        }
    }

    fn refusal(parameters: AmclParameters) -> Option<AmclParameterError> {
        parameters.checked().err()
    }

    #[test]
    fn nav2s_parameters_are_accepted() {
        assert_eq!(refusal(nav2_parameters()), None);
    }

    #[test]
    fn a_parameter_that_is_not_finite_is_refused_by_name() {
        let value = f64::INFINITY;
        let parameters = AmclParameters {
            alpha3: value,
            ..nav2_parameters()
        };
        assert_eq!(
            refusal(parameters),
            Some(AmclParameterError::NotFinite {
                name: "alpha3",
                value
            })
        );
        let parameters = AmclParameters {
            z_rand: f64::NAN,
            ..nav2_parameters()
        };
        assert!(matches!(
            refusal(parameters),
            Some(AmclParameterError::NotFinite { name: "z_rand", value }) if value.is_nan()
        ));
    }

    #[test]
    fn a_parameter_that_must_lie_above_0_is_refused_at_0() {
        let parameters = AmclParameters {
            sigma_hit: 0.0,
            ..nav2_parameters()
        };
        assert_eq!(
            refusal(parameters),
            Some(AmclParameterError::NotPositive {
                name: "sigma_hit",
                value: 0.0
            })
        );
    }

    #[test]
    fn a_parameter_that_must_be_0_or_more_is_refused_below_0_and_accepted_at_0() {
        let parameters = AmclParameters {
            update_min_d: -0.1,
            ..nav2_parameters()
        };
        assert_eq!(
            refusal(parameters),
            Some(AmclParameterError::Negative {
                name: "update_min_d",
                value: -0.1
            })
        );
        let parameters = AmclParameters {
            update_min_d: 0.0,
            ..nav2_parameters()
        };
        assert_eq!(refusal(parameters), None);
    }

    #[test]
    fn fewer_than_2_beams_are_refused() {
        let parameters = AmclParameters {
            max_beams: 1,
            ..nav2_parameters()
        };
        assert_eq!(
            refusal(parameters),
            Some(AmclParameterError::TooFewBeams(1))
        );
    }

    #[test]
    fn no_particles_are_refused() {
        let parameters = AmclParameters {
            max_particles: 0,
            min_particles: 0,
            ..nav2_parameters()
        };
        assert_eq!(refusal(parameters), Some(AmclParameterError::NoParticles));
    }

    #[test]
    fn a_min_particles_above_max_particles_is_refused_and_one_equal_to_it_accepted() {
        let parameters = AmclParameters {
            min_particles: 2001,
            ..nav2_parameters()
        };
        assert_eq!(
            refusal(parameters),
            Some(AmclParameterError::MinAboveMax {
                min: 2001,
                max: 2000
            })
        );
        let parameters = AmclParameters {
            min_particles: 2000,
            ..nav2_parameters()
        };
        assert_eq!(refusal(parameters), None);
    }

    #[test]
    fn a_resample_interval_of_0_is_refused() {
        let parameters = AmclParameters {
            resample_interval: 0,
            ..nav2_parameters()
        };
        assert_eq!(
            refusal(parameters),
            Some(AmclParameterError::NoResampleInterval)
        );
    }
}
