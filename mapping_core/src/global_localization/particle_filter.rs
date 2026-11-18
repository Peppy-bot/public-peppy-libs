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
// This file translates into Rust Nav2 1.5.2's particle filter of AMCL: nav2_amcl/src/pf/pf.c
// (Copyright (C) 2000 Brian Gerkey & Kasper Stoy) and `getMaxWeightHyp` of
// nav2_amcl/src/amcl_node.cpp (Copyright (c) 2008, Willow Garage, Inc.).
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

//! AMCL's particle filter (nav2_amcl src/pf/pf.c: `pf_alloc`, `pf_init_model`,
//! `pf_update_sensor`, `pf_update_resample`, `pf_resample_limit`, `pf_cluster_stats`, and
//! src/amcl_node.cpp `getMaxWeightHyp`), with the population parameters that
//! `initParticleFilter` sets and recovery rates of 0. Resampling draws each particle of the
//! new set from the old one by weight, and stops once the set holds as many particles as
//! KLD sampling asks for the bins of the kd-tree they fill. The statistics of the set,
//! computed when it is made, are each cluster's weight and mean and the whole set's
//! covariance, which `amcl_pose` carries.

use super::drand48::Drand48;
use super::kdtree::KdTree;
use crate::geometry::{Matrix3, Pose2, truncate_to_i32};

/// `pf_sample_t`: a particle's pose and weight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Sample {
    pub pose: Pose2,
    pub weight: f64,
}

/// A cluster's weight and mean, of `pf_cluster_t`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Cluster {
    weight: f64,
    mean: Pose2,
}

/// The parameters of the filter: the fewest and the most particles, and KLD sampling's
/// error bound and normal quantile (`pop_err`, `pop_z`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FilterParameters {
    pub min_samples: usize,
    pub max_samples: usize,
    pub pop_err: f64,
    pub pop_z: f64,
}

/// `pf_t`, of its current sample set: the particles, their clusters, and the covariance
/// of the whole set: of x and y in the upper 2 x 2 block, and of the heading, the circular
/// variance `-2 ln R` of the weighted mean length `R` of the heading vectors.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ParticleFilter {
    parameters: FilterParameters,
    samples: Vec<Sample>,
    clusters: Vec<Cluster>,
    covariance: Matrix3,
}

impl ParticleFilter {
    /// `pf_alloc`, with no particle until [`ParticleFilter::init_model`].
    pub fn new(parameters: FilterParameters) -> Self {
        Self {
            parameters,
            samples: Vec::new(),
            clusters: Vec::new(),
            covariance: Matrix3::zero(),
        }
    }

    pub fn samples_mut(&mut self) -> &mut [Sample] {
        &mut self.samples
    }

    /// `set->cov`: the covariance of the whole set, as of its last statistics.
    pub fn covariance(&self) -> &Matrix3 {
        &self.covariance
    }

    /// `pf_init_model`: the most particles, each of equal weight at the pose that
    /// `init_pose` draws.
    pub fn init_model(
        &mut self,
        generator: &mut Drand48,
        mut init_pose: impl FnMut(&mut Drand48) -> Pose2,
    ) {
        let max_samples = self.parameters.max_samples;
        let weight = 1.0 / max_samples as f64;
        let mut kdtree = KdTree::new(max_samples);
        let samples: Vec<Sample> = (0..max_samples)
            .map(|_| {
                let pose = init_pose(generator);
                kdtree.insert(pose, weight);
                Sample { pose, weight }
            })
            .collect();
        (self.clusters, self.covariance) = statistics_of(&samples, &mut kdtree);
        self.samples = samples;
    }

    /// `pf_update_sensor`: the weights that `weigh` gives the particles, which it sums,
    /// normalized; equal weights when they sum to no more than 0.
    pub fn update_sensor(&mut self, weigh: impl FnOnce(&mut [Sample]) -> f64) {
        let total = weigh(&mut self.samples);
        let count = self.samples.len() as f64;
        if total <= 0.0 || total.is_nan() {
            for sample in &mut self.samples {
                sample.weight = 1.0 / count;
            }
            return;
        }
        for sample in &mut self.samples {
            sample.weight /= total;
        }
    }

    /// `pf_update_resample`: a new set of particles, each an old particle drawn by weight;
    /// the set stops growing once it holds more particles than KLD sampling asks for the
    /// bins it fills. Then the set's statistics are computed again.
    pub fn update_resample(&mut self, generator: &mut Drand48) {
        let old = &self.samples;
        let cumulative: Vec<f64> = std::iter::once(0.0)
            .chain(old.iter().scan(0.0, |sum, sample| {
                *sum += sample.weight;
                Some(*sum)
            }))
            .collect();
        let max_samples = self.parameters.max_samples;
        let mut kdtree = KdTree::new(max_samples);
        let mut samples: Vec<Sample> = Vec::with_capacity(max_samples);
        let mut total = 0.0;
        while samples.len() < max_samples {
            // AMCL draws whether to take a random pose, which recovery rates of 0 never
            // take: the draw moves the generator's stream on.
            generator.draw();
            let r = generator.draw();
            let pose = old[drawn_by_weight(&cumulative, r)].pose;
            let weight = 1.0;
            total += weight;
            kdtree.insert(pose, weight);
            samples.push(Sample { pose, weight });
            if samples.len() > self.resample_limit(kdtree.leaf_count()) {
                break;
            }
        }
        for sample in &mut samples {
            sample.weight /= total;
        }
        (self.clusters, self.covariance) = statistics_of(&samples, &mut kdtree);
        self.samples = samples;
    }

    /// `pf_resample_limit`: the particles that KLD sampling asks for `k` filled bins
    /// (Fox), from the fewest to the most; the most for one bin or none.
    fn resample_limit(&self, k: usize) -> usize {
        let FilterParameters {
            min_samples,
            max_samples,
            pop_err,
            pop_z,
            ..
        } = self.parameters;
        if k <= 1 {
            return max_samples;
        }
        let a = 1.0;
        let b = 2.0 / (9.0 * (k as f64 - 1.0));
        let c = (2.0 / (9.0 * (k as f64 - 1.0))).sqrt() * pop_z;
        let x = a - b + c;
        let n = i64::from(truncate_to_i32(
            ((k - 1) as f64 / (2.0 * pop_err) * x * x * x).ceil(),
        ));
        if n < min_samples as i64 {
            return min_samples;
        }
        if n > max_samples as i64 {
            return max_samples;
        }
        n as usize
    }

    /// `getMaxWeightHyp`: the mean of the cluster of the largest weight, the first of
    /// equals, when it weighs more than 0.
    pub fn heaviest_cluster_mean(&self) -> Option<Pose2> {
        let (weight, mean) = self
            .clusters
            .iter()
            .fold((0.0, None), |(heaviest, mean), cluster| {
                if cluster.weight > heaviest {
                    (cluster.weight, Some(cluster.mean))
                } else {
                    (heaviest, mean)
                }
            });
        mean.filter(|_| weight > 0.0)
    }
}

/// The naive discrete event sampler of `pf_update_resample`: the first particle whose
/// span of the cumulative weights `cumulative` holds `r`, or the last when `r` lies at or
/// past the end of the spans, which the rounding of the weights' sum leaves below 1.
fn drawn_by_weight(cumulative: &[f64], r: f64) -> usize {
    let count = cumulative.len() - 1;
    (0..count)
        .find(|&i| cumulative[i] <= r && r < cumulative[i + 1])
        .unwrap_or(count - 1)
}

/// `pf_cluster_stats` of `samples`, which `kdtree` holds: the leaves of the tree labeled
/// with their clusters, each cluster's weight and mean, its heading the direction of the
/// weighted sum of its particles' heading vectors, and the covariance of the whole set.
fn statistics_of(samples: &[Sample], kdtree: &mut KdTree) -> (Vec<Cluster>, Matrix3) {
    kdtree.cluster();
    let mut sums = vec![(0.0, [0.0; 4]); kdtree.cluster_count()];
    let mut weight = 0.0;
    let mut m = [0.0; 4];
    let mut c = [[0.0; 2]; 2];
    for sample in samples {
        let index = kdtree
            .cluster_of(sample.pose)
            .expect("each particle's bin is a leaf of the tree that holds the particles");
        let (cluster_weight, cluster_m) = &mut sums[index];
        *cluster_weight += sample.weight;
        weight += sample.weight;
        let terms = [
            sample.weight * sample.pose.x,
            sample.weight * sample.pose.y,
            sample.weight * sample.pose.yaw.cos(),
            sample.weight * sample.pose.yaw.sin(),
        ];
        for (j, term) in terms.into_iter().enumerate() {
            cluster_m[j] += term;
            m[j] += term;
        }
        let position = [sample.pose.x, sample.pose.y];
        for (j, row) in c.iter_mut().enumerate() {
            for (k, value) in row.iter_mut().enumerate() {
                *value += sample.weight * position[j] * position[k];
            }
        }
    }
    let clusters = sums
        .into_iter()
        .map(|(weight, m)| Cluster {
            weight,
            mean: Pose2::new(m[0] / weight, m[1] / weight, m[3].atan2(m[2])),
        })
        .collect();
    let mean = [m[0] / weight, m[1] / weight];
    let mut covariance = Matrix3::zero();
    for (j, row) in c.iter().enumerate() {
        for (k, value) in row.iter().enumerate() {
            covariance.0[j][k] = value / weight - mean[j] * mean[k];
        }
    }
    covariance.0[2][2] = -2.0 * (m[2] * m[2] + m[3] * m[3]).sqrt().ln();
    (clusters, covariance)
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    fn parameters(min_samples: usize, max_samples: usize) -> FilterParameters {
        FilterParameters {
            min_samples,
            max_samples,
            pop_err: 0.05,
            pop_z: 0.99,
        }
    }

    #[test]
    fn kld_sampling_asks_for_the_most_particles_for_one_bin_and_at_least_the_fewest() {
        let filter = ParticleFilter::new(parameters(500, 2000));
        assert_eq!(filter.resample_limit(1), 2000);
        assert_eq!(filter.resample_limit(10), 500);
        let unbounded = ParticleFilter::new(parameters(0, 1_000_000));
        assert_eq!(unbounded.resample_limit(10), 131);
        assert_eq!(unbounded.resample_limit(99), 1118);
        assert_eq!(
            ParticleFilter::new(parameters(0, 1000)).resample_limit(1000),
            1000
        );
    }

    #[test]
    fn a_draw_past_the_cumulative_weights_takes_the_last_particle() {
        let cumulative = [0.0, 0.25, 0.75, 0.875];
        assert_eq!(drawn_by_weight(&cumulative, 0.0), 0);
        assert_eq!(drawn_by_weight(&cumulative, 0.5), 1);
        assert_eq!(drawn_by_weight(&cumulative, 0.9), 2);
    }

    #[test]
    fn resampling_from_one_particle_of_all_the_weight_takes_it_alone() {
        let mut filter = ParticleFilter::new(parameters(1, 4));
        let mut generator = Drand48::seeded(3);
        let mut poses = [(0.0, 0.0), (5.0, 5.0), (9.0, 1.0), (2.0, 8.0)].into_iter();
        filter.init_model(&mut generator, |_| {
            let (x, y) = poses.next().expect("four poses");
            Pose2::new(x, y, 0.0)
        });
        filter.update_sensor(|samples| {
            for (index, sample) in samples.iter_mut().enumerate() {
                sample.weight = if index == 1 { 1.0 } else { 0.0 };
            }
            1.0
        });
        filter.update_resample(&mut generator);
        assert!(
            filter
                .samples
                .iter()
                .all(|sample| sample.pose == Pose2::new(5.0, 5.0, 0.0))
        );
        assert_eq!(filter.covariance().0[0][0], 0.0);
        assert_eq!(filter.covariance().0[1][1], 0.0);
        assert_eq!(
            filter.heaviest_cluster_mean(),
            Some(Pose2::new(5.0, 5.0, 0.0))
        );
    }

    #[hegel::test]
    fn the_covariance_is_that_of_the_whole_set_over_its_clusters(tc: TestCase) {
        // Two groups of particles `2 * half_distance` apart along x, each at one pose, the
        // headings `heading` and `-heading`: two clusters, each with no spread of its own.
        let center = [
            tc.draw(gs::floats::<f64>().min_value(-10.0).max_value(10.0)),
            tc.draw(gs::floats::<f64>().min_value(-10.0).max_value(10.0)),
        ];
        let half_distance: f64 = tc.draw(gs::floats().min_value(0.5).max_value(5.0));
        let heading: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
        let per_group: usize = tc.draw(gs::integers().min_value(1).max_value(50));
        let mut filter = ParticleFilter::new(parameters(1, 2 * per_group));
        let mut drawn = 0;
        filter.init_model(&mut Drand48::seeded(0), |_| {
            drawn += 1;
            let side = if drawn <= per_group { -1.0 } else { 1.0 };
            Pose2::new(center[0] + side * half_distance, center[1], -side * heading)
        });
        let covariance = filter.covariance().0;
        let scale = center[0] * center[0] + half_distance * half_distance;
        assert!(
            (covariance[0][0] - half_distance * half_distance).abs() <= 1e-12 * scale,
            "{covariance:?}"
        );
        for (row, column) in [(0, 1), (1, 0), (1, 1)] {
            assert!(
                covariance[row][column].abs() <= 1e-12 * (scale + center[1] * center[1]),
                "{covariance:?}"
            );
        }
        let circular_variance = -2.0 * heading.cos().ln();
        assert!(
            (covariance[2][2] - circular_variance).abs() <= 1e-12,
            "{covariance:?}"
        );
    }

    #[hegel::test]
    fn kld_sampling_keeps_the_count_between_min_and_max_particles(tc: TestCase) {
        let max_samples: usize = tc.draw(gs::integers().min_value(1).max_value(500));
        let min_samples: usize = tc.draw(gs::integers().min_value(1).max_value(max_samples));
        let spread: f64 = tc.draw(gs::floats().min_value(0.0).max_value(20.0));
        let mut filter = ParticleFilter::new(parameters(min_samples, max_samples));
        let mut generator = Drand48::seeded(tc.draw(gs::integers()));
        filter.init_model(&mut generator, |generator| {
            Pose2::new(
                spread * generator.draw(),
                spread * generator.draw(),
                2.0 * std::f64::consts::PI * generator.draw(),
            )
        });
        for _ in 0..3 {
            filter.update_sensor(|samples| {
                samples.iter_mut().fold(0.0, |total, sample| {
                    sample.weight = generator.draw();
                    total + sample.weight
                })
            });
            filter.update_resample(&mut generator);
            let count = filter.samples.len();
            assert!(
                (min_samples..=max_samples).contains(&count),
                "{count} particles, between {min_samples} and {max_samples}"
            );
        }
    }
}
