// Copyright 2010 SRI International
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// slam_toolbox
// Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
// Copyright Work Modifications (c) 2019, Samsung Research America
//
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
// COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
// COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
// AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
//
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
// BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
// CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
// CONDITIONS.
//
// This file translates into Rust karto's mapper as slam_toolbox 2.10.0 carries it:
// lib/karto_sdk/src/Mapper.cpp and lib/karto_sdk/include/karto_sdk/Mapper.h (Copyright 2010 SRI
// International); and slam_toolbox 2.10.0's loading of a serialized pose graph,
// src/slam_toolbox_common.cpp (Copyright Work Modifications (c) 2018, Simbe Robotics, Inc., and
// (c) 2019, Samsung Research America).
//
// This file conveys karto's code under the GNU General Public License, version 3 or later.
//
// This translation takes slam_toolbox's own code under the GNU Lesser General Public License,
// version 2.1, of slam_toolbox's LICENSE; as section 3 of that license allows, the GNU General
// Public License, version 3 or later, applies in its place.
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

//! karto's mapper, as slam_toolbox 2.10.0 carries it (lib/karto_sdk/src/Mapper.cpp):
//! the scans and the running buffer of one laser (`MapperSensorManager`, `ScanManager`),
//! the processing of a scan against the running buffer (`Mapper::Process`), against the
//! nodes near a pose (`ProcessAgainstNodesNearBy`) and in a loaded map
//! (`ProcessLocalization`, `AddScanToLocalizationBuffer`, `RemoveNodeFromGraph`), the
//! gating of a scan by the base's motion (`HasMovedEnough`), and the graph's links and
//! loop closures (`MapperGraph::AddVertex`, `AddEdges`, `LinkScans`, `LinkNearChains`,
//! `LinkChainToScan`, `GetClosestScanToPose`, `FindNearChains`, `FindNearLinkedScans`,
//! `FindNearByScan`, `ComputeWeightedMean`, `TryCloseLoop`, `FindPossibleLoopClosure`,
//! `CorrectPoses`). Each step reads and writes the scans in karto's order.

use crate::geometry::{Matrix3, Pose2, TOLERANCE, Transform, Vector2, normalize_angle};
use crate::localized_scan::{Laser, LocalizedScan, ScanId, corrected_at, sensor_at};
use crate::optimizer::Optimizer;
use crate::pose_graph::{Edge, Graph, LinkInfo};
use crate::scan_matcher::{Match, MatchParameters, MatcherError, MatcherSizes, ScanMatcher};
use std::collections::{BTreeMap, VecDeque};

/// karto's `MinimumTimeInterval` (s), which slam_toolbox leaves at karto's default: a
/// scan this long after the last processed one has moved enough.
const MINIMUM_TIME_INTERVAL_S: f64 = 3600.0;

/// The mapper's parameters, with the values karto holds: slam_toolbox squares the
/// variances it sets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct KartoParameters {
    pub use_scan_barycenter: bool,
    pub minimum_travel_distance: f64,
    pub minimum_travel_heading: f64,
    pub scan_buffer_size: u32,
    pub scan_buffer_maximum_scan_distance: f64,
    pub link_match_minimum_response_fine: f64,
    pub link_scan_maximum_distance: f64,
    pub loop_search_maximum_distance: f64,
    pub do_loop_closing: bool,
    pub loop_match_minimum_chain_size: u32,
    /// The square of slam_toolbox's `loop_match_maximum_variance_coarse`.
    pub loop_match_maximum_variance_coarse: f64,
    pub loop_match_minimum_response_coarse: f64,
    pub loop_match_minimum_response_fine: f64,
    pub correlation_search_space: MatcherSizes,
    pub loop_search_space: MatcherSizes,
    pub match_parameters: MatchParameters,
}

/// A scan the mapper added, and the covariance of its match.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Processed {
    pub id: ScanId,
    pub covariance: Matrix3,
}

/// What a saved map holds of a mapper: the laser, the scans, the graph's edges in order,
/// the running buffer, the last scan and the next id. Every scan of a map is a vertex of
/// its graph.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct MapperState {
    pub laser: Laser,
    pub scans: BTreeMap<ScanId, LocalizedScan>,
    pub edges: Vec<Edge>,
    pub running_scans: Vec<ScanId>,
    pub last_scan: Option<ScanId>,
    pub next_id: u32,
}

/// `Mapper`, with one laser.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Mapper {
    parameters: KartoParameters,
    laser: Laser,
    scans: BTreeMap<ScanId, LocalizedScan>,
    running_scans: Vec<ScanId>,
    last_scan: Option<ScanId>,
    next_id: u32,
    running_buffer_maximum_size: u32,
    running_buffer_maximum_distance: f64,
    graph: Graph,
    sequential_matcher: ScanMatcher,
    loop_matcher: ScanMatcher,
    optimizer: Optimizer,
    localization_buffer: VecDeque<ScanId>,
}

impl Mapper {
    /// `Mapper::Initialize`: the matchers for the laser's range threshold, and an empty
    /// map.
    pub fn new(parameters: KartoParameters, laser: Laser) -> Result<Self, MatcherError> {
        Ok(Self {
            parameters,
            laser,
            scans: BTreeMap::new(),
            running_scans: Vec::new(),
            last_scan: None,
            next_id: 0,
            running_buffer_maximum_size: parameters.scan_buffer_size,
            running_buffer_maximum_distance: parameters.scan_buffer_maximum_scan_distance,
            graph: Graph::default(),
            sequential_matcher: ScanMatcher::new(
                parameters.match_parameters,
                parameters.correlation_search_space,
                laser.range_threshold,
            )?,
            loop_matcher: ScanMatcher::new(
                parameters.match_parameters,
                parameters.loop_search_space,
                laser.range_threshold,
            )?,
            optimizer: Optimizer::default(),
            localization_buffer: VecDeque::new(),
        })
    }

    /// karto's mapper as slam_toolbox loads a serialized one to localize in
    /// (src/slam_toolbox_common.cpp, `loadSerializedPoseGraph`; Mapper.cpp,
    /// `Mapper::Initialize` of a deserialized mapper): the map's scans, graph, running
    /// buffer and last scan, with `parameters` and new matchers, the running buffer's
    /// distance truncated to whole meters as the scan manager's setter takes it, and the
    /// optimizer made anew of each vertex and each edge in order and solved once.
    pub fn restore(parameters: KartoParameters, state: &MapperState) -> Result<Self, MatcherError> {
        let mut mapper = Self::new(parameters, state.laser)?;
        mapper.scans = state.scans.clone();
        mapper.running_scans = state.running_scans.clone();
        mapper.last_scan = state.last_scan;
        mapper.next_id = state.next_id;
        mapper.running_buffer_maximum_distance =
            f64::from(parameters.scan_buffer_maximum_scan_distance as u32);
        for (&id, scan) in &mapper.scans {
            mapper.graph.add_vertex(id);
            mapper.optimizer.add_node(id, scan.corrected_pose());
        }
        for edge in &state.edges {
            if mapper
                .graph
                .add_edge(edge.source, edge.target, || edge.link)
                .is_some()
            {
                mapper.optimizer.add_constraint(
                    edge.source,
                    edge.target,
                    edge.link.pose_difference,
                    edge.link.covariance,
                );
            }
        }
        mapper.optimizer.compute();
        Ok(mapper)
    }

    /// What a saved map holds of the mapper.
    pub fn state(&self) -> MapperState {
        MapperState {
            laser: self.laser,
            scans: self.scans.clone(),
            edges: self.graph.edges().copied().collect(),
            running_scans: self.running_scans.clone(),
            last_scan: self.last_scan,
            next_id: self.next_id,
        }
    }

    pub fn laser(&self) -> &Laser {
        &self.laser
    }

    /// The scans of the map, by id.
    pub fn scans(&self) -> &BTreeMap<ScanId, LocalizedScan> {
        &self.scans
    }

    /// `Mapper::Process` (or, with `localization`, `ProcessLocalization`): the scan
    /// corrected by the last scan's correction, gated by the base's motion, matched
    /// against the running buffer, added to the graph and linked, and the loops it
    /// closes closed. With `localization`, the scan also joins the localization buffer,
    /// which drops its oldest scan beyond the buffer's size. None when the base did not
    /// move enough.
    pub fn process(
        &mut self,
        mut scan: LocalizedScan,
        localization: bool,
    ) -> Result<Option<Processed>, MatcherError> {
        let laser = self.laser;
        if let Some(last) = self.last_scan.and_then(|id| self.scans.get(&id)) {
            let correction = Transform::between(last.odometric_pose(), last.corrected_pose());
            scan.set_corrected_pose(correction.transform_pose(scan.odometric_pose()), &laser);
        }
        let last = self.last_scan.and_then(|id| self.scans.get(&id));
        if !self.has_moved_enough(&scan, last) {
            return Ok(None);
        }
        let mut covariance = Matrix3::identity();
        if last.is_some() {
            let found = self.match_running_buffer(&scan)?;
            scan.set_sensor_pose(found.mean, &laser);
            covariance = found.covariance;
        }
        self.add_and_link(scan, covariance, localization).map(Some)
    }

    /// `ProcessAgainstNodesNearBy` as slam_toolbox's localization calls it: the scan
    /// matched against the map's scan nearest its odometric pose, which becomes the running
    /// buffer and the last scan, its odometric pose set to its corrected one, added and
    /// linked, its loops closed, and the scan added to the localization buffer.
    pub fn process_against_nodes_near_by(
        &mut self,
        mut scan: LocalizedScan,
    ) -> Result<Processed, MatcherError> {
        let laser = self.laser;
        let closest = self.find_near_by_scan(scan.odometric_pose());
        if let Some(closest) = closest {
            self.running_scans.clear();
            self.add_running_scan(closest);
            self.last_scan = Some(closest);
        }
        let mut covariance = Matrix3::identity();
        if closest.is_some() {
            let found = self.match_running_buffer(&scan)?;
            scan.set_sensor_pose(found.mean, &laser);
            covariance = found.covariance;
        }
        scan.set_odometric_pose(scan.corrected_pose());
        self.add_and_link(scan, covariance, true)
    }

    /// The best pose of `scan` within `radius` (m) of its sensor pose and `angle_offset`
    /// (rad) of its heading, a whole turn at pi or more, among the map's scans that lie
    /// within the loop search's distance of that disc, as karto matches a loop
    /// (`TryCloseLoop`): a coarse match by a matcher of the loop search's cells whose search
    /// holds the disc, then the sequential matcher's match about the coarse pose, neither
    /// penalized.
    pub fn match_within(
        &mut self,
        scan: &LocalizedScan,
        radius: f64,
        angle_offset: f64,
    ) -> Result<Match, MatcherError> {
        let laser = self.laser;
        let center = scan.sensor_pose();
        let reach = (radius + self.parameters.loop_search_maximum_distance).powi(2);
        let base: Vec<&LocalizedScan> = self
            .scans
            .values()
            .filter(|candidate| candidate.sensor_pose().squared_distance(center) <= reach)
            .collect();
        let mut matcher = ScanMatcher::new(
            self.parameters.match_parameters,
            self.parameters.loop_search_space.holding_disc(radius)?,
            laser.range_threshold,
        )?;
        let coarse = matcher.match_scan_within(scan, &base, radius, angle_offset)?;
        let mut placed = scan.clone();
        placed.set_sensor_pose(coarse.mean, &laser);
        self.sequential_matcher
            .match_scan(&placed, &base, false, true)
    }

    /// The sequential match of `scan` against the running buffer, penalized and refined.
    fn match_running_buffer(&mut self, scan: &LocalizedScan) -> Result<Match, MatcherError> {
        let base: Vec<&LocalizedScan> = self
            .running_scans
            .iter()
            .filter_map(|id| self.scans.get(id))
            .collect();
        self.sequential_matcher.match_scan(scan, &base, true, true)
    }

    /// The end of `Process` and its siblings: the scan added, its vertex added and linked,
    /// the loops it closes closed, the scan made the last one, and, with `localization`,
    /// added to the localization buffer.
    fn add_and_link(
        &mut self,
        scan: LocalizedScan,
        covariance: Matrix3,
        localization: bool,
    ) -> Result<Processed, MatcherError> {
        let id = self.add_scan(scan);
        self.add_vertex(id);
        self.add_edges(id, covariance)?;
        self.add_running_scan(id);
        if self.parameters.do_loop_closing {
            self.try_close_loop(id)?;
        }
        self.last_scan = Some(id);
        if localization {
            self.add_scan_to_localization_buffer(id);
        }
        Ok(Processed { id, covariance })
    }

    /// `HasMovedEnough`: whether the scan lies long enough after the last processed
    /// scan, or its laser turned or moved far enough by odometry.
    fn has_moved_enough(&self, scan: &LocalizedScan, last: Option<&LocalizedScan>) -> bool {
        let Some(last) = last else {
            return true;
        };
        if scan.time - last.time >= MINIMUM_TIME_INTERVAL_S {
            return true;
        }
        let last_scanner_pose = sensor_at(last.odometric_pose());
        let scanner_pose = sensor_at(scan.odometric_pose());
        let delta_heading = normalize_angle(scanner_pose.yaw - last_scanner_pose.yaw);
        if delta_heading.abs() >= self.parameters.minimum_travel_heading {
            return true;
        }
        last_scanner_pose.squared_distance(scanner_pose)
            >= self.parameters.minimum_travel_distance.powi(2) - TOLERANCE
    }

    /// `MapperSensorManager::AddScan`: the scan under the next id.
    fn add_scan(&mut self, scan: LocalizedScan) -> ScanId {
        let id = ScanId(self.next_id);
        self.next_id += 1;
        self.scans.insert(id, scan);
        id
    }

    /// `ScanManager::AddRunningScan`: the scan at the end of the running buffer, which then
    /// drops its oldest scans while it holds more than its size or its ends lie farther
    /// apart than its distance.
    fn add_running_scan(&mut self, id: ScanId) {
        self.running_scans.push(id);
        let maximum_squared_distance = self.running_buffer_maximum_distance.powi(2) - TOLERANCE;
        loop {
            let ends = self
                .running_scans
                .first()
                .zip(self.running_scans.last())
                .and_then(|(front, back)| Some((self.scans.get(front)?, self.scans.get(back)?)));
            let Some((front, back)) = ends else {
                return;
            };
            let squared_distance = front.sensor_pose().squared_distance(back.sensor_pose());
            if self.running_scans.len() <= self.running_buffer_maximum_size as usize
                && squared_distance <= maximum_squared_distance
            {
                return;
            }
            self.running_scans.remove(0);
        }
    }

    /// `MapperGraph::AddVertex`: the scan's vertex, and its node at its corrected pose.
    fn add_vertex(&mut self, id: ScanId) {
        self.graph.add_vertex(id);
        if let Some(scan) = self.scans.get(&id) {
            self.optimizer.add_node(id, scan.corrected_pose());
        }
    }

    /// `MapperGraph::AddEdges`: the scan linked to the scan before it and to the running
    /// buffer at its pose, and to the near chains it matches, and its pose set to the mean
    /// of those matches weighted by their covariances.
    fn add_edges(&mut self, id: ScanId, covariance: Matrix3) -> Result<(), MatcherError> {
        let laser = self.laser;
        let Some(scan_pose) = self.scans.get(&id).map(LocalizedScan::sensor_pose) else {
            return Ok(());
        };
        if self.last_scan.is_some() {
            let Some(previous) = id
                .previous()
                .filter(|previous| self.scans.contains_key(previous))
            else {
                return Ok(());
            };
            self.link_scans(previous, id, scan_pose, covariance);
        }
        let mut means = Vec::new();
        let mut covariances = Vec::new();
        if self.last_scan.is_some() {
            means.push(scan_pose);
            covariances.push(covariance);
            let chain = self.running_scans.clone();
            self.link_chain_to_scan(&chain, id, scan_pose, covariance);
        }
        self.link_near_chains(id, &mut means, &mut covariances)?;
        if !means.is_empty() {
            let mean = compute_weighted_mean(&means, &covariances);
            if let Some(scan) = self.scans.get_mut(&id) {
                scan.set_sensor_pose(mean, &laser);
            }
        }
        Ok(())
    }

    /// `MapperGraph::LinkScans`: the edge from `from` to `to`, and, when it is new, its
    /// link at the corrected pose of `from` and the corrected pose of `to` at `mean`, and
    /// its constraint.
    fn link_scans(&mut self, from: ScanId, to: ScanId, mean: Pose2, covariance: Matrix3) {
        let Some(from_pose) = self.scans.get(&from).map(LocalizedScan::corrected_pose) else {
            return;
        };
        let link = || LinkInfo::new(from_pose, corrected_at(mean), covariance);
        let Some((edge, true)) = self.graph.add_edge(from, to, link) else {
            return;
        };
        let edge = *self.graph.edge(edge);
        self.optimizer.add_constraint(
            edge.source,
            edge.target,
            edge.link.pose_difference,
            edge.link.covariance,
        );
    }

    /// `MapperGraph::LinkNearChains`: the scan matched against each near chain long
    /// enough, and linked to each chain it matches well enough, whose match joins the
    /// means.
    fn link_near_chains(
        &mut self,
        id: ScanId,
        means: &mut Vec<Pose2>,
        covariances: &mut Vec<Matrix3>,
    ) -> Result<(), MatcherError> {
        for chain in self.find_near_chains(id) {
            if chain.len() < self.parameters.loop_match_minimum_chain_size as usize {
                continue;
            }
            let Some(scan) = self.scans.get(&id) else {
                return Ok(());
            };
            let base: Vec<&LocalizedScan> = chain
                .iter()
                .filter_map(|chained| self.scans.get(chained))
                .collect();
            let found = self
                .sequential_matcher
                .match_scan(scan, &base, false, true)?;
            if found.response > self.parameters.link_match_minimum_response_fine - TOLERANCE {
                means.push(found.mean);
                covariances.push(found.covariance);
                self.link_chain_to_scan(&chain, id, found.mean, found.covariance);
            }
        }
        Ok(())
    }

    /// `MapperGraph::LinkChainToScan`: the scan linked from the scan of `chain` nearest
    /// it, when that one lies within the link distance.
    fn link_chain_to_scan(
        &mut self,
        chain: &[ScanId],
        id: ScanId,
        mean: Pose2,
        covariance: Matrix3,
    ) {
        let use_barycenter = self.parameters.use_scan_barycenter;
        let Some(pose) = self
            .scans
            .get(&id)
            .map(|scan| scan.reference_pose(use_barycenter))
        else {
            return;
        };
        let Some(closest) = self.closest_scan_to_pose(chain, pose) else {
            return;
        };
        let closest_pose = self.scans[&closest].reference_pose(use_barycenter);
        let squared_distance = pose.squared_distance(closest_pose);
        if squared_distance < self.parameters.link_scan_maximum_distance.powi(2) + TOLERANCE {
            self.link_scans(closest, id, mean, covariance);
        }
    }

    /// `MapperGraph::GetClosestScanToPose`: the first scan of `scans` nearest `pose`.
    fn closest_scan_to_pose(&self, scans: &[ScanId], pose: Pose2) -> Option<ScanId> {
        let use_barycenter = self.parameters.use_scan_barycenter;
        scans
            .iter()
            .filter_map(|&id| {
                let reference = self.scans.get(&id)?.reference_pose(use_barycenter);
                Some((id, pose.squared_distance(reference)))
            })
            .fold(
                (None, f64::MAX),
                |(closest, best), (id, squared_distance)| {
                    if squared_distance < best {
                        (Some(id), squared_distance)
                    } else {
                        (closest, best)
                    }
                },
            )
            .0
    }

    /// `MapperGraph::FindNearLinkedScans`: the scans reached through the graph from the
    /// scan by scans within `maximum_distance` of it.
    fn find_near_linked_scans(&self, id: ScanId, maximum_distance: f64) -> Vec<ScanId> {
        let use_barycenter = self.parameters.use_scan_barycenter;
        let Some(center) = self
            .scans
            .get(&id)
            .map(|scan| scan.reference_pose(use_barycenter))
        else {
            return Vec::new();
        };
        let maximum_squared_distance = maximum_distance.powi(2) - TOLERANCE;
        self.graph.traverse(id, |vertex| {
            self.scans.get(&vertex).is_some_and(|scan| {
                scan.reference_pose(use_barycenter).squared_distance(center)
                    <= maximum_squared_distance
            })
        })
    }

    /// `MapperGraph::FindNearChains`: about each scan linked near the scan, the run of
    /// consecutive scans within the link distance of it, but for a run that holds the
    /// scan.
    fn find_near_chains(&self, id: ScanId) -> Vec<Vec<ScanId>> {
        let use_barycenter = self.parameters.use_scan_barycenter;
        let Some(scan_pose) = self
            .scans
            .get(&id)
            .map(|scan| scan.reference_pose(use_barycenter))
        else {
            return Vec::new();
        };
        let maximum_squared_distance =
            self.parameters.link_scan_maximum_distance.powi(2) + TOLERANCE;
        let near = |candidate: &LocalizedScan| {
            scan_pose.squared_distance(candidate.reference_pose(use_barycenter))
                < maximum_squared_distance
        };
        let mut processed: Vec<ScanId> = Vec::new();
        let mut chains = Vec::new();
        let end = self.scans.len() as u32;
        for near_scan in self.find_near_linked_scans(id, self.parameters.link_scan_maximum_distance)
        {
            if near_scan == id || processed.contains(&near_scan) {
                continue;
            }
            processed.push(near_scan);
            let mut is_valid_chain = true;
            let mut chain = VecDeque::new();
            for candidate in (0..near_scan.0).rev().map(ScanId) {
                if candidate == id {
                    is_valid_chain = false;
                }
                let Some(candidate_scan) = self.scans.get(&candidate) else {
                    continue;
                };
                if !near(candidate_scan) {
                    break;
                }
                chain.push_front(candidate);
                processed.push(candidate);
            }
            chain.push_back(near_scan);
            for candidate in (near_scan.0 + 1..end).map(ScanId) {
                if candidate == id {
                    is_valid_chain = false;
                }
                let Some(candidate_scan) = self.scans.get(&candidate) else {
                    continue;
                };
                if !near(candidate_scan) {
                    break;
                }
                chain.push_back(candidate);
                processed.push(candidate);
            }
            if is_valid_chain {
                chains.push(chain.into_iter().collect());
            }
        }
        chains
    }

    /// `MapperGraph::FindNearByScan`: the first vertex, in id order, whose scan's
    /// corrected position lies nearest `pose`.
    fn find_near_by_scan(&self, pose: Pose2) -> Option<ScanId> {
        self.graph
            .vertex_ids()
            .filter_map(|id| {
                let scan = self.scans.get(&id)?;
                Some((id, pose.squared_distance(scan.corrected_pose())))
            })
            .fold(
                (None, f64::INFINITY),
                |(nearest, best), (id, squared_distance)| {
                    if squared_distance < best {
                        (Some(id), squared_distance)
                    } else {
                        (nearest, best)
                    }
                },
            )
            .0
    }

    /// `MapperGraph::TryCloseLoop`: each candidate chain that the scan matches coarsely
    /// and then finely well enough, linked to the scan, and the graph optimized.
    fn try_close_loop(&mut self, id: ScanId) -> Result<bool, MatcherError> {
        let laser = self.laser;
        let mut loop_closed = false;
        let mut scan_index = 0;
        let mut candidate = self.find_possible_loop_closure(id, &mut scan_index);
        while !candidate.is_empty() {
            let Some(scan) = self.scans.get(&id) else {
                break;
            };
            let base: Vec<&LocalizedScan> = candidate
                .iter()
                .filter_map(|chained| self.scans.get(chained))
                .collect();
            let coarse = self.loop_matcher.match_scan(scan, &base, false, false)?;
            let maximum_variance = self.parameters.loop_match_maximum_variance_coarse;
            if coarse.response > self.parameters.loop_match_minimum_response_coarse
                && coarse.covariance.0[0][0] < maximum_variance
                && coarse.covariance.0[1][1] < maximum_variance
            {
                let mut moved = scan.clone();
                moved.set_sensor_pose(coarse.mean, &laser);
                let fine = self
                    .sequential_matcher
                    .match_scan(&moved, &base, false, true)?;
                if fine.response >= self.parameters.loop_match_minimum_response_fine {
                    if let Some(scan) = self.scans.get_mut(&id) {
                        scan.set_sensor_pose(fine.mean, &laser);
                    }
                    self.link_chain_to_scan(&candidate, id, fine.mean, fine.covariance);
                    self.correct_poses();
                    loop_closed = true;
                }
            }
            candidate = self.find_possible_loop_closure(id, &mut scan_index);
        }
        Ok(loop_closed)
    }

    /// `MapperGraph::FindPossibleLoopClosure`: from `start`, the next run of scans within
    /// the loop search distance of the scan, long enough, with no scan linked near it.
    /// `start` stops at the scan past the run.
    fn find_possible_loop_closure(&self, id: ScanId, start: &mut u32) -> Vec<ScanId> {
        let use_barycenter = self.parameters.use_scan_barycenter;
        let Some(pose) = self
            .scans
            .get(&id)
            .map(|scan| scan.reference_pose(use_barycenter))
        else {
            return Vec::new();
        };
        let near_linked =
            self.find_near_linked_scans(id, self.parameters.loop_search_maximum_distance);
        let maximum_squared_distance =
            self.parameters.loop_search_maximum_distance.powi(2) + TOLERANCE;
        let mut chain = Vec::new();
        let scans = self.scans.len() as u32;
        while *start < scans {
            let candidate = ScanId(*start);
            if let Some(candidate_scan) = self.scans.get(&candidate) {
                let squared_distance = candidate_scan
                    .reference_pose(use_barycenter)
                    .squared_distance(pose);
                if squared_distance < maximum_squared_distance {
                    if near_linked.contains(&candidate) {
                        chain.clear();
                    } else {
                        chain.push(candidate);
                    }
                } else if chain.len() >= self.parameters.loop_match_minimum_chain_size as usize {
                    return chain;
                } else {
                    chain.clear();
                }
            }
            *start += 1;
        }
        chain
    }

    /// `MapperGraph::CorrectPoses`: the graph optimized, and each scan moved to its node.
    fn correct_poses(&mut self) {
        let laser = self.laser;
        self.optimizer.compute();
        for (id, pose) in self.optimizer.take_corrections() {
            if let Some(scan) = self.scans.get_mut(&id) {
                scan.set_corrected_pose(pose, &laser);
            }
        }
    }

    /// `AddScanToLocalizationBuffer`: the scan at the end of the buffer, which then drops
    /// its oldest scan, from the graph, the optimizer and the scans, when it holds more
    /// than the buffer's size.
    fn add_scan_to_localization_buffer(&mut self, id: ScanId) {
        self.localization_buffer.push_back(id);
        if self.localization_buffer.len() <= self.parameters.scan_buffer_size as usize {
            return;
        }
        let Some(oldest) = self.localization_buffer.pop_front() else {
            return;
        };
        for (source, target) in self.graph.remove_vertex(oldest) {
            self.optimizer.remove_constraint(source, target);
        }
        self.optimizer.remove_node(oldest);
        self.scans.remove(&oldest);
    }
}

/// `MapperGraph::ComputeWeightedMean`: the means weighted by their inverse covariances
/// over the inverse of their sum, with the circular mean of their headings.
fn compute_weighted_mean(means: &[Pose2], covariances: &[Matrix3]) -> Pose2 {
    let inverses: Vec<Matrix3> = covariances
        .iter()
        .map(|covariance| covariance.inverse())
        .collect();
    let sum_of_inverses = inverses
        .iter()
        .fold(Matrix3::zero(), |sum, inverse| sum.plus(*inverse));
    let inverse_of_sum = sum_of_inverses.inverse();
    let (position, theta_x, theta_y) = means.iter().zip(&inverses).fold(
        (Vector2::default(), 0.0, 0.0),
        |(position, theta_x, theta_y), (mean, inverse)| {
            let weighted = inverse_of_sum.times(*inverse).times_pose(*mean);
            (
                position.plus(weighted.position()),
                theta_x + mean.yaw.cos(),
                theta_y + mean.yaw.sin(),
            )
        },
    );
    let count = means.len() as f64;
    Pose2::at(position, (theta_y / count).atan2(theta_x / count))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::placed_scan::{Beam, ScanGeometry};

    /// A scan of a room 8 m by 6 m about the origin, all around the base at `pose`.
    fn room_scan(laser: &Laser, pose: Pose2) -> LocalizedScan {
        let beams = (0..laser.number_of_readings)
            .map(|beam| {
                let bearing = laser.minimum_angle + beam as f64 * laser.angular_resolution;
                let angle = pose.yaw + bearing;
                let (dx, dy) = (angle.cos(), angle.sin());
                let to_wall = |distance: f64, along: f64| {
                    if along == 0.0 {
                        f64::INFINITY
                    } else {
                        distance / along.abs()
                    }
                };
                let x_wall = to_wall(if dx > 0.0 { 4.0 - pose.x } else { 4.0 + pose.x }, dx);
                let y_wall = to_wall(if dy > 0.0 { 3.0 - pose.y } else { 3.0 + pose.y }, dy);
                Beam {
                    reading: x_wall.min(y_wall),
                    origin: Vector2::new(0.0, 0.0),
                    bearing,
                }
            })
            .collect();
        LocalizedScan::new(laser, beams, 0.0, pose)
    }

    #[test]
    fn the_localization_buffer_drops_its_oldest_scan_from_the_scans_the_graph_and_the_optimizer() {
        let parameters = crate::mapping::tests::parameters();
        let karto = KartoParameters {
            scan_buffer_size: 3,
            ..parameters.karto().expect("valid parameters")
        };
        let geometry = ScanGeometry {
            angle_min: -std::f32::consts::PI,
            angle_increment: 0.0175,
            rays: 359,
            range_min: 0.1,
            range_max: 12.0,
        };
        let laser = Laser::of(&geometry, 0.1, 12.0);
        let mut mapper = Mapper::new(karto, laser).expect("a mapper");
        let ids: Vec<ScanId> = (0..5)
            .map(|step| {
                let pose = Pose2::new(-1.2 + 0.6 * f64::from(step), 0.0, 0.0);
                mapper
                    .process(room_scan(&laser, pose), true)
                    .expect("a match")
                    .expect("the base moved enough")
                    .id
            })
            .collect();
        let (dropped, kept) = ids.split_at(2);
        for id in dropped {
            assert!(!mapper.scans().contains_key(id), "scan {id:?} stays");
            assert!(mapper.graph.vertex_ids().all(|vertex| vertex != *id));
            assert!(!mapper.optimizer.holds(*id));
        }
        for id in kept {
            assert!(mapper.scans().contains_key(id), "scan {id:?} is gone");
            assert!(mapper.graph.vertex_ids().any(|vertex| vertex == *id));
            assert!(mapper.optimizer.holds(*id));
        }
    }

    #[test]
    fn a_weighted_mean_leans_to_the_tighter_covariance() {
        let tight = Matrix3([[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]]);
        let loose = Matrix3([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        let mean = compute_weighted_mean(
            &[Pose2::new(0.0, 0.0, 0.2), Pose2::new(1.0, 1.0, 0.4)],
            &[tight, loose],
        );
        assert!(mean.x < 0.02 && mean.y < 0.02, "{mean:?}");
        assert!((mean.yaw - 0.3).abs() < 1e-12);
    }
}
