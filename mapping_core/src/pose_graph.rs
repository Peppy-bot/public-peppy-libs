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
// This file translates into Rust karto's graph of scans as slam_toolbox 2.10.0 carries it:
// lib/karto_sdk/include/karto_sdk/Mapper.h and lib/karto_sdk/src/Mapper.cpp (Copyright 2010 SRI
// International).
//
// This file conveys karto's code under the GNU General Public License, version 3 or later.
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

//! karto's graph of scans, as slam_toolbox 2.10.0 carries it (lib/karto_sdk: Mapper.h's
//! `LinkInfo`, `Vertex`, `Edge` and `Graph`; Mapper.cpp's `BreadthFirstTraversal`): a
//! vertex for each scan, by its id, and an edge for each link from one scan to another,
//! with the pose of the second in the frame of the first and its covariance. A vertex
//! lists its edges in the order they were made, which orders its neighbors.

use crate::geometry::{Matrix3, Pose2, Transform};
use crate::localized_scan::ScanId;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

/// `LinkInfo`: the poses of the two scans of a link when it was made, the pose of the
/// second in the frame of the first, and the covariance of the link turned into that
/// frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LinkInfo {
    pub pose1: Pose2,
    pub pose2: Pose2,
    pub pose_difference: Pose2,
    pub covariance: Matrix3,
}

impl LinkInfo {
    /// `LinkInfo::Update`.
    pub fn new(pose1: Pose2, pose2: Pose2, covariance: Matrix3) -> Self {
        let rotation = Matrix3::turn(-pose1.yaw);
        Self {
            pose1,
            pose2,
            pose_difference: Transform::between(pose1, Pose2::default()).transform_pose(pose2),
            covariance: rotation.times(covariance).times(rotation.transpose()),
        }
    }
}

/// `Edge`: a link from `source` to `target`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Edge {
    pub source: ScanId,
    pub target: ScanId,
    pub link: LinkInfo,
}

/// The place of an edge in the store of every edge the graph made.
pub(crate) type EdgeId = usize;

/// `Graph<LocalizedRangeScan>`: the vertices by scan id, each with its edges in order, and
/// the graph's edges in order.
#[derive(Debug, Clone, PartialEq, Default)]
pub(crate) struct Graph {
    vertices: BTreeMap<ScanId, Vec<EdgeId>>,
    edges: Vec<EdgeId>,
    store: Vec<Edge>,
}

impl Graph {
    /// `Graph::AddVertex`.
    pub fn add_vertex(&mut self, id: ScanId) {
        self.vertices.entry(id).or_default();
    }

    /// The ids of the vertices, in order.
    pub fn vertex_ids(&self) -> impl Iterator<Item = ScanId> + '_ {
        self.vertices.keys().copied()
    }

    /// The graph's edges, in the order in which it holds them.
    pub fn edges(&self) -> impl Iterator<Item = &Edge> + '_ {
        self.edges.iter().map(|&id| &self.store[id])
    }

    /// `MapperGraph::AddEdge`: the edge from `source` to `target` that `source` already
    /// holds, or a new one, with whether it is new. None when a vertex is missing.
    pub fn add_edge(
        &mut self,
        source: ScanId,
        target: ScanId,
        link: impl FnOnce() -> LinkInfo,
    ) -> Option<(EdgeId, bool)> {
        let (Some(source_edges), true) = (
            self.vertices.get(&source),
            self.vertices.contains_key(&target),
        ) else {
            return None;
        };
        if let Some(&existing) = source_edges
            .iter()
            .find(|&&edge| self.store[edge].target == target)
        {
            return Some((existing, false));
        }
        let id = self.store.len();
        self.store.push(Edge {
            source,
            target,
            link: link(),
        });
        self.vertices.entry(source).or_default().push(id);
        self.vertices.entry(target).or_default().push(id);
        self.edges.push(id);
        Some((id, true))
    }

    pub fn edge(&self, id: EdgeId) -> &Edge {
        &self.store[id]
    }

    /// `Vertex::GetAdjacentVertices`: the other end of each edge of `id`, in the order of
    /// its edges.
    pub fn adjacent_vertices(&self, id: ScanId) -> Vec<ScanId> {
        self.vertices
            .get(&id)
            .into_iter()
            .flatten()
            .flat_map(|&edge| {
                let Edge { source, target, .. } = self.store[edge];
                [
                    (source != id).then_some(source),
                    (target != id).then_some(target),
                ]
            })
            .flatten()
            .collect()
    }

    /// `BreadthFirstTraversal::TraverseForVertices`: from `start`, the vertices that
    /// `visit` accepts, each reached through accepted vertices, in the order of the
    /// traversal.
    pub fn traverse(&self, start: ScanId, mut visit: impl FnMut(ScanId) -> bool) -> Vec<ScanId> {
        let mut to_visit = VecDeque::from([start]);
        let mut seen = BTreeSet::from([start]);
        let mut valid = Vec::new();
        while let Some(next) = to_visit.pop_front() {
            if !self.vertices.contains_key(&next) || !visit(next) {
                continue;
            }
            valid.push(next);
            for adjacent in self.adjacent_vertices(next) {
                if seen.insert(adjacent) {
                    to_visit.push_back(adjacent);
                }
            }
        }
        valid
    }

    /// `Mapper::RemoveNodeFromGraph`, steps 1 and 3: each edge between `id` and an
    /// adjacent vertex removed from that vertex and from the graph, by karto's index
    /// arithmetic, and the vertex removed. The removed edges, each with its source and
    /// its target, in the order of removal.
    pub fn remove_vertex(&mut self, id: ScanId) -> Vec<(ScanId, ScanId)> {
        let mut removed = Vec::new();
        for adjacent in self.adjacent_vertices(id) {
            let edges = self.vertices.get(&adjacent).cloned().unwrap_or_default();
            for (index, &edge) in edges.iter().enumerate() {
                let Edge { source, target, .. } = self.store[edge];
                if target != id && source != id {
                    continue;
                }
                if let Some(live) = self.vertices.get_mut(&adjacent)
                    && index < live.len()
                {
                    live.remove(index);
                }
                removed.push((source, target));
                if let Some(position) = self.edges.iter().position(|&held| held == edge) {
                    self.edges.remove(position);
                }
            }
        }
        self.vertices.remove(&id);
        removed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(index: u32) -> ScanId {
        ScanId(index)
    }

    fn link() -> LinkInfo {
        LinkInfo::new(
            Pose2::default(),
            Pose2::new(1.0, 0.0, 0.0),
            Matrix3::identity(),
        )
    }

    #[test]
    fn a_link_holds_the_second_pose_in_the_frame_of_the_first() {
        let link = LinkInfo::new(
            Pose2::new(1.0, 1.0, std::f64::consts::FRAC_PI_2),
            Pose2::new(1.0, 3.0, 0.0),
            Matrix3::identity(),
        );
        assert!((link.pose_difference.x - 2.0).abs() < 1e-12);
        assert!(link.pose_difference.y.abs() < 1e-12);
        assert!((link.pose_difference.yaw + std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn an_edge_is_made_once_from_its_source_and_a_traversal_follows_the_edges() {
        let mut graph = Graph::default();
        for index in 0..4 {
            graph.add_vertex(id(index));
        }
        assert_eq!(graph.add_edge(id(0), id(1), link), Some((0, true)));
        assert_eq!(graph.add_edge(id(0), id(1), link), Some((0, false)));
        assert_eq!(graph.add_edge(id(1), id(2), link), Some((1, true)));
        assert_eq!(graph.add_edge(id(0), id(9), link), None);
        assert_eq!(graph.adjacent_vertices(id(1)), [id(0), id(2)]);
        assert_eq!(graph.traverse(id(0), |_| true), [id(0), id(1), id(2)]);
        assert_eq!(graph.traverse(id(0), |vertex| vertex != id(1)), [id(0)]);
        assert_eq!(graph.remove_vertex(id(2)), [(id(1), id(2))]);
        assert_eq!(graph.adjacent_vertices(id(1)), [id(0)]);
        assert_eq!(graph.edges().count(), 1);
    }
}
