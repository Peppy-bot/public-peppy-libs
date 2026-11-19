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
// This file translates into Rust Nav2 1.5.2's kd-tree of AMCL, nav2_amcl/src/pf/pf_kdtree.c
// (Copyright (C) 2000 Brian Gerkey & Kasper Stoy).
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

//! The histogram of AMCL's adaptive sampling, nav2_amcl's kd-tree (src/pf/pf_kdtree.c:
//! `pf_kdtree_alloc`, `pf_kdtree_insert`, `pf_kdtree_insert_node`, `pf_kdtree_find_node`,
//! `pf_kdtree_cluster`, `pf_kdtree_cluster_node`, `pf_kdtree_get_cluster`). A pose falls in
//! the bin of 0.5 m by 0.5 m by 10 degrees that holds it, a leaf holds the weight of a bin,
//! and a cluster is a set of bins joined through their 26 neighbors. The nodes lie in the
//! order the tree makes them, as the tree's array holds them, since clustering labels the
//! leaves in that order.

use crate::geometry::{Pose2, truncate_to_i32};

/// `size`: the sides of a bin (m, m, rad).
const BIN_SIZE: [f64; 3] = [0.50, 0.50, 10.0 * std::f64::consts::PI / 180.0];

/// The bin of a pose: each coordinate over the bin's side, floored and converted to an
/// int as the amd64 build converts it.
type Key = [i32; 3];

fn key_of(pose: Pose2) -> Key {
    let coordinates = [pose.x, pose.y, pose.yaw];
    [0, 1, 2].map(|i| truncate_to_i32((coordinates[i] / BIN_SIZE[i]).floor()))
}

/// The split of a node that is no leaf: the dimension and the value its children part
/// at, and the children below and at or above it.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Split {
    dimension: usize,
    value: f64,
    children: [usize; 2],
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Node {
    key: Key,
    value: f64,
    /// None for a leaf.
    split: Option<Split>,
    /// The cluster of a leaf, once labeled.
    cluster: Option<usize>,
}

/// `pf_kdtree_t`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct KdTree {
    nodes: Vec<Node>,
    root: Option<usize>,
    leaf_count: usize,
    cluster_count: usize,
}

impl KdTree {
    /// The empty tree, with room for the nodes of `samples` poses.
    pub fn new(samples: usize) -> Self {
        Self {
            nodes: Vec::with_capacity(2 * samples),
            root: None,
            leaf_count: 0,
            cluster_count: 0,
        }
    }

    /// The bins that hold a pose.
    pub fn leaf_count(&self) -> usize {
        self.leaf_count
    }

    /// The clusters that the last [`KdTree::cluster`] labeled.
    pub fn cluster_count(&self) -> usize {
        self.cluster_count
    }

    /// `pf_kdtree_insert`: `value` added to the bin of `pose`.
    pub fn insert(&mut self, pose: Pose2, value: f64) {
        let root = self.insert_node(self.root, key_of(pose), value);
        self.root = Some(root);
    }

    fn new_leaf(&mut self, key: Key, value: f64) -> usize {
        self.nodes.push(Node {
            key,
            value,
            split: None,
            cluster: None,
        });
        self.leaf_count += 1;
        self.nodes.len() - 1
    }

    /// `pf_kdtree_insert_node`: the node that `node` becomes with `value` added at `key`,
    /// a new leaf for none. A leaf of another key splits in the dimension where the keys
    /// lie farthest apart, the first of equals, at their midpoint.
    fn insert_node(&mut self, node: Option<usize>, key: Key, value: f64) -> usize {
        let Some(index) = node else {
            return self.new_leaf(key, value);
        };
        let Node {
            key: node_key,
            value: node_value,
            split,
            ..
        } = self.nodes[index];
        if let Some(split) = split {
            let child = split.children[usize::from(f64::from(key[split.dimension]) >= split.value)];
            self.insert_node(Some(child), key, value);
            return index;
        }
        if key == node_key {
            self.nodes[index].value += value;
            return index;
        }
        let distance = |i: usize| (i64::from(key[i]) - i64::from(node_key[i])).abs();
        let dimension = (1..3).fold(0, |widest, i| {
            if distance(i) > distance(widest) {
                i
            } else {
                widest
            }
        });
        let middle = (i64::from(key[dimension]) + i64::from(node_key[dimension])) as f64 / 2.0;
        let children = if f64::from(key[dimension]) < middle {
            let below = self.new_leaf(key, value);
            [below, self.new_leaf(node_key, node_value)]
        } else {
            let below = self.new_leaf(node_key, node_value);
            [below, self.new_leaf(key, value)]
        };
        self.nodes[index].split = Some(Split {
            dimension,
            value: middle,
            children,
        });
        self.leaf_count -= 1;
        index
    }

    /// `pf_kdtree_find_node` from the root: the leaf of `key`, if any.
    fn find(&self, key: Key) -> Option<usize> {
        let mut index = self.root?;
        while let Some(split) = self.nodes[index].split {
            index = split.children[usize::from(f64::from(key[split.dimension]) >= split.value)];
        }
        (self.nodes[index].key == key).then_some(index)
    }

    /// `pf_kdtree_cluster`: each leaf labeled with its cluster, the clusters counted from
    /// 0 in the order that the leaves come off the back of their list in the order of the
    /// nodes, each the leaves that its first one reaches through neighboring bins.
    pub fn cluster(&mut self) {
        let leaves: Vec<usize> = (0..self.nodes.len())
            .filter(|&index| self.nodes[index].split.is_none())
            .collect();
        for &leaf in &leaves {
            self.nodes[leaf].cluster = None;
        }
        self.cluster_count = 0;
        for &leaf in leaves.iter().rev() {
            if self.nodes[leaf].cluster.is_some() {
                continue;
            }
            self.label_cluster(leaf, self.cluster_count);
            self.cluster_count += 1;
        }
    }

    /// `pf_kdtree_cluster_node`: `cluster` given to `first` and to every unlabeled leaf
    /// that it reaches through the 26 neighbors of each bin.
    fn label_cluster(&mut self, first: usize, cluster: usize) {
        self.nodes[first].cluster = Some(cluster);
        let mut reached = vec![first];
        while let Some(index) = reached.pop() {
            let key = self.nodes[index].key;
            for i in 0..27 {
                let offset = [i / 9, (i % 9) / 3, (i % 9) % 3];
                let neighbor_key = [0, 1, 2].map(|d| key[d].wrapping_add(offset[d] - 1));
                let Some(neighbor) = self.find(neighbor_key) else {
                    continue;
                };
                if self.nodes[neighbor].cluster.is_none() {
                    self.nodes[neighbor].cluster = Some(cluster);
                    reached.push(neighbor);
                }
            }
        }
    }

    /// `pf_kdtree_get_cluster`: the cluster of the bin of `pose`, which holds a pose.
    pub fn cluster_of(&self, pose: Pose2) -> Option<usize> {
        self.find(key_of(pose))
            .and_then(|leaf| self.nodes[leaf].cluster)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree_of(poses: &[Pose2]) -> KdTree {
        let mut tree = KdTree::new(poses.len());
        for &pose in poses {
            tree.insert(pose, 1.0);
        }
        tree.cluster();
        tree
    }

    #[test]
    fn poses_of_one_bin_make_one_leaf() {
        let tree = tree_of(&[Pose2::new(0.1, 0.1, 0.0), Pose2::new(0.4, 0.2, 0.1)]);
        assert_eq!(tree.leaf_count(), 1);
        assert_eq!(tree.nodes[0].value, 2.0);
    }

    #[test]
    fn bins_that_touch_at_a_corner_make_one_cluster_and_a_gap_makes_two() {
        let touching = tree_of(&[Pose2::new(0.1, 0.1, 0.0), Pose2::new(0.6, 0.6, 0.2)]);
        assert_eq!((touching.leaf_count(), touching.cluster_count()), (2, 1));
        let apart = tree_of(&[Pose2::new(0.1, 0.1, 0.0), Pose2::new(1.1, 0.1, 0.0)]);
        assert_eq!((apart.leaf_count(), apart.cluster_count()), (2, 2));
        let far = Pose2::new(1.1, 0.1, 0.0);
        let near = Pose2::new(0.1, 0.1, 0.0);
        assert_ne!(apart.cluster_of(far), apart.cluster_of(near));
        assert_eq!(apart.cluster_of(Pose2::new(5.0, 5.0, 0.0)), None);
    }

    #[test]
    fn clusters_count_from_the_last_leaf_the_tree_made() {
        let poses = [
            Pose2::new(0.1, 0.1, 0.0),
            Pose2::new(3.1, 0.1, 0.0),
            Pose2::new(6.1, 0.1, 0.0),
        ];
        let tree = tree_of(&poses);
        let last_leaf = (0..tree.nodes.len())
            .rev()
            .find(|&index| tree.nodes[index].split.is_none())
            .expect("a leaf");
        assert_eq!(tree.nodes[last_leaf].cluster, Some(0));
        assert_eq!(tree.cluster_count(), 3);
    }
}
