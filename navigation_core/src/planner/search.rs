//! Smac 2D's A* on a cost grid, ported from nav2_smac_planner 1.5.2:
//! `AStarAlgorithm<Node2D>::createPath` (include/nav2_smac_planner/
//! a_star_impl.hpp), `Node2D`'s neighbors, traversal cost, heuristic and
//! backtrace (src/node_2d.cpp, include/nav2_smac_planner/node_2d.hpp), and
//! the check `GridCollisionChecker::inCollision` makes of a cell for a base
//! taken as its inscribed circle (src/collision_checker.cpp). Costs and
//! cell coordinates are single precision, as Smac holds them, and each
//! operation runs in Smac's order and precision, so the search expands the
//! cells Smac expands, in its order.

use std::collections::HashMap;

use libstdcxx_heap::OpenSet;

use crate::grid::{Cell, CostGrid, GridPoint, INSCRIBED, MAX_NON_OBSTACLE, UNKNOWN};

/// The squared length of a step, in cells, above which `getTraversalCost`
/// takes the step for a diagonal one (src/node_2d.cpp:75): a side step's
/// squared length is 1 and a diagonal step's 2.
const DIAGONAL_STEP_SQUARED_LENGTH: f64 = 1.05;

/// What the search reads beyond the grid and its ends.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SearchLimits {
    /// `cost_travel_multiplier` in single precision.
    pub cost_travel_multiplier: f32,
    pub allow_unknown: bool,
    pub max_iterations: u32,
}

/// Whether a cell of `cost` is in collision for the base's center:
/// `GridCollisionChecker::inCollision` for a base taken as its inscribed
/// circle. An unknown cell is passable only when unknown cells are.
pub(crate) fn in_collision(cost: u8, allow_unknown: bool) -> bool {
    if cost == UNKNOWN && allow_unknown {
        return false;
    }
    cost >= INSCRIBED
}

/// One cell of the search, as `Node2D` holds it.
#[derive(Debug, Clone, Copy)]
struct Node {
    /// The cell's cost once checked, NaN before.
    cell_cost: f32,
    accumulated_cost: f32,
    parent: Option<u64>,
    visited: bool,
    in_collision: bool,
}

impl Node {
    fn new() -> Self {
        Self {
            cell_cost: f32::NAN,
            accumulated_cost: f32::MAX,
            parent: None,
            visited: false,
            in_collision: false,
        }
    }

    /// `Node2D::isNodeValid`: checks the cell once and keeps the answer.
    fn is_valid(&mut self, cost: u8, allow_unknown: bool) -> bool {
        if !self.cell_cost.is_nan() {
            return !self.in_collision;
        }
        self.in_collision = in_collision(cost, allow_unknown);
        self.cell_cost = f32::from(cost);
        !self.in_collision
    }
}

/// The search over one grid towards one goal.
struct Search<'a> {
    costs: &'a [u8],
    width: u64,
    max_index: u64,
    /// `neighbors_grid_offsets`, in Node2D's order.
    offsets: [i64; 8],
    goal_index: u64,
    /// The goal in continuous cell coordinates, as the heuristic reads it.
    goal: [f32; 2],
    limits: SearchLimits,
    nodes: HashMap<u64, Node>,
}

impl Search<'_> {
    /// `Node2D::getCoords`: a cell index as its column and row, each in
    /// single precision.
    fn coords(&self, index: u64) -> [f32; 2] {
        [(index % self.width) as f32, (index / self.width) as f32]
    }

    fn node(&mut self, index: u64) -> &mut Node {
        self.nodes.entry(index).or_insert_with(Node::new)
    }

    /// `Node2D::getTraversalCost`: one step from `from` to `to`, 1 or the
    /// square root of 2 by the step's length, grown by the multiplier times
    /// `to`'s cost over [`MAX_NON_OBSTACLE`], as src/node_2d.cpp:67 divides
    /// it.
    fn traversal_cost(&self, from: u64, to: u64, to_cost: f32) -> f32 {
        let normalized_cost = (f64::from(to_cost) / f64::from(MAX_NON_OBSTACLE)) as f32;
        let a = self.coords(to);
        let b = self.coords(from);
        let dx = a[0] - b[0];
        let dy = a[1] - b[1];
        let grown = 1.0 + f64::from(self.limits.cost_travel_multiplier * normalized_cost);
        if f64::from(dx * dx + dy * dy) > DIAGONAL_STEP_SQUARED_LENGTH {
            let sqrt_2 = std::f64::consts::SQRT_2 as f32;
            return (f64::from(sqrt_2) * grown) as f32;
        }
        grown as f32
    }

    /// `Node2D::getHeuristicCost`: the straight-line distance in cells from
    /// the cell's corner to the goal.
    fn heuristic_cost(&self, index: u64) -> f32 {
        let node = self.coords(index);
        let dx = self.goal[0] - node[0];
        let dy = self.goal[1] - node[1];
        (dx * dx + dy * dy).sqrt()
    }

    /// `Node2D::getNeighbors`: the valid cells around `index` that the
    /// search has not visited, in the order of the offsets, a step whose cell
    /// lies more than one cell away on either axis left out (node_2d.cpp:139-143).
    /// On a grid three or more cells wide that leaves out each step that
    /// wraps across the grid's edge; on one of one or two cells a wrapping
    /// step can land within one cell and is kept, and two offsets can name
    /// one cell, as in Nav2.
    fn neighbors(&mut self, index: u64) -> Vec<u64> {
        let parent = self.coords(index);
        let mut neighbors = Vec::with_capacity(self.offsets.len());
        for offset in self.offsets {
            let child_index = index.wrapping_add(offset as u64);
            let child = self.coords(child_index);
            if (parent[0] - child[0]).abs() > 1.0 || (parent[1] - child[1]).abs() > 1.0 {
                continue;
            }
            if child_index >= self.max_index {
                continue;
            }
            let cost = self.costs[child_index as usize];
            let allow_unknown = self.limits.allow_unknown;
            let node = self.node(child_index);
            if node.is_valid(cost, allow_unknown) && !node.visited {
                neighbors.push(child_index);
            }
        }
        neighbors
    }

    /// `Node2D::backtracePath`: the cells from `index` back to the start,
    /// none when `index` has no parent.
    fn backtrace(&self, index: u64) -> Option<Vec<u64>> {
        self.nodes[&index].parent?;
        let mut path = vec![index];
        let mut current = index;
        while let Some(parent) = self.nodes[&current].parent {
            path.push(parent);
            current = parent;
        }
        Some(path)
    }

    /// `createPath` at a tolerance of 0: the cells from the goal back to the
    /// start, none when the search ends without reaching the goal.
    fn run(&mut self, start_index: u64) -> Option<Vec<u64>> {
        let mut queue = OpenSet::new();
        queue.push(0.0, start_index);
        self.node(start_index).accumulated_cost = 0.0;
        let mut iterations = 0;
        while iterations < self.limits.max_iterations {
            let Some((_, current)) = queue.pop() else {
                break;
            };
            if self.node(current).visited {
                continue;
            }
            iterations += 1;
            self.node(current).visited = true;
            if current == self.goal_index {
                return self.backtrace(current);
            }
            for neighbor in self.neighbors(current) {
                let to_cost = self.nodes[&neighbor].cell_cost;
                let g_cost = self.nodes[&current].accumulated_cost
                    + self.traversal_cost(current, neighbor, to_cost);
                if g_cost < self.nodes[&neighbor].accumulated_cost {
                    let node = self.node(neighbor);
                    node.accumulated_cost = g_cost;
                    node.parent = Some(current);
                    queue.push(g_cost + self.heuristic_cost(neighbor), neighbor);
                }
            }
        }
        None
    }
}

/// The cells of Smac 2D's A* path from `start` to `goal`, start first, ending
/// at the goal's cell; none when the search ends without reaching it. `start`
/// and `goal` lie in different cells.
pub(crate) fn a_star(
    grid: &CostGrid,
    start: Cell,
    goal: GridPoint,
    limits: SearchLimits,
) -> Option<Vec<Cell>> {
    let geometry = grid.geometry();
    let width = u64::from(geometry.size_x());
    let x_size = i64::from(geometry.size_x());
    let mut search = Search {
        costs: grid.costs(),
        width,
        max_index: width * u64::from(geometry.size_y()),
        offsets: [
            -1,
            1,
            -x_size,
            x_size,
            -x_size - 1,
            -x_size + 1,
            x_size - 1,
            x_size + 1,
        ],
        goal_index: geometry.index(goal.cell()) as u64,
        goal: goal.map(),
        limits,
        nodes: HashMap::new(),
    };
    let path = search.run(geometry.index(start) as u64)?;
    Some(
        path.into_iter()
            .rev()
            .map(|index| geometry.cell_at(index as usize))
            .collect(),
    )
}
