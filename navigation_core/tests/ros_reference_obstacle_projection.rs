//! The live obstacles against Nav2's obstacle layer (nav2_costmap_2d 1.5.2): after one depth
//! frame's points go into an occupancy map and the map's columns under the padded footprint
//! are cleared, `LiveObstacles::project` marks lethal the cells over the occupied voxels
//! whose centers lie in the height band, and Nav2's obstacle layer marks the cells of the
//! same points in the band, with clearing off, then clears the cells under the same
//! footprint, padded as Costmap2DROS pads it, at the same base pose. The map keeps a hit in
//! its own voxel, so with the band's ends on voxel boundaries, voxels of the grid's cells,
//! and every point clear of a cell's edge and of the band's ends, the lethal cells are
//! Nav2's, and the clearing's cells are, by its port of `convexFillCells`. A point in the
//! band inside the unpadded outline at its cloud's base pose is the robot's own body, which
//! marks nothing here, where Nav2 marks its cell and its clearing frees the cells under the
//! padded footprint, so the lethal cells are Nav2's with each such point in a cell Nav2
//! holds not lethal. The test asserts that the outline is convex and counterclockwise, as
//! its check of a point inside the outline needs, that each point in the band inside it or
//! within `EDGE_MARGIN_M` past it lies in a cell Nav2 holds not lethal, and that the
//! scenarios hold a point in the band inside it by more than `EDGE_MARGIN_M`. The evidence:
//! every cloud of every obstacle projection scenario of the ROS reference harness.

mod common;

use common::clouds::{
    EDGE_MARGIN_M, assert_clear_of_edges, assert_on_step, geometry_of, heights_of, insert_cloud,
    live_obstacles_over, padded_as_nav2,
};
use common::footprints::inside_by;
use common::layers::lethal_cells;
use common::poses::planar;
use control_core::planar_pose::PlanarPose;
use navigation_core::grid::GridGeometry;
use navigation_core::obstacles::ObstacleHeights;
use ros_reference::obstacle_projection::{Cases, Cloud};

/// The cells of the points of `cloud` in the band of `heights` that lie inside the convex,
/// counterclockwise unpadded outline `outline` by more than `margin` at the cloud's base
/// pose (a `margin` below 0 reaches that far past it): the robot's own body, each point's
/// cell on the grid of `geometry`.
fn cells_of_the_body(
    cloud: &Cloud,
    outline: &[[f64; 2]],
    margin: f64,
    heights: ObstacleHeights,
    geometry: GridGeometry,
) -> Vec<(u32, u32)> {
    let [x, y, yaw] = cloud.base;
    let base = planar(x, y, yaw);
    let band = heights.min_obstacle_height_m()..=heights.max_obstacle_height_m();
    cloud
        .points_in_grid
        .iter()
        .map(|point| point.map(f64::from))
        .filter(|&[px, py, pz]| {
            band.contains(&pz) && inside_by(outline, base.apply_inverse([px, py]), margin)
        })
        .filter_map(|[px, py, _]| geometry.cell_of([px, py]))
        .map(|cell| (cell.x(), cell.y()))
        .collect()
}

// The lethal cells of each cloud are those of Nav2's obstacle layer.
#[test]
fn the_lethal_cells_are_nav2s_for_the_same_points() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds an obstacle projection scenario"
    );
    let mut points_of_the_body = 0;
    for cases in &scenarios {
        let source = &cases.obstacle.depth;
        assert!(
            source.marking && !source.clearing && cases.obstacle.footprint_clearing_enabled,
            "{}: the source marks, its rays clear nothing, and the footprint clears",
            cases.name
        );
        let resolution = cases.grid.resolution;
        let heights = heights_of(
            cases.obstacle.min_obstacle_height,
            cases.obstacle.max_obstacle_height,
            resolution,
        );
        assert_on_step(
            heights.min_obstacle_height_m(),
            resolution,
            "the band's lower end",
        );
        assert_on_step(
            heights.max_obstacle_height_m(),
            resolution,
            "the band's upper end",
        );
        let footprint = padded_as_nav2(
            &cases.footprint,
            cases.footprint_padding,
            &cases.padded_footprint,
            &cases.name,
        );
        assert!(
            cases.footprint.iter().all(|&vertex| inside_by(
                &cases.footprint,
                vertex,
                -EDGE_MARGIN_M
            )),
            "{}: the unpadded outline is convex and counterclockwise",
            cases.name
        );
        let geometry = geometry_of(&cases.grid);
        for (index, cloud) in cases.clouds.iter().enumerate() {
            let what = format!("{} cloud {index}", cases.name);
            assert_clear_of_edges(&what, &cases.grid, heights, &cloud.points_in_grid);
            let body =
                cells_of_the_body(cloud, &cases.footprint, -EDGE_MARGIN_M, heights, geometry);
            let body_lethal: Vec<_> = body
                .iter()
                .filter(|cell| cloud.lethal.contains(cell))
                .collect();
            assert!(
                body_lethal.is_empty(),
                "{what}: points of the robot's body lie in cells Nav2 holds lethal: {body_lethal:?}"
            );
            points_of_the_body +=
                cells_of_the_body(cloud, &cases.footprint, EDGE_MARGIN_M, heights, geometry).len();
            let mut live = live_obstacles_over(
                &cases.grid,
                resolution,
                heights,
                source.obstacle_max_range,
                footprint.clone(),
            );
            assert_eq!(
                live.columns(),
                geometry,
                "{what}: the map's columns are the grid"
            );
            insert_cloud(
                &mut live,
                &cloud.sensor,
                &cloud.points,
                cloud.base,
                source.obstacle_max_range,
            );
            let projected = live
                .project(PlanarPose::IDENTITY, geometry)
                .expect("a finite placement");
            let ours = lethal_cells(&projected);
            assert_eq!(
                ours,
                cloud.lethal,
                "{what}: ours alone {:?}, Nav2's alone {:?}",
                ours.difference(&cloud.lethal).collect::<Vec<_>>(),
                cloud.lethal.difference(&ours).collect::<Vec<_>>()
            );
        }
    }
    assert!(
        points_of_the_body > 0,
        "a cloud holds a point of the robot's body inside its outline by more than the margin"
    );
}
