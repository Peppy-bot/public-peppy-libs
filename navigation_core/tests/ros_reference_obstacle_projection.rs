//! The live obstacles against Nav2's obstacle layer (nav2_costmap_2d 1.5.2): after one depth
//! frame's points go into an occupancy map and the map's columns under the padded footprint
//! are cleared, `LiveObstacles::project` marks lethal the cells over the occupied voxels
//! whose centers lie in the height band, and Nav2's obstacle layer marks the cells of the
//! same points in the band, with clearing off, then clears the cells under the same
//! footprint, padded as Costmap2DROS pads it, at the same base pose. The map keeps a hit in
//! its own voxel, so with the band's ends on voxel boundaries, voxels of the grid's cells,
//! and every point clear of a cell's edge and of the band's ends, the lethal cells are
//! Nav2's, and the clearing's cells are, by its port of `convexFillCells`. The evidence:
//! every cloud of every obstacle projection scenario of the ROS reference harness.

mod common;

use camera_core::pose::PlanarPose;
use common::clouds::{
    assert_clear_of_edges, assert_on_step, geometry_of, heights_of, insert_cloud,
    live_obstacles_over, padded_as_nav2,
};
use common::layers::lethal_cells;
use ros_reference::obstacle_projection::Cases;

// The lethal cells of each cloud are those of Nav2's obstacle layer.
#[test]
fn the_lethal_cells_are_nav2s_for_the_same_points() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds an obstacle projection scenario"
    );
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
        let geometry = geometry_of(&cases.grid);
        for (index, cloud) in cases.clouds.iter().enumerate() {
            let what = format!("{} cloud {index}", cases.name);
            assert_clear_of_edges(&what, &cases.grid, heights, &cloud.points_in_grid);
            let mut live = live_obstacles_over(&cases.grid, resolution, heights, footprint.clone());
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
}
