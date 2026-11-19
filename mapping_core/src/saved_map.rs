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

//! A saved map: a directory that holds the pose graph with its scans
//! (`pose_graph.json`), the map's description (`meta.json`), and its grid as Nav2 1.5.2's
//! map saver writes a trinary map (`map.pgm` and `map.yaml`; nav2_map_server/src/map_io.cpp,
//! `tryWriteMapToFile`). Loading reads the pose graph, makes its grid anew, and refuses a
//! directory whose files disagree with it or with each other, naming the file and the
//! field.

use crate::geometry::{Matrix3, Pose2, Vector2};
use crate::localized_scan::{Laser, LocalizedScan, ScanId};
use crate::mapper::MapperState;
use crate::occupancy_grid::{GridError, GridExtent, GridParameters, OccupancyGrid};
use crate::placed_scan::Beam;
use crate::pose_graph::{Edge, LinkInfo};
use control_core::positive_finite::PositiveFinite;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::Path;
use thiserror::Error;

const POSE_GRAPH_FILE: &str = "pose_graph.json";
const META_FILE: &str = "meta.json";
const IMAGE_FILE: &str = "map.pgm";
const DESCRIPTION_FILE: &str = "map.yaml";

/// A room's map: the pose graph with its scans, and its grid.
#[derive(Debug, Clone, PartialEq)]
pub struct RoomMap {
    pub(crate) state: MapperState,
    grid_parameters: GridParameters,
    grid: OccupancyGrid,
    created_at_unix_ns: i64,
}

/// A map as `room_map`'s map object describes it: where its grid lies, the number of
/// scans its mapping added to its graph, and when its mapping started (ns since the Unix
/// epoch, on the wall clock).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapSummary {
    pub extent: GridExtent,
    pub scans: u32,
    pub created_at_unix_ns: i64,
}

/// Why a map was not saved.
#[derive(Debug, Error)]
pub enum SaveError {
    #[error("cannot write {file}: {error}")]
    Write { file: String, error: std::io::Error },
}

/// Why a directory holds no map.
#[derive(Debug, Error)]
pub enum LoadError {
    #[error("cannot read {file}: {error}")]
    Read { file: String, error: std::io::Error },
    #[error("{file} does not parse: {error}")]
    Parse { file: &'static str, error: String },
    #[error("{file}: {field} {reason}")]
    Invalid {
        file: &'static str,
        field: String,
        reason: String,
    },
    #[error(
        "{file}: {field} disagrees with the map that pose_graph.json and meta.json give; save \
         the map again"
    )]
    Disagree { file: &'static str, field: String },
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct LaserFile {
    minimum_angle: f64,
    maximum_angle: f64,
    angular_resolution: f64,
    minimum_range: f64,
    maximum_range: f64,
    range_threshold: f64,
    readings: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct NodeFile {
    id: u32,
    /// The time of the scan (s).
    time_s: f64,
    odometric_pose: Pose2,
    corrected_pose: Pose2,
    /// Each beam, in ray order: its reading (m), null for a ray with no return, and where
    /// the laser stood at its time, `[x, y]` (m), and its bearing (rad), in the base's frame
    /// at the scan's timestamp.
    beams: Vec<BeamFile>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct BeamFile {
    reading: Option<f64>,
    origin: [f64; 2],
    bearing: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EdgeFile {
    source: u32,
    target: u32,
    pose1: Pose2,
    pose2: Pose2,
    pose_difference: Pose2,
    /// The link's covariance in the frame of the source, row by row.
    covariance: [[f64; 3]; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PoseGraphFile {
    laser: LaserFile,
    nodes: Vec<NodeFile>,
    edges: Vec<EdgeFile>,
    running_scans: Vec<u32>,
    last_scan: Option<u32>,
    next_id: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct MetaFile {
    resolution: f64,
    min_pass_through: u32,
    occupancy_threshold: f64,
    /// The corner of the grid's first cell (m).
    origin: [f64; 2],
    width: u32,
    height: u32,
    scans: usize,
    created_at_unix_ns: i64,
}

impl RoomMap {
    pub(crate) fn new(
        state: MapperState,
        grid_parameters: GridParameters,
        created_at_unix_ns: i64,
    ) -> Result<Option<Self>, GridError> {
        let Some(grid) =
            OccupancyGrid::from_scans(state.scans.values(), &state.laser, grid_parameters)?
        else {
            return Ok(None);
        };
        Ok(Some(Self {
            state,
            grid_parameters,
            grid,
            created_at_unix_ns,
        }))
    }

    /// The map's grid.
    pub fn grid(&self) -> &OccupancyGrid {
        &self.grid
    }

    /// The map as `room_map`'s map object describes it.
    pub fn summary(&self) -> MapSummary {
        MapSummary {
            extent: self.grid.extent(),
            scans: self.state.scans.len() as u32,
            created_at_unix_ns: self.created_at_unix_ns,
        }
    }

    /// Whether the point `(x, y)` (m) of the map frame lies inside the map's grid.
    pub fn contains(&self, x: f64, y: f64) -> bool {
        let GridExtent {
            min_x_m,
            min_y_m,
            width_m,
            height_m,
            ..
        } = self.grid.extent();
        (min_x_m..=min_x_m + width_m).contains(&x) && (min_y_m..=min_y_m + height_m).contains(&y)
    }

    /// Writes the map into `directory`, which it creates.
    pub fn save(&self, directory: &Path) -> Result<(), SaveError> {
        let write = |file: &str, bytes: &[u8]| {
            std::fs::write(directory.join(file), bytes).map_err(|error| SaveError::Write {
                file: directory.join(file).display().to_string(),
                error,
            })
        };
        std::fs::create_dir_all(directory).map_err(|error| SaveError::Write {
            file: directory.display().to_string(),
            error,
        })?;
        write(
            POSE_GRAPH_FILE,
            json_text(&self.pose_graph_file()).as_bytes(),
        )?;
        write(META_FILE, json_text(&self.meta_file()).as_bytes())?;
        let files = self.grid.map_saver_files(IMAGE_FILE);
        write(IMAGE_FILE, &files.image)?;
        write(DESCRIPTION_FILE, files.description.as_bytes())
    }

    fn pose_graph_file(&self) -> PoseGraphFile {
        let Laser {
            minimum_angle,
            maximum_angle,
            angular_resolution,
            minimum_range,
            maximum_range,
            range_threshold,
            number_of_readings,
        } = self.state.laser;
        PoseGraphFile {
            laser: LaserFile {
                minimum_angle,
                maximum_angle,
                angular_resolution,
                minimum_range,
                maximum_range,
                range_threshold,
                readings: number_of_readings,
            },
            nodes: self
                .state
                .scans
                .iter()
                .map(|(id, scan)| NodeFile {
                    id: id.0,
                    time_s: scan.time,
                    odometric_pose: scan.odometric_pose(),
                    corrected_pose: scan.corrected_pose(),
                    beams: scan
                        .beams
                        .iter()
                        .map(|beam| BeamFile {
                            reading: (beam.reading != f64::INFINITY).then_some(beam.reading),
                            origin: [beam.origin.x, beam.origin.y],
                            bearing: beam.bearing,
                        })
                        .collect(),
                })
                .collect(),
            edges: self
                .state
                .edges
                .iter()
                .map(|edge| EdgeFile {
                    source: edge.source.0,
                    target: edge.target.0,
                    pose1: edge.link.pose1,
                    pose2: edge.link.pose2,
                    pose_difference: edge.link.pose_difference,
                    covariance: edge.link.covariance.0,
                })
                .collect(),
            running_scans: self.state.running_scans.iter().map(|id| id.0).collect(),
            last_scan: self.state.last_scan.map(|id| id.0),
            next_id: self.state.next_id,
        }
    }

    fn meta_file(&self) -> MetaFile {
        MetaFile {
            resolution: self.grid_parameters.resolution,
            min_pass_through: self.grid_parameters.min_pass_through,
            occupancy_threshold: self.grid_parameters.occupancy_threshold,
            origin: self.grid.origin(),
            width: self.grid.width(),
            height: self.grid.height(),
            scans: self.state.scans.len(),
            created_at_unix_ns: self.created_at_unix_ns,
        }
    }

    /// The map that `directory` holds.
    pub fn load(directory: &Path) -> Result<RoomMap, LoadError> {
        let pose_graph: PoseGraphFile = parse_file(directory, POSE_GRAPH_FILE)?;
        let meta: MetaFile = parse_file(directory, META_FILE)?;
        let state = state_of(&pose_graph)?;
        if meta.scans != state.scans.len() {
            return Err(LoadError::Disagree {
                file: META_FILE,
                field: "scans".into(),
            });
        }
        let invalid_resolution = |reason: String| LoadError::Invalid {
            file: META_FILE,
            field: "resolution".into(),
            reason,
        };
        let resolution = PositiveFinite::parse(meta.resolution)
            .map_err(|_| {
                invalid_resolution(format!(
                    "is {}, and the side of a cell is finite and above 0",
                    meta.resolution
                ))
            })?
            .get();
        let grid_parameters = GridParameters {
            resolution,
            min_pass_through: meta.min_pass_through,
            occupancy_threshold: meta.occupancy_threshold,
        };
        let map = RoomMap::new(state, grid_parameters, meta.created_at_unix_ns)
            .map_err(|error| invalid_resolution(format!("is {resolution}: {error}")))?
            .ok_or_else(|| LoadError::Invalid {
                file: POSE_GRAPH_FILE,
                field: "nodes".into(),
                reason: "is empty, and a map holds a scan".into(),
            })?;
        let fields = [
            ("origin", meta.origin == map.grid.origin()),
            ("width", meta.width == map.grid.width()),
            ("height", meta.height == map.grid.height()),
        ];
        if let Some((field, _)) = fields.into_iter().find(|(_, agrees)| !agrees) {
            return Err(LoadError::Disagree {
                file: META_FILE,
                field: field.into(),
            });
        }
        let files = map.grid.map_saver_files(IMAGE_FILE);
        if read_file(directory, IMAGE_FILE)? != files.image {
            return Err(LoadError::Disagree {
                file: IMAGE_FILE,
                field: "pixels".into(),
            });
        }
        let description =
            String::from_utf8_lossy(&read_file(directory, DESCRIPTION_FILE)?).into_owned();
        if let Some(field) = first_differing_key(&description, &files.description) {
            return Err(LoadError::Disagree {
                file: DESCRIPTION_FILE,
                field,
            });
        }
        Ok(map)
    }
}

/// The text of a file of the map: pretty JSON with a final newline.
fn json_text(value: &impl Serialize) -> String {
    let mut text = serde_json::to_string_pretty(value).expect("a map's files serialize");
    text.push('\n');
    text
}

fn read_file(directory: &Path, file: &str) -> Result<Vec<u8>, LoadError> {
    std::fs::read(directory.join(file)).map_err(|error| LoadError::Read {
        file: directory.join(file).display().to_string(),
        error,
    })
}

fn parse_file<T: serde::de::DeserializeOwned>(
    directory: &Path,
    file: &'static str,
) -> Result<T, LoadError> {
    serde_json::from_slice(&read_file(directory, file)?).map_err(|error| LoadError::Parse {
        file,
        error: error.to_string(),
    })
}

/// The key of the first line of `held` that differs from `expected`, or `held`'s extra
/// or missing line, or none when they agree.
fn first_differing_key(held: &str, expected: &str) -> Option<String> {
    if held == expected {
        return None;
    }
    let key = |line: &str| line.split(':').next().unwrap_or(line).to_owned();
    let (mut held_lines, mut expected_lines) = (held.lines(), expected.lines());
    loop {
        match (held_lines.next(), expected_lines.next()) {
            (Some(a), Some(b)) if a == b => continue,
            (Some(a), _) => return Some(key(a)),
            (None, Some(b)) => return Some(key(b)),
            (None, None) => return Some("the final newline".into()),
        }
    }
}

/// The mapper's state that the pose graph file holds, checked: each node's ranges are the
/// laser's readings, the ids are those of earlier scans than the next id, and each edge,
/// the running buffer and the last scan name nodes.
fn state_of(file: &PoseGraphFile) -> Result<MapperState, LoadError> {
    let invalid = |field: String, reason: &str| LoadError::Invalid {
        file: POSE_GRAPH_FILE,
        field,
        reason: reason.to_owned(),
    };
    let LaserFile {
        minimum_angle,
        maximum_angle,
        angular_resolution,
        minimum_range,
        maximum_range,
        range_threshold,
        readings,
    } = file.laser;
    let laser = Laser {
        minimum_angle,
        maximum_angle,
        angular_resolution,
        minimum_range,
        maximum_range,
        range_threshold,
        number_of_readings: readings,
    };
    let mut scans = BTreeMap::new();
    for (index, node) in file.nodes.iter().enumerate() {
        if node.beams.len() != readings {
            return Err(invalid(
                format!("nodes[{index}].beams"),
                &format!(
                    "holds {} beams, and the laser reads {readings}",
                    node.beams.len()
                ),
            ));
        }
        if node.id >= file.next_id {
            return Err(invalid(
                format!("nodes[{index}].id"),
                "is not below next_id",
            ));
        }
        let beams = node
            .beams
            .iter()
            .map(|beam| Beam {
                reading: beam.reading.unwrap_or(f64::INFINITY),
                origin: Vector2::new(beam.origin[0], beam.origin[1]),
                bearing: beam.bearing,
            })
            .collect();
        let scan = LocalizedScan::with_poses(
            &laser,
            beams,
            node.time_s,
            node.odometric_pose,
            node.corrected_pose,
        );
        if scans.insert(ScanId(node.id), scan).is_some() {
            return Err(invalid(format!("nodes[{index}].id"), "names a node twice"));
        }
    }
    let known = |id: u32, field: String| {
        if scans.contains_key(&ScanId(id)) {
            Ok(ScanId(id))
        } else {
            Err(invalid(field, "names no node"))
        }
    };
    let edges = file
        .edges
        .iter()
        .enumerate()
        .map(|(index, edge)| {
            Ok(Edge {
                source: known(edge.source, format!("edges[{index}].source"))?,
                target: known(edge.target, format!("edges[{index}].target"))?,
                link: LinkInfo {
                    pose1: edge.pose1,
                    pose2: edge.pose2,
                    pose_difference: edge.pose_difference,
                    covariance: Matrix3(edge.covariance),
                },
            })
        })
        .collect::<Result<Vec<_>, LoadError>>()?;
    let running_scans = file
        .running_scans
        .iter()
        .enumerate()
        .map(|(index, &id)| known(id, format!("running_scans[{index}]")))
        .collect::<Result<Vec<_>, LoadError>>()?;
    let last_scan = file
        .last_scan
        .map(|id| known(id, "last_scan".into()))
        .transpose()?;
    Ok(MapperState {
        laser,
        scans,
        edges,
        running_scans,
        last_scan,
        next_id: file.next_id,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_differing_line_is_named_by_its_key() {
        let expected = "image: map.pgm\nresolution: 0.050";
        assert_eq!(first_differing_key(expected, expected), None);
        assert_eq!(
            first_differing_key("image: map.pgm\nresolution: 0.100", expected),
            Some("resolution".into())
        );
        assert_eq!(
            first_differing_key("image: map.pgm\nresolution: 0.050\n", expected),
            Some("the final newline".into())
        );
    }
}
