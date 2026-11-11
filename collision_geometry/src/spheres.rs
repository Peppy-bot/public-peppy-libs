// Software License Agreement (BSD License)
//
//  Copyright (c) 2010, Willow Garage, Inc.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of the Willow Garage nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//
// Copyright 2008 Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This file translates into Rust MoveIt 2.15.2's placement of collision spheres
// (`determineCollisionSpheres`, collision_distance_field_types.cpp, under the first notice)
// and geometric_shapes 2.3.4's bounding cylinder of a body (bodies.cpp, under the second).

//! The collision spheres of a body, as MoveIt 2.15.2's distance-field collision checker
//! places them for CHOMP (`determineCollisionSpheres`): a sphere body is one sphere of
//! its own radius, unpadded, as the checker reads a sphere's dimensions; every other body
//! is spheres of its bounding cylinder's radius on the cylinder's axis,
//! `ceil(length / (radius / 2))` points evenly spaced from end to end, the two at the
//! ends left out. The bounding cylinder is geometric_shapes 2.3.4's, of the body grown by
//! the padding: a box's runs along its longest side, its radius the half diagonal of the
//! other two sides; a cylinder's is the cylinder; a mesh's runs along the longest side of
//! the box of its vertices in the mesh's own frame, through the box's center, as long as
//! that side, its radius the farthest vertex's distance from the axis, and the padding
//! adds to the radius and twice to the length.

use std::f64::consts::FRAC_PI_2;

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use crate::urdf_collision::{Collision, CollisionGeometry};

/// The fewest points on a cylinder's axis that leave a sphere between the two at its
/// ends.
const LEAST_POINTS: f64 = 3.0;

/// One collision sphere, in the frame of the link that carries it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionSphere {
    pub center: Point3<f64>,
    pub radius: f64,
}

/// The body a `<collision>` element's collision spheres are placed on, in its link's
/// frame.
#[derive(Debug, Clone, PartialEq)]
pub enum SphereSource {
    /// A sphere: itself.
    Ball { center: Point3<f64>, radius: f64 },
    /// A box of these half sides, placed at `pose`.
    Cuboid {
        pose: Isometry3<f64>,
        half: Vector3<f64>,
    },
    /// A cylinder along its z axis, placed at `pose`.
    Cylinder {
        pose: Isometry3<f64>,
        radius: f64,
        half_length: f64,
    },
    /// A mesh's bounding cylinder, along the z axis of `pose`.
    Mesh {
        pose: Isometry3<f64>,
        length: f64,
        radius: f64,
    },
}

impl SphereSource {
    /// The body of `element`, whose geometry in its own frame, a mesh's vertices scaled,
    /// is `local`.
    pub(crate) fn of(element: &Collision, local: &[Point3<f64>]) -> Self {
        let pose = element.origin;
        match &element.geometry {
            CollisionGeometry::Sphere { radius } => Self::Ball {
                center: Point3::from(pose.translation.vector),
                radius: *radius,
            },
            CollisionGeometry::Box { size } => Self::Cuboid {
                pose,
                half: size / 2.0,
            },
            CollisionGeometry::Cylinder { radius, length } => Self::Cylinder {
                pose,
                radius: *radius,
                half_length: length / 2.0,
            },
            CollisionGeometry::Mesh { .. } => mesh_cylinder(&pose, local),
        }
    }

    /// The collision spheres of the body grown by `padding`, in its link's frame.
    pub fn spheres(&self, padding: f64) -> Vec<CollisionSphere> {
        match self {
            Self::Ball { center, radius } => vec![CollisionSphere {
                center: *center,
                radius: *radius,
            }],
            Self::Cuboid { pose, half } => {
                let grown = half.add_scalar(padding);
                let (turn, axis, [a, b]) = longest_side(&grown);
                along_axis(
                    &(pose * turn),
                    2.0 * grown[axis],
                    (grown[a] * grown[a] + grown[b] * grown[b]).sqrt(),
                )
            }
            Self::Cylinder {
                pose,
                radius,
                half_length,
            } => along_axis(pose, 2.0 * (half_length + padding), radius + padding),
            Self::Mesh {
                pose,
                length,
                radius,
            } => along_axis(pose, length + 2.0 * padding, radius + padding),
        }
    }
}

/// The bounding cylinder of the mesh of `vertices`, in the mesh's own frame, placed by
/// `pose`.
fn mesh_cylinder(pose: &Isometry3<f64>, vertices: &[Point3<f64>]) -> SphereSource {
    let Some(first) = vertices.first() else {
        return SphereSource::Mesh {
            pose: *pose,
            length: 0.0,
            radius: 0.0,
        };
    };
    let (min, max) = vertices
        .iter()
        .fold((first.coords, first.coords), |(min, max), v| {
            (min.inf(&v.coords), max.sup(&v.coords))
        });
    let center = (min + max) / 2.0;
    let (turn, axis, [a, b]) = longest_side(&(max - min));
    let radius = vertices
        .iter()
        .map(|v| (v[a] - center[a]).hypot(v[b] - center[b]))
        .fold(0.0, f64::max);
    SphereSource::Mesh {
        pose: pose * Translation3::from(center) * turn,
        length: max[axis] - min[axis],
        radius,
    }
}

/// The turn that lays a cylinder's z axis along the longest of the sides `sides`, the
/// index of that side, and the indices of the other two, as geometric_shapes'
/// `Box::computeBoundingCylinder` takes them: x when it is longer than both others, else
/// y when it is longer than z, else z; a quarter turn about y lays z on x, and one about
/// x lays z on -y.
fn longest_side(sides: &Vector3<f64>) -> (UnitQuaternion<f64>, usize, [usize; 2]) {
    if sides.x > sides.y && sides.x > sides.z {
        (
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), FRAC_PI_2),
            0,
            [1, 2],
        )
    } else if sides.y > sides.z {
        (
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_2),
            1,
            [2, 0],
        )
    } else {
        (UnitQuaternion::identity(), 2, [0, 1])
    }
}

/// MoveIt's spheres of `radius` on the z axis of `pose` along a cylinder of `length`
/// centered there, from the -z end toward +z. A cylinder of fewer than three points, and one
/// whose count is no number, a radius of 0, has none: there MoveIt's loop places none,
/// or its count has no value.
fn along_axis(pose: &Isometry3<f64>, length: f64, radius: f64) -> Vec<CollisionSphere> {
    let points = (length / (radius / 2.0)).ceil();
    if !(points.is_finite() && points >= LEAST_POINTS) {
        return Vec::new();
    }
    let points = points as usize;
    let spacing = length / (points - 1) as f64;
    (1..points - 1)
        .map(|i| CollisionSphere {
            center: pose * Point3::new(0.0, 0.0, -length / 2.0 + i as f64 * spacing),
            radius,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn element(geometry: CollisionGeometry, origin: Isometry3<f64>) -> Collision {
        Collision {
            link: "link".to_owned(),
            geometry,
            origin,
        }
    }

    fn assert_near(a: Point3<f64>, b: Point3<f64>) {
        assert!((a - b).norm() < 1e-12, "{a} against {b}");
    }

    // A box 0.1 by 0.4 by 0.2: its cylinder runs along y, the longest side, 0.4 long,
    // its radius the half diagonal of the x and z sides, sqrt(0.05^2 + 0.1^2); there are
    // ceil(0.4 / (radius / 2)) points, the two at the ends left out, from -y to +y after
    // the quarter turn about x lays z on -y.
    #[test]
    fn a_box_spheres_lie_along_its_longest_side() {
        let source = SphereSource::of(
            &element(
                CollisionGeometry::Box {
                    size: Vector3::new(0.1, 0.4, 0.2),
                },
                Isometry3::translation(1.0, 0.0, 0.0),
            ),
            &[],
        );
        let radius = (0.05_f64.powi(2) + 0.1_f64.powi(2)).sqrt();
        let points = (0.4 / (radius / 2.0)).ceil() as usize;
        let spheres = source.spheres(0.0);
        assert_eq!(spheres.len(), points - 2);
        let spacing = 0.4 / (points - 1) as f64;
        for (i, sphere) in spheres.iter().enumerate() {
            assert!((sphere.radius - radius).abs() < 1e-15);
            let y = 0.2 - (i + 1) as f64 * spacing;
            assert_near(sphere.center, Point3::new(1.0, y, 0.0));
        }
    }

    // Padding grows a box's half sides before its cylinder is taken: the radius is the
    // half diagonal of the grown sides, and the length twice the grown longest half.
    #[test]
    fn padding_grows_a_box_before_its_cylinder() {
        let source = SphereSource::Cuboid {
            pose: Isometry3::identity(),
            half: Vector3::new(0.05, 0.05, 0.3),
        };
        let spheres = source.spheres(0.01);
        let radius = (2.0 * 0.06_f64.powi(2)).sqrt();
        let points = (0.62 / (radius / 2.0)).ceil() as usize;
        assert_eq!(spheres.len(), points - 2);
        assert!(spheres.iter().all(|s| (s.radius - radius).abs() < 1e-15));
    }

    // A box's ties go as geometric_shapes breaks them: x only when longer than both
    // others, so a cube's cylinder runs along z.
    #[test]
    fn a_cube_runs_along_z() {
        let (turn, axis, others) = longest_side(&Vector3::repeat(0.2));
        assert_eq!((axis, others), (2, [0, 1]));
        assert_eq!(turn, UnitQuaternion::identity());
        let (turn, axis, _) = longest_side(&Vector3::new(0.3, 0.1, 0.1));
        assert_eq!(axis, 0);
        assert_near(
            turn * Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 0.0),
        );
    }

    // A sphere is one sphere at its center, at its own radius whatever the padding.
    #[test]
    fn a_sphere_is_itself_unpadded() {
        let source = SphereSource::of(
            &element(
                CollisionGeometry::Sphere { radius: 0.03 },
                Isometry3::translation(0.0, 0.2, 0.0),
            ),
            &[],
        );
        assert_eq!(
            source.spheres(0.05),
            vec![CollisionSphere {
                center: Point3::new(0.0, 0.2, 0.0),
                radius: 0.03,
            }]
        );
    }

    // A cylinder's spheres run along its own axis, the padding added to its radius and
    // to each end.
    #[test]
    fn a_cylinder_runs_along_its_own_axis() {
        let source = SphereSource::Cylinder {
            pose: Isometry3::rotation(Vector3::x() * FRAC_PI_2),
            radius: 0.02,
            half_length: 0.1,
        };
        let spheres = source.spheres(0.01);
        let points = (0.22_f64 / (0.03 / 2.0)).ceil() as usize;
        assert_eq!(spheres.len(), points - 2);
        // A quarter turn about x lays the cylinder's z axis on -y.
        for sphere in &spheres {
            assert!((sphere.radius - 0.03).abs() < 1e-15);
            assert!(sphere.center.x.abs() < 1e-15 && sphere.center.z.abs() < 1e-15);
            assert!(sphere.center.y.abs() < 0.11);
        }
    }

    // A mesh's cylinder runs through the center of the box of its vertices along its
    // longest side, its radius the farthest vertex's distance from that axis, placed by
    // the element's origin.
    #[test]
    fn a_mesh_cylinder_runs_through_its_box() {
        let vertices = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.1, 0.0),
            Point3::new(0.2, 0.0, 0.06),
            Point3::new(0.4, -0.02, 0.01),
        ];
        let origin = Isometry3::translation(0.0, 0.0, 1.0);
        let SphereSource::Mesh {
            pose,
            length,
            radius,
        } = SphereSource::of(
            &element(
                CollisionGeometry::Mesh {
                    basename: "mesh.stl".to_owned(),
                    scale: Vector3::repeat(1.0),
                },
                origin,
            ),
            &vertices,
        )
        else {
            panic!("a mesh's source is its cylinder");
        };
        assert!((length - 0.5).abs() < 1e-15);
        let (center_y, center_z) = (0.04, 0.03);
        let farthest = vertices
            .iter()
            .map(|v| (v.y - center_y).hypot(v.z - center_z))
            .fold(0.0, f64::max);
        assert!((radius - farthest).abs() < 1e-15);
        assert_near(
            pose * Point3::origin(),
            Point3::new(0.25, center_y, 1.0 + center_z),
        );
        assert_near(
            pose * Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.25, center_y, 1.0 + center_z),
        );
    }

    // A body shorter than its radius gives fewer than three points, so no sphere, and a
    // mesh with no vertices none either.
    #[test]
    fn a_short_body_has_no_sphere() {
        let disc = SphereSource::Cylinder {
            pose: Isometry3::identity(),
            radius: 0.1,
            half_length: 0.04,
        };
        assert!(disc.spheres(0.0).is_empty());
        assert!(
            mesh_cylinder(&Isometry3::identity(), &[])
                .spheres(0.0)
                .is_empty()
        );
    }
}
