//! URDF collision extraction for the fit: the surface of each `<collision>` element of
//! a link, with its origin and, for a mesh, its scale applied, the tree of links and
//! joints, the fixed-link world poses, and the 1-DOF joint transforms that place a
//! link on a movable joint ([`place_1dof`]).

use std::collections::HashMap;
use std::sync::LazyLock;

use nalgebra::{Isometry3, Point3, Translation3, Unit, UnitQuaternion, Vector3};

use crate::GeometryError;
use crate::hull::exact_hull;

/// The sides of the prism that holds a `<cylinder>`: its corners lie on a circle
/// `1 / cos(pi / CYLINDER_SIDES)` times the radius, so its faces touch the cylinder.
const CYLINDER_SIDES: usize = 64;

/// The points of the polytope that holds a `<sphere>`, spread over the sphere and
/// moved out until the nearest face of their hull touches it.
const SPHERE_POINTS: usize = 1024;

/// One `<collision>` element of a link: its geometry, and the pose of the geometry in
/// the link frame.
#[derive(Debug, Clone)]
pub struct Collision {
    pub link: String,
    pub geometry: CollisionGeometry,
    pub origin: Isometry3<f64>,
}

/// The geometry of a `<collision>` element.
#[derive(Debug, Clone, PartialEq)]
pub enum CollisionGeometry {
    /// A mesh file by its basename, e.g. `link3_symp.stl`; the caller resolves it
    /// against its own assets directory (the URDF's `package://` URI is not a
    /// filesystem path). `scale` applies componentwise before the origin, and a
    /// component can be negative (mirrored geometry).
    Mesh {
        basename: String,
        scale: Vector3<f64>,
    },
    /// A box of these side lengths, centered on the origin.
    Box { size: Vector3<f64> },
    /// A cylinder of this radius and length along z, centered on the origin.
    Cylinder { radius: f64, length: f64 },
    /// A sphere of this radius, centered on the origin.
    Sphere { radius: f64 },
}

impl Collision {
    /// The element's surface in the link frame, as triangles of three points each. A
    /// mesh gives its own triangles, scaled. A box gives its own faces, a cylinder the
    /// faces of the [`CYLINDER_SIDES`]-sided prism about it, and a sphere the faces of
    /// a hull of [`SPHERE_POINTS`] points about it, so the surface holds the shape.
    pub fn surface(&self, meshes_dir: &str) -> Result<Vec<Point3<f64>>, String> {
        let local = match &self.geometry {
            CollisionGeometry::Mesh { basename, scale } => {
                crate::stl::load_stl(&format!("{meshes_dir}/{basename}"))?
                    .into_iter()
                    .map(|v| Point3::from(v.coords.component_mul(scale)))
                    .collect()
            }
            CollisionGeometry::Box { size } => triangles_of_hull(&box_corners(size))?,
            CollisionGeometry::Cylinder { radius, length } => {
                triangles_of_hull(&prism_corners(*radius, *length))?
            }
            CollisionGeometry::Sphere { radius } => {
                UNIT_SPHERE_SURFACE.iter().map(|v| v * *radius).collect()
            }
        };
        Ok(local.into_iter().map(|v| self.origin * v).collect())
    }
}

/// The geometry of a URDF `<collision>` element of `link`, refusing a shape the fit
/// cannot hold and a primitive whose size is not positive and finite.
fn collision_geometry(
    link: &str,
    geometry: &urdf_rs::Geometry,
) -> Result<CollisionGeometry, GeometryError> {
    let unsupported = |reason: String| GeometryError::UnsupportedCollision {
        link: link.to_owned(),
        reason,
    };
    let positive = |name: &str, value: f64| {
        if value.is_finite() && value > 0.0 {
            Ok(value)
        } else {
            Err(unsupported(format!(
                "its {name} is {value}; a primitive needs a positive finite size"
            )))
        }
    };
    match geometry {
        urdf_rs::Geometry::Mesh { filename, scale } => Ok(CollisionGeometry::Mesh {
            basename: filename.rsplit('/').next().unwrap_or(filename).to_owned(),
            scale: scale.map_or_else(|| Vector3::repeat(1.0), |s| Vector3::from(s.0)),
        }),
        urdf_rs::Geometry::Box { size } => Ok(CollisionGeometry::Box {
            size: Vector3::new(
                positive("x size", size[0])?,
                positive("y size", size[1])?,
                positive("z size", size[2])?,
            ),
        }),
        urdf_rs::Geometry::Cylinder { radius, length } => Ok(CollisionGeometry::Cylinder {
            radius: positive("radius", *radius)?,
            length: positive("length", *length)?,
        }),
        urdf_rs::Geometry::Sphere { radius } => Ok(CollisionGeometry::Sphere {
            radius: positive("radius", *radius)?,
        }),
        urdf_rs::Geometry::Capsule { .. } => Err(unsupported(
            "a <capsule> is no URDF shape the fit holds; give a <cylinder> and two <sphere>s"
                .to_owned(),
        )),
    }
}

/// The eight corners of a box of side lengths `size` about the origin.
fn box_corners(size: &Vector3<f64>) -> Vec<Point3<f64>> {
    (0..8)
        .map(|corner: usize| {
            Point3::from(Vector3::from_fn(|axis, _| {
                let sign = if (corner >> axis) & 1 == 0 { -0.5 } else { 0.5 };
                sign * size[axis]
            }))
        })
        .collect()
}

/// The corners of the [`CYLINDER_SIDES`]-sided prism whose faces touch the cylinder of
/// `radius` and `length` along z about the origin.
fn prism_corners(radius: f64, length: f64) -> Vec<Point3<f64>> {
    let sides = CYLINDER_SIDES as f64;
    let corner_radius = radius / (std::f64::consts::PI / sides).cos();
    (0..CYLINDER_SIDES)
        .flat_map(|side| {
            let angle = std::f64::consts::TAU * side as f64 / sides;
            let (sin, cos) = angle.sin_cos();
            [-0.5, 0.5]
                .map(|end| Point3::new(corner_radius * cos, corner_radius * sin, end * length))
        })
        .collect()
}

/// The faces of a polytope that holds the unit sphere, as triangles: the hull of
/// [`SPHERE_POINTS`] points spread evenly over the sphere (a Fibonacci lattice), moved
/// out until the nearest plane of the hull lies at distance 1.
static UNIT_SPHERE_SURFACE: LazyLock<Vec<Point3<f64>>> = LazyLock::new(|| {
    let count = SPHERE_POINTS as f64;
    let golden_angle = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
    let on_sphere: Vec<Point3<f64>> = (0..SPHERE_POINTS)
        .map(|i| {
            let z = 1.0 - (2.0 * i as f64 + 1.0) / count;
            let ring = (1.0 - z * z).sqrt();
            let (sin, cos) = (golden_angle * i as f64).sin_cos();
            Point3::new(ring * cos, ring * sin, z)
        })
        .collect();
    let hull = exact_hull(&on_sphere).expect("points spread over a sphere span a solid");
    let nearest_plane = hull
        .planes
        .iter()
        .map(|&(_, offset)| offset)
        .fold(f64::INFINITY, f64::min);
    hull.faces
        .iter()
        .flatten()
        .map(|&vertex| hull.vertices[vertex] / nearest_plane)
        .collect()
});

/// The faces of the convex hull of `points`, as triangles of three points each.
fn triangles_of_hull(points: &[Point3<f64>]) -> Result<Vec<Point3<f64>>, String> {
    let hull = exact_hull(points)?;
    Ok(hull
        .faces
        .iter()
        .flatten()
        .map(|&vertex| hull.vertices[vertex])
        .collect())
}

/// Parsed URDF with the lookups the fit pipeline needs.
pub struct UrdfCollisions {
    collisions: Vec<Collision>,
    /// Child link name to its parent joint: parent link, joint kind, origin,
    /// axis, and position limits.
    parent_joints: HashMap<String, ParentJoint>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum JointKind {
    Fixed,
    /// A translation along a single axis, between its limits.
    Prismatic,
    /// A rotation about a single axis, between its limits.
    Revolute,
    /// A rotation about a single axis, with no limits.
    Continuous,
    /// A planar or floating joint: more than one degree of freedom.
    OtherMovable,
}

#[derive(Debug, Clone)]
pub struct ParentJoint {
    pub parent_link: String,
    pub kind: JointKind,
    pub origin: Isometry3<f64>,
    pub axis: Vector3<f64>,
    pub lower_limit: f64,
    pub upper_limit: f64,
}

impl ParentJoint {
    pub fn is_fixed(&self) -> bool {
        self.kind == JointKind::Fixed
    }

    /// Placement of the child link in the parent frame at joint position `q`: the
    /// joint origin for a fixed joint (`q` ignored), the origin composed with a
    /// translation along the axis for a prismatic joint, and with a rotation about
    /// the axis for a revolute or continuous joint. A planar or floating joint has more
    /// than one degree of freedom and is refused. It reads no mesh;
    /// [`UrdfCollisions::child_surfaces_in_parent`] places a child's surfaces with it.
    pub fn offset(&self, q: f64) -> Result<Isometry3<f64>, String> {
        let revolute = match self.kind {
            JointKind::Fixed => return Ok(self.origin),
            JointKind::Prismatic => false,
            JointKind::Revolute | JointKind::Continuous => true,
            JointKind::OtherMovable => {
                return Err(
                    "a planar or floating joint has more than one degree of freedom, \
                     which one position cannot place"
                        .into(),
                );
            }
        };
        if self.axis.norm() < ZERO_AXIS_EPS {
            return Err("parent joint has a zero axis".into());
        }
        // URDF axes are conventionally unit but the spec does not require it.
        let axis = Unit::new_normalize(self.axis);
        Ok(place_1dof(&self.origin, &axis, revolute, q))
    }
}

/// An axis whose norm is below this cannot be normalized into a joint
/// direction; shared by every joint-axis validation in the crate.
pub const ZERO_AXIS_EPS: f64 = 1e-12;

/// Compose a joint `origin` with a 1-DOF motion of `q` about (revolute) or along
/// (prismatic) the unit `axis`: the placement of [`ParentJoint::offset`], for a caller
/// that places a link at a position each query.
pub fn place_1dof(
    origin: &Isometry3<f64>,
    axis: &Unit<Vector3<f64>>,
    revolute: bool,
    q: f64,
) -> Isometry3<f64> {
    if revolute {
        origin * UnitQuaternion::from_axis_angle(axis, q)
    } else {
        origin * Translation3::from(axis.into_inner() * q)
    }
}

impl UrdfCollisions {
    /// Reads every `<collision>` element and every joint of the URDF. Refuses a
    /// collision shape the fit cannot hold, naming its link.
    pub fn from_urdf(urdf: &str) -> Result<Self, GeometryError> {
        let robot = urdf_rs::read_from_string(urdf).map_err(|e| format!("parse URDF: {e}"))?;

        let mut collisions = Vec::new();
        // Meshes are resolved by basename in a single flat directory (the vendored
        // convention), so two distinct meshes that share a basename would silently
        // resolve to the same file. Reject that ambiguity rather than fit the
        // wrong geometry.
        let mut by_basename: HashMap<String, String> = HashMap::new();
        for link in &robot.links {
            for c in &link.collision {
                if let urdf_rs::Geometry::Mesh { filename, .. } = &c.geometry {
                    let basename = filename.rsplit('/').next().unwrap_or(filename).to_string();
                    if by_basename
                        .insert(basename.clone(), filename.clone())
                        .is_some_and(|prev| prev != *filename)
                    {
                        return Err(format!(
                            "collision meshes share the basename '{basename}' from different paths; meshes resolve by basename in a flat directory, so they must be unique"
                        )
                        .into());
                    }
                }
                collisions.push(Collision {
                    link: link.name.clone(),
                    geometry: collision_geometry(&link.name, &c.geometry)?,
                    origin: pose_to_isometry(&c.origin),
                });
            }
        }

        let mut parent_joints = HashMap::new();
        for j in &robot.joints {
            let kind = match j.joint_type {
                urdf_rs::JointType::Fixed => JointKind::Fixed,
                urdf_rs::JointType::Prismatic => JointKind::Prismatic,
                urdf_rs::JointType::Revolute => JointKind::Revolute,
                urdf_rs::JointType::Continuous => JointKind::Continuous,
                _ => JointKind::OtherMovable,
            };
            // A caller places a link on a movable joint at positions between its limits,
            // so a movable joint's limits must be finite and ordered.
            let movable = !matches!(kind, JointKind::Fixed);
            let valid_limits = j.limit.lower.is_finite()
                && j.limit.upper.is_finite()
                && j.limit.lower <= j.limit.upper;
            if movable && !valid_limits {
                return Err(format!(
                    "joint to '{}' has invalid limits [{}, {}]",
                    j.child.link, j.limit.lower, j.limit.upper
                )
                .into());
            }
            let pj = ParentJoint {
                parent_link: j.parent.link.clone(),
                kind,
                origin: pose_to_isometry(&j.origin),
                axis: Vector3::new(j.axis.xyz[0], j.axis.xyz[1], j.axis.xyz[2]),
                lower_limit: j.limit.lower,
                upper_limit: j.limit.upper,
            };
            if parent_joints.insert(j.child.link.clone(), pj).is_some() {
                return Err(format!(
                    "link '{}' has two parent joints; URDF must be a tree",
                    j.child.link
                )
                .into());
            }
        }

        Ok(Self {
            collisions,
            parent_joints,
        })
    }

    pub fn from_file(path: &str) -> Result<Self, GeometryError> {
        let urdf = std::fs::read_to_string(path).map_err(|e| format!("read urdf '{path}': {e}"))?;
        Self::from_urdf(&urdf)
    }

    /// All collision elements of `link`, in URDF order (empty if it declares none).
    pub fn collisions_of(&self, link: &str) -> Vec<&Collision> {
        self.collisions.iter().filter(|c| c.link == link).collect()
    }

    /// Distinct names of links that declare collisions, in URDF order.
    pub fn collision_link_names(&self) -> Vec<String> {
        let mut names: Vec<String> = Vec::new();
        for c in &self.collisions {
            if !names.contains(&c.link) {
                names.push(c.link.clone());
            }
        }
        names
    }

    /// The joint whose child is `link`, if any.
    pub fn parent_joint(&self, link: &str) -> Option<&ParentJoint> {
        self.parent_joints.get(link)
    }

    /// Links whose parent joint hangs them directly below `link`.
    pub fn children_of(&self, link: &str) -> Vec<String> {
        let mut children: Vec<String> = self
            .parent_joints
            .iter()
            .filter(|(_, j)| j.parent_link == link)
            .map(|(child, _)| child.clone())
            .collect();
        children.sort(); // HashMap order is not deterministic; callers need stable output
        children
    }

    /// The surface of each collision element of `link`, in URDF order, in the link
    /// frame (see [`Collision::surface`]). Mesh files are resolved as
    /// `<meshes_dir>/<basename>`.
    pub fn link_surfaces(
        &self,
        link: &str,
        meshes_dir: &str,
    ) -> Result<Vec<Vec<Point3<f64>>>, String> {
        let entries = self.collisions_of(link);
        if entries.is_empty() {
            return Err(format!("link '{link}' has no collision elements"));
        }
        entries.iter().map(|c| c.surface(meshes_dir)).collect()
    }

    /// The collision surfaces of a world-fixed `link`, mapped into the URDF root frame
    /// through its fixed mount chain.
    pub fn fixed_surfaces_in_root(
        &self,
        link: &str,
        meshes_dir: &str,
    ) -> Result<Vec<Vec<Point3<f64>>>, String> {
        let pose = self.fixed_pose_in_root(link)?;
        Ok(posed(pose, self.link_surfaces(link, meshes_dir)?))
    }

    /// The collision surfaces of `child` posed at joint position `q`, mapped into the
    /// parent link's frame by [`ParentJoint::offset`]: a fixed (`q` ignored),
    /// prismatic, revolute or continuous child; a planar or floating one is refused.
    pub fn child_surfaces_in_parent(
        &self,
        child: &str,
        q: f64,
        meshes_dir: &str,
    ) -> Result<Vec<Vec<Point3<f64>>>, String> {
        let pose = self
            .parent_joint(child)
            .ok_or_else(|| format!("link '{child}' has no parent joint"))?
            .offset(q)
            .map_err(|e| format!("link '{child}': {e}"))?;
        Ok(posed(pose, self.link_surfaces(child, meshes_dir)?))
    }

    /// Pose of `link` in the URDF root frame, composing only fixed joints.
    /// Errs if any joint on the path is movable (such a link has no constant
    /// root pose and belongs to an FK chain instead), or if the parent chain
    /// does not terminate (a malformed, cyclic URDF).
    fn fixed_pose_in_root(&self, link: &str) -> Result<Isometry3<f64>, String> {
        let mut pose = Isometry3::identity();
        let mut current = link.to_string();
        for _ in 0..=self.parent_joints.len() {
            let Some(j) = self.parent_joints.get(&current) else {
                return Ok(pose); // reached the root
            };
            if !j.is_fixed() {
                return Err(format!(
                    "link '{link}' hangs below movable joint into '{current}', not world-fixed"
                ));
            }
            pose = j.origin * pose;
            current = j.parent_link.clone();
        }
        Err(format!(
            "link '{link}' has a cyclic parent chain in the URDF"
        ))
    }
}

/// `surfaces` moved by `pose`.
fn posed(pose: Isometry3<f64>, surfaces: Vec<Vec<Point3<f64>>>) -> Vec<Vec<Point3<f64>>> {
    surfaces
        .into_iter()
        .map(|surface| surface.into_iter().map(|v| pose * v).collect())
        .collect()
}

/// URDF `<origin xyz rpy>` to an isometry. URDF rpy is fixed-axis XYZ, which
/// is exactly `from_euler_angles(roll, pitch, yaw)`.
fn pose_to_isometry(p: &urdf_rs::Pose) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(p.xyz[0], p.xyz[1], p.xyz[2]),
        UnitQuaternion::from_euler_angles(p.rpy[0], p.rpy[1], p.rpy[2]),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const URDF: &str = r#"<?xml version="1.0"?>
    <robot name="t">
      <link name="world"/>
      <link name="body">
        <collision><origin rpy="0 0 0" xyz="0 0 0"/>
          <geometry><mesh filename="package://d/meshes/body.stl" scale="0.001 0.001 0.001"/></geometry>
        </collision>
      </link>
      <link name="mount"/>
      <link name="arm1">
        <collision><origin rpy="0 0 0" xyz="0.1 0 -0.2"/>
          <geometry><mesh filename="package://d/meshes/arm1.stl" scale="0.001 -0.001 0.001"/></geometry>
        </collision>
      </link>
      <link name="finger"/>
      <joint name="wb" type="fixed">
        <parent link="world"/><child link="body"/><origin rpy="0 0 0" xyz="0 0 0"/>
      </joint>
      <joint name="bm" type="fixed">
        <parent link="body"/><child link="mount"/><origin rpy="1.5707963267948966 0 0" xyz="0 0.1 0.5"/>
      </joint>
      <joint name="ma" type="revolute">
        <parent link="mount"/><child link="arm1"/><origin rpy="0 0 0" xyz="0 0 0.05"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="af" type="prismatic">
        <parent link="arm1"/><child link="finger"/><origin rpy="0 0 0" xyz="0 0 0.1"/>
        <axis xyz="0 -1 0"/><limit lower="0" upper="0.04" effort="1" velocity="1"/>
      </joint>
    </robot>"#;

    #[test]
    fn poses_a_revolute_child_rather_than_rejecting_it() {
        let u = UrdfCollisions::from_urdf(URDF).expect("parse");
        // A revolute child (arm1 hangs off revolute joint `ma`) is now posed by rotation,
        // so the only failure here is the nonexistent mesh dir, not the joint kind.
        let err = u
            .child_surfaces_in_parent("arm1", 0.5, "/nonexistent")
            .expect_err("mesh dir does not exist");
        assert!(
            err.contains("/nonexistent"),
            "the revolute path must reach mesh loading and fail on the missing dir, got: {err}"
        );
    }

    /// A child on a continuous joint is placed as on a revolute one, with no limits.
    #[test]
    fn places_a_continuous_child_by_a_rotation_about_its_axis() {
        let continuous = URDF.replace(
            r#"<joint name="ma" type="revolute">"#,
            r#"<joint name="ma" type="continuous">"#,
        );
        let u = UrdfCollisions::from_urdf(&continuous).expect("parse");
        let joint = u.parent_joint("arm1").expect("arm1 has a parent joint");
        assert_eq!(joint.kind, JointKind::Continuous);
        let turned = joint
            .offset(FRAC_PI_2)
            .expect("a continuous joint places its child");
        let expected =
            joint.origin * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        assert!((turned.translation.vector - expected.translation.vector).norm() < 1e-12);
        assert!(turned.rotation.angle_to(&expected.rotation) < 1e-12);
    }

    #[test]
    fn rejects_a_prismatic_child_with_a_zero_axis() {
        let zero_axis = URDF.replace(r#"<axis xyz="0 -1 0"/>"#, r#"<axis xyz="0 0 0"/>"#);
        let u = UrdfCollisions::from_urdf(&zero_axis).expect("parse");
        let err = u
            .child_surfaces_in_parent("finger", 0.02, "/nonexistent")
            .expect_err("zero axis");
        assert!(err.contains("zero axis"), "{err}");
    }

    #[test]
    fn extracts_mesh_scale_and_origin() {
        let u = UrdfCollisions::from_urdf(URDF).expect("parse");
        let arm = u.collisions_of("arm1");
        assert_eq!(arm.len(), 1);
        assert_eq!(
            arm[0].geometry,
            CollisionGeometry::Mesh {
                basename: "arm1.stl".to_owned(),
                scale: Vector3::new(0.001, -0.001, 0.001),
            }
        );
        assert!((arm[0].origin.translation.vector - Vector3::new(0.1, 0.0, -0.2)).norm() < 1e-12);
        assert!(u.collisions_of("mount").is_empty());
    }

    /// A mesh's surface is its triangles, scaled componentwise, then moved by the origin.
    #[test]
    fn a_mesh_surface_applies_scale_then_origin() {
        let meshes = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../openarm_description/assets/meshes"
        );
        let urdf = URDF.replace("package://d/meshes/arm1.stl", "link6_symp.stl");
        let u = UrdfCollisions::from_urdf(&urdf).expect("parse");
        let surface = u.collisions_of("arm1")[0]
            .surface(meshes)
            .expect("the mesh reads");
        let raw =
            crate::stl::load_stl(&format!("{meshes}/link6_symp.stl")).expect("the mesh reads");
        assert_eq!(surface.len(), raw.len());
        for (placed, v) in surface.iter().zip(&raw) {
            let expected = Point3::new(0.1 + v.x * 0.001, -v.y * 0.001, -0.2 + v.z * 0.001);
            assert!(
                (placed - expected).norm() < 1e-12,
                "{placed} is not {expected}"
            );
        }
    }

    /// A URDF of one link whose only collision element is `geometry`, at `xyz`.
    fn primitive(geometry: &str, xyz: &str) -> String {
        format!(
            r#"<robot name="t"><link name="part"><collision><origin xyz="{xyz}" rpy="0 0 0"/>
               <geometry>{geometry}</geometry></collision></link></robot>"#
        )
    }

    /// The surface of the only collision element of `urdf`.
    fn surface_of(urdf: &str) -> Vec<Point3<f64>> {
        UrdfCollisions::from_urdf(urdf)
            .expect("parse")
            .link_surfaces("part", "/nonexistent")
            .expect("a primitive needs no mesh")
            .concat()
    }

    /// The planes of the hull of `surface`.
    fn planes_of(surface: &[Point3<f64>]) -> Vec<(Vector3<f64>, f64)> {
        exact_hull(surface).expect("a solid surface").planes
    }

    /// The farthest `point` lies outside a plane of `planes`, negative inside them all.
    fn escape(planes: &[(Vector3<f64>, f64)], point: &Point3<f64>) -> f64 {
        planes
            .iter()
            .map(|(normal, offset)| normal.dot(&point.coords) - offset)
            .fold(f64::NEG_INFINITY, f64::max)
    }

    #[test]
    fn reads_box_cylinder_and_sphere_collisions() {
        let read = |geometry: &str| {
            UrdfCollisions::from_urdf(&primitive(geometry, "0 0 0"))
                .expect("parse")
                .collisions_of("part")[0]
                .geometry
                .clone()
        };
        assert_eq!(
            read(r#"<box size="0.1 0.2 0.3"/>"#),
            CollisionGeometry::Box {
                size: Vector3::new(0.1, 0.2, 0.3)
            }
        );
        assert_eq!(
            read(r#"<cylinder radius="0.05" length="0.2"/>"#),
            CollisionGeometry::Cylinder {
                radius: 0.05,
                length: 0.2
            }
        );
        assert_eq!(
            read(r#"<sphere radius="0.04"/>"#),
            CollisionGeometry::Sphere { radius: 0.04 }
        );
    }

    #[test]
    fn a_capsule_is_refused_naming_its_link() {
        let error = UrdfCollisions::from_urdf(&primitive(
            r#"<capsule radius="0.05" length="0.2"/>"#,
            "0 0 0",
        ))
        .err()
        .expect("a capsule is refused");
        assert!(
            matches!(&error, GeometryError::UnsupportedCollision { link, .. } if link == "part"),
            "{error}"
        );
    }

    #[test]
    fn a_primitive_without_a_positive_size_is_refused_naming_its_link() {
        for geometry in [
            r#"<box size="0.1 0 0.3"/>"#,
            r#"<cylinder radius="-0.05" length="0.2"/>"#,
            r#"<sphere radius="0"/>"#,
        ] {
            let error = UrdfCollisions::from_urdf(&primitive(geometry, "0 0 0"))
                .err()
                .unwrap_or_else(|| panic!("{geometry} is refused"));
            assert!(
                matches!(&error, GeometryError::UnsupportedCollision { link, .. } if link == "part"),
                "{geometry}: {error}"
            );
        }
    }

    /// A box's surface is the box, moved by its origin.
    #[test]
    fn a_box_surface_is_the_box_at_its_origin() {
        let surface = surface_of(&primitive(r#"<box size="0.1 0.2 0.3"/>"#, "1 2 3"));
        for v in &surface {
            let from_center = v - Point3::new(1.0, 2.0, 3.0);
            for (axis, half) in [0.05, 0.1, 0.15].into_iter().enumerate() {
                assert!(
                    (from_center[axis].abs() - half).abs() < 1e-12,
                    "{v} is no corner"
                );
            }
        }
    }

    /// A cylinder's surface holds every point of the cylinder, its rims included, and
    /// reaches no farther than its prism's corners.
    #[test]
    fn a_cylinder_surface_holds_the_cylinder() {
        let (radius, half_length) = (0.05, 0.1);
        let surface = surface_of(&primitive(
            r#"<cylinder radius="0.05" length="0.2"/>"#,
            "0 0 0",
        ));
        let planes = planes_of(&surface);
        for step in 0..1000 {
            let (sin, cos) = (step as f64 * 0.001 * std::f64::consts::TAU).sin_cos();
            for z in [-half_length, 0.0, half_length] {
                let rim = Point3::new(radius * cos, radius * sin, z);
                assert!(escape(&planes, &rim) <= 1e-12, "{rim} escapes");
            }
        }
        let corner = radius / (std::f64::consts::PI / CYLINDER_SIDES as f64).cos();
        for v in &surface {
            assert!(v.xy().coords.norm() <= corner + 1e-12 && v.z.abs() <= half_length + 1e-12);
        }
    }

    /// A sphere's surface holds every point of the sphere, and reaches less than 1%
    /// farther than its radius.
    #[test]
    fn a_sphere_surface_holds_the_sphere() {
        let radius = 0.04;
        let surface = surface_of(&primitive(r#"<sphere radius="0.04"/>"#, "0 0 0"));
        let planes = planes_of(&surface);
        for step in 0..2000 {
            let z = -1.0 + 2.0 * (step as f64 + 0.5) / 2000.0;
            let (sin, cos) = (step as f64 * 2.399963).sin_cos();
            let ring = (1.0 - z * z).sqrt();
            let on_sphere = Point3::new(ring * cos, ring * sin, z) * radius;
            assert!(escape(&planes, &on_sphere) <= 1e-12, "{on_sphere} escapes");
        }
        for v in &surface {
            assert!(
                v.coords.norm() < 1.01 * radius,
                "{v} reaches past 1% of the radius"
            );
        }
    }

    #[test]
    fn fixed_pose_composes_through_fixed_joints_only() {
        let u = UrdfCollisions::from_urdf(URDF).expect("parse");
        let m = u.fixed_pose_in_root("mount").expect("mount is fixed");
        assert!((m.translation.vector - Vector3::new(0.0, 0.1, 0.5)).norm() < 1e-12);
        // The +90 degree X rotation maps local z to world -y.
        let z = m.rotation * Vector3::z();
        assert!((z + Vector3::y()).norm() < 1e-9);
        assert!(u.fixed_pose_in_root("arm1").is_err());
        assert!(u.fixed_pose_in_root("finger").is_err());
    }

    #[test]
    fn parent_joint_reports_prismatic_finger() {
        let u = UrdfCollisions::from_urdf(URDF).expect("parse");
        let j = u.parent_joint("finger").expect("finger has parent");
        assert_eq!(j.parent_link, "arm1");
        assert!(!j.is_fixed());
        assert_eq!(j.axis, Vector3::new(0.0, -1.0, 0.0));
        assert!((j.upper_limit - 0.04).abs() < 1e-12);
    }
}
