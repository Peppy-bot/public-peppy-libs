//! URDF collision extraction for the offline fit pipeline: which mesh each
//! link's `<collision>` uses, with what origin and scale, plus the fixed-link
//! world poses and the prismatic finger transforms the fit composes.
//!
//! Runtime code never touches this module's output directly; it consumes the
//! generated capsule config.

use std::collections::HashMap;

use srs_model::nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

/// One `<collision><mesh>` entry of a link, with everything needed to map the
/// mesh's vertices into the link frame.
#[derive(Debug, Clone)]
pub struct CollisionMesh {
    pub link: String,
    /// Mesh file basename, e.g. `link3_symp.stl`; the caller resolves it
    /// against its own assets directory (the URDF's `package://` URI is not a
    /// filesystem path).
    pub mesh_basename: String,
    /// Componentwise scale applied to mesh vertices before the origin
    /// transform; components can be negative (mirrored geometry).
    pub scale: Vector3<f64>,
    /// Pose of the scaled mesh in the link frame.
    pub origin: Isometry3<f64>,
}

impl CollisionMesh {
    /// Map raw mesh vertices into the link frame: scale componentwise, then
    /// apply the collision origin.
    pub fn to_link_frame(&self, vertices: &[Point3<f64>]) -> Vec<Point3<f64>> {
        vertices
            .iter()
            .map(|v| self.origin * Point3::new(v.x * self.scale.x, v.y * self.scale.y, v.z * self.scale.z))
            .collect()
    }
}

/// Parsed URDF with the lookups the fit pipeline needs.
pub struct UrdfCollisions {
    collisions: Vec<CollisionMesh>,
    /// Child link name to its parent joint: (parent link, joint type, origin,
    /// axis, upper position limit).
    parent_joints: HashMap<String, ParentJoint>,
}

#[derive(Debug, Clone)]
pub struct ParentJoint {
    pub parent_link: String,
    pub is_fixed: bool,
    pub origin: Isometry3<f64>,
    pub axis: Vector3<f64>,
    pub lower_limit: f64,
    pub upper_limit: f64,
}

impl UrdfCollisions {
    pub fn from_urdf(urdf: &str) -> Result<Self, String> {
        let robot = urdf_rs::read_from_string(urdf).map_err(|e| format!("parse URDF: {e}"))?;

        let mut collisions = Vec::new();
        for link in &robot.links {
            for c in &link.collision {
                let urdf_rs::Geometry::Mesh { filename, scale } = &c.geometry else {
                    continue; // only mesh collisions exist in this pipeline
                };
                let basename = filename.rsplit('/').next().unwrap_or(filename).to_string();
                let s = scale.map(|s| Vector3::new(s[0], s[1], s[2])).unwrap_or_else(|| Vector3::repeat(1.0));
                collisions.push(CollisionMesh {
                    link: link.name.clone(),
                    mesh_basename: basename,
                    scale: s,
                    origin: pose_to_isometry(&c.origin),
                });
            }
        }

        let mut parent_joints = HashMap::new();
        for j in &robot.joints {
            let pj = ParentJoint {
                parent_link: j.parent.link.clone(),
                is_fixed: matches!(j.joint_type, urdf_rs::JointType::Fixed),
                origin: pose_to_isometry(&j.origin),
                axis: Vector3::new(j.axis.xyz[0], j.axis.xyz[1], j.axis.xyz[2]),
                lower_limit: j.limit.lower,
                upper_limit: j.limit.upper,
            };
            if parent_joints.insert(j.child.link.clone(), pj).is_some() {
                return Err(format!("link '{}' has two parent joints; URDF must be a tree", j.child.link));
            }
        }

        Ok(Self { collisions, parent_joints })
    }

    pub fn from_file(path: &str) -> Result<Self, String> {
        let urdf = std::fs::read_to_string(path).map_err(|e| format!("read urdf '{path}': {e}"))?;
        Self::from_urdf(&urdf)
    }

    /// All mesh collision entries of `link` (empty if it declares none).
    pub fn collisions_of(&self, link: &str) -> Vec<&CollisionMesh> {
        self.collisions.iter().filter(|c| c.link == link).collect()
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
        children.sort(); // HashMap order is not deterministic; the config is
        children
    }

    /// All collision-mesh vertices of `link`, in the link frame. Mesh files
    /// are resolved as `<meshes_dir>/<basename>`.
    pub fn link_vertices(&self, link: &str, meshes_dir: &str) -> Result<Vec<Point3<f64>>, String> {
        let entries = self.collisions_of(link);
        if entries.is_empty() {
            return Err(format!("link '{link}' has no mesh collision entries"));
        }
        let mut all = Vec::new();
        for c in entries {
            let raw = crate::stl::load_stl(&format!("{meshes_dir}/{}", c.mesh_basename))?;
            all.extend(c.to_link_frame(&raw));
        }
        Ok(all)
    }

    /// Collision vertices of a world-fixed `link`, mapped into the URDF root
    /// frame through its fixed mount chain.
    pub fn fixed_vertices_in_root(&self, link: &str, meshes_dir: &str) -> Result<Vec<Point3<f64>>, String> {
        let pose = self.fixed_pose_in_root(link)?;
        Ok(self.link_vertices(link, meshes_dir)?.into_iter().map(|v| pose * v).collect())
    }

    /// Collision vertices of `child` (a movable joint's child link, e.g. a
    /// prismatic finger) at joint position `q`, mapped into the parent link's
    /// frame.
    pub fn child_vertices_in_parent(&self, child: &str, q: f64, meshes_dir: &str) -> Result<Vec<Point3<f64>>, String> {
        let j = self
            .parent_joint(child)
            .ok_or_else(|| format!("link '{child}' has no parent joint"))?;
        let norm = j.axis.norm();
        if norm < 1e-12 {
            return Err(format!("link '{child}' parent joint has a zero axis"));
        }
        // URDF axes are conventionally unit but the spec does not require it.
        let pose = j.origin * Translation3::from(j.axis * (q / norm));
        Ok(self.link_vertices(child, meshes_dir)?.into_iter().map(|v| pose * v).collect())
    }

    /// Pose of `link` in the URDF root frame, composing only fixed joints.
    /// Errs if any joint on the path is movable (such a link has no constant
    /// root pose and belongs to an FK chain instead), or if the parent chain
    /// does not terminate (a malformed, cyclic URDF).
    pub fn fixed_pose_in_root(&self, link: &str) -> Result<Isometry3<f64>, String> {
        let mut pose = Isometry3::identity();
        let mut current = link.to_string();
        for _ in 0..=self.parent_joints.len() {
            let Some(j) = self.parent_joints.get(&current) else {
                return Ok(pose); // reached the root
            };
            if !j.is_fixed {
                return Err(format!("link '{link}' hangs below movable joint into '{current}', not world-fixed"));
            }
            pose = j.origin * pose;
            current = j.parent_link.clone();
        }
        Err(format!("link '{link}' has a cyclic parent chain in the URDF"))
    }
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
    fn extracts_mesh_scale_and_origin() {
        let u = UrdfCollisions::from_urdf(URDF).expect("parse");
        let arm = u.collisions_of("arm1");
        assert_eq!(arm.len(), 1);
        assert_eq!(arm[0].mesh_basename, "arm1.stl");
        assert_eq!(arm[0].scale, Vector3::new(0.001, -0.001, 0.001));
        assert!((arm[0].origin.translation.vector - Vector3::new(0.1, 0.0, -0.2)).norm() < 1e-12);
        assert!(u.collisions_of("mount").is_empty());
    }

    #[test]
    fn to_link_frame_applies_scale_then_origin() {
        let u = UrdfCollisions::from_urdf(URDF).expect("parse");
        let arm = u.collisions_of("arm1")[0].clone();
        let out = arm.to_link_frame(&[Point3::new(1000.0, 1000.0, 0.0)]);
        // Scale gives (1, -1, 0), origin shifts to (1.1, -1.0, -0.2).
        assert!((out[0] - Point3::new(1.1, -1.0, -0.2)).norm() < 1e-12);
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
        assert!(!j.is_fixed);
        assert_eq!(j.axis, Vector3::new(0.0, -1.0, 0.0));
        assert!((j.upper_limit - 0.04).abs() < 1e-12);
    }
}
