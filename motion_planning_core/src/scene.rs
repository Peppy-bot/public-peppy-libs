//! The planning scene besides the map, as MoveIt's planning scene holds it: the collision
//! objects other nodes add, the objects attached to an arm's grasp link that move with
//! it, and the pairs of names allowed to touch.
//!
//! [`PlanningScene`] holds them and decides every rule of motion_planning's scene
//! members: a batch of objects applied all or none, an attach and a detach, the allowed
//! pairs, and the snapshot get_planning_scene reports. It clears the map where an object
//! is added, moved or attached. Each shape of an object attached to a grasp link rides
//! it, a [`Rider`] of the arm. A plan of one or more arms meets the scene as
//! [`PlanObjects`]: the objects the arms carry and the [`Obstacle`]s, which stand still
//! through the plan; a depth frame's intake cuts out the objects where they stand and the
//! riders of each arm.

use std::collections::{BTreeMap, BTreeSet};

use collision_geometry::chain::{ChainGeometry, Rider, RiderBody};
use collision_geometry::gjk::Placed;
use collision_geometry::shapes::{PlacedShape, Shape, ShapeKind};
use nalgebra::{Isometry3, Point3};
use occupancy_map::VoxelMap;

use crate::arm::grasp_frame;
use crate::pose::{pose_from_parts, pose_parts};

/// The name a pair gives the voxel map.
pub const OCCUPANCY_MAP: &str = "occupancy_map";

/// The operations of apply_collision_objects and of apply_attached_collision_object, as
/// a refusal lists them.
const OBJECT_OPERATIONS: &str = "\"add\", \"move\" or \"remove\"";
const ATTACH_OPERATIONS: &str = "\"attach\" or \"detach\"";

/// A shape as motion_planning writes it: its kind, "box", "sphere", "cylinder" or
/// "mesh"; a primitive's dimensions, a box's sides, a sphere's radius, a cylinder's
/// height then its radius; a mesh's vertices, three coordinates each, and its triangles,
/// three indices each; and its pose in the object's frame.
#[derive(Debug, Clone, PartialEq)]
pub struct ShapeFields {
    pub kind: String,
    pub dimensions: Vec<f64>,
    pub mesh_vertices: Vec<f64>,
    pub mesh_triangles: Vec<u32>,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// A shape of an object, posed in the object's frame.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ObjectShape {
    pub shape: Shape,
    pub pose: Isometry3<f64>,
}

impl TryFrom<&ShapeFields> for ObjectShape {
    type Error = String;

    /// The shape the fields describe; the message says what to send instead.
    fn try_from(fields: &ShapeFields) -> Result<Self, String> {
        let dims = fields.dimensions.as_slice();
        let primitive = |names: &str, count: usize| {
            if dims.len() != count {
                return Err(format!(
                    "a {} has {count} dimensions, {names}; got {}",
                    fields.kind,
                    dims.len()
                ));
            }
            if !fields.mesh_vertices.is_empty() || !fields.mesh_triangles.is_empty() {
                return Err(format!(
                    "a {} has no mesh_vertices or mesh_triangles",
                    fields.kind
                ));
            }
            Ok(())
        };
        let shape = match fields.kind.as_str() {
            "box" => {
                primitive("its sides x, y and z", 3)?;
                Shape::cuboid([dims[0], dims[1], dims[2]])
            }
            "sphere" => {
                primitive("its radius", 1)?;
                Shape::sphere(dims[0])
            }
            "cylinder" => {
                primitive("its height then its radius", 2)?;
                Shape::cylinder(dims[1], dims[0])
            }
            "mesh" => {
                if !dims.is_empty() {
                    return Err("a mesh has no dimensions".to_string());
                }
                if !fields.mesh_vertices.len().is_multiple_of(3)
                    || !fields.mesh_triangles.len().is_multiple_of(3)
                {
                    return Err(format!(
                        "a mesh has three coordinates per vertex and three indices per \
                         triangle; got {} coordinates and {} indices",
                        fields.mesh_vertices.len(),
                        fields.mesh_triangles.len()
                    ));
                }
                Shape::mesh(
                    fields
                        .mesh_vertices
                        .chunks_exact(3)
                        .map(|v| Point3::new(v[0], v[1], v[2]))
                        .collect(),
                    fields
                        .mesh_triangles
                        .chunks_exact(3)
                        .map(|t| [t[0], t[1], t[2]])
                        .collect(),
                )
            }
            kind => {
                return Err(format!(
                    "kind is {kind:?}; a shape is \"box\", \"sphere\", \"cylinder\" or \"mesh\""
                ));
            }
        }
        .map_err(|e| e.to_string())?;
        Ok(ObjectShape {
            shape,
            pose: pose_from_parts(fields.position, fields.orientation)?,
        })
    }
}

impl From<&ObjectShape> for ShapeFields {
    fn from(shape: &ObjectShape) -> Self {
        let (position, orientation) = pose_parts(&shape.pose);
        let (kind, dimensions, mesh_vertices, mesh_triangles) = match shape.shape.kind() {
            ShapeKind::Box { size } => ("box", size.to_vec(), Vec::new(), Vec::new()),
            ShapeKind::Sphere { radius } => ("sphere", vec![radius], Vec::new(), Vec::new()),
            ShapeKind::Cylinder { radius, height } => {
                ("cylinder", vec![height, radius], Vec::new(), Vec::new())
            }
            ShapeKind::Mesh {
                vertices,
                triangles,
            } => (
                "mesh",
                Vec::new(),
                vertices.iter().flat_map(|v| [v.x, v.y, v.z]).collect(),
                triangles.iter().flatten().copied().collect(),
            ),
        };
        ShapeFields {
            kind: kind.to_string(),
            dimensions,
            mesh_vertices,
            mesh_triangles,
            position,
            orientation,
        }
    }
}

/// An object as motion_planning writes it: its id, its pose in the robot frame, or in
/// the frame of the link that carries it, and its shapes.
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectFields {
    pub id: String,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub shapes: Vec<ShapeFields>,
}

impl ObjectFields {
    fn of(id: &str, pose: &Isometry3<f64>, shapes: &[ObjectShape]) -> Self {
        let (position, orientation) = pose_parts(pose);
        ObjectFields {
            id: id.to_string(),
            position,
            orientation,
            shapes: shapes.iter().map(ShapeFields::from).collect(),
        }
    }
}

/// One entry of apply_collision_objects: its operation, "add", "move" or "remove", and
/// its object.
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectRequest {
    pub operation: String,
    pub object: ObjectFields,
}

/// apply_attached_collision_object's request.
#[derive(Debug, Clone, PartialEq)]
pub struct AttachRequest {
    pub object_id: String,
    pub link_name: String,
    pub touch_links: Vec<String>,
    /// "attach" or "detach".
    pub operation: String,
}

/// An attached object as get_planning_scene reports it: the link that carries it, the
/// links it may touch, and the object, posed in that link's frame.
#[derive(Debug, Clone, PartialEq)]
pub struct AttachedObjectFields {
    pub object: ObjectFields,
    pub link_name: String,
    pub touch_links: Vec<String>,
}

/// What get_planning_scene reports of the scene: the objects no link carries and the
/// attached objects, each in the order of its id, the pairs allowed to touch, each once,
/// its names in order, the clearance each state keeps, and the side of the map's voxels.
#[derive(Debug, Clone, PartialEq)]
pub struct SceneSnapshot {
    pub objects: Vec<ObjectFields>,
    pub attached_objects: Vec<AttachedObjectFields>,
    pub allowed_pairs: Vec<[String; 2]>,
    /// The clearance (m) each state keeps from the map, the objects and the robot's own
    /// body: the plans' `min_clearance_m`.
    pub clearance_m: f64,
    /// The side of the map's voxels (m).
    pub voxel_size_m: f64,
}

/// Why the scene refuses a request; each but `Unplaced` refuses it with `rejected`, and
/// the message says what to send instead.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SceneError {
    #[error("objects[{index}] ({id:?}): {error}; the call applied none of its objects")]
    Batch {
        index: usize,
        id: String,
        error: Box<SceneError>,
    },
    #[error("an object's id is empty; give each object a name, and that name to move or remove it")]
    EmptyId,
    #[error("{0:?} is a link of the robot; an object's id is a name no link has")]
    LinkId(String),
    #[error("\"occupancy_map\" names the voxel map; an object's id is another name")]
    MapId,
    #[error("no object of the scene is named {0:?}; get_planning_scene lists the objects")]
    NoObject(String),
    #[error("{id:?} is attached to {link}; detach it first")]
    Attached { id: String, link: String },
    #[error("{0:?} is attached to no link; detach names an attached object")]
    NotAttached(String),
    #[error(
        "{link} is no arm's grasp_link; an object is attached to one of {}",
        .grasp_links.join(", ")
    )]
    NotGraspLink {
        link: String,
        grasp_links: Vec<String>,
    },
    #[error("{0:?} is no link of the robot; describe_arms names the links")]
    NoLink(String),
    #[error(
        "{0:?} is no link of the robot, no object of the scene and not \"occupancy_map\"; \
         describe_arms names the links and get_planning_scene the objects"
    )]
    NoName(String),
    #[error("{0:?} is paired with itself; other_names names links and objects other than it")]
    SelfPair(String),
    #[error("operation is {given:?}; it is {expected}")]
    Operation {
        given: String,
        expected: &'static str,
    },
    #[error("an object has at least one shape")]
    NoShape,
    #[error("shapes[{index}]: {error}")]
    Shape { index: usize, error: String },
    #[error("{0}")]
    Pose(String),
    #[error("the pose of {0} is not known yet")]
    Unplaced(String),
}

/// The pairs of names allowed to touch, each pair unordered: each name with the names it
/// may touch.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct AllowedPairs(BTreeMap<String, BTreeSet<String>>);

impl AllowedPairs {
    /// No pair allowed.
    const NONE: AllowedPairs = AllowedPairs(BTreeMap::new());

    /// Whether `a` and `b` may touch.
    pub(crate) fn allows(&self, a: &str, b: &str) -> bool {
        self.0.get(a).is_some_and(|names| names.contains(b))
    }

    /// Every allowed pair once, its names in order, in the order of the names.
    fn pairs(&self) -> impl Iterator<Item = (&str, &str)> {
        self.0.iter().flat_map(|(a, names)| {
            names
                .iter()
                .filter(move |b| a < *b)
                .map(move |b| (a.as_str(), b.as_str()))
        })
    }

    /// Lets `a` and `b` touch, or stops them.
    fn set(&mut self, a: &str, b: &str, allowed: bool) {
        for (one, other) in [(a, b), (b, a)] {
            if allowed {
                self.0
                    .entry(one.to_string())
                    .or_default()
                    .insert(other.to_string());
            } else if let Some(names) = self.0.get_mut(one) {
                names.remove(other);
                if names.is_empty() {
                    self.0.remove(one);
                }
            }
        }
    }

    /// Drops every pair that names `name`.
    fn drop_naming(&mut self, name: &str) {
        let partners = self.0.remove(name).unwrap_or_default();
        for partner in partners {
            self.set(&partner, name, false);
        }
    }
}

/// The links of the robot that a scene's requests name: every link, and each arm's
/// grasp link, the one link of an arm that carries an object.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotLinks {
    links: BTreeSet<String>,
    grasp_links: BTreeSet<String>,
}

impl RobotLinks {
    /// The robot of `links`, `grasp_links` among them.
    ///
    /// # Panics
    ///
    /// When a grasp link is not one of `links`.
    pub fn new(
        links: impl IntoIterator<Item = String>,
        grasp_links: impl IntoIterator<Item = String>,
    ) -> Self {
        let links: BTreeSet<String> = links.into_iter().collect();
        let grasp_links: BTreeSet<String> = grasp_links.into_iter().collect();
        assert!(
            grasp_links.is_subset(&links),
            "each grasp link {grasp_links:?} is a link of the robot"
        );
        Self { links, grasp_links }
    }
}

/// An object no link carries: its pose in the robot frame and its shapes.
#[derive(Debug, Clone, PartialEq)]
struct WorldObject {
    pose: Isometry3<f64>,
    shapes: Vec<ObjectShape>,
}

/// An object attached to a grasp link: its pose in the link's frame, its shapes, and the
/// links it may touch besides the one that carries it.
#[derive(Debug, Clone, PartialEq)]
struct Attachment {
    link: String,
    touch_links: BTreeSet<String>,
    pose: Isometry3<f64>,
    shapes: Vec<ObjectShape>,
}

/// One operation of apply_collision_objects, parsed: it holds only the fields its
/// operation reads.
enum ObjectOperation {
    Add {
        pose: Isometry3<f64>,
        shapes: Vec<ObjectShape>,
    },
    Move {
        pose: Isometry3<f64>,
    },
    Remove,
}

impl ObjectOperation {
    fn parse(request: &ObjectRequest) -> Result<Self, SceneError> {
        let object = &request.object;
        let pose =
            || pose_from_parts(object.position, object.orientation).map_err(SceneError::Pose);
        match request.operation.as_str() {
            "add" => {
                if object.shapes.is_empty() {
                    return Err(SceneError::NoShape);
                }
                let shapes = object
                    .shapes
                    .iter()
                    .enumerate()
                    .map(|(index, fields)| {
                        ObjectShape::try_from(fields)
                            .map_err(|error| SceneError::Shape { index, error })
                    })
                    .collect::<Result<_, _>>()?;
                Ok(Self::Add {
                    pose: pose()?,
                    shapes,
                })
            }
            "move" => Ok(Self::Move { pose: pose()? }),
            "remove" => Ok(Self::Remove),
            given => Err(SceneError::Operation {
                given: given.to_string(),
                expected: OBJECT_OPERATIONS,
            }),
        }
    }
}

/// The collision objects, the attached objects and the allowed pairs of a robot's
/// planning scene.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanningScene {
    robot: RobotLinks,
    objects: BTreeMap<String, WorldObject>,
    attached: BTreeMap<String, Attachment>,
    allowed: AllowedPairs,
}

impl PlanningScene {
    /// The empty scene of the robot of `robot`.
    pub fn new(robot: RobotLinks) -> Self {
        Self {
            robot,
            objects: BTreeMap::new(),
            attached: BTreeMap::new(),
            allowed: AllowedPairs::NONE,
        }
    }

    /// Applies `requests` in order, all or none: "add" places an object with its shapes,
    /// in place of an object of the same id, whose allowed pairs it keeps; "move" places
    /// an object at a new pose; "remove" removes it and drops every pair that names it.
    /// An operation reads only its own fields. Once every request applies, each object
    /// an add or a move places clears from `map` every voxel it intersects where the
    /// batch leaves it; an object a later request of the batch removes clears nothing.
    /// Refused: an empty id, an id that names a link or the voxel map, an unknown id for
    /// "move" or "remove", an attached object, and a shape that cannot be built, an open
    /// mesh among them.
    pub fn apply_collision_objects(
        &mut self,
        requests: &[ObjectRequest],
        map: &mut VoxelMap,
    ) -> Result<(), SceneError> {
        let mut next = self.clone();
        let mut placed: BTreeSet<&str> = BTreeSet::new();
        for (index, request) in requests.iter().enumerate() {
            let id = &request.object.id;
            let at = next.apply(request).map_err(|error| SceneError::Batch {
                index,
                id: id.clone(),
                error: Box::new(error),
            })?;
            placed.extend(at);
        }
        *self = next;
        for object in placed.iter().filter_map(|id| self.objects.get(*id)) {
            clear(map, &object.pose, &object.shapes);
        }
        Ok(())
    }

    /// Applies one entry of apply_collision_objects; the id of the object it places, an
    /// add's or a move's.
    fn apply<'r>(&mut self, request: &'r ObjectRequest) -> Result<Option<&'r str>, SceneError> {
        let id = request.object.id.as_str();
        if id.is_empty() {
            return Err(SceneError::EmptyId);
        }
        self.refuse_attached(id)?;
        match ObjectOperation::parse(request)? {
            ObjectOperation::Add { pose, shapes } => {
                if self.robot.links.contains(id) {
                    return Err(SceneError::LinkId(id.to_string()));
                }
                if id == OCCUPANCY_MAP {
                    return Err(SceneError::MapId);
                }
                self.objects
                    .insert(id.to_string(), WorldObject { pose, shapes });
                Ok(Some(id))
            }
            ObjectOperation::Move { pose } => {
                self.object_mut(id)?.pose = pose;
                Ok(Some(id))
            }
            ObjectOperation::Remove => {
                self.objects
                    .remove(id)
                    .ok_or_else(|| SceneError::NoObject(id.to_string()))?;
                self.allowed.drop_naming(id);
                Ok(None)
            }
        }
    }

    /// Applies apply_attached_collision_object's `request`. An attach puts an object of
    /// the scene that no link carries on an arm's grasp link, where it stands, the link
    /// at the pose `link_pose` gives it in the robot frame; it may touch that link and
    /// `touch_links`, and every voxel of `map` it intersects is cleared. A detach, which
    /// reads only the object's id, leaves the object in the scene where it stands, its
    /// link at the pose `link_pose` gives. Its allowed pairs stay through both.
    /// Refused: an attach of an attached object, of an unknown one, to a link that is no
    /// grasp link, or with a touch link the robot does not have; a detach of an object
    /// no link carries; and, with `Unplaced`, a request whose link `link_pose` does not
    /// place.
    pub fn apply_attached_collision_object(
        &mut self,
        request: &AttachRequest,
        link_pose: impl FnOnce(&str) -> Option<Isometry3<f64>>,
        map: &mut VoxelMap,
    ) -> Result<(), SceneError> {
        let id = request.object_id.as_str();
        match request.operation.as_str() {
            "attach" => {
                self.refuse_attached(id)?;
                let pose = self.object(id)?.pose;
                let link = request.link_name.as_str();
                if !self.robot.grasp_links.contains(link) {
                    return Err(SceneError::NotGraspLink {
                        link: link.to_string(),
                        grasp_links: self.robot.grasp_links.iter().cloned().collect(),
                    });
                }
                if let Some(unknown) = request
                    .touch_links
                    .iter()
                    .find(|name| !self.robot.links.contains(*name))
                {
                    return Err(SceneError::NoLink(unknown.clone()));
                }
                let at = link_pose(link).ok_or_else(|| SceneError::Unplaced(link.to_string()))?;
                let object = self
                    .objects
                    .remove(id)
                    .expect("the object the attach found");
                clear(map, &pose, &object.shapes);
                self.attached.insert(
                    id.to_string(),
                    Attachment {
                        link: link.to_string(),
                        touch_links: request.touch_links.iter().cloned().collect(),
                        pose: at.inverse() * pose,
                        shapes: object.shapes,
                    },
                );
                Ok(())
            }
            "detach" => {
                let link = match self.attached.get(id) {
                    Some(attachment) => attachment.link.clone(),
                    None if self.objects.contains_key(id) => {
                        return Err(SceneError::NotAttached(id.to_string()));
                    }
                    None => return Err(SceneError::NoObject(id.to_string())),
                };
                let at = link_pose(&link).ok_or(SceneError::Unplaced(link))?;
                let attachment = self
                    .attached
                    .remove(id)
                    .expect("the attachment the detach found");
                self.objects.insert(
                    id.to_string(),
                    WorldObject {
                        pose: at * attachment.pose,
                        shapes: attachment.shapes,
                    },
                );
                Ok(())
            }
            given => Err(SceneError::Operation {
                given: given.to_string(),
                expected: ATTACH_OPERATIONS,
            }),
        }
    }

    /// Lets the object `name`, attached or not, touch each of `other_names`, or stops it:
    /// links, objects, or "occupancy_map" for the voxel map; all or none. Refused: a
    /// `name` that is no object of the scene, another name the robot and the scene do
    /// not have, and `name` among `other_names`.
    pub fn set_allowed_collision(
        &mut self,
        name: &str,
        other_names: &[String],
        allowed: bool,
    ) -> Result<(), SceneError> {
        if !self.holds(name) {
            return Err(SceneError::NoObject(name.to_string()));
        }
        if let Some(other) = other_names.iter().find(|other| !self.names(other)) {
            return Err(SceneError::NoName(other.clone()));
        }
        if other_names.iter().any(|other| other == name) {
            return Err(SceneError::SelfPair(name.to_string()));
        }
        for other in other_names {
            self.allowed.set(name, other, allowed);
        }
        Ok(())
    }

    /// Everything the scene holds, as get_planning_scene reports it, with the clearance
    /// `clearance_m` that each state of a plan keeps and the side `voxel_size_m` of the
    /// map's voxels.
    pub fn snapshot(&self, clearance_m: f64, voxel_size_m: f64) -> SceneSnapshot {
        SceneSnapshot {
            clearance_m,
            voxel_size_m,
            objects: self
                .objects
                .iter()
                .map(|(id, object)| ObjectFields::of(id, &object.pose, &object.shapes))
                .collect(),
            attached_objects: self
                .attached
                .iter()
                .map(|(id, attachment)| AttachedObjectFields {
                    object: ObjectFields::of(id, &attachment.pose, &attachment.shapes),
                    link_name: attachment.link.clone(),
                    touch_links: attachment.touch_links.iter().cloned().collect(),
                })
                .collect(),
            allowed_pairs: self
                .allowed
                .pairs()
                .map(|(a, b)| [a.to_string(), b.to_string()])
                .collect(),
        }
    }

    /// The scene as a plan of the arms whose grasp links are `grasp_links` meets it: the
    /// objects those links carry, and the obstacles, each object no link carries where it
    /// stands and each object another grasp link carries where `grasp_pose` puts that
    /// link in the robot frame.
    pub fn for_arms(
        &self,
        grasp_links: &[&str],
        grasp_pose: impl Fn(&str) -> Isometry3<f64>,
    ) -> PlanObjects<'_> {
        let moved = |attachment: &Attachment| grasp_links.contains(&attachment.link.as_str());
        let carried = self
            .attached
            .iter()
            .filter(|(_, attachment)| moved(attachment))
            .map(|(id, attachment)| Carried { id, attachment })
            .collect();
        let held = self
            .attached
            .iter()
            .filter(|(_, attachment)| !moved(attachment))
            .map(|(id, attachment)| Obstacle {
                id,
                shapes: placed(
                    &(grasp_pose(&attachment.link) * attachment.pose),
                    &attachment.shapes,
                )
                .collect(),
                holder: Some(attachment),
            });
        PlanObjects {
            carried,
            obstacles: self.standing_obstacles().chain(held).collect(),
            allowed: &self.allowed,
        }
    }

    /// The shapes of the objects `grasp_link` carries, each a rider of the child link of
    /// the last joint of `geometry`'s chain, whose grasp point, the chain's end effector,
    /// is that link: how a plan carries the objects an arm holds and a depth frame's
    /// intake cuts them out.
    pub fn riders<'s, const N: usize>(
        &'s self,
        grasp_link: &str,
        geometry: &ChainGeometry<N>,
    ) -> Vec<Rider<'s>> {
        self.attached
            .iter()
            .filter(|(_, attachment)| attachment.link == grasp_link)
            .flat_map(|(id, attachment)| riders_of(id, attachment, geometry))
            .collect()
    }

    /// Each shape of each object no link carries, where it stands in the robot frame.
    pub fn standing_shapes(&self) -> impl Iterator<Item = PlacedShape<'_>> {
        self.objects
            .values()
            .flat_map(|object| placed(&object.pose, &object.shapes))
    }

    /// Each object no link carries, as an obstacle where it stands.
    fn standing_obstacles(&self) -> impl Iterator<Item = Obstacle<'_>> {
        self.objects.iter().map(|(id, object)| Obstacle {
            id,
            shapes: placed(&object.pose, &object.shapes).collect(),
            holder: None,
        })
    }

    /// Whether the scene holds an object `id`, attached or not.
    fn holds(&self, id: &str) -> bool {
        self.objects.contains_key(id) || self.attached.contains_key(id)
    }

    /// Whether a pair may name `name`: a link, an object, or the voxel map.
    fn names(&self, name: &str) -> bool {
        self.robot.links.contains(name) || self.holds(name) || name == OCCUPANCY_MAP
    }

    /// The object `id` that no link carries.
    fn object(&self, id: &str) -> Result<&WorldObject, SceneError> {
        self.objects
            .get(id)
            .ok_or_else(|| SceneError::NoObject(id.to_string()))
    }

    fn object_mut(&mut self, id: &str) -> Result<&mut WorldObject, SceneError> {
        self.objects
            .get_mut(id)
            .ok_or_else(|| SceneError::NoObject(id.to_string()))
    }

    fn refuse_attached(&self, id: &str) -> Result<(), SceneError> {
        match self.attached.get(id) {
            Some(attachment) => Err(SceneError::Attached {
                id: id.to_string(),
                link: attachment.link.clone(),
            }),
            None => Ok(()),
        }
    }
}

/// `shapes`, each placed by `pose` in the frame `pose` is in.
fn placed<'s>(
    pose: &Isometry3<f64>,
    shapes: &'s [ObjectShape],
) -> impl Iterator<Item = PlacedShape<'s>> + use<'s> {
    let pose = *pose;
    shapes
        .iter()
        .map(move |s| PlacedShape::new(&s.shape, pose * s.pose))
}

/// Clears from `map` every voxel that `shapes`, placed by `pose`, intersect.
fn clear(map: &mut VoxelMap, pose: &Isometry3<f64>, shapes: &[ObjectShape]) {
    for shape in placed(pose, shapes) {
        map.clear_intersecting(&shape);
    }
}

/// Each shape of the object `id` that `attachment` puts on a grasp link, as a rider of
/// the child link of the last joint of `geometry`'s chain, whose end effector is that
/// grasp link.
fn riders_of<'s, const N: usize>(
    id: &'s str,
    attachment: &'s Attachment,
    geometry: &ChainGeometry<N>,
) -> impl Iterator<Item = Rider<'s>> + use<'s, N> {
    let offset = grasp_frame(geometry.chain()) * attachment.pose;
    attachment.shapes.iter().map(move |s| Rider {
        name: id,
        segment: N - 1,
        offset: offset * s.pose,
        body: RiderBody::Shape(&s.shape),
    })
}

/// The scene as a plan of one or more arms meets it: the objects the arms carry, the
/// obstacles, which stand still through the plan, and the pairs allowed to touch.
pub struct PlanObjects<'a> {
    carried: Vec<Carried<'a>>,
    obstacles: Vec<Obstacle<'a>>,
    allowed: &'a AllowedPairs,
}

impl<'a> PlanObjects<'a> {
    /// The obstacles, which a plan of the arms keeps clear of.
    pub fn obstacles(&self) -> &[Obstacle<'a>] {
        &self.obstacles
    }

    /// What a plan's scene keeps clear of besides the map, with `parts`, the pieces of
    /// the robot the plan does not move.
    pub fn objects<'b>(&'b self, parts: &'b [RobotPart<'b>]) -> Objects<'b> {
        Objects {
            obstacles: &self.obstacles,
            parts,
            carried: &self.carried,
            allowed: self.allowed,
        }
    }
}

/// An object as a plan meets it: its shapes placed in the robot frame, where it stands
/// or where the grasp link of an arm the plan does not move holds it, and that arm's
/// attachment.
pub struct Obstacle<'a> {
    id: &'a str,
    shapes: Vec<PlacedShape<'a>>,
    holder: Option<&'a Attachment>,
}

impl<'a> Obstacle<'a> {
    pub fn id(&self) -> &'a str {
        self.id
    }

    pub fn shapes(&self) -> &[PlacedShape<'a>] {
        &self.shapes
    }

    /// Whether the object, held by an arm the plan does not move, may touch the link
    /// `link`: the link that holds it, or a touch link.
    pub(crate) fn may_touch(&self, link: &str) -> bool {
        self.holder
            .is_some_and(|attachment| attachment.may_touch(link))
    }
}

/// An object attached to the grasp link of an arm a plan moves, by its id.
pub(crate) struct Carried<'a> {
    id: &'a str,
    attachment: &'a Attachment,
}

impl Attachment {
    /// Whether the object may touch the link `link`: the link that carries it, or a touch
    /// link.
    fn may_touch(&self, link: &str) -> bool {
        link == self.link || self.touch_links.contains(link)
    }
}

/// A piece of the robot that a plan does not move, by the name of its link: a piece of
/// an arm the plan does not move where it stands, or of the body.
pub struct RobotPart<'a> {
    pub link: &'a str,
    pub piece: Placed<'a>,
}

/// What a plan keeps clear of besides the map: the obstacles, the parts of the robot it
/// does not move, the objects the arms carry, and the pairs allowed to touch.
#[derive(Clone, Copy)]
pub struct Objects<'a> {
    pub(crate) obstacles: &'a [Obstacle<'a>],
    pub(crate) parts: &'a [RobotPart<'a>],
    carried: &'a [Carried<'a>],
    allowed: &'a AllowedPairs,
}

/// No pair allowed to touch.
static NO_PAIRS: AllowedPairs = AllowedPairs::NONE;

impl Objects<'static> {
    /// No object: a scene of the map alone.
    pub fn none() -> Self {
        Self {
            obstacles: &[],
            parts: &[],
            carried: &[],
            allowed: &NO_PAIRS,
        }
    }
}

impl Objects<'_> {
    /// Whether a plan never checks the pair of `a`, a body of an arm by its link's name
    /// or its object's id, and `b`, a link, an object or the map: a pair allowed to
    /// touch, or an object an arm carries and its grasp link or a touch link.
    pub(crate) fn exempt(&self, a: &str, b: &str) -> bool {
        self.allowed.allows(a, b)
            || self
                .carried
                .iter()
                .any(|carried| carried.id == a && carried.attachment.may_touch(b))
    }
}
