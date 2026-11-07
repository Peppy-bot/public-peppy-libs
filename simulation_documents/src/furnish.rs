//! Furnish poses: the objects of a world pattern that the simulation lays
//! out together in a room, by rules, in one call of its solver.
//!
//! The pose of an entry of `objects` may be `{ furnish: <rule>, stations:
//! [<station>, ...] }`, and the `scale` of such an object a number or
//! `{ min, max }`. The rule says where the piece stands in the room
//! ([`FurnishRule`]), and each station is the room that a robot needs to
//! work a work surface of the piece from one of its sides
//! ([`FurnishStation`]). The expansion of a pattern gives the simulation
//! every such piece in one placement, the furnish layout
//! ([`crate::PlacementKind::Furnish`]), at the path `furnish`. Its placed
//! value holds the place of each piece: `{ "<object>": { position, yaw,
//! scale }, ... }`.
//!
//! The root of a pattern may list `stations`, its fixed stations
//! ([`FixedStation`]): the room of a station at a work surface of an object
//! with a fixed pose or of a scene object, such as a belt of a conveyor
//! line, which the layout keeps free.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Deserializer, Serialize};
use serde_json::{Map, Value, json};

use crate::geometry::Point;
use crate::names::first_named_twice;
use crate::pattern::{ObjectEntry, physics_candidates};
use crate::sampler::{DrawnValue, Sampled, Segment, SitePath, SiteRule, placeholder};
use crate::world::{Object, Robot, SceneObject, positive_scale};
use crate::{Error, Name, Offset, Physics, Pose, RobotName, Side, Stand};

/// The path of the placed value of the furnish layout.
const FURNISH: &str = "furnish";

/// The path of the furnish layout among the placements: `furnish`.
pub(crate) fn furnish_path() -> SitePath {
    SitePath::new(vec![Segment::Key(FURNISH.to_owned())])
}

/// Where the solver puts a piece in the room. Written as an object with one
/// key that names the rule: `{ against_wall: { on_wall } }` (`on_wall`
/// optional), `{ in_corner: {} }`, `{ free: { near } }` (`near` optional),
/// `{ beside: { object, gap_m } }` or `{ facing: { object, distance_m } }`.
#[derive(Clone, Debug, PartialEq)]
pub enum FurnishRule {
    /// The back of the piece, the side opposite its front, against a wall:
    /// the wall whose id `on_wall` gives, or any wall.
    AgainstWall { on_wall: Option<Name> },
    /// The back of the piece against one wall and one of its sides against
    /// another wall that meets it.
    InCorner,
    /// Walkway room on every side of the piece, nearest the point `near`
    /// (x and y in the world frame, metres) when it is given.
    Free { near: Option<[f64; 2]> },
    /// Against the same wall as `object`, a piece whose rule is
    /// `against_wall` or `in_corner`, side by side with it on either side at
    /// a gap of `gap_m` metres, at least 0.
    Beside { object: Name, gap_m: f64 },
    /// Front to front with `object`, `distance_m` metres between the two
    /// fronts, above 0.
    Facing { object: Name, distance_m: f64 },
}

/// The keys that name a furnish rule, each the key of one variant of
/// [`FurnishRule`].
const RULES: [&str; 5] = ["against_wall", "in_corner", "free", "beside", "facing"];

impl FurnishRule {
    /// The key that writes the rule.
    pub fn key(&self) -> &'static str {
        match self {
            FurnishRule::AgainstWall { .. } => "against_wall",
            FurnishRule::InCorner => "in_corner",
            FurnishRule::Free { .. } => "free",
            FurnishRule::Beside { .. } => "beside",
            FurnishRule::Facing { .. } => "facing",
        }
    }

    /// The piece that the rule places this piece by: the object of `beside`
    /// and of `facing`.
    pub fn object(&self) -> Option<&Name> {
        match self {
            FurnishRule::Beside { object, .. } | FurnishRule::Facing { object, .. } => Some(object),
            FurnishRule::AgainstWall { .. } | FurnishRule::InCorner | FurnishRule::Free { .. } => {
                None
            }
        }
    }

    /// Whether the rule puts the back of the piece against a wall that a
    /// piece `beside` it can stand against too.
    fn is_against_a_wall(&self) -> bool {
        matches!(
            self,
            FurnishRule::AgainstWall { .. } | FurnishRule::InCorner
        )
    }
}

impl<'de> Deserialize<'de> for FurnishRule {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<FurnishRule, D::Error> {
        let value = Value::deserialize(deserializer)?;
        let names_one_rule = value.as_object().is_some_and(|fields| {
            fields.len() == 1 && fields.keys().all(|key| RULES.contains(&key.as_str()))
        });
        if !names_one_rule {
            let rules = RULES.map(|rule| format!("`{rule}`")).join(", ");
            return Err(serde::de::Error::custom(format!(
                "a furnish rule is an object with one key naming a rule: {rules}"
            )));
        }
        let fields: RuleFields =
            serde_path_to_error::deserialize(value).map_err(serde::de::Error::custom)?;
        FurnishRule::try_from(fields).map_err(serde::de::Error::custom)
    }
}

/// A furnish rule as written, its numbers unchecked.
#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum RuleFields {
    AgainstWall(AgainstWallFields),
    InCorner(InCornerFields),
    Free(FreeFields),
    Beside(BesideFields),
    Facing(FacingFields),
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, expecting = "an against_wall, { on_wall }")]
struct AgainstWallFields {
    on_wall: Option<Name>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, expecting = "an in_corner, {}")]
struct InCornerFields {}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, expecting = "a free, { near }")]
struct FreeFields {
    near: Option<[f64; 2]>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, expecting = "a beside, { object, gap_m }")]
struct BesideFields {
    object: Name,
    gap_m: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, expecting = "a facing, { object, distance_m }")]
struct FacingFields {
    object: Name,
    distance_m: f64,
}

impl TryFrom<RuleFields> for FurnishRule {
    type Error = Error;

    fn try_from(fields: RuleFields) -> Result<FurnishRule, Error> {
        let rule = match fields {
            RuleFields::AgainstWall(AgainstWallFields { on_wall }) => {
                FurnishRule::AgainstWall { on_wall }
            }
            RuleFields::InCorner(InCornerFields {}) => FurnishRule::InCorner,
            RuleFields::Free(FreeFields { near }) => FurnishRule::Free { near },
            RuleFields::Beside(BesideFields { object, gap_m }) => FurnishRule::Beside {
                object,
                gap_m: at_least_zero("gap_m", "a gap", gap_m)?,
            },
            RuleFields::Facing(FacingFields { object, distance_m }) => FurnishRule::Facing {
                object,
                distance_m: above_zero("distance_m", "a distance", distance_m)?,
            },
        };
        Ok(rule)
    }
}

/// `metres`, the value of `field`, when it is above 0; `what` names the
/// value in the refusal. A document holds finite numbers only.
fn above_zero(field: &str, what: &str, metres: f64) -> Result<f64, Error> {
    let above = metres > 0.0;
    if !above {
        return Err(Error::rule(format!(
            "the {field} {metres} is not above 0; write {what} above 0 metres"
        )));
    }
    Ok(metres)
}

/// `metres`, the value of `field`, when it is at least 0; `what` names the
/// value in the refusal.
fn at_least_zero(field: &str, what: &str, metres: f64) -> Result<f64, Error> {
    let at_least = metres >= 0.0;
    if !at_least {
        return Err(Error::rule(format!(
            "the {field} {metres} is not at least 0; write {what} of 0 m or more"
        )));
    }
    Ok(metres)
}

/// The room that a robot needs to work a work surface of a piece from one
/// of its sides: `{ landmark, side, robot }` for a robot of the pattern, or
/// `{ landmark, side, depth_m, width_m }` for a room of that size.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(try_from = "StationFields")]
pub struct FurnishStation {
    /// The landmark of the work surface, as the catalogue names it.
    pub landmark: String,
    /// The side of the work surface that the robot works it from.
    pub side: Side,
    pub room: StationRoom,
}

/// What a station holds room for.
#[derive(Clone, Debug, PartialEq)]
pub enum StationRoom {
    /// A robot of the pattern, which stands at the station: its stand is
    /// `{ object: <the piece>, landmark, side }` of the station.
    Robot(RobotName),
    /// The rectangle `depth_m` deep out of the edge of the side and
    /// `width_m` wide along that edge, centred on the centre of the edge,
    /// metres, each above 0.
    Size { depth_m: f64, width_m: f64 },
}

const STATION_FORM: &str = "a station names a robot, { landmark, side, robot }, or gives a size, { landmark, side, depth_m, width_m }";

#[derive(Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a station, { landmark, side, robot } or { landmark, side, depth_m, width_m }"
)]
struct StationFields {
    landmark: String,
    side: Side,
    robot: Option<RobotName>,
    depth_m: Option<f64>,
    width_m: Option<f64>,
}

impl TryFrom<StationFields> for FurnishStation {
    type Error = Error;

    fn try_from(fields: StationFields) -> Result<FurnishStation, Error> {
        if fields.landmark.trim().is_empty() {
            return Err(Error::rule(
                "the landmark of a station is blank; name the landmark of a work surface of the piece, such as `top`",
            ));
        }
        let room = match (fields.robot, fields.depth_m, fields.width_m) {
            (Some(robot), None, None) => StationRoom::Robot(robot),
            (None, Some(depth_m), Some(width_m)) => StationRoom::Size {
                depth_m: above_zero("depth_m", "a depth", depth_m)?,
                width_m: above_zero("width_m", "a width", width_m)?,
            },
            _ => return Err(Error::rule(STATION_FORM)),
        };
        Ok(FurnishStation {
            landmark: fields.landmark,
            side: fields.side,
            room,
        })
    }
}

impl FurnishStation {
    /// The robot that stands at the station, if it names one.
    fn robot(&self) -> Option<&RobotName> {
        match &self.room {
            StationRoom::Robot(robot) => Some(robot),
            StationRoom::Size { .. } => None,
        }
    }

    /// The stand of the robot of the station at `piece`.
    fn stand_at(&self, piece: &Name) -> Stand {
        Stand::Surface {
            object: piece.clone(),
            landmark: self.landmark.clone(),
            side: Some(self.side),
            offset: Offset::default(),
        }
    }
}

/// A station that the layout of the furnish poses keeps free, at a work
/// surface of an object of the pattern with a fixed pose or of a scene
/// object: `{ object, landmark, side, depth_m, width_m }` in `stations` at
/// the root of a world pattern. Its room is the rectangle `depth_m` deep out
/// of the edge of the side and `width_m` wide along that edge, centred on
/// the centre of the edge, metres, each above 0. The solver takes it as a
/// station of no piece: each piece and each station of the layout stays
/// clear of its room, and the base of each station of the layout stands
/// away from its centre as from the base of another station.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "FixedStationFields", into = "FixedStationFields")]
pub struct FixedStation {
    /// The object whose work surface the station works: an object of
    /// `objects` with a fixed pose, or a scene object.
    pub object: Name,
    /// The landmark of the work surface, as the catalogue names it.
    pub landmark: String,
    /// The side of the work surface that the station stands at.
    pub side: Side,
    pub depth_m: f64,
    pub width_m: f64,
}

/// A fixed station as written, its numbers unchecked.
#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a fixed station, { object, landmark, side, depth_m, width_m }"
)]
struct FixedStationFields {
    object: Name,
    landmark: String,
    side: Side,
    depth_m: f64,
    width_m: f64,
}

impl TryFrom<FixedStationFields> for FixedStation {
    type Error = Error;

    fn try_from(fields: FixedStationFields) -> Result<FixedStation, Error> {
        if fields.landmark.trim().is_empty() {
            return Err(Error::rule(
                "the landmark of a station is blank; name the landmark of a work surface of the object, such as `top`",
            ));
        }
        Ok(FixedStation {
            object: fields.object,
            landmark: fields.landmark,
            side: fields.side,
            depth_m: above_zero("depth_m", "a depth", fields.depth_m)?,
            width_m: above_zero("width_m", "a width", fields.width_m)?,
        })
    }
}

impl From<FixedStation> for FixedStationFields {
    fn from(station: FixedStation) -> FixedStationFields {
        FixedStationFields {
            object: station.object,
            landmark: station.landmark,
            side: station.side,
            depth_m: station.depth_m,
            width_m: station.width_m,
        }
    }
}

/// Refuses the fixed stations `stations` of a pattern: two stations at one
/// side of one landmark of one object; a station on an object of `objects`
/// whose pose is `furnish`, of an item kind, or holds a sampler; and a
/// station on a scene object that the pattern removes or whose pose holds a
/// sampler. A station on a name that is no entry of `objects` works a scene
/// object, which the simulation checks against the drawn scene.
pub(crate) fn check_fixed_stations(
    stations: &[FixedStation],
    objects: &BTreeMap<&Name, &ObjectEntry>,
    scene_objects: &BTreeMap<&Name, &Sampled<SceneObject>>,
) -> Result<(), Error> {
    if let Some(twice) = first_named_twice(stations, |one, other| {
        one.object == other.object && one.landmark == other.landmark && one.side == other.side
    }) {
        return Err(Error::field(
            "stations",
            format!(
                "two stations work the side `{}` of `{}` of `{}`; give each station a side of its own",
                twice.side, twice.landmark, twice.object
            ),
        ));
    }
    for station in stations {
        let object = &station.object;
        let moving = match (objects.get(object), scene_objects.get(object)) {
            (Some(ObjectEntry::Furnished(_)), _) => Some("its pose is `furnish`"),
            (Some(ObjectEntry::ItemKind(_)), _) => Some("it is an entry of an item kind"),
            (Some(ObjectEntry::Object(Sampled::Drawn(drawn))), _) if draws_the_pose(drawn) => {
                Some("its pose holds a sampler")
            }
            (Some(ObjectEntry::Object(_)), _) => None,
            (None, Some(Sampled::Fixed(SceneObject::Removed))) => Some("the pattern removes it"),
            (None, Some(Sampled::Drawn(drawn))) if draws_the_pose(drawn) => {
                Some("its pose holds a sampler")
            }
            (None, _) => None,
        };
        if let Some(why) = moving {
            return Err(Error::field(
                "stations",
                format!(
                    "a station works `{object}`, and {why}; a station works an object of `objects` with a fixed pose, or a scene object"
                ),
            ));
        }
    }
    Ok(())
}

/// Whether the entry `drawn` holds a sampler in its pose.
fn draws_the_pose(drawn: &DrawnValue) -> bool {
    drawn
        .sites
        .iter()
        .any(|site| site.path.segments().first() == Some(&Segment::Key("pose".to_owned())))
}

/// The scales that a piece can take, from `min` to `max`, both included and
/// each above 0. One scale is the range from it to itself.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScaleRange {
    pub min: f64,
    pub max: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, expecting = "a scale range, { min, max }")]
struct ScaleRangeFields {
    min: f64,
    max: f64,
}

impl ScaleRange {
    /// Whether `scale` is from `min` to `max`.
    pub fn contains(&self, scale: f64) -> bool {
        self.min <= scale && scale <= self.max
    }

    /// The scales from `min` to `max`. Refuses an end that is not above 0
    /// and a `min` above the `max`.
    fn new(min: f64, max: f64) -> Result<ScaleRange, Error> {
        for (end, scale) in [("min", min), ("max", max)] {
            let above_zero = scale > 0.0;
            if !above_zero {
                return Err(Error::rule(format!(
                    "the scale {end} {scale} is not above 0; write a scale above 0"
                )));
            }
        }
        if min > max {
            return Err(Error::rule(format!(
                "the scale min {min} is above the max {max}; write a min no higher than the max"
            )));
        }
        Ok(ScaleRange { min, max })
    }

    /// The scale range that an object whose pose is `furnish` writes in
    /// `scale`: one scale, `{ min, max }`, or the scale 1 when it writes
    /// none. Refuses `{ one_of }` and `{ range }`: the solver picks the
    /// scale.
    fn written(scale: Option<Value>) -> Result<ScaleRange, Error> {
        let at_scale = |reason: String| Error::field("scale", reason);
        let fields = match scale {
            None => return Ok(ScaleRange { min: 1.0, max: 1.0 }),
            Some(Value::Object(fields)) => fields,
            Some(scale) => {
                let scale: f64 =
                    serde_json::from_value(scale).map_err(|_| at_scale(SCALE_FORM.to_owned()))?;
                let scale =
                    positive_scale(Some(scale)).map_err(|error| at_scale(error.to_string()))?;
                return Ok(ScaleRange {
                    min: scale,
                    max: scale,
                });
            }
        };
        if fields.contains_key("one_of") || fields.contains_key("range") {
            return Err(at_scale(
                "the solver picks the scale of an object whose pose is `furnish` within its range; write one scale or `{ min, max }`, not a sampler".to_owned(),
            ));
        }
        let range: ScaleRangeFields = serde_path_to_error::deserialize(Value::Object(fields))
            .map_err(|error| at_scale(error.to_string()))?;
        ScaleRange::new(range.min, range.max).map_err(|error| at_scale(error.to_string()))
    }
}

const SCALE_FORM: &str =
    "the scale of an object whose pose is `furnish` is a number or `{ min, max }`";

/// An object of `objects` whose pose is `furnish`, as a world pattern holds
/// it.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FurnishEntry {
    pub(crate) rule: FurnishRule,
    pub(crate) stations: Vec<FurnishStation>,
    pub(crate) scale: ScaleRange,
    /// The object with its pose at the origin and its scale at the least of
    /// its range: it holds the samplers that the seed draws, of its asset,
    /// its physics, its mass and its controls.
    pub(crate) template: Sampled<Object>,
    pub(crate) written: Value,
}

/// The pose of an object whose pose is `furnish`, as written.
#[derive(Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a furnish pose, { furnish, stations }"
)]
struct FurnishPoseFields {
    furnish: FurnishRule,
    #[serde(default)]
    stations: Vec<FurnishStation>,
}

/// Whether the entry `fields` of `objects` writes a pose `{ furnish }`.
pub(crate) fn writes_a_furnish_pose(fields: &Map<String, Value>) -> bool {
    fields
        .get("pose")
        .and_then(Value::as_object)
        .is_some_and(|pose| pose.contains_key(FURNISH))
}

impl FurnishEntry {
    /// The entry `fields`, which writes a furnish pose. Refuses a pose of
    /// another form, two stations at one side of one landmark, a scale of
    /// another form, a sampler in the scale, a piece with a station whose
    /// physics can be other than `static`, and a scale range whose `min`
    /// and `max` differ on a piece without a station of a robot.
    pub(crate) fn of(fields: Map<String, Value>) -> Result<FurnishEntry, Error> {
        let written = Value::Object(fields.clone());
        let mut template = fields;
        let pose = template.remove("pose").unwrap_or(Value::Null);
        let FurnishPoseFields {
            furnish: rule,
            stations,
        } = serde_path_to_error::deserialize(pose)
            .map_err(|error| Error::field("pose", error.to_string()))?;
        if let Some(twice) = first_named_twice(&stations, |one, other| {
            one.landmark == other.landmark && one.side == other.side
        }) {
            return Err(Error::field(
                "pose",
                format!(
                    "two stations work the side `{}` of `{}`; give each station a side of its own",
                    twice.side, twice.landmark
                ),
            ));
        }
        let scale = ScaleRange::written(template.remove("scale"))?;
        template.insert("pose".to_owned(), placeholder(SiteRule::Pose));
        template.insert("scale".to_owned(), Value::from(scale.min));
        let entry = FurnishEntry {
            rule,
            stations,
            scale,
            template: Sampled::of(Value::Object(template))?,
            written,
        };
        entry.check_physics()?;
        entry.check_scale_range()?;
        Ok(entry)
    }

    /// Refuses a piece with a station whose physics can be other than
    /// `static`: the robot of a station works a piece that stands still.
    fn check_physics(&self) -> Result<(), Error> {
        if self.stations.is_empty() {
            return Ok(());
        }
        let moving = physics_candidates(&self.template)
            .into_iter()
            .flatten()
            .find(|physics| *physics != Physics::Static);
        let Some(moving) = moving else {
            return Ok(());
        };
        Err(Error::field(
            "physics",
            format!(
                "can be `{}`, and a piece with a station stands still for the robot that works it; write `static`, or leave `physics` out",
                String::from(moving)
            ),
        ))
    }

    /// Refuses a scale range whose `min` and `max` differ on a piece without
    /// a station of a robot: the solver picks the scale of a piece by the
    /// robots of its stations.
    fn check_scale_range(&self) -> Result<(), Error> {
        let measured = self
            .stations
            .iter()
            .any(|station| station.robot().is_some());
        if measured || self.scale.min == self.scale.max {
            return Ok(());
        }
        Err(Error::field(
            "scale",
            format!(
                "is a range from {} to {}, and the solver picks the scale of a piece by the robots of its stations; give the piece a station of a robot, or write one scale",
                self.scale.min, self.scale.max
            ),
        ))
    }

    /// The physics of the piece in the concrete world, from `drawn`, the
    /// physics that the seed draws: `static` for a piece with a station,
    /// which the solver makes static, else `drawn`.
    fn physics_in_world(&self, drawn: Option<Physics>) -> Option<Physics> {
        match self.stations.is_empty() {
            true => drawn,
            false => Some(Physics::Static),
        }
    }

    /// Each physics that the piece can take in the concrete world, `None`
    /// for the physics of the catalogue.
    pub(crate) fn physics_in_worlds(&self) -> BTreeSet<Option<Physics>> {
        physics_candidates(&self.template)
            .into_iter()
            .map(|physics| self.physics_in_world(physics))
            .collect()
    }

    /// The piece `object` for the solver, with the values that the seed
    /// drew in `drawn`, the template made concrete.
    pub(crate) fn piece(&self, object: &Name, drawn: Object) -> FurnishPiece {
        FurnishPiece {
            object: object.clone(),
            asset: drawn.asset,
            scale: self.scale,
            physics: self.physics_in_world(drawn.physics),
            mass: drawn.mass,
            controls: drawn.controls,
            rule: self.rule.clone(),
            stations: self.stations.clone(),
        }
    }

    /// The object at `place`, with the values that the seed drew in
    /// `drawn`, the template made concrete.
    pub(crate) fn placed(&self, drawn: Object, place: &FurnishedPlace) -> Object {
        Object {
            pose: Pose::Yawed {
                position: place.position,
                yaw: place.yaw,
            },
            scale: place.scale,
            physics: self.physics_in_world(drawn.physics),
            ..drawn
        }
    }
}

/// A piece that the solver lays out, as the furnish layout gives it to the
/// simulation: the object, its asset, physics, mass and controls after the
/// draws of the seed, its scale range, its rule and its stations. The
/// physics is that of the concrete world: `static` for a piece with a
/// station.
#[derive(Clone, Debug, PartialEq)]
pub struct FurnishPiece {
    pub object: Name,
    pub asset: String,
    pub scale: ScaleRange,
    pub physics: Option<Physics>,
    pub mass: Option<f64>,
    pub controls: BTreeMap<String, f64>,
    pub rule: FurnishRule,
    pub stations: Vec<FurnishStation>,
}

/// Refuses the furnish poses of a pattern, `pieces` by their names, that
/// break a rule across its entries, with the robots of the pattern: a rule
/// that names the piece itself or an object whose pose is not `furnish`;
/// `beside` a piece whose rule is not `against_wall` or `in_corner`; a cycle
/// of `beside` and `facing`; a station of a robot that the pattern does not
/// hold, or that another station names too; a robot of a station whose
/// stand is not at its station; and a robot that stands at a piece and that
/// no station of the piece names.
pub(crate) fn check_furnish_poses(
    pieces: &BTreeMap<&Name, &FurnishEntry>,
    robots: &BTreeMap<&RobotName, &Sampled<Robot>>,
) -> Result<(), Error> {
    check_named_pieces(pieces)?;
    refuse_a_cycle(pieces)?;
    let stations = robot_stations(pieces, robots)?;
    check_stands(pieces, robots, &stations)
}

/// Refuses a rule that names the piece itself or an object whose pose is
/// not `furnish`, and `beside` a piece whose rule is not `against_wall` or
/// `in_corner`.
fn check_named_pieces(pieces: &BTreeMap<&Name, &FurnishEntry>) -> Result<(), Error> {
    for (piece, entry) in pieces {
        let Some(named) = entry.rule.object() else {
            continue;
        };
        let at = format!("objects.{piece}.pose");
        let rule = entry.rule.key();
        if named == *piece {
            return Err(Error::field(
                at,
                format!(
                    "`{rule}` names `{piece}` itself; name another object whose pose is `furnish`"
                ),
            ));
        }
        let Some(other) = pieces.get(named) else {
            return Err(Error::field(
                at,
                format!(
                    "`{rule}` names `{named}`, which is no object of the pattern whose pose is `furnish`; name such an object"
                ),
            ));
        };
        let beside = matches!(entry.rule, FurnishRule::Beside { .. });
        if beside && !other.rule.is_against_a_wall() {
            return Err(Error::field(
                at,
                format!(
                    "`beside` names `{named}`, whose rule is `{}`; a piece stands beside a piece against a wall, so give `{named}` the rule `against_wall` or `in_corner`",
                    other.rule.key()
                ),
            ));
        }
    }
    Ok(())
}

/// Refuses pieces whose `beside` and `facing` name each other in a cycle,
/// naming the pieces of the cycle. Each rule names one piece at most, so the
/// pieces that a piece leads to form one chain.
fn refuse_a_cycle(pieces: &BTreeMap<&Name, &FurnishEntry>) -> Result<(), Error> {
    for start in pieces.keys() {
        let mut chain = vec![*start];
        let mut next = pieces[start].rule.object();
        while let Some(named) = next {
            if named == *start {
                let cycle: Vec<String> = chain.iter().map(|piece| format!("`{piece}`")).collect();
                return Err(Error::rule(format!(
                    "the furnish poses of {} place each other in a cycle of `beside` and `facing`; give one of them `against_wall`, `in_corner` or `free`",
                    cycle.join(", ")
                )));
            }
            if chain.contains(&named) {
                break;
            }
            chain.push(named);
            next = pieces.get(named).and_then(|entry| entry.rule.object());
        }
    }
    Ok(())
}

/// The piece and the station of each robot that a station names. Refuses a
/// robot that the pattern does not hold and a robot that two stations name.
fn robot_stations<'a>(
    pieces: &BTreeMap<&'a Name, &'a FurnishEntry>,
    robots: &BTreeMap<&RobotName, &Sampled<Robot>>,
) -> Result<BTreeMap<&'a RobotName, (&'a Name, &'a FurnishStation)>, Error> {
    let mut stations = BTreeMap::new();
    for (piece, entry) in pieces {
        for station in &entry.stations {
            let Some(robot) = station.robot() else {
                continue;
            };
            let at = format!("objects.{piece}.pose");
            if !robots.contains_key(robot) {
                return Err(Error::field(
                    at,
                    format!(
                        "a station names the robot `{robot}`, which is no robot of the pattern; name a robot of `robots`"
                    ),
                ));
            }
            if let Some((other, _)) = stations.insert(robot, (*piece, station)) {
                return Err(Error::field(
                    at,
                    format!(
                        "a station names the robot `{robot}`, which a station of `{other}` names too; a robot stands at one station"
                    ),
                ));
            }
        }
    }
    Ok(stations)
}

/// Refuses a robot of a station whose stand can be other than `{ object,
/// landmark, side }` of its station, without an offset, and a robot that can
/// stand at a piece whose stations do not name it.
fn check_stands(
    pieces: &BTreeMap<&Name, &FurnishEntry>,
    robots: &BTreeMap<&RobotName, &Sampled<Robot>>,
    stations: &BTreeMap<&RobotName, (&Name, &FurnishStation)>,
) -> Result<(), Error> {
    for (robot, entry) in robots {
        let at = format!("robots.{robot}.stand");
        let stands = stand_candidates(entry)?;
        if let Some((piece, station)) = stations.get(robot) {
            let expected = station.stand_at(piece);
            if stands.iter().any(|stand| stand.as_ref() != Some(&expected)) {
                return Err(Error::field(
                    at,
                    format!(
                        "a station of `{piece}` names `{robot}`, so the robot stands at the side `{side}` of `{landmark}`: write its stand {{ object: \"{piece}\", landmark: \"{landmark}\", side: \"{side}\" }}, without an offset",
                        side = station.side,
                        landmark = station.landmark,
                    ),
                ));
            }
            continue;
        }
        let at_a_piece = stands.iter().find_map(|stand| match stand {
            Some(Stand::Surface { object, .. }) if pieces.contains_key(object) => Some(object),
            _ => None,
        });
        if let Some(piece) = at_a_piece {
            return Err(Error::field(
                at,
                format!(
                    "stands at `{piece}`, whose pose is `furnish`, and no station of `{piece}` names `{robot}`; add the station {{ landmark, side, robot: \"{robot}\" }} to `{piece}`, so that the layout leaves room for the robot"
                ),
            ));
        }
    }
    Ok(())
}

/// Each stand that the robot entry can take: its stand, or the stand of
/// each candidate of an entry that holds samplers.
fn stand_candidates(robot: &Sampled<Robot>) -> Result<Vec<Option<Stand>>, Error> {
    let stands = match robot {
        Sampled::Fixed(robot) => vec![robot.stand.clone()],
        Sampled::Drawn(drawn) => drawn
            .candidate_values::<Robot>()?
            .into_iter()
            .map(|robot| robot.stand)
            .collect(),
    };
    Ok(stands)
}

/// The place of a piece in the placed value of the furnish layout: a
/// position in the world frame, a yaw about +z and a scale.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "the place of a piece, { position, yaw, scale }"
)]
pub(crate) struct FurnishedPlace {
    position: Point,
    yaw: f64,
    scale: f64,
}

/// The placed value of the furnish layout with each piece of `pieces` at
/// the origin, with yaw 0, at the least scale of its range.
pub(crate) fn placeholder_places(pieces: &[FurnishPiece]) -> Value {
    let places = pieces.iter().map(|piece| {
        let place = json!({ "position": [0.0, 0.0, 0.0], "yaw": 0.0, "scale": piece.scale.min });
        (piece.object.to_string(), place)
    });
    Value::Object(places.collect())
}

/// The place of each piece of `pieces`, by its name, from the placed value
/// of the furnish layout in `placed`; no place for a pattern without a
/// piece. Refuses a layout without a placed value, a value that is not an
/// object, a place of an object that is not a piece, a piece without a
/// place, a place of another form, and a scale outside the range of its
/// piece.
pub(crate) fn places_of(
    pieces: &BTreeMap<&Name, &FurnishEntry>,
    placed: &BTreeMap<SitePath, Value>,
) -> Result<BTreeMap<Name, FurnishedPlace>, Error> {
    if pieces.is_empty() {
        return Ok(BTreeMap::new());
    }
    let Some(value) = placed.get(&furnish_path()) else {
        return Err(Error::field(FURNISH, "has no placed value; place it first"));
    };
    let Value::Object(entries) = value else {
        return Err(Error::field(
            FURNISH,
            "is an object from the name of each object whose pose is `furnish` to its place, { position, yaw, scale }",
        ));
    };
    let mut places = BTreeMap::new();
    for (key, place) in entries {
        let piece = Name::parse(key)
            .ok()
            .and_then(|name| pieces.get_key_value(&name));
        let Some((name, entry)) = piece else {
            return Err(Error::field(
                FURNISH,
                format!(
                    "holds a place of `{key}`, which is no object of the pattern whose pose is `furnish`; give a place to each such object alone"
                ),
            ));
        };
        let at = format!("{FURNISH}.{key}");
        let place: FurnishedPlace = serde_path_to_error::deserialize(place.clone())
            .map_err(|error| Error::field(&at, error.to_string()))?;
        if !entry.scale.contains(place.scale) {
            return Err(Error::field(
                format!("{at}.scale"),
                format!(
                    "{} is not from {} to {}, the scale range of `{key}`",
                    place.scale, entry.scale.min, entry.scale.max
                ),
            ));
        }
        places.insert((*name).clone(), place);
    }
    if let Some(name) = pieces.keys().find(|name| !places.contains_key(**name)) {
        return Err(Error::field(
            FURNISH,
            format!(
                "holds no place of `{name}`; give a place to each object whose pose is `furnish`"
            ),
        ));
    }
    Ok(places)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pattern::tests::placed;
    use crate::world::tests::{RELEASE, name, robot};
    use crate::{PlacementKind, PlacementSite, Placements, Seed, World, WorldPattern};

    /// A room of five pieces, one of each rule: a desk against the north
    /// wall and a counter in a corner, each with the station of a robot, a
    /// shelf beside the counter, an armchair facing the desk, and a rug free
    /// near a point with a station of a size. A lamp has a fixed pose and a
    /// drawn yaw, a mug stands on the desk, a crate on the counter where
    /// `bravo` reaches it, a tray and the robot `courier` in a region, and
    /// the robot `fixed` at a position.
    fn room_text() -> String {
        format!(
            r#"{{
  simulation_schema: "world/v1",
  manifest: {{ name: "furnished_room", revision: 1 }},
  catalogue: "{RELEASE}",
  scene: {{ asset: "scene/empty_room" }},
  robots: {{
    alpha: {{ join: {{ option: "openarm_sim" }}, stand: {{ object: "desk", landmark: "top", side: "-y" }} }},
    bravo: {{ join: {{ option: "so101_sim" }}, stand: {{ object: "counter", landmark: "top", side: "+x" }} }},
    courier: {{ join: {{ option: "sourccey_sim" }}, stand: {{ position: {{ in: "aisle" }}, yaw: 0 }} }},
    fixed: {{ join: {{ option: "lekiwi_sim" }}, stand: {{ position: [3, 3, 0], yaw: 0 }} }},
  }},
  objects: {{
    desk: {{ asset: {{ one_of: ["object/standing_desk", "object/office_desk"] }}, pose: {{ furnish: {{ against_wall: {{ on_wall: "north" }} }}, stations: [{{ landmark: "top", side: "-y", robot: "alpha" }}] }}, scale: {{ min: 0.6, max: 1.0 }}, controls: {{ height: {{ range: [0.7, 1.1] }} }} }},
    counter: {{ asset: "object/counter", pose: {{ furnish: {{ in_corner: {{}} }}, stations: [{{ landmark: "top", side: "+x", robot: "bravo" }}] }}, physics: "static", scale: {{ min: 0.8, max: 1.2 }} }},
    shelf: {{ asset: "object/shelf", pose: {{ furnish: {{ beside: {{ object: "counter", gap_m: 0.1 }} }} }} }},
    armchair: {{ asset: "object/armchair", pose: {{ furnish: {{ facing: {{ object: "desk", distance_m: 1.5 }} }} }}, scale: 0.9, mass: 12 }},
    rug: {{ asset: "object/rug", pose: {{ furnish: {{ free: {{ near: [1, 2] }} }}, stations: [{{ landmark: "top", side: "+y", depth_m: 0.6, width_m: 0.8 }}] }} }},
    lamp: {{ asset: "object/lamp", pose: {{ position: [2, 2, 0], yaw: {{ range: [0, 1] }} }} }},
    mug: {{ asset: "object/mug", pose: {{ on: {{ object: "desk", landmark: "top" }} }} }},
    crate: {{ asset: "object/crate", pose: {{ on: {{ object: "counter", landmark: "top" }}, reachable_by: "bravo" }} }},
    tray: {{ asset: "object/tray", pose: {{ in: "aisle" }} }},
  }},
  regions: {{ aisle: {{ centre: [0, -1.2, 0.5], size: [3, 0.8, 1] }} }},
}}"#
        )
    }

    fn room() -> WorldPattern {
        WorldPattern::parse(&room_text()).unwrap()
    }

    /// The room text with `from` replaced by `to`, which it holds once.
    fn room_with(from: &str, to: &str) -> String {
        assert_eq!(room_text().matches(from).count(), 1, "{from}");
        room_text().replace(from, to)
    }

    fn placements_of(pattern: &WorldPattern, seed: u32) -> Placements {
        let expansion = pattern.expansion(Seed(seed), &|_| Vec::new()).unwrap();
        expansion.placements().unwrap()
    }

    fn pieces_of(placements: &Placements) -> &[FurnishPiece] {
        match &placements.sites()[0].kind {
            PlacementKind::Furnish { pieces } => pieces,
            other => panic!("the first placement is the furnish layout: {other:?}"),
        }
    }

    fn piece<'a>(pieces: &'a [FurnishPiece], object: &str) -> &'a FurnishPiece {
        pieces
            .iter()
            .find(|piece| piece.object.as_str() == object)
            .unwrap()
    }

    fn paths(sites: &[PlacementSite]) -> Vec<String> {
        sites.iter().map(|site| site.path.to_string()).collect()
    }

    fn sorted_names<T>(map: &BTreeMap<Name, T>) -> Vec<&str> {
        map.keys().map(Name::as_str).collect()
    }

    fn standing(world: &World) -> Vec<&str> {
        world
            .robots
            .iter()
            .filter(|(_, robot)| robot.stand.is_some())
            .map(|(name, _)| name.as_str())
            .collect()
    }

    #[test]
    fn each_rule_and_station_reads_into_its_piece() {
        let placements = placements_of(&room(), 3);
        let pieces = pieces_of(&placements);
        let objects: Vec<&str> = pieces.iter().map(|piece| piece.object.as_str()).collect();
        assert_eq!(objects, ["armchair", "counter", "desk", "rug", "shelf"]);
        assert_eq!(
            piece(pieces, "desk").rule,
            FurnishRule::AgainstWall {
                on_wall: Some(name("north"))
            }
        );
        assert_eq!(piece(pieces, "counter").rule, FurnishRule::InCorner);
        assert_eq!(
            piece(pieces, "shelf").rule,
            FurnishRule::Beside {
                object: name("counter"),
                gap_m: 0.1
            }
        );
        assert_eq!(
            piece(pieces, "armchair").rule,
            FurnishRule::Facing {
                object: name("desk"),
                distance_m: 1.5
            }
        );
        assert_eq!(
            piece(pieces, "rug").rule,
            FurnishRule::Free {
                near: Some([1.0, 2.0])
            }
        );
        assert_eq!(
            piece(pieces, "desk").stations,
            [FurnishStation {
                landmark: "top".into(),
                side: Side::NegY,
                room: StationRoom::Robot(robot("alpha")),
            }]
        );
        assert_eq!(
            piece(pieces, "rug").stations,
            [FurnishStation {
                landmark: "top".into(),
                side: Side::PosY,
                room: StationRoom::Size {
                    depth_m: 0.6,
                    width_m: 0.8
                },
            }]
        );
        assert!(piece(pieces, "shelf").stations.is_empty());
        let keys: Vec<&str> = pieces.iter().map(|piece| piece.rule.key()).collect();
        assert_eq!(
            keys,
            ["facing", "in_corner", "against_wall", "free", "beside"]
        );
        let named: Vec<Option<&str>> = pieces
            .iter()
            .map(|piece| piece.rule.object().map(Name::as_str))
            .collect();
        assert_eq!(named, [Some("desk"), None, None, None, Some("counter")]);
    }

    #[test]
    fn the_optional_fields_of_a_rule_and_the_stations_may_be_left_out() {
        let text = room_with(
            r#"{ against_wall: { on_wall: "north" } }"#,
            "{ against_wall: {} }",
        )
        .replace("{ free: { near: [1, 2] } }", "{ free: {} }")
        .replace(
            r#", stations: [{ landmark: "top", side: "+y", depth_m: 0.6, width_m: 0.8 }]"#,
            "",
        );
        let placements = placements_of(&WorldPattern::parse(&text).unwrap(), 3);
        let pieces = pieces_of(&placements);
        assert_eq!(
            piece(pieces, "desk").rule,
            FurnishRule::AgainstWall { on_wall: None }
        );
        assert_eq!(piece(pieces, "rug").rule, FurnishRule::Free { near: None });
        assert!(piece(pieces, "rug").stations.is_empty());
    }

    /// Each piece holds the values that the seed drew, its scale range (one
    /// scale for a number, 1 for none), and the physics of the concrete
    /// world: `static` for a piece with a station.
    #[test]
    fn a_piece_holds_the_draws_of_the_seed_its_scale_range_and_its_physics() {
        let mut assets = BTreeSet::new();
        for seed in 0..16 {
            let placements = placements_of(&room(), seed);
            let pieces = pieces_of(&placements);
            let desk = piece(pieces, "desk");
            assets.insert(desk.asset.clone());
            assert!((0.7..=1.1).contains(&desk.controls["height"]), "{desk:?}");
            assert_eq!(desk.scale, ScaleRange { min: 0.6, max: 1.0 });
            assert_eq!(desk.physics, Some(Physics::Static));
            let world = placements
                .expansion()
                .world(&placed(placements.sites()))
                .unwrap();
            let in_world = &world.objects[&name("desk")];
            assert_eq!(in_world.asset, desk.asset);
            assert_eq!(in_world.controls, desk.controls);
        }
        assert_eq!(
            assets,
            BTreeSet::from(["object/office_desk".into(), "object/standing_desk".into()])
        );
        let placements = placements_of(&room(), 3);
        let pieces = pieces_of(&placements);
        let armchair = piece(pieces, "armchair");
        assert_eq!(armchair.scale, ScaleRange { min: 0.9, max: 0.9 });
        assert_eq!((armchair.physics, armchair.mass), (None, Some(12.0)));
        assert_eq!(
            piece(pieces, "shelf").scale,
            ScaleRange { min: 1.0, max: 1.0 }
        );
        assert_eq!(piece(pieces, "rug").physics, Some(Physics::Static));
        let pattern = room();
        assert_eq!(
            pattern.physics_of(&name("desk")),
            Some(BTreeSet::from([Some(Physics::Static)]))
        );
        assert_eq!(
            pattern.physics_of(&name("shelf")),
            Some(BTreeSet::from([None]))
        );
        assert!(!pattern.is_concrete());
    }

    /// The furnish layout comes first, before every pose `on` or `in` and
    /// every stand; the stands of the robots of stations, each pose on a
    /// piece and each stand in a region come after it.
    #[test]
    fn the_furnish_layout_comes_first_and_what_stands_on_it_after_it() {
        let placements = placements_of(&room(), 3);
        assert_eq!(
            paths(placements.sites()),
            [
                "furnish",
                "objects.mug.pose",
                "objects.tray.pose",
                "robots.alpha.stand",
                "robots.bravo.stand",
                "robots.courier.stand.position",
                "objects.crate.pose",
            ]
        );
        assert_eq!(placements.sites()[0].path, furnish_path());
        let unfurnished = WorldPattern::parse(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "bare", revision: 1 }}, catalogue: "{RELEASE}", scene: {{ asset: "scene/empty_room" }},
              objects: {{ lamp: {{ asset: "object/lamp", pose: {{ in: "aisle" }} }} }},
              regions: {{ aisle: {{ centre: [0, 0, 0.5], size: [1, 1, 1] }} }} }}"#
        ))
        .unwrap();
        assert_eq!(
            paths(placements_of(&unfurnished, 3).sites()),
            ["objects.lamp.pose"],
            "a pattern without a furnish pose has no furnish layout"
        );
    }

    #[test]
    fn the_world_so_far_holds_the_pieces_once_the_layout_is_placed() {
        let placements = placements_of(&room(), 3);
        let before = placements.world_so_far(&BTreeMap::new()).unwrap();
        assert_eq!(sorted_names(&before.objects), ["lamp"]);
        assert_eq!(standing(&before), ["fixed"]);

        let all = placed(placements.sites());
        let layout = BTreeMap::from([(furnish_path(), all[&furnish_path()].clone())]);
        let after = placements.world_so_far(&layout).unwrap();
        assert_eq!(
            sorted_names(&after.objects),
            ["armchair", "counter", "desk", "lamp", "rug", "shelf"]
        );
        assert_eq!(standing(&after), ["alpha", "bravo", "fixed"]);
        assert_eq!(
            after.objects[&name("counter")].pose,
            Pose::Yawed {
                position: [1.0, 3.0, 0.0],
                yaw: 0.5
            }
        );
        assert_eq!(after.objects[&name("counter")].scale, 1.2);

        let everything = placements.world_so_far(&all).unwrap();
        assert_eq!(everything, placements.expansion().world(&all).unwrap());
    }

    /// The concrete world gives each piece the pose and the scale of its
    /// place, and `static` to a piece with a station.
    #[test]
    fn the_concrete_world_holds_the_placed_poses_and_scales() {
        let placements = placements_of(&room(), 3);
        let world = placements
            .expansion()
            .world(&placed(placements.sites()))
            .unwrap();
        let places = [
            ("armchair", 0.0, 0.9, None),
            ("counter", 1.0, 1.2, Some(Physics::Static)),
            ("desk", 2.0, 1.0, Some(Physics::Static)),
            ("rug", 3.0, 1.0, Some(Physics::Static)),
            ("shelf", 4.0, 1.0, None),
        ];
        for (object, x, scale, physics) in places {
            let placed = &world.objects[&name(object)];
            assert_eq!(
                placed.pose,
                Pose::Yawed {
                    position: [x, 3.0, 0.0],
                    yaw: 0.5
                },
                "{object}"
            );
            assert_eq!((placed.scale, placed.physics), (scale, physics), "{object}");
        }
        assert_eq!(world.objects[&name("armchair")].mass, Some(12.0));
        let text = world.to_json5().unwrap();
        assert_eq!(World::parse(&text).unwrap(), world);
    }

    /// The placeholder world, and so each candidate world, holds each piece
    /// at the origin with yaw 0 and the least scale of its range.
    #[test]
    fn the_placeholder_world_holds_each_piece_at_the_origin_and_its_least_scale() {
        let pattern = room();
        let origin = Pose::Yawed {
            position: [0.0, 0.0, 0.0],
            yaw: 0.0,
        };
        let placeholder = placements_of(&pattern, 3).placeholder_world().unwrap();
        for (object, scale) in [
            ("armchair", 0.9),
            ("counter", 0.8),
            ("desk", 0.6),
            ("rug", 1.0),
            ("shelf", 1.0),
        ] {
            let placed = &placeholder.objects[&name(object)];
            assert_eq!((&placed.pose, placed.scale), (&origin, scale), "{object}");
        }
        let worlds = pattern.candidate_worlds(&|_| Vec::new()).unwrap();
        // The first world, and one more for the other asset of the desk, the
        // other end of the height of the desk and of the yaw of the lamp.
        assert_eq!(worlds.len(), 4);
        let desk_assets: BTreeSet<&str> = worlds
            .iter()
            .map(|world| world.objects[&name("desk")].asset.as_str())
            .collect();
        assert_eq!(
            desk_assets,
            BTreeSet::from(["object/office_desk", "object/standing_desk"])
        );
        for world in &worlds {
            assert_eq!(world.objects[&name("desk")].scale, 0.6);
        }
    }

    #[test]
    fn a_furnish_value_of_another_form_is_refused() {
        let placements = placements_of(&room(), 3);
        let all = placed(placements.sites());
        let with_layout = |layout: Value| {
            let mut placed = all.clone();
            placed.insert(furnish_path(), layout);
            placements
                .expansion()
                .world(&placed)
                .unwrap_err()
                .to_string()
        };
        let layout = all[&furnish_path()].clone();
        let edited = |edit: &dyn Fn(&mut Map<String, Value>)| {
            let mut layout = layout.as_object().unwrap().clone();
            edit(&mut layout);
            with_layout(Value::Object(layout))
        };
        let place = json!({ "position": [0, 0, 0], "yaw": 0, "scale": 1 });
        let cases = [
            (
                edited(&|layout| {
                    layout.remove("rug");
                }),
                "furnish: holds no place of `rug`; give a place to each object whose pose is `furnish`",
            ),
            (
                edited(&|layout| {
                    layout.insert("lamp".into(), place.clone());
                }),
                "furnish: holds a place of `lamp`, which is no object of the pattern whose pose is `furnish`; give a place to each such object alone",
            ),
            (
                edited(&|layout| {
                    layout.insert("Sofa".into(), place.clone());
                }),
                "furnish: holds a place of `Sofa`, which is no object of the pattern whose pose is `furnish`; give a place to each such object alone",
            ),
            (
                edited(&|layout| {
                    layout.insert(
                        "rug".into(),
                        json!({ "position": [0, 0], "yaw": 0, "scale": 1 }),
                    );
                }),
                "furnish.rug: position: invalid length 2, expected an array of length 3",
            ),
            (
                edited(&|layout| {
                    layout.insert("rug".into(), json!({ "position": [0, 0, 0], "yaw": 0 }));
                }),
                "furnish.rug: missing field `scale`",
            ),
            (
                edited(&|layout| {
                    layout.insert(
                        "rug".into(),
                        json!({ "position": [0, 0, 0], "yaw": 0, "scale": 1, "pitch": 0 }),
                    );
                }),
                "furnish.rug: pitch: unknown field `pitch`, expected one of `position`, `yaw`, `scale`",
            ),
            (
                edited(&|layout| {
                    layout.insert(
                        "desk".into(),
                        json!({ "position": [0, 0, 0], "yaw": 0, "scale": 1.1 }),
                    );
                }),
                "furnish.desk.scale: 1.1 is not from 0.6 to 1, the scale range of `desk`",
            ),
            (
                edited(&|layout| {
                    layout.insert(
                        "shelf".into(),
                        json!({ "position": [0, 0, 0], "yaw": 0, "scale": 0.9 }),
                    );
                }),
                "furnish.shelf.scale: 0.9 is not from 1 to 1, the scale range of `shelf`",
            ),
            (
                with_layout(json!([])),
                "furnish: is an object from the name of each object whose pose is `furnish` to its place, { position, yaw, scale }",
            ),
        ];
        for (refused, expected) in cases {
            assert_eq!(refused, expected);
        }
        let mut without_layout = all.clone();
        without_layout.remove(&furnish_path());
        assert_eq!(
            placements
                .expansion()
                .world(&without_layout)
                .unwrap_err()
                .to_string(),
            "furnish: has no placed value; place it first"
        );
    }

    #[test]
    fn a_pattern_with_furnish_poses_expands_to_the_same_placements_and_world_twice() {
        let pattern = room();
        for seed in [0, 7, 4_000_000_000] {
            let first = placements_of(&pattern, seed);
            let second = placements_of(&pattern, seed);
            assert_eq!(first.sites(), second.sites());
            let placed = placed(first.sites());
            assert_eq!(
                first.expansion().world(&placed).unwrap(),
                second.expansion().world(&placed).unwrap()
            );
            let stream = |placements: &Placements| {
                placements
                    .expansion()
                    .stream(&placements.sites()[0].path)
                    .next_u64()
            };
            assert_eq!(stream(&first), stream(&second));
        }
    }

    /// One more furnish pose adds its piece to the furnish layout and changes
    /// no other drawn value: the other pieces, the other placements, the
    /// stream of the layout and every other value of the world stay the
    /// same.
    #[test]
    fn an_added_furnish_pose_changes_no_other_drawn_value() {
        let more = WorldPattern::parse(&room_with(
            "    lamp:",
            "    bench: { asset: { one_of: [\"object/bench_a\", \"object/bench_b\"] }, pose: { furnish: { free: {} } } },\n    lamp:",
        ))
        .unwrap();
        for seed in 0..8 {
            let fewer = placements_of(&room(), seed);
            let more = placements_of(&more, seed);
            let mut pieces = pieces_of(&more).to_vec();
            pieces.retain(|piece| piece.object.as_str() != "bench");
            assert_eq!(pieces, pieces_of(&fewer));
            assert_eq!(more.sites()[1..], fewer.sites()[1..]);
            assert_eq!(
                more.expansion().stream(&furnish_path()).next_u64(),
                fewer.expansion().stream(&furnish_path()).next_u64()
            );
            let place_of = |placements: &Placements| {
                let mut placed = placed(placements.sites());
                let layout = placed.get_mut(&furnish_path()).unwrap();
                for (object, place) in layout.as_object_mut().unwrap() {
                    let x: u32 = object.bytes().map(u32::from).sum();
                    place["position"] = json!([f64::from(x), 3.0, 0.0]);
                }
                placements.expansion().world(&placed).unwrap()
            };
            let (fewer_world, mut more_world) = (place_of(&fewer), place_of(&more));
            assert!(more_world.objects.remove(&name("bench")).is_some());
            more_world.manifest = fewer_world.manifest.clone();
            assert_eq!(more_world, fewer_world);
        }
    }

    /// The pattern text with each `(from, to)` of `edits` made, each `from`
    /// held once.
    fn refusal(edits: &[(&str, &str)]) -> String {
        let mut text = room_text();
        for (from, to) in edits {
            assert_eq!(text.matches(from).count(), 1, "{from}");
            text = text.replace(from, to);
        }
        WorldPattern::parse(&text).unwrap_err().to_string()
    }

    const DESK_RULE: &str = r#"{ against_wall: { on_wall: "north" } }"#;
    const ALPHA_STAND: &str = r#"stand: { object: "desk", landmark: "top", side: "-y" }"#;
    const ALPHA_STATION: &str = r#"{ landmark: "top", side: "-y", robot: "alpha" }"#;
    const RUG_STATION: &str = r#"{ landmark: "top", side: "+y", depth_m: 0.6, width_m: 0.8 }"#;

    #[test]
    fn a_rule_or_a_station_of_another_form_is_refused_by_its_object() {
        let rules = "a furnish rule is an object with one key naming a rule: `against_wall`, `in_corner`, `free`, `beside`, `facing`";
        let cases: [(&[(&str, &str)], String); 21] = [
            (
                &[(DESK_RULE, "{ against_the_wall: {} }")],
                format!("objects.desk: pose: furnish: {rules}"),
            ),
            (
                &[(DESK_RULE, "{ against_wall: {}, in_corner: {} }")],
                format!("objects.desk: pose: furnish: {rules}"),
            ),
            (
                &[(DESK_RULE, r#""against_wall""#)],
                format!("objects.desk: pose: furnish: {rules}"),
            ),
            (
                &[(DESK_RULE, r#"{ against_wall: { on_wall: "north", side: 1 } }"#)],
                "objects.desk: pose: furnish: against_wall.side: unknown field `side`, expected `on_wall`".into(),
            ),
            (
                &[(DESK_RULE, r#"{ against_wall: { on_wall: "North" } }"#)],
                "objects.desk: pose: furnish: against_wall.on_wall: the name `North` is not lowercase letters, digits and `_`".into(),
            ),
            (
                &[("{ in_corner: {} }", "{ in_corner: { wall: \"north\" } }")],
                "objects.counter: pose: furnish: in_corner.wall: unknown field `wall`, there are no fields".into(),
            ),
            (
                &[("{ free: { near: [1, 2] } }", "{ free: { near: [1] } }")],
                "objects.rug: pose: furnish: free.near: invalid length 1, expected an array of length 2".into(),
            ),
            (
                &[("gap_m: 0.1", "gap_m: -0.1")],
                "objects.shelf: pose: furnish: the gap_m -0.1 is not at least 0; write a gap of 0 m or more".into(),
            ),
            (
                &[("{ object: \"counter\", gap_m: 0.1 }", "{ object: \"counter\" }")],
                "objects.shelf: pose: furnish: beside: missing field `gap_m`".into(),
            ),
            (
                &[("distance_m: 1.5", "distance_m: 0")],
                "objects.armchair: pose: furnish: the distance_m 0 is not above 0; write a distance above 0 metres".into(),
            ),
            (
                &[(RUG_STATION, r#"{ landmark: "top", side: "+y", depth_m: 0, width_m: 0.8 }"#)],
                "objects.rug: pose: stations[0]: the depth_m 0 is not above 0; write a depth above 0 metres".into(),
            ),
            (
                &[(RUG_STATION, r#"{ landmark: "top", side: "+y", depth_m: 0.6, width_m: -1 }"#)],
                "objects.rug: pose: stations[0]: the width_m -1 is not above 0; write a width above 0 metres".into(),
            ),
            (
                &[(RUG_STATION, r#"{ landmark: "top", side: "+y", depth_m: 0.6 }"#)],
                format!("objects.rug: pose: stations[0]: {STATION_FORM}"),
            ),
            (
                &[(RUG_STATION, r#"{ landmark: "top", side: "+y" }"#)],
                format!("objects.rug: pose: stations[0]: {STATION_FORM}"),
            ),
            (
                &[(ALPHA_STATION, r#"{ landmark: "top", side: "-y", robot: "alpha", depth_m: 0.6, width_m: 0.8 }"#)],
                format!("objects.desk: pose: stations[0]: {STATION_FORM}"),
            ),
            (
                &[(RUG_STATION, r#"{ landmark: "top", side: "+z", depth_m: 0.6, width_m: 0.8 }"#)],
                "objects.rug: pose: stations[0].side: unknown variant `+z`, expected one of `+x`, `-x`, `+y`, `-y`".into(),
            ),
            (
                &[(RUG_STATION, r#"{ landmark: " ", side: "+y", depth_m: 0.6, width_m: 0.8 }"#)],
                "objects.rug: pose: stations[0]: the landmark of a station is blank; name the landmark of a work surface of the piece, such as `top`".into(),
            ),
            (
                &[(RUG_STATION, r#"{ landmark: "top", side: "+y", depth_m: 0.6, width_m: 0.8, height_m: 1 }"#)],
                "objects.rug: pose: stations[0].height_m: unknown field `height_m`, expected one of `landmark`, `side`, `robot`, `depth_m`, `width_m`".into(),
            ),
            (
                &[(RUG_STATION, &format!("{RUG_STATION}, {{ landmark: \"top\", side: \"+y\", depth_m: 0.3, width_m: 0.3 }}"))],
                "objects.rug: pose: two stations work the side `+y` of `top`; give each station a side of its own".into(),
            ),
            (
                &[("{ furnish: { free: { near: [1, 2] } }", "{ furnish: { free: { near: [1, 2] } }, position: [0, 0, 0]")],
                "objects.rug: pose: position: unknown field `position`, expected `furnish` or `stations`".into(),
            ),
            (
                &[(r#"stations: [{ landmark: "top", side: "+y", depth_m: 0.6, width_m: 0.8 }]"#, "stations: 1")],
                "objects.rug: pose: stations: invalid type: integer `1`, expected a sequence".into(),
            ),
        ];
        for (edits, expected) in cases {
            assert_eq!(refusal(edits), expected, "{edits:?}");
        }
    }

    #[test]
    fn a_scale_of_another_form_is_refused_by_its_object() {
        let cases: [(&[(&str, &str)], &str); 9] = [
            (
                &[(
                    "scale: { min: 0.6, max: 1.0 }",
                    "scale: { min: 1.2, max: 1.0 }",
                )],
                "objects.desk: scale: the scale min 1.2 is above the max 1; write a min no higher than the max",
            ),
            (
                &[(
                    "scale: { min: 0.6, max: 1.0 }",
                    "scale: { min: 0, max: 1.0 }",
                )],
                "objects.desk: scale: the scale min 0 is not above 0; write a scale above 0",
            ),
            (
                &[("scale: { min: 0.6, max: 1.0 }", "scale: { min: 0.6 }")],
                "objects.desk: scale: missing field `max`",
            ),
            (
                &[(
                    "scale: { min: 0.6, max: 1.0 }",
                    "scale: { min: 0.6, max: 1.0, step: 0.1 }",
                )],
                "objects.desk: scale: step: unknown field `step`, expected `min` or `max`",
            ),
            (
                &[(
                    "scale: { min: 0.6, max: 1.0 }",
                    "scale: { range: [0.6, 1.0] }",
                )],
                "objects.desk: scale: the solver picks the scale of an object whose pose is `furnish` within its range; write one scale or `{ min, max }`, not a sampler",
            ),
            (
                &[(
                    "scale: { min: 0.6, max: 1.0 }",
                    "scale: { one_of: [0.6, 1.0] }",
                )],
                "objects.desk: scale: the solver picks the scale of an object whose pose is `furnish` within its range; write one scale or `{ min, max }`, not a sampler",
            ),
            (
                &[("scale: 0.9", "scale: 0")],
                "objects.armchair: scale: the scale 0 is not above 0; write a scale above 0, or leave it out for 1",
            ),
            (
                &[("scale: 0.9", "scale: \"large\"")],
                "objects.armchair: scale: the scale of an object whose pose is `furnish` is a number or `{ min, max }`",
            ),
            (
                &[("scale: 0.9", "scale: { min: 0.8, max: 1 }")],
                "objects.armchair: scale: is a range from 0.8 to 1, and the solver picks the scale of a piece by the robots of its stations; give the piece a station of a robot, or write one scale",
            ),
        ];
        for (edits, expected) in cases {
            assert_eq!(refusal(edits), expected, "{edits:?}");
        }
        let rug_range = refusal(&[(
            r#"asset: "object/rug","#,
            r#"asset: "object/rug", scale: { min: 0.5, max: 1 },"#,
        )]);
        assert!(
            rug_range.starts_with("objects.rug: scale: is a range from 0.5 to 1"),
            "a station of a size measures no scale: {rug_range}"
        );
        let one_scale = room_with(
            r#"asset: "object/shelf","#,
            r#"asset: "object/shelf", scale: { min: 1.1, max: 1.1 },"#,
        );
        let placements = placements_of(&WorldPattern::parse(&one_scale).unwrap(), 3);
        assert_eq!(
            piece(pieces_of(&placements), "shelf").scale,
            ScaleRange { min: 1.1, max: 1.1 }
        );
    }

    #[test]
    fn a_scale_range_or_a_furnish_pose_elsewhere_is_refused_by_its_entry() {
        let range = "a scale `{ min, max }` is for an object whose pose is `furnish`, whose scale the solver picks; write one scale, or `{ range }` or `{ one_of }` of scales";
        let elsewhere = "`furnish` lays out an object of `objects` that names its asset; a scene object and an entry of an item kind take a pose, `on` or `in`";
        let cases: [(&[(&str, &str)], String); 5] = [
            (
                &[(r#"asset: "object/lamp","#, r#"asset: "object/lamp", scale: { min: 0.5, max: 1 },"#)],
                format!("objects.lamp: scale: {range}"),
            ),
            (
                &[("    lamp:", "    books: { item_kind: \"book\", count: 2, pose: { in: \"aisle\" }, scale: { min: 1, max: 2 } },\n    lamp:")],
                format!("objects.books: scale: {range}"),
            ),
            (
                &[("    lamp:", "    books: { item_kind: \"book\", count: 2, pose: { furnish: { free: {} } } },\n    lamp:")],
                format!("objects.books: pose: {elsewhere}"),
            ),
            (
                &[("  robots: {", "  scene_objects: { sofa: { pose: { furnish: { free: {} } } } },\n  robots: {")],
                format!("scene_objects.sofa: pose: {elsewhere}"),
            ),
            (
                &[("  robots: {", "  scene_objects: { desk: { removed: true } },\n  robots: {")],
                "`desk` is an object name and a scene object name; the two share one namespace, so give the object another name".into(),
            ),
        ];
        for (edits, expected) in cases {
            assert_eq!(refusal(edits), expected, "{edits:?}");
        }
        let concrete = format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "one_rug", revision: 1 }}, catalogue: "{RELEASE}", scene: {{ asset: "scene/empty_room" }},
              objects: {{ rug: {{ asset: "object/rug", pose: {{ furnish: {{ free: {{}} }} }} }} }} }}"#
        );
        assert_eq!(
            World::parse(&concrete).unwrap_err().to_string(),
            "objects.rug: holds a sampler, so the world is a world pattern; expand it with a seed, or write each value"
        );
    }

    #[test]
    fn the_rules_across_furnish_poses_are_refused_naming_the_object_or_the_robot() {
        let cases: [(&[(&str, &str)], &str); 13] = [
            (
                &[("{ object: \"counter\", gap_m", "{ object: \"lamp\", gap_m")],
                "objects.shelf.pose: `beside` names `lamp`, which is no object of the pattern whose pose is `furnish`; name such an object",
            ),
            (
                &[(
                    "{ object: \"desk\", distance_m",
                    "{ object: \"sofa\", distance_m",
                )],
                "objects.armchair.pose: `facing` names `sofa`, which is no object of the pattern whose pose is `furnish`; name such an object",
            ),
            (
                &[(
                    "{ object: \"desk\", distance_m",
                    "{ object: \"armchair\", distance_m",
                )],
                "objects.armchair.pose: `facing` names `armchair` itself; name another object whose pose is `furnish`",
            ),
            (
                &[("{ object: \"counter\", gap_m", "{ object: \"rug\", gap_m")],
                "objects.shelf.pose: `beside` names `rug`, whose rule is `free`; a piece stands beside a piece against a wall, so give `rug` the rule `against_wall` or `in_corner`",
            ),
            (
                &[(
                    "{ object: \"counter\", gap_m",
                    "{ object: \"armchair\", gap_m",
                )],
                "objects.shelf.pose: `beside` names `armchair`, whose rule is `facing`; a piece stands beside a piece against a wall, so give `armchair` the rule `against_wall` or `in_corner`",
            ),
            (
                &[(
                    DESK_RULE,
                    "{ facing: { object: \"armchair\", distance_m: 1.5 } }",
                )],
                "the furnish poses of `armchair`, `desk` place each other in a cycle of `beside` and `facing`; give one of them `against_wall`, `in_corner` or `free`",
            ),
            (
                &[(
                    ALPHA_STATION,
                    r#"{ landmark: "top", side: "-y", robot: "zulu" }"#,
                )],
                "objects.desk.pose: a station names the robot `zulu`, which is no robot of the pattern; name a robot of `robots`",
            ),
            (
                &[(
                    r#"side: "+x", robot: "bravo""#,
                    r#"side: "+x", robot: "alpha""#,
                )],
                "objects.desk.pose: a station names the robot `alpha`, which a station of `counter` names too; a robot stands at one station",
            ),
            (
                &[(
                    ALPHA_STAND,
                    r#"stand: { object: "desk", landmark: "top", side: "+x" }"#,
                )],
                r#"robots.alpha.stand: a station of `desk` names `alpha`, so the robot stands at the side `-y` of `top`: write its stand { object: "desk", landmark: "top", side: "-y" }, without an offset"#,
            ),
            (
                &[(
                    ALPHA_STAND,
                    r#"stand: { object: "desk", landmark: "top", side: "-y", offset: { ahead_m: 0.1 } }"#,
                )],
                r#"robots.alpha.stand: a station of `desk` names `alpha`, so the robot stands at the side `-y` of `top`: write its stand { object: "desk", landmark: "top", side: "-y" }, without an offset"#,
            ),
            (
                &[(
                    ALPHA_STAND,
                    r#"stand: { object: "desk", landmark: "top", side: "-y", offset: { side_m: { range: [-0.1, 0.1] } } }"#,
                )],
                r#"robots.alpha.stand: a station of `desk` names `alpha`, so the robot stands at the side `-y` of `top`: write its stand { object: "desk", landmark: "top", side: "-y" }, without an offset"#,
            ),
            (
                &[(&format!(", {ALPHA_STAND}"), "")],
                r#"robots.alpha.stand: a station of `desk` names `alpha`, so the robot stands at the side `-y` of `top`: write its stand { object: "desk", landmark: "top", side: "-y" }, without an offset"#,
            ),
            (
                &[(
                    r#"stand: { position: { in: "aisle" }, yaw: 0 }"#,
                    r#"stand: { object: "rug", landmark: "top", side: "-x" }"#,
                )],
                r#"robots.courier.stand: stands at `rug`, whose pose is `furnish`, and no station of `rug` names `courier`; add the station { landmark, side, robot: "courier" } to `rug`, so that the layout leaves room for the robot"#,
            ),
        ];
        for (edits, expected) in cases {
            assert_eq!(refusal(edits), expected, "{edits:?}");
        }
        let three = refusal(&[
            (
                DESK_RULE,
                "{ facing: { object: \"rug\", distance_m: 1.5 } }",
            ),
            (
                "{ free: { near: [1, 2] } }",
                "{ facing: { object: \"armchair\", distance_m: 1 } }",
            ),
        ]);
        assert!(
            three.starts_with(
                "the furnish poses of `armchair`, `desk`, `rug` place each other in a cycle"
            ),
            "{three}"
        );
        let a_robot_that_draws_its_limbs = room_with(
            r#"alpha: { join: { option: "openarm_sim" }, "#,
            r#"alpha: { join: { option: "openarm_sim" }, limbs: { grippers: { gripper: { range: [0, 1] } } }, "#,
        );
        WorldPattern::parse(&a_robot_that_draws_its_limbs).unwrap();
    }

    /// A piece with a station stands still: its physics is `static` or left
    /// out, which the solver makes static. A piece without one takes any.
    #[test]
    fn a_piece_with_a_station_that_can_move_is_refused() {
        let moving = |object: &str, physics: &str| {
            let asset = format!("asset: \"object/{object}\",");
            refusal(&[(&asset, &format!("{asset} physics: {physics},"))])
        };
        let words = "and a piece with a station stands still for the robot that works it; write `static`, or leave `physics` out";
        assert_eq!(
            refusal(&[(r#"physics: "static""#, r#"physics: "dynamic""#)]),
            format!("objects.counter: physics: can be `dynamic`, {words}")
        );
        assert_eq!(
            refusal(&[(
                r#"physics: "static""#,
                r#"physics: { one_of: ["static", "none"] }"#
            )]),
            format!("objects.counter: physics: can be `none`, {words}")
        );
        assert_eq!(
            moving("rug", "\"dynamic\""),
            format!("objects.rug: physics: can be `dynamic`, {words}")
        );
        let shelf = room_with(
            r#"asset: "object/shelf","#,
            r#"asset: "object/shelf", physics: "dynamic","#,
        );
        let pattern = WorldPattern::parse(&shelf).unwrap();
        assert_eq!(
            pattern.physics_of(&name("shelf")),
            Some(BTreeSet::from([Some(Physics::Dynamic)]))
        );
    }

    #[test]
    fn a_pattern_with_furnish_poses_writes_as_a_document_that_reads_back() {
        let pattern = room();
        let written = WorldPattern::parse(&pattern.to_json5().unwrap()).unwrap();
        assert_eq!(written, pattern);
    }

    /// The fixed stations of the room: one at `bench`, an object with a
    /// fixed pose, and one at `window_sill`, a scene object.
    const FIXED_STATIONS: &str = r#"stations: [
    { object: "bench", landmark: "top", side: "-y", depth_m: 0.7, width_m: 0.9 },
    { object: "window_sill", landmark: "top", side: "+x", depth_m: 0.5, width_m: 0.6 },
  ],
  regions:"#;
    const BENCH: &str = r#"bench: { asset: "object/bench", pose: { position: [4, 0, 0], yaw: 0 } },
    lamp:"#;

    /// The room with the bench and the fixed stations.
    fn room_with_fixed_stations() -> String {
        room_with("lamp:", BENCH).replace("regions:", FIXED_STATIONS)
    }

    #[test]
    fn fixed_stations_read_with_the_pattern_and_write_back_the_same() {
        let pattern = WorldPattern::parse(&room_with_fixed_stations()).unwrap();
        assert_eq!(
            pattern.stations,
            [
                FixedStation {
                    object: name("bench"),
                    landmark: "top".into(),
                    side: Side::NegY,
                    depth_m: 0.7,
                    width_m: 0.9,
                },
                FixedStation {
                    object: name("window_sill"),
                    landmark: "top".into(),
                    side: Side::PosX,
                    depth_m: 0.5,
                    width_m: 0.6,
                },
            ]
        );
        let written = pattern.to_json5().unwrap();
        assert!(written.contains("stations"), "{written}");
        assert_eq!(WorldPattern::parse(&written).unwrap(), pattern);
        assert!(room().stations.is_empty());
        let placements = placements_of(&pattern, 3);
        let world = placements
            .expansion()
            .world(&placed(placements.sites()))
            .unwrap();
        let world_text = world.to_json5().unwrap();
        assert!(!world_text.contains("stations"), "{world_text}");
    }

    #[test]
    fn a_pattern_gives_each_scene_and_each_asset_of_an_object_that_it_can_draw() {
        let pattern = room();
        let scene = |asset: &str| crate::Scene {
            asset: asset.into(),
            scale: 1.0,
        };
        assert_eq!(pattern.scenes(), [scene("scene/empty_room")]);
        let rooms = room_with(
            r#"scene: { asset: "scene/empty_room" }"#,
            r#"scene: { asset: { one_of: ["scene/empty_room", "scene/bare_room"] } }"#,
        );
        assert_eq!(
            WorldPattern::parse(&rooms).unwrap().scenes(),
            [scene("scene/empty_room"), scene("scene/bare_room")]
        );
        let assets: BTreeMap<String, Vec<String>> = pattern
            .object_assets()
            .into_iter()
            .map(|(name, assets)| (name.to_string(), assets.into_iter().collect()))
            .collect();
        assert_eq!(
            assets["desk"],
            ["object/office_desk", "object/standing_desk"]
        );
        assert_eq!(assets["lamp"], ["object/lamp"]);
        assert_eq!(assets["mug"], ["object/mug"]);
        assert_eq!(assets.len(), 9);
        let item_kind = room_with(
            r#"tray: { asset: "object/tray", pose: { in: "aisle" } },"#,
            r#"tray: { item_kind: "tray", count: 2, pose: { in: "aisle" } },"#,
        );
        let pattern = WorldPattern::parse(&item_kind).unwrap();
        assert!(!pattern.object_assets().contains_key(&name("tray")));
    }

    #[test]
    fn a_pattern_gives_the_stations_of_its_furnish_poses_by_piece() {
        let pattern = room();
        let stations: Vec<(&str, &str, Side)> = pattern
            .furnish_stations()
            .into_iter()
            .map(|(piece, station)| (piece.as_str(), station.landmark.as_str(), station.side))
            .collect();
        assert_eq!(
            stations,
            [
                ("counter", "top", Side::PosX),
                ("desk", "top", Side::NegY),
                ("rug", "top", Side::PosY),
            ]
        );
    }

    #[test]
    fn a_concrete_world_refuses_fixed_stations() {
        let text = format!(
            r#"{{
  simulation_schema: "world/v1",
  manifest: {{ name: "bench_room", revision: 1 }},
  catalogue: "{RELEASE}",
  scene: {{ asset: "scene/empty_room" }},
  objects: {{ bench: {{ asset: "object/bench", pose: {{ position: [4, 0, 0], yaw: 0 }} }} }},
  stations: [{{ object: "bench", landmark: "top", side: "-y", depth_m: 0.7, width_m: 0.9 }}],
}}"#
        );
        let refusal = crate::WorldDocument::parse(&text)
            .unwrap()
            .resolve(&[])
            .unwrap_err();
        assert_eq!(
            refusal.to_string(),
            "stations: is a field of a world pattern; resolve the world as a pattern, or leave the field out"
        );
        assert!(WorldPattern::parse(&text).unwrap().is_concrete());
    }

    #[test]
    fn a_fixed_station_of_another_form_or_on_an_object_that_moves_is_refused() {
        let refused = |from: &str, to: &str| {
            let text = room_with_fixed_stations();
            assert_eq!(text.matches(from).count(), 1, "{from}");
            WorldPattern::parse(&text.replace(from, to))
                .unwrap_err()
                .to_string()
        };
        let bench_station =
            r#"{ object: "bench", landmark: "top", side: "-y", depth_m: 0.7, width_m: 0.9 }"#;
        let on = |object: &str| bench_station.replace(r#""bench""#, &format!(r#""{object}""#));
        let cases = [
            (
                bench_station.to_owned(),
                bench_station.replace("depth_m: 0.7", "depth_m: 0"),
                "stations[0]: the depth_m 0 is not above 0; write a depth above 0 metres",
            ),
            (
                bench_station.to_owned(),
                bench_station.replace("width_m: 0.9", "width_m: -1"),
                "stations[0]: the width_m -1 is not above 0; write a width above 0 metres",
            ),
            (
                bench_station.to_owned(),
                bench_station.replace(r#"landmark: "top""#, r#"landmark: " ""#),
                "stations[0]: the landmark of a station is blank; name the landmark of a work surface of the object, such as `top`",
            ),
            (
                bench_station.to_owned(),
                bench_station.replace("width_m: 0.9", "width_m: 0.9, robot: \"alpha\""),
                "stations[0].robot: unknown field `robot`, expected one of `object`, `landmark`, `side`, `depth_m`, `width_m`",
            ),
            (
                bench_station.to_owned(),
                format!("{bench_station}, {}", bench_station.replace("0.7", "0.4")),
                "stations: two stations work the side `-y` of `top` of `bench`; give each station a side of its own",
            ),
            (
                bench_station.to_owned(),
                on("desk"),
                "stations: a station works `desk`, and its pose is `furnish`; a station works an object of `objects` with a fixed pose, or a scene object",
            ),
            (
                bench_station.to_owned(),
                on("mug"),
                "stations: a station works `mug`, and its pose holds a sampler; a station works an object of `objects` with a fixed pose, or a scene object",
            ),
            (
                bench_station.to_owned(),
                on("lamp"),
                "stations: a station works `lamp`, and its pose holds a sampler; a station works an object of `objects` with a fixed pose, or a scene object",
            ),
        ];
        for (from, to, expected) in &cases {
            assert_eq!(refused(from, to), *expected, "{to}");
        }
        let removed = room_with_fixed_stations().replace(
            "objects: {",
            "scene_objects: { window_sill: { removed: true } },\n  objects: {",
        );
        assert_eq!(
            WorldPattern::parse(&removed).unwrap_err().to_string(),
            "stations: a station works `window_sill`, and the pattern removes it; a station works an object of `objects` with a fixed pose, or a scene object"
        );
        let item_kind = room_with_fixed_stations().replace(
            BENCH,
            r#"bench: { item_kind: "bench", count: 1, pose: { position: [4, 0, 0], yaw: 0 } },
    lamp:"#,
        );
        assert_eq!(
            WorldPattern::parse(&item_kind).unwrap_err().to_string(),
            "stations: a station works `bench`, and it is an entry of an item kind; a station works an object of `objects` with a fixed pose, or a scene object"
        );
    }

    #[test]
    fn a_number_of_a_rule_a_station_or_a_scale_out_of_its_range_is_refused() {
        assert_eq!(above_zero("depth_m", "a depth", 0.4).unwrap(), 0.4);
        assert_eq!(at_least_zero("gap_m", "a gap", 0.0).unwrap(), 0.0);
        for refused in [0.0, -1.0, f64::NAN] {
            assert!(
                above_zero("depth_m", "a depth", refused).is_err(),
                "{refused}"
            );
        }
        for refused in [-0.01, f64::NAN] {
            assert!(
                at_least_zero("gap_m", "a gap", refused).is_err(),
                "{refused}"
            );
        }
        assert_eq!(
            ScaleRange::new(f64::NAN, 1.0).unwrap_err().to_string(),
            "the scale min NaN is not above 0; write a scale above 0"
        );
        let range = ScaleRange::new(0.5, 1.5).unwrap();
        assert!(range.contains(0.5) && range.contains(1.5) && range.contains(1.0));
        assert!(!range.contains(0.49) && !range.contains(1.51));
    }
}
