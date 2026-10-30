//! Properties on drawn worlds, records and judges: a world survives its text,
//! the record of a world with its robots is that world, a document that
//! extends a base merges each key as its entry says and its daylight as a
//! whole, a robot name that
//! another begins is refused by the pair, judges survive their text, and a
//! seed expands a world pattern into the same world each time, with each
//! drawn value inside its sampler.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;
use std::fmt::Debug;

use hegel::TestCase;
use hegel::generators::{self as gs, Generator, PrintableGenerator};
use serde::Serialize;
use serde_json::{Value, json};
use simulation_documents::{
    Band, Bounds, Catalogue, Check, Cone, ContentsOn, Control, Count, Date, Daylight, Digest,
    Error, Feature, Finish, Holding, Join, Judge, Landmark, LandmarkOn, Light, Location, Manifest,
    Material, Name, NearPoint, Object, Offset, Physics, Place, Placement, Point, Pose, Reachable,
    Region, Revision, Robot, RobotName, Scene, SceneObject, Seed, SetupRecord, StackRobot, Stand,
    Target, TimeOfDay, Weather, WithinM, World, WorldDocument, WorldPattern,
};

/// Names of objects, scene objects, locations, poses and regions: over the
/// characters a name allows, short enough to collide.
fn names() -> impl PrintableGenerator<Name> {
    gs::text()
        .alphabet("abz_1")
        .min_size(1)
        .max_size(4)
        .map(|text| Name::parse(&text).expect("the alphabet is a name's"))
}

/// Short free texts: labels, ids, assets.
fn labels() -> impl PrintableGenerator<String> {
    gs::text().alphabet("ab 1").min_size(1).max_size(5)
}

/// Options, features and core nodes of a stack: short texts that are not
/// blank.
fn stack_texts() -> impl PrintableGenerator<String> {
    gs::text().alphabet("ab1").min_size(1).max_size(5)
}

/// Robot names: over the characters a robot name allows, short enough to
/// collide.
fn robot_names() -> impl PrintableGenerator<RobotName> {
    gs::text()
        .alphabet("ab_-1")
        .min_size(1)
        .max_size(4)
        .map(|text| RobotName::parse(&text).expect("the alphabet is a robot name's"))
}

/// Finite coordinates within a few metres.
fn coordinates() -> impl PrintableGenerator<f64> {
    gs::floats::<f64>().min_value(-5.0).max_value(5.0)
}

fn points() -> impl PrintableGenerator<Point> {
    gs::arrays(coordinates())
}

fn yaws() -> impl PrintableGenerator<f64> {
    gs::floats::<f64>().min_value(-PI).max_value(PI)
}

/// A unit quaternion: an axis with one component at least 0.1 in magnitude,
/// normalized, and an angle.
#[hegel::composite]
fn unit_quaternions(tc: &TestCase) -> [f64; 4] {
    let mut axis: [f64; 3] = tc.draw(gs::arrays(
        gs::floats::<f64>().min_value(-1.0).max_value(1.0),
    ));
    let strong = tc.draw(gs::integers::<usize>().min_value(0).max_value(2));
    axis[strong] = tc.draw(gs::sampled_from(&[-1.0, 1.0]))
        * tc.draw(gs::floats::<f64>().min_value(0.1).max_value(1.0));
    let norm = axis.iter().map(|c| c * c).sum::<f64>().sqrt();
    let half = tc.draw(gs::floats::<f64>().min_value(0.0).max_value(PI)) / 2.0;
    let s = half.sin() / norm;
    [axis[0] * s, axis[1] * s, axis[2] * s, half.cos()]
}

#[hegel::composite]
fn poses(tc: &TestCase) -> Pose {
    match tc.draw(gs::integers::<u8>().min_value(0).max_value(2)) {
        0 => Pose::Oriented {
            position: tc.draw(points()),
            orientation: tc.draw(unit_quaternions()),
        },
        1 => Pose::Yawed {
            position: tc.draw(points()),
            yaw: tc.draw(yaws()),
        },
        _ => Pose::At(tc.draw(names())),
    }
}

#[hegel::composite]
fn offsets(tc: &TestCase) -> Offset {
    Offset {
        ahead_m: tc.draw(coordinates()),
        side_m: tc.draw(coordinates()),
    }
}

#[hegel::composite]
fn placements(tc: &TestCase) -> Placement {
    Placement {
        position: tc.draw(points()),
        yaw: tc.draw(yaws()),
    }
}

#[hegel::composite]
fn stands(tc: &TestCase) -> Stand {
    match tc.draw(gs::integers::<u8>().min_value(0).max_value(2)) {
        0 => Stand::Surface {
            surface: tc.draw(labels()),
            offset: tc.draw(offsets()),
        },
        1 => Stand::At {
            at: tc.draw(names()),
            offset: tc.draw(offsets()),
        },
        _ => Stand::Placed(tc.draw(placements())),
    }
}

fn controls() -> impl PrintableGenerator<BTreeMap<String, f64>> {
    gs::btree_maps(labels(), coordinates()).max_size(2)
}

#[hegel::composite]
fn objects(tc: &TestCase) -> Object {
    Object {
        asset: format!("object/{}", tc.draw(names())),
        pose: tc.draw(poses()),
        scale: tc.draw(gs::floats::<f64>().min_value(0.1).max_value(3.0)),
        physics: tc.draw(gs::optional(gs::sampled_from(&[
            Physics::Dynamic,
            Physics::Static,
            Physics::None,
        ]))),
        mass: tc.draw(gs::optional(
            gs::floats::<f64>().min_value(0.0).max_value(20.0),
        )),
        controls: tc.draw(controls()),
    }
}

#[hegel::composite]
fn scene_objects(tc: &TestCase) -> SceneObject {
    if tc.draw(gs::booleans()) {
        return SceneObject::Removed;
    }
    SceneObject::Placed {
        pose: tc.draw(poses()),
        controls: tc.draw(controls()),
    }
}

/// A light with at least one setting.
#[hegel::composite]
fn lights(tc: &TestCase) -> Light {
    let light = Light {
        intensity: tc.draw(gs::optional(
            gs::floats::<f64>().min_value(0.0).max_value(1000.0),
        )),
        color: tc.draw(gs::optional(gs::arrays(
            gs::floats::<f64>().min_value(0.0).max_value(1.0),
        ))),
        position: tc.draw(gs::optional(points())),
        direction: tc.draw(gs::optional(points())),
        cone: tc.draw(gs::optional(gs::tuples2(yaws(), yaws()).map(
            |(inner_angle, outer_angle)| Cone {
                inner_angle,
                outer_angle,
            },
        ))),
        orientation: tc.draw(gs::optional(unit_quaternions())),
    };
    if light == Light::EMPTY {
        return Light {
            intensity: Some(tc.draw(gs::floats::<f64>().min_value(0.0).max_value(1000.0))),
            ..Light::EMPTY
        };
    }
    light
}

/// A material with at least one setting.
#[hegel::composite]
fn materials(tc: &TestCase) -> Material {
    let unit = || gs::floats::<f64>().min_value(0.0).max_value(1.0);
    let material = Material {
        color: tc.draw(gs::optional(gs::arrays(unit()))),
        finish: tc.draw(gs::optional(gs::tuples2(unit(), unit()).map(
            |(metallic, roughness)| Finish {
                metallic,
                roughness,
            },
        ))),
    };
    if material == Material::EMPTY {
        return Material {
            color: Some(tc.draw(gs::arrays(unit()))),
            finish: None,
        };
    }
    material
}

/// A day of the calendar: a drawn day the month has not is the month's
/// 28th.
#[hegel::composite]
fn dates(tc: &TestCase) -> Date {
    let year = tc.draw(gs::integers::<i32>().min_value(1).max_value(9999));
    let month = tc.draw(gs::integers::<u32>().min_value(1).max_value(12));
    let day = tc.draw(gs::integers::<u32>().min_value(1).max_value(31));
    Date::new(year, month, day).unwrap_or_else(|_| Date::new(year, month, 28).unwrap())
}

fn times() -> impl PrintableGenerator<TimeOfDay> {
    gs::integers::<u32>()
        .min_value(0)
        .max_value(86_399)
        .map(|seconds| TimeOfDay::from_seconds(seconds).unwrap())
}

/// A daylight with at least one setting.
#[hegel::composite]
fn daylights(tc: &TestCase) -> Daylight {
    let rates = || gs::floats::<f64>().min_value(0.0).max_value(3600.0);
    let daylight = Daylight {
        date: tc.draw(gs::optional(dates())),
        time: tc.draw(gs::optional(times())),
        rate: tc.draw(gs::optional(rates())),
        weather: tc.draw(gs::optional(
            gs::tuples2(
                gs::floats::<f64>().min_value(0.0).max_value(1.0),
                gs::floats::<f64>().min_value(0.0).max_value(50.0),
            )
            .map(|(cloud_cover, rain_mm_h)| Weather {
                cloud_cover,
                rain_mm_h,
            }),
        )),
    };
    if daylight == Daylight::EMPTY {
        return Daylight {
            rate: Some(tc.draw(rates())),
            ..Daylight::EMPTY
        };
    }
    daylight
}

#[hegel::composite]
fn locations(tc: &TestCase) -> Location {
    match tc.draw(gs::integers::<u8>().min_value(0).max_value(3)) {
        0 => Location::Landmark {
            object: tc.draw(names()),
            landmark: tc.draw(labels()),
        },
        1 => Location::Surface(tc.draw(labels())),
        2 => Location::At(tc.draw(names())),
        _ => Location::In(tc.draw(names())),
    }
}

#[hegel::composite]
fn region_boxes(tc: &TestCase) -> Region {
    Region {
        centre: tc.draw(points()),
        size: tc.draw(gs::arrays(
            gs::floats::<f64>().min_value(0.01).max_value(3.0),
        )),
        yaw: tc.draw(yaws()),
    }
}

/// A join whose features are named once each.
#[hegel::composite]
fn joins(tc: &TestCase) -> Join {
    let names: BTreeSet<String> = tc.draw(gs::btree_sets(stack_texts()).max_size(2));
    let features = names
        .into_iter()
        .map(|name| match tc.draw(gs::booleans()) {
            true => Feature::Named(name),
            false => Feature::Placed {
                feature: name,
                core_node: tc.draw(stack_texts()),
            },
        })
        .collect();
    Join {
        option: tc.draw(stack_texts()),
        features,
        core_node: tc.draw(gs::optional(stack_texts())),
    }
}

/// Whether `shorter` followed by `_` begins `longer`.
fn begins(shorter: &RobotName, longer: &RobotName) -> bool {
    longer.to_string().starts_with(&format!("{shorter}_"))
}

/// Robot names that stand together: none followed by `_` begins another.
#[hegel::composite]
fn standing_robot_names(tc: &TestCase) -> Vec<RobotName> {
    let drawn: BTreeSet<RobotName> = tc.draw(gs::btree_sets(robot_names()).max_size(3));
    let mut kept: Vec<RobotName> = Vec::new();
    for name in drawn {
        if !kept
            .iter()
            .any(|other| begins(other, &name) || begins(&name, other))
        {
            kept.push(name);
        }
    }
    kept
}

#[hegel::composite]
fn manifests(tc: &TestCase) -> Manifest {
    Manifest {
        name: format!("world_{}", tc.draw(names())),
        revision: Revision::new(tc.draw(gs::integers::<u32>().min_value(1).max_value(50))).unwrap(),
    }
}

#[hegel::composite]
fn catalogues(tc: &TestCase) -> Catalogue {
    if tc.draw(gs::booleans()) {
        return Catalogue::Local;
    }
    let hex = tc.draw(
        gs::text()
            .alphabet("0123456789abcdef")
            .min_size(64)
            .max_size(64),
    );
    Catalogue::Release(Digest::parse(&hex).expect("64 lowercase hex digits"))
}

#[hegel::composite]
fn scenes(tc: &TestCase) -> Scene {
    Scene {
        asset: format!("scene/{}", tc.draw(names())),
        scale: tc.draw(gs::floats::<f64>().min_value(0.5).max_value(2.0)),
    }
}

/// A concrete world: its object names and scene object names disjoint, its
/// robot names standing together, each region of its locations declared.
#[hegel::composite]
fn worlds(tc: &TestCase) -> World {
    let scene_objects: BTreeMap<Name, SceneObject> =
        tc.draw(gs::btree_maps(names(), scene_objects()).max_size(3));
    let objects: BTreeMap<Name, Object> = tc
        .draw(gs::btree_maps(names(), objects()).max_size(3))
        .into_iter()
        .filter(|(name, _)| !scene_objects.contains_key(name))
        .collect();
    let robots = tc
        .draw(standing_robot_names())
        .into_iter()
        .map(|name| {
            (
                name,
                Robot {
                    join: tc.draw(joins()),
                    stand: tc.draw(gs::optional(stands())),
                },
            )
        })
        .collect();
    let locations: BTreeMap<Name, Location> =
        tc.draw(gs::btree_maps(names(), locations()).max_size(3));
    let mut regions: BTreeMap<Name, Region> =
        tc.draw(gs::btree_maps(names(), region_boxes()).max_size(2));
    for location in locations.values() {
        if let Location::In(region) = location
            && !regions.contains_key(region)
        {
            regions.insert(region.clone(), tc.draw(region_boxes()));
        }
    }
    World {
        manifest: tc.draw(manifests()),
        catalogue: tc.draw(catalogues()),
        scene: tc.draw(scenes()),
        robots,
        scene_objects,
        objects,
        lighting: tc.draw(gs::btree_maps(labels(), lights()).max_size(2)),
        materials: tc.draw(gs::btree_maps(labels(), materials()).max_size(2)),
        daylight: tc.draw(gs::optional(daylights())),
        regions,
        locations,
    }
}

/// The robots of `world` as the stack lists them: a robot with a stand has a
/// body, one without may or may not.
fn listed(tc: &TestCase, world: &World) -> Vec<StackRobot> {
    world
        .robots
        .iter()
        .map(|(name, robot)| StackRobot {
            name: name.clone(),
            join: robot.join.clone(),
            body: robot.stand.is_some() || tc.draw(gs::booleans()),
        })
        .collect()
}

#[hegel::test]
fn a_world_survives_its_text(tc: TestCase) {
    let world = tc.draw(worlds());
    let text = world.to_json5().unwrap();
    let parsed = World::parse(&text).unwrap_or_else(|error| panic!("{error}\n{text}"));
    assert_eq!(parsed, world);
    assert_eq!(
        parsed.document().unwrap().digest(),
        world.document().unwrap().digest()
    );
    assert!(parsed.same_setup_as(&world));
}

#[hegel::test]
fn a_robot_name_that_another_begins_is_refused_by_the_pair(tc: TestCase) {
    let mut world = tc.draw(worlds());
    let shorter = tc.draw(robot_names());
    let suffix = tc.draw(gs::text().alphabet("ab").min_size(1).max_size(2));
    let longer = RobotName::parse(&format!("{shorter}_{suffix}")).unwrap();
    let robot = Robot {
        join: tc.draw(joins()),
        stand: None,
    };
    world.robots = BTreeMap::from([(shorter.clone(), robot.clone()), (longer.clone(), robot)]);
    let refused = World::parse(&world.to_json5().unwrap()).unwrap_err();
    assert_eq!(
        refused,
        Error::RobotNamePrefix {
            shorter: shorter.to_string(),
            longer: longer.to_string()
        }
    );
    assert!(
        refused
            .to_string()
            .contains(&format!("`{shorter}` and `{longer}`")),
        "{refused}"
    );
}

#[hegel::test]
fn the_record_of_a_world_with_its_robots_is_the_world(tc: TestCase) {
    let world = tc.draw(worlds());
    let record = SetupRecord::built(&world);
    assert_eq!(
        SetupRecord::parse(&record.to_json5().unwrap()).unwrap(),
        record
    );
    let composed = World::from_setup(
        world.manifest.clone(),
        world.catalogue.clone(),
        &record,
        &listed(&tc, &world),
    )
    .unwrap();
    assert_eq!(composed, world);
}

/// What a document that extends a base does to one key of a base map.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Edit {
    Replace,
    Remove,
    Keep,
}

/// A drawn edit of some of `keys`, `Keep` among them only when `kept_allowed`.
fn edits<K: Ord>(
    tc: &TestCase,
    keys: impl Iterator<Item = K>,
    kept_allowed: bool,
) -> BTreeMap<K, Edit> {
    keys.filter_map(|key| {
        let edit = tc.draw(gs::sampled_from(&[
            None,
            Some(Edit::Replace),
            Some(Edit::Remove),
            Some(Edit::Keep),
        ]))?;
        (kept_allowed || edit != Edit::Keep).then_some((key, edit))
    })
    .collect()
}

/// The map of a document that extends a base: the entry each edit writes,
/// under its key.
fn edited<K: ToString, V: Serialize>(
    edits: &BTreeMap<K, Edit>,
    replacement: &V,
) -> serde_json::Map<String, Value> {
    edits
        .iter()
        .map(|(key, edit)| {
            let entry = match edit {
                Edit::Replace => serde_json::to_value(replacement).unwrap(),
                Edit::Remove => json!({ "removed": true }),
                Edit::Keep => json!({ "kept": true }),
            };
            (key.to_string(), entry)
        })
        .collect()
}

/// Each entry of `base` after the resolve: as it was, the replacement, the
/// `removed` state of the map or gone, as its edit says.
fn assert_merged<K: Ord + Debug, V: PartialEq + Debug>(
    resolved: &BTreeMap<K, V>,
    base: &BTreeMap<K, V>,
    edits: &BTreeMap<K, Edit>,
    replacement: &V,
    removed: Option<&V>,
) {
    for (key, value) in base {
        let expected = match edits.get(key) {
            None => Some(value),
            Some(Edit::Replace) => Some(replacement),
            Some(Edit::Remove) => removed,
            Some(Edit::Keep) => None,
        };
        assert_eq!(resolved.get(key), expected, "{key:?}");
    }
}

#[hegel::test]
fn a_document_that_extends_a_base_merges_each_key_as_its_entry_says(tc: TestCase) {
    let base = tc.draw(worlds());
    let robot_edits = edits(&tc, base.robots.keys().cloned(), false);
    let object_edits = edits(&tc, base.objects.keys().cloned(), false);
    let scene_edits = edits(&tc, base.scene_objects.keys().cloned(), true);
    let light_edits = edits(&tc, base.lighting.keys().cloned(), false);
    let material_edits = edits(&tc, base.materials.keys().cloned(), false);
    let location_edits = edits(&tc, base.locations.keys().cloned(), false);
    let robot = Robot {
        join: tc.draw(joins()),
        stand: tc.draw(gs::optional(stands())),
    };
    let standing: Vec<&RobotName> = base
        .robots
        .keys()
        .filter(|name| robot_edits.get(*name) != Some(&Edit::Remove))
        .collect();
    let removed: Vec<RobotName> = robot_edits
        .iter()
        .filter(|(_, edit)| **edit == Edit::Remove)
        .map(|(name, _)| name.clone())
        .collect();
    let added_name = |tc: &TestCase| {
        if removed.is_empty() || !tc.draw(gs::booleans()) {
            return tc.draw(robot_names());
        }
        let freed = tc.draw(gs::sampled_from(removed.clone()));
        let suffix = tc.draw(gs::text().alphabet("ab").min_size(1).max_size(2));
        RobotName::parse(&format!("{freed}_{suffix}")).unwrap()
    };
    let new_robot = tc
        .draw(gs::booleans())
        .then(|| added_name(&tc))
        .filter(|name| {
            !base.robots.contains_key(name)
                && standing
                    .iter()
                    .all(|other| !begins(other, name) && !begins(name, other))
        });
    let mut robots = edited(&robot_edits, &robot);
    if let Some(name) = &new_robot {
        robots.insert(name.to_string(), serde_json::to_value(&robot).unwrap());
    }
    let object = tc.draw(objects());
    let scene_object = SceneObject::Placed {
        pose: tc.draw(poses()),
        controls: tc.draw(controls()),
    };
    let light = tc.draw(lights());
    let material = tc.draw(materials());
    let daylight = tc.draw(daylights());
    let daylight_edit = tc
        .draw(gs::sampled_from(&[
            None,
            Some(Edit::Replace),
            Some(Edit::Remove),
        ]))
        .filter(|edit| *edit != Edit::Remove || base.daylight.is_some());
    let location = tc.draw(locations());
    // A region stays while a location of the resolved world names it, and
    // the extending document declares the region of its location when the
    // base has none of that name.
    let region_of = |location: &Location| match location {
        Location::In(region) => Some(region.clone()),
        _ => None,
    };
    let named_regions: BTreeSet<Name> = base
        .locations
        .iter()
        .filter_map(|(key, kept)| match location_edits.get(key) {
            None => region_of(kept),
            Some(Edit::Replace) => region_of(&location),
            Some(Edit::Remove | Edit::Keep) => None,
        })
        .collect();
    let region_edits: BTreeMap<Name, Edit> = edits(&tc, base.regions.keys().cloned(), false)
        .into_iter()
        .filter(|(key, edit)| *edit != Edit::Remove || !named_regions.contains(key))
        .collect();
    let region = tc.draw(region_boxes());
    let mut regions = edited(&region_edits, &region);
    let new_region = region_of(&location).filter(|name| {
        !base.regions.contains_key(name)
            && location_edits.values().any(|edit| *edit == Edit::Replace)
    });
    if let Some(name) = &new_region {
        regions.insert(name.to_string(), serde_json::to_value(region).unwrap());
    }
    let new_object = tc
        .draw(gs::optional(names()))
        .filter(|name| !base.objects.contains_key(name) && !base.scene_objects.contains_key(name));
    let mut objects = edited(&object_edits, &object);
    if let Some(name) = &new_object {
        objects.insert(name.to_string(), serde_json::to_value(&object).unwrap());
    }
    let mut extending = json!({
        "simulation_schema": "world/v1",
        "manifest": { "name": "world_extending", "revision": 1 },
        "extends": { "name": base.manifest.name, "revision": base.manifest.revision.get() },
        "robots": robots,
        "objects": objects,
        "scene_objects": edited(&scene_edits, &scene_object),
        "lighting": edited(&light_edits, &light),
        "materials": edited(&material_edits, &material),
        "regions": regions,
        "locations": edited(&location_edits, &location),
    });
    match daylight_edit {
        None => {}
        Some(Edit::Replace) => extending["daylight"] = serde_json::to_value(&daylight).unwrap(),
        Some(Edit::Remove) => extending["daylight"] = json!({ "removed": true }),
        Some(Edit::Keep) => unreachable!("the daylight takes no `kept`"),
    }
    let resolved = WorldDocument::parse(&extending.to_string())
        .unwrap()
        .resolve(&[WorldDocument::parse(&base.to_json5().unwrap()).unwrap()])
        .unwrap();

    assert_eq!(
        (&resolved.catalogue, &resolved.scene),
        (&base.catalogue, &base.scene)
    );
    assert_merged(&resolved.robots, &base.robots, &robot_edits, &robot, None);
    if let Some(name) = &new_robot {
        assert_eq!(resolved.robots.get(name), Some(&robot));
    }
    assert_merged(
        &resolved.objects,
        &base.objects,
        &object_edits,
        &object,
        None,
    );
    if let Some(name) = &new_object {
        assert_eq!(resolved.objects.get(name), Some(&object));
    }
    assert_merged(
        &resolved.scene_objects,
        &base.scene_objects,
        &scene_edits,
        &scene_object,
        Some(&SceneObject::Removed),
    );
    assert_merged(
        &resolved.lighting,
        &base.lighting,
        &light_edits,
        &light,
        None,
    );
    assert_merged(
        &resolved.materials,
        &base.materials,
        &material_edits,
        &material,
        None,
    );
    assert_merged(
        &resolved.regions,
        &base.regions,
        &region_edits,
        &region,
        None,
    );
    if let Some(name) = &new_region {
        assert_eq!(resolved.regions.get(name), Some(&region));
    }
    assert_merged(
        &resolved.locations,
        &base.locations,
        &location_edits,
        &location,
        None,
    );
    let expected_daylight = match daylight_edit {
        None => base.daylight.as_ref(),
        Some(Edit::Replace) => Some(&daylight),
        Some(Edit::Remove | Edit::Keep) => None,
    };
    assert_eq!(resolved.daylight.as_ref(), expected_daylight);
}

fn within() -> impl PrintableGenerator<WithinM> {
    gs::floats::<f64>()
        .min_value(0.001)
        .max_value(2.0)
        .map(|m| WithinM::try_from(m).unwrap())
}

#[hegel::composite]
fn landmarks(tc: &TestCase) -> Landmark {
    Landmark {
        object: tc.draw(names()),
        name: tc.draw(labels()),
    }
}

#[hegel::composite]
fn targets(tc: &TestCase) -> Target {
    match tc.draw(gs::integers::<u8>().min_value(0).max_value(3)) {
        0 => Target::Object(tc.draw(names())),
        1 => Target::Landmark(tc.draw(landmarks())),
        2 => Target::Region(tc.draw(names())),
        _ => Target::Location(tc.draw(names())),
    }
}

#[hegel::composite]
fn bounds(tc: &TestCase) -> Bounds<u32> {
    let least = tc.draw(gs::integers::<u32>().min_value(0).max_value(9));
    match tc.draw(gs::integers::<u8>().min_value(0).max_value(2)) {
        0 => Bounds {
            at_least: Some(least),
            at_most: None,
        },
        1 => Bounds {
            at_least: None,
            at_most: Some(least),
        },
        _ => Bounds {
            at_least: Some(least),
            at_most: Some(least + tc.draw(gs::integers::<u32>().min_value(0).max_value(5))),
        },
    }
}

#[hegel::composite]
fn bands(tc: &TestCase) -> Band {
    let line = || gs::vecs(points()).min_size(2).max_size(4);
    Band {
        item_kind: tc.draw(labels()),
        between: [tc.draw(line()), tc.draw(line())],
        within_m: tc.draw(within()),
    }
}

#[hegel::composite]
fn checks(tc: &TestCase) -> Check {
    match tc.draw(gs::integers::<u8>().min_value(0).max_value(10)) {
        0 => Check::Count(Count {
            item_kind: tc.draw(labels()),
            place: match tc.draw(targets()) {
                Target::Object(name) | Target::Landmark(Landmark { object: name, .. }) => {
                    Place::Object(name)
                }
                Target::Region(name) => Place::Region(name),
                Target::Location(name) => Place::Location(name),
            },
            bounds: tc.draw(bounds()),
            each_holding: tc.draw(gs::optional(
                gs::tuples2(labels(), gs::integers::<u32>().min_value(1).max_value(5)).map(
                    |(item_kind, at_least)| Holding {
                        item_kind,
                        at_least,
                    },
                ),
            )),
        }),
        1 => Check::LandmarkOn(LandmarkOn {
            landmark: tc.draw(landmarks()),
            on: tc.draw(landmarks()),
            within_m: tc.draw(within()),
        }),
        2 => Check::ContentsOn(ContentsOn {
            object: tc.draw(names()),
            on: tc.draw(names()),
        }),
        3 => {
            let modules: BTreeSet<Name> = tc.draw(gs::btree_sets(names()).min_size(2).max_size(4));
            Check::BeltLine(modules.into_iter().collect::<Vec<_>>().try_into().unwrap())
        }
        4 => Check::InBand(tc.draw(bands())),
        5 => Check::SpansBand(tc.draw(bands())),
        6 => Check::NearPoint(NearPoint {
            object: tc.draw(names()),
            point: tc.draw(points()),
            within_m: tc.draw(within()),
        }),
        7 => Check::Broken(tc.draw(names())),
        8 => Check::Crushed(tc.draw(names())),
        9 => Check::Reachable(Reachable {
            robot: tc.draw(robot_names()),
            target: tc.draw(targets()),
        }),
        _ => {
            let bounds = tc.draw(bounds());
            Check::Control(Control {
                object: tc.draw(names()),
                id: tc.draw(labels()),
                bounds: Bounds {
                    at_least: bounds.at_least.map(f64::from),
                    at_most: bounds.at_most.map(f64::from),
                },
            })
        }
    }
}

fn judges() -> impl PrintableGenerator<Judge> {
    gs::recursive(
        hegel::one_of!(gs::just(Judge::True), checks().map(Judge::Check)),
        |subjudges| {
            gs::tuples2(
                gs::integers::<u8>().min_value(0).max_value(2),
                gs::vecs(subjudges).min_size(1).max_size(3),
            )
            .map(|(kind, mut judges)| {
                Judge::Check(match kind {
                    0 => Check::All(judges),
                    1 => Check::Any(judges),
                    _ => Check::Not(Box::new(judges.swap_remove(0))),
                })
            })
        },
    )
}

#[hegel::test]
fn judges_survive_their_text(tc: TestCase) {
    let judges = tc.draw(gs::vecs(judges()).max_size(4));
    let text = Judge::list_to_json5(&judges).unwrap();
    assert_eq!(
        Judge::parse_list(&text).unwrap_or_else(|error| panic!("{error}\n{text}")),
        judges
    );
}

/// A world pattern with a drawn mass range, a drawn count range of an item
/// kind and a choice of scenes expands, for each seed, into the same world
/// each time, with each value inside its sampler.
#[hegel::test]
fn a_seed_expands_a_pattern_into_one_world_inside_its_samplers(tc: TestCase) {
    let low = tc.draw(gs::floats::<f64>().min_value(0.0).max_value(5.0));
    let high = low + tc.draw(gs::floats::<f64>().min_value(0.0).max_value(5.0));
    let fewest = tc.draw(gs::integers::<u32>().min_value(0).max_value(4));
    let most = fewest + tc.draw(gs::integers::<u32>().min_value(0).max_value(4));
    let text = json!({
        "simulation_schema": "world/v1",
        "manifest": { "name": "drawn", "revision": 1 },
        "catalogue": "local",
        "scene": { "one_of": [{ "asset": "scene/kitchen" }, { "asset": "scene/living_room" }] },
        "objects": {
            "crate": { "asset": "object/crate", "pose": { "position": [0, 0, 0], "yaw": 0 }, "mass": { "range": [low, high] } },
            "books": { "item_kind": "book", "count": { "range": [fewest, most] }, "pose": { "position": [1, 0, 0], "yaw": 0 } },
        },
    });
    let pattern = WorldPattern::parse(&text.to_string()).unwrap();
    let seed = Seed(tc.draw(gs::integers::<u32>()));
    let assets = |_: &str| vec!["object/book_red".to_owned(), "object/book_blue".to_owned()];
    let expand = || {
        pattern
            .expansion(seed, &assets)
            .unwrap()
            .world(&BTreeMap::new())
            .unwrap()
    };
    let world = expand();
    assert_eq!(world, expand());
    let mass = world.objects[&Name::parse("crate").unwrap()].mass.unwrap();
    assert!(low <= mass && mass <= high, "{mass} in [{low}, {high}]");
    let books = world.objects.len() as u32 - 1;
    assert!(
        fewest <= books && books <= most,
        "{books} in [{fewest}, {most}]"
    );
    assert!(["scene/kitchen", "scene/living_room"].contains(&world.scene.asset.as_str()));
}
