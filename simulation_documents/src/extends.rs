//! Resolving a world document against its bases: the base gives each omitted
//! field, a map entry replaces the base's entry of its key, `{ removed: true }`
//! drops it (and in `scene_objects` removes the object), `{ kept: true }` drops
//! the base's `scene_objects` entry.

use std::collections::{BTreeMap, BTreeSet};

use crate::daylight::Daylight;
use crate::sampler::Sampled;
use crate::world::{
    Entry, Origin, WorldFields, check_name_rules, kept_daylight, kept_outside_scene_objects,
};
use crate::{Error, Manifest, Revision, World, WorldDocument, WorldPattern};

/// The most bases a world's chain of bases holds.
pub(crate) const MAX_BASES: usize = 128;

/// A document's name and revision, as a set of the documents met orders
/// them.
#[derive(PartialEq, Eq, PartialOrd, Ord)]
struct WorldRef {
    name: String,
    revision: Revision,
}

impl WorldRef {
    fn of(manifest: &Manifest) -> WorldRef {
        WorldRef {
            name: manifest.name.clone(),
            revision: manifest.revision,
        }
    }
}

impl WorldDocument {
    /// The concrete world of this document, with each base found among
    /// `bases` by name and revision. Refuses a missing base, a cycle, and a
    /// base that breaks a rule of its own, naming the base: a refusal that
    /// comes of a base, or of a base of a base, names each base it passes
    /// through. The chain of bases is walked and folded in a loop, and holds
    /// at most 128 bases. Refuses a world pattern: one that holds a sampler,
    /// an entry of an item kind, `work_cell_parameters` or `min_spacing_m`.
    pub fn resolve(self, bases: &[WorldDocument]) -> Result<World, Error> {
        let (fields, origin) = self.folded(bases)?;
        WorldDocument { fields }.into_world(origin)
    }

    /// The world pattern of this document with each base found among
    /// `bases`, with the refusals of [`WorldDocument::resolve`] but those of
    /// a world pattern. A concrete world is a world pattern whose every seed
    /// gives that world.
    pub fn resolve_pattern(self, bases: &[WorldDocument]) -> Result<WorldPattern, Error> {
        let (fields, origin) = self.folded(bases)?;
        WorldPattern::of_fields(fields, origin)
    }

    /// The fields of this document with its bases folded in, and where they
    /// come from.
    fn folded(self, bases: &[WorldDocument]) -> Result<(WorldFields, Origin), Error> {
        let origin = match self.fields.extends {
            Some(_) => Origin::WithBases,
            None => Origin::Alone,
        };
        let chain = base_chain(&self, bases)?;
        Ok((folded(self.fields, &chain)?, origin))
    }
}

/// The bases `document` extends, from its own base to the base that extends
/// none, each found among `bases`. Refuses a missing base, a cycle and two
/// documents given as one base, naming the bases on the way to it, and a
/// chain longer than [`MAX_BASES`], as a refusal of the document.
fn base_chain<'a>(
    document: &WorldDocument,
    bases: &'a [WorldDocument],
) -> Result<Vec<&'a WorldDocument>, Error> {
    let mut seen = BTreeSet::from([WorldRef::of(document.manifest())]);
    let mut chain: Vec<&WorldDocument> = Vec::new();
    let mut next = document.fields.extends.clone();
    while let Some(base_ref) = next {
        if chain.len() == MAX_BASES {
            return Err(Error::rule(format!(
                "the chain of bases holds more than {MAX_BASES} bases; extend a base nearer the root"
            )));
        }
        if !seen.insert(WorldRef::of(&base_ref)) {
            return Err(in_bases(&chain, Error::BaseCycle(base_ref.name)));
        }
        let mut named = bases.iter().filter(|base| *base.manifest() == base_ref);
        let Some(base) = named.next() else {
            let missing = Error::MissingBase {
                name: base_ref.name,
                revision: base_ref.revision.get(),
            };
            return Err(in_bases(&chain, missing));
        };
        if named.any(|other| other != base) {
            let conflict = Error::BaseConflict {
                name: base_ref.name,
                revision: base_ref.revision.get(),
            };
            return Err(in_bases(&chain, conflict));
        }
        chain.push(base);
        next = base.fields.extends.clone();
    }
    Ok(chain)
}

/// `refusal` as a refusal of the last base of `chain`, named by each base of
/// `chain`, outermost first.
fn in_bases(chain: &[&WorldDocument], refusal: Error) -> Error {
    chain
        .iter()
        .rev()
        .fold(refusal, |refusal, base| Error::InBase {
            name: base.manifest().name.clone(),
            revision: base.manifest().revision.get(),
            refusal: Box::new(refusal),
        })
}

/// `fields` with the bases of `chain` folded in, from the base that extends
/// none up to the document's own base: the base that extends none holds
/// values alone, and each document folded in keeps the name rules.
fn folded(fields: WorldFields, chain: &[&WorldDocument]) -> Result<WorldFields, Error> {
    let Some((root, above)) = chain.split_last() else {
        return Ok(fields);
    };
    root.fields
        .marked_maps(Origin::Alone)
        .map_err(|refusal| in_bases(chain, refusal))?;
    let base = above.iter().enumerate().rev().try_fold(
        root.fields.clone(),
        |base, (depth, document)| {
            merged(document.fields.clone(), base)
                .map_err(|refusal| in_bases(&chain[..=depth], refusal))
        },
    )?;
    merged(fields, base)
}

/// `fields` over `base`, whose own bases are folded in already, checked
/// against the name rules.
fn merged(fields: WorldFields, base: WorldFields) -> Result<WorldFields, Error> {
    let merged = WorldFields {
        manifest: fields.manifest,
        catalogue: fields.catalogue.or(base.catalogue),
        extends: None,
        scene: fields.scene.or(base.scene),
        robots: merged_entries("robots", MapKind::Other, base.robots, fields.robots)?,
        scene_objects: merged_entries(
            "scene_objects",
            MapKind::SceneObjects,
            base.scene_objects,
            fields.scene_objects,
        )?,
        objects: merged_entries("objects", MapKind::Other, base.objects, fields.objects)?,
        lighting: merged_entries("lighting", MapKind::Other, base.lighting, fields.lighting)?,
        materials: merged_entries(
            "materials",
            MapKind::Other,
            base.materials,
            fields.materials,
        )?,
        daylight: merged_daylight(base.daylight, fields.daylight)?,
        regions: merged_entries("regions", MapKind::Other, base.regions, fields.regions)?,
        locations: merged_entries(
            "locations",
            MapKind::Other,
            base.locations,
            fields.locations,
        )?,
        work_cell_parameters: merged_entries(
            "work_cell_parameters",
            MapKind::Other,
            base.work_cell_parameters,
            fields.work_cell_parameters,
        )?,
        min_spacing_m: fields.min_spacing_m.or(base.min_spacing_m),
    };
    check_name_rules(&merged)?;
    Ok(merged)
}

/// What the marks do in a map. In `scene_objects`, `{ removed: true }` stays
/// as the removed state of the object and `{ kept: true }` drops the base's
/// entry and stays as the kept state, under which the scene object keeps its
/// name for the name rules; in every other map `{ removed: true }` drops the
/// base's entry and `{ kept: true }` is refused.
#[derive(Clone, Copy, PartialEq)]
enum MapKind {
    SceneObjects,
    Other,
}

/// The base's entries of `field` with the world's folded in: a value
/// replaces, the marks act as `kind` says.
fn merged_entries<K: Ord + std::fmt::Display, T>(
    field: &str,
    kind: MapKind,
    base: BTreeMap<K, Entry<T>>,
    over: BTreeMap<K, Entry<T>>,
) -> Result<BTreeMap<K, Entry<T>>, Error> {
    over.into_iter().try_fold(base, |mut merged, (key, entry)| {
        match (entry, kind) {
            (Entry::Value(value), _) => {
                merged.insert(key, Entry::Value(value));
            }
            (Entry::Removed, MapKind::SceneObjects) => {
                merged.insert(key, Entry::Removed);
            }
            (Entry::Removed, MapKind::Other) => {
                drop_base_entry(&mut merged, field, &key, "removed")?
            }
            (Entry::Kept, MapKind::SceneObjects) => {
                drop_base_entry(&mut merged, field, &key, "kept")?;
                merged.insert(key, Entry::Kept);
            }
            (Entry::Kept, MapKind::Other) => return Err(kept_outside_scene_objects(field, &key)),
        }
        Ok(merged)
    })
}

/// The base's daylight with the world's folded in: a value replaces it
/// whole, `{ removed: true }` drops it and `{ kept: true }` is refused.
/// Refuses a `{ removed: true }` over a base that leaves no daylight, after
/// its own marks.
fn merged_daylight(
    base: Option<Entry<Sampled<Daylight>>>,
    over: Option<Entry<Sampled<Daylight>>>,
) -> Result<Option<Entry<Sampled<Daylight>>>, Error> {
    match (over, base) {
        (None, base) => Ok(base),
        (Some(Entry::Value(daylight)), _) => Ok(Some(Entry::Value(daylight))),
        (Some(Entry::Removed), Some(Entry::Value(_))) => Ok(None),
        (Some(Entry::Removed), _) => Err(Error::field(
            "daylight",
            "`removed` drops the daylight of a base, and the bases leave no daylight; leave `daylight` out",
        )),
        (Some(Entry::Kept), _) => Err(kept_daylight()),
    }
}

/// Drops the base's entry of `key` from `merged`, for the mark `mark` at
/// `key` of `field`; refuses a mark whose key the bases leave no entry
/// under, after their own marks: no entry, or the kept state of a scene
/// object.
fn drop_base_entry<K: Ord + std::fmt::Display, T>(
    merged: &mut BTreeMap<K, Entry<T>>,
    field: &str,
    key: &K,
    mark: &str,
) -> Result<(), Error> {
    if matches!(merged.remove(key), None | Some(Entry::Kept)) {
        return Err(Error::field(
            format!("{field}.{key}"),
            format!(
                "`{mark}` drops an entry of a base, and the bases leave no entry `{key}`; write the key of an entry they leave"
            ),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::tests::{RELEASE, example, example_text, name};
    use crate::{Physics, Pose, RobotName, SceneObject};

    fn document(text: &str) -> WorldDocument {
        WorldDocument::parse(text).unwrap()
    }

    /// The text of a document that extends the example's base, with `body`.
    fn extending_text(body: &str) -> String {
        format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "world_extending", revision: 1 }}, extends: {{ name: "world_3f9a1c27", revision: 3 }}, {body} }}"#
        )
    }

    fn extending(body: &str) -> WorldDocument {
        document(&extending_text(body))
    }

    #[test]
    fn a_base_gives_each_omitted_field_and_the_world_replaces_entries_by_key() {
        let world = extending(
            r#"objects: { crate: { asset: "object/box", pose: { position: [0, 0, 0], yaw: 0 }, physics: "static" }, book: { asset: "object/book", pose: { position: [1, 0, 0], yaw: 0 } } }"#,
        )
        .resolve(&[document(&example_text())])
        .unwrap();
        assert_eq!(world.manifest.name, "world_extending");
        assert_eq!(
            (world.catalogue, world.scene),
            (example().catalogue, example().scene)
        );
        assert_eq!(world.robots, example().robots);
        assert_eq!(world.objects[&name("crate")].physics, Some(Physics::Static));
        assert_eq!(
            world.objects[&name("book")].pose,
            Pose::Yawed {
                position: [1.0, 0.0, 0.0],
                yaw: 0.0
            }
        );
    }

    #[test]
    fn removed_drops_a_base_entry_and_kept_restores_a_scene_object() {
        let world = extending(r#"objects: { crate: { removed: true } }, scene_objects: { mantel_clock_01: { kept: true } }, robots: { alpha: { removed: true } }"#)
            .resolve(&[document(&example_text())])
            .unwrap();
        assert!(
            world.objects.is_empty() && world.scene_objects.is_empty() && world.robots.is_empty()
        );
    }

    #[test]
    fn removed_in_scene_objects_removes_the_object_from_the_world() {
        let world = extending(r#"scene_objects: { modern_coffee_table_01: { removed: true } }"#)
            .resolve(&[document(&example_text())])
            .unwrap();
        assert_eq!(
            world.scene_objects[&name("modern_coffee_table_01")],
            SceneObject::Removed
        );
        assert_eq!(
            world.scene_objects[&name("mantel_clock_01")],
            SceneObject::Removed
        );
    }

    #[test]
    fn bases_chain_and_a_missing_base_or_a_cycle_is_refused() {
        let middle = document(
            r#"{ simulation_schema: "world/v1", manifest: { name: "world_3f9a1c27", revision: 3 }, extends: { name: "world_root", revision: 1 }, scene: { asset: "scene/kitchen" } }"#,
        );
        let root = document(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "world_root", revision: 1 }}, catalogue: "{RELEASE}", scene: {{ asset: "scene/default" }} }}"#
        ));
        let world = extending("")
            .resolve(&[middle.clone(), root.clone()])
            .unwrap();
        assert_eq!(world.scene.asset, "scene/kitchen");
        assert_eq!(
            extending("").resolve(std::slice::from_ref(&middle)),
            Err(Error::InBase {
                name: "world_3f9a1c27".into(),
                revision: 3,
                refusal: Box::new(Error::MissingBase {
                    name: "world_root".into(),
                    revision: 1
                })
            })
        );
        assert_eq!(
            extending("")
                .resolve(std::slice::from_ref(&middle))
                .unwrap_err()
                .to_string(),
            "the base `world_3f9a1c27` revision 3: the world extends `world_root` revision 1, which is not among the bases given"
        );
        assert_eq!(
            extending("").resolve(&[]).unwrap_err().to_string(),
            "the world extends `world_3f9a1c27` revision 3, which is not among the bases given",
            "the world's own base is the world's to name"
        );
        let looped = document(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "world_root", revision: 1 }}, extends: {{ name: "world_3f9a1c27", revision: 3 }}, catalogue: "{RELEASE}" }}"#
        ));
        let cycle = extending("").resolve(&[middle, looped]).unwrap_err();
        assert_eq!(
            cycle,
            Error::InBase {
                name: "world_3f9a1c27".into(),
                revision: 3,
                refusal: Box::new(Error::InBase {
                    name: "world_root".into(),
                    revision: 1,
                    refusal: Box::new(Error::BaseCycle("world_3f9a1c27".into()))
                })
            }
        );
        assert_eq!(
            cycle.to_string(),
            "the base `world_3f9a1c27` revision 3: the base `world_root` revision 1: the bases of `world_3f9a1c27` extend one another in a cycle; remove the `extends` of one of them",
            "the refusal names each base on the way round the cycle"
        );
    }

    /// A middle base of a chain whose own document holds `body`, on a root
    /// that stands alpha and spawns `x`.
    fn chain_middle(body: &str) -> (WorldDocument, WorldDocument) {
        let middle = document(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "world_3f9a1c27", revision: 3 }}, extends: {{ name: "world_root", revision: 1 }}, {body} }}"#
        ));
        let root = document(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "world_root", revision: 1 }}, catalogue: "{RELEASE}", scene: {{ asset: "scene/default" }}, robots: {{ alpha: {{ join: {{ option: "openarm_sim" }} }} }}, objects: {{ x: {{ asset: "object/box", pose: {{ position: [0, 0, 0], yaw: 0 }}, physics: "static" }} }} }}"#
        ));
        (middle, root)
    }

    /// The refusal of `extending` with `body`, on the chain whose middle base
    /// holds `middle_body`, as the middle base's refusal.
    fn refusal_in_the_middle(middle_body: &str, body: &str) -> Error {
        let (middle, root) = chain_middle(middle_body);
        let Err(Error::InBase {
            name,
            revision,
            refusal,
        }) = extending(body).resolve(&[middle, root])
        else {
            panic!("the middle base is refused");
        };
        assert_eq!((name.as_str(), revision), ("world_3f9a1c27", 3));
        *refusal
    }

    #[test]
    fn a_middle_base_that_breaks_a_rule_of_its_own_is_refused_naming_it() {
        assert_eq!(
            refusal_in_the_middle(r#"objects: { x: { kept: true } }"#, ""),
            kept_outside_scene_objects("objects", &"x")
        );
        assert_eq!(
            refusal_in_the_middle(
                r#"robots: { alpha_2: { join: { option: "openarm_sim" } } }"#,
                "robots: { alpha: { removed: true } }"
            ),
            Error::RobotNamePrefix {
                shorter: "alpha".into(),
                longer: "alpha_2".into()
            }
        );
        assert_eq!(
            refusal_in_the_middle(
                r#"scene_objects: { x: { removed: true } }"#,
                "objects: { x: { removed: true } }"
            ),
            Error::NameInBothNamespaces("x".into()),
            "the two name rules hold at the same levels"
        );
    }

    #[test]
    fn a_concrete_world_is_read_from_a_document_that_extends_no_base() {
        let unresolved = World::parse(&extending("").document_text()).unwrap_err();
        assert_eq!(unresolved, Error::Unresolved("world_extending".into()));
        assert_eq!(
            unresolved.to_string(),
            "the world `world_extending` extends a base; apply it after it is resolved against its bases"
        );
    }

    #[test]
    fn a_removed_robot_stands_no_robot_for_the_name_rule() {
        let world = extending(
            r#"robots: { alpha: { removed: true }, alpha_2: { join: { option: "so101_sim" } } }"#,
        )
        .resolve(&[document(&example_text())])
        .unwrap();
        assert_eq!(
            world
                .robots
                .keys()
                .map(RobotName::as_str)
                .collect::<Vec<_>>(),
            ["alpha_2"]
        );
    }

    /// The document of revision `revision` of `world_chain`: revision 1 names
    /// a catalogue and a scene, and each later revision extends the one
    /// before.
    fn chain_revision(revision: usize) -> WorldDocument {
        let body = match revision {
            1 => format!(r#"catalogue: "{RELEASE}", scene: {{ asset: "scene/default" }}"#),
            _ => format!(
                r#"extends: {{ name: "world_chain", revision: {} }}"#,
                revision - 1
            ),
        };
        document(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "world_chain", revision: {revision} }}, {body} }}"#
        ))
    }

    /// The document `name` revision 1, extending `base` revision 1 when one
    /// is given, with the fields `body`.
    fn link(name: &str, base: Option<&str>, body: &str) -> WorldDocument {
        let extends = base
            .map(|base| format!(r#"extends: {{ name: "{base}", revision: 1 }}, "#))
            .unwrap_or_default();
        document(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "{name}", revision: 1 }}, {extends}{body} }}"#
        ))
    }

    /// The chain `world_top` extends `world_near`, which extends
    /// `world_middle`, which extends `world_root`, with the fields `near`,
    /// `middle` and `root` in each base.
    fn three_bases(top: &str, near: &str, middle: &str, root: &str) -> Result<World, Error> {
        let root = format!(r#"catalogue: "{RELEASE}", scene: {{ asset: "scene/root" }}, {root}"#);
        link("world_top", Some("world_near"), top).resolve(&[
            link("world_near", Some("world_middle"), near),
            link("world_middle", Some("world_root"), middle),
            link("world_root", None, &root),
        ])
    }

    #[test]
    fn a_drop_of_an_entry_the_bases_do_not_leave_is_refused() {
        let base = [document(&example_text())];
        assert_eq!(
            extending("objects: { crat: { removed: true } }")
                .resolve(&base)
                .unwrap_err()
                .to_string(),
            "objects.crat: `removed` drops an entry of a base, and the bases leave no entry `crat`; write the key of an entry they leave"
        );
        assert_eq!(
            extending("scene_objects: { far_shelf: { kept: true } }")
                .resolve(&base)
                .unwrap_err()
                .to_string(),
            "scene_objects.far_shelf: `kept` drops an entry of a base, and the bases leave no entry `far_shelf`; write the key of an entry they leave"
        );
    }

    #[test]
    fn a_mark_that_repeats_the_mark_of_a_nearer_base_is_refused() {
        let crate_at_the_root =
            r#"objects: { crate: { asset: "object/box", pose: { at: "corner_a" } } }"#;
        let removal = "objects: { crate: { removed: true } }";
        assert!(three_bases("", removal, "", crate_at_the_root).is_ok());
        assert_eq!(
            three_bases(removal, removal, "", crate_at_the_root)
                .unwrap_err()
                .to_string(),
            "objects.crate: `removed` drops an entry of a base, and the bases leave no entry `crate`; write the key of an entry they leave"
        );
    }

    #[test]
    fn a_base_with_a_blank_name_is_refused_as_the_document_reads() {
        let refused = WorldDocument::parse(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "w", revision: 2 }}, extends: {{ name: " ", revision: 1 }}, catalogue: "{RELEASE}" }}"#
        ))
        .unwrap_err();
        assert_eq!(
            refused.to_string(),
            "extends: the name is blank; write a name that holds a character other than whitespace"
        );
    }

    #[test]
    fn a_base_nearer_the_document_wins_over_a_base_farther_from_it() {
        let world = three_bases(
            "",
            r#"scene: { asset: "scene/near" }"#,
            r#"scene: { asset: "scene/middle" }"#,
            "",
        )
        .unwrap();
        assert_eq!(world.scene.asset, "scene/near");
    }

    #[test]
    fn a_refusal_deep_in_the_chain_names_each_base_on_the_way_to_it() {
        let in_middle = three_bases("", "", r#"objects: { x: { kept: true } }"#, "").unwrap_err();
        assert_eq!(
            in_middle.to_string(),
            "the base `world_near` revision 1: the base `world_middle` revision 1: objects.x: `kept` drops a `scene_objects` entry of a base; write `removed: true` to drop this entry, or the entry's fields"
        );
        let in_root = three_bases("", "", "", r#"objects: { y: { removed: true } }"#).unwrap_err();
        assert_eq!(
            in_root.to_string(),
            "the base `world_near` revision 1: the base `world_middle` revision 1: the base `world_root` revision 1: objects.y: `removed` drops an entry of a base; this world extends no base"
        );
    }

    /// A chain of the most bases resolves on a thread with the 2 MiB stack of
    /// a tokio worker, and a longer one is refused.
    #[test]
    fn a_chain_of_the_most_bases_resolves_on_a_small_stack_and_a_longer_one_is_refused() {
        let resolved = std::thread::Builder::new()
            .stack_size(2 << 20)
            .spawn(|| {
                let bases: Vec<_> = (1..=MAX_BASES).map(chain_revision).collect();
                let longer = chain_revision(MAX_BASES + 2)
                    .resolve(&[bases.clone(), vec![chain_revision(MAX_BASES + 1)]].concat());
                (chain_revision(MAX_BASES + 1).resolve(&bases), longer)
            })
            .unwrap()
            .join()
            .unwrap();
        assert_eq!(resolved.0.unwrap().scene.asset, "scene/default");
        assert_eq!(
            resolved.1.unwrap_err().to_string(),
            "the chain of bases holds more than 128 bases; extend a base nearer the root"
        );
    }

    #[test]
    fn a_removed_object_stands_no_object_for_the_namespace_rule() {
        let world = extending(
            r#"objects: { crate: { removed: true } }, scene_objects: { crate: { removed: true } }"#,
        )
        .resolve(&[document(&example_text())])
        .unwrap();
        assert!(!world.objects.contains_key(&name("crate")));
        assert_eq!(world.scene_objects[&name("crate")], SceneObject::Removed);
    }

    #[test]
    fn the_catalogue_of_the_world_wins_over_the_catalogue_of_its_base() {
        let world = extending(r#"catalogue: "local""#)
            .resolve(&[document(&example_text())])
            .unwrap();
        assert_eq!(world.catalogue, crate::Catalogue::Local);
        assert_ne!(example().catalogue, crate::Catalogue::Local);
    }

    #[test]
    fn two_different_bases_of_one_name_and_revision_are_refused() {
        let base = example_text();
        let same = extending("").resolve(&[document(&base), document(&base)]);
        assert_eq!(same.unwrap().scene, example().scene);
        let other = document(&base.replace("scene/living_room", "scene/kitchen"));
        assert_eq!(
            extending("")
                .resolve(&[document(&base), other])
                .unwrap_err()
                .to_string(),
            "the bases given hold two different documents as `world_3f9a1c27` revision 3; a revision never changes, so give one"
        );
    }

    #[test]
    fn kept_is_refused_in_every_map_but_scene_objects() {
        for (field, key) in [
            ("robots", "alpha"),
            ("objects", "crate"),
            ("lighting", "key"),
            ("materials", "alpha_arm_metal"),
            ("locations", "drop"),
        ] {
            let refused = extending(&format!("{field}: {{ {key}: {{ kept: true }} }}"))
                .resolve(&[document(&example_text())]);
            assert_eq!(
                refused.unwrap_err().to_string(),
                format!(
                    "{field}.{key}: `kept` drops a `scene_objects` entry of a base; write `removed: true` to drop this entry, or the entry's fields"
                )
            );
        }
    }

    fn with_daylight(body: &str) -> String {
        example_text().replace(
            "\n  objects: {",
            &format!("\n  daylight: {body}, objects: {{"),
        )
    }

    #[test]
    fn the_daylight_of_the_world_replaces_the_daylight_of_its_base_whole() {
        let base = document(&with_daylight(r#"{ date: "2026-11-24", rate: 0 }"#));
        let replaced = extending(r#"daylight: { time: "06:30" }"#)
            .resolve(std::slice::from_ref(&base))
            .unwrap()
            .daylight
            .unwrap();
        assert_eq!(
            (
                replaced.date,
                replaced.time.map(|time| time.to_string()),
                replaced.rate
            ),
            (None, Some("06:30:00".to_owned()), None)
        );
        let kept = extending("")
            .resolve(std::slice::from_ref(&base))
            .unwrap()
            .daylight
            .unwrap();
        assert_eq!(kept.rate, Some(0.0));
        let dropped = extending("daylight: { removed: true }")
            .resolve(&[base])
            .unwrap();
        assert_eq!(dropped.daylight, None);
    }

    #[test]
    fn a_drop_of_a_daylight_the_bases_do_not_leave_is_refused() {
        assert_eq!(
            extending("daylight: { removed: true }")
                .resolve(&[document(&example_text())])
                .unwrap_err()
                .to_string(),
            "daylight: `removed` drops the daylight of a base, and the bases leave no daylight; leave `daylight` out"
        );
        let middle = WorldDocument::parse(
            r#"{ simulation_schema: "world/v1", manifest: { name: "middle", revision: 1 }, extends: { name: "world_3f9a1c27", revision: 3 }, daylight: { removed: true } }"#,
        )
        .unwrap();
        let top = WorldDocument::parse(
            r#"{ simulation_schema: "world/v1", manifest: { name: "top", revision: 1 }, extends: { name: "middle", revision: 1 }, daylight: { removed: true } }"#,
        )
        .unwrap();
        let base = document(&with_daylight("{ rate: 0 }"));
        assert_eq!(
            top.resolve(&[middle, base]).unwrap_err().to_string(),
            "daylight: `removed` drops the daylight of a base, and the bases leave no daylight; leave `daylight` out"
        );
        assert_eq!(
            extending("daylight: { kept: true }")
                .resolve(&[document(&with_daylight("{ rate: 0 }"))])
                .unwrap_err()
                .to_string(),
            "daylight: `kept` drops a `scene_objects` entry of a base; write `removed: true` to drop the daylight of a base, or the daylight's settings"
        );
    }

    #[test]
    fn a_resolved_world_still_names_its_catalogue_and_scene() {
        let root = document(
            r#"{ simulation_schema: "world/v1", manifest: { name: "world_3f9a1c27", revision: 3 }, catalogue: "local" }"#,
        );
        assert_eq!(
            extending("").resolve(&[root]).unwrap_err().to_string(),
            "scene: neither the world nor its bases name a scene; add `scene: { asset: \"scene/<id>\" }`"
        );
        let root = document(
            r#"{ simulation_schema: "world/v1", manifest: { name: "world_3f9a1c27", revision: 3 }, scene: { asset: "scene/default" } }"#,
        );
        assert_eq!(
            extending("").resolve(&[root]).unwrap_err().to_string(),
            "catalogue: neither the world nor its bases name a catalogue release; add `catalogue: \"<digest of the release>\"`, or `catalogue: \"local\"` for a local catalogue"
        );
        let scene_from_the_world = extending(r#"scene: { asset: "scene/kitchen" }"#)
            .resolve(&[document(
                r#"{ simulation_schema: "world/v1", manifest: { name: "world_3f9a1c27", revision: 3 }, catalogue: "local" }"#,
            )])
            .unwrap();
        assert_eq!(scene_from_the_world.scene.asset, "scene/kitchen");
    }

    /// A root base extends no base, so a mark in it is refused before the
    /// merge, whatever the world that extends it writes over it.
    #[test]
    fn a_root_base_is_refused_for_a_mark_of_its_own_by_name() {
        let root = |maps: &str| {
            document(&format!(
                r#"{{ simulation_schema: "world/v1", manifest: {{ name: "world_3f9a1c27", revision: 3 }}, catalogue: "local", scene: {{ asset: "scene/default" }}, {maps} }}"#
            ))
        };
        let kept = extending(r#"robots: { alpha: { join: { option: "so101_sim" } } }"#)
            .resolve(&[root("robots: { alpha: { kept: true } }")]);
        assert_eq!(
            kept.unwrap_err().to_string(),
            "the base `world_3f9a1c27` revision 3: robots.alpha: `kept` drops a `scene_objects` entry of a base; write `removed: true` to drop this entry, or the entry's fields"
        );
        let kept_scene_object =
            extending("").resolve(&[root("scene_objects: { mantel_clock_01: { kept: true } }")]);
        assert_eq!(
            kept_scene_object.unwrap_err().to_string(),
            "the base `world_3f9a1c27` revision 3: scene_objects.mantel_clock_01: `kept` drops an entry of a base; this world extends no base"
        );
        let removed = extending(
            r#"objects: { box: { asset: "object/box", pose: { position: [0, 0, 0], yaw: 0 } } }"#,
        )
        .resolve(&[root("objects: { box: { removed: true } }")]);
        assert_eq!(
            removed.unwrap_err().to_string(),
            "the base `world_3f9a1c27` revision 3: objects.box: `removed` drops an entry of a base; this world extends no base"
        );
    }

    #[test]
    fn the_rules_on_names_hold_after_the_merge() {
        let world = extending(r#"robots: { alpha_2: { join: { option: "so101_sim" } } }"#)
            .resolve(&[document(&example_text())]);
        assert_eq!(
            world,
            Err(Error::RobotNamePrefix {
                shorter: "alpha".into(),
                longer: "alpha_2".into()
            })
        );
        let world = extending(r#"objects: { mantel_clock_01: { asset: "object/clock", pose: { position: [0, 0, 0], yaw: 0 } } }"#).resolve(&[document(&example_text())]);
        assert_eq!(
            world,
            Err(Error::NameInBothNamespaces("mantel_clock_01".into()))
        );
        let kept = WorldDocument::parse(&extending_text(
            r#"scene_objects: { mantel_clock_01: { kept: true } }, objects: { mantel_clock_01: { asset: "object/clock", pose: { position: [0, 0, 0], yaw: 0 } } }"#,
        ));
        assert_eq!(
            kept.map(|_| ()),
            Err(Error::NameInBothNamespaces("mantel_clock_01".into())),
            "a kept scene object keeps its name in the scene"
        );
    }

    /// A scene object a base keeps holds its name through the chain, as it
    /// does in one document, and stands under no entry of the world; a
    /// nearer document writes over the kept state, and a second `kept` finds
    /// no entry to drop.
    #[test]
    fn a_scene_object_a_base_keeps_holds_its_name_in_the_resolved_world() {
        let removed_at_the_root = "scene_objects: { x: { removed: true } }";
        let kept = "scene_objects: { x: { kept: true } }";
        let world = three_bases("", "", kept, removed_at_the_root).unwrap();
        assert!(world.scene_objects.is_empty());
        let spawned = r#"objects: { x: { asset: "object/box", pose: { at: "corner_a" } } }"#;
        assert_eq!(
            three_bases(spawned, "", kept, removed_at_the_root),
            Err(Error::NameInBothNamespaces("x".into()))
        );
        let removed_again =
            three_bases(removed_at_the_root, "", kept, removed_at_the_root).unwrap();
        assert_eq!(
            removed_again.scene_objects[&name("x")],
            SceneObject::Removed
        );
        assert_eq!(
            three_bases(kept, "", kept, removed_at_the_root)
                .unwrap_err()
                .to_string(),
            "scene_objects.x: `kept` drops an entry of a base, and the bases leave no entry `x`; write the key of an entry they leave"
        );
    }

    impl WorldDocument {
        fn document_text(&self) -> String {
            documents_core::Document::of(crate::world::WORLD, &self.fields)
                .unwrap()
                .to_json5()
        }
    }
}
