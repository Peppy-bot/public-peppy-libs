//! The world document of a live simulation: the setup record with the robots
//! of the stack, and the start world of a launch.

use std::collections::BTreeMap;

use documents_core::Manifest;

use crate::geometry::Placement;
use crate::names::first_named_twice;
use crate::world::{
    Catalogue, Join, Robot, Stand, check_namespaces, check_region_names,
    robot_names_free_of_prefixes,
};
use crate::{Error, RobotName, SetupRecord, World, text};

/// A simulated robot of the stack as its listing gives it: its name, its join,
/// and whether it has a body, which stands in the world.
#[derive(Clone, Debug, PartialEq)]
pub struct StackRobot {
    pub name: RobotName,
    pub join: Join,
    pub body: bool,
}

impl World {
    /// The world of the live simulation: `record` with each robot of `robots`
    /// joined as the listing says and standing as the record says.
    pub fn from_setup(
        manifest: Manifest,
        catalogue: Catalogue,
        record: &SetupRecord,
        robots: &[StackRobot],
    ) -> Result<World, Error> {
        World::composed(manifest, catalogue, record, robots, |robot| {
            record.stands.get(&robot.name).cloned()
        })
    }

    fn composed(
        manifest: Manifest,
        catalogue: Catalogue,
        record: &SetupRecord,
        robots: &[StackRobot],
        stand_of: impl Fn(&StackRobot) -> Option<Stand>,
    ) -> Result<World, Error> {
        if let Some(twice) = first_named_twice(robots, |a, b| a.name == b.name) {
            return Err(Error::rule(format!(
                "the stack lists the robot `{}` twice; a stack holds each copy name once",
                twice.name
            )));
        }
        robot_names_free_of_prefixes(robots.iter().map(|robot| &robot.name))?;
        let world = World {
            manifest,
            catalogue,
            scene: record.scene.clone(),
            robots: robots
                .iter()
                .map(|robot| {
                    (
                        robot.name.clone(),
                        Robot {
                            join: robot.join.clone(),
                            stand: robot.body.then(|| stand_of(robot)).flatten(),
                        },
                    )
                })
                .collect(),
            scene_objects: record.scene_objects.clone(),
            objects: record.objects.clone(),
            lighting: record.lighting.clone(),
            materials: record.materials.clone(),
            daylight: record.daylight.clone(),
            regions: record.regions.clone(),
            locations: record.locations.clone(),
        };
        check_namespaces(&world)?;
        check_region_names(&world)?;
        Ok(world)
    }
}

/// The setup the simulation started with, and where each robot with a body
/// first joined.
#[derive(Clone, Debug, PartialEq)]
pub struct StartSetup {
    pub record: SetupRecord,
    pub first_stands: BTreeMap<RobotName, Placement>,
}

impl StartSetup {
    /// The start setup as `get` of `simulation_world` answers it with
    /// `start`: the setup record and the first stands, each as JSON5 text.
    pub fn parse(setup_record: &str, first_stands: &str) -> Result<StartSetup, Error> {
        Ok(StartSetup {
            record: SetupRecord::parse(setup_record)?,
            first_stands: text::from_json5(first_stands)?,
        })
    }

    /// The first stands as JSON5 text: each robot name to `{ position, yaw }`,
    /// with the refusals of [`World::to_json5`].
    pub fn first_stands_to_json5(&self) -> Result<String, Error> {
        text::to_json5(&self.first_stands)
    }

    /// The start world: the start setup with the copies the launch started,
    /// each with a body standing where it first joined.
    pub fn world(
        &self,
        manifest: Manifest,
        catalogue: Catalogue,
        launched: &[StackRobot],
    ) -> Result<World, Error> {
        World::composed(manifest, catalogue, &self.record, launched, |robot| {
            self.first_stands
                .get(&robot.name)
                .map(|placement| Stand::Placed(*placement))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::tests::{example, name, robot};
    use crate::{Feature, Revision, Scene, SceneObject};

    fn alpha() -> StackRobot {
        StackRobot {
            name: robot("alpha"),
            join: Join {
                option: "openarm_sim".into(),
                features: vec![Feature::Named("ai_brain".into())],
                core_node: None,
            },
            body: true,
        }
    }

    #[test]
    fn the_record_with_the_listed_robots_is_the_world() {
        let example = example();
        let record = SetupRecord::built(&example);
        let world = World::from_setup(
            example.manifest.clone(),
            example.catalogue.clone(),
            &record,
            &[alpha()],
        )
        .unwrap();
        assert_eq!(world, example);
    }

    #[test]
    fn a_robot_the_stack_lists_twice_is_refused() {
        let example = example();
        let twice = StackRobot {
            join: Join {
                option: "so101_sim".into(),
                features: vec![],
                core_node: None,
            },
            ..alpha()
        };
        assert_eq!(
            World::from_setup(
                example.manifest.clone(),
                example.catalogue.clone(),
                &SetupRecord::built(&example),
                &[alpha(), twice],
            )
            .unwrap_err()
            .to_string(),
            "the stack lists the robot `alpha` twice; a stack holds each copy name once"
        );
    }

    #[test]
    fn a_robot_without_a_body_stands_nowhere_and_a_robot_without_a_stand_has_none() {
        let example = example();
        let mut record = SetupRecord::built(&example);
        record.stands.insert(
            robot("feeder"),
            Stand::Placed(Placement {
                position: [1.0, 0.0, 0.0],
                yaw: 0.0,
            }),
        );
        let twin = StackRobot {
            name: robot("feeder"),
            join: Join {
                option: "feeder_twin".into(),
                features: vec![],
                core_node: None,
            },
            body: false,
        };
        let newcomer = StackRobot {
            name: robot("bravo"),
            ..alpha()
        };
        let world = World::from_setup(
            example.manifest.clone(),
            example.catalogue.clone(),
            &record,
            &[alpha(), twin, newcomer],
        )
        .unwrap();
        assert_eq!(
            world.robots[&robot("feeder")].stand,
            None,
            "the record stands the twin, which has no body"
        );
        assert_eq!(world.robots[&robot("bravo")].stand, None);
        assert_eq!(
            world.robots[&robot("alpha")].stand,
            example.robots[&robot("alpha")].stand
        );
    }

    #[test]
    fn the_robot_name_rule_holds_for_the_listed_robots() {
        let example = example();
        let record = SetupRecord::built(&example);
        let clash = StackRobot {
            name: robot("alpha_2"),
            ..alpha()
        };
        assert_eq!(
            World::from_setup(
                example.manifest.clone(),
                example.catalogue,
                &record,
                &[alpha(), clash]
            ),
            Err(Error::RobotNamePrefix {
                shorter: "alpha".into(),
                longer: "alpha_2".into()
            })
        );
    }

    #[test]
    fn a_record_whose_object_and_scene_object_share_a_name_is_refused() {
        let example = example();
        let mut record = SetupRecord::built(&example);
        record
            .scene_objects
            .insert(name("crate"), SceneObject::Removed);
        assert!(record.objects.contains_key(&name("crate")));
        let refused = World::from_setup(
            example.manifest.clone(),
            example.catalogue.clone(),
            &record,
            &[alpha()],
        )
        .unwrap_err();
        assert_eq!(refused, Error::NameInBothNamespaces("crate".into()));
        assert_eq!(
            refused.to_string(),
            "`crate` is an object name and a scene object name; the two share one namespace, so give the object another name"
        );
    }

    #[test]
    fn a_start_setup_survives_its_two_texts() {
        let start = StartSetup {
            record: SetupRecord::at_start(Scene {
                asset: "scene/default".into(),
                scale: 1.0,
            }),
            first_stands: BTreeMap::from([(
                robot("alpha"),
                Placement {
                    position: [0.0, -0.8, 0.0],
                    yaw: 1.57,
                },
            )]),
        };
        let record = start.record.to_json5().unwrap();
        let parsed = StartSetup::parse(&record, &start.first_stands_to_json5().unwrap()).unwrap();
        assert_eq!(parsed, start);
        assert!(StartSetup::parse(&record, "[]").is_err());
        let mut fallen = start.clone();
        fallen.first_stands.get_mut(&robot("alpha")).unwrap().yaw = f64::NAN;
        assert_eq!(
            fallen.first_stands_to_json5().unwrap_err().to_string(),
            "alpha.yaw: the number NaN is not finite; a document holds finite numbers only"
        );
    }

    #[test]
    fn the_start_world_stands_each_launched_robot_where_it_first_joined() {
        let first = Placement {
            position: [0.0, -0.8, 0.0],
            yaw: 1.57,
        };
        let start = StartSetup {
            record: SetupRecord::at_start(Scene {
                asset: "scene/default".into(),
                scale: 1.0,
            }),
            first_stands: BTreeMap::from([(robot("alpha"), first)]),
        };
        let manifest = Manifest {
            name: "world_start".into(),
            revision: Revision::FIRST,
        };
        let world = start.world(manifest, Catalogue::Local, &[alpha()]).unwrap();
        assert_eq!(
            world.robots[&robot("alpha")].stand,
            Some(Stand::Placed(first))
        );
        assert_eq!(world.scene.asset, "scene/default");
        assert!(world.objects.is_empty() && world.scene_objects.is_empty());
    }
}
