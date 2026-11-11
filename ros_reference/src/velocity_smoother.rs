//! Velocity smoother. The reference is Nav2's velocity smoother (nav2_velocity_smoother
//! 1.5.2): configured with each scenario's parameters, it takes the scenario's setpoints
//! through its command callback and runs its timer's cycle at each tick, each event at its
//! own time in integer nanoseconds on the node's clock, and gives the velocity each cycle
//! publishes, or that it publishes none.

use crate::{Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a velocity smoother scenario and of its reference output, which refuses
/// every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "velocity_smoother")]
    VelocitySmoother,
}

/// The smoother's parameters, by Nav2's names: the limits of linear x, linear y and
/// angular z in that order, and the node's settings the port pins.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SmootherParameters {
    pub smoothing_frequency: f64,
    pub feedback: String,
    pub scale_velocities: bool,
    pub max_velocity: [f64; 3],
    pub min_velocity: [f64; 3],
    pub max_accel: [f64; 3],
    pub max_decel: [f64; 3],
    pub deadband_velocity: [f64; 3],
    pub velocity_timeout: f64,
    pub odom_topic: String,
    pub odom_duration: f64,
    pub enable_stamped_cmd_vel: bool,
    pub use_realtime_priority: bool,
    pub allow_parameter_qos_overrides: bool,
}

/// One event of a scenario: a setpoint that arrives, or a cycle of the smoother's timer,
/// at a time in nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(tag = "event", rename_all = "snake_case", deny_unknown_fields)]
pub enum Event {
    Setpoint { at_ns: i64, twist: [f64; 3] },
    Tick { at_ns: i64 },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    smoother: SmootherParameters,
    events: Vec<Event>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    smoother: SmootherRead,
    events: Vec<EventRead>,
}

/// The parameters the smoother read: those of the scenario that its timer's cycle reads.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SmootherRead {
    smoothing_frequency: f64,
    feedback: String,
    scale_velocities: bool,
    max_velocity: [f64; 3],
    min_velocity: [f64; 3],
    max_accel: [f64; 3],
    max_decel: [f64; 3],
    deadband_velocity: [f64; 3],
    velocity_timeout: f64,
}

/// An event as the reference read it, and for a tick the velocity its cycle published:
/// three numbers, or none when the cycle published nothing.
#[derive(Deserialize)]
#[serde(tag = "event", rename_all = "snake_case", deny_unknown_fields)]
enum EventRead {
    Setpoint {
        at_ns: i64,
        twist: [f64; 3],
    },
    Tick {
        at_ns: i64,
        published: bool,
        twist: Vec<f64>,
    },
}

/// One cycle of the smoother: when it ran, and the velocity it published, linear x,
/// linear y and angular z, if it published one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cycle {
    pub at_ns: i64,
    pub published: Option<[f64; 3]>,
}

/// A velocity smoother scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub smoother: SmootherParameters,
    pub events: Vec<Event>,
    /// The cycle of each tick, in the order of the ticks.
    pub cycles: Vec<Cycle>,
}

impl Cases {
    /// Every velocity smoother scenario under `scenarios/`, in the order of their names,
    /// each paired with its reference output: the reference read the parameters and every
    /// event.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::VelocitySmoother)
            .into_iter()
            .map(|name| {
                let scenario = read_scenario(&name);
                let reference = read_reference(&name);
                pair(name, scenario, reference)
            })
            .collect()
    }
}

fn pair(name: String, scenario: Scenario, reference: Reference) -> Cases {
    let Scenario {
        kind: ThisKind::VelocitySmoother,
        smoother,
        events,
    } = scenario;
    let Reference {
        kind: ThisKind::VelocitySmoother,
        smoother: read,
        events: events_read,
    } = reference;
    let smoother_read = SmootherParameters {
        smoothing_frequency: read.smoothing_frequency,
        feedback: read.feedback,
        scale_velocities: read.scale_velocities,
        max_velocity: read.max_velocity,
        min_velocity: read.min_velocity,
        max_accel: read.max_accel,
        max_decel: read.max_decel,
        deadband_velocity: read.deadband_velocity,
        velocity_timeout: read.velocity_timeout,
        ..smoother.clone()
    };
    assert_eq!(
        smoother_read, smoother,
        "{name}: the reference read the smoother's parameters"
    );
    assert_eq!(
        events.len(),
        events_read.len(),
        "{name}: the reference ran every event"
    );
    let cycles = events
        .iter()
        .zip(events_read)
        .enumerate()
        .filter_map(|(index, (event, read))| match (*event, read) {
            (
                Event::Setpoint { at_ns, twist },
                EventRead::Setpoint {
                    at_ns: read_ns,
                    twist: read_twist,
                },
            ) => {
                assert_eq!(
                    (read_ns, read_twist),
                    (at_ns, twist),
                    "{name} event {index}: the reference read the setpoint"
                );
                None
            }
            (
                Event::Tick { at_ns },
                EventRead::Tick {
                    at_ns: read_ns,
                    published,
                    twist,
                },
            ) => {
                assert_eq!(
                    read_ns, at_ns,
                    "{name} event {index}: the reference read the tick"
                );
                let published = match (published, twist.as_slice()) {
                    (true, &[x, y, z]) => Some([x, y, z]),
                    (false, []) => None,
                    _ => panic!(
                        "{name} event {index}: a published cycle holds three axes and an \
                         unpublished one none, and this one holds {twist:?}"
                    ),
                };
                Some(Cycle { at_ns, published })
            }
            _ => panic!("{name} event {index}: the reference ran another kind of event"),
        })
        .collect();
    Cases {
        name,
        smoother,
        events,
        cycles,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "inflation", "smoother": {}, "events": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    #[test]
    fn an_event_reads_by_its_tag() {
        let setpoint: Event =
            serde_json::from_str(r#"{"event": "setpoint", "at_ns": 5, "twist": [1.0, 0.0, -0.5]}"#)
                .expect("a setpoint");
        assert_eq!(
            setpoint,
            Event::Setpoint {
                at_ns: 5,
                twist: [1.0, 0.0, -0.5]
            }
        );
        assert!(
            serde_json::from_str::<Event>(r#"{"event": "tick", "at_ns": 5, "twist": []}"#).is_err()
        );
    }

    #[test]
    fn every_velocity_smoother_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a velocity smoother scenario"
        );
        for cases in scenarios {
            assert!(
                cases.cycles.iter().any(|cycle| cycle.published.is_some()),
                "{} publishes a velocity",
                cases.name
            );
        }
    }
}
