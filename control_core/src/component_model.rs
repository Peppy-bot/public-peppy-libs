//! The model of the robot a component (an arm, a gripper) is a part of, as the robot's
//! initializer names it with component_ready's `set_model`: the first call configures
//! the component for the model, and every later call names the same model.

use std::fmt::Display;
use std::sync::Mutex;

use crate::fault::{Fault, Reason};
use crate::sync::lock_ignoring_poison;

/// Why a component refuses the model a call names.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum ModelRefusal<E: Display> {
    /// The component does not drive the model: `E` names the model and the models it
    /// drives.
    #[error("{0}")]
    Undriven(E),
    /// The component runs as a part of a robot of `configured`, which the first call
    /// named, and a later call names `named`.
    #[error(
        "this component runs as a part of a robot of model {configured}, which the robot's \
         initializer named first, and the initializer now names {named}"
    )]
    Other { configured: String, named: String },
}

/// The model a component runs as: none until a call names a model it drives.
#[derive(Debug, Default)]
pub struct ComponentModel(Mutex<Option<String>>);

impl ComponentModel {
    /// Takes the model one call names: the first call that names one has `configure`
    /// build the component's configuration for it, which this gives back once; a later
    /// call that names the same model gives `None`. A model `configure` refuses, or a
    /// model other than the one the first call named, is refused.
    pub fn name<C, E: Display>(
        &self,
        model: &str,
        configure: impl FnOnce(&str) -> Result<C, E>,
    ) -> Result<Option<C>, ModelRefusal<E>> {
        let mut configured = lock_ignoring_poison(&self.0);
        match configured.as_deref() {
            Some(same) if same == model => Ok(None),
            Some(other) => Err(ModelRefusal::Other {
                configured: other.to_owned(),
                named: model.to_owned(),
            }),
            None => {
                let configuration = configure(model).map_err(ModelRefusal::Undriven)?;
                *configured = Some(model.to_owned());
                Ok(Some(configuration))
            }
        }
    }

    /// Answers one `set_model` call naming `model`, as [`name`](Self::name) takes it: the
    /// configuration of the first call goes to `configured`. A refused model raises `fault`,
    /// which `stop` ends the node for, and gives the text of the call's error reply.
    pub fn set<C, E: Display>(
        &self,
        model: &str,
        configure: impl FnOnce(&str) -> Result<C, E>,
        configured: impl FnOnce(C),
        fault: &Fault,
        stop: impl FnOnce(),
    ) -> Result<(), String>
    where
        ModelRefusal<E>: Into<Reason>,
    {
        match self.name(model, configure) {
            Ok(configuration) => {
                configuration.map(configured);
                Ok(())
            }
            Err(refusal) => {
                let reply = refusal.to_string();
                fault.raise(refusal, stop);
                Err(reply)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A component that drives `openarm_v2` alone, its configuration the model's
    /// name.
    fn configure(model: &str) -> Result<String, String> {
        match model {
            "openarm_v2" => Ok(format!("configured for {model}")),
            other => Err(format!("model {other} is no model this node drives")),
        }
    }

    #[test]
    fn the_first_call_configures_and_later_calls_of_the_model_do_not() {
        let component = ComponentModel::default();
        assert_eq!(
            component.name("openarm_v2", configure),
            Ok(Some("configured for openarm_v2".to_owned()))
        );
        assert_eq!(component.name("openarm_v2", configure), Ok(None));
    }

    #[test]
    fn a_model_the_component_does_not_drive_is_refused_and_configures_nothing() {
        let component = ComponentModel::default();
        assert_eq!(
            component.name("so101", configure),
            Err(ModelRefusal::Undriven(
                "model so101 is no model this node drives".to_owned()
            ))
        );
        assert_eq!(
            component.name("openarm_v2", configure),
            Ok(Some("configured for openarm_v2".to_owned()))
        );
    }

    #[test]
    fn a_model_other_than_the_first_is_refused_naming_both() {
        let component = ComponentModel::default();
        component
            .name("openarm_v2", configure)
            .expect("the first model");
        let refused = component
            .name("openarm_v1", configure)
            .expect_err("another model");
        assert_eq!(
            refused.to_string(),
            "this component runs as a part of a robot of model openarm_v2, which the robot's \
             initializer named first, and the initializer now names openarm_v1"
        );
    }

    #[test]
    fn set_hands_the_first_configuration_on_once_and_a_refusal_raises_the_fault() {
        use std::cell::{Cell, RefCell};
        let component = ComponentModel::default();
        let fault = Fault::default();
        let handed = RefCell::new(Vec::new());
        let stops = Cell::new(0);
        let set = |model: &str| {
            component.set(
                model,
                configure,
                |configuration| handed.borrow_mut().push(configuration),
                &fault,
                || stops.set(stops.get() + 1),
            )
        };
        assert_eq!(set("openarm_v2"), Ok(()));
        assert_eq!(set("openarm_v2"), Ok(()));
        assert_eq!(*handed.borrow(), ["configured for openarm_v2"]);
        assert_eq!(stops.get(), 0, "a driven model raises nothing");
        let reply = set("openarm_v1").expect_err("another model");
        assert!(reply.contains("now names openarm_v1"), "{reply}");
        assert_eq!(stops.get(), 1);
        assert_eq!(
            fault.take().map(|reason| reason.to_string()),
            Some(reply),
            "the fault's reason is the reply's text"
        );
    }
}
