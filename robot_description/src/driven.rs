//! The models a node drives: a bundled model whose arms are the ones a node is compiled
//! for, with its description ([`Driven`], which [`NodeArms::model`](crate::NodeArms::model)
//! gives), the models a node lists by model id with what it holds for each
//! ([`DrivenModels`]), and why a node does not drive a model ([`UndrivenModel`]).

use crate::{Model, NodeArms, PerArm, RobotDescription, UnknownModel};

/// Why a node does not drive the model it is given.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum UndrivenModel {
    #[error(transparent)]
    Unknown(#[from] UnknownModel),
    #[error("model {model} is no model this node drives; it drives {}", ids(.driven))]
    Unlisted {
        model: String,
        driven: Vec<&'static str>,
    },
    #[error(
        "model {model} describes {found}, and this node drives {built_for}; the models of \
         that shape are {}", ids(.fitting)
    )]
    OtherArms {
        model: String,
        found: String,
        built_for: String,
        fitting: Vec<&'static str>,
    },
}

/// Model ids, quoted, as a refusal lists them: "none" for no id.
fn ids(ids: &[&str]) -> String {
    match ids {
        [] => "none".to_owned(),
        _ => {
            let quoted: Vec<String> = ids.iter().map(|id| format!("{id:?}")).collect();
            quoted.join(" and ")
        }
    }
}

/// A bundled model whose description's arms are the `K` arms of `N` joints a node is
/// compiled for, in place order ([`NodeArms::model`](crate::NodeArms::model)), with the
/// description, read once.
#[derive(Debug)]
pub struct Driven<const N: usize, const K: usize> {
    model: &'static Model,
    description: RobotDescription,
    arms: NodeArms<N, K>,
    joint_limits: PerArm<[[f64; 2]; N], K>,
}

impl<const N: usize, const K: usize> Driven<N, K> {
    /// `model`, whose `description` has the arms `arms` and each arm's `joint_limits`.
    pub(crate) fn new(
        model: &'static Model,
        description: RobotDescription,
        arms: NodeArms<N, K>,
        joint_limits: PerArm<[[f64; 2]; N], K>,
    ) -> Self {
        Self {
            model,
            description,
            arms,
            joint_limits,
        }
    }

    /// The bundled model.
    pub fn model(&self) -> &'static Model {
        self.model
    }

    /// The model's description.
    pub fn description(&self) -> &RobotDescription {
        &self.description
    }

    /// Each arm's joint limits, `[lower, upper]` per joint, in place order.
    pub fn joint_limits(&self) -> PerArm<[[f64; 2]; N], K> {
        self.joint_limits
    }

    /// Each arm's position in the state `name` of its group, a value per joint, in place
    /// order, or the name of the first arm the description gives no state `name`.
    pub fn state(&self, name: &str) -> Result<PerArm<[f64; N], K>, &'static str> {
        PerArm::from_fn(|arm| {
            let arm_name = self.arms.arm_name(arm);
            self.description
                .state(arm_name, name)
                .and_then(|values| values.try_into().ok())
                .ok_or(arm_name)
        })
        .transpose()
    }
}

/// The models a node drives, each by its model id with what the node holds for it, such
/// as the friction constants of its arms, in the order the node lists them.
#[derive(Debug)]
pub struct DrivenModels<T, const M: usize>([(&'static str, T); M]);

impl<T: Copy, const M: usize> DrivenModels<T, M> {
    /// The models of `models`, each its model id and what the node holds for it.
    pub const fn new(models: [(&'static str, T); M]) -> Self {
        Self(models)
    }

    /// The bundled model `id`, as robot_identity names it, and what the node holds for it:
    /// refused when the node lists no model `id`, naming the ones it lists, or when no
    /// bundled model is named `id`.
    pub fn model(&self, id: &str) -> Result<(&'static Model, T), UndrivenModel> {
        let Some(&(_, held)) = self.0.iter().find(|(listed, _)| *listed == id) else {
            return Err(UndrivenModel::Unlisted {
                model: id.to_owned(),
                driven: self.0.iter().map(|&(listed, _)| listed).collect(),
            });
        };
        Ok((crate::model(id)?, held))
    }
}

impl<T, const M: usize> IntoIterator for DrivenModels<T, M> {
    type Item = (&'static str, T);
    type IntoIter = std::array::IntoIter<(&'static str, T), M>;

    /// Each model id with what the node holds for it, in the order the node lists them.
    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A node's table of a bundled model and an id of no bundled model.
    const LISTED: DrivenModels<u8, 2> = DrivenModels::new([("so101", 1), ("no_such_model", 2)]);

    // A listed model is the bundled model of its id with what the table holds for it, in
    // the table's order; an id the table lacks is refused naming the listed ids, and a
    // listed id of no bundled model is refused naming the bundled ones.
    #[test]
    fn a_listed_model_is_its_bundled_model_and_any_other_is_refused() {
        let (model, held) = LISTED.model("so101").expect("a listed bundled model");
        assert_eq!((model.id(), held), ("so101", 1));
        assert_eq!(
            LISTED.into_iter().collect::<Vec<_>>(),
            [("so101", 1), ("no_such_model", 2)]
        );
        assert_eq!(
            LISTED.model("openarm_v2").unwrap_err().to_string(),
            "model openarm_v2 is no model this node drives; it drives \"so101\" and \
             \"no_such_model\""
        );
        assert!(matches!(
            LISTED.model("no_such_model"),
            Err(UndrivenModel::Unknown(_))
        ));
    }
}
