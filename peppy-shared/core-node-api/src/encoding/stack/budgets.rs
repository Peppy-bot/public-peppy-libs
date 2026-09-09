//! The environment and time budgets shared by every goal that adds nodes to
//! a stack: `stack launch`, `stack join` and `stack remove`.

use crate::Result;
use crate::encoding::{required_text, with_default};
use crate::launch_capnp;

/// Default idle timeout in seconds for the add/build/run phases (used as fallback when 0 is
/// received on the wire; Cap'n Proto defaults unset `UInt64` to 0).
pub const DEFAULT_IDLE_TIMEOUT_SECS: u64 = 600;

/// What every goal that adds nodes to a stack runs under: the caller's
/// environment, forwarded to the builds and runs on the coordinator, and
/// the idle budget of each phase. On the wire an absent timeout is 0 and
/// decodes to its default.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StackBudgets {
    pub env_vars: Vec<(String, String)>,
    pub node_add_idle_timeout_secs: u64,
    pub node_build_idle_timeout_secs: u64,
    pub node_run_idle_timeout_secs: u64,
    /// Whole-operation deadline. `None` means no overall deadline is
    /// enforced (only idle timeouts apply); the wire carries 0 for it.
    pub max_timeout_secs: Option<u64>,
}

impl Default for StackBudgets {
    fn default() -> Self {
        Self {
            env_vars: Vec::new(),
            node_add_idle_timeout_secs: DEFAULT_IDLE_TIMEOUT_SECS,
            node_build_idle_timeout_secs: DEFAULT_IDLE_TIMEOUT_SECS,
            node_run_idle_timeout_secs: DEFAULT_IDLE_TIMEOUT_SECS,
            max_timeout_secs: None,
        }
    }
}

impl StackBudgets {
    pub fn new(
        node_add_idle_timeout_secs: u64,
        node_build_idle_timeout_secs: u64,
        node_run_idle_timeout_secs: u64,
        max_timeout_secs: Option<u64>,
    ) -> Self {
        Self {
            env_vars: Vec::new(),
            node_add_idle_timeout_secs,
            node_build_idle_timeout_secs,
            node_run_idle_timeout_secs,
            max_timeout_secs,
        }
    }

    pub fn with_env_vars(mut self, env_vars: Vec<(String, String)>) -> Self {
        self.env_vars = env_vars;
        self
    }

    /// The budgets as one goal's wire fields carry them. A defaulted variable
    /// name is refused: the receiving side exports what it is handed, and
    /// there is no variable called the empty string.
    pub(super) fn decode(
        env_vars: capnp::struct_list::Reader<'_, launch_capnp::env_var::Owned>,
        node_add_idle_timeout_secs: u64,
        node_build_idle_timeout_secs: u64,
        node_run_idle_timeout_secs: u64,
        max_timeout_secs: u64,
    ) -> Result<Self> {
        let env_vars = env_vars
            .iter()
            .map(|entry| {
                Ok((
                    required_text(entry.get_key()?.to_str()?, "env_vars.key")?,
                    entry.get_value()?.to_str()?.to_owned(),
                ))
            })
            .collect::<Result<_>>()?;
        Ok(Self {
            env_vars,
            node_add_idle_timeout_secs: with_default(
                node_add_idle_timeout_secs,
                DEFAULT_IDLE_TIMEOUT_SECS,
            ),
            node_build_idle_timeout_secs: with_default(
                node_build_idle_timeout_secs,
                DEFAULT_IDLE_TIMEOUT_SECS,
            ),
            node_run_idle_timeout_secs: with_default(
                node_run_idle_timeout_secs,
                DEFAULT_IDLE_TIMEOUT_SECS,
            ),
            max_timeout_secs: (max_timeout_secs > 0).then_some(max_timeout_secs),
        })
    }

    pub(super) fn write_env_vars(
        &self,
        mut list: capnp::struct_list::Builder<'_, launch_capnp::env_var::Owned>,
    ) {
        for (index, (key, value)) in self.env_vars.iter().enumerate() {
            let mut entry = list.reborrow().get(index as u32);
            entry.set_key(key);
            entry.set_value(value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::encoding::decoding_error;
    use capnp::message::Builder;

    /// The budgets of a goal carrying `env_vars` and those wire timeouts, in
    /// the order `decode` reads them.
    fn decode_budgets(env_vars: &[(&str, &str)], timeouts: [u64; 4]) -> Result<StackBudgets> {
        let mut message = Builder::new_default();
        {
            let goal = message.init_root::<launch_capnp::launch_goal::Builder>();
            let mut wire = goal.init_env_vars(env_vars.len() as u32);
            for (index, (key, value)) in env_vars.iter().enumerate() {
                let mut entry = wire.reborrow().get(index as u32);
                entry.set_key(key);
                entry.set_value(value);
            }
        }
        let goal = message
            .get_root_as_reader::<launch_capnp::launch_goal::Reader>()
            .expect("root");
        let [add, build, run, max] = timeouts;
        StackBudgets::decode(goal.get_env_vars().expect("env vars"), add, build, run, max)
    }

    /// Zero on the wire is an absent budget: the idle timeouts decode to their
    /// default and the whole-operation deadline to `None`.
    #[test]
    fn absent_budgets_decode_to_their_defaults() {
        assert_eq!(
            decode_budgets(&[], [0, 0, 0, 0]).expect("decode"),
            StackBudgets::default()
        );
    }

    /// An empty VALUE is a variable set to the empty string, which is a thing
    /// a caller's environment holds, so it survives.
    #[test]
    fn budgets_carry_their_timeouts_and_environment() {
        let budgets = decode_budgets(
            &[("PATH", "/usr/bin"), ("PEPPY_QUIET", "")],
            [42, 900, 73, 1200],
        )
        .expect("decode");
        assert_eq!(
            budgets,
            StackBudgets::new(42, 900, 73, Some(1200)).with_env_vars(vec![
                ("PATH".to_owned(), "/usr/bin".to_owned()),
                ("PEPPY_QUIET".to_owned(), String::new()),
            ])
        );
    }

    /// A defaulted key names no variable, so the decode refuses it.
    #[test]
    fn an_empty_env_var_key_is_refused() {
        let refusal = decoding_error(decode_budgets(&[("", "/usr/bin")], [0, 0, 0, 0]));
        assert!(refusal.contains("env_vars.key"), "{refusal}");
    }
}
