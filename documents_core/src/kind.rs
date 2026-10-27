use std::fmt;

/// A kind of document on one side: the side's root key, the kind's name and
/// its version, which the root key's value names as `<name>/v<version>`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Kind {
    root_key: &'static str,
    name: &'static str,
    version: u32,
}

impl Kind {
    pub const fn new(root_key: &'static str, name: &'static str, version: u32) -> Kind {
        Kind {
            root_key,
            name,
            version,
        }
    }

    /// The root key of the side, such as `simulation_schema`.
    pub(crate) fn root_key(&self) -> &'static str {
        self.root_key
    }

    /// The value of the root key, such as `world/v1`.
    pub(crate) fn tag(&self) -> String {
        format!("{}/v{}", self.name, self.version)
    }
}

impl fmt::Display for Kind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: \"{}\"", self.root_key, self.tag())
    }
}
