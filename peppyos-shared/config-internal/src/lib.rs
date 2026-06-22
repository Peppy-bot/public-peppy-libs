#![forbid(unsafe_code)]

//! Parsing and validation of Peppy node configuration documents.
//!
//! This crate owns the on-disk Peppy node config format (`peppy.json5`) and the
//! typed API the rest of the workspace builds on: parsing and validating those
//! documents, the node-argument/parameter schema, the runtime config types, and
//! codegen fingerprints.
//!
//! Boundaries: it performs no I/O beyond reading config files and codegen
//! fingerprints, and requires no global init — every type is built through an
//! explicit parser/constructor.

mod common;
mod error;
mod parsing;

/// Private module that contains all implementation modules.
/// The `#[path = "."]` attribute tells Rust to resolve child modules from `src/`,
/// the same directory as this file, so existing file paths are preserved.
#[path = "."]
mod internal {
    pub mod consts;
    pub mod fingerprint;
    pub mod launcher;
    pub mod node;
    pub mod peppy_config;
    pub mod repo_node_id;
    pub mod runtime;
    pub mod schema;
}

// -- launcher (only the `Name` identifier newtype survives; the launcher
// document parser is daemon-only and not part of this library) --
pub mod launcher {
    pub use crate::internal::launcher::Name;
}

// -- common --
pub use common::{
    AnyType, NodeArguments, NodeArgumentsError, ParameterSchema, ParameterSpec, TypeMismatch,
    validate_node_arguments,
};

// -- error --
pub use error::{Error as ConfigError, ParsingError};

// -- consts --
pub mod consts {
    pub use crate::internal::consts::{
        ALLOWED_CONFIG_CHARS, DEFAULT_LINK_ID_SENTINEL, DEFAULT_MESSAGING_HOST,
        DEFAULT_MESSAGING_PORT, NODE_CONFIG_FILE, PEPPYGEN_OUTPUT_PATH, RUNTIME_CONFIG_VAR_NAME,
    };
}

// -- fingerprint --
pub mod fingerprint {
    pub use crate::internal::fingerprint::read_codegen_fingerprint;

    #[cfg(feature = "test_helpers")]
    pub use crate::internal::fingerprint::{
        create_codegen_fingerprint, create_wrong_codegen_fingerprint,
    };
}

// -- node --
pub mod node {
    pub use crate::internal::node::{
        DependsOn, Name, NodeConfig, NodeConfigParser, NodeDependency, QoSProfile, Toolchain,
        TypeToken, load_standalone_node_config,
    };
}

// -- runtime --
pub mod runtime {
    pub use crate::internal::runtime::{
        DiscoveryConfig, NodeInstanceConfig, ProducerRef, RuntimeConfig, SlotBinding,
    };
}

// -- peppy_config --
pub mod peppy_config {
    pub use crate::internal::peppy_config::{
        DAEMON_HEARTBEAT_INTERVAL_SECS, DEFAULT_DAEMON_GRACE_SECS,
        DEFAULT_HIGH_THROUGHPUT_BUFFER_SIZE, DEFAULT_SHUTDOWN_GRACE_SECS,
        DEFAULT_STANDARD_BUFFER_SIZE, EVENT_LOOP_JOIN_BUDGET_SECS, PeerConfig,
    };
}

// -- schema (not part of the external API; re-exported so `crate::schema` paths
// inside `node` resolve) --
pub(crate) mod schema {
    pub(crate) use crate::internal::schema::PeppySchema;
}

// -- repo node id --
pub mod repo_node_id {
    pub use crate::internal::repo_node_id::{validate_repo_node_name, validate_repo_node_tag};
}
