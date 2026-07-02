#![forbid(unsafe_code)]

//! Parsing and validation of the shared Peppy configuration documents.
//!
//! This crate owns the wire-facing config tier every peppy surface builds
//! on: the `peppy.json5` node config model, the runtime configs shipped to
//! spawned nodes, codegen fingerprints, org namespaces, and the schema tags
//! identifying each document shape. (Cap'n Proto schema generation for a
//! [`node::MessageFormat`] lives in the separate `encoding` crate; the
//! daemon-side documents, launcher files, interface documents,
//! `peppy_config.json5`, and the `PeppyDirs` filesystem layout, live in the
//! peppyos `daemon-config` crate, which builds on this one.)
//!
//! Boundaries: it performs no I/O beyond reading/writing config files and
//! fingerprints, holds no process-global state, and requires no global
//! init; every type is built through an explicit parser/constructor.

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
    pub mod node;
    pub mod org;
    pub mod peppy_config;
    pub mod repo_node_id;
    pub mod runtime;
    pub mod schema;
}

// -- common --
pub use common::{
    AnyType, DefaultValue, NodeArguments, NodeArgumentsError, ParameterSchema, ParameterSpec,
    TypeMismatch, apply_parameter_defaults, resolve_argument_path, type_token_name,
    validate_node_arguments,
};

// -- error --
pub use error::{Error as ConfigError, MissingInterface, ParsingError};

// -- consts --
pub mod consts {
    pub use crate::internal::consts::{
        ALLOWED_CONFIG_CHARS, DEFAULT_LINK_ID_SENTINEL, DEFAULT_MESSAGING_HOST,
        DEFAULT_MESSAGING_PORT, NODE_CONFIG_FILE, PEPPY_HOME_ENV, PEPPYGEN_OUTPUT_PATH,
        PYTHON_MAX_VERSION, PYTHON_MIN_VERSION, RUNTIME_CONFIG_VAR_NAME,
    };
}

// -- fingerprint --
pub mod fingerprint {
    pub use crate::internal::fingerprint::{
        fingerprint_for_bytes, generate_node_config_fingerprint, read_codegen_fingerprint,
        verify_codegen_fingerprint,
    };

    #[cfg(feature = "fingerprint_test_helpers")]
    pub use crate::internal::fingerprint::{
        create_codegen_fingerprint, create_wrong_codegen_fingerprint,
    };
}

// -- node --
pub mod node {
    pub use crate::internal::node::{
        ActionInterfaces, ActionServiceEndpoint, ActionTopicEndpoint, ArrayKind, ArraySchema,
        ConformsToItem, ConsumedAction, ConsumedService, ConsumedTopic, ContainerConfig,
        DependencySpec, DependsOn, EmittedTopic, Execution, ExposedAction, ExposedService,
        InterfaceConformanceEdge, InterfaceKind, Interfaces, Manifest, MessageFormat,
        MessageSizeEstimate, NodeConfig, NodeConfigParser, NodeDependency, ObjectKind,
        ObjectSchema, PeppygenLanguage, PrimitiveSchema, QoSProfile, SchemaType, ServiceInterfaces,
        Toolchain, TopicInterfaces, TypeToken, collect_dependency_specs,
        collect_interface_conformance_edges, estimate_serialized_size, is_blocked_mount_source,
        load_standalone_node_config, node_conforms_to, validate_dependency_specs,
    };
}

// -- runtime --
pub mod runtime {
    pub use crate::internal::runtime::{
        DiscoveryConfig, LifecycleRuntimeConfig, Name, NodeInstanceConfig, ProducerRef,
        ResolvedFramework, RuntimeConfig, SlotBinding,
    };
}

// -- org --
pub mod org {
    pub use crate::internal::org::{
        InvalidOrgNamespace, LOCAL_NAMESPACE, OrgNamespace, resolve_session_namespace,
        should_federate,
    };
}

// -- peppy_config --
pub mod peppy_config {
    pub use crate::internal::peppy_config::{
        DEFAULT_DAEMON_GRACE_SECS, DEFAULT_HIGH_THROUGHPUT_BUFFER_SIZE,
        DEFAULT_SHUTDOWN_GRACE_SECS, DEFAULT_STANDARD_BUFFER_SIZE, EVENT_LOOP_JOIN_BUDGET_SECS,
        PeerConfig, RUNTIME_FINALIZE_MARGIN_SECS,
    };
}

// -- schema --
pub mod schema {
    pub use crate::internal::schema::PeppySchema;
}

// -- repo node id --
pub mod repo_node_id {
    pub use crate::internal::repo_node_id::{validate_repo_node_name, validate_repo_node_tag};
}
