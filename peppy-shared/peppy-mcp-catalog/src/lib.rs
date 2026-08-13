//! The MCP exposure bundle format.
//!
//! An exposure bundle is the product of validating one `mcp_exposure/v1`
//! document against its sha256-pinned contracts: the public catalog (stable
//! names, prose, policies, derived JSON Schemas) plus the identity and
//! contract slots of the generated MCP server node. The peppy generator
//! writes bundles at publication time; the MCP server runtime parses them at
//! boot and serves exactly what they declare.
//!
//! Entry points:
//!
//! - [`ExposureBundle`] with [`ExposureBundle::from_json_str`] (reader) and
//!   [`ExposureBundle::to_json_string`] (writer, the committed bytes).
//! - [`EXPOSURE_BUNDLE_FORMAT`] and [`SCHEMA_MAPPING_VERSION`], the two
//!   version gates a reader checks before trusting a bundle.
//! - [`policy`], the operational policy vocabulary embedded in bundles and in
//!   the `mcp_exposure/v1` document model.

#![forbid(unsafe_code)]

pub mod bundle;
pub mod policy;

pub use bundle::{
    BundleContractPin, BundleIdentity, BundleNode, BundleServer, EXPOSURE_BUNDLE_FORMAT,
    ExposureBundle, I64_DECIMAL_PATTERN, ResourceEntry, ResourcePolicies, SCHEMA_MAPPING_VERSION,
    TaskEntry, ToolEntry, U64_DECIMAL_PATTERN, is_canonical_i64_decimal, is_canonical_u64_decimal,
};
pub use policy::{
    ActionOperation, FreshnessPolicy, ImageCodec, ImageFieldMap, ImageRepresentation, JpegQuality,
    MaxHz, OversizePolicy, ServiceOperation, UpdatePolicy,
};
