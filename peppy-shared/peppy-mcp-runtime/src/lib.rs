//! The shared MCP server runtime composed by generated Peppy MCP server
//! nodes.
//!
//! A generated node parses its committed exposure bundle, registers one
//! bridge per exposed member, and hands both to this crate. The runtime then
//! owns everything protocol-shaped: the Streamable HTTP endpoint on
//! `127.0.0.1` speaking MCP `2026-07-28`, the catalog served through
//! `server/discover`, `tools/list`, and `resources/list` with their caching
//! hints, snapshot freshness on an injected clock, `subscriptions/listen`
//! notifications, tool-input validation against the bundle's derived
//! schemas, and the mapping from bridge failures to MCP errors.
//!
//! Entry points:
//!
//! - [`ExposureServer::builder`] takes a parsed
//!   [`ExposureBundle`](peppy_mcp_catalog::ExposureBundle) plus one
//!   [`ToolHandler`] per exposed service and builds the server.
//! - [`ExposureServer::ingest`] hands out the [`ResourceIngest`] a topic
//!   pump feeds: [`ResourceIngest::admit`] applies the update-rate gate
//!   before any decoding work, [`ResourceIngest::publish`] applies the
//!   representation and size policies and stores the snapshot.
//! - [`ExposureServer::serve`] binds the endpoint under
//!   [`MCP_HTTP_PATH`] until the supplied cancellation token fires.
//! - [`Clock`] injects the time source for freshness and rate gating; a
//!   generated node passes its sim-time-aware node clock, tests pass a
//!   counter.

#![forbid(unsafe_code)]

pub mod bridge;
pub mod clock;
pub mod error;
pub mod representation;
pub mod server;
mod state;

pub use clock::Clock;
pub use error::{BuildError, PublishError, ToolCallError};
pub use peppy_mcp_catalog as catalog;
pub use server::{ExposureServer, ExposureServerBuilder, MCP_HTTP_PATH, ToolHandler};
pub use state::ResourceIngest;
