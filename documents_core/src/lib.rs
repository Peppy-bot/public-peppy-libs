//! What the documents of every side of peppy share.
//!
//! A document is JSON5 text whose root object carries one root key, such as
//! `simulation_schema`, naming the document's kind and version: `world/v1`.
//! The parser of a kind refuses a document whose root key or kind is not its
//! own before it reads another field ([`Document::parse`]). A document has a
//! manifest, `{ name, revision }`, and a digest: the sha256 of its canonical
//! JSON in the JSON Canonicalization Scheme of RFC 8785 ([`Digest`]).
//!
//! The value tree a document parses into holds only what that scheme holds
//! exactly: a key once per object and finite numbers within 2^53, integers
//! among them exact ([`parse`], and [`tree_of`] for a typed body). A refusal
//! names the path of the value.

mod digest;
mod document;
mod error;
mod kind;
mod manifest;
mod parse;
mod tree;

pub use digest::Digest;
pub use document::Document;
pub use error::Error;
pub use kind::Kind;
pub use manifest::{Manifest, Revision};
pub use parse::parse;
pub use tree::tree_of;
