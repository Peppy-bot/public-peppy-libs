//! What the integration suites share.
//!
//! Each integration test is its own crate, so items unused by one suite are
//! expected; the allow keeps clippy's -D warnings gate meaningful for the rest.
#![allow(dead_code)]

pub mod finite;
