//! Organization namespace: the zenoh session-level namespace prefix that
//! routing-isolates one organization's robot traffic from another's across the
//! shared federation.
//!
//! The value is **always present**. Logged out it resolves to the constant
//! [`LOCAL_NAMESPACE`] (`"local"`); logged in it resolves to the organization id
//! (a stable UUID). A logged-out daemon never dials the shared cloud router, so
//! `local` never reaches it, while two logged-out robots on the same LAN share
//! `local` and interoperate. The value is parsed into a zenoh non-wild key
//! expression up front, so an invalid id can never reach a live session: zenoh's
//! egress prepends `<ns>/` to every declared key and ingress strips it, so two
//! sessions interoperate iff their namespaces are equal.

use std::fmt;

use zenoh_keyexpr::OwnedNonWildKeyExpr;

/// Namespace used whenever there is no organization id (logged out). A
/// logged-out daemon never federates, so `local` never reaches the shared cloud
/// router; two logged-out robots on the same LAN share it and interoperate.
pub const LOCAL_NAMESPACE: &str = "local";

/// A validated zenoh session namespace prefix for one organization.
///
/// Constructed only through [`OrgNamespace::parse`] or the infallible
/// [`OrgNamespace::local`]; the wrapped string is guaranteed to be a valid
/// non-wild key expression, so it is always safe to render into a zenoh session
/// config's `namespace` field.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct OrgNamespace(String);

/// A candidate organization id that is not a valid non-wild key expression
/// (empty, contains a wildcard such as `*`/`**`/`$*`, ...) and so cannot be used
/// as a zenoh session namespace.
#[derive(Debug, Clone, thiserror::Error)]
#[error("invalid organization namespace {value:?}: {message}")]
pub struct InvalidOrgNamespace {
    value: String,
    message: String,
}

impl InvalidOrgNamespace {
    /// The rejected value, for diagnostics.
    pub fn value(&self) -> &str {
        &self.value
    }
}

impl OrgNamespace {
    /// The local namespace used when logged out. Infallible: [`LOCAL_NAMESPACE`]
    /// is a constant, valid non-wild key expression.
    pub fn local() -> Self {
        Self::parse(LOCAL_NAMESPACE)
            .expect("LOCAL_NAMESPACE must be a valid non-wild key expression")
    }

    /// Parse and validate a candidate namespace (an organization id or the local
    /// constant). Rejects anything zenoh would not accept as a non-wild key
    /// expression so the value can never poison a live session.
    ///
    /// A namespace must be a *single* chunk: zenoh prepends `<ns>/` on egress and
    /// strips it chunk-by-chunk on ingress, so a multi-chunk value like `a/b`
    /// would collide with namespace `a` + key `b/...` and break org isolation. A
    /// UUID and the `local` constant contain no `/`, so this never rejects a
    /// legitimate id.
    pub fn parse(s: &str) -> Result<Self, InvalidOrgNamespace> {
        if s.contains('/') {
            return Err(InvalidOrgNamespace {
                value: s.to_owned(),
                message: "namespace must be a single chunk (must not contain '/')".to_owned(),
            });
        }
        match OwnedNonWildKeyExpr::try_from(s.to_owned()) {
            Ok(_) => Ok(Self(s.to_owned())),
            Err(err) => Err(InvalidOrgNamespace {
                value: s.to_owned(),
                message: err.to_string(),
            }),
        }
    }

    /// The validated namespace string, ready to render into a session config.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for OrgNamespace {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

/// Resolve a raw organization id into a session namespace. **Always returns a
/// namespace.** `None` (logged out) resolves to [`OrgNamespace::local`]; a valid
/// org id resolves to that org; an invalid org id resolves to `local` with a
/// warning (a UUID is always valid, so the invalid arm is defensive).
pub fn resolve_session_namespace(raw: Option<&str>) -> OrgNamespace {
    match raw {
        None => OrgNamespace::local(),
        Some(s) => OrgNamespace::parse(s).unwrap_or_else(|e| {
            tracing::warn!(
                value = %s,
                error = %e,
                "invalid organization id; falling back to the local namespace"
            );
            OrgNamespace::local()
        }),
    }
}

/// The single source of the federation gate. Fail-closed: only a present, valid
/// organization id that is not the local namespace federates. An absent id
/// (logged out), an invalid one, or the literal `local` keeps the router
/// standalone, so an unprefixed/`local` session can never reach the shared
/// multi-tenant router.
pub fn should_federate(raw: Option<&str>) -> bool {
    matches!(raw, Some(s) if s != LOCAL_NAMESPACE && OrgNamespace::parse(s).is_ok())
}

#[cfg(test)]
mod tests {
    use super::*;

    const UUID: &str = "550e8400-e29b-41d4-a716-446655440000";

    #[test]
    fn parse_accepts_uuid_and_local() {
        assert!(
            OrgNamespace::parse(UUID).is_ok(),
            "hyphenated UUID must parse"
        );
        assert!(OrgNamespace::parse(LOCAL_NAMESPACE).is_ok());
    }

    #[test]
    fn parse_rejects_multi_chunk() {
        for bad in ["a/b", "a/b/c"] {
            assert!(
                OrgNamespace::parse(bad).is_err(),
                "{bad:?} must be rejected: a namespace is a single chunk \
                 (multi-chunk collides once zenoh prepends the namespace)"
            );
        }
    }

    #[test]
    fn parse_rejects_empty_and_wildcards() {
        for bad in ["", "a*", "$*", "**"] {
            assert!(
                OrgNamespace::parse(bad).is_err(),
                "{bad:?} must be rejected as a namespace"
            );
        }
    }

    #[test]
    fn local_round_trips() {
        assert_eq!(OrgNamespace::local().as_str(), LOCAL_NAMESPACE);
        assert_eq!(
            OrgNamespace::local(),
            OrgNamespace::parse(LOCAL_NAMESPACE).unwrap()
        );
    }

    #[test]
    fn resolve_session_namespace_covers_none_bad_uuid() {
        assert_eq!(resolve_session_namespace(None), OrgNamespace::local());
        assert_eq!(
            resolve_session_namespace(Some("**")),
            OrgNamespace::local(),
            "an invalid org id falls back to local"
        );
        assert_eq!(resolve_session_namespace(Some(UUID)).as_str(), UUID);
    }

    #[test]
    fn should_federate_is_fail_closed() {
        assert!(!should_federate(None), "logged out never federates");
        assert!(
            !should_federate(Some("**")),
            "an invalid org id never federates"
        );
        assert!(
            !should_federate(Some(LOCAL_NAMESPACE)),
            "the local namespace is valid but must never federate"
        );
        assert!(should_federate(Some(UUID)), "a valid org id federates");
    }
}
