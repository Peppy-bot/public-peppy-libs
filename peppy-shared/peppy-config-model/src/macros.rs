//! Macros for the validated string newtypes peppy keys repository items on.
//! Exported so the daemon-side repository model declares its own identities
//! (git commits, item names) with the same boilerplate the fingerprint here
//! uses, rather than a second copy of it.

/// Declares a fixed-width lowercase-hex string newtype: the struct, its
/// error enum, `parse`, `as_str`, `Display`, and the `Deserialize` that
/// routes through `parse` so a hand-edited document cannot state a value the
/// rest of peppy could never key on.
///
/// Parsing lowercases, so a value written by hand in upper case compares
/// equal to the one the tool that produced it reports.
#[macro_export]
macro_rules! hex_identity {
    (
        $(#[$meta:meta])*
        $name:ident,
        $error:ident {
            $empty:ident = $empty_msg:literal,
            $malformed:ident = $malformed_msg:literal $(,)?
        },
        $width:literal $(,)?
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, ::serde::Serialize)]
        #[serde(transparent)]
        pub struct $name(String);

        #[derive(Debug, ::thiserror::Error, PartialEq, Eq)]
        pub enum $error {
            #[error($empty_msg)]
            $empty,
            #[error($malformed_msg)]
            $malformed(String),
        }

        impl $name {
            pub fn parse(raw: &str) -> ::core::result::Result<Self, $error> {
                let value = raw.trim();
                if value.is_empty() {
                    return Err($error::$empty);
                }
                if value.len() != $width || !value.chars().all(|c| c.is_ascii_hexdigit()) {
                    return Err($error::$malformed(value.to_owned()));
                }
                Ok(Self(value.to_ascii_lowercase()))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl ::core::fmt::Display for $name {
            fn fmt(&self, f: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl<'de> ::serde::Deserialize<'de> for $name {
            fn deserialize<D>(deserializer: D) -> ::core::result::Result<Self, D::Error>
            where
                D: ::serde::de::Deserializer<'de>,
            {
                let raw = <String as ::serde::Deserialize>::deserialize(deserializer)?;
                Self::parse(&raw).map_err(::serde::de::Error::custom)
            }
        }

        $crate::compares_to_str!($name);
    };
}

/// Comparison against a plain `&str` for a validated string newtype whose
/// single field is a `String`.
///
/// A caller holding an unvalidated string has a question about equality, not
/// a value to construct: `entry.name == "uvc_camera"` should not have to
/// parse the literal first. Only the newtype-on-the-left direction is
/// emitted; the mirrored impls would be four more per type that nothing can
/// report as unreachable once they stop being called.
#[macro_export]
macro_rules! compares_to_str {
    ($ty:ty) => {
        impl ::core::cmp::PartialEq<str> for $ty {
            fn eq(&self, other: &str) -> bool {
                self.0 == other
            }
        }

        impl ::core::cmp::PartialEq<&str> for $ty {
            fn eq(&self, other: &&str) -> bool {
                self.0 == *other
            }
        }
    };
}
