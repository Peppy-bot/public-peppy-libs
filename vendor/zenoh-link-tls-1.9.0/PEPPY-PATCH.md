# Peppy transport trust patch

This directory vendors the source published as `zenoh-link-tls` 1.9.0 from
<https://crates.io/crates/zenoh-link-tls/1.9.0>. The original source headers,
authors, repository, and dual `EPL-2.0 OR Apache-2.0` license declaration are
preserved. `.cargo_vcs_info.json` records upstream commit
`81c6c933b6e41d72a05f04c4442ef57717ddc72b`.
The upstream `LICENSE` and `NOTICE.md` files are included alongside the source.

Peppy's local patch changes only client-side trust construction:

- an explicitly configured CA bundle is the complete, exclusive trust store;
- when no bundle is configured, certificate verification is delegated to
  `rustls-platform-verifier` and the operating system trust policy;
- mTLS clients continue to negotiate TLS 1.3 only.

Patched binaries carry the stable, non-secret marker
`PEPPY_ZENOHD_TLS_POLICY=custom-roots-exclusive;system-roots=platform-verifier;mtls=tls13;v=1`
so release tooling can distinguish them from stock Zenoh artifacts.

Keep this patch narrow. Rebase it onto the exact Zenoh version whenever Zenoh
is upgraded, and increment the policy tag in `peppy-messaging-interface/build.rs`
for any policy change.
