# Peppy Messaging Interface (PMI)

This interface is used in the peppy cli as well as its controller libraries.

## Federation mTLS

Use `TlsConfig::mtls_client_with_system_roots` for a production federation
upstream whose server certificate chains to the operating system's WebPKI trust
store. The client certificate and private-key paths are rendered only on that
`UpstreamLink`; they do not alter the managed router's local listener. The DNS
hostname in `UpstreamLink::endpoint` is the TLS server name, so it must appear in
the federation server certificate. When `root_ca_certificate` is set, that PEM
bundle is the complete trust store: public/system roots are not merged into it.
When it is absent, both Zenoh and PMI's reachability probe use
`rustls-platform-verifier`, delegating to the operating system trust policy.
Federation mTLS negotiates TLS 1.3 only; ordinary one-way TLS retains rustls's
safe protocol defaults.

Zenoh 1.9 does not provide that private-root isolation upstream, so this
repository carries a narrowly scoped `vendor/zenoh-link-tls-1.9.0` patch. The
`build_zenoh` feature always compiles `zenohd@1.9.0` with that source patch and
uses a policy-and-content-tagged cache entry. Release builds start only an
adjacent packaged `zenohd` or that exact build artifact; arbitrary `PATH`
binaries are accepted only by debug builds. Because Cargo ignores `[patch]`
sections in dependencies, every downstream workspace that builds PMI/Zenoh
must repeat the `zenoh-link-tls` path patch at its own workspace root.

Certificate rotation requires new generation-specific certificate/key paths,
followed by `ZenohAdapter::refederate` and a managed-router reload. Replacing the
contents at unchanged paths does not cause a running zenohd process to reload
its identity.
