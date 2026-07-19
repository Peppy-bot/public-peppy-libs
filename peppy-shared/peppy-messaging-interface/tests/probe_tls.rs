//! Unit tests for [`pmi::probe_tls_reachable`]: a raw rustls handshake against a
//! tokio-rustls TLS server built from the same fixtures as `tests/zenoh_tls.rs`
//! (`minica_ca.pem` CA, `server_localhost.pem` leaf with SAN `localhost`, and its
//! key). Deliberately needs neither `build_zenoh` nor a compiled `zenohd`: the
//! probe is a pure TLS handshake, so the test stands up its own bare TLS acceptor
//! rather than a zenoh router.

#![cfg(feature = "zenoh")]

mod probe_tls_tests {
    use pmi::{TlsConfig, probe_tls_reachable};
    use std::io::Write;
    use std::path::PathBuf;
    use std::sync::Arc;
    use std::time::Duration;
    use tokio_rustls::TlsAcceptor;
    use tokio_rustls::rustls::server::WebPkiClientVerifier;
    use tokio_rustls::rustls::{
        DEFAULT_VERSIONS, RootCertStore, ServerConfig, SupportedProtocolVersion,
    };

    const CA_PEM: &[u8] = include_bytes!("fixtures/minica_ca.pem");
    const SERVER_CERT_PEM: &[u8] = include_bytes!("fixtures/server_localhost.pem");
    const SERVER_KEY_PEM: &[u8] = include_bytes!("fixtures/server_localhost.key");

    const PROBE_TIMEOUT: Duration = Duration::from_secs(5);

    /// Writes embedded fixture PEM bytes to a tempfile and returns it.
    /// `TlsConfig` takes filesystem paths, so the fixture bytes are materialized.
    /// The returned `TempPath` keeps the file alive until dropped at end of test.
    fn pem_tempfile(bytes: &[u8]) -> tempfile::TempPath {
        let mut f = tempfile::NamedTempFile::new().expect("create PEM tempfile");
        f.write_all(bytes).expect("write PEM tempfile");
        f.flush().expect("flush PEM tempfile");
        f.into_temp_path()
    }

    /// Builds a tokio-rustls `TlsAcceptor` serving the `localhost` server leaf and
    /// its key (loaded from the embedded fixtures).
    fn server_acceptor(require_client_auth: bool) -> TlsAcceptor {
        server_acceptor_with_versions(require_client_auth, DEFAULT_VERSIONS)
    }

    fn server_acceptor_with_versions(
        require_client_auth: bool,
        protocol_versions: &[&'static SupportedProtocolVersion],
    ) -> TlsAcceptor {
        let chain = rustls_pemfile::certs(&mut &SERVER_CERT_PEM[..])
            .collect::<Result<Vec<_>, _>>()
            .expect("parse server cert chain");
        let key = rustls_pemfile::private_key(&mut &SERVER_KEY_PEM[..])
            .expect("read server key")
            .expect("server key present");
        // Name the `ring` crypto provider explicitly: rustls is built with both
        // its `ring` and `aws-lc-rs` features in this tree, so it cannot pick a
        // process-default provider and a bare `ServerConfig::builder()` panics.
        let provider = Arc::new(tokio_rustls::rustls::crypto::ring::default_provider());
        let builder = ServerConfig::builder_with_provider(provider.clone())
            .with_protocol_versions(protocol_versions)
            .expect("selected protocol versions");
        let builder = if require_client_auth {
            let mut roots = RootCertStore::empty();
            for cert in rustls_pemfile::certs(&mut &CA_PEM[..]) {
                roots
                    .add(cert.expect("parse client CA"))
                    .expect("add client CA");
            }
            let verifier = WebPkiClientVerifier::builder_with_provider(roots.into(), provider)
                .build()
                .expect("build client verifier");
            builder.with_client_cert_verifier(verifier)
        } else {
            builder.with_no_client_auth()
        };
        let config = builder
            .with_single_cert(chain, key)
            .expect("build server config");
        TlsAcceptor::from(Arc::new(config))
    }

    /// Binds a TLS acceptor on `127.0.0.1:0`, spawns an accept loop that performs
    /// the TLS handshake on each connection (then drops it), and returns the bound
    /// port. The loop task lives for the duration of the test process.
    async fn start_tls_server(require_client_auth: bool) -> u16 {
        let acceptor = server_acceptor(require_client_auth);
        start_tls_server_with_acceptor(acceptor).await
    }

    async fn start_tls_server_with_acceptor(acceptor: TlsAcceptor) -> u16 {
        let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0))
            .await
            .expect("bind tls test listener");
        let port = listener.local_addr().expect("local addr").port();
        tokio::spawn(async move {
            loop {
                let Ok((stream, _)) = listener.accept().await else {
                    break;
                };
                let acceptor = acceptor.clone();
                tokio::spawn(async move {
                    // Keep an accepted TLS connection open beyond the probe's
                    // post-handshake alert grace window. A healthy long-lived
                    // Zenoh listener does the same while it waits for protocol
                    // traffic from the client.
                    if let Ok(stream) = acceptor.accept(stream).await {
                        tokio::time::sleep(Duration::from_millis(400)).await;
                        drop(stream);
                    }
                });
            }
        });
        port
    }

    /// Correct CA + matching server name (`localhost` SAN) → the handshake
    /// validates. We dial `localhost`, which resolves to the `127.0.0.1` listener
    /// while still matching the leaf's SAN.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn correct_ca_and_name_succeeds() {
        let port = start_tls_server(false).await;
        let ca = pem_tempfile(CA_PEM);
        let tls = TlsConfig::client(PathBuf::from(&*ca));

        let result = probe_tls_reachable("localhost", port, &tls, PROBE_TIMEOUT).await;
        assert!(result.is_ok(), "expected Ok, got {result:?}");
    }

    /// No CA → system roots, which do not trust the private `minica` CA, so the
    /// chain cannot be validated.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn untrusted_system_roots_fails() {
        let port = start_tls_server(false).await;
        let tls = TlsConfig::default();

        let result = probe_tls_reachable("localhost", port, &tls, PROBE_TIMEOUT).await;
        assert!(
            result.is_err(),
            "expected Err (untrusted CA), got {result:?}"
        );
    }

    /// A supplied identity cannot be silently ignored when mTLS is disabled.
    /// Validation happens before material loading or the network dial, so these
    /// intentionally nonexistent paths still produce the configuration error.
    #[tokio::test]
    async fn disabled_mtls_identity_is_rejected_before_the_probe_dials() {
        let tls = TlsConfig {
            connect_certificate: Some("/does-not-exist/client-chain.pem".into()),
            connect_private_key: Some("/does-not-exist/client-key.pem".into()),
            ..TlsConfig::default()
        };

        let error = probe_tls_reachable("127.0.0.1", 1, &tls, PROBE_TIMEOUT)
            .await
            .expect_err("a disabled client identity must be rejected");
        assert!(
            error.contains("identity is configured while mTLS is disabled"),
            "unexpected probe error: {error}"
        );
    }

    /// Correct CA but a server name (`127.0.0.1`) that does not match the leaf's
    /// SAN (`localhost`) → name verification (always on) rejects it.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn name_mismatch_fails() {
        let port = start_tls_server(false).await;
        let ca = pem_tempfile(CA_PEM);
        let tls = TlsConfig::client(PathBuf::from(&*ca));

        let result = probe_tls_reachable("127.0.0.1", port, &tls, PROBE_TIMEOUT).await;
        assert!(
            result.is_err(),
            "expected Err (name mismatch), got {result:?}"
        );
    }

    /// An mTLS probe presents its configured connect certificate and key. The
    /// server trusts the fixture CA and accepts the dual-EKU fixture leaf as a
    /// client identity.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn mtls_connect_identity_succeeds() {
        let port = start_tls_server(true).await;
        let ca = pem_tempfile(CA_PEM);
        let certificate = pem_tempfile(SERVER_CERT_PEM);
        let private_key = pem_tempfile(SERVER_KEY_PEM);
        let tls = TlsConfig::mtls_client(
            PathBuf::from(&*ca),
            PathBuf::from(&*certificate),
            PathBuf::from(&*private_key),
        );

        let start = tokio::time::Instant::now();
        let result = probe_tls_reachable("localhost", port, &tls, PROBE_TIMEOUT).await;
        let elapsed = start.elapsed();

        assert!(result.is_ok(), "expected Ok, got {result:?}");
        // The server engaged client auth, so its verdict on our certificate can
        // arrive after the handshake — the probe must sit out the full 250ms
        // post-handshake grace window before declaring the link good.
        assert!(
            elapsed >= Duration::from_millis(250),
            "a client-auth probe must keep the post-handshake grace, took {elapsed:?}"
        );
    }

    /// The probe must mirror Zenoh's protocol policy: ordinary TLS keeps safe
    /// compatibility defaults, while federation mTLS is TLS 1.3-only.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn tls12_only_server_is_accepted_for_plain_tls_but_rejected_for_mtls() {
        use tokio_rustls::rustls::version::TLS12;

        let ca = pem_tempfile(CA_PEM);
        let plain_port =
            start_tls_server_with_acceptor(server_acceptor_with_versions(false, &[&TLS12])).await;
        let plain_tls = TlsConfig::client(PathBuf::from(&*ca));
        assert!(
            probe_tls_reachable("localhost", plain_port, &plain_tls, PROBE_TIMEOUT)
                .await
                .is_ok(),
            "plain TLS should retain safe TLS 1.2 compatibility"
        );

        let certificate = pem_tempfile(SERVER_CERT_PEM);
        let private_key = pem_tempfile(SERVER_KEY_PEM);
        let mtls_port =
            start_tls_server_with_acceptor(server_acceptor_with_versions(true, &[&TLS12])).await;
        let mtls = TlsConfig::mtls_client(
            PathBuf::from(&*ca),
            PathBuf::from(&*certificate),
            PathBuf::from(&*private_key),
        );
        assert!(
            probe_tls_reachable("localhost", mtls_port, &mtls, PROBE_TIMEOUT)
                .await
                .is_err(),
            "mTLS must not negotiate a TLS 1.2-only endpoint"
        );
    }

    /// If the probe's overall deadline leaves less than the full grace interval,
    /// expiring that shortened interval is a probe timeout, not proof that the
    /// server accepted the client certificate.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn truncated_post_handshake_grace_fails() {
        let port = start_tls_server(true).await;
        let ca = pem_tempfile(CA_PEM);
        let certificate = pem_tempfile(SERVER_CERT_PEM);
        let private_key = pem_tempfile(SERVER_KEY_PEM);
        let tls = TlsConfig::mtls_client(
            PathBuf::from(&*ca),
            PathBuf::from(&*certificate),
            PathBuf::from(&*private_key),
        );

        let result = probe_tls_reachable("localhost", port, &tls, Duration::from_millis(200)).await;

        let error = result.expect_err("a truncated post-handshake grace must time out");
        assert!(
            error.contains("post-handshake check"),
            "unexpected truncated-grace error: {error}"
        );
    }

    /// A probe with no client identity against a listener that REQUIRES client
    /// certificates. In TLS 1.3 the client sends an empty Certificate and
    /// completes its side of the handshake before the server's
    /// `certificate_required` alert arrives, so only the post-handshake grace
    /// read can surface the rejection. Guards the silent-failure class the
    /// grace exists for — gating it on our local mTLS setting (instead of on
    /// the server engaging client auth) would report this link as validating.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn plain_probe_against_client_auth_server_fails() {
        let port = start_tls_server(true).await;
        let ca = pem_tempfile(CA_PEM);
        let tls = TlsConfig::client(PathBuf::from(&*ca));

        let result = probe_tls_reachable("localhost", port, &tls, PROBE_TIMEOUT).await;
        assert!(
            result.is_err(),
            "expected Err (client certificate required), got {result:?}"
        );
    }

    /// A server that never engaged client-certificate auth cannot reject after
    /// the handshake, so the probe returns as soon as the handshake validates
    /// instead of sitting out the 250ms post-handshake grace (the server here
    /// holds every accepted connection open for 400ms, past that window).
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn plain_tls_probe_returns_without_the_post_handshake_grace() {
        let port = start_tls_server(false).await;
        let ca = pem_tempfile(CA_PEM);
        let tls = TlsConfig::client(PathBuf::from(&*ca));

        let start = tokio::time::Instant::now();
        let result = probe_tls_reachable("localhost", port, &tls, PROBE_TIMEOUT).await;
        let elapsed = start.elapsed();

        assert!(result.is_ok(), "expected Ok, got {result:?}");
        assert!(
            elapsed < Duration::from_millis(250),
            "a no-client-auth probe must skip the grace wait, took {elapsed:?}"
        );
    }

    /// A peer that completes the TCP accept but never drives the TLS handshake
    /// must fail within ~`timeout` TOTAL (the connect + handshake share one
    /// deadline), not hang or stretch toward 2x. Guards the single-deadline bound
    /// the daemon's federation ack budget relies on.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn stalled_handshake_fails_within_total_timeout() {
        let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0))
            .await
            .expect("bind stalling listener");
        let port = listener.local_addr().expect("local addr").port();
        // Accept connections but never perform the TLS handshake — hold the raw
        // TCP streams open so the probe's handshake phase stalls to its deadline.
        tokio::spawn(async move {
            let mut held = Vec::new();
            while let Ok((stream, _)) = listener.accept().await {
                held.push(stream);
            }
        });

        let ca = pem_tempfile(CA_PEM);
        let tls = TlsConfig::client(PathBuf::from(&*ca));

        let timeout = Duration::from_secs(1);
        let start = tokio::time::Instant::now();
        let result = probe_tls_reachable("localhost", port, &tls, timeout).await;
        let elapsed = start.elapsed();

        assert!(
            result.is_err(),
            "a stalled handshake must fail, got {result:?}"
        );
        // Single shared deadline ⇒ the whole probe stays within ~timeout (allowing
        // generous scheduling slack), never ~2x.
        assert!(
            elapsed < timeout + Duration::from_millis(800),
            "probe must be bounded by ~timeout total, took {elapsed:?}"
        );
    }

    /// A port with no listener → the connect (or handshake) fails within the
    /// timeout, returning `Err` rather than hanging.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn unreachable_port_fails_within_timeout() {
        // Reserve then release a port so nothing is listening on it.
        let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0))
            .await
            .expect("reserve port");
        let port = listener.local_addr().expect("local addr").port();
        drop(listener);

        let ca = pem_tempfile(CA_PEM);
        let tls = TlsConfig::client(PathBuf::from(&*ca));

        let result = probe_tls_reachable("127.0.0.1", port, &tls, Duration::from_secs(2)).await;
        assert!(
            result.is_err(),
            "expected Err (unreachable port), got {result:?}"
        );
    }

    /// Zenoh endpoint parsing retains brackets around IPv6 literals. The raw TLS
    /// probe must remove them before both rustls name parsing and TCP dialing.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn bracketed_ipv6_host_is_normalized() {
        let Ok(listener) = tokio::net::TcpListener::bind(("::1", 0)).await else {
            return;
        };
        let port = listener.local_addr().expect("local IPv6 addr").port();
        drop(listener);
        let ca = pem_tempfile(CA_PEM);
        let tls = TlsConfig::client(PathBuf::from(&*ca));

        let error = probe_tls_reachable("[::1]", port, &tls, Duration::from_secs(1))
            .await
            .expect_err("released IPv6 port must be unreachable");

        assert!(
            error.contains(&format!("to ::1:{port}")),
            "probe did not use the normalized IPv6 host: {error}"
        );
    }
}
