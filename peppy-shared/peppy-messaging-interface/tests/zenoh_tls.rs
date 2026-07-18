//! End-to-end TLS transport tests: a real `zenohd` router listening on `tls/`
//! and a `connect_to_tls` client completing a TLS handshake and a pub/sub
//! round-trip. This is the *authoritative* check that the `transport.link.tls`
//! block pmi renders is actually accepted by zenoh — a wrong key would be
//! silently dropped at config parse, so only a live handshake proves it.
//!
//! Gated behind `build_zenoh` like the other integration tests (it needs the
//! compiled `zenohd` binary). Fixture certs live in `tests/fixtures/` and were
//! lifted verbatim from zenoh 1.9's own `tests/authentication.rs` (a `minica`
//! CA + a `localhost` server leaf); see that file for provenance.

#![cfg(feature = "build_zenoh")]

mod common;

mod zenoh_tls_tests {
    use crate::common::{
        RECV_TIMEOUT, ZENOH_SERIAL, receiver, sender, wait_for_subscriber_discovery,
    };
    use bytes::Bytes;
    use pmi::{
        Messenger, MessengerAdapter, MessengerBackend, Payload, PublisherQoS, RouterLinks,
        SubscriberBufferSizes, SubscriberQoS, TlsConfig, ZenohAdapter, ZenohNetProtocol,
        probe_tls_reachable,
    };
    use rcgen::{CertifiedKey, generate_simple_self_signed};
    use std::io::Write;
    use std::path::PathBuf;
    use std::time::Duration;

    const CA_PEM: &[u8] = include_bytes!("fixtures/minica_ca.pem");
    const SERVER_CERT_PEM: &[u8] = include_bytes!("fixtures/server_localhost.pem");
    const SERVER_KEY_PEM: &[u8] = include_bytes!("fixtures/server_localhost.key");

    /// Cert files materialized into a tempdir for a single test. zenoh's TLS
    /// config takes filesystem paths, so the embedded fixtures are written out.
    struct Certs {
        // Held only for its `Drop` (cleans up the tempdir at end of test).
        #[allow(dead_code)]
        dir: tempfile::TempDir,
        ca: PathBuf,
        cert: PathBuf,
        key: PathBuf,
    }

    fn write_certs() -> Certs {
        let dir = tempfile::tempdir().expect("create cert tempdir");
        let put = |name: &str, bytes: &[u8]| {
            let path = dir.path().join(name);
            let mut f = std::fs::File::create(&path).expect("create cert file");
            f.write_all(bytes).expect("write cert file");
            path
        };
        let ca = put("ca.pem", CA_PEM);
        let cert = put("server.pem", SERVER_CERT_PEM);
        let key = put("server.key", SERVER_KEY_PEM);
        Certs { dir, ca, cert, key }
    }

    /// Materializes a self-signed identity that the fixture CA does not trust.
    /// It is used to prove the fragment-configured listener rejects a client
    /// certificate from the wrong CA. Self-signed means the leaf is its own
    /// trust root, so one file serves as both `ca` and `cert`.
    fn write_rogue_identity() -> Certs {
        let dir = tempfile::tempdir().expect("create rogue cert tempdir");
        let CertifiedKey { cert, signing_key } =
            generate_simple_self_signed(vec!["rogue.local".to_string()])
                .expect("generate rogue identity");
        let put = |name: &str, contents: &str| {
            let path = dir.path().join(name);
            std::fs::write(&path, contents).expect("write rogue identity file");
            path
        };
        let cert = put("rogue.pem", &cert.pem());
        let key = put("rogue.key", &signing_key.serialize_pem());
        Certs {
            dir,
            ca: cert.clone(),
            cert,
            key,
        }
    }

    /// The server leaf's SAN is `localhost`, but we dial `127.0.0.1` (no DNS
    /// resolution ambiguity), so name verification is off — exactly how zenoh's
    /// own TLS test uses these fixtures. The CA-trust check stays on, which is
    /// what the negative test below exercises.
    fn trusting_client_tls(certs: &Certs) -> TlsConfig {
        TlsConfig {
            verify_name_on_connect: false,
            ..TlsConfig::client(certs.ca.clone())
        }
    }

    /// Starts a `zenohd` router listening on `tls/127.0.0.1:<port>` with the
    /// server leaf/key. Returns the owning `Messenger` (drop it to stop zenohd)
    /// and the port. `gossip = false`: the router seeds nothing extra here.
    async fn start_tls_router(certs: &Certs) -> (Messenger, u16) {
        let listener = std::net::TcpListener::bind(("127.0.0.1", 0)).expect("reserve port");
        let port = listener.local_addr().expect("local addr").port();
        drop(listener);

        let adapter = ZenohAdapter::with_router(
            ZenohNetProtocol::Tls,
            "127.0.0.1",
            port,
            false,
            SubscriberBufferSizes::default(),
            RouterLinks {
                tls: Some(TlsConfig::server(certs.cert.clone(), certs.key.clone())),
                ..RouterLinks::default()
            },
        )
        .expect("build tls router adapter");
        let mut messenger = Messenger::new(MessengerAdapter::Zenoh(adapter));
        messenger
            .start_router()
            .await
            .expect("start tls zenohd router");
        (messenger, port)
    }

    /// Opens a `tls/` client session, retrying briefly while the router's TLS
    /// listener finishes settling after the TCP socket starts accepting.
    async fn open_tls_client(port: u16, tls: &TlsConfig) -> ZenohAdapter {
        for _ in 0..40 {
            if let Ok(mut adapter) = ZenohAdapter::connect_to_tls("127.0.0.1", port, tls.clone())
                && adapter.start_session().await.is_ok()
            {
                return adapter;
            }
            tokio::time::sleep(Duration::from_millis(250)).await;
        }
        panic!("could not open a tls client session on 127.0.0.1:{port}");
    }

    /// Starts a `zenohd` router that serves local clients over plaintext `tcp/`
    /// AND *federates* to a remote `tls/` router at `remote_port`. This is the
    /// peppy daemon's shape in the per-user-router design: local nodes speak
    /// plaintext loopback, and only the inter-router hop is encrypted.
    async fn start_federated_router(certs: &Certs, remote_port: u16) -> (Messenger, u16) {
        let listener = std::net::TcpListener::bind(("127.0.0.1", 0)).expect("reserve port");
        let port = listener.local_addr().expect("local addr").port();
        drop(listener);

        let adapter = ZenohAdapter::with_router(
            ZenohNetProtocol::Tcp,
            "127.0.0.1",
            port,
            false,
            SubscriberBufferSizes::default(),
            // Federate to the remote TLS router, trusting it via the same CA the
            // TLS clients use (name verification off because the leaf's SAN is
            // `localhost` while we dial `127.0.0.1` — the CA-trust check stays on).
            RouterLinks {
                connect_endpoints: vec![format!("tls/127.0.0.1:{remote_port}")],
                tls: Some(trusting_client_tls(certs)),
                ..RouterLinks::default()
            },
        )
        .expect("build federated router adapter");
        let mut messenger = Messenger::new(MessengerAdapter::Zenoh(adapter));
        messenger
            .start_router()
            .await
            .expect("start federated zenohd router");
        (messenger, port)
    }

    /// Opens a plaintext `tcp/` client session to a local router, retrying while
    /// the listener settles.
    async fn open_plaintext_client(port: u16) -> ZenohAdapter {
        for _ in 0..40 {
            if let Ok(mut adapter) =
                ZenohAdapter::connect_to(ZenohNetProtocol::Tcp, "127.0.0.1", port)
                && adapter.start_session().await.is_ok()
            {
                return adapter;
            }
            tokio::time::sleep(Duration::from_millis(250)).await;
        }
        panic!("could not open a plaintext client session on 127.0.0.1:{port}");
    }

    /// A real zenohd keeps per-endpoint TLS fragments local to the TLS listener:
    /// its primary listener remains plaintext, the fragment listener requires a
    /// client certificate, and no global TLS block is needed. This pins Zenoh
    /// 1.9's endpoint-fragment merge behavior used by federation.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn fragment_mtls_listener_accepts_right_cert_and_rejects_wrong_ca() {
        let _lock = ZENOH_SERIAL.lock().await;
        let certs = write_certs();
        let rogue = write_rogue_identity();

        let primary_listener =
            std::net::TcpListener::bind(("127.0.0.1", 0)).expect("reserve primary port");
        let primary_port = primary_listener.local_addr().expect("primary addr").port();
        let tls_listener = std::net::TcpListener::bind(("127.0.0.1", 0)).expect("reserve TLS port");
        let tls_port = tls_listener.local_addr().expect("TLS addr").port();
        drop(primary_listener);
        drop(tls_listener);

        let fragment_listener = format!(
            concat!(
                "tls/127.0.0.1:{tls_port}#",
                "root_ca_certificate_file={ca};",
                "listen_certificate_file={cert};",
                "listen_private_key_file={key};",
                "enable_mtls=true"
            ),
            tls_port = tls_port,
            ca = certs.ca.display(),
            cert = certs.cert.display(),
            key = certs.key.display(),
        );
        let adapter = ZenohAdapter::with_router(
            ZenohNetProtocol::Tcp,
            "127.0.0.1",
            primary_port,
            false,
            SubscriberBufferSizes::default(),
            RouterLinks {
                extra_listen_endpoints: vec![fragment_listener],
                ..RouterLinks::default()
            },
        )
        .expect("build router with fragment mTLS listener");
        let mut router = Messenger::new(MessengerAdapter::Zenoh(adapter));
        router.start_router().await.expect("start fragment router");

        let plaintext_client = open_plaintext_client(primary_port).await;
        let right_client =
            TlsConfig::mtls_client(certs.ca.clone(), certs.cert.clone(), certs.key.clone());
        let mut last_error = None;
        for _ in 0..20 {
            match probe_tls_reachable("localhost", tls_port, &right_client, Duration::from_secs(1))
                .await
            {
                Ok(()) => {
                    last_error = None;
                    break;
                }
                Err(error) => {
                    last_error = Some(error);
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            }
        }
        assert!(
            last_error.is_none(),
            "right-CA client did not reach the fragment listener: {last_error:?}"
        );

        // Trusts the genuine server CA but presents the rogue (untrusted-CA)
        // client identity.
        let wrong_client =
            TlsConfig::mtls_client(certs.ca.clone(), rogue.cert.clone(), rogue.key.clone());
        let error =
            probe_tls_reachable("localhost", tls_port, &wrong_client, Duration::from_secs(2))
                .await
                .expect_err("wrong-CA client certificate must be rejected");
        assert!(
            error.contains("handshake") || error.contains("rejected") || error.contains("closed"),
            "unexpected wrong-CA rejection: {error}"
        );

        drop(plaintext_client);
        router.stop_router().await.expect("stop fragment router");
    }

    /// Positive path: a TLS router + two TLS clients complete the handshake and
    /// deliver a message end-to-end. Proves the rendered `transport.link.tls`
    /// block is valid and the encrypted transport actually carries traffic.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn tls_router_and_client_round_trip() {
        const TOPIC: &str = "tls_round_trip";
        let _lock = ZENOH_SERIAL.lock().await;
        let certs = write_certs();
        let (_router, port) = start_tls_router(&certs).await;
        let client_tls = trusting_client_tls(&certs);

        let subscriber = open_tls_client(port, &client_tls).await;
        let subscription = subscriber
            .subscribe_topic(&receiver(TOPIC), SubscriberQoS::Standard)
            .await
            .expect("subscribe over tls");

        let mut publisher = open_tls_client(port, &client_tls).await;
        wait_for_subscriber_discovery().await;
        publisher
            .publish_topic(
                &sender(TOPIC),
                Payload::from_bytes(Bytes::from_static(b"tls-hello")),
                PublisherQoS::Standard,
                true,
            )
            .await
            .expect("publish over tls");

        let msg = tokio::time::timeout(RECV_TIMEOUT, subscription.rx.recv_async())
            .await
            .expect("timed out waiting for tls message")
            .expect("tls subscription channel closed");
        assert_eq!(msg.payload(), &Bytes::from_static(b"tls-hello"));

        drop(_router); // stop zenohd
    }

    /// Negative path: a client that does NOT trust the router's CA (no
    /// `root_ca_certificate`, so zenoh falls back to system WebPKI roots, which
    /// do not include the private `minica` CA) cannot establish a usable link —
    /// it must receive nothing, while a properly-trusting publisher's message
    /// flows. Proves cert validation is actually enforced (not bypassed).
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn tls_client_with_untrusted_ca_receives_nothing() {
        const TOPIC: &str = "tls_untrusted";
        let _lock = ZENOH_SERIAL.lock().await;
        let certs = write_certs();
        let (_router, port) = start_tls_router(&certs).await;

        // Untrusted: no CA provided → WebPKI roots → the minica server cert is
        // not trusted → the TLS link to the router cannot be validated.
        let untrusted = TlsConfig {
            verify_name_on_connect: false,
            ..TlsConfig::default()
        };

        // Confirm the router's TLS listener is actually up *first* (a trusting
        // client opens, retrying while it settles, then is dropped). Otherwise a
        // `start_session` error below could be the router still starting rather
        // than the untrusted CA being rejected — making a readiness failure look
        // like the intended negative result. Same retry pattern as the trusted
        // and plaintext opens in this file.
        drop(open_tls_client(port, &trusting_client_tls(&certs)).await);

        let mut subscriber =
            ZenohAdapter::connect_to_tls("127.0.0.1", port, untrusted).expect("build adapter");
        // In client mode `zenoh::open` succeeds even if the link can't be
        // validated (the failure is async — no data ever flows). The router is
        // now known to be up, so an error here can only be the untrusted CA — a
        // strictly stronger negative result.
        if subscriber.start_session().await.is_err() {
            return;
        }
        let subscription = subscriber
            .subscribe_topic(&receiver(TOPIC), SubscriberQoS::Standard)
            .await
            .expect("subscribe (local declare succeeds even with a dead link)");

        // A correctly-trusting publisher proves traffic *is* flowing on the
        // router — so a no-delivery result is the untrusted link's fault, not a
        // dead test.
        let mut publisher = open_tls_client(port, &trusting_client_tls(&certs)).await;
        wait_for_subscriber_discovery().await;
        publisher
            .publish_topic(
                &sender(TOPIC),
                Payload::from_bytes(Bytes::from_static(b"should-not-arrive")),
                PublisherQoS::Standard,
                true,
            )
            .await
            .expect("publish over trusted tls");

        let delivered = tokio::time::timeout(Duration::from_secs(3), subscription.rx.recv_async())
            .await
            .is_ok();
        assert!(
            !delivered,
            "an untrusted-CA subscriber must not receive any message"
        );

        drop(_router);
    }

    /// The per-user-router topology end-to-end: a *local* router (plaintext for
    /// its own nodes) federated over `tls/` to a *remote* router. A subscriber on
    /// the LOCAL router receives a message a publisher sends into the REMOTE
    /// router — proving the two zenohd routers join one network (messages cross
    /// transparently) and that only the inter-router hop is TLS-encrypted. This is
    /// the exact shape the peppy daemon establishes against a per-user cloud
    /// router; a plain client→router connection (the prior design) could not
    /// bridge the local router's nodes to the remote network.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn federated_routers_relay_across_the_tls_link() {
        const TOPIC: &str = "federation_round_trip";
        let _lock = ZENOH_SERIAL.lock().await;
        let certs = write_certs();

        let (_remote, remote_port) = start_tls_router(&certs).await;
        let (_local, local_port) = start_federated_router(&certs, remote_port).await;
        // Give the local router a moment to dial and federate with the remote
        // (the inter-router session establishes asynchronously after spawn).
        tokio::time::sleep(Duration::from_secs(2)).await;

        // Subscriber attaches to the LOCAL router over plaintext loopback.
        let subscriber = open_plaintext_client(local_port).await;
        let subscription = subscriber
            .subscribe_topic(&receiver(TOPIC), SubscriberQoS::Standard)
            .await
            .expect("subscribe on the local router");

        // Publisher attaches to the REMOTE router over TLS.
        let mut publisher = open_tls_client(remote_port, &trusting_client_tls(&certs)).await;
        // The subscription must propagate local-router → (tls federation) →
        // remote-router before the publish; a single discovery wait is too short
        // for the cross-router hop, so allow a couple of rounds.
        wait_for_subscriber_discovery().await;
        wait_for_subscriber_discovery().await;
        publisher
            .publish_topic(
                &sender(TOPIC),
                Payload::from_bytes(Bytes::from_static(b"across-the-federation")),
                PublisherQoS::Standard,
                true,
            )
            .await
            .expect("publish on the remote router");

        let msg = tokio::time::timeout(RECV_TIMEOUT, subscription.rx.recv_async())
            .await
            .expect("timed out waiting for a message across the federation")
            .expect("subscription channel closed");
        assert_eq!(msg.payload(), &Bytes::from_static(b"across-the-federation"));

        drop(_local);
        drop(_remote);
    }
}
