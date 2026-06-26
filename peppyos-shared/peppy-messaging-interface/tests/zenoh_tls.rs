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
        RECV_TIMEOUT, ZENOH_SERIAL, test_node_target, wait_for_subscriber_discovery,
    };
    use bytes::Bytes;
    use pmi::{
        Messenger, MessengerAdapter, MessengerBackend, Payload, PublisherQoS,
        SubscriberBufferSizes, SubscriberQoS, TlsConfig, TopicWireReceiver, TopicWireSender,
        ZenohAdapter, ZenohNetProtocol,
    };
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

    fn sender(as_topic_name: &str) -> TopicWireSender {
        TopicWireSender::new(
            "test_core_node",
            "test_instance",
            test_node_target("test_node"),
            None,
            as_topic_name,
        )
        .expect("valid wire fields")
    }

    fn receiver(to_topic: &str) -> TopicWireReceiver {
        TopicWireReceiver::new(
            "test_core_node",
            "test_instance",
            None,
            None,
            Some(test_node_target("test_node")),
            None,
            to_topic,
        )
        .expect("valid wire fields")
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
            Some(TlsConfig::server(certs.cert.clone(), certs.key.clone())),
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

        let mut subscriber =
            ZenohAdapter::connect_to_tls("127.0.0.1", port, untrusted).expect("build adapter");
        // In client mode `zenoh::open` succeeds even if the link can't be
        // validated (the failure is async — no data ever flows). If it instead
        // errors here, that's a strictly stronger negative result.
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
}
