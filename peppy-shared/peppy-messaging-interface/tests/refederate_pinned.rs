//! Operator-pinned `ZENOH_CONFIG`: `refederate` must be a no-op and leave the
//! pinned config untouched, so the daemon skips a pointless zenohd restart that
//! could apply nothing.
//!
//! This is an integration test (its own binary) on purpose: it pins a router by
//! setting the process-global `ZENOH_CONFIG`, and the lib's config-render tests
//! read that same var. Isolated here, this test is the only reader/writer of it
//! in its process, so there is no race — do not add other `ZENOH_CONFIG`-sensitive
//! tests to this file. It exercises the real public path end to end (a hand-pinned
//! config → `with_router` adopts it → `refederate`), rather than poking internal
//! state.

#![cfg(feature = "router")]

use pmi::{
    RouterLinks, SubscriberBufferSizes, TlsConfig, ZenohAdapter, ZenohNetProtocol,
    render_router_config,
};

#[test]
fn refederate_is_a_no_op_under_an_operator_pinned_config() {
    let port = 59250;

    // An operator hand-writes a router config and points `ZENOH_CONFIG` at it.
    // `render_router_config` produces exactly the shape zenohd (and the facade's
    // config parser) expect, so this stands in for a real operator-owned file.
    let pinned_config = render_router_config(
        ZenohNetProtocol::Tcp,
        "127.0.0.1",
        port,
        false,
        RouterLinks::default(),
    );
    let cfg_path = std::env::temp_dir().join(format!("peppy_pinned_router_{port}.json5"));
    std::fs::write(&cfg_path, &pinned_config).expect("write the operator-pinned config");

    // SAFETY: this test is the only code in its binary that touches `ZENOH_CONFIG`
    // (the lib's render tests run in a different process), so nothing reads or
    // writes it concurrently.
    unsafe {
        std::env::set_var("ZENOH_CONFIG", &cfg_path);
    }

    // Started the proper way: `with_router` resolves the config via `ZENOH_CONFIG`,
    // adopts the pinned file verbatim, and the facade records that it is pinned.
    let mut adapter = ZenohAdapter::with_router(
        ZenohNetProtocol::Tcp,
        "127.0.0.1",
        port,
        false,
        SubscriberBufferSizes::default(),
        RouterLinks::default(),
    )
    .expect("build a router adapter from the operator-pinned config");
    assert!(
        adapter.router_config_is_pinned(),
        "the adapter must expose the ownership captured from ZENOH_CONFIG"
    );

    let before = std::fs::read_to_string(&cfg_path).expect("read the pinned config");

    let rewrote = adapter
        .refederate(RouterLinks {
            connect_endpoints: vec!["tls/cap.zenoh.localhost:7443".to_string()],
            tls: Some(TlsConfig::client(std::path::PathBuf::from("/certs/ca.pem"))),
            ..RouterLinks::default()
        })
        .expect("refederate under a pinned config succeeds as a no-op");

    assert!(
        !rewrote,
        "a pinned config must report no rewrite so the caller skips the restart"
    );
    let after = std::fs::read_to_string(&cfg_path).expect("read the config after refederate");
    assert_eq!(
        before, after,
        "the operator-pinned config must be left untouched"
    );

    let _ = std::fs::remove_file(&cfg_path);
}
