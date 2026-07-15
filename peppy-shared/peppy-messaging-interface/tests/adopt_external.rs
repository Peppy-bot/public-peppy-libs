#![cfg(feature = "build_zenoh")]

use pmi::{
    Messenger, MessengerAdapter, MessengerBackend, SubscriberBufferSizes, ZenohAdapter,
    ZenohNetProtocol,
};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;

fn unused_ephemeral_port() -> u16 {
    let listener = TcpListener::bind(("127.0.0.1", 0)).expect("reserve an ephemeral port");
    listener.local_addr().expect("read local address").port()
}

fn router_adapter(port: u16, external_zenohd: Option<PathBuf>) -> ZenohAdapter {
    ZenohAdapter::with_router(
        ZenohNetProtocol::Tcp,
        "127.0.0.1",
        port,
        false,
        SubscriberBufferSizes::default(),
        Vec::new(),
        None,
        external_zenohd,
    )
    .expect("build router adapter")
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn adopts_a_responsive_external_router_without_owning_it() {
    let router_a = ZenohAdapter::start_router_ephemeral("127.0.0.1", None)
        .await
        .expect("start operator-managed router A");
    let port = router_a.port;

    let adapter_b = router_adapter(port, Some(PathBuf::from("/any/zenohd")));
    let mut messenger_b = Messenger::new(MessengerAdapter::Zenoh(adapter_b));
    assert!(!messenger_b.router_is_adopted());

    messenger_b
        .start_router()
        .await
        .expect("adopt responsive router A");
    assert!(messenger_b.router_is_adopted());

    messenger_b
        .stop_router()
        .await
        .expect("stopping an adopted router is a no-op");
    assert!(messenger_b.router_is_adopted());
    drop(messenger_b);

    TcpStream::connect(("127.0.0.1", port))
        .expect("operator-managed router A remains available after B is dropped");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rejects_a_non_zenoh_process_holding_the_router_port() {
    let listener = TcpListener::bind(("127.0.0.1", 0)).expect("bind non-zenoh listener");
    let port = listener.local_addr().expect("read local address").port();
    let mut adapter = router_adapter(port, Some(PathBuf::from("/any/zenohd")));

    let error = adapter
        .start_router()
        .await
        .expect_err("a raw TCP listener is not a responsive zenoh router");
    assert!(
        error
            .to_string()
            .contains("not by a responsive zenoh router"),
        "unexpected error: {error}"
    );
    assert!(!adapter.router_is_adopted());

    drop(listener);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn fails_loud_when_the_configured_external_router_is_not_running() {
    let port = unused_ephemeral_port();
    let configured_path = PathBuf::from("/definitely/not/a/real/zenohd");
    let mut adapter = router_adapter(port, Some(configured_path.clone()));

    let error = adapter
        .start_router()
        .await
        .expect_err("external mode must not spawn a missing router");
    let message = error.to_string();
    assert!(
        message.contains("nothing is serving"),
        "unexpected error: {error}"
    );
    assert!(
        message.contains(configured_path.to_str().expect("UTF-8 test path")),
        "configured path is missing from error: {error}"
    );
    assert!(!adapter.router_is_adopted());

    let listener = TcpListener::bind(("127.0.0.1", port))
        .expect("peppy did not spawn any router on the free port");
    drop(listener);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn null_external_path_keeps_the_managed_port_busy_error() {
    let listener = TcpListener::bind(("127.0.0.1", 0)).expect("hold managed router port");
    let port = listener.local_addr().expect("read local address").port();
    let mut adapter = router_adapter(port, None);

    let error = adapter
        .start_router()
        .await
        .expect_err("managed mode must reject a busy port");
    assert!(
        error
            .to_string()
            .contains("Zenoh router port already in use"),
        "unexpected error: {error}"
    );
    assert!(!adapter.router_is_adopted());

    drop(listener);
}
