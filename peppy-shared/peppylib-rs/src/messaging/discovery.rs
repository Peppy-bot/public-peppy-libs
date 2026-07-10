use super::{MessengerHandle, PROBE_TIMEOUT};
use crate::error::{Error, Result};
use crate::messaging::ProducerRef;
use crate::types::Payload;
use pmi::{ServiceQueryKind, ServiceWireSender};
use tokio::time::{Duration, Instant};

/// Resolves a service or action probe selector to a single concrete
/// producer [`ProducerRef`] before the real request is dispatched. Only
/// non-pinned call scopes reach this path — a fully pinned target carries
/// its `(core_node, instance_id)` and never discovers. The selector is
/// either a genuine wildcard (`ServiceTarget::Any` / `CoreNode`) or one of
/// [`discover_producer_among`]'s per-producer pinned probes.
///
/// Sends a probe (empty payload, `ServiceQueryKind::Probe` on the
/// attachment) to `probe_sender`; the producer-side transport adapter
/// auto-responds to probes in its query dispatch — even while user code
/// holds the producer's request loop — so the discovery is side-effect-free
/// and starvation-free even when multiple producers match the wildcard.
///
/// The first responder wins. Subsequent producer replies are ignored — they
/// drop on the floor when `poll_service` returns. The Zenoh keyexpr embedded
/// in the reply yields the responder's identity via the existing
/// `TopicMessage` parser.
///
/// **Why this exists**: `QueryTarget::All` delivers a wildcard query to every
/// matching producer; without discovery, every producer would also execute
/// the user handler. For services that's wasted work; for actions it is a
/// real-world safety hazard (e.g. two manipulators both executing the same
/// goal). Discovery pins the consumer to one producer so only that
/// producer's handler runs.
pub(super) async fn discover_producer(
    messenger: &MessengerHandle,
    probe_sender: &ServiceWireSender,
    discovery_timeout: Duration,
) -> Result<ProducerRef> {
    // `poll_service` itself retries on a peer-mode cold-start miss (the probe's
    // `QueryTarget::All` finalizing with no reply before discovery has settled),
    // bounded by `discovery_timeout`, so a single probe call here waits for a
    // producer to appear rather than failing the instant it runs ahead of
    // discovery.
    let response = messenger
        .poll_service(
            probe_sender,
            Payload::new(),
            ServiceQueryKind::Probe,
            discovery_timeout,
        )
        .await?;
    Ok(ProducerRef::new(
        response.core_node(),
        response.instance_id(),
    ))
}

/// Restricted discovery for a `from_any` slot bound to two or more
/// producers ([`ServiceTarget::OneOf`](super::ServiceTarget::OneOf)): one
/// fully-pinned probe per bound producer, raced with `select_ok` — the
/// first producer to answer wins the pin and the real call is delivered
/// pinned to it. Because every probe is pinned, a producer outside the
/// bound set can never win, no matter what else conforms on the wire.
///
/// All probes failing means no bound producer answered within
/// `discovery_timeout` → [`Error::ServiceUnreachable`] (no instance_id:
/// there is no single producer to blame).
pub(super) async fn discover_producer_among(
    messenger: &MessengerHandle,
    probe_senders: &[ServiceWireSender],
    discovery_timeout: Duration,
    service_name: &str,
) -> Result<ProducerRef> {
    // Callers guarantee a non-empty set (an empty bound set never leaves
    // the filter layer); handle it gracefully anyway — `select_ok` panics
    // on an empty iterator.
    if probe_senders.is_empty() {
        return Err(Error::ServiceUnreachable {
            instance_id: None,
            service_name: service_name.to_string(),
        });
    }
    let probes: Vec<_> = probe_senders
        .iter()
        .map(|sender| Box::pin(discover_producer(messenger, sender, discovery_timeout)))
        .collect();
    match futures::future::select_ok(probes).await {
        // Losers' in-flight probes are dropped here; late replies fall on
        // the floor exactly like wildcard discovery's non-winning replies.
        Ok((producer, _losers)) => Ok(producer),
        Err(_) => Err(Error::ServiceUnreachable {
            instance_id: None,
            service_name: service_name.to_string(),
        }),
    }
}

/// [`probe_reachable`] over a bound producer set: race one pinned probe
/// per producer and report `true` as soon as any answers. `false` only
/// when every bound producer is unreachable; a hard (non-reachability)
/// error surfaces only if no producer answered `true`.
pub(super) async fn probe_any_reachable(
    messenger: &MessengerHandle,
    senders: &[ServiceWireSender],
) -> Result<bool> {
    let mut probes: Vec<_> = senders
        .iter()
        .map(|sender| Box::pin(probe_reachable(messenger, sender)))
        .collect();
    let mut first_error = None;
    while !probes.is_empty() {
        let (result, _index, rest) = futures::future::select_all(probes).await;
        match result {
            Ok(true) => return Ok(true),
            Ok(false) => {}
            Err(err) => {
                if first_error.is_none() {
                    first_error = Some(err);
                }
            }
        }
        probes = rest;
    }
    match first_error {
        Some(err) => Err(err),
        None => Ok(false),
    }
}

/// [`probe_round_trip`] over a bound producer set: race one pinned sized
/// probe per producer and return the first successful sample — the
/// round-trip of whichever bound producer answers first, i.e. the one
/// restricted discovery would pin. If every probe fails, the last error
/// propagates (there is no usable latency sample). An empty set reports
/// [`Error::ServiceUnreachable`], exactly like [`discover_producer_among`].
pub(super) async fn probe_fastest_round_trip(
    messenger: &MessengerHandle,
    senders: &[ServiceWireSender],
    service_name: &str,
    request_size: usize,
    response_size: u32,
    response_timeout: Duration,
) -> Result<(Duration, usize)> {
    // Callers guarantee a non-empty set (an empty bound set never leaves
    // the filter layer); handle it gracefully anyway — `select_ok` panics
    // on an empty iterator.
    if senders.is_empty() {
        return Err(Error::ServiceUnreachable {
            instance_id: None,
            service_name: service_name.to_string(),
        });
    }
    let probes: Vec<_> = senders
        .iter()
        .map(|sender| {
            Box::pin(probe_round_trip(
                messenger,
                sender,
                request_size,
                response_size,
                response_timeout,
            ))
        })
        .collect();
    let (sample, _losers) = futures::future::select_ok(probes).await?;
    Ok(sample)
}

/// Probe `sender` once and classify reachability. The probe is auto-answered by
/// the producer-side transport, so the user handler never runs. A successful
/// reply or a [`Error::ServiceTimeout`] (producer reached but slow) both count
/// as reachable; only [`Error::ServiceUnreachable`] is `false`. Shared by
/// [`ServiceMessenger::is_reachable`](super::ServiceMessenger::is_reachable) and
/// [`ActionMessenger::is_reachable`](super::ActionMessenger::is_reachable),
/// which probes the action's goal service.
pub(super) async fn probe_reachable(
    messenger: &MessengerHandle,
    sender: &ServiceWireSender,
) -> Result<bool> {
    match messenger
        .poll_service(
            sender,
            Payload::new(),
            ServiceQueryKind::Probe,
            PROBE_TIMEOUT,
        )
        .await
    {
        Ok(_) => Ok(true),
        Err(Error::ServiceUnreachable { .. }) => Ok(false),
        Err(Error::ServiceTimeout { .. }) => Ok(true),
        Err(e) => Err(e),
    }
}

/// Measure the round-trip latency of a single sized `Probe` to `sender`. Like
/// [`probe_reachable`], the probe is auto-answered and the user handler never
/// runs, so the sample is a single-clock round-trip of the messaging/routing
/// path only. `request_size`/`response_size` make the probe carry a
/// real-payload-sized body and ask the producer to reply with `response_size`
/// bytes. Returns `(elapsed, response_bytes_received)`. Shared by the service
/// and action `probe_latency` methods.
pub(super) async fn probe_round_trip(
    messenger: &MessengerHandle,
    sender: &ServiceWireSender,
    request_size: usize,
    response_size: u32,
    response_timeout: Duration,
) -> Result<(Duration, usize)> {
    let request = Payload::from(pmi::build_sized_probe_request(request_size, response_size));
    let started = Instant::now();
    let reply = messenger
        .poll_service(sender, request, ServiceQueryKind::Probe, response_timeout)
        .await?;
    Ok((started.elapsed(), reply.payload_bytes().len()))
}
