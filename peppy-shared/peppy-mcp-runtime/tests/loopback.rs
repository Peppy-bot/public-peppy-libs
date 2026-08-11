//! Loopback integration: a real Streamable HTTP endpoint on `127.0.0.1`
//! driven by the real rmcp client under MCP `2026-07-28`.
//!
//! The Peppy side is stubbed: tool handlers are closures and snapshots are
//! fed through the ingest directly, so this exercises the whole protocol
//! surface without a messaging mesh. Freshness runs on a manual clock the
//! test advances; every wait is on a response or a notification.

use peppy_mcp_catalog::ExposureBundle;
use peppy_mcp_runtime::{Clock, ExposureServer, MCP_HTTP_PATH, ToolCallError};
use rmcp::model::{
    CacheScope, CallToolRequestParams, ClientInfo, ErrorCode, ProtocolVersion,
    ReadResourceRequestParams, ServerNotification, SubscriptionFilter, object,
};
use rmcp::service::ServiceError;
use rmcp::transport::StreamableHttpClientTransport;
use rmcp::transport::streamable_http_client::StreamableHttpClientTransportConfig;
use rmcp::{ClientLifecycleMode, ClientServiceExt};
use serde_json::{Value, json};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

const STATUS_URI: &str = "peppy://resource/front_camera.status";
const FRAME_URI: &str = "peppy://resource/front_camera.latest_frame";

/// Guard for waits that are already response-driven; generous on purpose so
/// it only fires when something is genuinely broken.
const GUARD: Duration = Duration::from_secs(30);

fn loopback_bundle() -> ExposureBundle {
    ExposureBundle::from_json_str(
        r#"{
  "bundle_format": 1,
  "schema_mapping_version": 1,
  "exposure": { "name": "camera_and_recording", "tag": "v1" },
  "server": {
    "title": "OpenArm camera",
    "instructions": "Observe the front camera on this robot."
  },
  "node": {
    "name": "camera_and_recording_mcp",
    "tag": "v1",
    "contracts": [
      { "name": "rgb_camera", "tag": "v1", "sha256": "aa", "link_id": "front_camera" }
    ]
  },
  "resources": [
    {
      "name": "front_camera.status",
      "uri": "peppy://resource/front_camera.status",
      "description": "Latest camera status.",
      "target": "front_camera",
      "member": "camera_status",
      "policies": {
        "freshness": { "max_age_ms": 2000 },
        "update": { "max_hz": 2.0 }
      },
      "schema": {
        "type": "object",
        "properties": { "battery": { "type": "integer", "minimum": 0, "maximum": 255 } },
        "required": ["battery"],
        "additionalProperties": false
      }
    },
    {
      "name": "front_camera.latest_frame",
      "uri": "peppy://resource/front_camera.latest_frame",
      "description": "Latest frame from the front-facing camera, JPEG encoded.",
      "target": "front_camera",
      "member": "video_stream",
      "policies": {
        "freshness": { "max_age_ms": 2000 },
        "update": { "max_hz": 2.0 },
        "representation": {
          "image": "jpeg",
          "quality": 80,
          "fields": { "data": "frame", "encoding": "encoding", "width": "width", "height": "height" }
        },
        "max_result_bytes": 524288,
        "on_oversize": "downscale"
      },
      "schema": { "type": "object" }
    }
  ],
  "tools": [
    {
      "name": "front_camera.info",
      "description": "Report the camera's resolution, frame rate, and encoding.",
      "target": "front_camera",
      "member": "video_stream_info",
      "operation": "read_only",
      "deadline_ms": 2000,
      "input_schema": { "type": "object", "properties": {}, "additionalProperties": false },
      "output_schema": {
        "type": "object",
        "properties": {
          "width": { "type": "integer", "minimum": 0, "maximum": 65535 },
          "height": { "type": "integer", "minimum": 0, "maximum": 65535 }
        },
        "required": ["width", "height"],
        "additionalProperties": false
      }
    },
    {
      "name": "front_camera.set_brightness",
      "description": "Set the camera brightness in device units.",
      "target": "front_camera",
      "member": "set_brightness",
      "operation": "mutating",
      "deadline_ms": 2000,
      "input_schema": {
        "type": "object",
        "properties": {
          "value": { "type": "integer", "minimum": -64, "maximum": 64 }
        },
        "required": ["value"],
        "additionalProperties": false
      },
      "output_schema": {
        "type": "object",
        "properties": { "applied": { "type": "boolean" } },
        "required": ["applied"],
        "additionalProperties": false
      }
    }
  ],
  "tasks": []
}"#,
    )
    .expect("loopback bundle parses")
}

struct Endpoint {
    url: String,
    server: ExposureServer,
    nanos: Arc<AtomicU64>,
    shutdown: tokio_util::sync::CancellationToken,
}

async fn start_endpoint() -> Endpoint {
    let nanos = Arc::new(AtomicU64::new(0));
    let source = Arc::clone(&nanos);
    let clock = Clock::from_nanos_fn(move || source.load(Ordering::SeqCst));

    let server = ExposureServer::builder(loopback_bundle())
        .with_clock(clock)
        .with_tool("front_camera.info", |_input: Value| async move {
            Ok(json!({ "width": 640, "height": 480 }))
        })
        .with_tool("front_camera.set_brightness", |input: Value| async move {
            let value = input["value"].as_i64().expect("validated integer");
            if value == 13 {
                return Err(ToolCallError::Failed("13 is reserved".to_string()));
            }
            Ok(json!({ "applied": true }))
        })
        .build()
        .expect("bundle and handlers agree");

    let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0))
        .await
        .expect("an OS-assigned loopback port binds");
    let address = listener
        .local_addr()
        .expect("bound listener has an address");
    let shutdown = tokio_util::sync::CancellationToken::new();
    tokio::spawn(server.clone().serve(listener, shutdown.clone()));

    Endpoint {
        url: format!("http://{address}{MCP_HTTP_PATH}"),
        server,
        nanos,
        shutdown,
    }
}

async fn connect(
    url: &str,
) -> rmcp::service::RunningService<rmcp::RoleClient, rmcp::model::ClientInfo> {
    let transport = StreamableHttpClientTransport::from_config(
        StreamableHttpClientTransportConfig::with_uri(url.to_string()),
    );
    ClientInfo::default()
        .serve_with_lifecycle(
            transport,
            ClientLifecycleMode::Discover {
                preferred_versions: vec![ProtocolVersion::V_2026_07_28],
            },
        )
        .await
        .expect("client negotiates 2026-07-28 over loopback")
}

fn protocol_error(error: ServiceError) -> rmcp::ErrorData {
    match error {
        ServiceError::McpError(data) => data,
        other => panic!("expected a protocol error, got {other:?}"),
    }
}

const MS: u64 = 1_000_000;

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_real_client_walks_the_catalog_snapshots_and_tools() {
    let endpoint = start_endpoint().await;
    let client = connect(&endpoint.url).await;

    // The catalog matches the bundle and carries the caching hints.
    let tools = client.list_tools(None).await.expect("tools/list answers");
    let mut tool_names: Vec<_> = tools.tools.iter().map(|tool| tool.name.as_ref()).collect();
    tool_names.sort_unstable();
    assert_eq!(
        tool_names,
        ["front_camera.info", "front_camera.set_brightness"]
    );
    assert_eq!(tools.ttl_ms, Some(3_600_000));
    assert_eq!(tools.cache_scope, Some(CacheScope::Private));
    let info_tool = tools
        .tools
        .iter()
        .find(|tool| tool.name.as_ref() == "front_camera.info")
        .expect("info tool is listed");
    assert_eq!(
        info_tool
            .annotations
            .as_ref()
            .and_then(|a| a.read_only_hint),
        Some(true)
    );
    assert!(info_tool.output_schema.is_some());

    let resources = client
        .list_resources(None)
        .await
        .expect("resources/list answers");
    let mut resource_uris: Vec<_> = resources
        .resources
        .iter()
        .map(|resource| resource.uri.as_str())
        .collect();
    resource_uris.sort_unstable();
    assert_eq!(resource_uris, [FRAME_URI, STATUS_URI]);
    assert_eq!(resources.ttl_ms, Some(3_600_000));
    assert_eq!(resources.cache_scope, Some(CacheScope::Private));

    // Before any publish, reads report the resource unavailable.
    let error = protocol_error(
        client
            .read_resource(ReadResourceRequestParams::new(STATUS_URI))
            .await
            .expect_err("nothing published yet"),
    );
    assert!(
        error.message.contains("unavailable"),
        "got {}",
        error.message
    );

    // A resource outside the exposure is refused. Under 2026-07-28 the SDK
    // remaps resource-not-found onto invalid-params (SEP-2164).
    let error = protocol_error(
        client
            .read_resource(ReadResourceRequestParams::new("peppy://resource/absent"))
            .await
            .expect_err("absent resources are refused"),
    );
    assert_eq!(error.code, ErrorCode::INVALID_PARAMS);

    // Subscribe, then publish: the notification names the updated resource.
    let mut subscription = client
        .listen(
            SubscriptionFilter::builder()
                .resource_subscription(STATUS_URI)
                .build(),
        )
        .await
        .expect("subscriptions/listen is accepted");

    let ingest = endpoint
        .server
        .ingest("front_camera.status")
        .expect("resource exists");
    let token = ingest.admit().expect("gate open");
    ingest
        .publish(token, json!({ "battery": 87 }))
        .expect("publishes");

    let notification = tokio::time::timeout(GUARD, subscription.next())
        .await
        .expect("a notification arrives")
        .expect("the subscription stream is healthy")
        .expect("the stream did not end");
    match notification {
        ServerNotification::ResourceUpdatedNotification(updated) => {
            assert_eq!(updated.params.uri, STATUS_URI);
        }
        other => panic!("expected a resource-updated notification, got {other:?}"),
    }
    subscription.cancel().await.expect("subscription cancels");

    // The published snapshot serves with the remaining freshness as ttlMs.
    let read = client
        .read_resource(ReadResourceRequestParams::new(STATUS_URI))
        .await
        .expect("fresh snapshot serves");
    assert_eq!(read.ttl_ms, Some(2000));
    assert_eq!(read.cache_scope, Some(CacheScope::Private));
    let contents = read.contents.first().expect("one content item");
    let text = match contents {
        rmcp::model::ResourceContents::TextResourceContents {
            text,
            mime_type,
            uri,
            ..
        } => {
            assert_eq!(uri, STATUS_URI);
            assert_eq!(mime_type.as_deref(), Some("application/json"));
            text
        }
        other => panic!("expected text contents, got {other:?}"),
    };
    assert_eq!(text, "{\"battery\":87}");

    // An rgb8 frame published through the ingest serves as JPEG.
    let frame_ingest = endpoint
        .server
        .ingest("front_camera.latest_frame")
        .expect("resource exists");
    let pixels: Vec<u8> = (0..8u32 * 8 * 3).map(|index| (index % 251) as u8).collect();
    let pixels_b64 = base64::Engine::encode(&base64::engine::general_purpose::STANDARD, &pixels);
    let frame = json!({
        "frame": pixels_b64,
        "encoding": "rgb8",
        "width": 8,
        "height": 8,
    });
    let token = frame_ingest.admit().expect("gate open");
    frame_ingest.publish(token, frame).expect("frame publishes");
    let read = client
        .read_resource(ReadResourceRequestParams::new(FRAME_URI))
        .await
        .expect("frame snapshot serves");
    let rmcp::model::ResourceContents::TextResourceContents { text, .. } =
        read.contents.first().expect("one content item")
    else {
        panic!("expected text contents");
    };
    let snapshot: Value = serde_json::from_str(text).expect("snapshot is JSON");
    assert_eq!(snapshot["encoding"], "mjpeg");
    assert_eq!(snapshot["width"], 8);
    let jpeg = base64::Engine::decode(
        &base64::engine::general_purpose::STANDARD,
        snapshot["frame"].as_str().expect("frame is base64"),
    )
    .expect("frame decodes");
    assert_eq!(&jpeg[..2], &[0xFF, 0xD8], "JPEG magic bytes");

    // Tool calls round-trip structured output against the derived schemas.
    let called = client
        .call_tool(CallToolRequestParams::new("front_camera.info"))
        .await
        .expect("read-only tool answers");
    assert_ne!(called.is_error, Some(true));
    assert_eq!(
        called.structured_content,
        Some(json!({ "width": 640, "height": 480 }))
    );

    let called = client
        .call_tool(
            CallToolRequestParams::new("front_camera.set_brightness")
                .with_arguments(object(json!({ "value": 12 }))),
        )
        .await
        .expect("mutating tool answers");
    assert_eq!(called.structured_content, Some(json!({ "applied": true })));

    // A value outside the reflected restrict bounds is rejected before the
    // bridge runs.
    let error = protocol_error(
        client
            .call_tool(
                CallToolRequestParams::new("front_camera.set_brightness")
                    .with_arguments(object(json!({ "value": 65 }))),
            )
            .await
            .expect_err("65 is outside the restrict bounds"),
    );
    assert_eq!(error.code, ErrorCode::INVALID_PARAMS);

    // A name absent from the exposure is rejected.
    let error = protocol_error(
        client
            .call_tool(CallToolRequestParams::new("front_camera.set_gain"))
            .await
            .expect_err("set_gain is not exposed"),
    );
    assert_eq!(error.code, ErrorCode::INVALID_PARAMS);

    // A bridge failure comes back as a readable tool error, not a protocol
    // error.
    let called = client
        .call_tool(
            CallToolRequestParams::new("front_camera.set_brightness")
                .with_arguments(object(json!({ "value": 13 }))),
        )
        .await
        .expect("bridge failures are tool errors");
    assert_eq!(called.is_error, Some(true));

    client.cancel().await.expect("client disconnects");

    // Once the injected clock passes max_age_ms, a fresh connection reads
    // the snapshot as stale.
    endpoint.nanos.store(2_500 * MS, Ordering::SeqCst);
    let late_client = connect(&endpoint.url).await;
    let error = protocol_error(
        late_client
            .read_resource(ReadResourceRequestParams::new(STATUS_URI))
            .await
            .expect_err("2500 ms old is stale"),
    );
    assert!(error.message.contains("stale"), "got {}", error.message);
    late_client.cancel().await.expect("client disconnects");

    endpoint.shutdown.cancel();
}
