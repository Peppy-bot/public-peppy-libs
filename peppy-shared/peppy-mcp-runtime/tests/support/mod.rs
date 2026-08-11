//! Shared fixture and client helpers for the HTTP integration tests.
//!
//! Each integration test binary compiles this module separately and uses a
//! subset of it, so unused-item warnings are suppressed for the module.
#![allow(dead_code)]

use peppy_mcp_catalog::ExposureBundle;
use peppy_mcp_runtime::{ActionContext, ActionExit, Clock, ExposureServer, MCP_HTTP_PATH};
use rmcp::model::{ClientCapabilities, ClientInfo, ProtocolVersion, UpdateTaskParams};
use rmcp::service::ServiceError;
use rmcp::transport::StreamableHttpClientTransport;
use rmcp::transport::streamable_http_client::StreamableHttpClientTransportConfig;
use rmcp::{ClientLifecycleMode, ClientServiceExt};
use serde_json::{Value, json};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

pub const STATUS_URI: &str = "peppy://resource/front_camera.status";
pub const FRAME_URI: &str = "peppy://resource/front_camera.latest_frame";

/// Guard for waits that are already response-driven; generous on purpose so
/// it only fires when something is genuinely broken.
pub const GUARD: Duration = Duration::from_secs(30);

pub type Client = rmcp::service::RunningService<rmcp::RoleClient, ClientInfo>;

pub fn loopback_bundle() -> ExposureBundle {
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
      { "name": "rgb_camera", "tag": "v1", "sha256": "aa", "link_id": "front_camera" },
      { "name": "episode_recording", "tag": "v1", "sha256": "bb", "link_id": "recorder" }
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
  "tasks": [
    {
      "name": "recorder.record_episode",
      "description": "Record one teleoperation episode to the local dataset.",
      "target": "recorder",
      "member": "record_episode",
      "operation": "long_running",
      "safety_sensitive": true,
      "confirmation_required": true,
      "deadline_ms": 600000,
      "input_schema": {
        "type": "object",
        "properties": { "episode_name": { "type": "string" } },
        "required": ["episode_name"],
        "additionalProperties": false
      },
      "output_schema": {
        "type": "object",
        "properties": { "frames": { "type": "integer" } },
        "required": ["frames"],
        "additionalProperties": false
      },
      "feedback_schema": {
        "type": "object",
        "properties": { "frame": { "type": "integer" } },
        "required": ["frame"],
        "additionalProperties": false
      }
    }
  ]
}"#,
    )
    .expect("loopback bundle parses")
}

pub struct Endpoint {
    pub url: String,
    pub server: ExposureServer,
    pub nanos: Arc<AtomicU64>,
    pub shutdown: tokio_util::sync::CancellationToken,
}

/// Serves the full loopback bundle: two tool handlers and the
/// `recorder.record_episode` task handler on a manual clock.
pub async fn start_endpoint() -> Endpoint {
    let (clock, nanos) = manual_clock();
    let server = with_camera_tools(ExposureServer::builder(loopback_bundle()).with_clock(clock))
        .with_task(
            "recorder.record_episode",
            |input: Value, context: ActionContext| async move {
                let episode = input["episode_name"]
                    .as_str()
                    .expect("validated string")
                    .to_string();
                context.report_feedback(format!("recording `{episode}`"));
                if episode == "wait_for_cancel" {
                    context.cancel_requested().await;
                    return Err(ActionExit::Cancelled);
                }
                Ok(json!({ "frames": 120 }))
            },
        )
        .build()
        .expect("bundle and handlers agree");
    serve(server, nanos).await
}

/// Serves the loopback bundle stripped of its tasks, so the endpoint does
/// not advertise the tasks extension.
pub async fn start_task_less_endpoint() -> Endpoint {
    let mut bundle = loopback_bundle();
    bundle.tasks.clear();
    let (clock, nanos) = manual_clock();
    let server = with_camera_tools(ExposureServer::builder(bundle).with_clock(clock))
        .build()
        .expect("bundle and handlers agree");
    serve(server, nanos).await
}

fn manual_clock() -> (Clock, Arc<AtomicU64>) {
    let nanos = Arc::new(AtomicU64::new(0));
    let source = Arc::clone(&nanos);
    (
        Clock::from_nanos_fn(move || source.load(Ordering::SeqCst)),
        nanos,
    )
}

fn with_camera_tools(
    builder: peppy_mcp_runtime::ExposureServerBuilder,
) -> peppy_mcp_runtime::ExposureServerBuilder {
    builder
        .with_tool("front_camera.info", |_input: Value| async move {
            Ok(json!({ "width": 640, "height": 480 }))
        })
        .with_tool("front_camera.set_brightness", |input: Value| async move {
            let value = input["value"].as_i64().expect("validated integer");
            if value == 13 {
                return Err(peppy_mcp_runtime::ToolCallError::Failed(
                    "13 is reserved".to_string(),
                ));
            }
            Ok(json!({ "applied": true }))
        })
}

async fn serve(server: ExposureServer, nanos: Arc<AtomicU64>) -> Endpoint {
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

pub async fn connect(url: &str) -> Client {
    connect_as(url, ClientCapabilities::default()).await
}

/// Connects a client that declares the SEP-2663 tasks extension capability;
/// in discover mode the SDK attaches it to every request's `_meta`.
pub async fn connect_with_tasks(url: &str) -> Client {
    connect_as(url, ClientCapabilities::builder().enable_tasks().build()).await
}

async fn connect_as(url: &str, capabilities: ClientCapabilities) -> Client {
    let transport = StreamableHttpClientTransport::from_config(
        StreamableHttpClientTransportConfig::with_uri(url.to_string()),
    );
    let mut info = ClientInfo::default();
    info.capabilities = capabilities;
    info.serve_with_lifecycle(
        transport,
        ClientLifecycleMode::Discover {
            preferred_versions: vec![ProtocolVersion::V_2026_07_28],
        },
    )
    .await
    .expect("client negotiates 2026-07-28 over loopback")
}

pub fn protocol_error(error: ServiceError) -> rmcp::ErrorData {
    match error {
        ServiceError::McpError(data) => data,
        other => panic!("expected a protocol error, got {other:?}"),
    }
}

pub fn confirmation_accept(task_id: &str) -> UpdateTaskParams {
    UpdateTaskParams::new(
        task_id,
        [("confirmation".to_string(), json!({ "action": "accept" }))]
            .into_iter()
            .collect(),
    )
}

/// A tiny valid rgb8 frame in the shape the JPEG representation expects.
pub fn sample_rgb8_frame() -> Value {
    let pixels: Vec<u8> = (0..8u32 * 8 * 3).map(|index| (index % 251) as u8).collect();
    let pixels_b64 = base64::Engine::encode(&base64::engine::general_purpose::STANDARD, &pixels);
    json!({
        "frame": pixels_b64,
        "encoding": "rgb8",
        "width": 8,
        "height": 8,
    })
}
