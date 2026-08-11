//! The MCP server: a catalog-driven
//! [`ServerHandler`](rmcp::ServerHandler) serving one exposure bundle over
//! Streamable HTTP under MCP `2026-07-28`.

use crate::clock::Clock;
use crate::error::{BuildError, ToolCallError};
use crate::state::{CatalogEvent, ReadRefusal, ResourceIngest, ResourceState};
use peppy_mcp_catalog::{
    BundleIdentity, BundleServer, ExposureBundle, ServiceOperation, ToolEntry,
};
use rmcp::model::{
    CacheScope, CallToolRequestParams, CallToolResponse, CallToolResult, ContentBlock,
    DiscoverResult, Implementation, JsonObject, ListResourcesResult, ListToolsResult,
    PaginatedRequestParams, ProtocolVersion, ReadResourceRequestParams, ReadResourceResponse,
    ReadResourceResult, Resource, ResourceContents, ServerCapabilities, ServerInfo,
    SubscriptionFilter, Tool, ToolAnnotations,
};
use rmcp::service::{RequestContext, SubscriptionContext, SubscriptionSendError};
use rmcp::transport::streamable_http_server::session::local::LocalSessionManager;
use rmcp::transport::streamable_http_server::{StreamableHttpServerConfig, StreamableHttpService};
use rmcp::{ErrorData as McpError, RoleServer, ServerHandler};
use serde_json::{Value, json};
use std::borrow::Cow;
use std::collections::{HashMap, HashSet};
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::broadcast;

/// The path the Streamable HTTP endpoint is mounted under.
pub const MCP_HTTP_PATH: &str = "/mcp";

/// `ttlMs` for the catalog-shaped results: discovery, `tools/list`, and
/// `resources/list`. The catalog is fixed for the life of the server (a
/// changed exposure regenerates the node), so clients may cache it for as
/// long as they keep the connection.
const CATALOG_TTL_MS: u64 = 3_600_000;

/// Capacity of the resource-updated event channel; a listener lagging this
/// far behind skips to the newest events, which for latest-snapshot
/// semantics loses nothing that a fresh read would not recover.
const EVENT_CHANNEL_CAPACITY: usize = 256;

/// One registered bridge: validated canonical-JSON input in, canonical-JSON
/// output or a [`ToolCallError`] out. Any `Fn(Value) -> impl Future` with
/// those shapes implements it.
pub trait ToolHandler: Send + Sync + 'static {
    fn call(
        &self,
        input: Value,
    ) -> Pin<Box<dyn Future<Output = Result<Value, ToolCallError>> + Send>>;
}

impl<F, Fut> ToolHandler for F
where
    F: Fn(Value) -> Fut + Send + Sync + 'static,
    Fut: Future<Output = Result<Value, ToolCallError>> + Send + 'static,
{
    fn call(
        &self,
        input: Value,
    ) -> Pin<Box<dyn Future<Output = Result<Value, ToolCallError>> + Send>> {
        Box::pin(self(input))
    }
}

struct ToolState {
    entry: ToolEntry,
    /// Compiled from the bundle's derived input schema; every call is
    /// validated before it can reach the Peppy graph.
    validator: jsonschema::Validator,
    handler: Arc<dyn ToolHandler>,
}

struct ServerState {
    server: BundleServer,
    exposure: BundleIdentity,
    node_name: String,
    resources_by_uri: HashMap<String, Arc<ResourceState>>,
    resource_uri_by_name: HashMap<String, String>,
    resource_list: Vec<Resource>,
    tools: HashMap<String, Arc<ToolState>>,
    tool_list: Vec<Tool>,
    events: broadcast::Sender<CatalogEvent>,
    clock: Clock,
}

/// Builds an [`ExposureServer`] from a parsed bundle and one registered
/// handler per exposed tool.
pub struct ExposureServerBuilder {
    bundle: ExposureBundle,
    clock: Clock,
    handlers: HashMap<String, Arc<dyn ToolHandler>>,
}

impl ExposureServerBuilder {
    /// Injects the time source for freshness and rate gating. Defaults to
    /// the wall clock; a generated node passes its node clock so sim time
    /// governs policies too.
    pub fn with_clock(mut self, clock: Clock) -> Self {
        self.clock = clock;
        self
    }

    /// Registers the bridge behind one tool entry of the bundle.
    pub fn with_tool(mut self, name: impl Into<String>, handler: impl ToolHandler) -> Self {
        self.handlers.insert(name.into(), Arc::new(handler));
        self
    }

    /// Checks the bundle and the registered handlers against each other and
    /// prepares the served catalog.
    pub fn build(self) -> Result<ExposureServer, BuildError> {
        let Self {
            bundle,
            clock,
            mut handlers,
        } = self;
        if !bundle.tasks.is_empty() {
            return Err(BuildError::TasksUnsupported {
                count: bundle.tasks.len(),
            });
        }

        let mut names = HashSet::new();
        let mut resources_by_uri = HashMap::new();
        let mut resource_uri_by_name = HashMap::new();
        let mut resource_list = Vec::new();
        for entry in &bundle.resources {
            if !names.insert(entry.name.clone()) {
                return Err(BuildError::DuplicateName {
                    name: entry.name.clone(),
                });
            }
            resource_list.push(
                Resource::new(entry.uri.clone(), entry.name.clone())
                    .with_description(entry.description.clone())
                    .with_mime_type("application/json"),
            );
            resource_uri_by_name.insert(entry.name.clone(), entry.uri.clone());
            if resources_by_uri
                .insert(
                    entry.uri.clone(),
                    Arc::new(ResourceState::new(entry.clone())),
                )
                .is_some()
            {
                return Err(BuildError::DuplicateName {
                    name: entry.uri.clone(),
                });
            }
        }

        let mut tools = HashMap::new();
        let mut tool_list = Vec::new();
        for entry in &bundle.tools {
            if !names.insert(entry.name.clone()) {
                return Err(BuildError::DuplicateName {
                    name: entry.name.clone(),
                });
            }
            let handler =
                handlers
                    .remove(&entry.name)
                    .ok_or_else(|| BuildError::MissingToolHandler {
                        name: entry.name.clone(),
                    })?;
            let validator = jsonschema::validator_for(&entry.input_schema).map_err(|error| {
                BuildError::InvalidInputSchema {
                    name: entry.name.clone(),
                    error: error.to_string(),
                }
            })?;
            let Value::Object(input_schema) = entry.input_schema.clone() else {
                return Err(BuildError::InvalidInputSchema {
                    name: entry.name.clone(),
                    error: "the input schema root is not an object".to_string(),
                });
            };
            let mut tool = Tool::new(
                entry.name.clone(),
                entry.description.clone(),
                Arc::new(input_schema),
            )
            .with_annotations(annotations_for(entry.operation));
            if let Value::Object(output_schema) = entry.output_schema.clone() {
                tool = tool.with_raw_output_schema(Arc::new(output_schema));
            }
            tool_list.push(tool);
            tools.insert(
                entry.name.clone(),
                Arc::new(ToolState {
                    entry: entry.clone(),
                    validator,
                    handler,
                }),
            );
        }
        if let Some(name) = handlers.into_keys().next() {
            return Err(BuildError::UnknownToolHandler { name });
        }

        let (events, _) = broadcast::channel(EVENT_CHANNEL_CAPACITY);
        Ok(ExposureServer {
            state: Arc::new(ServerState {
                server: bundle.server,
                exposure: bundle.exposure,
                node_name: bundle.node.name,
                resources_by_uri,
                resource_uri_by_name,
                resource_list,
                tools,
                tool_list,
                events,
                clock,
            }),
        })
    }
}

fn annotations_for(operation: ServiceOperation) -> ToolAnnotations {
    match operation {
        ServiceOperation::ReadOnly => ToolAnnotations::default()
            .read_only(true)
            .destructive(false),
        ServiceOperation::Mutating => ToolAnnotations::default().read_only(false),
    }
}

/// The MCP server for one exposure bundle. Cheap to clone; all clones share
/// the same snapshots, gates, and subscriptions.
#[derive(Clone)]
pub struct ExposureServer {
    state: Arc<ServerState>,
}

impl std::fmt::Debug for ExposureServer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ExposureServer")
            .field("exposure", &self.state.exposure.name)
            .field("node", &self.state.node_name)
            .finish_non_exhaustive()
    }
}

impl ExposureServer {
    pub fn builder(bundle: ExposureBundle) -> ExposureServerBuilder {
        ExposureServerBuilder {
            bundle,
            clock: Clock::wall(),
            handlers: HashMap::new(),
        }
    }

    /// The ingest feeding the named resource, or `None` when the bundle
    /// exposes no such resource.
    pub fn ingest(&self, resource_name: &str) -> Option<ResourceIngest> {
        let uri = self.state.resource_uri_by_name.get(resource_name)?;
        Some(ResourceIngest {
            state: Arc::clone(self.state.resources_by_uri.get(uri)?),
            events: self.state.events.clone(),
            clock: self.state.clock.clone(),
        })
    }

    /// Serves the exposure on the listener until the token cancels. The
    /// listener decides the address; bind it to `127.0.0.1` (the design's
    /// trust boundary is the machine).
    pub async fn serve(
        self,
        listener: tokio::net::TcpListener,
        shutdown: tokio_util::sync::CancellationToken,
    ) -> std::io::Result<()> {
        let config = StreamableHttpServerConfig::default()
            .with_legacy_session_mode(false)
            .with_cancellation_token(shutdown.child_token());
        let service: StreamableHttpService<Self, LocalSessionManager> =
            StreamableHttpService::new(move || Ok(self.clone()), Default::default(), config);
        let router = axum::Router::new().nest_service(MCP_HTTP_PATH, service);
        axum::serve(listener, router)
            .with_graceful_shutdown(shutdown.cancelled_owned())
            .await
    }

    fn capabilities() -> ServerCapabilities {
        ServerCapabilities::builder()
            .enable_resources()
            .enable_resources_subscribe()
            .enable_resources_list_changed()
            .enable_tools()
            .enable_tool_list_changed()
            .build()
    }

    fn read_snapshot(&self, uri: &str) -> Result<ReadResourceResult, McpError> {
        let Some(resource) = self.state.resources_by_uri.get(uri) else {
            return Err(McpError::resource_not_found(
                format!("`{uri}` is not a resource of this exposure"),
                Some(json!({ "uri": uri })),
            ));
        };
        match resource.snapshot_for_read(self.state.clock.now_nanos()) {
            Ok(view) => Ok(ReadResourceResult::new(vec![
                ResourceContents::text(view.serialized, uri).with_mime_type("application/json"),
            ])
            .with_ttl_ms(view.remaining_fresh_ms)
            .with_cache_scope(CacheScope::Private)),
            Err(ReadRefusal::Unavailable) => Err(McpError::internal_error(
                format!(
                    "resource `{uri}` is unavailable: nothing has been published since the \
                     server started"
                ),
                None,
            )),
            Err(ReadRefusal::Stale { age_ms, max_age_ms }) => Err(McpError::internal_error(
                format!(
                    "resource `{uri}` is stale: the snapshot is {age_ms} ms old and \
                     `max_age_ms` is {max_age_ms}"
                ),
                None,
            )),
        }
    }

    async fn execute_tool(
        &self,
        name: &str,
        arguments: JsonObject,
    ) -> Result<CallToolResult, McpError> {
        let Some(tool) = self.state.tools.get(name) else {
            return Err(McpError::invalid_params(
                format!("`{name}` is not a tool of this exposure"),
                None,
            ));
        };
        let input = Value::Object(arguments);
        let problems: Vec<String> = tool
            .validator
            .iter_errors(&input)
            .map(|error| {
                let path = error.instance_path().to_string();
                if path.is_empty() {
                    error.to_string()
                } else {
                    format!("{path}: {error}")
                }
            })
            .collect();
        if !problems.is_empty() {
            return Err(McpError::invalid_params(
                format!("invalid arguments for `{name}`: {}", problems.join("; ")),
                None,
            ));
        }

        let deadline = Duration::from_millis(tool.entry.deadline_ms.get());
        let result = match tokio::time::timeout(deadline, tool.handler.call(input)).await {
            Err(_elapsed) => {
                return Ok(tool_error(format!(
                    "deadline exceeded: the provider did not answer within {} ms",
                    tool.entry.deadline_ms
                )));
            }
            Ok(Err(error)) => return Ok(tool_error(error.to_string())),
            Ok(Ok(value)) => value,
        };

        if let Some(limit) = tool.entry.max_result_bytes {
            let size = serde_json::to_string(&result)
                .expect("JSON value serializes")
                .len() as u64;
            if size > limit.get() {
                return Ok(tool_error(format!(
                    "result of {size} bytes exceeds the {} byte limit",
                    limit.get()
                )));
            }
        }
        Ok(CallToolResult::structured(result))
    }
}

fn tool_error(message: String) -> CallToolResult {
    CallToolResult::error(vec![ContentBlock::text(message)])
}

impl ServerHandler for ExposureServer {
    fn supported_protocol_versions(&self) -> Cow<'static, [ProtocolVersion]> {
        Cow::Borrowed(&[ProtocolVersion::V_2026_07_28])
    }

    fn get_info(&self) -> ServerInfo {
        let implementation = Implementation::new(
            self.state.node_name.clone(),
            self.state.exposure.tag.clone(),
        )
        .with_title(self.state.server.title.clone());
        let mut info = ServerInfo::new(Self::capabilities())
            .with_server_info(implementation)
            .with_protocol_version(ProtocolVersion::V_2026_07_28);
        if let Some(instructions) = &self.state.server.instructions {
            info = info.with_instructions(instructions.clone());
        }
        info
    }

    async fn discover(
        &self,
        _context: RequestContext<RoleServer>,
    ) -> Result<DiscoverResult, McpError> {
        Ok(DiscoverResult::from_server_info(
            self.supported_protocol_versions().into_owned(),
            self.get_info(),
        )
        .with_ttl_ms(CATALOG_TTL_MS)
        .with_cache_scope(CacheScope::Private))
    }

    async fn list_resources(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> Result<ListResourcesResult, McpError> {
        Ok(
            ListResourcesResult::with_all_items(self.state.resource_list.clone())
                .with_ttl_ms(CATALOG_TTL_MS)
                .with_cache_scope(CacheScope::Private),
        )
    }

    async fn read_resource(
        &self,
        request: ReadResourceRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<ReadResourceResponse, McpError> {
        self.read_snapshot(&request.uri).map(Into::into)
    }

    async fn list_tools(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> Result<ListToolsResult, McpError> {
        Ok(
            ListToolsResult::with_all_items(self.state.tool_list.clone())
                .with_ttl_ms(CATALOG_TTL_MS)
                .with_cache_scope(CacheScope::Private),
        )
    }

    fn get_tool(&self, name: &str) -> Option<Tool> {
        self.state
            .tool_list
            .iter()
            .find(|tool| tool.name.as_ref() == name)
            .cloned()
    }

    async fn call_tool(
        &self,
        request: CallToolRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<CallToolResponse, McpError> {
        self.execute_tool(request.name.as_ref(), request.arguments.unwrap_or_default())
            .await
            .map(Into::into)
    }

    fn accepted_subscription_filter(
        &self,
        requested: &SubscriptionFilter,
    ) -> Option<SubscriptionFilter> {
        Some(requested.supported_by(&Self::capabilities()))
    }

    async fn listen(&self, context: SubscriptionContext) -> Result<(), McpError> {
        let sink = context.sink().clone();
        let mut events = self.state.events.subscribe();
        loop {
            tokio::select! {
                _ = context.cancelled() => return Ok(()),
                event = events.recv() => match event {
                    Ok(CatalogEvent::ResourceUpdated { uri }) => {
                        match sink.notify_resource_updated(uri).await {
                            Ok(()) => {}
                            Err(
                                SubscriptionSendError::SubscriptionClosed
                                | SubscriptionSendError::Service(_),
                            ) => return Ok(()),
                            // The subscription's accepted filter does not
                            // cover this notification; other listeners may
                            // still want it.
                            Err(_) => {}
                        }
                    }
                    // Latest-snapshot semantics: a lagged listener re-reads
                    // and loses nothing.
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => return Ok(()),
                },
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::test_support::manual_clock;
    use rmcp::model::ErrorCode;
    use std::sync::atomic::Ordering;

    fn test_bundle() -> ExposureBundle {
        ExposureBundle::from_json_str(
            r#"{
  "bundle_format": 1,
  "schema_mapping_version": 1,
  "exposure": { "name": "camera_and_recording", "tag": "v1" },
  "server": {
    "title": "OpenArm camera",
    "instructions": "Observe the front camera."
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
      "schema": { "type": "object" }
    }
  ],
  "tools": [
    {
      "name": "front_camera.set_brightness",
      "description": "Set the camera brightness in device units.",
      "target": "front_camera",
      "member": "set_brightness",
      "operation": "mutating",
      "deadline_ms": 2000,
      "max_result_bytes": 64,
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
        .expect("test bundle parses")
    }

    fn brightness_handler(
        input: Value,
    ) -> impl Future<Output = Result<Value, ToolCallError>> + Send {
        async move {
            let value = input["value"].as_i64().expect("validated integer");
            Ok(json!({ "applied": value >= 0 }))
        }
    }

    fn built_server() -> ExposureServer {
        ExposureServer::builder(test_bundle())
            .with_tool("front_camera.set_brightness", brightness_handler)
            .build()
            .expect("bundle and handlers agree")
    }

    fn arguments(raw: Value) -> JsonObject {
        match raw {
            Value::Object(map) => map,
            other => panic!("arguments must be an object, got {other}"),
        }
    }

    #[test]
    fn a_bundle_with_tasks_is_refused() {
        let mut bundle = test_bundle();
        bundle.tasks = vec![
            serde_json::from_value(json!({
                "name": "recorder.record_episode",
                "description": "Record one episode.",
                "target": "recorder",
                "member": "record_episode",
                "operation": "long_running",
                "safety_sensitive": true,
                "confirmation_required": true,
                "deadline_ms": 900000,
                "input_schema": { "type": "object" },
                "output_schema": { "type": "object" }
            }))
            .expect("valid task entry"),
        ];
        let error = ExposureServer::builder(bundle)
            .with_tool("front_camera.set_brightness", brightness_handler)
            .build()
            .expect_err("tasks are not supported yet");
        assert_eq!(error, BuildError::TasksUnsupported { count: 1 });
    }

    #[test]
    fn a_tool_without_a_handler_is_refused() {
        let error = ExposureServer::builder(test_bundle())
            .build()
            .expect_err("the brightness tool has no handler");
        assert_eq!(
            error,
            BuildError::MissingToolHandler {
                name: "front_camera.set_brightness".to_string()
            }
        );
    }

    #[test]
    fn a_handler_without_a_tool_is_refused() {
        let error = ExposureServer::builder(test_bundle())
            .with_tool("front_camera.set_brightness", brightness_handler)
            .with_tool("front_camera.set_gain", brightness_handler)
            .build()
            .expect_err("set_gain is not in the bundle");
        assert_eq!(
            error,
            BuildError::UnknownToolHandler {
                name: "front_camera.set_gain".to_string()
            }
        );
    }

    #[test]
    fn the_catalog_carries_descriptions_schemas_and_annotations() {
        let server = built_server();
        let resource = &server.state.resource_list[0];
        assert_eq!(resource.uri, "peppy://resource/front_camera.status");
        assert_eq!(resource.name, "front_camera.status");
        assert_eq!(resource.mime_type.as_deref(), Some("application/json"));

        let tool = &server.state.tool_list[0];
        assert_eq!(tool.name.as_ref(), "front_camera.set_brightness");
        assert!(tool.output_schema.is_some());
        let annotations = tool.annotations.as_ref().expect("annotations set");
        assert_eq!(annotations.read_only_hint, Some(false));
        let read_only = annotations_for(ServiceOperation::ReadOnly);
        assert_eq!(read_only.read_only_hint, Some(true));
        assert_eq!(read_only.destructive_hint, Some(false));
    }

    #[tokio::test]
    async fn calling_an_unknown_tool_is_a_protocol_error() {
        let error = built_server()
            .execute_tool("front_camera.set_gain", JsonObject::new())
            .await
            .expect_err("set_gain is not exposed");
        assert_eq!(error.code, ErrorCode::INVALID_PARAMS);
        assert!(error.message.contains("not a tool of this exposure"));
    }

    #[tokio::test]
    async fn arguments_failing_the_derived_schema_never_reach_the_handler() {
        let server = built_server();
        for (raw, expected_fragment) in [
            (json!({ "value": 65 }), "value"),
            (json!({ "value": "bright" }), "value"),
            (json!({}), "value"),
            (json!({ "value": 1, "extra": true }), "extra"),
        ] {
            let error = server
                .execute_tool("front_camera.set_brightness", arguments(raw.clone()))
                .await
                .expect_err("invalid arguments are rejected");
            assert_eq!(error.code, ErrorCode::INVALID_PARAMS, "for {raw}");
            assert!(
                error.message.contains(expected_fragment),
                "error for {raw} should mention `{expected_fragment}`: {}",
                error.message
            );
        }
    }

    #[tokio::test]
    async fn a_valid_call_returns_structured_output() {
        let result = built_server()
            .execute_tool(
                "front_camera.set_brightness",
                arguments(json!({ "value": 12 })),
            )
            .await
            .expect("valid call");
        assert_ne!(result.is_error, Some(true));
        assert_eq!(result.structured_content, Some(json!({ "applied": true })));
    }

    #[tokio::test]
    async fn a_bridge_failure_is_a_readable_tool_error() {
        let server = ExposureServer::builder(test_bundle())
            .with_tool("front_camera.set_brightness", |_input: Value| async {
                Err(ToolCallError::Unavailable("no producer bound".to_string()))
            })
            .build()
            .expect("builds");
        let result = server
            .execute_tool(
                "front_camera.set_brightness",
                arguments(json!({ "value": 1 })),
            )
            .await
            .expect("tool errors are results, not protocol errors");
        assert_eq!(result.is_error, Some(true));
        let rendered = serde_json::to_string(&result.content).expect("content serializes");
        assert!(
            rendered.contains("provider unavailable: no producer bound"),
            "got {rendered}"
        );
    }

    #[tokio::test]
    async fn an_oversize_result_is_a_tool_error() {
        let server = ExposureServer::builder(test_bundle())
            .with_tool("front_camera.set_brightness", |_input: Value| async {
                Ok(json!({ "applied": "y".repeat(128) }))
            })
            .build()
            .expect("builds");
        let result = server
            .execute_tool(
                "front_camera.set_brightness",
                arguments(json!({ "value": 1 })),
            )
            .await
            .expect("oversize is a tool error");
        assert_eq!(result.is_error, Some(true));
        let rendered = serde_json::to_string(&result.content).expect("content serializes");
        assert!(
            rendered.contains("exceeds the 64 byte limit"),
            "got {rendered}"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn a_handler_slower_than_the_deadline_is_a_tool_error() {
        let server = ExposureServer::builder(test_bundle())
            .with_tool("front_camera.set_brightness", |_input: Value| async {
                tokio::time::sleep(Duration::from_secs(3600)).await;
                Ok(json!({ "applied": true }))
            })
            .build()
            .expect("builds");
        let result = server
            .execute_tool(
                "front_camera.set_brightness",
                arguments(json!({ "value": 1 })),
            )
            .await
            .expect("deadline is a tool error");
        assert_eq!(result.is_error, Some(true));
        let rendered = serde_json::to_string(&result.content).expect("content serializes");
        assert!(
            rendered.contains("did not answer within 2000 ms"),
            "got {rendered}"
        );
    }

    #[test]
    fn reads_walk_unavailable_fresh_and_stale_with_freshness_as_ttl() {
        let (clock, nanos) = manual_clock();
        let server = ExposureServer::builder(test_bundle())
            .with_clock(clock)
            .with_tool("front_camera.set_brightness", brightness_handler)
            .build()
            .expect("builds");
        let uri = "peppy://resource/front_camera.status";

        let error = server
            .read_snapshot(uri)
            .expect_err("nothing published yet");
        assert!(error.message.contains("unavailable"));

        let ingest = server
            .ingest("front_camera.status")
            .expect("resource exists");
        let token = ingest.admit().expect("gate open");
        ingest
            .publish(token, json!({ "battery": 87 }))
            .expect("publishes");

        nanos.store(500 * 1_000_000, Ordering::SeqCst);
        let read = server.read_snapshot(uri).expect("fresh snapshot serves");
        assert_eq!(read.ttl_ms, Some(1500), "ttl is the remaining freshness");
        assert_eq!(read.cache_scope, Some(CacheScope::Private));

        nanos.store(2_500 * 1_000_000, Ordering::SeqCst);
        let error = server.read_snapshot(uri).expect_err("2500 ms old is stale");
        assert!(error.message.contains("stale"), "got {}", error.message);
        assert!(
            error.message.contains("2500 ms old"),
            "got {}",
            error.message
        );
    }

    #[test]
    fn reading_an_unknown_uri_is_resource_not_found() {
        let error = built_server()
            .read_snapshot("peppy://resource/absent")
            .expect_err("absent resources are refused");
        assert_eq!(error.code, ErrorCode::RESOURCE_NOT_FOUND);
    }

    #[test]
    fn the_ingest_lookup_uses_public_resource_names() {
        let server = built_server();
        assert!(server.ingest("front_camera.status").is_some());
        assert!(server.ingest("front_camera.absent").is_none());
    }

    #[test]
    fn server_info_advertises_the_exposure_identity_and_2026_07_28() {
        let info = built_server().get_info();
        assert_eq!(info.protocol_version, ProtocolVersion::V_2026_07_28);
        assert_eq!(info.server_info.name, "camera_and_recording_mcp");
        assert_eq!(info.server_info.version, "v1");
        assert_eq!(
            info.instructions.as_deref(),
            Some("Observe the front camera.")
        );
        let resources = info.capabilities.resources.expect("resources capability");
        assert_eq!(resources.subscribe, Some(true));
        assert_eq!(resources.list_changed, Some(true));
        assert!(info.capabilities.tools.is_some());
    }
}
