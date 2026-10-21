use crate::{Credential, Scope, Store, authorize};
use axum::{
    Json, Router,
    extract::{DefaultBodyLimit, Query, State},
    http::{HeaderMap, StatusCode},
    routing::{get, post},
};
use serde::Deserialize;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use terra_observability::{EventEnvelope, wall_time_ns};
#[derive(Clone)]
pub struct App {
    pub store: Store,
    pub credentials: Arc<Vec<Credential>>,
    pub rates: Arc<Mutex<HashMap<String, (u64, u64)>>>,
}
impl App {
    pub fn new(store: Store, credentials: Vec<Credential>) -> Self {
        Self {
            store,
            credentials: Arc::new(credentials),
            rates: Default::default(),
        }
    }
}
pub fn router(app: App) -> Router {
    Router::new()
        .route("/v1/events", post(ingest).get(events))
        .route("/v1/sessions", get(sessions))
        .layer(DefaultBodyLimit::max(1024 * 1024))
        .with_state(app)
}
fn scope(app: &App, headers: &HeaderMap, read: bool) -> Result<Scope, StatusCode> {
    let token = headers
        .get("authorization")
        .and_then(|h| h.to_str().ok())
        .and_then(|v| v.strip_prefix("Bearer "))
        .ok_or(StatusCode::UNAUTHORIZED)?;
    authorize(token, &app.credentials, read).ok_or(StatusCode::UNAUTHORIZED)
}
#[derive(Deserialize)]
struct Batch {
    events: Vec<EventEnvelope>,
}
async fn ingest(
    State(app): State<App>,
    headers: HeaderMap,
    Json(batch): Json<Batch>,
) -> Result<Json<serde_json::Value>, StatusCode> {
    let scope = scope(&app, &headers, false)?;
    let now = wall_time_ns() / 1_000_000_000;
    {
        let mut rates = app
            .rates
            .lock()
            .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
        let global = rates.get("_all").filter(|v| v.0 == now).map_or(0, |v| v.1);
        let source = rates
            .get(&scope.producer_id)
            .filter(|v| v.0 == now)
            .map_or(0, |v| v.1);
        let count = batch.events.len() as u64;
        if global + count > 1000 || source + count > 200 {
            return Err(StatusCode::TOO_MANY_REQUESTS);
        }
        rates.insert("_all".into(), (now, global + count));
        rates.insert(scope.producer_id.clone(), (now, source + count));
    }
    tokio::task::spawn_blocking(move || app.store.ingest(&scope, &batch.events))
        .await
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
        .map(|ack| Json(serde_json::json!({"acknowledged":ack})))
        .map_err(|_| StatusCode::BAD_REQUEST)
}
#[derive(Deserialize)]
struct Filter {
    session: Option<String>,
    cursor: Option<i64>,
    limit: Option<usize>,
}
async fn events(
    State(app): State<App>,
    headers: HeaderMap,
    Query(q): Query<Filter>,
) -> Result<Json<serde_json::Value>, StatusCode> {
    let scope = scope(&app, &headers, true)?;
    tokio::task::spawn_blocking(move || {
        app.store.events(
            &scope,
            q.session.as_deref(),
            q.cursor.unwrap_or(0),
            q.limit.unwrap_or(256),
        )
    })
    .await
    .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
    .map(|e| Json(serde_json::json!({"events":e})))
    .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)
}
async fn sessions(
    State(app): State<App>,
    headers: HeaderMap,
) -> Result<Json<serde_json::Value>, StatusCode> {
    let scope = scope(&app, &headers, true)?;
    tokio::task::spawn_blocking(move || app.store.sessions(&scope))
        .await
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
        .map(Json)
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)
}
