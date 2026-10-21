use terra_observability_collector::{App, Credential, Store, StoreConfig, router};
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let credentials: Vec<Credential> = serde_json::from_slice(&std::fs::read(std::env::var(
        "TERRA_OBSERVABILITY_CREDENTIALS",
    )?)?)?;
    let path = std::env::var("TERRA_OBSERVABILITY_DB").unwrap_or_else(|_| "observations.db".into());
    let app = App::new(
        Store::open(StoreConfig {
            path: path.into(),
            ..Default::default()
        })?,
        credentials,
    );
    let listener = tokio::net::TcpListener::bind(
        std::env::var("TERRA_OBSERVABILITY_BIND").unwrap_or_else(|_| "127.0.0.1:7450".into()),
    )
    .await?;
    axum::serve(listener, router(app))
        .with_graceful_shutdown(async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await?;
    Ok(())
}
