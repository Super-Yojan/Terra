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
            max_bytes: limit("TERRA_OBSERVABILITY_EVENT_BYTES", 10 * 1024 * 1024 * 1024)?,
            retention_seconds: limit("TERRA_OBSERVABILITY_RETENTION_SECONDS", 30 * 86400)?,
            export_max_bytes: limit("TERRA_OBSERVABILITY_EXPORT_BYTES", 2 * 1024 * 1024 * 1024)?,
            attachment_max_bytes: limit(
                "TERRA_OBSERVABILITY_ATTACHMENT_BYTES",
                5 * 1024 * 1024 * 1024,
            )?,
        })?,
        credentials,
    );
    let maintenance = app.store.clone();
    std::thread::Builder::new()
        .name("observation-retention".into())
        .spawn(move || {
            loop {
                let cutoff = terra_observability::wall_time_ns().saturating_sub(
                    maintenance
                        .config
                        .retention_seconds
                        .saturating_mul(1_000_000_000),
                );
                if maintenance
                    .prune(cutoff)
                    .and_then(|_| maintenance.prune_attachments())
                    .is_err()
                {
                    eprintln!("Observability retention maintenance failed; inspect storage");
                }
                std::thread::sleep(std::time::Duration::from_secs(300));
            }
        })?;
    if std::env::var("TERRA_OBSERVABILITY_FOLLOW_JOURNAL").as_deref() == Ok("1") {
        let cursor = app.store.config.path.with_extension("cursor");
        terra_observability_collector::journal_source::follow_router_journal(
            app.store.clone(),
            cursor,
        )?;
    }
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

fn limit(name: &str, default: u64) -> Result<u64, Box<dyn std::error::Error + Send + Sync>> {
    let value = match std::env::var(name) {
        Ok(v) => v.parse::<u64>()?,
        Err(std::env::VarError::NotPresent) => default,
        Err(e) => return Err(e.into()),
    };
    if value == 0 {
        return Err("zero observability storage limit".into());
    }
    Ok(value)
}
