use axum::{
    body::Body,
    http::{Request, StatusCode},
};
use sha2::{Digest, Sha256};
use terra_observability_collector::{App, Credential, Scope, Store, StoreConfig, router};
use tower::ServiceExt;
#[tokio::test]
async fn reader_and_writer_tokens_are_separate() {
    let d = tempfile::tempdir().unwrap();
    let s = Store::open(StoreConfig {
        path: d.path().join("db"),
        ..Default::default()
    })
    .unwrap();
    let credentials = vec![Credential {
        token_sha256: format!("{:x}", Sha256::digest(b"writer")),
        scope: Scope {
            producer_id: "phone".into(),
            rovers: vec!["4".into()],
            read: false,
        },
    }];
    let r = router(App::new(s, credentials));
    for token in [None, Some("Bearer writer")] {
        let mut req = Request::builder().uri("/v1/sessions");
        if let Some(t) = token {
            req = req.header("authorization", t);
        }
        let res = r
            .clone()
            .oneshot(req.body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(res.status(), StatusCode::UNAUTHORIZED);
    }
}
#[tokio::test]
async fn malformed_request_never_acknowledges() {
    let d = tempfile::tempdir().unwrap();
    let s = Store::open(StoreConfig {
        path: d.path().join("db"),
        ..Default::default()
    })
    .unwrap();
    let r = router(App::new(s, vec![]));
    let res = r
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/v1/events")
                .header("content-type", "application/json")
                .body(Body::from("{}"))
                .unwrap(),
        )
        .await
        .unwrap();
    assert!(!res.status().is_success());
}
#[tokio::test]
async fn exports_require_reader_authentication() {
    let d = tempfile::tempdir().unwrap();
    let store = Store::open(StoreConfig {
        path: d.path().join("db"),
        ..Default::default()
    })
    .unwrap();
    let r = router(App::new(store, vec![]));
    let res = r
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/v1/exports")
                .header("content-type", "application/json")
                .body(Body::from(r#"{"rover_id":"4","session_id":"run"}"#))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::UNAUTHORIZED);
}
#[tokio::test]
async fn clock_probe_requires_source_auth_and_returns_server_time() {
    let d = tempfile::tempdir().unwrap();
    let store = Store::open(StoreConfig {
        path: d.path().join("db"),
        ..Default::default()
    })
    .unwrap();
    let credentials = vec![Credential {
        token_sha256: format!("{:x}", Sha256::digest(b"writer")),
        scope: Scope {
            producer_id: "phone".into(),
            rovers: vec!["4".into()],
            read: false,
        },
    }];
    let r = router(App::new(store, credentials));
    let res = r
        .oneshot(
            Request::builder()
                .uri("/v1/clock")
                .header("authorization", "Bearer writer")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::OK);
    use http_body_util::BodyExt;
    let body = res.into_body().collect().await.unwrap().to_bytes();
    let value: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert!(value["server_wall_ns"].as_u64().unwrap() > 0);
}

#[tokio::test]
async fn transient_attachment_storage_failure_is_retryable() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("db");
    let store = Store::open(StoreConfig {
        path: path.clone(),
        ..Default::default()
    })
    .unwrap();
    let credentials = vec![Credential {
        token_sha256: format!("{:x}", Sha256::digest(b"writer")),
        scope: Scope {
            producer_id: "phone".into(),
            rovers: vec!["4".into()],
            read: false,
        },
    }];
    let app = router(App::new(store, credentials));
    let bytes = b"camera";
    let hash = format!("{:x}", Sha256::digest(bytes));
    std::fs::write(path.with_extension("attachments"), b"unavailable mount").unwrap();
    let request = || {
        Request::builder()
            .method("PUT")
            .uri(format!("/v1/attachments/{hash}"))
            .header("authorization", "Bearer writer")
            .header("x-rover-id", "4")
            .body(Body::from(bytes.as_slice()))
            .unwrap()
    };
    assert_eq!(
        app.clone().oneshot(request()).await.unwrap().status(),
        StatusCode::SERVICE_UNAVAILABLE
    );
    std::fs::remove_file(path.with_extension("attachments")).unwrap();
    assert_eq!(
        app.oneshot(request()).await.unwrap().status(),
        StatusCode::NO_CONTENT
    );
}
