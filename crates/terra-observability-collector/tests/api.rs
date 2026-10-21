use axum::{body::Body,http::{Request,StatusCode}};
use terra_observability_collector::{App,Credential,Scope,Store,StoreConfig,router};
use tower::ServiceExt;
use sha2::{Digest,Sha256};
#[tokio::test]
async fn reader_and_writer_tokens_are_separate(){
 let d=tempfile::tempdir().unwrap();let s=Store::open(StoreConfig{path:d.path().join("db"),..Default::default()}).unwrap();
 let credentials=vec![Credential{token_sha256:format!("{:x}",Sha256::digest(b"writer")),scope:Scope{producer_id:"phone".into(),rovers:vec!["4".into()],read:false}}];
 let r=router(App::new(s,credentials));
 for token in [None,Some("Bearer writer")] {let mut req=Request::builder().uri("/v1/sessions");if let Some(t)=token{req=req.header("authorization",t);}let res=r.clone().oneshot(req.body(Body::empty()).unwrap()).await.unwrap();assert_eq!(res.status(),StatusCode::UNAUTHORIZED);}
}
#[tokio::test]
async fn malformed_request_never_acknowledges(){
 let d=tempfile::tempdir().unwrap();let s=Store::open(StoreConfig{path:d.path().join("db"),..Default::default()}).unwrap();
 let r=router(App::new(s,vec![]));
 let res=r.oneshot(Request::builder().method("POST").uri("/v1/events").header("content-type","application/json").body(Body::from("{}" )).unwrap()).await.unwrap();assert!(!res.status().is_success());
}
