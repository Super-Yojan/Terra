use sha2::{Digest, Sha256};
use terra_observability_collector::*;
fn scope(rover: &str) -> Scope {
    Scope {
        producer_id: "phone".into(),
        rovers: vec![rover.into()],
        read: false,
    }
}
#[test]
fn hash_and_rover_scope_are_enforced() {
    let d = tempfile::tempdir().unwrap();
    let s = Store::open(StoreConfig {
        path: d.path().join("db"),
        ..Default::default()
    })
    .unwrap();
    let data = b"image";
    let hash = format!("{:x}", Sha256::digest(data));
    s.put_attachment(&scope("4"), "4", &hash, data).unwrap();
    assert_eq!(s.attachment(&scope("4"), &hash).unwrap(), data);
    assert!(s.attachment(&scope("5"), &hash).is_err());
    assert!(
        s.put_attachment(&scope("4"), "4", &hash, b"corrupted")
            .is_err()
    );
}
#[test]
fn malformed_hash_cannot_escape_directory() {
    let d = tempfile::tempdir().unwrap();
    let s = Store::open(StoreConfig {
        path: d.path().join("db"),
        ..Default::default()
    })
    .unwrap();
    assert!(s.put_attachment(&scope("4"), "4", "../file", b"x").is_err());
}
#[test]
fn configured_attachment_quota_is_enforced() {
    let d = tempfile::tempdir().unwrap();
    let store = Store::open(StoreConfig {
        path: d.path().join("db"),
        attachment_max_bytes: 2,
        ..Default::default()
    })
    .unwrap();
    let bytes = b"image";
    let hash = format!("{:x}", Sha256::digest(bytes));
    assert!(
        store
            .put_attachment(&scope("4"), "4", &hash, bytes)
            .is_err()
    );
}

#[test]
fn concurrent_uploads_never_exceed_attachment_quota_or_share_temporary_files() {
    for _ in 0..4 {
        let d = tempfile::tempdir().unwrap();
        let store = Store::open(StoreConfig {
            path: d.path().join("db"),
            attachment_max_bytes: 512 * 1024,
            ..Default::default()
        })
        .unwrap();
        let barrier = std::sync::Arc::new(std::sync::Barrier::new(8));
        let handles = (0..8)
            .map(|n| {
                let s = store.clone();
                let b = barrier.clone();
                std::thread::spawn(move || {
                    let bytes = vec![n / 2; 256 * 1024];
                    let hash = format!("{:x}", Sha256::digest(&bytes));
                    b.wait();
                    s.put_attachment(&scope("4"), "4", &hash, &bytes).unwrap();
                })
            })
            .collect::<Vec<_>>();
        for h in handles {
            h.join().unwrap();
        }
        let bytes: u64 = std::fs::read_dir(store.config.path.with_extension("attachments"))
            .unwrap()
            .map(|e| e.unwrap().metadata().unwrap().len())
            .sum();
        assert!(
            bytes <= store.config.attachment_max_bytes,
            "concurrent writes exceeded quota: {bytes}"
        );
    }
}
