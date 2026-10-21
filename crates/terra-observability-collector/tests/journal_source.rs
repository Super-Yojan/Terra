use serde_json::json;
use terra_observability_collector::journal_source::project_journal;
#[test]
fn router_cursor_replay_has_stable_identity() {
    let entry = json!({"_SYSTEMD_UNIT":"argos-router.service","__CURSOR":"cursor-1","_BOOT_ID":"boot","__REALTIME_TIMESTAMP":"1000000","__MONOTONIC_TIMESTAMP":"500000","MESSAGE":"router connected"});
    let a = project_journal(&entry, "router").unwrap();
    let b = project_journal(&entry, "router").unwrap();
    assert_eq!(a.id, b.id);
    assert_eq!(a.input.wall_time_ns, 1_000_000_000);
    assert_eq!(a.input.monotonic_ns, 500_000_000);
}
#[test]
fn unrelated_units_are_not_collected() {
    let entry = json!({"_SYSTEMD_UNIT":"ssh.service","__CURSOR":"cursor","MESSAGE":"private"});
    assert!(project_journal(&entry, "router").is_none());
}

#[cfg(unix)]
#[test]
fn failed_cursor_is_retried_and_ended_stream_reconnects() {
    use std::{
        os::unix::fs::PermissionsExt,
        sync::{
            Arc,
            atomic::{AtomicBool, Ordering},
        },
        time::{Duration, Instant},
    };
    use terra_observability_collector::{Scope, Store, StoreConfig};
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("db");
    let store = Store::open(StoreConfig {
        path: path.clone(),
        ..Default::default()
    })
    .unwrap();
    let cursor = d.path().join("cursor");
    std::fs::write(&cursor, "before").unwrap();
    let program = d.path().join("journal");
    let calls = d.path().join("calls");
    let a = json!({"_SYSTEMD_UNIT":"argos-router.service","__CURSOR":"A","__REALTIME_TIMESTAMP":"1000000","MESSAGE":"A"});
    let b = json!({"_SYSTEMD_UNIT":"argos-router.service","__CURSOR":"B","__REALTIME_TIMESTAMP":"1000001","MESSAGE":"B"});
    std::fs::write(&program,format!("#!/bin/sh\nprintf '%s\\n' \"$*\" >> '{}'\ncase \"$*\" in *--after-cursor=B*) exit 0;; esac\nprintf '%s\\n' '{}' '{}'\n",calls.display(),a,b)).unwrap();
    std::fs::set_permissions(&program, std::fs::Permissions::from_mode(0o755)).unwrap();
    let backup = d.path().join("saved");
    std::fs::rename(&path, &backup).unwrap();
    std::fs::create_dir(&path).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let handle = terra_observability_collector::journal_source::follow_router_journal_program(
        store.clone(),
        cursor.clone(),
        program,
        stop.clone(),
        Duration::from_millis(20),
    )
    .unwrap();
    let deadline = Instant::now() + Duration::from_secs(2);
    while !calls.exists() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    std::thread::sleep(Duration::from_millis(60));
    assert_eq!(std::fs::read_to_string(&cursor).unwrap(), "before");
    std::fs::remove_dir(&path).unwrap();
    std::fs::rename(backup, &path).unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while std::fs::read_to_string(&cursor).unwrap_or_default() != "B" && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    stop.store(true, Ordering::Release);
    handle.join().unwrap();
    assert_eq!(std::fs::read_to_string(&cursor).unwrap(), "B");
    let rows = store
        .events(
            &Scope {
                producer_id: "gcp-router".into(),
                rovers: vec!["_producer".into()],
                read: true,
            },
            None,
            0,
            100,
        )
        .unwrap();
    for text in ["A", "B"] {
        assert!(
            rows.iter()
                .any(|e| e.event.input.attributes["message"] == text)
        );
    }
    assert!(std::fs::read_to_string(calls).unwrap().lines().count() > 1);
}
