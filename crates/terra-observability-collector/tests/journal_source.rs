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
