use terra_observability_collector::exports::enforce_export_budget;
#[test]
fn export_history_reclaims_bytes_before_reserving_new_work() {
    let d = tempfile::tempdir().unwrap();
    for name in ["a", "b"] {
        let dir = d.path().join(name);
        std::fs::create_dir(&dir).unwrap();
        std::fs::write(dir.join("source.bin"), vec![0; 80]).unwrap();
    }
    enforce_export_budget(d.path(), 100, 60, None).unwrap();
    assert_eq!(std::fs::read_dir(d.path()).unwrap().count(), 0);
    assert!(enforce_export_budget(d.path(), 100, 101, None).is_err());
    let current = d.path().join("current");
    std::fs::create_dir(&current).unwrap();
    std::fs::write(current.join("rrd"), vec![0; 120]).unwrap();
    assert!(enforce_export_budget(d.path(), 100, 0, Some(&current)).is_err());
    assert!(
        current.exists(),
        "active export must be failed explicitly, not evicted behind its worker"
    );
}
