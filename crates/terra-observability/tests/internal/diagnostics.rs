// Included as unit tests so the implementation can stay private.
#[cfg(test)]
mod tests {
    use crate::diagnostics::Diagnostics;
    use serde_json::json;
    #[test]
    fn rotation_keeps_only_two_bounded_private_segments() {
        let dir = tempfile::tempdir().unwrap();
        let db = dir.path().join("events.db");
        let mut log = Diagnostics::new(&db, "journal");
        for n in 0..3000 {
            log.record(
                "test",
                json!({"n":n,"password":"not-recorded","padding":"x".repeat(1000)}),
            );
        }
        let current = db.with_extension("journal-diagnostics.jsonl");
        for path in [&current, &current.with_extension("jsonl.1")] {
            assert!(std::fs::metadata(path).unwrap().len() <= 1024 * 1024);
            let text = std::fs::read_to_string(path).unwrap();
            assert!(!text.contains("not-recorded"));
            assert!(
                text.lines()
                    .all(|line| serde_json::from_str::<serde_json::Value>(line).is_ok())
            );
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                assert_eq!(
                    std::fs::metadata(path).unwrap().permissions().mode() & 0o777,
                    0o600
                );
            }
        }
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 2);
    }
    #[test]
    fn http_rejection_keeps_status_without_server_body_or_url() {
        let error = crate::diagnostics::HttpStatusError(429);
        let fields = crate::diagnostics::error_fields(&error);
        assert_eq!(fields["error_class"], "http_status");
        assert_eq!(fields["http_status"], 429);
    }
}
