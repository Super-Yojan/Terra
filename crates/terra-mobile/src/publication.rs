//! Content caching for visualization only. Never use this gate for safety or authority.
use std::time::{Duration, Instant};

/// Source and control timestamps share the phone monotonic clock. Do not turn
/// an old cached map into fresh visualization evidence after gate reset.
pub(crate) fn fresh_map_sample(now: f64, sample: f64) -> bool {
    now.is_finite()
        && sample.is_finite()
        && now >= 0.
        && sample >= 0.
        && now >= sample
        && now - sample < 0.5
}

/// Dashboard limits are intentionally narrower than onboard mapping.
pub(crate) fn map_visualization_valid(map: &terra_mapping::MapSnapshot, run: &str) -> bool {
    !run.is_empty()
        && run.len() <= 256
        && (1..=512).contains(&map.width)
        && (1..=512).contains(&map.height)
        && map.width as usize * map.height as usize <= 250_000
        && map.occupancy.len() == map.width as usize * map.height as usize
        && map.resolution.is_finite()
        && (0.01..=2.).contains(&map.resolution)
        && map.origin_x.is_finite()
        && map.origin_y.is_finite()
        && (map.origin_x + f64::from(map.width) * map.resolution).is_finite()
        && (map.origin_y + f64::from(map.height) * map.resolution).is_finite()
        && map.occupancy.iter().all(|cell| (-1..=100).contains(cell))
}

#[derive(Debug, PartialEq)]
/// Decision for a newer visualization source sample.
pub(crate) enum Publication {
    /// Publish full content and establish its source sequence as the baseline.
    Full,
    /// Publish freshness referencing the accepted full-baseline sequence.
    Heartbeat(u64),
    /// Publish nothing for this observation.
    Skip,
}

/// Content/identity cache with strict per-source sequence ordering and timed baseline recovery.
pub(crate) struct Gate<C, S> {
    content: Option<C>,
    source: Option<S>,
    full_at: Option<Instant>,
    sent_at: Option<Instant>,
    sequence: u64,
    observed_sequence: u64,
}

impl<C, S> Default for Gate<C, S> {
    fn default() -> Self {
        Self {
            content: None,
            source: None,
            full_at: None,
            sent_at: None,
            sequence: 0,
            observed_sequence: 0,
        }
    }
}

impl<C, S> Gate<C, S> {
    /// Confirm whether the full baseline entered transport; rejection clears baseline
    /// state while preserving the observed source-sequence high-water mark.
    pub(crate) fn full_enqueued(&mut self, accepted: bool) {
        if !accepted {
            // A rejected baseline cannot be referenced by a heartbeat. Retain
            // the observed high-water mark so cached ticks do not reserialize
            // the rejected sample; the next newer source must retry a full.
            self.content = None;
            self.full_at = None;
            self.sent_at = None;
            self.sequence = 0;
        }
    }
}

impl<C: PartialEq, S: PartialEq> Gate<C, S> {
    /// Return true for a changed source identity or strictly greater source sequence.
    pub(crate) fn is_newer(&self, source: &S, sequence: u64) -> bool {
        self.source.as_ref() != Some(source) || sequence > self.observed_sequence
    }

    /// Invalidate the baseline for a newer incompatible source without permitting replay
    /// of the same rejected sample. Older or duplicate samples have no effect.
    pub(crate) fn reject_source(&mut self, source: S, sequence: u64) {
        if self.is_newer(&source, sequence) {
            self.source = Some(source);
            self.observed_sequence = sequence;
            self.full_enqueued(false);
        }
    }

    /// Observe exact content with source identity, sequence, monotonic instant, and
    /// heartbeat interval. New identities/content or a two-second recovery deadline
    /// produce full content; unchanged newer samples can heartbeat. Duplicate/older
    /// sequences are skipped even if their content differs. `now` must not precede
    /// previous observations, as elapsed-time calculation uses `duration_since`.
    pub(crate) fn observe(
        &mut self,
        content: C,
        source: S,
        sequence: u64,
        now: Instant,
        interval: Duration,
    ) -> Publication {
        // Sequence is immutable within a run/frame, even if conflicting content
        // arrives. Only an explicit run/frame change resets source ordering.
        let same_identity = self.source.as_ref() == Some(&source);
        if !self.is_newer(&source, sequence) {
            return Publication::Skip;
        }
        self.source = Some(source);
        self.observed_sequence = sequence;
        if !same_identity
            || self.content.as_ref() != Some(&content)
            || self
                .full_at
                .is_none_or(|t| now.duration_since(t) >= Duration::from_secs(2))
        {
            self.content = Some(content);
            self.sequence = sequence;
            self.full_at = Some(now);
            self.sent_at = Some(now);
            Publication::Full
        } else if self
            .sent_at
            .is_none_or(|t| now.duration_since(t) >= interval)
        {
            self.sent_at = Some(now);
            Publication::Heartbeat(self.sequence)
        } else {
            Publication::Skip
        }
    }
}

/// Cache all occupancy cells and geometry under the run identity.
pub(crate) type MapGate = Gate<(String, u32, u32, f64, f64, f64, Vec<i8>), String>;
/// Cache exact parsed cloud content under its frame identity.
pub(crate) type CloudGate = Gate<serde_json::Value, String>;

/// Deterministic contract and regression scenarios for this module.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Exercises accepted dashboard dimensions and rejects excessive dimensions/cell count,
    /// invalid resolution/cells/origin, and empty or oversized run identities.
    fn visualization_envelope_matches_dashboard_without_narrowing_local_maps() {
        // Build unknown-free test geometry with the requested dimensions and valid 0.1 m cells.
        let map = |width, height| terra_mapping::MapSnapshot {
            width,
            height,
            resolution: 0.1,
            origin_x: 0.,
            origin_y: 0.,
            occupancy: vec![0; width as usize * height as usize],
        };
        for (width, height) in [(400, 400), (500, 500), (512, 1)] {
            assert!(map_visualization_valid(&map(width, height), "run"));
        }
        for (width, height) in [(600, 500), (512, 500), (513, 1), (1, 513)] {
            let unsupported = map(width, height);
            // All examples fit the wire cap; byte acceptance alone is insufficient.
            assert!(serde_json::to_vec(&unsupported.occupancy).unwrap().len() < 2_000_000);
            assert!(!map_visualization_valid(&unsupported, "run"));
        }
        for resolution in [0.009, 2.001, f64::NAN, f64::INFINITY] {
            let mut unsupported = map(2, 2);
            unsupported.resolution = resolution;
            assert!(!map_visualization_valid(&unsupported, "run"));
        }
        for cell in [-2, 101] {
            let mut unsupported = map(2, 2);
            unsupported.occupancy[0] = cell;
            assert!(!map_visualization_valid(&unsupported, "run"));
        }
        let mut unsupported = map(2, 2);
        unsupported.origin_x = f64::INFINITY;
        assert!(!map_visualization_valid(&unsupported, "run"));
        assert!(!map_visualization_valid(&map(2, 2), ""));
        assert!(!map_visualization_valid(&map(2, 2), &"x".repeat(257)));
        assert!(map_visualization_valid(&map(2, 2), &"x".repeat(256)));
    }

    #[test]
    /// Rejects a newer source sample, verifies it cannot be retried at the same sequence,
    /// and requires a new full baseline before subsequent heartbeats.
    fn incompatible_new_source_invalidates_baseline_but_not_source_ordering() {
        let now = Instant::now();
        let interval = Duration::from_millis(200);
        let mut gate = Gate::default();
        assert_eq!(
            gate.observe(vec![0], "run", 1, now, interval),
            Publication::Full
        );
        gate.reject_source("run", 2);
        assert!(!gate.is_newer(&"run", 2));
        assert_eq!(
            gate.observe(vec![0], "run", 2, now + interval, interval),
            Publication::Skip
        );
        assert_eq!(
            gate.observe(vec![0], "run", 3, now + interval, interval),
            Publication::Full
        );
        // An older invalid observation cannot invalidate the accepted baseline.
        gate.reject_source("run", 2);
        assert_eq!(
            gate.observe(vec![0], "run", 4, now + interval * 2, interval),
            Publication::Heartbeat(3)
        );
    }

    #[test]
    /// Checks older, duplicate, and conflicting samples are suppressed; newer samples
    /// can heartbeat or replace content, and a changed identity resets ordering.
    fn ordering_is_strict_within_identity_and_resets_only_for_new_identity() {
        let now = Instant::now();
        let interval = Duration::from_millis(200);
        let mut gate = Gate::default();
        assert_eq!(
            gate.observe(vec![0], "run-a", 10, now, interval),
            Publication::Full
        );
        assert_eq!(
            gate.observe(vec![1], "run-a", 9, now, interval),
            Publication::Skip
        );
        assert_eq!(
            gate.observe(vec![1], "run-a", 10, now, interval),
            Publication::Skip
        );
        assert_eq!(
            gate.observe(vec![0], "run-a", 11, now, interval),
            Publication::Skip
        );
        // A suppressed sample still advances source ordering, not full sequence.
        assert_eq!(
            gate.observe(vec![1], "run-a", 11, now, interval),
            Publication::Skip
        );
        assert_eq!(
            gate.observe(vec![0], "run-a", 12, now + interval, interval),
            Publication::Heartbeat(10)
        );
        assert_eq!(
            gate.observe(vec![1], "run-a", 13, now + interval, interval),
            Publication::Full
        );
        assert_eq!(
            gate.observe(vec![1], "run-b", 1, now + interval, interval),
            Publication::Full
        );
    }

    #[test]
    /// Simulates failed enqueue, preserving the source high-water mark while forcing
    /// the next newer sample to supply a full baseline before heartbeats resume.
    fn rejected_full_is_retried_and_never_becomes_heartbeat_identity() {
        let now = Instant::now();
        let interval = Duration::from_millis(200);
        let mut gate = Gate::default();
        assert_eq!(
            gate.observe(vec![0], "frame", 1, now, interval),
            Publication::Full
        );
        gate.full_enqueued(false);
        assert_eq!(
            gate.observe(vec![0], "frame", 1, now + interval, interval),
            Publication::Skip
        );
        gate.full_enqueued(false);
        assert_eq!(
            gate.observe(vec![0], "frame", 2, now + interval * 2, interval),
            Publication::Full
        );
        gate.full_enqueued(true);
        assert_eq!(
            gate.observe(vec![0], "frame", 3, now + interval * 3, interval),
            Publication::Heartbeat(2)
        );
    }

    #[test]
    /// Checks stale cached maps cannot seed or recover a visualization baseline,
    /// including the strict half-second freshness boundary and invalid clock values.
    fn stale_cached_map_cannot_publish_on_attachment_or_recovery() {
        let start = Instant::now();
        let interval = Duration::from_millis(200);
        let mut gate = Gate::default();
        // Advance source time separately from wall-clock publication time to test stale-cache recovery.
        let mut publish = |now, sample, sequence, elapsed| {
            if !fresh_map_sample(now, sample) {
                return Publication::Skip;
            }
            gate.observe(
                vec![-1],
                "run",
                sequence,
                start + Duration::from_secs(elapsed),
                interval,
            )
        };
        assert_eq!(publish(10., 9., 1, 0), Publication::Skip);
        assert_eq!(publish(10., 10., 2, 0), Publication::Full);
        // A sample not previously observed by the gate is still too old to
        // trigger the recovery baseline, even after the two-second deadline.
        assert_eq!(publish(13., 10.1, 3, 3), Publication::Skip);
        assert_eq!(publish(13., 13., 4, 3), Publication::Full);
        for (now, sample) in [
            (f64::NAN, 0.),
            (0., f64::NAN),
            (f64::INFINITY, 0.),
            (0., f64::INFINITY),
            (-1., 0.),
            (0., -1.),
            (0., 1.),
            (1., 0.5),
        ] {
            assert!(!fresh_map_sample(now, sample));
        }
        assert!(fresh_map_sample(1., 0.501));
    }

    #[test]
    /// Varies each cell, dimensions, resolution, origin, and run identity to require
    /// a complete map publication instead of a heartbeat.
    fn every_map_cell_geometry_and_run_change_forces_full() {
        let now = Instant::now();
        let interval = Duration::from_millis(200);
        let mut gate = MapGate::default();
        let base = ("run-a".to_string(), 2, 1, 0.1, 0., 0., vec![-1, 0]);
        assert_eq!(
            gate.observe(base.clone(), "run-a".into(), 1, now, interval),
            Publication::Full
        );
        let mut variants = Vec::new();
        let mut changed = base.clone();
        changed.6[0] = 100;
        variants.push(changed);
        let mut changed = base.clone();
        changed.6[1] = -1;
        variants.push(changed);
        let mut changed = base.clone();
        changed.1 = 1;
        changed.2 = 2;
        variants.push(changed);
        let mut changed = base.clone();
        changed.3 = 0.2;
        variants.push(changed);
        let mut changed = base.clone();
        changed.4 = 1.;
        variants.push(changed);
        let mut changed = base.clone();
        changed.5 = 1.;
        variants.push(changed);
        let mut changed = base;
        changed.0 = "run-b".into();
        variants.push(changed);
        for (index, changed) in variants.into_iter().enumerate() {
            let identity = changed.0.clone();
            assert_eq!(
                gate.observe(changed, identity, index as u64 + 2, now, interval),
                Publication::Full
            );
        }
    }

    #[test]
    /// Checks unchanged clouds obey the 500 ms heartbeat interval, duplicates remain suppressed,
    /// new source samples recover a full baseline after two seconds, and content/frame changes force full payloads.
    fn exact_cloud_content_frame_and_rate_limit() {
        let now = Instant::now();
        let interval = Duration::from_millis(500);
        let mut gate = CloudGate::default();
        let content = serde_json::json!({"version":1,"frameID":"a","points":[[1.,2.,3.]]});
        assert_eq!(
            gate.observe(content.clone(), "a".into(), 1, now, interval),
            Publication::Full
        );
        assert_eq!(
            gate.observe(content.clone(), "a".into(), 2, now + interval / 2, interval),
            Publication::Skip
        );
        assert_eq!(
            gate.observe(content.clone(), "a".into(), 3, now + interval, interval),
            Publication::Heartbeat(1)
        );
        assert_eq!(
            gate.observe(
                content.clone(),
                "a".into(),
                3,
                now + Duration::from_secs(3),
                interval
            ),
            Publication::Skip
        );
        assert_eq!(
            gate.observe(
                content.clone(),
                "a".into(),
                4,
                now + Duration::from_secs(3),
                interval
            ),
            Publication::Full
        );
        let mut moved = content.clone();
        moved["points"][0][0] = serde_json::json!(1.00001);
        assert_eq!(
            gate.observe(moved, "a".into(), 5, now + Duration::from_secs(3), interval),
            Publication::Full
        );
        let mut reframed = content;
        reframed["frameID"] = serde_json::json!("b");
        assert_eq!(
            gate.observe(
                reframed,
                "b".into(),
                1,
                now + Duration::from_secs(3),
                interval
            ),
            Publication::Full
        );
    }

    #[test]
    /// Checks initial full publication, heartbeat pacing, duplicate suppression,
    /// two-second recovery, content invalidation, and an explicit gate reset.
    fn freshness_content_recovery_and_reset() {
        let mut gate = Gate::default();
        let start = Instant::now();
        let interval = Duration::from_millis(200);
        // Vary content, source sequence, and elapsed milliseconds against one gate baseline.
        let mut observe = |content, source, ms| {
            gate.observe(
                content,
                (),
                source,
                start + Duration::from_millis(ms),
                interval,
            )
        };
        assert_eq!(observe(vec![-1], 1, 0), Publication::Full);
        assert_eq!(observe(vec![-1], 2, 100), Publication::Skip);
        assert_eq!(observe(vec![-1], 3, 200), Publication::Heartbeat(1));
        assert_eq!(observe(vec![-1], 3, 2100), Publication::Skip);
        assert_eq!(observe(vec![-1], 4, 2101), Publication::Full);
        assert_eq!(observe(vec![100], 5, 2102), Publication::Full);
        gate = Gate::default();
        assert_eq!(
            gate.observe(vec![100], (), 5, start, interval),
            Publication::Full
        );
    }
}
