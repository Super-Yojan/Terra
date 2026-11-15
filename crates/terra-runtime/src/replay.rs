//! Bounded memory of request tokens, so a replayed request is not applied twice.
use std::collections::VecDeque;

/// The last `N` tokens seen; the oldest is forgotten first.
#[derive(Clone, Debug, Default)]
pub(crate) struct ReplayGuard<const N: usize> {
    seen: VecDeque<String>,
}

impl<const N: usize> ReplayGuard<N> {
    pub(crate) fn seen(&self, token: &str) -> bool {
        self.seen.iter().any(|t| t == token)
    }
    pub(crate) fn record(&mut self, token: &str) {
        if self.seen.len() >= N {
            self.seen.pop_front();
        }
        self.seen.push_back(token.into());
    }
    /// Record a token not seen before; `false` for a replay.
    pub(crate) fn admit(&mut self, token: &str) -> bool {
        if self.seen(token) {
            return false;
        }
        self.record(token);
        true
    }
}
