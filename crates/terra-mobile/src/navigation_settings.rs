pub(crate) use terra_runtime::navigation::Parameters;
pub(crate) fn apply(b: &mut super::Brain, bytes: &[u8], age: f64) {
    let now = b.last_tick;
    terra_runtime::navigation::apply(&mut b.runtime, bytes, age, now);
}
