use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Scope {
    pub producer_id: String,
    pub rovers: Vec<String>,
    pub read: bool,
}
#[derive(Clone, Deserialize)]
pub struct Credential {
    pub token_sha256: String,
    pub scope: Scope,
}
/// Config contains token hashes, not cleartext bearer credentials.
pub fn authorize(token: &str, credentials: &[Credential], read: bool) -> Option<Scope> {
    let hash = format!("{:x}", Sha256::digest(token.as_bytes()));
    credentials
        .iter()
        .find(|c| {
            c.scope.read == read && constant_equal(hash.as_bytes(), c.token_sha256.as_bytes())
        })
        .map(|c| c.scope.clone())
}
fn constant_equal(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b).fold(0u8, |diff, (x, y)| diff | (x ^ y)) == 0
}
