//! The `[comms]` table.
//!
//! ```toml
//! [comms]
//! endpoint = "tls/argos.example.com:7448"   # tcp/host:port or tls/host:port
//! prefix   = "terra"                        # first part of every key (default "terra")
//! rover_id = 0                              # rover only
//! user     = "terra"                        # optional router login
//! ```
//!
//! The password is never read from the file; pass it with
//! [`CommsConfig::with_password`].
use std::fmt;

use serde::Deserialize;
use serde::de::IgnoredAny;

use crate::topics::check_prefix;

#[derive(Clone, PartialEq, Eq, Deserialize)]
#[serde(try_from = "RawCommsConfig")]
pub struct CommsConfig {
    pub endpoint: String,
    pub prefix: String,
    pub rover_id: Option<u64>,
    pub user: Option<String>,
    pub password: Option<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawCommsConfig {
    endpoint: String,
    #[serde(default = "default_prefix")]
    prefix: String,
    rover_id: Option<u64>,
    user: Option<String>,
    // Accepted by the parser only to give a clear error.
    password: Option<IgnoredAny>,
}

fn default_prefix() -> String {
    "terra".into()
}

impl TryFrom<RawCommsConfig> for CommsConfig {
    type Error = ConfigError;

    fn try_from(raw: RawCommsConfig) -> Result<Self, ConfigError> {
        if raw.password.is_some() {
            return Err(ConfigError::new(
                "[comms] password must not be in the file; pass it from the Keychain or env",
            ));
        }
        let config = Self {
            endpoint: raw.endpoint,
            prefix: raw.prefix,
            rover_id: raw.rover_id,
            user: raw.user,
            password: None,
        };
        config.check()?;
        Ok(config)
    }
}

impl CommsConfig {
    /// Parse the `[comms]` table of a TOML document. Other tables in the
    /// document are ignored (they belong to the rest of the file). Rejects a
    /// missing `[comms]`, unknown keys, a bad endpoint and a prefix that isn't
    /// a plain key (`*`, `?`, `#`, `$`, empty parts).
    pub fn from_toml(table: &str) -> Result<Self, ConfigError> {
        #[derive(Deserialize)]
        struct Document {
            comms: Option<CommsConfig>,
        }
        let doc: Document = toml::from_str(table).map_err(|e| ConfigError::new(e.to_string()))?;
        doc.comms.ok_or_else(|| ConfigError::new("missing [comms] table"))
    }

    /// Use this password instead of whatever the config had.
    pub fn with_password(mut self, password: Option<String>) -> Self {
        if password.is_some() {
            self.password = password;
        }
        self
    }

    /// Check the fields (they are public, so they may have changed since
    /// parsing).
    pub fn check(&self) -> Result<(), ConfigError> {
        check_endpoint(&self.endpoint)?;
        check_prefix(&self.prefix)?;
        if self.user.as_deref() == Some("") {
            return Err(ConfigError::new("[comms] user must not be empty"));
        }
        Ok(())
    }
}

// Keep the password out of logs.
impl fmt::Debug for CommsConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CommsConfig")
            .field("endpoint", &self.endpoint)
            .field("prefix", &self.prefix)
            .field("rover_id", &self.rover_id)
            .field("user", &self.user)
            .field("password", &self.password.as_ref().map(|_| "***"))
            .finish()
    }
}

/// `tcp/host:port` or `tls/host:port`, optionally followed by Zenoh
/// endpoint metadata/config (`?…`, `#…`).
fn check_endpoint(endpoint: &str) -> Result<(), ConfigError> {
    let bad = |why: &str| ConfigError::new(format!("[comms] endpoint {endpoint:?}: {why}"));
    let (proto, rest) = endpoint.split_once('/').ok_or_else(|| bad("expected tcp/host:port or tls/host:port"))?;
    if proto != "tcp" && proto != "tls" {
        return Err(bad("protocol must be tcp or tls"));
    }
    let address = rest.split(['?', '#']).next().unwrap_or("");
    let (host, port) = address.rsplit_once(':').ok_or_else(|| bad("missing :port"))?;
    if host.is_empty() || host == "[]" {
        return Err(bad("missing host"));
    }
    if !matches!(port.parse::<u16>(), Ok(p) if p != 0) {
        return Err(bad("port must be 1-65535"));
    }
    // Zenoh's own parser catches the rest (bad metadata/config syntax, …).
    #[cfg(feature = "zenoh")]
    endpoint.parse::<zenoh::config::EndPoint>().map_err(|e| bad(&e.to_string()))?;
    Ok(())
}

/// A bad config. Shown to the user, never retried.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConfigError {
    message: String,
}

impl ConfigError {
    pub(crate) fn new(message: impl Into<String>) -> Self {
        Self { message: message.into() }
    }
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ConfigError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_full_table() {
        let c = CommsConfig::from_toml(
            r#"
            [vehicle]
            type = "diff_drive"
            [comms]
            endpoint = "tls/argos.example.com:7448"
            prefix = "terra"
            rover_id = 3
            user = "terra"
            "#,
        )
        .unwrap();
        assert_eq!(c.endpoint, "tls/argos.example.com:7448");
        assert_eq!(c.rover_id, Some(3));
        assert_eq!(c.user.as_deref(), Some("terra"));
        assert_eq!(c.password, None);
        let c = c.with_password(Some("secret".into()));
        assert_eq!(c.password.as_deref(), Some("secret"));
        assert!(!format!("{c:?}").contains("secret"));
    }

    #[test]
    fn defaults() {
        let c = CommsConfig::from_toml("[comms]\nendpoint = \"tcp/10.0.0.1:7447\"").unwrap();
        assert_eq!(c.prefix, "terra");
        assert_eq!(c.rover_id, None);
        assert_eq!(c.user, None);
    }

    #[test]
    fn rejects_bad_input() {
        let bad = [
            "",                                                          // no [comms]
            "[comms]\nendpoint = \"tcp/h:1\"\nfoo = 1",                  // unknown key
            "[comms]\nendpoint = \"tcp/h:1\"\npassword = \"x\"",         // password in file
            "[comms]\nendpoint = \"udp/h:1\"",                           // protocol
            "[comms]\nendpoint = \"tcp/h\"",                             // no port
            "[comms]\nendpoint = \"tcp/:7447\"",                         // no host
            "[comms]\nendpoint = \"tcp/h:99999\"",                       // port
            "[comms]\nendpoint = \"tcp/h:1\"\nprefix = \"a/*\"",         // wildcard
            "[comms]\nendpoint = \"tcp/h:1\"\nprefix = \"a//b\"",        // empty part
            "[comms]\nendpoint = \"tcp/h:1\"\nprefix = \"\"",            // empty
            "[comms]\nendpoint = \"tcp/h:1\"\nrover_id = -1",            // negative id
        ];
        for text in bad {
            assert!(CommsConfig::from_toml(text).is_err(), "accepted {text:?}");
        }
    }

    #[test]
    fn endpoint_variants() {
        for ok in ["tcp/[::1]:7447", "tls/host:7448?x=y", "tcp/1.2.3.4:1#a=b"] {
            check_endpoint(ok).unwrap();
        }
    }
}
