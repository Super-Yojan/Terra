use crate::{Scope, Store};
use rusqlite::params;
use sha2::{Digest, Sha256};
use terra_observability::{JournalError, valid_hash};
#[derive(Debug)]
pub struct InvalidAttachment;
impl std::fmt::Display for InvalidAttachment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("invalid attachment or scope")
    }
}
impl std::error::Error for InvalidAttachment {}
impl Store {
    pub fn put_attachment(
        &self,
        scope: &Scope,
        rover: &str,
        hash: &str,
        bytes: &[u8],
    ) -> Result<(), JournalError> {
        if scope.read
            || (!scope.rovers.iter().any(|r| r == rover) && rover != "_producer")
            || !valid_hash(hash)
            || bytes.len() > 32 * 1024 * 1024
            || bytes.len() as u64 > self.config.attachment_max_bytes
            || format!("{:x}", Sha256::digest(bytes)) != hash
        {
            return Err(Box::new(InvalidAttachment));
        }
        let _guard = self
            .attachment_lock
            .lock()
            .map_err(|_| "attachment storage lock unavailable")?;
        let directory = self.config.path.with_extension("attachments");
        std::fs::create_dir_all(&directory)?;
        let db = self.db()?;
        db.execute_batch("CREATE TABLE IF NOT EXISTS attachments(hash TEXT,rover TEXT,producer TEXT,bytes INTEGER,stored_ns INTEGER,PRIMARY KEY(hash,rover,producer))")?;
        let path = directory.join(hash);
        if !path.exists() {
            let mut files = std::fs::read_dir(&directory)?
                .filter_map(Result::ok)
                .filter_map(|f| {
                    if !f.file_name().to_str().is_some_and(valid_hash) {
                        return None;
                    }
                    let m = f.metadata().ok()?;
                    Some((m.modified().ok()?, m.len(), f.path()))
                })
                .collect::<Vec<_>>();
            files.sort_by_key(|f| f.0);
            let mut total: u64 = files.iter().map(|f| f.1).sum();
            for (_, size, path) in files {
                if total + bytes.len() as u64 <= self.config.attachment_max_bytes {
                    break;
                }
                if let Some(hash) = path.file_name().and_then(|s| s.to_str()) {
                    note_loss(&db, hash)?;
                }
                std::fs::remove_file(path)?;
                total = total.saturating_sub(size);
            }
            let tmp = directory.join(format!("{hash}.tmp"));
            let mut file = std::fs::File::create(&tmp)?;
            use std::io::Write;
            file.write_all(bytes)?;
            file.sync_all()?;
            std::fs::rename(tmp, &path)?;
        }
        db.execute("INSERT INTO attachments(hash,rover,producer,bytes,stored_ns) VALUES(?,?,?,?,?) ON CONFLICT(hash,rover,producer) DO UPDATE SET stored_ns=excluded.stored_ns",params![hash,rover,scope.producer_id,bytes.len(),terra_observability::wall_time_ns()])?;
        std::fs::File::open(&path)?.set_modified(std::time::SystemTime::now())?;
        Ok(())
    }
    pub fn attachment(&self, scope: &Scope, hash: &str) -> Result<Vec<u8>, JournalError> {
        let _guard = self
            .attachment_lock
            .lock()
            .map_err(|_| "attachment storage lock unavailable")?;
        if !valid_hash(hash) {
            return Err("invalid attachment hash".into());
        }
        let db = self.db()?;
        db.execute_batch("CREATE TABLE IF NOT EXISTS attachments(hash TEXT,rover TEXT,producer TEXT,bytes INTEGER,stored_ns INTEGER,PRIMARY KEY(hash,rover,producer))")?;
        let mut query = db.prepare("SELECT rover,producer FROM attachments WHERE hash=?")?;
        let allowed = query
            .query_map([hash], |r| {
                Ok((r.get::<_, String>(0)?, r.get::<_, String>(1)?))
            })?
            .filter_map(Result::ok)
            .any(|(r, p)| {
                scope.rovers.contains(&r) || (r == "_producer" && scope.producer_id == p)
            });
        if !allowed {
            return Err("attachment not found".into());
        }
        let path = self.config.path.with_extension("attachments").join(hash);
        if std::fs::metadata(&path)?.len() > 32 * 1024 * 1024 {
            return Err("attachment too large".into());
        }
        Ok(std::fs::read(path)?)
    }
}

fn note_loss(db: &rusqlite::Connection, hash: &str) -> Result<(), JournalError> {
    db.execute("INSERT INTO retention_gaps(producer,rover,at_ns,count,attachments) SELECT producer,rover,?,0,1 FROM attachments WHERE hash=? ON CONFLICT(producer,rover) DO UPDATE SET attachments=attachments+1,at_ns=MAX(at_ns,excluded.at_ns)",params![terra_observability::wall_time_ns(),hash])?;
    db.execute("DELETE FROM attachments WHERE hash=?", [hash])?;
    Ok(())
}
impl Store {
    pub fn prune_attachments(&self) -> Result<(), JournalError> {
        let _guard = self
            .attachment_lock
            .lock()
            .map_err(|_| "attachment storage lock unavailable")?;
        let directory = self.config.path.with_extension("attachments");
        if !directory.exists() {
            return Ok(());
        }
        let db = self.db()?;
        db.execute_batch("CREATE TABLE IF NOT EXISTS attachments(hash TEXT,rover TEXT,producer TEXT,bytes INTEGER,stored_ns INTEGER,PRIMARY KEY(hash,rover,producer))")?;
        let now = std::time::SystemTime::now();
        for entry in std::fs::read_dir(&directory)?.filter_map(Result::ok) {
            if !entry.file_name().to_str().is_some_and(valid_hash) {
                continue;
            }
            if entry
                .metadata()?
                .modified()
                .ok()
                .and_then(|t| now.duration_since(t).ok())
                .is_some_and(|age| age.as_secs() >= self.config.retention_seconds)
            {
                if let Some(hash) = entry.file_name().to_str() {
                    note_loss(&db, hash)?;
                }
                std::fs::remove_file(entry.path())?;
            }
        }
        Ok(())
    }
}
