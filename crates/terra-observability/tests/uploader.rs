use serde_json::json;
use std::time::Duration;
use terra_observability::*;
struct LostReply {
    lost: bool,
    ids: Vec<EventId>,
}
impl UploadTransport for LostReply {
    fn put_attachment(&mut self, _: &str, _: &str, _: &[u8]) -> Result<(), JournalError> {
        Ok(())
    }
    fn send(&mut self, events: &[EventEnvelope]) -> Result<Vec<EventId>, JournalError> {
        self.ids.extend(events.iter().map(|e| e.id.clone()));
        if !self.lost {
            self.lost = true;
            return Err("response lost".into());
        }
        Ok(events.iter().map(|e| e.id.clone()).collect())
    }
}
#[test]
fn reconnect_retries_same_identity_and_original_time() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("db");
    let j = Journal::open(JournalConfig {
        path: path.clone(),
        ..Default::default()
    })
    .unwrap();
    let mut e = EventInput::new("4", "phone", "state", json!({}));
    e.monotonic_ns = 42;
    j.try_record(e);
    j.flush(Duration::from_secs(2)).unwrap();
    let r = JournalReader::open(&path).unwrap();
    let mut transport = LostReply {
        lost: false,
        ids: vec![],
    };
    assert!(upload_once(&r, &mut transport, None).is_err());
    assert_eq!(r.batch(10).unwrap()[0].input.monotonic_ns, 42);
    upload_once(&r, &mut transport, None).unwrap();
    assert!(r.batch(10).unwrap().is_empty());
    assert_eq!(transport.ids[0], transport.ids[1]);
}
#[test]
fn unsafe_endpoint_rejected() {
    assert!(HttpTransport::new("http://example.com", "token").is_err());
    assert!(HttpTransport::new("https://example.com", "token").is_ok());
}
#[test]
fn corrupt_image_does_not_block_durable_event_delivery() {
    use sha2::{Digest, Sha256};
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("db");
    let j = Journal::open(JournalConfig {
        path: path.clone(),
        ..Default::default()
    })
    .unwrap();
    let directory = d.path().join("evidence");
    std::fs::create_dir(&directory).unwrap();
    let hash = format!("{:x}", Sha256::digest(b"image"));
    std::fs::write(directory.join(&hash), b"wrong").unwrap();
    j.try_record(EventInput::new("4","mapping","mapping.evidence",json!({"attachments":[{"name":"rgb","sha256":hash,"encoding":"jpeg","dimensions":[1,1],"units":"rgb","byte_length":5}]})));
    j.flush(Duration::from_secs(2)).unwrap();
    let reader = JournalReader::open(&path).unwrap();
    let mut t = LostReply {
        lost: true,
        ids: vec![],
    };
    assert_eq!(upload_once(&reader, &mut t, Some(&directory)).unwrap(), 1);
    assert!(
        reader
            .batch(10)
            .unwrap()
            .iter()
            .any(|e| e.input.kind == "observability.gap")
    );
}
#[test]
fn queued_burst_drains_without_one_second_per_batch() {
    use std::io::{BufRead, Read, Write};
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    listener.set_nonblocking(true).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let stop_server = stop.clone();
    let server = std::thread::spawn(move || {
        while !stop_server.load(Ordering::Acquire) {
            let Ok((mut stream, _)) = listener.accept() else {
                std::thread::sleep(Duration::from_millis(5));
                continue;
            };
            // macOS inherits nonblocking mode from the listening socket.
            stream.set_nonblocking(false).unwrap();
            stream
                .set_read_timeout(Some(Duration::from_secs(2)))
                .unwrap();
            let mut reader = std::io::BufReader::new(&mut stream);
            let mut line = String::new();
            reader.read_line(&mut line).unwrap();
            let clock = line.starts_with("GET ");
            let mut length = 0;
            loop {
                line.clear();
                reader.read_line(&mut line).unwrap();
                if line == "\r\n" {
                    break;
                }
                if let Some(v) = line.to_lowercase().strip_prefix("content-length:") {
                    length = v.trim().parse::<usize>().unwrap();
                }
            }
            let mut data = vec![0; length];
            reader.read_exact(&mut data).unwrap();
            drop(reader);
            let body = if clock {
                json!({"server_wall_ns":terra_observability::wall_time_ns()})
            } else {
                let v: serde_json::Value = serde_json::from_slice(&data).unwrap();
                json!({"acknowledged":v["events"].as_array().unwrap().iter().map(|e|e["id"].clone()).collect::<Vec<_>>()})
            };
            let body = body.to_string();
            write!(stream,"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: {}\r\n\r\n{}",body.len(),body).unwrap();
        }
    });
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("db");
    let j = Journal::open(JournalConfig {
        path: path.clone(),
        ..Default::default()
    })
    .unwrap();
    for _ in 0..1280 {
        j.try_record(EventInput::new("4", "control", "state", json!({})));
    }
    j.flush(Duration::from_secs(5)).unwrap();
    let reader = JournalReader::open(&path).unwrap();
    let uploader = UploadHandle::start(
        reader.clone(),
        UploadConfig {
            endpoint: format!("http://{address}"),
            token: "test".into(),
            evidence_directory: None,
            clock_journal: None,
        },
    )
    .unwrap();
    let start = std::time::Instant::now();
    while reader.pending_count().unwrap() > 0 && start.elapsed() < Duration::from_secs(15) {
        std::thread::sleep(Duration::from_millis(20));
    }
    let pending = reader.pending_count().unwrap();
    let health = serde_json::to_string(&uploader.health()).unwrap();
    let drained = pending == 0;
    drop(uploader);
    stop.store(true, Ordering::Release);
    server.join().unwrap();
    assert!(
        drained,
        "a healthy connection must drain twenty batches faster than one batch per second: pending={pending}, health={health}"
    );
}
