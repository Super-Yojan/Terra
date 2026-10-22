use std::{
    io::{BufRead, Read, Write},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    time::{Duration, Instant},
};
use terra_observability::*;
struct Server {
    endpoint: String,
    posts: Arc<AtomicUsize>,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Server {
    fn start() -> Self {
        Self::with_status(200)
    }
    fn with_status(status: u16) -> Self {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("http://{}", listener.local_addr().unwrap());
        listener.set_nonblocking(true).unwrap();
        let posts = Arc::new(AtomicUsize::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let count = posts.clone();
        let stopping = stop.clone();
        let thread = std::thread::spawn(move || {
            while !stopping.load(Ordering::Relaxed) {
                let Ok((mut stream, _)) = listener.accept() else {
                    std::thread::sleep(Duration::from_millis(5));
                    continue;
                };
                stream.set_nonblocking(false).unwrap();
                stream
                    .set_read_timeout(Some(Duration::from_secs(2)))
                    .unwrap();
                let mut reader = std::io::BufReader::new(&mut stream);
                let mut line = String::new();
                if reader.read_line(&mut line).is_err() {
                    continue;
                }
                let clock = line.starts_with("GET ");
                let mut length = 0;
                loop {
                    line.clear();
                    if reader.read_line(&mut line).unwrap_or(0) == 0 || line == "\r\n" {
                        break;
                    }
                    if let Some(value) = line.to_lowercase().strip_prefix("content-length:") {
                        length = value.trim().parse::<usize>().unwrap();
                    }
                }
                let mut data = vec![0; length];
                if reader.read_exact(&mut data).is_err() {
                    continue;
                }
                drop(reader);
                let body=if clock{serde_json::json!({"server_wall_ns":wall_time_ns()})}else{count.fetch_add(1,Ordering::Relaxed);let v:serde_json::Value=serde_json::from_slice(&data).unwrap();serde_json::json!({"acknowledged":v["events"].as_array().unwrap().iter().map(|e|e["id"].clone()).collect::<Vec<_>>()})}.to_string();
                let _ = write!(
                    stream,
                    "HTTP/1.1 {status} response\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: {}\r\n\r\n{}",
                    body.len(),
                    body
                );
            }
        });
        Self {
            endpoint,
            posts,
            stop,
            thread: Some(thread),
        }
    }
}
impl Drop for Server {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        self.thread.take().unwrap().join().unwrap();
    }
}
#[test]
fn cloud_accepted_batch_retries_only_local_ack_until_storage_recovers() {
    let server = Server::start();
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    let journal = Journal::open(JournalConfig {
        path: path.clone(),
        ..Default::default()
    })
    .unwrap();
    journal.try_record(EventInput::new(
        "0",
        "control",
        "state",
        serde_json::json!({}),
    ));
    journal.flush(Duration::from_secs(2)).unwrap();
    let reader = JournalReader::open(&path).unwrap();
    let original = reader.batch(1).unwrap()[0].clone();
    let db = rusqlite::Connection::open(&path).unwrap();
    db.execute_batch("BEGIN IMMEDIATE").unwrap();
    let uploader = UploadHandle::start(
        reader.clone(),
        UploadConfig {
            endpoint: server.endpoint.clone(),
            token: "test".into(),
            clock_journal: None,
            evidence_directory: None,
        },
    )
    .unwrap();
    let deadline = Instant::now() + Duration::from_secs(4);
    while server.posts.load(Ordering::Relaxed) == 0 {
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(10));
    }
    std::thread::sleep(Duration::from_millis(5300));
    let sends = server.posts.load(Ordering::Relaxed);
    db.execute_batch("ROLLBACK").unwrap();
    assert_eq!(
        sends, 1,
        "GCP already accepted this batch; retry only the local ack write"
    );
    let deadline = Instant::now() + Duration::from_secs(8);
    while reader.pending_count().unwrap() > 0 {
        assert!(Instant::now() < deadline, "queue did not recover");
        std::thread::sleep(Duration::from_millis(20));
    }
    assert_eq!(server.posts.load(Ordering::Relaxed), 1);
    assert_eq!(reader.export().unwrap()[0].id, original.id);
    assert_eq!(
        reader.export().unwrap()[0].input.wall_time_ns,
        original.input.wall_time_ns
    );
    assert_eq!(uploader.health().acknowledged, 1);
}

#[test]
fn authentication_and_throttling_responses_leave_original_events_pending() {
    for status in [401, 403, 429] {
        let server = Server::with_status(status);
        let d = tempfile::tempdir().unwrap();
        let path = d.path().join("events.db");
        let j = Journal::open(JournalConfig {
            path: path.clone(),
            ..Default::default()
        })
        .unwrap();
        j.try_record(EventInput::new("0", "test", "state", serde_json::json!({})));
        j.flush(Duration::from_secs(2)).unwrap();
        let reader = JournalReader::open(&path).unwrap();
        let original = reader.batch(1).unwrap()[0].clone();
        let mut transport = HttpTransport::new(&server.endpoint, "never-log-me").unwrap();
        let error = upload_once(&reader, &mut transport, None).unwrap_err();
        assert!(error.to_string().contains(&status.to_string()));
        assert!(!error.to_string().contains("never-log-me"));
        assert_eq!(reader.pending_count().unwrap(), 1);
        assert_eq!(reader.batch(1).unwrap()[0], original);
    }
}
