//! Summarize a JSONL experiment recording as pretty-printed JSON on standard output.
//!
//! Usage: `terra-run-summary <run.jsonl>`. An interrupted final line is ignored;
//! malformed complete records, missing arguments, and I/O errors produce an error exit.

/// Read the requested recording, compute metrics, and print the summary.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: terra-run-summary <run.jsonl>")?;
    let records = terra_experiment::read_records(&std::fs::read(path)?)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&terra_experiment::summarize(&records))?
    );
    Ok(())
}
