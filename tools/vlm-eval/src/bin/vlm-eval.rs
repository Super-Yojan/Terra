//! `vlm-eval` builds the NEXT packet pack and scores an OpenAI-compatible VLM.
//! Mock mode does not call a model. This binary never writes motor commands.

use std::env;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use terra_vlm_eval::{
    RunConfig, rules_sha256, run_eval, scenarios, score_output, user_prompt, write_fixtures,
    write_report,
};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("generate") => cmd_generate(&args),
        Some("run") => cmd_run(&args),
        Some("prompt") => cmd_prompt(&args),
        Some("score") => cmd_score(&args),
        Some("list") => cmd_list(),
        _ => {
            usage();
            ExitCode::from(2)
        }
    }
}

fn cmd_list() -> ExitCode {
    println!("rules_sha256 {}", rules_sha256());
    println!("camera frames are placeholders, not Zorvane renders");
    for scenario in scenarios() {
        println!(
            "{}  {:<16}  {:<28}  dump={}",
            scenario.id,
            format!("{:?}", scenario.level).to_lowercase(),
            scenario.expected.decision.as_str(),
            scenario.expected.dump_proposal
        );
        println!("    {}", scenario.title);
    }
    ExitCode::SUCCESS
}

fn cmd_generate(args: &[String]) -> ExitCode {
    let out = flag(args, "--out").unwrap_or_else(|| "tools/vlm-eval/fixtures".to_string());
    match write_fixtures(PathBuf::from(out).as_path()) {
        Ok(()) => {
            println!(
                "wrote {}",
                flag(args, "--out").unwrap_or_else(|| "tools/vlm-eval/fixtures".into())
            );
            ExitCode::SUCCESS
        }
        Err(err) => {
            eprintln!("generate: {err}");
            ExitCode::from(2)
        }
    }
}

fn cmd_prompt(args: &[String]) -> ExitCode {
    let Some(id) = args.get(1).filter(|value| !value.starts_with("--")) else {
        eprintln!("usage: vlm-eval prompt <scenario-id>");
        return ExitCode::from(2);
    };
    match scenarios().iter().find(|scenario| scenario.id == *id) {
        Some(scenario) => {
            println!("{}", user_prompt(scenario));
            ExitCode::SUCCESS
        }
        None => {
            eprintln!("unknown scenario {id}");
            ExitCode::from(2)
        }
    }
}

fn cmd_score(args: &[String]) -> ExitCode {
    let Some(id) = flag(args, "--scenario") else {
        eprintln!("usage: vlm-eval score --scenario <id> --response <file>");
        return ExitCode::from(2);
    };
    let Some(path) = flag(args, "--response") else {
        eprintln!("usage: vlm-eval score --scenario <id> --response <file>");
        return ExitCode::from(2);
    };
    let Some(scenario) = scenarios().iter().find(|scenario| scenario.id == id) else {
        eprintln!("unknown scenario {id}");
        return ExitCode::from(2);
    };
    let raw = match std::fs::read_to_string(&path) {
        Ok(text) => text,
        Err(err) => {
            eprintln!("read {path}: {err}");
            return ExitCode::from(2);
        }
    };
    let score = score_output(&raw, scenario);
    println!(
        "valid_json={} no_actuator={} action={} waypoint={} pass={}",
        score.valid_json,
        score.no_actuator,
        score.action_ok,
        score.waypoint_ok,
        score.passed()
    );
    for note in &score.notes {
        println!("note: {note}");
    }
    if score.passed() {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

fn cmd_run(args: &[String]) -> ExitCode {
    let mock = has(args, "--mock");
    let model = if mock {
        flag(args, "--model").unwrap_or_else(|| "mock".to_string())
    } else if let Some(model) = flag(args, "--model") {
        model
    } else {
        eprintln!("usage: vlm-eval run --base-url URL --model NAME [--out FILE]");
        eprintln!("       vlm-eval run --mock [--out FILE]");
        return ExitCode::from(2);
    };
    let timeout: u64 = flag(args, "--timeout-secs")
        .and_then(|text| text.parse().ok())
        .unwrap_or(180);
    let config = RunConfig {
        mock,
        base_url: flag(args, "--base-url"),
        model,
        api_key: flag(args, "--api-key").or_else(|| env::var("VLM_API_KEY").ok()),
        timeout: Duration::from_secs(timeout),
    };
    let report = match run_eval(&config) {
        Ok(report) => report,
        Err(err) => {
            eprintln!("run: {err}");
            return ExitCode::from(2);
        }
    };
    let markdown = terra_vlm_eval::render_markdown(&report);
    print!("{markdown}");
    if let Some(path) = flag(args, "--out") {
        if let Err(err) = write_report(&report, PathBuf::from(&path).as_path()) {
            eprintln!("write {path}: {err}");
            return ExitCode::from(2);
        }
    }
    if report.passed == report.total {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

fn flag(args: &[String], name: &str) -> Option<String> {
    args.windows(2)
        .find(|pair| pair[0] == name)
        .map(|pair| pair[1].clone())
}

fn has(args: &[String], name: &str) -> bool {
    args.iter().any(|arg| arg == name)
}

fn usage() {
    eprintln!(
        "\
vlm-eval list
vlm-eval generate [--out DIR]
vlm-eval prompt <scenario-id>
vlm-eval score --scenario <id> --response <file>
vlm-eval run --mock [--out FILE]
vlm-eval run --base-url http://127.0.0.1:8080/v1 --model NAME [--api-key KEY] [--timeout-secs N] [--out FILE]

Mock mode replays the labels. It is not a model score.
The score subcommand reads a saved completion. See the README for mock versus a real server."
    );
}
