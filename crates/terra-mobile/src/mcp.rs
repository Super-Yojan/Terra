//! The rover's MCP server. Any agent can run an L5 task through these tools; every
//! write goes through the arbiter's task checks. Swift only carries the bytes: an
//! in-process agent (Qwen on the phone) or an HTTP listener calls [`MobileController::mcp_handle`].
use crate::*;
use serde_json::{Value, json};
use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};
use terra_autonomy::{AgentOutcome, RequestedWaypoint, TaskPhase};

const PROTOCOL_VERSION: &str = "2025-06-18";
/// Session name the on-phone agent uses.
pub const IN_PROCESS_SESSION: &str = "in-process";
const CALLS_PER_SECOND: usize = 10;
const GO_TO_INTERVAL: Duration = Duration::from_secs(1);
const WAIT_MAX_SECONDS: f64 = 30.;
const LOOK_MAX_AGE_SECONDS: f64 = 2.;
/// Recent tool calls the on-phone agent sees in each prompt.
const TRANSCRIPT_LIMIT: usize = 8;
/// Replies without a tool call before the on-phone agent gives up the task.
const BAD_REPLY_LIMIT: u32 = 3;

#[derive(Default)]
pub(crate) struct McpState {
    /// Task id and session that holds write access to it.
    controller: Option<(String, String)>,
    calls: HashMap<String, VecDeque<Instant>>,
    last_go: HashMap<String, Instant>,
    /// Latest camera JPEG for `look` and its timestamp, set by the phone.
    frame: Option<(Vec<u8>, f64)>,
    agent: InProcessAgent,
    next_stop: u64,
}

/// Conversation state of the on-phone agent. Rust writes every prompt; the phone only generates.
#[derive(Default)]
struct InProcessAgent {
    attached: bool,
    task_id: Option<String>,
    next_prompt: u64,
    /// Prompt handed to the phone and not yet answered.
    outstanding: Option<u64>,
    /// Prompt again only after the task state moves on from this snapshot.
    resume: Option<(Snapshot, Option<Instant>)>,
    transcript: VecDeque<String>,
    attach_image: bool,
    bad_replies: u32,
}

#[derive(Clone, Debug, PartialEq)]
struct Snapshot {
    phase: Option<TaskPhase>,
    drive_request: bool,
    drive_grant: bool,
    history: usize,
    last: Option<String>,
}

#[derive(Clone, Debug, uniffi::Record)]
/// One prompt for the on-phone agent model. Answer it with
/// [`MobileController::submit_agent_reply`] using the same `prompt_id`.
pub struct MobileAgentPrompt {
    pub prompt_id: u64,
    /// Rules and tool list; use as the model's instructions.
    pub system: String,
    /// Current task, map, recent tool results and the request for one tool call.
    pub prompt: String,
    /// Attach the latest camera frame to this prompt: the agent called `look`.
    pub attach_image: bool,
}

fn tools() -> Value {
    let empty = json!({"type":"object","properties":{},"additionalProperties":false});
    json!([
        {"name":"get_state","description":"Rover pose, autonomy level, safety state and task phase.","inputSchema":empty},
        {"name":"get_task","description":"The operator's task: instruction, phase, time and steps left, current plan, the operator's last answer and history.","inputSchema":empty},
        {"name":"get_map","description":"The task, a 1 m text map around the rover and the ids go_to accepts: frontiers F#, obstacles O#, landmarks L# (detected objects) and the GPS goal G1. Call it before go_to.","inputSchema":empty},
        {"name":"get_detections","description":"Objects the detector has mapped, with class, colour, local x/y, confidence and whether they are confirmed.","inputSchema":empty},
        {"name":"look","description":"The latest camera frame as a JPEG.","inputSchema":{"type":"object","properties":{},"additionalProperties":false}},
        {"name":"go_to","description":"Drive to 1 to 5 waypoints in order. Each is {\"target\":\"F1\"} from get_map or {\"x\":1.5,\"y\":-2.0} in metres. The rover checks each against seen free space and an A* route; a refused plan changes nothing.","inputSchema":{"type":"object","properties":{"waypoints":{"type":"array","minItems":1,"maxItems":5,"items":{"type":"object","properties":{"target":{"type":"string"},"x":{"type":"number"},"y":{"type":"number"}}}},"rationale":{"type":"string"}},"required":["waypoints"],"additionalProperties":false}},
        {"name":"wait","description":"Wait up to 30 s for the plan to finish, be blocked or abandoned, or for the operator to answer.","inputSchema":{"type":"object","properties":{"seconds":{"type":"number","minimum":0,"maximum":30}},"additionalProperties":false}},
        {"name":"ask_operator","description":"Ask the operator a short question. The rover holds until they answer.","inputSchema":{"type":"object","properties":{"question":{"type":"string"}},"required":["question"],"additionalProperties":false}},
        {"name":"finish","description":"End the task: status done when it is accomplished, fail when it cannot be done.","inputSchema":{"type":"object","properties":{"status":{"type":"string","enum":["done","fail"]},"rationale":{"type":"string"}},"required":["status","rationale"],"additionalProperties":false}},
        {"name":"request_drive","description":"Ask the operator for raw drive for up to 60 s. Only after they approve does drive work.","inputSchema":{"type":"object","properties":{"reason":{"type":"string"},"seconds":{"type":"number","exclusiveMinimum":0,"maximum":60}},"required":["reason","seconds"],"additionalProperties":false}},
        {"name":"drive","description":"During an approved grant, drive at linear m/s (at most 0.5) and angular rad/s (at most 1.0) for up to 1 s. The obstacle planner still checks every command.","inputSchema":{"type":"object","properties":{"linear":{"type":"number"},"angular":{"type":"number"},"seconds":{"type":"number","exclusiveMinimum":0,"maximum":1}},"required":["linear","angular"],"additionalProperties":false}},
        {"name":"stop","description":"Emergency stop. Only the operator can reset it.","inputSchema":empty}
    ])
}

const WRITE_TOOLS: &[&str] = &["go_to", "ask_operator", "finish", "request_drive", "drive"];

/// Rules for the on-phone agent model.
fn agent_system_prompt() -> String {
    let mut text = String::from(
        "You control Terra, a small ground rover, to do the operator's task. You act only by calling tools.\n\
Coordinates are metres in the rover's map frame; bearings are relative to the rover, +90 is left.\n\
Use get_map before go_to and prefer its ids. Use wait while the rover drives. Ask the operator when the task is unclear. \
Call finish when the task is done or cannot be done.\n\n\
Reply with exactly one tool call and nothing else, in this form:\n\
<tool_call>{\"name\": \"get_map\", \"arguments\": {}}</tool_call>\n\nTOOLS:\n",
    );
    for tool in tools().as_array().into_iter().flatten() {
        text.push_str(&format!(
            "- {}: {} Arguments: {}\n",
            tool["name"].as_str().unwrap_or_default(),
            tool["description"].as_str().unwrap_or_default(),
            tool["inputSchema"]["properties"]
        ));
    }
    text
}

fn text(body: impl Into<String>) -> Value {
    json!({"content":[{"type":"text","text":body.into()}],"isError":false})
}
fn error(body: impl Into<String>) -> Value {
    json!({"content":[{"type":"text","text":body.into()}],"isError":true})
}

fn base64(bytes: &[u8]) -> String {
    const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let n = chunk
            .iter()
            .enumerate()
            .fold(0u32, |n, (i, b)| n | (u32::from(*b) << (16 - 8 * i)));
        for i in 0..4 {
            if i <= chunk.len() {
                out.push(ALPHABET[(n >> (18 - 6 * i) & 63) as usize] as char);
            } else {
                out.push('=');
            }
        }
    }
    out
}

fn waypoints(args: &Value) -> Result<Vec<RequestedWaypoint>, String> {
    let list = args["waypoints"]
        .as_array()
        .ok_or("waypoints must be a list")?;
    list.iter()
        .map(|w| {
            if let Some(id) = w["target"].as_str().or(w.as_str()) {
                Ok(RequestedWaypoint::Target(id.trim().to_ascii_uppercase()))
            } else if let (Some(x), Some(y)) = (w["x"].as_f64(), w["y"].as_f64()) {
                Ok(RequestedWaypoint::Point(x, y))
            } else {
                Err("each waypoint needs \"target\" or \"x\" and \"y\"".to_string())
            }
        })
        .collect()
}

impl MobileController {
    fn snapshot(&self) -> Snapshot {
        let Ok(b) = self.brain.lock() else {
            return Snapshot {
                phase: None,
                drive_request: false,
                drive_grant: false,
                history: 0,
                last: None,
            };
        };
        let status = b.autonomy.task_status(b.last_tick);
        Snapshot {
            phase: status.as_ref().map(|s| s.phase),
            drive_request: status.as_ref().is_some_and(|s| s.drive_request.is_some()),
            drive_grant: status
                .as_ref()
                .is_some_and(|s| s.drive_remaining_seconds.is_some()),
            history: status.as_ref().map_or(0, |s| s.history.len()),
            last: status.and_then(|s| s.history.last().map(|h| h.text.clone())),
        }
    }

    /// Run one tool. Returns an MCP `tools/call` result.
    fn call_tool(&self, session: &str, name: &str, args: &Value) -> Value {
        if let Ok(mut m) = self.mcp.lock() {
            let now = Instant::now();
            let calls = m.calls.entry(session.to_owned()).or_default();
            while calls
                .front()
                .is_some_and(|t| now.duration_since(*t) > Duration::from_secs(1))
            {
                calls.pop_front();
            }
            if calls.len() >= CALLS_PER_SECOND {
                return error("rate_limited: at most 10 calls per second");
            }
            calls.push_back(now);
            if name == "go_to" {
                if m.last_go
                    .get(session)
                    .is_some_and(|t| now.duration_since(*t) < GO_TO_INTERVAL)
                {
                    return error("rate_limited: at most one go_to per second");
                }
                m.last_go.insert(session.to_owned(), now);
            }
        }
        if name == "wait" {
            return self.wait_tool(args);
        }
        if name == "look" {
            return self.look_tool(session);
        }
        let Ok(mut b) = self.brain.lock() else {
            return error("internal");
        };
        let now = b.last_tick;
        if WRITE_TOOLS.contains(&name) {
            let Some(task_id) = b.autonomy.agent_task_id().map(str::to_owned) else {
                return error(
                    "not_in_control: ARGOS has not started a task for an MCP agent (level task, agent mcp)",
                );
            };
            let Ok(mut m) = self.mcp.lock() else {
                return error("internal");
            };
            match &m.controller {
                Some((id, owner)) if id == &task_id && owner != session => {
                    return error("not_in_control: another agent session controls this task");
                }
                Some((id, _)) if id == &task_id => {}
                _ => m.controller = Some((task_id, session.to_owned())),
            }
        }
        let pose = b.pose.map(|p| p.0).unwrap_or_default();
        let map = b.map.as_ref().map(|(m, t, _)| (m.clone(), *t));
        let map = map.as_ref().map(|(m, t)| (m, *t));
        let result = match name {
            "get_state" => {
                let brief: Value = serde_json::from_str(&b.autonomy_brief).unwrap_or_default();
                let task = b.autonomy.task_status(now);
                Ok(text(
                    json!({
                        "pose": {"x": pose.x, "y": pose.y, "yaw_deg": pose.yaw.to_degrees()},
                        "requested_level": brief["status"]["requested_level"],
                        "effective_level": brief["status"]["effective_level"],
                        "safety": brief["status"]["safety"],
                        "reason": brief["status"]["reason"],
                        "task_phase": task.as_ref().map(|t| t.phase),
                        "task_agent": task.as_ref().map(|t| t.agent),
                    })
                    .to_string(),
                ))
            }
            "get_task" => Ok(match b.autonomy.task_status(now) {
                Some(status) => text(serde_json::to_string(&status).unwrap_or_default()),
                None => error("no_task: ARGOS has not started a task"),
            }),
            "get_map" => b
                .autonomy
                .agent_view(map, pose, now)
                .map(text)
                .map_err(str::to_owned),
            "get_detections" => {
                let objects = b.objects.objects().to_vec();
                Ok(text(json!({"objects": objects}).to_string()))
            }
            "go_to" => waypoints(args)
                .and_then(|w| {
                    b.autonomy.agent_go(
                        &w,
                        args["rationale"].as_str().map(str::to_owned),
                        map,
                        pose,
                        now,
                    )
                })
                .map(|o| self.outcome(o)),
            "ask_operator" => b
                .autonomy
                .agent_ask(args["question"].as_str().unwrap_or_default())
                .map(|o| self.outcome(o)),
            "finish" => match args["status"].as_str() {
                Some(s @ ("done" | "fail")) => b
                    .autonomy
                    .agent_finish(
                        s == "done",
                        args["rationale"].as_str().unwrap_or_default(),
                        now,
                    )
                    .map(|o| self.outcome(o)),
                _ => Err("status must be done or fail".into()),
            },
            "request_drive" => b
                .autonomy
                .agent_request_drive(
                    args["reason"].as_str().unwrap_or_default(),
                    args["seconds"].as_f64().unwrap_or(f64::NAN),
                    now,
                )
                .map(|o| self.outcome(o)),
            "drive" => b
                .autonomy
                .agent_drive(
                    args["linear"].as_f64().unwrap_or(f64::NAN),
                    args["angular"].as_f64().unwrap_or(f64::NAN),
                    args["seconds"]
                        .as_f64()
                        .unwrap_or(terra_autonomy::TASK_DRIVE_COMMAND_SECONDS),
                    now,
                )
                .map(|o| self.outcome(o)),
            "stop" => {
                let token = self
                    .mcp
                    .lock()
                    .map(|mut m| {
                        m.next_stop += 1;
                        format!("agent-stop-{}", m.next_stop)
                    })
                    .unwrap_or_else(|_| "agent-stop".into());
                b.autonomy.set_safety(
                    terra_autonomy::SafetyRequest {
                        action: terra_autonomy::SafetyAction::Stop,
                        token,
                    },
                    true,
                );
                Ok(text("stopped; only the operator can reset it"))
            }
            other => Err(format!("unknown tool {other}")),
        };
        let result = result.unwrap_or_else(error);
        if let Some(recorder) = b.recorder.as_ref() {
            let _ = recorder.record(json!({
                "kind": "task.agent_call",
                "session": session,
                "tool": name,
                "arguments": args,
                "error": result["isError"],
                "time": now - b.run_start.unwrap_or(now),
                "run_id": b.run_id,
            }));
        }
        result
    }

    fn outcome(&self, o: AgentOutcome) -> Value {
        text(match o {
            AgentOutcome::Following(plan) => format!(
                "accepted; driving {}. Call wait to follow progress.",
                plan.iter()
                    .map(|w| format!("{} ({:.1}, {:.1})", w.label, w.x, w.y))
                    .collect::<Vec<_>>()
                    .join(" then ")
            ),
            AgentOutcome::Asked => "asked; the rover holds until the operator answers".into(),
            AgentOutcome::Ended(reason) => format!("task ended: {reason}"),
            AgentOutcome::DriveRequested => {
                "requested; drive works only after the operator approves".into()
            }
            AgentOutcome::Driving(_) => "driving".into(),
        })
    }

    fn wait_tool(&self, args: &Value) -> Value {
        let seconds = args["seconds"]
            .as_f64()
            .filter(|s| s.is_finite() && *s >= 0.)
            .unwrap_or(WAIT_MAX_SECONDS)
            .min(WAIT_MAX_SECONDS);
        let start = self.snapshot();
        let deadline = Instant::now() + Duration::from_secs_f64(seconds);
        while Instant::now() < deadline && self.snapshot() == start {
            std::thread::sleep(Duration::from_millis(100));
        }
        let end = self.snapshot();
        text(format!(
            "{}; phase {}{}",
            if end == start { "no change" } else { "changed" },
            end.phase.map_or("none", |p| p.as_str()),
            end.last
                .map(|l| format!("; latest: {l}"))
                .unwrap_or_default()
        ))
    }

    fn look_tool(&self, session: &str) -> Value {
        let now = self.brain.lock().map(|b| b.last_tick).unwrap_or_default();
        let Ok(mut m) = self.mcp.lock() else {
            return error("internal");
        };
        if session == IN_PROCESS_SESSION {
            m.agent.attach_image = true;
            return text("the latest camera frame is attached to your next prompt");
        }
        match &m.frame {
            Some((jpeg, at)) if now - at <= LOOK_MAX_AGE_SECONDS => json!({
                "content":[{"type":"image","data":base64(jpeg),"mimeType":"image/jpeg"}],
                "isError":false
            }),
            _ => error("no recent camera frame"),
        }
    }
}

#[uniffi::export]
impl MobileController {
    /// Handle one MCP JSON-RPC message from `session` and return the response, or an
    /// empty string for a notification. Call off the control queue: `wait` can block for 30 s.
    pub fn mcp_handle(&self, session: String, request: String) -> String {
        let Ok(message) = serde_json::from_str::<Value>(&request) else {
            return json!({"jsonrpc":"2.0","id":null,"error":{"code":-32700,"message":"parse error"}})
                .to_string();
        };
        let Some(id) = message.get("id").cloned() else {
            return String::new();
        };
        let params = &message["params"];
        let result = match message["method"].as_str().unwrap_or_default() {
            "initialize" => Ok(json!({
                "protocolVersion": PROTOCOL_VERSION,
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "terra-rover", "version": env!("CARGO_PKG_VERSION")},
                "instructions": "Run the operator's L5 task. Start with get_task and get_map."
            })),
            "ping" => Ok(json!({})),
            "tools/list" => Ok(json!({"tools": tools()})),
            "tools/call" => Ok(self.call_tool(
                &session,
                params["name"].as_str().unwrap_or_default(),
                params.get("arguments").unwrap_or(&json!({})),
            )),
            _ => Err((-32601, "method not found")),
        };
        match result {
            Ok(result) => json!({"jsonrpc":"2.0","id":id,"result":result}),
            Err((code, msg)) => {
                json!({"jsonrpc":"2.0","id":id,"error":{"code":code,"message":msg}})
            }
        }
        .to_string()
    }

    /// Advertise (or withdraw with `None`) an MCP agent by name. `in_process` means the
    /// phone runs the agent model itself and answers [`Self::take_agent_prompt`].
    pub fn set_task_agent(
        &self,
        name: Option<String>,
        in_process: bool,
    ) -> Result<(), ControllerError> {
        let attached = name.is_some() && in_process;
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .autonomy
            .set_task_agent(name);
        self.mcp
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .agent
            .attached = attached;
        Ok(())
    }

    /// Latest camera JPEG for an MCP client's `look`. Only needed while a remote client is connected.
    pub fn set_look_frame(&self, jpeg: Vec<u8>, timestamp: f64) -> Result<(), ControllerError> {
        self.mcp
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .frame = Some((jpeg, timestamp));
        Ok(())
    }

    /// Next prompt for the on-phone agent, when its task needs a decision. Each prompt is returned once.
    pub fn take_agent_prompt(&self) -> Option<MobileAgentPrompt> {
        let snapshot = self.snapshot();
        let (task_id, view) = {
            let mut b = self.brain.lock().ok()?;
            let task_id = b.autonomy.agent_task_id()?.to_owned();
            let now = b.last_tick;
            let pose = b.pose.map(|p| p.0).unwrap_or_default();
            let map = b.map.as_ref().map(|(m, t, _)| (m.clone(), *t));
            let view = b
                .autonomy
                .agent_view(map.as_ref().map(|(m, t)| (m, *t)), pose, now)
                .ok()?;
            (task_id, view)
        };
        let mut m = self.mcp.lock().ok()?;
        if m.controller
            .as_ref()
            .is_some_and(|(id, owner)| id == &task_id && owner != IN_PROCESS_SESSION)
        {
            return None;
        }
        let a = &mut m.agent;
        if !a.attached || a.outstanding.is_some() {
            return None;
        }
        if a.task_id.as_deref() != Some(task_id.as_str()) {
            *a = InProcessAgent {
                attached: true,
                task_id: Some(task_id),
                next_prompt: a.next_prompt,
                ..Default::default()
            };
        }
        if let Some((waiting_on, until)) = &a.resume {
            let timed_out = until.is_some_and(|t| Instant::now() >= t);
            if *waiting_on == snapshot && !timed_out {
                return None;
            }
            a.resume = None;
        }
        a.next_prompt += 1;
        a.outstanding = Some(a.next_prompt);
        let mut prompt = view;
        if !a.transcript.is_empty() {
            prompt.push_str("\n\nYOUR RECENT TOOL CALLS (oldest first):\n");
            for line in &a.transcript {
                prompt.push_str(line);
                prompt.push('\n');
            }
        }
        prompt.push_str("\nCall exactly one tool.");
        let attach_image = std::mem::take(&mut a.attach_image);
        Some(MobileAgentPrompt {
            prompt_id: a.next_prompt,
            system: agent_system_prompt(),
            prompt,
            attach_image,
        })
    }

    /// Return the on-phone model's raw text for `prompt_id`. Rust parses the tool call,
    /// runs it like any MCP client's call, and decides when to prompt again.
    /// Returns false for an unknown prompt.
    pub fn submit_agent_reply(&self, prompt_id: u64, reply: String, latency_seconds: f64) -> bool {
        {
            let Ok(mut m) = self.mcp.lock() else {
                return false;
            };
            if m.agent.outstanding != Some(prompt_id) {
                return false;
            }
            m.agent.outstanding = None;
        }
        let call = parse_tool_call(&reply);
        let (line, resume) = match call {
            Err(why) => {
                let give_up = self
                    .mcp
                    .lock()
                    .map(|mut m| {
                        m.agent.bad_replies += 1;
                        m.agent.bad_replies >= BAD_REPLY_LIMIT
                    })
                    .unwrap_or(true);
                if give_up {
                    let _ = self.call_tool(
                        IN_PROCESS_SESSION,
                        "finish",
                        &json!({"status":"fail","rationale":"the on-phone agent sent 3 replies without a tool call"}),
                    );
                }
                (
                    format!("(no tool call: {why}; reply with one <tool_call>)"),
                    None,
                )
            }
            Ok((name, args)) => {
                if let Ok(mut m) = self.mcp.lock() {
                    m.agent.bad_replies = 0;
                }
                // The on-phone agent waits by not being prompted, so `wait` never blocks here.
                let before = self.snapshot();
                let result = if name == "wait" {
                    text("waiting")
                } else {
                    self.call_tool(IN_PROCESS_SESSION, &name, &args)
                };
                let ok = result["isError"] == false;
                let body = result["content"][0]["text"].as_str().unwrap_or_default();
                let body: String = body.chars().take(600).collect();
                let resume = match name.as_str() {
                    "wait" => Some(Some(
                        Instant::now() + Duration::from_secs_f64(WAIT_MAX_SECONDS),
                    )),
                    "go_to" | "ask_operator" | "request_drive" if ok => Some(None),
                    _ => None,
                };
                let line = format!(
                    "[{latency_seconds:.1} s] {name} {args} -> {}{body}",
                    if ok { "" } else { "ERROR " }
                );
                (
                    line,
                    resume.map(|until| {
                        (
                            if name == "wait" {
                                before
                            } else {
                                self.snapshot()
                            },
                            until,
                        )
                    }),
                )
            }
        };
        if let Ok(mut m) = self.mcp.lock() {
            let a = &mut m.agent;
            a.transcript.push_back(line);
            while a.transcript.len() > TRANSCRIPT_LIMIT {
                a.transcript.pop_front();
            }
            a.resume = resume;
        }
        true
    }

    /// Report that the on-phone model could not answer `prompt_id`. It is prompted again.
    pub fn fail_agent_prompt(&self, prompt_id: u64, reason: String) -> bool {
        let Ok(mut m) = self.mcp.lock() else {
            return false;
        };
        if m.agent.outstanding != Some(prompt_id) {
            return false;
        }
        m.agent.outstanding = None;
        let line: String = format!("(model error: {reason})")
            .chars()
            .take(200)
            .collect();
        m.agent.transcript.push_back(line);
        while m.agent.transcript.len() > TRANSCRIPT_LIMIT {
            m.agent.transcript.pop_front();
        }
        true
    }
}

/// The tool call in a model reply: `<tool_call>{"name":..,"arguments":{..}}</tool_call>`,
/// or the first JSON object that names a tool.
fn parse_tool_call(text: &str) -> Result<(String, Value), String> {
    let from = text.find("<tool_call>").map_or(text, |i| &text[i..]);
    let object = terra_autonomy::extract_json_object(from).ok_or("no JSON object")?;
    let value: Value = serde_json::from_str(object).map_err(|e| format!("invalid JSON: {e}"))?;
    let name = value["name"]
        .as_str()
        .ok_or("the tool call needs a \"name\"")?
        .trim()
        .to_owned();
    if !tools()
        .as_array()
        .is_some_and(|t| t.iter().any(|t| t["name"] == name.as_str()))
    {
        return Err(format!(
            "unknown tool {}",
            name.chars().take(32).collect::<String>()
        ));
    }
    let args = match value.get("arguments") {
        Some(Value::String(s)) => serde_json::from_str(s).unwrap_or(json!({})),
        Some(v @ Value::Object(_)) => v.clone(),
        _ => json!({}),
    };
    Ok((name, args))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base64_matches_rfc_4648_vectors() {
        assert_eq!(base64(b""), "");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foo"), "Zm9v");
        assert_eq!(base64(b"foobar"), "Zm9vYmFy");
    }

    #[test]
    fn tool_calls_parse_from_qwen_tags_or_bare_json() {
        let (name, args) = parse_tool_call(
            "<think>near F1</think>\n<tool_call>\n{\"name\": \"go_to\", \"arguments\": {\"waypoints\": [{\"target\": \"F1\"}]}}\n</tool_call>",
        )
        .unwrap();
        assert_eq!(name, "go_to");
        assert_eq!(args["waypoints"][0]["target"], "F1");
        let (name, args) = parse_tool_call(r#"{"name":"get_map","arguments":"{}"}"#).unwrap();
        assert_eq!((name.as_str(), args), ("get_map", json!({})));
        assert!(parse_tool_call("I will go to F1.").is_err());
        assert!(parse_tool_call(r#"{"name":"set_motor","arguments":{}}"#).is_err());
    }
}
