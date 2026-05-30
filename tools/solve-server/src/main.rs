//! solve-server: min2phase HTTP API server
//!
//! 求解路径：`robo_pipeline::translate_optimal`，即 `robo_solver::search::Search`
//! 多候选 + handstep 翻译择 mech 最短。**不再使用 Search2L**：
//! `/v1/solve` 和 `/v1/solve2l` 现在行为等价（都走 Search 路径），
//! 后者保留是为了向后兼容旧 API 客户端。
//!
//! 端点：GET /v1/health, POST /v1/verify, POST /v1/solve, POST /v1/solve2l
//!
//! Usage:
//!   solve-server [--host 127.0.0.1] [--port 8080]

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use robo_core::CubeFace;
use robo_pipeline::multi::translate_optimal;
use robo_solver::search::{Search, SearchOptions};
use serde::{Deserialize, Serialize};
use tiny_http::{Header, Method, Request, Response, Server};

// ===== State =====

struct AppState {
    /// 求解表是否已初始化完成。pipeline 是 stateless，只需要 Search::init()
    /// 跑过一次即可（内部 OnceLock 全局表）。
    ready: AtomicBool,
}

// ===== Request/Response DTOs =====

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct VerifyRequest {
    facelets: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SolveRequest {
    facelets: Option<String>,
    scramble: Option<String>,
    #[allow(dead_code)]
    options: Option<SolveOptions>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
#[allow(dead_code)]
struct SolveOptions {
    max_depth: Option<i32>,
    probe_max: Option<i64>,
    probe_min: Option<i64>,
}

#[derive(Serialize)]
struct HealthResponse {
    status: &'static str,
    ready: bool,
    tables: TablesStatus,
}

#[derive(Serialize)]
struct TablesStatus {
    search: bool,
    search2l: bool,
}

#[derive(Serialize)]
struct VerifyResponse {
    ok: bool,
    status: &'static str,
    verify: VerifyDetail,
}

#[derive(Serialize)]
struct VerifyDetail {
    code: i32,
    name: &'static str,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct SolveResponse {
    ok: bool,
    status: &'static str,
    /// Kociemba face 序列字符串（如 "R2 F' D2 ..."）
    #[serde(skip_serializing_if = "Option::is_none")]
    solution: Option<String>,
    /// face 数量（kociemba 长度）
    length: i32,
    #[serde(skip_serializing_if = "Option::is_none")]
    message: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    facelets: Option<String>,
    /// 机械步骤 mnemonic 列表（M_L1 / M_R3 / ...）
    #[serde(skip_serializing_if = "Option::is_none")]
    hardware_commands: Option<Vec<String>>,
    /// 机械步骤数字编码（默认 digit map）
    #[serde(skip_serializing_if = "Option::is_none")]
    encoded_steps: Option<String>,
    /// 最终选中候选的机械步数（pipeline 已最小化）
    #[serde(skip_serializing_if = "Option::is_none")]
    mech_steps: Option<i32>,
    /// solver 阶段耗时（毫秒）
    #[serde(skip_serializing_if = "Option::is_none")]
    search_elapsed_ms: Option<u64>,
    /// solver 产出的候选数量
    #[serde(skip_serializing_if = "Option::is_none")]
    candidate_count: Option<usize>,
}

#[derive(Serialize)]
struct ErrorResponse {
    error: ErrorDetail,
}

#[derive(Serialize)]
struct ErrorDetail {
    code: &'static str,
    message: String,
}

// ===== Main =====

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp_millis()
        .init();

    let args: Vec<String> = std::env::args().collect();
    let host = get_arg(&args, "--host").unwrap_or_else(|| "127.0.0.1".to_string());
    let port = get_arg(&args, "--port").unwrap_or_else(|| "8080".to_string());
    let bind = format!("{host}:{port}");

    let state = Arc::new(AppState {
        ready: AtomicBool::new(false),
    });

    // Init solver tables in background
    let state_init = Arc::clone(&state);
    thread::spawn(move || {
        log::info!("正在初始化 solver 表...");
        let t0 = Instant::now();
        Search::init();
        state_init.ready.store(true, Ordering::Release);
        log::info!("Solver 初始化完成 ({:.2}s)", t0.elapsed().as_secs_f64());
    });

    let server = Server::http(&bind).unwrap_or_else(|e| {
        eprintln!("无法绑定 {bind}: {e}");
        std::process::exit(1);
    });
    log::info!("CubeSolver HTTP API listening on http://{bind}");
    log::info!("Endpoints: GET /v1/health, POST /v1/verify, POST /v1/solve, POST /v1/solve2l");

    for request in server.incoming_requests() {
        let state = Arc::clone(&state);
        thread::spawn(move || {
            handle_request(request, &state);
        });
    }
}

fn handle_request(request: Request, state: &AppState) {
    let method = request.method().clone();
    let url = request.url().to_string();

    // CORS preflight
    if matches!(method, Method::Options) {
        let response = Response::empty(204)
            .with_header(Header::from_bytes("Access-Control-Allow-Origin", "*").unwrap())
            .with_header(Header::from_bytes("Access-Control-Allow-Methods", "GET, POST, OPTIONS").unwrap())
            .with_header(Header::from_bytes("Access-Control-Allow-Headers", "Content-Type").unwrap());
        let _ = request.respond(response);
        return;
    }

    match (method, url.as_str()) {
        (Method::Get, "/v1/health") => handle_health(request, state),
        (Method::Post, "/v1/verify") => handle_verify(request, state),
        (Method::Post, "/v1/solve") => handle_solve(request, state, false),
        (Method::Post, "/v1/solve2l") => handle_solve(request, state, true),
        (_, path) if path.starts_with("/v1/") => {
            if matches!(request.method(), &Method::Get | &Method::Post) {
                respond_error(request, 404, "not_found", "endpoint not found");
            } else {
                respond_error(request, 405, "method_not_allowed", "method not allowed");
            }
        }
        _ => respond_error(request, 404, "not_found", "endpoint not found"),
    }
}

// ===== Handlers =====

fn handle_health(request: Request, state: &AppState) {
    let ready = state.ready.load(Ordering::Acquire);
    let resp = HealthResponse {
        status: if ready { "ready" } else { "starting" },
        ready,
        tables: TablesStatus {
            search: ready,
            // 已不再使用 Search2L；保留字段为 false 让旧 client 看到改动而不崩溃
            search2l: false,
        },
    };
    respond_json(request, 200, &resp);
}

fn handle_verify(mut request: Request, state: &AppState) {
    if !state.ready.load(Ordering::Acquire) {
        respond_error(request, 503, "not_ready", "solver tables are not ready yet");
        return;
    }

    let mut body = String::new();
    if request.as_reader().read_to_string(&mut body).is_err() {
        respond_error(request, 400, "bad_request", "failed to read request body");
        return;
    }

    let req: VerifyRequest = match serde_json::from_str(&body) {
        Ok(r) => r,
        Err(e) => {
            respond_error(request, 400, "bad_request", &format!("invalid JSON: {e}"));
            return;
        }
    };

    match CubeFace::new(req.facelets) {
        Ok(_) => {
            let resp = VerifyResponse {
                ok: true,
                status: "ok",
                verify: VerifyDetail { code: 0, name: "ok" },
            };
            respond_json(request, 200, &resp);
        }
        Err(e) => {
            let msg = format!("{e}");
            let (code, name) = classify_verify_error(&msg);
            let resp = VerifyResponse {
                ok: false,
                status: name,
                verify: VerifyDetail { code, name },
            };
            respond_json(request, 200, &resp);
        }
    }
}

fn classify_verify_error(msg: &str) -> (i32, &'static str) {
    if msg.contains("color") || msg.contains("count") || msg.contains("54") || msg.contains("character") {
        (1, "invalid_color_count")
    } else if msg.contains("edge") {
        (2, "invalid_edge")
    } else if msg.contains("flip") {
        (3, "invalid_flip")
    } else if msg.contains("corner") {
        (4, "invalid_corner")
    } else if msg.contains("twist") {
        (5, "invalid_twist")
    } else if msg.contains("parity") {
        (6, "invalid_parity")
    } else {
        (1, "invalid_color_count")
    }
}

fn handle_solve(mut request: Request, state: &AppState, _use_2l: bool) {
    if !state.ready.load(Ordering::Acquire) {
        respond_error(request, 503, "not_ready", "solver tables are not ready yet");
        return;
    }

    // Read body
    let mut body = String::new();
    if request.as_reader().read_to_string(&mut body).is_err() {
        respond_error(request, 400, "bad_request", "failed to read request body");
        return;
    }

    let req: SolveRequest = match serde_json::from_str(&body) {
        Ok(r) => r,
        Err(e) => {
            respond_error(request, 400, "bad_request", &format!("invalid JSON: {e}"));
            return;
        }
    };

    // Validate: exactly one of facelets or scramble
    let facelets = match (&req.facelets, &req.scramble) {
        (Some(f), None) => f.clone(),
        (None, Some(s)) => match scramble_to_facelets(s) {
            Ok(f) => f,
            Err(e) => {
                respond_error(request, 400, "bad_request", &e);
                return;
            }
        },
        (Some(_), Some(_)) => {
            respond_error(request, 400, "bad_request", "exactly one of facelets or scramble is required");
            return;
        }
        (None, None) => {
            respond_error(request, 400, "bad_request", "exactly one of facelets or scramble is required");
            return;
        }
    };

    // Validate facelets
    let face = match CubeFace::new(facelets.clone()) {
        Ok(f) => f,
        Err(e) => {
            let resp = SolveResponse {
                ok: false,
                status: "invalid_cube",
                solution: None,
                length: -1,
                message: Some(format!("{e}")),
                facelets: Some(facelets),
                hardware_commands: None,
                encoded_steps: None,
                mech_steps: None,
                search_elapsed_ms: None,
                candidate_count: None,
            };
            respond_json(request, 200, &resp);
            return;
        }
    };

    // 求解：100ms / max=∞ / slack=0（实测 mech 平均 68.9，与 300ms 旧基线持平，
    // solver 100ms 严格用满）。SolveOptions 字段（max_depth/probe_*）暂不暴露
    // 给客户端覆盖，行为统一可控。
    let opts = SearchOptions {
        timeout: Duration::from_millis(100),
        max_solutions: usize::MAX,
        length_slack: 0,
        ..Default::default()
    };

    match translate_optimal(face.as_str(), opts) {
        Ok(res) => {
            let solution_str = res.best.kociemba.clone();
            let length = solution_str.split_whitespace().count() as i32;

            // mech_encoded 用 handstep 默认 digit map（与 BasicTranslator 默认一致）
            // 反查得到 mnemonic 列表
            let hardware_commands = decode_to_mnemonics(&res.best.mech_encoded);

            let resp = SolveResponse {
                ok: true,
                status: "ok",
                solution: Some(solution_str),
                length,
                message: None,
                facelets: Some(facelets),
                hardware_commands: Some(hardware_commands),
                encoded_steps: Some(res.best.mech_encoded.clone()),
                mech_steps: Some(res.best.mech_steps),
                search_elapsed_ms: Some(res.solver_elapsed.as_millis() as u64),
                candidate_count: Some(res.candidates.len()),
            };
            respond_json(request, 200, &resp);
        }
        Err(e) => {
            let resp = SolveResponse {
                ok: false,
                status: "no_solution",
                solution: None,
                length: -1,
                message: Some(format!("{e}")),
                facelets: Some(facelets),
                hardware_commands: None,
                encoded_steps: None,
                mech_steps: None,
                search_elapsed_ms: None,
                candidate_count: None,
            };
            respond_json(request, 200, &resp);
        }
    }
}

/// 把 handstep 输出的"默认 digit map 编码字符串"反查成 mnemonic 列表。
///
/// handstep `MOVE_STR = ["4","3","2","0","1","9","8","7","5","6"]` 与
/// 助记符 `["M_L1","M_L2","M_L3","M_LC","M_LO","M_R1","M_R2","M_R3","M_RC","M_RO"]`
/// 一一对应（顺序固定，无需引入 robo-translator）。
const HARDWARE_MNEMONICS: [&str; 10] = [
    "M_L1", "M_L2", "M_L3", "M_LC", "M_LO", "M_R1", "M_R2", "M_R3", "M_RC", "M_RO",
];
const HARDWARE_DIGITS: [char; 10] = ['4', '3', '2', '0', '1', '9', '8', '7', '5', '6'];

fn decode_to_mnemonics(encoded: &str) -> Vec<String> {
    encoded
        .chars()
        .filter(|c| !c.is_whitespace())
        .filter_map(|c| HARDWARE_DIGITS.iter().position(|d| *d == c))
        .map(|i| HARDWARE_MNEMONICS[i].to_string())
        .collect()
}

// ===== Helpers =====

fn scramble_to_facelets(scramble: &str) -> Result<String, String> {
    // Apply scramble moves to solved cube
    // For now, use the Search standard solver's from_scramble equivalent
    // We'll just validate and pass through — the solver handles it via CubeFace
    // Actually, we need to simulate the scramble on a solved cube
    // This is a simplified implementation that just returns an error for now
    // if the scramble contains invalid moves
    if scramble.trim().is_empty() {
        return Ok("UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB".to_string());
    }
    Err(format!("scramble parsing not yet implemented: '{scramble}'. Please use facelets instead."))
}

fn respond_json<T: Serialize>(request: Request, status: u16, body: &T) {
    let json = serde_json::to_string(body).unwrap_or_else(|_| "{}".to_string());
    let response = Response::from_string(&json)
        .with_status_code(status)
        .with_header(
            Header::from_bytes("Content-Type", "application/json; charset=utf-8").unwrap(),
        )
        .with_header(Header::from_bytes("Access-Control-Allow-Origin", "*").unwrap())
        .with_header(Header::from_bytes("Access-Control-Allow-Methods", "GET, POST, OPTIONS").unwrap())
        .with_header(Header::from_bytes("Access-Control-Allow-Headers", "Content-Type").unwrap());
    let _ = request.respond(response);
}

fn respond_error(request: Request, status: u16, code: &'static str, message: &str) {
    let body = ErrorResponse {
        error: ErrorDetail {
            code,
            message: message.to_string(),
        },
    };
    respond_json(request, status, &body);
}

fn get_arg(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|a| a == flag)
        .and_then(|i| args.get(i + 1))
        .cloned()
}
