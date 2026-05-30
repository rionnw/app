//! 端到端"多候选并行择优"编排。
//!
//! 流程：
//!   cube facelets
//!     → robo_solver::Search.solutions() （限时 + 限量产 N 条 Kociemba 串）
//!     → 并行 robo_handstep::Engine.search() （N 条翻译，rayon par_iter）
//!     → 取机械步数最少的胜出
//!     → 返回最终机械序列字符串
//!
//! handstep 不限时（实测一条 ~1 ms），多候选总开销小。
//! 主要瓶颈在 solver；超时只作用于 solver 段。

use std::time::{Duration, Instant};

use rayon::prelude::*;

use robo_solver::search::{Search, SearchOptions, SolverResult};
use robo_handstep::Engine as HandstepEngine;

/// 单个候选的端到端结果（用于诊断/日志）。
#[derive(Clone, Debug)]
pub struct CandidateResult {
    /// 人手记号（Kociemba 风格 "F R2 U' ..."）
    pub kociemba: String,
    /// 翻译后的机械步骤数（mech_steps，越小越优）
    pub mech_steps: i32,
    /// 机械序列编码字符串（与 robo-handstep::Engine::get_steps() 输出一致）
    pub mech_encoded: String,
}

/// 端到端多候选择优结果。
#[derive(Clone, Debug)]
pub struct OptimalResult {
    /// 所有候选的 (kociemba, mech) 对，已按 mech_steps 升序排序
    pub candidates: Vec<CandidateResult>,
    /// 最优候选索引（candidates[0]）
    pub best: CandidateResult,
    /// solver 阶段耗时
    pub solver_elapsed: Duration,
    /// solver 是否超时
    pub solver_timed_out: bool,
    /// handstep 阶段总耗时（并行）
    pub handstep_elapsed: Duration,
}

/// 端到端：cube facelets → 候选 Kociemba 解 → 并行翻译 → 选机械最短。
///
/// 注意 `Search::init()` 应在调用前已经执行（一次即可，可在程序启动时调）。
/// `HandstepEngine::new()` 在每个 worker 内独立创建（~500 ns，可忽略）。
pub fn translate_optimal(
    facelets: &str,
    solver_opts: SearchOptions,
) -> anyhow::Result<OptimalResult> {
    // === Stage 1: solver 拿候选 ===
    let mut search = Search::new();
    let SolverResult { solutions, elapsed: solver_elapsed, timed_out: solver_timed_out } =
        search.solutions(facelets, solver_opts);

    if solutions.is_empty() {
        anyhow::bail!(
            "solver 未在 {:?} 内产出任何候选解（timed_out={}）",
            solver_opts.timeout, solver_timed_out
        );
    }

    // === Stage 2: 并行翻译（每个 worker 自己的 Engine 实例）===
    let handstep_start = Instant::now();
    let mut candidates: Vec<CandidateResult> = solutions
        .par_iter()
        .map(|kociemba| {
            let robotstep = kociemba_to_robotstep(kociemba);
            let mut engine = HandstepEngine::new();
            let mech_steps = engine.search(&robotstep);
            let mech_encoded = engine.get_steps();
            CandidateResult {
                kociemba: kociemba.clone(),
                mech_steps,
                mech_encoded,
            }
        })
        .collect();
    let handstep_elapsed = handstep_start.elapsed();

    // === Stage 3: 按 mech_steps 升序排，取最小 ===
    candidates.sort_by_key(|c| c.mech_steps);
    let best = candidates[0].clone();

    Ok(OptimalResult {
        candidates,
        best,
        solver_elapsed,
        solver_timed_out,
        handstep_elapsed,
    })
}

/// 把标准 Kociemba 记号（`F / F2 / F'`）转换成 RobotStep 内部格式（`F1 / F2 / F3 `）。
///
/// 注：robo-handstep 内部测试已有同名函数，但当前未公开为 `pub fn`；这里独立实现
/// 一份避免跨 crate 的可见性问题。逻辑与 handstep 内部一致。
fn kociemba_to_robotstep(kociemba: &str) -> String {
    let mut out = String::with_capacity(kociemba.len());
    for token in kociemba.split_whitespace() {
        let mut chars = token.chars();
        let face = match chars.next() {
            Some(c) if "FRUBLD".contains(c) => c,
            _ => continue,
        };
        let suffix: String = chars.collect();
        let dist_char = match suffix.as_str() {
            "" => '1',  // F   → +90 → _1
            "2" => '2', // F2  → 180 → _2
            "'" => '3', // F'  → -90 → _3
            _ => continue,
        };
        out.push(face);
        out.push(dist_char);
        out.push(' ');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn end_to_end_finds_min_mech() {
        Search::init();
        // 同 search.rs 测试用例
        let cube = "UDFUURRLDBFLURRDRUUFLLFRFDBRBRLDBUDLRBBFLBBUDDFFDBUFLL";
        let res = translate_optimal(cube, SearchOptions {
            timeout: Duration::from_millis(500),
            max_solutions: 5,
            ..Default::default()
        }).expect("应当至少产出 1 条候选");

        eprintln!(
            "solver: {} 候选, {:?} {}; handstep: {:?}",
            res.candidates.len(),
            res.solver_elapsed,
            if res.solver_timed_out { "(timeout)" } else { "" },
            res.handstep_elapsed
        );
        for (i, c) in res.candidates.iter().enumerate() {
            let face_count = c.kociemba.split_whitespace().count();
            eprintln!(
                "  [{}] {}f → mech={} | {}",
                i, face_count, c.mech_steps, c.mech_encoded
            );
        }
        eprintln!("best: mech={}, kociemba={}", res.best.mech_steps, res.best.kociemba);

        // 候选必须按 mech_steps 升序
        for w in res.candidates.windows(2) {
            assert!(w[0].mech_steps <= w[1].mech_steps);
        }
        // best 是 candidates[0]
        assert_eq!(res.best.mech_steps, res.candidates[0].mech_steps);
    }

    #[test]
    fn kociemba_translation_basic() {
        assert_eq!(kociemba_to_robotstep("F R' U2"), "F1 R3 U2 ");
        assert_eq!(kociemba_to_robotstep(""), "");
    }
}
