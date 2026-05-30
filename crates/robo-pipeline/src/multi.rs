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
    /// 机械步骤助记符列表（如 `["M_L1", "M_LO", ...]`），来自
    /// `robo-handstep::Engine::get_mnemonics()`。最终发到下位机时，
    /// 由 transport 层用 user `digit_map` 编码成具体数字字符。
    pub mech_mnemonics: Vec<&'static str>,
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
            let mech_mnemonics = engine.get_mnemonics();
            CandidateResult {
                kociemba: kociemba.clone(),
                mech_steps,
                mech_mnemonics,
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
                "  [{}] {}f → mech={} | {} mnemonics",
                i, face_count, c.mech_steps, c.mech_mnemonics.len()
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

    /// 加载 bench.cubes 前 N 行
    fn load_bench_cubes(n: usize) -> Vec<String> {
        use std::path::Path;
        let candidates = [
            "../../RobotApp/rob-twophase/bench.cubes",
            "RobotApp/rob-twophase/bench.cubes",
        ];
        let path = candidates.iter()
            .find(|p| Path::new(p).exists())
            .copied()
            .expect("bench.cubes 找不到");
        let content = std::fs::read_to_string(path).expect("读 bench.cubes 失败");
        content.lines()
            .filter(|l| l.len() == 54)
            .take(n)
            .map(|s| s.to_string())
            .collect()
    }

    /// 通用 A/B 对比框架。
    fn ab_compare(label_a: &str, opts_a: SearchOptions,
                  label_b: &str, opts_b: SearchOptions,
                  n: usize) {
        use robo_solver::search::Search;
        Search::init();

        let cubes = load_bench_cubes(n);
        eprintln!("\n=== A: {}  vs  B: {}  ({} cubes) ===", label_a, label_b, cubes.len());
        eprintln!("  A: timeout={:?}, max_sol={}, slack={}",
            opts_a.timeout, opts_a.max_solutions, opts_a.length_slack);
        eprintln!("  B: timeout={:?}, max_sol={}, slack={}",
            opts_b.timeout, opts_b.max_solutions, opts_b.length_slack);

        let mut wins = 0i32;
        let mut ties = 0i32;
        let mut regressions = 0i32;
        let mut total_a_mech = 0i64;
        let mut total_b_mech = 0i64;
        let mut total_a_cands = 0i64;
        let mut total_b_cands = 0i64;
        let mut max_improvement = 0i32;
        let mut sample_idx_at_max: usize = 0;
        let mut a_solver_total = Duration::ZERO;
        let mut b_solver_total = Duration::ZERO;

        for (i, cube) in cubes.iter().enumerate() {
            let r_a = match translate_optimal(cube, opts_a) {
                Ok(r) => r,
                Err(e) => { eprintln!("  [{}] A 失败: {}", i, e); continue; }
            };
            let r_b = match translate_optimal(cube, opts_b) {
                Ok(r) => r,
                Err(e) => { eprintln!("  [{}] B 失败: {}", i, e); continue; }
            };
            let a = r_a.best.mech_steps;
            let b = r_b.best.mech_steps;
            total_a_mech += a as i64;
            total_b_mech += b as i64;
            total_a_cands += r_a.candidates.len() as i64;
            total_b_cands += r_b.candidates.len() as i64;
            a_solver_total += r_a.solver_elapsed;
            b_solver_total += r_b.solver_elapsed;

            let b_face_best = r_b.best.kociemba.split_whitespace().count();
            let b_face_min = r_b.candidates.iter()
                .map(|c| c.kociemba.split_whitespace().count())
                .min().unwrap_or(0);

            if b < a {
                wins += 1;
                if a - b > max_improvement {
                    max_improvement = a - b;
                    sample_idx_at_max = i;
                }
                eprintln!(
                    "  [{:2}] mech A→B: {} → {} (-{}); B 候选={}, best={}f, min_face={}f",
                    i, a, b, a - b, r_b.candidates.len(), b_face_best, b_face_min
                );
            } else if b == a {
                ties += 1;
            } else {
                regressions += 1;
                eprintln!("  [{:2}] !! 回归: {} → {} (+{}); A best={}f, B best={}f",
                    i, a, b, b - a,
                    r_a.best.kociemba.split_whitespace().count(),
                    b_face_best);
            }
        }

        let total = wins + ties + regressions;
        eprintln!("\n  --- {} cube 汇总 ---", total);
        eprintln!("  B 优于 A:    {} ({:.0}%)", wins, 100.0 * wins as f64 / total as f64);
        eprintln!("  持平:        {} ({:.0}%)", ties, 100.0 * ties as f64 / total as f64);
        eprintln!("  B 劣于 A:    {} ({:.0}%)", regressions, 100.0 * regressions as f64 / total as f64);
        eprintln!("  A mech 平均: {:.1}", total_a_mech as f64 / total as f64);
        eprintln!("  B mech 平均: {:.1}  (差 {:+.1})",
            total_b_mech as f64 / total as f64,
            (total_b_mech - total_a_mech) as f64 / total as f64);
        eprintln!("  A 平均候选数: {:.1}", total_a_cands as f64 / total as f64);
        eprintln!("  B 平均候选数: {:.1}", total_b_cands as f64 / total as f64);
        eprintln!("  A solver 平均耗时: {:?}", a_solver_total / total as u32);
        eprintln!("  B solver 平均耗时: {:?}", b_solver_total / total as u32);
        eprintln!("  最大单 cube 提升: {} mech (cube #{})", max_improvement, sample_idx_at_max);
    }

    /// 策略有效性 stress（旧版默认参数：300ms / 8 条 / slack=0 vs 仅首搜）。
    #[test]
    #[ignore]
    fn urf_diversity_helps_stress() {
        use robo_solver::search::Search;
        use std::path::Path;

        Search::init();

        // 找 bench.cubes（仓库相对路径，从 crate 目录回退两级）
        let candidates = [
            "../../RobotApp/rob-twophase/bench.cubes",
            "RobotApp/rob-twophase/bench.cubes",
        ];
        let path = candidates.iter()
            .find(|p| Path::new(p).exists())
            .copied()
            .expect("bench.cubes 找不到");
        let content = std::fs::read_to_string(path).expect("读 bench.cubes 失败");
        let cubes: Vec<&str> = content.lines()
            .filter(|l| l.len() == 54)
            .take(50)
            .collect();
        eprintln!("使用 {} 个 cube 跑对比", cubes.len());

        let opts_baseline = SearchOptions {
            timeout: Duration::from_millis(300),
            max_solutions: 1,
            ..Default::default()
        };
        let opts_new = SearchOptions {
            timeout: Duration::from_millis(300),
            max_solutions: 8,
            ..Default::default()
        };

        let mut wins = 0i32;       // 新策略严格更优
        let mut ties = 0i32;       // 持平
        let mut regressions = 0i32; // 新策略反而更差（不应发生）
        let mut total_baseline_mech = 0i64;
        let mut total_new_mech = 0i64;
        let mut total_diff = 0i64;
        let mut max_improvement = 0i32;
        let mut sample_idx_at_max: usize = 0;

        for (i, cube) in cubes.iter().enumerate() {
            let r_base = match translate_optimal(cube, opts_baseline) {
                Ok(r) => r,
                Err(e) => { eprintln!("  [{}] baseline 失败: {}", i, e); continue; }
            };
            let r_new = match translate_optimal(cube, opts_new) {
                Ok(r) => r,
                Err(e) => { eprintln!("  [{}] new 失败: {}", i, e); continue; }
            };
            let b = r_base.best.mech_steps;
            let n = r_new.best.mech_steps;
            total_baseline_mech += b as i64;
            total_new_mech += n as i64;
            total_diff += (b - n) as i64;

            // 新策略 best 来自哪条 face 长度？baseline 用了哪条？
            let n_face_best = r_new.best.kociemba.split_whitespace().count();
            let n_face_min = r_new.candidates.iter()
                .map(|c| c.kociemba.split_whitespace().count())
                .min().unwrap_or(0);
            let best_is_min_face = n_face_best == n_face_min;

            if n < b {
                wins += 1;
                if b - n > max_improvement {
                    max_improvement = b - n;
                    sample_idx_at_max = i;
                }
                eprintln!(
                    "  [{:2}] mech: {} → {} (-{}); new 候选数={}, best face={}f, min face={}f, best_is_min_face={}",
                    i, b, n, b - n, r_new.candidates.len(),
                    n_face_best, n_face_min, best_is_min_face
                );
            } else if n == b {
                ties += 1;
            } else {
                regressions += 1;
                eprintln!("  [{:2}] !! REGRESSION: {} → {}", i, b, n);
            }
        }

        let total = wins + ties + regressions;
        eprintln!("\n=== 汇总 ({} cube) ===", total);
        eprintln!("  wins (新更优):     {} ({:.1}%)", wins, 100.0 * wins as f64 / total as f64);
        eprintln!("  ties (持平):       {} ({:.1}%)", ties, 100.0 * ties as f64 / total as f64);
        eprintln!("  regressions (变差): {}", regressions);
        eprintln!("  baseline mech 平均: {:.1}", total_baseline_mech as f64 / total as f64);
        eprintln!("  new      mech 平均: {:.1}", total_new_mech as f64 / total as f64);
        eprintln!("  累计节省 mech:      {} (平均 {:.1}/cube)",
            total_diff, total_diff as f64 / total as f64);
        eprintln!("  最大单 cube 提升:   {} mech (cube #{})",
            max_improvement, sample_idx_at_max);

        assert_eq!(regressions, 0, "新策略不应出现 mech 回归");
    }

    /// 100ms / max_solutions=∞ vs 之前 300ms/8 baseline：
    /// 看新策略放开 slack 后，能否在 1/3 延迟下达到或超过原 mech 表现。
    #[test]
    #[ignore]
    fn stress_100ms_unlimited_vs_300ms_8() {
        for slack in [0i32, 1, 2] {
            eprintln!("\n##### slack = {} #####", slack);
            ab_compare(
                "300ms/8/slack=0",
                SearchOptions {
                    timeout: Duration::from_millis(300),
                    max_solutions: 8,
                    length_slack: 0,
                    ..Default::default()
                },
                &format!("100ms/∞/slack={}", slack),
                SearchOptions {
                    timeout: Duration::from_millis(100),
                    max_solutions: usize::MAX,
                    length_slack: slack,
                    ..Default::default()
                },
                50,
            );
        }
    }

    /// 同 slack=0 / 500ms 旧版基线对比 100ms/slack=2/∞ 新设定。
    /// 看延迟从 300ms 砍到 100ms 后 mech 是否显著退化（或反而更好）。
    #[test]
    #[ignore]
    fn stress_slack_sweep() {
        use robo_solver::search::Search;
        Search::init();
        let cubes = load_bench_cubes(50);

        let configs: &[(&str, SearchOptions)] = &[
            ("100ms/∞/slack=0", SearchOptions {
                timeout: Duration::from_millis(100),
                max_solutions: usize::MAX,
                length_slack: 0,
                ..Default::default()
            }),
            ("100ms/∞/slack=1", SearchOptions {
                timeout: Duration::from_millis(100),
                max_solutions: usize::MAX,
                length_slack: 1,
                ..Default::default()
            }),
            ("100ms/∞/slack=2", SearchOptions {
                timeout: Duration::from_millis(100),
                max_solutions: usize::MAX,
                length_slack: 2,
                ..Default::default()
            }),
            ("100ms/∞/slack=3", SearchOptions {
                timeout: Duration::from_millis(100),
                max_solutions: usize::MAX,
                length_slack: 3,
                ..Default::default()
            }),
        ];

        eprintln!("\n=== Slack sweep on {} cubes ===", cubes.len());
        for (label, opts) in configs {
            let mut total_mech = 0i64;
            let mut total_cands = 0i64;
            let mut total_solver = Duration::ZERO;
            let mut min_face_distrib = std::collections::BTreeMap::<usize, usize>::new();
            for cube in &cubes {
                if let Ok(r) = translate_optimal(cube, *opts) {
                    total_mech += r.best.mech_steps as i64;
                    total_cands += r.candidates.len() as i64;
                    total_solver += r.solver_elapsed;
                    let best_face = r.best.kociemba.split_whitespace().count();
                    *min_face_distrib.entry(best_face).or_insert(0) += 1;
                }
            }
            let n = cubes.len() as f64;
            eprintln!(
                "  {}: mech 平均 {:.1}, 候选平均 {:.1}, solver 平均 {:?}",
                label,
                total_mech as f64 / n,
                total_cands as f64 / n,
                total_solver / cubes.len() as u32,
            );
            eprintln!("    best face 分布: {:?}", min_face_distrib);
        }
    }
}
