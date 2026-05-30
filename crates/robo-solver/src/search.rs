//! Search - Two-phase IDA* Rubik's Cube solver.
//! Ported from Search.java.

use std::time::{Duration, Instant};

use crate::util;
use crate::cubie_cube::{self, CubieCube, URF_MOVE};
use crate::coord_cube::{self, CoordCubeNode};

pub const USE_SEPARATOR: i32 = 0x1;
pub const INVERSE_SOLUTION: i32 = 0x2;
pub const APPEND_LENGTH: i32 = 0x4;
pub const OPTIMAL_SOLUTION: i32 = 0x8;

/// 多解搜索参数。
///
/// ## 实测调优建议（50 真实 cube benchmark, M-series Mac, release）
///
/// | 配置                         | mech 平均 | solver 耗时 |
/// |------------------------------|----------|-------------|
/// | 300ms / max=8 / slack=0      | 68.9     | ~199ms      |
/// | **100ms / max=∞ / slack=0**  | **68.8** | 100ms       |  ← 推荐
/// | 100ms / max=∞ / slack=1      | 68.9     | 100ms       |  ← 高方差
/// | 100ms / max=∞ / slack=2      | 69.4     | 100ms       |  ← 略差
///
/// 关键洞察：
/// - **`max_solutions = usize::MAX` 是免费收益**：让 URF 多样性扫描跑透，相同
///   timeout 内多收 1-3 条候选，mech 略有改善。
/// - **`length_slack > 0` 不是免费午餐**：放开 face 上限 1-2 步看似让 handstep
///   有更多选择，但 face 多 1 步通常意味着 mech 展开多 3-7 步，这远大于
///   "同 face 不同 URF 路径"的 mech 差距（通常 3-10 步）。slack 把预算分给
///   "更长的解的变体"，单变体收益往往不能补偿 face 数本身的展开成本。
/// - 因此默认 `slack=0`，仅在希望"多样性优先于最优性"的边缘场景下打开。
#[derive(Clone, Copy, Debug)]
pub struct SearchOptions {
    /// 总超时（涵盖所有 IDA* 探索 + next() 推进 + URF 重启）。
    /// 达到后立即返回已收集的解。
    pub timeout: Duration,
    /// 候选数量上限（硬上限，到达即停）。设为 `usize::MAX` 表示"timeout 内能
    /// 找多少是多少"。
    ///
    /// 候选来自两个互补来源：
    /// 1. **URF 多样性**：6 个 URF 起点，每个起点用 `solution()` 限深
    ///    `L0 + length_slack`，再用 `next()` 在该 URF 上反复推进取更短解。
    /// 2. **next 递减**：跨 URF 找严格更短的解。
    ///
    /// 上层 `translate_optimal` 用 handstep 翻译每条候选选机械最短，所以
    /// "同 face 长度路径不同"非常有价值（mech 步数差距常常远大于 face 差）。
    pub max_solutions: usize,
    /// 单条解的最大 face 数（同 `solution()` 的 `max_depth`）。
    pub max_depth: i32,
    /// **长度容差**：找到首搜长度 `L0` 后，允许接受长度 ≤ `L0 + length_slack`
    /// 的候选。0 = 严格不超过 L0；2 = 允许多 2 步 face（mech 可能反而更短）。
    ///
    /// 实测 cube #19 / #46 等场景：face 长 2 步但 mech 少 16-21 步——这是
    /// face 数和机械时间的非线性关系决定的。
    pub length_slack: i32,
    /// IDA* 内部 probe 上限（影响搜索强度），通常用 10_000_000。
    pub probe_max: i64,
    /// IDA* 内部 probe 下限。
    pub probe_min: i64,
}

impl Default for SearchOptions {
    fn default() -> Self {
        Self {
            timeout: Duration::from_millis(500),
            max_solutions: 5,
            max_depth: 21,
            length_slack: 0,
            probe_max: 10_000_000,
            probe_min: 100,
        }
    }
}

/// 多解搜索结果。
#[derive(Clone, Debug)]
pub struct SolverResult {
    /// 候选解列表，按发现顺序（同长度或递减长度）。
    pub solutions: Vec<String>,
    /// 总耗时。
    pub elapsed: Duration,
    /// 是否因超时而提前返回（true 表示候选可能少于 `max_solutions`）。
    pub timed_out: bool,
}

const MAX_PRE_MOVES: usize = 20;
const TRY_INVERSE: bool = true;
const TRY_THREE_AXES: bool = true;
const MIN_P1LENGTH_PRE: i32 = 7;
const MAX_DEPTH2: i32 = 13;

pub struct Search {
    mov: [i32; 31],
    move_sol: [i32; 31],

    node_ud: Vec<CoordCubeNode>,
    node_rl: Vec<CoordCubeNode>,
    node_fb: Vec<CoordCubeNode>,

    self_sym: i64,
    conj_mask: i32,
    urf_idx: usize,
    length1: i32,
    depth1: i32,
    max_dep2: i32,
    sol: i32,
    solution: Option<String>,
    probe: i64,
    probe_max: i64,
    probe_min: i64,
    verbose: i32,
    valid1: i32,
    allow_shorter: bool,
    cc: CubieCube,
    urf_cubie_cube: [CubieCube; 6],
    urf_coord_cube: [CoordCubeNode; 6],
    phase1_cubie: Vec<CubieCube>,

    pre_move_cubes: Vec<CubieCube>,
    pre_moves: [i32; MAX_PRE_MOVES],
    pre_move_len: i32,
    max_pre_moves: i32,
    phase2_cubie: CubieCube,

    is_rec: bool,

    /// 超时截止点；None 表示无超时。被 IDA* 检查（伪装成 probe 耗尽）。
    deadline: Option<Instant>,

    /// 仅在 `solution()` 入口生效：限定 URF 索引为某个值，只用这一个起点搜。
    /// `solutions()` 用它枚举 0..6 个 URF 取"同长度不同路径"的多样候选。
    /// `None` 表示默认行为（遍历所有 URF）。
    force_urf_only: Option<usize>,
}

impl Default for Search {
    fn default() -> Self {
        Search {
            mov: [0; 31],
            move_sol: [0; 31],
            node_ud: vec![CoordCubeNode::default(); 21],
            node_rl: vec![CoordCubeNode::default(); 21],
            node_fb: vec![CoordCubeNode::default(); 21],
            self_sym: 0,
            conj_mask: 0,
            urf_idx: 0,
            length1: 0,
            depth1: 0,
            max_dep2: 0,
            sol: 0,
            solution: None,
            probe: 0,
            probe_max: 0,
            probe_min: 0,
            verbose: 0,
            valid1: 0,
            allow_shorter: false,
            cc: CubieCube::default(),
            urf_cubie_cube: [CubieCube::default(); 6],
            urf_coord_cube: [CoordCubeNode::default(); 6],
            phase1_cubie: vec![CubieCube::default(); 21],
            pre_move_cubes: vec![CubieCube::default(); MAX_PRE_MOVES + 1],
            pre_moves: [0; MAX_PRE_MOVES],
            pre_move_len: 0,
            max_pre_moves: 0,
            phase2_cubie: CubieCube::default(),
            is_rec: false,
            deadline: None,
            force_urf_only: None,
        }
    }
}

impl Search {
    pub fn new() -> Self {
        Self::default()
    }

    /// Initialize the solver tables. Call once before solving.
    pub fn init() {
        cubie_cube::ensure_tables_initialized();
        coord_cube::ensure_initialized();
    }

    /// Solve the given cube facelets string.
    pub fn solution(
        &mut self,
        facelets: &str,
        max_depth: i32,
        probe_max: i64,
        probe_min: i64,
        verbose: i32,
    ) -> String {
        let check = self.verify_facelets(facelets);
        if check != 0 {
            return format!("Error {}", check.abs());
        }
        self.sol = max_depth + 1;
        self.probe = 0;
        self.probe_max = probe_max;
        self.probe_min = probe_min.min(probe_max);
        self.verbose = verbose;
        self.solution = None;
        self.is_rec = false;

        Self::init();
        self.init_search();

        if (verbose & OPTIMAL_SOLUTION) == 0 {
            self.search()
        } else {
            self.searchopt()
        }
    }

    /// 多解搜索：在 `timeout` / `max_solutions` 限制下尽量收集**互不相同**的候选。
    ///
    /// ## 策略（混合 3 个候选源）
    /// 1. **首搜**（无 URF 限制）：标准 `solution()` 拿到长度 `L0`。
    /// 2. **next() 递减**：在共享 IDA* 上下文里反复推进，找严格更短的解。
    /// 3. **URF 多样性补充**：每发现一个新长度后，对 6 个 URF 起点做一轮限深
    ///    `cap = current_best + length_slack` 的复搜（每次仅 force 一个 URF）。
    ///    这一步利用首搜已经预热的 IDA* 表，每个 URF 限深搜索通常很快。
    ///
    /// 关键设计：**混合而非串行**。串行"6 个 URF 各自从 0 开始深挖"看起来
    /// 干净，但实测 100ms 内只跑得完 2-3 个 URF，候选数比混合策略少；
    /// 混合让 next() 推进和 URF 复搜交叉进行，能在相同 timeout 内多产 50%+ 候选。
    ///
    /// ## 设计动机
    /// 上层 `translate_optimal` 用 handstep 翻译每条候选选机械最短。
    /// **face 数最少 ≠ mech 步数最少**：实测有 cube face 长 2 步但 mech 少 16-21 步。
    /// `length_slack > 0` 允许接受比当前最优略长的候选，handstep 反而能挑出
    /// 更优变体；同时同 face 长度的不同 URF 路径也是常见的"mech 优化机会"。
    ///
    /// `max_solutions = usize::MAX` 时退化成"timeout 内能找多少是多少"。
    pub fn solutions(&mut self, facelets: &str, opts: SearchOptions) -> SolverResult {
        let start = Instant::now();
        let deadline_at = start + opts.timeout;
        self.deadline = Some(deadline_at);

        let initial_cap = opts.max_solutions.min(64);
        let mut out: Vec<String> = Vec::with_capacity(initial_cap);
        let mut seen: std::collections::HashSet<String> =
            std::collections::HashSet::with_capacity(initial_cap * 2);
        let mut timed_out = false;

        let timed_out_now = |t: Instant| t >= deadline_at;

        // === 阶段 1：首搜，确定 L0 ===
        self.force_urf_only = None;
        let s = self.solution(facelets, opts.max_depth, opts.probe_max, opts.probe_min, 0);
        if s.starts_with("Error") {
            self.deadline = None;
            return SolverResult {
                solutions: out,
                elapsed: start.elapsed(),
                timed_out: timed_out_now(Instant::now()),
            };
        }
        let s_trim = s.trim().to_string();
        let l0: i32 = s_trim.split_whitespace().count() as i32;
        seen.insert(s_trim.clone());
        out.push(s_trim);

        // 当前已知最短长度（每次找到更短解会下降）
        let mut best_len = l0;

        // 单 URF 复搜：调一次 solution(facelets, cap) + force_urf_only=Some(urf)。
        // 用宏避免闭包 borrow self 问题。返回 (是否超时, 是否找到新解)。
        macro_rules! force_urf_search {
            ($urf:expr, $cap:expr) => {{
                let urf_v: usize = $urf;
                let cap_v: i32 = $cap;
                let mut hit_timeout = false;
                let mut got_new = false;
                let mut got_shorter = false;
                self.force_urf_only = Some(urf_v);
                let s = self.solution(facelets, cap_v, opts.probe_max, opts.probe_min, 0);
                self.force_urf_only = None;

                if s.starts_with("Error") {
                    if s.starts_with("Error 8") && timed_out_now(Instant::now()) {
                        hit_timeout = true;
                    }
                    // Error 7：该 URF 在 cap 内无解 → 跳过即可
                } else {
                    let s_trim = s.trim().to_string();
                    let new_len = s_trim.split_whitespace().count() as i32;
                    if seen.insert(s_trim.clone()) {
                        out.push(s_trim);
                        got_new = true;
                        if new_len < best_len {
                            best_len = new_len;
                            got_shorter = true;
                        }
                    }
                }
                (hit_timeout, got_new, got_shorter)
            }};
        }

        // === 阶段 2A：URF 多样性扫描（cap = best_len + slack）===
        //
        // 每个 (urf, cap) 至多调一次 solution()。单 URF 调用稳定 ~1ms。
        // 找到更短解时下调 best_len → cap 收紧 → 已试过的 URF 在新 cap 下再得机会。
        let mut tried: std::collections::HashSet<(usize, i32)> =
            std::collections::HashSet::with_capacity(48);

        macro_rules! diversity_scan {
            () => {
                loop {
                    if out.len() >= opts.max_solutions { break; }
                    if timed_out_now(Instant::now()) { timed_out = true; break; }

                    let cap_now = (best_len + opts.length_slack).min(opts.max_depth);
                    let mut progressed = false;
                    for urf in 0..6usize {
                        if out.len() >= opts.max_solutions { break; }
                        if timed_out_now(Instant::now()) { timed_out = true; break; }
                        if (self.conj_mask & (1 << urf)) != 0 { continue; }
                        if !tried.insert((urf, cap_now)) { continue; }
                        // 不能用 urf_coord_cube[urf].prun > cap_now 短路：pre-move
                        // 路径会让 effective phase1 比 root prun 短，简单魔方会丢解。
                        // 让 solution() 自己用 phase1 内部 prun（基于 pre-moved cc）
                        // 证伪即可——开销很小。

                        let (timeout, _new, shorter) = force_urf_search!(urf, cap_now);
                        if timeout { timed_out = true; break; }
                        progressed = true;
                        if shorter { break; } // cap 已变，重启外循环
                    }
                    if !progressed { break; }
                }
            };
        }

        diversity_scan!();

        // === 阶段 2B：multi-URF solution() 找严格更短的解 ===
        //
        // 阶段 2A 做完后若还有时间预算，再尝试 cap=best_len-1 找更短解。
        // 单次 solution(facelets, target=best_len-1) 调用：
        //   - 命中：~1-50ms，best_len 下降；下面再扫一轮 URF 多样性。
        //   - 不命中：穷尽搜索证明"无更短"，可能吃掉剩余预算，但这个开销
        //     是必须的（确认全局最优）。如果这是 Kociemba 最优，确认无解就
        //     是 IDA* 在该深度上的结论。
        //
        // 用全局 deadline 做超时控制：solution() 内部 init_phase2_pre 会检查
        // deadline，超时返回 Error 8。
        while out.len() < opts.max_solutions && !timed_out {
            if timed_out_now(Instant::now()) { timed_out = true; break; }
            if best_len <= 1 { break; }

            self.force_urf_only = None;
            let target = best_len - 1;
            let s = self.solution(facelets, target, opts.probe_max, opts.probe_min, 0);
            if s.starts_with("Error") {
                if s.starts_with("Error 8") && timed_out_now(Instant::now()) {
                    timed_out = true;
                }
                break; // Error 7 = 无更短解；Error 8 = 超时
            }
            let s_trim = s.trim().to_string();
            let new_len = s_trim.split_whitespace().count() as i32;
            if seen.insert(s_trim.clone()) {
                out.push(s_trim);
            }
            if new_len < best_len {
                best_len = new_len;
                // 在新 best_len 上再扫一轮多样性
                diversity_scan!();
            } else {
                break;
            }
        }

        self.deadline = None;
        self.force_urf_only = None;
        SolverResult {
            solutions: out,
            elapsed: start.elapsed(),
            timed_out,
        }
    }

    /// Continue searching for shorter solutions.
    pub fn next(&mut self, probe_max: i64, probe_min: i64, verbose: i32) -> String {
        self.probe = 0;
        self.probe_max = probe_max;
        self.probe_min = probe_min.min(probe_max);
        self.solution = None;
        self.is_rec = (self.verbose & OPTIMAL_SOLUTION) == (verbose & OPTIMAL_SOLUTION);
        self.verbose = verbose;
        if (verbose & OPTIMAL_SOLUTION) == 0 {
            self.search()
        } else {
            self.searchopt()
        }
    }

    fn verify_facelets(&mut self, facelets: &str) -> i32 {
        if facelets.len() != 54 {
            return -1;
        }
        let chars: Vec<char> = facelets.chars().collect();
        let center = [
            chars[util::U5 as usize],
            chars[util::R5 as usize],
            chars[util::F5 as usize],
            chars[util::D5 as usize],
            chars[util::L5 as usize],
            chars[util::B5 as usize],
        ];
        let mut f = [0u8; 54];
        let mut count = 0u32;
        for i in 0..54 {
            let idx = center.iter().position(|&c| c == chars[i]);
            match idx {
                Some(v) => {
                    f[i] = v as u8;
                    count += 1 << (v << 2);
                }
                None => return -1,
            }
        }
        if count != 0x999999 {
            return -1;
        }
        util::to_cubie_cube(&f, &mut self.cc.ca, &mut self.cc.ea);
        self.cc.verify()
    }

    fn init_search(&mut self) {
        self.conj_mask = (if TRY_INVERSE { 0 } else { 0x38 })
            | (if TRY_THREE_AXES { 0 } else { 0x36 });
        self.self_sym = self.cc.self_symmetry();
        self.conj_mask |= if ((self.self_sym >> 16) & 0xffff) != 0 { 0x12 } else { 0 };
        self.conj_mask |= if ((self.self_sym >> 32) & 0xffff) != 0 { 0x24 } else { 0 };
        self.conj_mask |= if ((self.self_sym >> 48) & 0xffff) != 0 { 0x38 } else { 0 };
        self.self_sym &= 0xffffffffffff;
        self.max_pre_moves = if self.conj_mask > 7 { 0 } else { MAX_PRE_MOVES as i32 };

        for i in 0..6 {
            self.urf_cubie_cube[i] = self.cc;
            self.urf_coord_cube[i].set_with_prun(&self.urf_cubie_cube[i], 20);
            self.cc.urf_conjugate();
            if i % 3 == 2 {
                self.cc.inv_cubie_cube();
            }
        }
    }

    fn search(&mut self) -> String {
        let mut length1 = if self.is_rec { self.length1 } else { 0 };
        let mut first_iter = true;
        while length1 < self.sol {
            self.length1 = length1;
            self.max_dep2 = MAX_DEPTH2.min(self.sol - length1);
            // URF 起点：is_rec 时延续上次；否则默认从 0 开始。
            // `force_urf_only = Some(u)` 时，限定只跑 urf=u 这一个起点
            //（用于 solutions() 枚举不同 URF 收集"同长度不同路径"的候选）。
            let (start_urf, end_urf) = if let Some(u) = self.force_urf_only {
                (u, u + 1)
            } else if first_iter && self.is_rec {
                (self.urf_idx, 6)
            } else {
                (0, 6)
            };
            first_iter = false;
            for urf_idx in start_urf..end_urf {
                self.urf_idx = urf_idx;
                if (self.conj_mask & (1 << urf_idx)) != 0 {
                    continue;
                }
                if self.phase1_pre_moves(
                    self.max_pre_moves,
                    -30,
                    self.urf_cubie_cube[urf_idx],
                    (self.self_sym & 0xffff) as i32,
                ) == 0
                {
                    return match &self.solution {
                        Some(s) => s.clone(),
                        None => "Error 8".to_string(),
                    };
                }
            }
            length1 += 1;
        }
        match &self.solution {
            Some(s) => s.clone(),
            None => "Error 7".to_string(),
        }
    }

    fn phase1_pre_moves(&mut self, maxl: i32, lm: i32, cc: CubieCube, ssym: i32) -> i32 {
        self.pre_move_len = self.max_pre_moves - maxl;
        if if self.is_rec {
            self.depth1 == self.length1 - self.pre_move_len
        } else {
            self.pre_move_len == 0 || (0x36FB7 >> lm) & 1 == 0
        } {
            self.depth1 = self.length1 - self.pre_move_len;
            self.phase1_cubie[0] = cc;
            self.allow_shorter = self.depth1 == MIN_P1LENGTH_PRE && self.pre_move_len != 0;

            let depth1 = self.depth1;
            if self.node_ud[(depth1 + 1) as usize].set_with_prun(&cc, depth1) {
                let node = self.node_ud[(depth1 + 1) as usize];
                if self.phase1(&node, ssym, depth1, -1) == 0 {
                    return 0;
                }
            }
        }

        if maxl == 0 || self.pre_move_len + MIN_P1LENGTH_PRE >= self.length1 {
            return 1;
        }

        let mut skip_moves = cubie_cube::get_skip_moves(ssym as i64);
        if maxl == 1 || self.pre_move_len + 1 + MIN_P1LENGTH_PRE >= self.length1 {
            skip_moves |= 0x36FB7;
        }

        let lm3 = lm / 3 * 3;
        let ct = cubie_cube::get_tables();
        let mut m = 0i32;
        while m < 18 {
            if m == lm3 || m == lm3 - 9 || m == lm3 + 9 {
                m += 3; // skip entire axis (3 moves)
                continue;
            }
            if self.is_rec && m != self.pre_moves[(self.max_pre_moves - maxl) as usize]
                || (skip_moves & (1 << m)) != 0
            {
                m += 1;
                continue;
            }
            let mut new_cube = CubieCube::new();
            CubieCube::corn_mult(&ct.move_cube[m as usize], &cc, &mut new_cube);
            CubieCube::edge_mult(&ct.move_cube[m as usize], &cc, &mut new_cube);
            self.pre_moves[(self.max_pre_moves - maxl) as usize] = m;
            let new_ssym = ssym & ct.move_cube_sym[m as usize] as i32;
            let ret = self.phase1_pre_moves(maxl - 1, m, new_cube, new_ssym);
            if ret == 0 {
                return 0;
            }
            m += 1;
        }
        1
    }

    fn phase1(&mut self, node: &CoordCubeNode, ssym: i32, maxl: i32, lm: i32) -> i32 {
        if node.prun == 0 && maxl < 5 {
            if self.allow_shorter || maxl == 0 {
                self.depth1 -= maxl;
                let ret = self.init_phase2_pre();
                self.depth1 += maxl;
                return ret;
            } else {
                return 1;
            }
        }

        let skip_moves = cubie_cube::get_skip_moves(ssym as i64);
        let ct = cubie_cube::get_tables();

        for axis in (0..18).step_by(3) {
            if axis == lm || axis == lm - 9 {
                continue;
            }
            for power in 0..3 {
                let m = axis + power;

                if self.is_rec && m != self.mov[(self.depth1 - maxl) as usize] {
                    continue;
                }
                if skip_moves != 0 && (skip_moves & (1 << m)) != 0 {
                    continue;
                }

                let prun = self.node_ud[maxl as usize].do_move_prun(node, m as usize, true);
                if prun > maxl {
                    break;
                } else if prun == maxl {
                    continue;
                }

                // USE_CONJ_PRUN
                let prun_conj = self.node_ud[maxl as usize].do_move_prun_conj(node, m as usize);
                if prun_conj > maxl {
                    break;
                } else if prun_conj == maxl {
                    continue;
                }

                self.mov[(self.depth1 - maxl) as usize] = m;
                self.valid1 = self.valid1.min(self.depth1 - maxl);
                let new_node = self.node_ud[maxl as usize];
                let new_ssym = ssym & ct.move_cube_sym[m as usize] as i32;
                let ret = self.phase1(&new_node, new_ssym, maxl - 1, axis);
                if ret == 0 {
                    return 0;
                } else if ret == 2 {
                    break;
                }
            }
        }
        1
    }

    fn init_phase2_pre(&mut self) -> i32 {
        self.is_rec = false;
        // 超时检查：伪装 probe 耗尽，让 IDA* 自然退栈
        if let Some(d) = self.deadline {
            if Instant::now() >= d {
                self.probe = self.probe_max + 1;
                return 0;
            }
        }
        if self.probe >= if self.solution.is_none() { self.probe_max } else { self.probe_min } {
            return 0;
        }
        self.probe += 1;

        let ct = cubie_cube::get_tables();
        for i in self.valid1..self.depth1 {
            let prev = self.phase1_cubie[i as usize];
            let m = self.mov[i as usize] as usize;
            let mut next = CubieCube::new();
            CubieCube::corn_mult(&prev, &ct.move_cube[m], &mut next);
            CubieCube::edge_mult(&prev, &ct.move_cube[m], &mut next);
            self.phase1_cubie[(i + 1) as usize] = next;
        }
        self.valid1 = self.depth1;
        self.phase2_cubie = self.phase1_cubie[self.depth1 as usize];

        let ret = self.init_phase2();
        if ret == 0 || self.pre_move_len == 0 || ret == 2 {
            return ret;
        }

        // Try x2 pre-move
        let m = (self.pre_moves[(self.pre_move_len - 1) as usize] / 3 * 3 + 1) as usize;
        let mut new_p2 = CubieCube::new();
        CubieCube::corn_mult(&ct.move_cube[m], &self.phase1_cubie[self.depth1 as usize], &mut new_p2);
        CubieCube::edge_mult(&ct.move_cube[m], &self.phase1_cubie[self.depth1 as usize], &mut new_p2);
        self.phase2_cubie = new_p2;

        let pm_idx = (self.pre_move_len - 1) as usize;
        self.pre_moves[pm_idx] += 2 - (self.pre_moves[pm_idx] % 3) * 2;
        let ret = self.init_phase2();
        self.pre_moves[pm_idx] += 2 - (self.pre_moves[pm_idx] % 3) * 2;
        ret
    }

    fn init_phase2(&mut self) -> i32 {
        let coord = coord_cube::get_coord_tables();
        let ct = cubie_cube::get_tables();

        let p2corn = self.phase2_cubie.get_c_perm_sym();
        let p2csym = (p2corn & 0xf) as usize;
        let p2corn_idx = (p2corn >> 4) as usize;
        let p2edge = self.phase2_cubie.get_e_perm_sym();
        let p2esym = (p2edge & 0xf) as usize;
        let p2edge_idx = (p2edge >> 4) as usize;
        let p2mid = self.phase2_cubie.get_m_perm() as usize;

        let prun = coord_cube::get_pruning(
            &coord.e_perm_c_comb_p_prun,
            p2edge_idx * coord_cube::N_COMB
                + coord.c_comb_p_conj[ct.perm2_comb_p[p2corn_idx] as usize]
                    [ct.sym_mult_inv[p2esym][p2csym]] as usize,
        )
        .max(coord_cube::get_pruning(
            &coord.mc_perm_prun,
            p2corn_idx * coord_cube::N_MPERM + coord.m_perm_conj[p2mid][p2csym] as usize,
        ));

        if prun >= self.max_dep2 {
            return if prun > self.max_dep2 { 2 } else { 1 };
        }

        let mut depth2 = self.max_dep2 - 1;
        while depth2 >= prun {
            let ret = self.phase2(
                p2edge_idx as i32,
                p2esym as i32,
                p2corn_idx as i32,
                p2csym as i32,
                p2mid as i32,
                depth2,
                self.depth1,
                10,
            );
            if ret < 0 {
                break;
            }
            depth2 -= ret;
            self.sol = 0;
            for i in 0..(self.depth1 + depth2) {
                self.append_sol_move(self.mov[i as usize]);
            }
            for i in (0..self.pre_move_len).rev() {
                self.append_sol_move(self.pre_moves[i as usize]);
            }
            self.solution = Some(self.solution_to_string());
            depth2 -= 1;
        }

        if depth2 != self.max_dep2 - 1 {
            self.max_dep2 = MAX_DEPTH2.min(self.sol - self.length1);
            return if self.probe >= self.probe_min { 0 } else { 1 };
        }
        1
    }

    fn phase2(
        &mut self,
        edge: i32,
        esym: i32,
        corn: i32,
        csym: i32,
        mid: i32,
        maxl: i32,
        depth: i32,
        lm: i32,
    ) -> i32 {
        if edge == 0 && corn == 0 && mid == 0 {
            return maxl;
        }
        let coord = coord_cube::get_coord_tables();
        let ct = cubie_cube::get_tables();
        let ckmv2bit = util::get_ckmv2bit();
        let move_mask = ckmv2bit[lm as usize];

        let mut m = 0i32;
        while m < 10 {
            if (move_mask >> m) & 1 != 0 {
                m += (0x42 >> m) & 3;
                m += 1;
                continue;
            }
            let midx = coord.m_perm_move[mid as usize][m as usize] as i32;
            let cornx_full = coord.c_perm_move[corn as usize]
                [ct.sym_move_ud[csym as usize][m as usize] as usize] as i32;
            let csymx = ct.sym_mult[(cornx_full & 0xf) as usize][csym as usize];
            let cornx = cornx_full >> 4;
            let edgex_full = coord.e_perm_move[edge as usize]
                [ct.sym_move_ud[esym as usize][m as usize] as usize] as i32;
            let esymx = ct.sym_mult[(edgex_full & 0xf) as usize][esym as usize];
            let edgex = edgex_full >> 4;
            let edgei = cubie_cube::get_perm_sym_inv(edgex, esymx as usize, false);
            let corni = cubie_cube::get_perm_sym_inv(cornx, csymx as usize, true);

            let prun = coord_cube::get_pruning(
                &coord.e_perm_c_comb_p_prun,
                (edgei >> 4) as usize * coord_cube::N_COMB
                    + coord.c_comb_p_conj[ct.perm2_comb_p[(corni >> 4) as usize] as usize]
                        [ct.sym_mult_inv[(edgei & 0xf) as usize][(corni & 0xf) as usize]]
                        as usize,
            );
            if prun > maxl + 1 {
                break;
            } else if prun >= maxl {
                m += (0x42 >> m) & 3 & (maxl - prun);
                m += 1;
                continue;
            }
            let prun = coord_cube::get_pruning(
                &coord.mc_perm_prun,
                cornx as usize * coord_cube::N_MPERM
                    + coord.m_perm_conj[midx as usize][csymx as usize] as usize,
            )
            .max(coord_cube::get_pruning(
                &coord.e_perm_c_comb_p_prun,
                edgex as usize * coord_cube::N_COMB
                    + coord.c_comb_p_conj[ct.perm2_comb_p[cornx as usize] as usize]
                        [ct.sym_mult_inv[esymx as usize][csymx as usize]]
                        as usize,
            ));
            if prun >= maxl {
                m += (0x42 >> m) & 3 & (maxl - prun);
                m += 1;
                continue;
            }
            let ret = self.phase2(edgex, esymx, cornx, csymx, midx, maxl - 1, depth + 1, m);
            if ret >= 0 {
                self.mov[depth as usize] = util::UD2STD[m as usize];
                return ret;
            }
            m += 1;
        }
        -1
    }

    fn append_sol_move(&mut self, cur_move: i32) {
        if self.sol == 0 {
            self.move_sol[self.sol as usize] = cur_move;
            self.sol += 1;
            return;
        }
        let axis_cur = cur_move / 3;
        let axis_last = self.move_sol[(self.sol - 1) as usize] / 3;
        if axis_cur == axis_last {
            let pow = (cur_move % 3 + self.move_sol[(self.sol - 1) as usize] % 3 + 1) % 4;
            if pow == 3 {
                self.sol -= 1;
            } else {
                self.move_sol[(self.sol - 1) as usize] = axis_cur * 3 + pow;
            }
            return;
        }
        if self.sol > 1
            && axis_cur % 3 == axis_last % 3
            && axis_cur == self.move_sol[(self.sol - 2) as usize] / 3
        {
            let pow = (cur_move % 3 + self.move_sol[(self.sol - 2) as usize] % 3 + 1) % 4;
            if pow == 3 {
                self.move_sol[(self.sol - 2) as usize] = self.move_sol[(self.sol - 1) as usize];
                self.sol -= 1;
            } else {
                self.move_sol[(self.sol - 2) as usize] = axis_cur * 3 + pow;
            }
            return;
        }
        self.move_sol[self.sol as usize] = cur_move;
        self.sol += 1;
    }

    fn solution_to_string(&self) -> String {
        let mut sb = String::new();
        let urf = if (self.verbose & INVERSE_SOLUTION) != 0 {
            (self.urf_idx + 3) % 6
        } else {
            self.urf_idx
        };
        if urf < 3 {
            for s in 0..self.sol as usize {
                if (self.verbose & USE_SEPARATOR) != 0 && s == self.depth1 as usize {
                    sb.push_str(".  ");
                }
                sb.push_str(util::MOVE2STR[URF_MOVE[urf][self.move_sol[s] as usize] as usize]);
                sb.push(' ');
            }
        } else {
            for s in (0..self.sol as usize).rev() {
                sb.push_str(util::MOVE2STR[URF_MOVE[urf][self.move_sol[s] as usize] as usize]);
                sb.push(' ');
                if (self.verbose & USE_SEPARATOR) != 0 && s == self.depth1 as usize {
                    sb.push_str(".  ");
                }
            }
        }
        if (self.verbose & APPEND_LENGTH) != 0 {
            sb.push_str(&format!("({}f)", self.sol));
        }
        sb
    }

    fn searchopt(&mut self) -> String {
        let mut maxprun1 = 0;
        let mut maxprun2 = 0;
        for i in 0..6 {
            self.urf_coord_cube[i].calc_pruning(false);
            if i < 3 {
                maxprun1 = maxprun1.max(self.urf_coord_cube[i].prun);
            } else {
                maxprun2 = maxprun2.max(self.urf_coord_cube[i].prun);
            }
        }
        self.urf_idx = if maxprun2 > maxprun1 { 3 } else { 0 };
        self.phase1_cubie[0] = self.urf_cubie_cube[self.urf_idx];
        let start = if self.is_rec { self.length1 } else { 0 };
        for length1 in start..self.sol {
            self.length1 = length1;
            let ud = self.urf_coord_cube[self.urf_idx];
            let rl = self.urf_coord_cube[1 + self.urf_idx];
            let fb = self.urf_coord_cube[2 + self.urf_idx];

            if ud.prun <= length1 && rl.prun <= length1 && fb.prun <= length1
                && self.phase1opt(&ud, &rl, &fb, self.self_sym, length1, -1) == 0
            {
                return match &self.solution {
                    Some(s) => s.clone(),
                    None => "Error 8".to_string(),
                };
            }
        }
        match &self.solution {
            Some(s) => s.clone(),
            None => "Error 7".to_string(),
        }
    }

    fn phase1opt(
        &mut self,
        ud: &CoordCubeNode,
        rl: &CoordCubeNode,
        fb: &CoordCubeNode,
        ssym: i64,
        maxl: i32,
        lm: i32,
    ) -> i32 {
        if ud.prun == 0 && rl.prun == 0 && fb.prun == 0 && maxl < 5 {
            self.max_dep2 = maxl + 1;
            self.depth1 = self.length1 - maxl;
            return if self.init_phase2_pre() == 0 { 0 } else { 1 };
        }

        let skip_moves = cubie_cube::get_skip_moves(ssym);
        let ct = cubie_cube::get_tables();

        for axis in (0..18).step_by(3) {
            if axis == lm || axis == lm - 9 {
                continue;
            }
            for power in 0..3 {
                let m = axis + power;

                if self.is_rec && m != self.mov[(self.length1 - maxl) as usize] {
                    continue;
                }
                if skip_moves != 0 && (skip_moves & (1 << m)) != 0 {
                    continue;
                }

                // UD Axis
                let prun_ud = self.node_ud[maxl as usize].do_move_prun(ud, m as usize, false)
                    .max(self.node_ud[maxl as usize].do_move_prun_conj(ud, m as usize));
                if prun_ud > maxl {
                    break;
                } else if prun_ud == maxl {
                    continue;
                }

                // RL Axis
                let m_rl = URF_MOVE[2][m as usize] as i32;
                let prun_rl = self.node_rl[maxl as usize].do_move_prun(rl, m_rl as usize, false)
                    .max(self.node_rl[maxl as usize].do_move_prun_conj(rl, m_rl as usize));
                if prun_rl > maxl {
                    break;
                } else if prun_rl == maxl {
                    continue;
                }

                // FB Axis
                let m_fb = URF_MOVE[2][m_rl as usize] as i32;
                let mut prun_fb = self.node_fb[maxl as usize].do_move_prun(fb, m_fb as usize, false)
                    .max(self.node_fb[maxl as usize].do_move_prun_conj(fb, m_fb as usize));
                if prun_ud == prun_rl && prun_rl == prun_fb && prun_fb != 0 {
                    prun_fb += 1;
                }
                if prun_fb > maxl {
                    break;
                } else if prun_fb == maxl {
                    continue;
                }

                let m_back = URF_MOVE[2][m_fb as usize] as i32;
                self.mov[(self.length1 - maxl) as usize] = m_back;
                self.valid1 = self.valid1.min(self.length1 - maxl);
                let new_ud = self.node_ud[maxl as usize];
                let new_rl = self.node_rl[maxl as usize];
                let new_fb = self.node_fb[maxl as usize];
                let ret = self.phase1opt(&new_ud, &new_rl, &new_fb, ssym & ct.move_cube_sym[m as usize], maxl - 1, axis);
                if ret == 0 {
                    return 0;
                }
            }
        }
        1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_solve_cube() {
        Search::init();
        let mut search = Search::new();
        // First cube from bench.cubes
        let cube = "UDFUURRLDBFLURRDRUUFLLFRFDBRBRLDBUDLRBBFLBBUDDFFDBUFLL";
        let sol = search.solution(cube, 21, 10000000, 100, 0);
        log::info!("Solution: {}", sol);
        assert!(!sol.starts_with("Error"), "Failed to solve: {}", sol);
        // Solution should be non-empty and have reasonable length
        let moves: Vec<&str> = sol.trim().split_whitespace().collect();
        assert!(moves.len() <= 21, "Solution too long: {} moves", moves.len());
        assert!(moves.len() >= 1, "Solution empty");
    }

    #[test]
    fn test_solve_solved_cube() {
        Search::init();
        let mut search = Search::new();
        let solved = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
        let sol = search.solution(solved, 21, 10000000, 0, 0);
        // Solved cube should give empty or very short solution
        assert!(!sol.starts_with("Error"), "Failed: {}", sol);
    }

    #[test]
    fn test_verify_invalid() {
        let mut search = Search::new();
        let invalid = "UUUUUUUUUUUUUUUUUUFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
        let sol = search.solution(invalid, 21, 1000, 0, 0);
        assert!(sol.starts_with("Error"));
    }

    /// 多解 API 基础测试：默认参数（500ms / 5 条）能产出至少 1 条解。
    ///
    /// 新语义（URF 多样性 + next 递减混合）：候选**不再保证逐条递减**，
    /// 因为 URF 多样性会在第一条 L0 长度上拿同长度的多个变体，再用 next()
    /// 找更短的，再回来拿剩余 URF 的同长度变体（路径不同对 handstep 有价值）。
    /// 保证：
    ///   1. 全部互不相同（HashSet 去重）。
    ///   2. 第 1 条是首搜结果；最短的一条 ≤ 第 1 条长度。
    #[test]
    fn test_solutions_basic() {
        Search::init();
        let cube = "UDFUURRLDBFLURRDRUUFLLFRFDBRBRLDBUDLRBBFLBBUDDFFDBUFLL";
        let mut s = Search::new();
        let res = s.solutions(cube, SearchOptions::default());
        eprintln!(
            "solutions: {} 条, 耗时 {:?}, timed_out={}",
            res.solutions.len(), res.elapsed, res.timed_out
        );
        for (i, sol) in res.solutions.iter().enumerate() {
            let n = sol.split_whitespace().count();
            eprintln!("  [{}] {}f: {}", i, n, sol);
        }
        assert!(!res.solutions.is_empty(), "至少应产出 1 条解");
        // 互不相同
        let uniq: std::collections::HashSet<_> = res.solutions.iter().collect();
        assert_eq!(uniq.len(), res.solutions.len(), "候选不应重复");
        // 最短候选 ≤ 第 1 条
        let len0 = res.solutions[0].split_whitespace().count();
        let min_len = res.solutions.iter()
            .map(|s| s.split_whitespace().count())
            .min().unwrap();
        assert!(min_len <= len0, "最短候选不应超过第 1 条");
    }

    /// 超时测试：极短 timeout 时 timed_out=true 且不会 panic。
    /// 注意：超时检查也作用于第 1 条 solution()，所以 1ms 可能连 1 条都拿不到——
    /// 这是预期行为（IDA* 第 1 条 phase2 入口就被超时拦下）。
    #[test]
    fn test_solutions_timeout_fallback() {
        Search::init();
        let cube = "UDFUURRLDBFLURRDRUUFLLFRFDBRBRLDBUDLRBBFLBBUDDFFDBUFLL";
        let mut s = Search::new();
        let res = s.solutions(cube, SearchOptions {
            timeout: Duration::from_millis(1),
            max_solutions: 5,
            ..Default::default()
        });
        eprintln!(
            "timeout=1ms: {} 条, 耗时 {:?}, timed_out={}",
            res.solutions.len(), res.elapsed, res.timed_out
        );
        // 1ms 太短，可能拿不到任何解；至少应该被标记为 timed_out。
        assert!(res.timed_out, "1ms timeout 必须被检测到");
        // 耗时不应过分超过 timeout（允许少量调度抖动）
        assert!(res.elapsed < Duration::from_millis(50),
            "elapsed {:?} 远超 1ms timeout", res.elapsed);
    }

    /// 较合理的 timeout 应当能拿到至少 1 条解。
    #[test]
    fn test_solutions_with_reasonable_timeout() {
        Search::init();
        let cube = "UDFUURRLDBFLURRDRUUFLLFRFDBRBRLDBUDLRBBFLBBUDDFFDBUFLL";
        let mut s = Search::new();
        let res = s.solutions(cube, SearchOptions {
            timeout: Duration::from_millis(200),
            max_solutions: 5,
            ..Default::default()
        });
        eprintln!(
            "timeout=200ms: {} 条, 耗时 {:?}, timed_out={}",
            res.solutions.len(), res.elapsed, res.timed_out
        );
        for (i, sol) in res.solutions.iter().enumerate() {
            eprintln!("  [{}] {}f", i, sol.split_whitespace().count());
        }
        assert!(!res.solutions.is_empty(), "200ms 应当至少产出 1 条解");
    }

    /// 探测 length_slack 对候选数量和长度分布的影响。
    /// 跑：cargo test -p robo-solver --release probe_slack -- --ignored --nocapture
    #[test]
    #[ignore]
    fn probe_slack_distribution() {
        Search::init();
        let cube = "UDFUURRLDBFLURRDRUUFLLFRFDBRBRLDBUDLRBBFLBBUDDFFDBUFLL";
        for slack in 0..=4 {
            let mut s = Search::new();
            let res = s.solutions(cube, SearchOptions {
                timeout: Duration::from_millis(100),
                max_solutions: usize::MAX,
                length_slack: slack,
                ..Default::default()
            });
            eprintln!(
                "\n--- slack={} ---  共 {} 条, {:?}, timed_out={}",
                slack, res.solutions.len(), res.elapsed, res.timed_out
            );
            let mut len_counts = std::collections::BTreeMap::<usize, usize>::new();
            for sol in &res.solutions {
                *len_counts.entry(sol.split_whitespace().count()).or_insert(0) += 1;
            }
            for (l, c) in &len_counts {
                eprintln!("  {}f × {}", l, c);
            }
        }
    }
}

