use anyhow::Result;
use robo_core::{Moves, Steps, Translator};

// ===== Action Parser =====

/// 从 2L solver 输出中提取动作序列。
/// 括号内 (z1s0) 等是姿态描述，不是指令。
/// 实际动作是后面的 U/U2/U'/R/R2/R'/x/x2/x'/y/y2/y' 或无（regrab）。
#[derive(Debug, Clone, PartialEq)]
enum Action {
    U(i32),       // 单层 U: 90, 180, -90
    R(i32),       // 单层 R: 90, 180, -90
    X(i32),       // 整体 x: 90, 180, -90
    Y(i32),       // 整体 y: 90, 180, -90
    RegrabU,      // U 爪 regrab
    RegrabR,      // R 爪 regrab
}

fn parse_solution(solution: &str) -> Vec<Action> {
    let mut actions = Vec::new();
    let mut s = solution.trim();

    while !s.is_empty() {
        s = s.trim_start();
        if s.is_empty() {
            break;
        }

        // 跳过括号内的姿态描述
        if s.starts_with('(') {
            if let Some(end) = s.find(')') {
                let inside = &s[1..end];
                s = s[end + 1..].trim_start();

                // 如果括号后没有动作标记，判断 regrab 类型
                if s.is_empty() || s.starts_with('(') {
                    // 根据括号内容判断是哪侧 regrab
                    // s1 在右边 (z0s1) → R regrab; s1 在左边 (s1z0) → U regrab
                    if inside.starts_with("s1") {
                        actions.push(Action::RegrabU);
                    } else {
                        actions.push(Action::RegrabR);
                    }
                    continue;
                }

                // 提取动作标记
                if let Some(action) = parse_action_token(&mut s) {
                    actions.push(action);
                }
            } else {
                s = &s[1..]; // malformed, skip
            }
        } else {
            s = &s[1..]; // skip unrecognized
        }
    }

    actions
}

fn parse_action_token(s: &mut &str) -> Option<Action> {
    let candidates: &[(&str, Action)] = &[
        ("U2", Action::U(180)),
        ("U'", Action::U(90)),
        ("U",  Action::U(-90)),
        ("R2", Action::R(180)),
        ("R'", Action::R(90)),
        ("R",  Action::R(-90)),
        ("x2", Action::X(180)),
        ("x'", Action::X(90)),
        ("x",  Action::X(-90)),
        ("y2", Action::Y(180)),
        ("y'", Action::Y(90)),
        ("y",  Action::Y(-90)),
    ];

    for (token, action) in candidates {
        if s.starts_with(token) {
            *s = &s[token.len()..];
            return Some(action.clone());
        }
    }
    None
}

// ===== Action 序列预处理：同轴相邻动作合并 / 抵消 =====

fn deg_mod360(d: i32) -> i32 {
    let mut v = d % 360;
    if v < 0 { v += 360; }
    v
}

/// 把同种 kind 的相邻动作角度合并。
/// 注意：X / Y 整体旋转会改变魔方朝向，因此不跨它们合并 U/R。
/// 但相邻 U+U / R+R / X+X / Y+Y 是同一物理动作可以累加。
fn simplify_actions(actions: Vec<Action>) -> Vec<Action> {
    let mut out: Vec<Action> = Vec::with_capacity(actions.len());
    for a in actions {
        let merged = match (out.last(), &a) {
            (Some(Action::U(prev)), Action::U(cur)) => Some(Action::U(*prev + cur)),
            (Some(Action::R(prev)), Action::R(cur)) => Some(Action::R(*prev + cur)),
            (Some(Action::X(prev)), Action::X(cur)) => Some(Action::X(*prev + cur)),
            (Some(Action::Y(prev)), Action::Y(cur)) => Some(Action::Y(*prev + cur)),
            _ => None,
        };
        match merged {
            Some(new_act) => {
                out.pop();
                // 归一化到 (-180, 180]
                let normalize = |d: i32| -> Option<i32> {
                    let mut v = deg_mod360(d);
                    if v > 180 { v -= 360; }
                    if v == 0 { None } else { Some(v) }
                };
                let normalized = match new_act {
                    Action::U(d) => normalize(d).map(Action::U),
                    Action::R(d) => normalize(d).map(Action::R),
                    Action::X(d) => normalize(d).map(Action::X),
                    Action::Y(d) => normalize(d).map(Action::Y),
                    _ => unreachable!(),
                };
                if let Some(act) = normalized {
                    out.push(act);
                }
            }
            None => out.push(a),
        }
    }
    out
}

// ===== Hardware State Machine =====

/// 臂角度归一化到 (-180, 180]
fn normalize_angle(a: i32) -> i32 {
    let mut v = a % 360;
    if v > 180 { v -= 360; }
    if v <= -180 { v += 360; }
    v
}

/// 臂是否处于"平放"位置（0° 或 ±180°）
fn is_flat(angle: i32) -> bool {
    angle == 0 || angle.abs() == 180
}

/// 选择回正旋转量：限定单次旋转 |delta| <= 180，
/// 在符合的候选中挑能让累计 accum 朝 0 收敛的方向。
fn pick_back_rotation(angle: i32, accum: i32) -> i32 {
    // 当前 angle 已是非平放（normalize 在 ±90），目标平放位置是 0 或 ±180。
    // 单次 |delta| ≤ 180 的候选：
    //   到 0:    -angle  (|delta| = 90)
    //   到 +180: 180 - angle  (angle= -90 → +270 ❌; angle= +90 → +90 ✓)
    //   到 -180: -180 - angle (angle= +90 → -270 ❌; angle= -90 → -90 ✓)
    // 即：要么走 -angle 到 0，要么走 -angle 的反向到 ±180（绝对值同为 90）。
    let to_zero = -angle;
    // 反方向同样 90°：angle=+90 → +90 到 180；angle=-90 → -90 到 -180
    let to_flip = if angle > 0 { 180 - angle } else { -180 - angle };
    debug_assert_eq!(to_zero.abs(), 90);
    debug_assert_eq!(to_flip.abs(), 90);
    // 两者 |delta| 相同，挑让 accum 更接近 0 的
    if (accum + to_zero).abs() <= (accum + to_flip).abs() {
        to_zero
    } else {
        to_flip
    }
}

struct HardwareTranslator {
    u_angle: i32,
    r_angle: i32,
    /// 累计转动度数（不归一化），用于回正时挑选让累计绝对值更小的方向
    u_accum: i32,
    r_accum: i32,
    u_claw: bool,  // true = 闭合
    r_claw: bool,
    commands: Vec<String>,
}

impl HardwareTranslator {
    fn new() -> Self {
        Self {
            u_angle: 0,
            r_angle: 0,
            u_accum: 0,
            r_accum: 0,
            u_claw: true,
            r_claw: true,
            commands: Vec::new(),
        }
    }

    /// 规范化单次旋转量到允许集合 {-90, +90, +180}（180° 统一为正）。
    /// 输入应是 90 的倍数；其他值会被取模到 [-180, 180]。
    fn canonical_degrees(d: i32) -> i32 {
        let mut v = d % 360;
        if v > 180 { v -= 360; }
        if v < -180 { v += 360; }
        if v == -180 { v = 180; }
        v
    }

    fn rotate_kind(s: &str) -> Option<(&'static str, i32)> {
        for k in ["ROTATE_U", "ROTATE_R"] {
            if let Some(rest) = s.strip_prefix(k) {
                let inner = rest.trim_start_matches('(').trim_end_matches(';').trim_end_matches(')');
                if let Ok(v) = inner.trim().parse::<i32>() {
                    return Some((k, v));
                }
            }
        }
        None
    }

    fn claw_kind(s: &str) -> Option<(&'static str, i32)> {
        for k in ["CLAW_U", "CLAW_R"] {
            if let Some(rest) = s.strip_prefix(k) {
                let inner = rest.trim_start_matches('(').trim_end_matches(';').trim_end_matches(')');
                if let Ok(v) = inner.trim().parse::<i32>() {
                    return Some((k, v));
                }
            }
        }
        None
    }

    // ===== 基础操作（在 emit 时即与上一条同类指令合并）=====

    fn claw_u(&mut self, close: bool) {
        if self.u_claw == close { return; }
        self.u_claw = close;
        let want = if close { 1 } else { 0 };
        // 检查上一条 CLAW_U：与本次相反则抵消（且物理状态变化也已抵消）
        if let Some(last) = self.commands.last() {
            if let Some((k, v)) = Self::claw_kind(last) {
                if k == "CLAW_U" && v != want {
                    self.commands.pop();
                    return;
                }
            }
        }
        self.commands.push(format!("CLAW_U({});", want));
    }

    fn claw_r(&mut self, close: bool) {
        if self.r_claw == close { return; }
        self.r_claw = close;
        let want = if close { 1 } else { 0 };
        if let Some(last) = self.commands.last() {
            if let Some((k, v)) = Self::claw_kind(last) {
                if k == "CLAW_R" && v != want {
                    self.commands.pop();
                    return;
                }
            }
        }
        self.commands.push(format!("CLAW_R({});", want));
    }

    fn rotate_u(&mut self, degrees: i32) {
        let d = Self::canonical_degrees(degrees);
        if d == 0 { return; }
        self.u_angle = normalize_angle(self.u_angle + d);
        self.u_accum += d;
        // 与上一条 ROTATE_U 合并
        if let Some(last) = self.commands.last() {
            if let Some((k, v)) = Self::rotate_kind(last) {
                if k == "ROTATE_U" {
                    self.commands.pop();
                    let merged = Self::canonical_degrees(v + d);
                    if merged != 0 {
                        self.commands.push(format!("ROTATE_U({:+});", merged));
                    }
                    return;
                }
            }
        }
        self.commands.push(format!("ROTATE_U({:+});", d));
    }

    fn rotate_r(&mut self, degrees: i32) {
        let d = Self::canonical_degrees(degrees);
        if d == 0 { return; }
        self.r_angle = normalize_angle(self.r_angle + d);
        self.r_accum += d;
        if let Some(last) = self.commands.last() {
            if let Some((k, v)) = Self::rotate_kind(last) {
                if k == "ROTATE_R" {
                    self.commands.pop();
                    let merged = Self::canonical_degrees(v + d);
                    if merged != 0 {
                        self.commands.push(format!("ROTATE_R({:+});", merged));
                    }
                    return;
                }
            }
        }
        self.commands.push(format!("ROTATE_R({:+});", d));
    }

    // ===== 安全包装 =====
    // 不变量：任何时候至少一只爪闭合（否则魔方掉落）

    /// 松开 U 爪：必须先确保 R 闭合
    fn open_u(&mut self) {
        if !self.u_claw { return; }
        self.claw_r(true);
        self.claw_u(false);
    }

    /// 松开 R 爪：必须先确保 U 闭合
    fn open_r(&mut self) {
        if !self.r_claw { return; }
        self.claw_u(true);
        self.claw_r(false);
    }

    // ===== 回正（爪子竖着时需要回正）=====
    // 回正时该爪松开（另一爪夹住魔方固定），然后空臂转回平放位置（0 或 ±180 任一）

    fn ensure_u_flat(&mut self) {
        if is_flat(self.u_angle) { return; }
        self.open_u();        // 安全松 U（含先夹紧 R）
        let back = pick_back_rotation(self.u_angle, self.u_accum);
        self.rotate_u(back);
        self.claw_u(true);    // 回正后夹紧 U
    }

    fn ensure_r_flat(&mut self) {
        if is_flat(self.r_angle) { return; }
        self.open_r();
        let back = pick_back_rotation(self.r_angle, self.r_accum);
        self.rotate_r(back);
        self.claw_r(true);
    }

    // ===== 动作执行 =====
    //
    // 约束（运行时不变量）：
    //   约束1：两爪不能同时松开（由 open_u/open_r 包装保证）
    //   约束2：两爪不能同时竖起 —— 等价于「任何转动开始前，对侧爪必须平放」
    //
    // 优化策略（懒惰回正）：
    //   每个动作只保证「入口约束」满足——即在自己要转之前，对侧爪平放。
    //   动作出口**不主动回正自己**，让下一个动作按需补：
    //     - 下一个动作如果也转同一爪 → 完全不需要回正（可累积到 ±180 自然平放）
    //     - 下一个动作转对侧爪 → 由对侧动作入口的 ensure_*_flat 触发回正
    //   这把多次相同方向单层动作之间的"回正→再转"指令对消除。
    //
    // 对于 180° 转动，落点本身是平放，ensure_*_flat 自动 no-op。

    /// 单层 U：R 必须平放（入口），两爪闭合，U 带顶层转 d。
    /// 不在出口主动回正 U——由下一动作（若需 R 转动）按需触发。
    fn single_u(&mut self, degrees: i32) {
        self.ensure_r_flat();
        self.claw_u(true);
        self.claw_r(true);
        self.rotate_u(degrees);
    }

    /// 单层 R：U 必须平放（入口），两爪闭合，R 带右层转 d。
    fn single_r(&mut self, degrees: i32) {
        self.ensure_u_flat();
        self.claw_u(true);
        self.claw_r(true);
        self.rotate_r(degrees);
    }

    /// 整体 y（绕 U 轴）：U 夹紧带转 d，R 松开。
    /// 入口要求 R 平放（约束2，R 即将停在松开但角度不变状态）；
    /// 但事实上后续动作若需要 R 转动，会先 ensure_r_flat（其内部 open_r 已含夹 U 保护）；
    /// 出口 R 保持松开状态（懒惰），由下一动作前置 claw_r 闭合即可。
    fn whole_y(&mut self, degrees: i32) {
        self.ensure_r_flat();
        self.open_r();                  // 内含 claw_u(true) 保护（约束1）
        self.rotate_u(degrees);         // U 带整魔方转
        // 出口：U 可能竖起、R 松开。下一动作按需补 ensure_*_flat / claw_*(true)。
    }

    /// 整体 x（绕 R 轴）：R 夹紧带转 d，U 松开。
    fn whole_x(&mut self, degrees: i32) {
        self.ensure_u_flat();
        self.open_u();
        self.rotate_r(degrees);
    }

    /// 在序列末尾收尾：确保两爪都平放且都闭合（机器停在安全姿态）。
    fn finalize(&mut self) {
        self.ensure_u_flat();
        self.ensure_r_flat();
        self.claw_u(true);
        self.claw_r(true);
    }

    fn execute(&mut self, action: &Action) {
        match action {
            Action::U(d) => self.single_u(*d),
            Action::R(d) => self.single_r(*d),
            Action::Y(d) => self.whole_y(*d),
            Action::X(d) => self.whole_x(*d),
            Action::RegrabU | Action::RegrabR => {} // regrab 是 solver 内部状态，不生成硬件指令
        }
    }
}

// ===== Public API =====

#[derive(Clone, Debug, Default)]
pub struct BasicTranslator;

impl BasicTranslator {
    pub fn new() -> Self { Self }
}

/// 邻接合并 / 抵消：
impl Translator for BasicTranslator {
    fn translate(&self, moves: &Moves) -> Result<Steps> {
        let solution_str = moves.to_solution_string();
        let raw_actions = parse_solution(&solution_str);
        // 反复简化直至稳定（例如 U2 U2 → ε 后可能让外层动作变得可合并）
        let mut actions = raw_actions;
        loop {
            let next = simplify_actions(actions.clone());
            if next == actions { break; }
            actions = next;
        }

        if actions.is_empty() {
            return Ok(Steps { commands: vec![], encoded: String::new() });
        }

        let mut hw = HardwareTranslator::new();
        for action in &actions {
            hw.execute(action);
        }
        hw.finalize();  // 收尾：确保机器停在两爪闭合+平放的安全姿态

        let commands = hw.commands;
        let encoded = commands.join(" ");
        Ok(Steps { commands, encoded })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_actions() {
        let raw = "(z0s1)    (s0z1) x  (z1z0) U";
        let actions = parse_solution(raw);
        assert_eq!(actions, vec![Action::RegrabR, Action::X(-90), Action::U(-90)]);
    }

    #[test]
    fn parse_regrab_u() {
        let actions = parse_solution("(s1z0)");
        assert_eq!(actions, vec![Action::RegrabU]);
    }

    #[test]
    fn single_u_move() {
        // 单层 U(-90)：转 U → 出口懒惰不回正 → finalize 收尾回正。
        let moves = Moves::from_solution_string("(z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "ROTATE_U(-90);",   // 带魔方
            "CLAW_U(0);",       // finalize: 松 U
            "ROTATE_U(+90);",   // finalize: 空臂回正
            "CLAW_U(1);",       // finalize: 夹回
        ]);
    }

    #[test]
    fn whole_x_then_u() {
        // 懒惰策略：whole_x 末尾不回 R，single_u 入口 ensure_r_flat 触发回正
        let moves = Moves::from_solution_string("(s0z1) x  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_U(0);",        // whole_x: U 松
            "ROTATE_R(-90);",    // R 带魔方转
            "CLAW_U(1);",        // single_u 入口 ensure_r_flat→open_r 内 claw_u(1)
            "CLAW_R(0);",        // open_r 内 claw_r(0)
            "ROTATE_R(+90);",    // 空臂回正 R
            "CLAW_R(1);",        // 夹回 R
            "ROTATE_U(-90);",    // single_u 转
            "CLAW_U(0);",        // finalize: 回正 U
            "ROTATE_U(+90);",
            "CLAW_U(1);",
        ]);
    }

    #[test]
    fn regrab_r_then_x_then_u() {
        let moves = Moves::from_solution_string("(z0s1)    (s0z1) x  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_U(0);",
            "ROTATE_R(-90);",
            "CLAW_U(1);",
            "CLAW_R(0);",
            "ROTATE_R(+90);",
            "CLAW_R(1);",
            "ROTATE_U(-90);",
            "CLAW_U(0);",
            "ROTATE_U(+90);",
            "CLAW_U(1);",
        ]);
    }

    #[test]
    fn whole_y2_no_reset() {
        // y2：R 松 → U 带转 180°（落到 ±180 平放）→ finalize 把 R 夹回
        let moves = Moves::from_solution_string("(z2s0) y2");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_R(0);",
            "ROTATE_U(+180);",
            "CLAW_R(1);",        // finalize: R 夹回（不变量）
        ]);
    }

    #[test]
    fn whole_y_then_u() {
        // 关键优化：y(-90) 带 U 转 -90（U=-90 竖起），
        // 然后 single_u(-90) 又带 U 转 -90 → U 累积 -180 = 自然平放！
        // finalize 时 U 已平放、R 已闭合，无需任何额外指令。
        let moves = Moves::from_solution_string("(z1s0) y  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_R(0);",        // whole_y: R 松
            "ROTATE_U(-90);",    // U 带魔方转 (U=-90)
            "CLAW_R(1);",        // single_u 入口 claw_r(1)
            "ROTATE_U(-90);",    // single_u 带魔方转 (U=-180=平放)
        ]);
    }

    #[test]
    fn empty_solution() {
        let moves = Moves::from_solution_string("");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert!(steps.commands.is_empty());
    }

    #[test]
    fn simplify_cancels_inverse() {
        // U U' 抵消
        let actions = simplify_actions(vec![Action::U(-90), Action::U(90)]);
        assert!(actions.is_empty());
    }

    #[test]
    fn simplify_merges_double() {
        // U U → U2 (180°)
        let actions = simplify_actions(vec![Action::U(-90), Action::U(-90)]);
        assert_eq!(actions.len(), 1);
        match actions[0] {
            Action::U(d) => assert!(d.abs() == 180),
            _ => panic!("expected Action::U"),
        }
    }

    #[test]
    fn merge_adjacent_claw_cancels() {
        // 直接连续 close→open（通过 HardwareTranslator 公共方法）应抵消
        let mut hw = HardwareTranslator::new();
        hw.claw_r(false);
        hw.claw_r(true);
        // 初始已是 CLOSED；先 open→close 应产生 CLAW_R(0); 然后被抵消为空
        assert!(hw.commands.is_empty());
    }

    #[test]
    fn merge_adjacent_rotate_combines() {
        let mut hw = HardwareTranslator::new();
        hw.rotate_u(90);
        hw.rotate_u(90);
        assert_eq!(hw.commands, vec!["ROTATE_U(+180);"]);
    }

    #[test]
    fn merge_adjacent_rotate_cancels() {
        let mut hw = HardwareTranslator::new();
        hw.rotate_r(-90);
        hw.rotate_r(90);
        assert!(hw.commands.is_empty());
    }

    #[test]
    fn no_oversize_rotation() {
        // 任何单条 ROTATE 角度必须在 {-90, +90, +180} 集合内
        let mut hw = HardwareTranslator::new();
        hw.rotate_u(270);   // 应当被规范化到 -90
        assert_eq!(hw.commands, vec!["ROTATE_U(-90);"]);
        hw.rotate_u(-270);  // 累加到 0，被抵消
        assert!(hw.commands.is_empty());
    }

    #[test]
    fn pick_back_minimizes_accum() {
        // accum 偏正时应选负方向回正
        assert_eq!(pick_back_rotation(90, 270), -90);  // -90 → accum=180, +90 → accum=360
        // accum 偏负时应选正方向
        assert_eq!(pick_back_rotation(-90, -270), 90);
    }

    #[test]
    fn invariant_after_single_u_is_flat() {
        // 懒惰策略下 single_u 出口不保证 U 平放，但 R 必须平放（约束2 入口前置）。
        let mut hw = HardwareTranslator::new();
        hw.single_u(-90);
        assert!(is_flat(hw.r_angle), "single_u 出口 R 非平放: {}", hw.r_angle);
        // finalize 后两爪都平放
        hw.finalize();
        assert!(is_flat(hw.u_angle));
        assert!(is_flat(hw.r_angle));
        assert!(hw.u_claw && hw.r_claw);
    }

    #[test]
    fn invariant_after_whole_y_both_flat_and_closed() {
        // whole_y 出口：R 平放（入口已保证），U 可能竖起；finalize 后两爪闭合+平放。
        let mut hw = HardwareTranslator::new();
        hw.whole_y(-90);
        assert!(is_flat(hw.r_angle));
        hw.finalize();
        assert!(is_flat(hw.u_angle));
        assert!(is_flat(hw.r_angle));
        assert!(hw.u_claw);
        assert!(hw.r_claw);
    }

    #[test]
    fn invariant_after_whole_x_both_flat_and_closed() {
        let mut hw = HardwareTranslator::new();
        hw.whole_x(90);
        assert!(is_flat(hw.u_angle));
        hw.finalize();
        assert!(is_flat(hw.u_angle));
        assert!(is_flat(hw.r_angle));
        assert!(hw.u_claw);
        assert!(hw.r_claw);
    }

    #[test]
    fn full_solve_does_not_panic() {
        let raw = "(z1s0) y  (z1z0) U  (s1z2) x2 (z3z0) U' (z0z3) R' (z2s1) y2 (z0z2) R2 (z1z0) U  (z1s1) y  (z0z1) R  (z3z0) U' (s1z3) x' (z0z1) R  (z2z0) U2 (z0z1) R  (z2s0) y2 (z0z3) R' (z3z0) U' (s1z3) x' (z0z2) R2 (z2s0) y2 (z0z2) R2 (z1s1) y  (z1z0) U  (z0z2) R2 (s1z2) x2 (z1z0) U  (z0z2) R2 (s1z0)    (z1s0) y  (z0z2) R2";
        let moves = Moves::from_solution_string(raw);
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert!(!steps.commands.is_empty());
    }

    /// 模拟硬件，逐条执行指令并断言两个约束永远成立：
    ///   约束1：两爪不能同时松开
    ///   约束2：两爪不能同时竖起
    fn simulate_and_check(commands: &[String]) {
        let mut u_angle = 0i32;
        let mut r_angle = 0i32;
        let mut u_close = true;
        let mut r_close = true;
        for (i, c) in commands.iter().enumerate() {
            if let Some((k, v)) = HardwareTranslator::claw_kind(c) {
                let close = v != 0;
                if k == "CLAW_U" { u_close = close; } else { r_close = close; }
            } else if let Some((k, v)) = HardwareTranslator::rotate_kind(c) {
                if k == "ROTATE_U" {
                    u_angle = normalize_angle(u_angle + v);
                } else {
                    r_angle = normalize_angle(r_angle + v);
                }
            }
            // 约束1
            assert!(u_close || r_close,
                "约束1违反 @第{}步({}): 两爪同时松开", i, c);
            // 约束2
            assert!(is_flat(u_angle) || is_flat(r_angle),
                "约束2违反 @第{}步({}): U={} R={} 两爪同时竖起", i, c, u_angle, r_angle);
        }
    }

    #[test]
    fn full_solve_respects_both_constraints() {
        let raw = "(z1s0) y  (z1z0) U  (s1z2) x2 (z3z0) U' (z0z3) R' (z2s1) y2 (z0z2) R2 (z1z0) U  (z1s1) y  (z0z1) R  (z3z0) U' (s1z3) x' (z0z1) R  (z2z0) U2 (z0z1) R  (z2s0) y2 (z0z3) R' (z3z0) U' (s1z3) x' (z0z2) R2 (z2s0) y2 (z0z2) R2 (z1s1) y  (z1z0) U  (z0z2) R2 (s1z2) x2 (z1z0) U  (z0z2) R2 (s1z0)    (z1s0) y  (z0z2) R2";
        let moves = Moves::from_solution_string(raw);
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        simulate_and_check(&steps.commands);
        eprintln!("总指令数: {}", steps.commands.len());
    }

    #[test]
    fn single_combos_respect_constraints() {
        // 各种动作组合检查
        let cases = [
            "(z1z0) U (z1z0) R (z1z0) U' (z1z0) R'",
            "(z1s0) y (z1z0) U (s0z1) x (z1z0) R",
            "(z2s0) y2 (s1z2) x2 (z1z0) U2 (z1z0) R2",
        ];
        for raw in cases {
            let moves = Moves::from_solution_string(raw);
            let steps = BasicTranslator::new().translate(&moves).unwrap();
            simulate_and_check(&steps.commands);
        }
    }
}
