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

    // ===== 回正（爪子竖着时需要回正）=====

    fn ensure_u_flat(&mut self) {
        if is_flat(self.u_angle) { return; }
        self.claw_r(true);    // R 夹紧固定魔方
        self.claw_u(false);   // U 松开
        let back = pick_back_rotation(self.u_angle, self.u_accum);
        self.rotate_u(back);  // U 回正
        self.claw_u(true);    // U 夹紧
    }

    fn ensure_r_flat(&mut self) {
        if is_flat(self.r_angle) { return; }
        self.claw_u(true);    // U 夹紧固定魔方
        self.claw_r(false);   // R 松开
        let back = pick_back_rotation(self.r_angle, self.r_accum);
        self.rotate_r(back);  // R 回正
        self.claw_r(true);    // R 夹紧
    }

    // ===== 动作执行 =====

    /// 单层 U：对面(R)必须平放，两爪闭合，U 转
    fn single_u(&mut self, degrees: i32) {
        self.ensure_r_flat();
        self.claw_u(true);
        self.claw_r(true);
        self.rotate_u(degrees);
    }

    /// 单层 R：对面(U)必须平放，两爪闭合，R 转
    fn single_r(&mut self, degrees: i32) {
        self.ensure_u_flat();
        self.claw_u(true);
        self.claw_r(true);
        self.rotate_r(degrees);
    }

    /// 整体 y（绕 U 轴）：R 松开，U 臂带整个魔方转，R 夹紧，U 竖则回正
    fn whole_y(&mut self, degrees: i32) {
        self.claw_r(false);   // R 松开
        self.claw_u(true);    // U 夹紧带魔方
        self.rotate_u(degrees);
        self.claw_r(true);    // R 夹紧固定新位置
        if !is_flat(self.u_angle) {
            self.ensure_u_flat();
        }
    }

    /// 整体 x（绕 R 轴）：U 松开，R 臂带整个魔方转，U 夹紧，R 竖则回正
    fn whole_x(&mut self, degrees: i32) {
        self.claw_u(false);   // U 松开
        self.claw_r(true);    // R 夹紧带魔方
        self.rotate_r(degrees);
        self.claw_u(true);    // U 夹紧固定新位置
        if !is_flat(self.r_angle) {
            self.ensure_r_flat();
        }
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
        let moves = Moves::from_solution_string("(z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec!["ROTATE_U(-90);"]);
    }

    #[test]
    fn whole_x_then_u() {
        // x = 绕 R 轴：U 松开，R 带转，U 夹紧，R 回正
        let moves = Moves::from_solution_string("(s0z1) x  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_U(0);",        // x: U 松开
            "ROTATE_R(-90);",    // x: R 带魔方转
            "CLAW_U(1);",        // x: U 夹紧
            "CLAW_R(0);",        // x: R 回正-松开
            "ROTATE_R(+90);",    // x: R 回正
            "CLAW_R(1);",        // x: R 回正-夹紧
            "ROTATE_U(-90);",    // 单层 U
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
        ]);
    }

    #[test]
    fn whole_y2_no_reset() {
        // y2 = 绕 U 轴 180°：R 松开，U 带转 180°（不回正）
        let moves = Moves::from_solution_string("(z2s0) y2");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_R(0);",        // y2: R 松开
            "ROTATE_U(+180);",   // y2: U 带转 180°
            "CLAW_R(1);",        // y2: R 夹紧
        ]);
    }

    #[test]
    fn whole_y_then_u() {
        // y = 绕 U 轴：R 松开，U 带转，R 夹紧，U 回正
        let moves = Moves::from_solution_string("(z1s0) y  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_R(0);",        // y: R 松开
            "ROTATE_U(-90);",    // y: U 带魔方转
            "CLAW_R(1);",        // y: R 夹紧
            "CLAW_U(0);",        // y: U 回正-松开
            "ROTATE_U(+90);",    // y: U 回正
            "CLAW_U(1);",        // y: U 回正-夹紧
            "ROTATE_U(-90);",    // 单层 U
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
    fn full_solve_does_not_panic() {
        let raw = "(z1s0) y  (z1z0) U  (s1z2) x2 (z3z0) U' (z0z3) R' (z2s1) y2 (z0z2) R2 (z1z0) U  (z1s1) y  (z0z1) R  (z3z0) U' (s1z3) x' (z0z1) R  (z2z0) U2 (z0z1) R  (z2s0) y2 (z0z3) R' (z3z0) U' (s1z3) x' (z0z2) R2 (z2s0) y2 (z0z2) R2 (z1s1) y  (z1z0) U  (z0z2) R2 (s1z2) x2 (z1z0) U  (z0z2) R2 (s1z0)    (z1s0) y  (z0z2) R2";
        let moves = Moves::from_solution_string(raw);
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert!(!steps.commands.is_empty());
    }
}
