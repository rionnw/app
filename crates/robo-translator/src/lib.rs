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
        ("U'", Action::U(-90)),
        ("U",  Action::U(90)),
        ("R2", Action::R(180)),
        ("R'", Action::R(-90)),
        ("R",  Action::R(90)),
        ("x2", Action::X(180)),
        ("x'", Action::X(-90)),
        ("x",  Action::X(90)),
        ("y2", Action::Y(180)),
        ("y'", Action::Y(-90)),
        ("y",  Action::Y(90)),
    ];

    for (token, action) in candidates {
        if s.starts_with(token) {
            *s = &s[token.len()..];
            return Some(action.clone());
        }
    }
    None
}

// ===== Hardware State Machine =====

/// 臂角度归一化到 [-180, 180]
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

struct HardwareTranslator {
    u_angle: i32,
    r_angle: i32,
    u_claw: bool,  // true = 闭合
    r_claw: bool,
    commands: Vec<String>,
}

impl HardwareTranslator {
    fn new() -> Self {
        Self {
            u_angle: 0,
            r_angle: 0,
            u_claw: true,
            r_claw: true,
            commands: Vec::new(),
        }
    }

    fn emit(&mut self, cmd: &str) {
        self.commands.push(cmd.to_string());
    }

    // ===== 基础操作（只在状态变化时发指令）=====

    fn claw_u(&mut self, close: bool) {
        if self.u_claw != close {
            self.emit(if close { "CLAW_U(1);" } else { "CLAW_U(0);" });
            self.u_claw = close;
        }
    }

    fn claw_r(&mut self, close: bool) {
        if self.r_claw != close {
            self.emit(if close { "CLAW_R(1);" } else { "CLAW_R(0);" });
            self.r_claw = close;
        }
    }

    fn rotate_u(&mut self, degrees: i32) {
        if degrees != 0 {
            self.emit(&format!("ROTATE_U({:+});", degrees));
            self.u_angle = normalize_angle(self.u_angle + degrees);
        }
    }

    fn rotate_r(&mut self, degrees: i32) {
        if degrees != 0 {
            self.emit(&format!("ROTATE_R({:+});", degrees));
            self.r_angle = normalize_angle(self.r_angle + degrees);
        }
    }

    // ===== 回正（爪子竖着时需要回正）=====

    fn ensure_u_flat(&mut self) {
        if is_flat(self.u_angle) { return; }
        self.claw_r(true);    // R 夹紧固定魔方
        self.claw_u(false);   // U 松开
        let back = -self.u_angle;
        self.rotate_u(back);  // U 回正
        self.claw_u(true);    // U 夹紧
    }

    fn ensure_r_flat(&mut self) {
        if is_flat(self.r_angle) { return; }
        self.claw_u(true);    // U 夹紧固定魔方
        self.claw_r(false);   // R 松开
        let back = -self.r_angle;
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

    /// 整体 y（绕 U 轴）：U 爪松开，R 臂带整个魔方转，U 夹紧，R 竖则回正
    fn whole_y(&mut self, degrees: i32) {
        self.claw_u(false);   // U 松开
        self.claw_r(true);    // R 夹紧带魔方
        self.rotate_r(degrees);
        self.claw_u(true);    // U 夹紧固定新位置
        if !is_flat(self.r_angle) {
            self.ensure_r_flat();
        }
    }

    /// 整体 x（绕 R 轴）：R 爪松开，U 臂带整个魔方转，R 夹紧，U 竖则回正
    fn whole_x(&mut self, degrees: i32) {
        self.claw_r(false);   // R 松开
        self.claw_u(true);    // U 夹紧带魔方
        self.rotate_u(degrees);
        self.claw_r(true);    // R 夹紧固定新位置
        if !is_flat(self.u_angle) {
            self.ensure_u_flat();
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

impl Translator for BasicTranslator {
    fn translate(&self, moves: &Moves) -> Result<Steps> {
        let solution_str = moves.to_solution_string();
        let actions = parse_solution(&solution_str);

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
        assert_eq!(actions, vec![Action::RegrabR, Action::X(90), Action::U(90)]);
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
        assert_eq!(steps.commands, vec!["ROTATE_U(+90);"]);
    }

    #[test]
    fn whole_x_then_u() {
        // x → 整体绕 R 轴：R 开，U 转，R 关，U 回正，然后单层 U
        let moves = Moves::from_solution_string("(s0z1) x  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_R(0);",        // x: R 松开
            "ROTATE_U(+90);",    // x: U 带魔方转
            "CLAW_R(1);",        // x: R 夹紧
            "CLAW_U(0);",        // x: U 回正-松开
            "ROTATE_U(-90);",    // x: U 回正
            "CLAW_U(1);",        // x: U 回正-夹紧
            "ROTATE_U(+90);",    // 单层 U
        ]);
    }

    #[test]
    fn regrab_r_then_x_then_u() {
        let moves = Moves::from_solution_string("(z0s1)    (s0z1) x  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_R(0);",        // x: R 松开
            "ROTATE_U(+90);",    // x: U 带魔方转
            "CLAW_R(1);",        // x: R 夹紧
            "CLAW_U(0);",        // x: U 回正-松开
            "ROTATE_U(-90);",    // x: U 回正
            "CLAW_U(1);",        // x: U 回正-夹紧
            "ROTATE_U(+90);",    // 单层 U
        ]);
    }

    #[test]
    fn whole_y2_no_reset() {
        let moves = Moves::from_solution_string("(z2s0) y2");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_U(0);",        // y2: U 松开
            "ROTATE_R(+180);",   // y2: R 转 180°
            "CLAW_U(1);",        // y2: U 夹紧（180° 不回正）
        ]);
    }

    #[test]
    fn whole_y_then_u() {
        let moves = Moves::from_solution_string("(z1s0) y  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_U(0);",        // y: U 松开
            "ROTATE_R(+90);",    // y: R 带魔方转
            "CLAW_U(1);",        // y: U 夹紧
            "CLAW_R(0);",        // y: R 回正-松开
            "ROTATE_R(-90);",    // y: R 回正
            "CLAW_R(1);",        // y: R 回正-夹紧
            "ROTATE_U(+90);",    // 单层 U
        ]);
    }

    #[test]
    fn empty_solution() {
        let moves = Moves::from_solution_string("");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert!(steps.commands.is_empty());
    }

    #[test]
    fn full_solve_does_not_panic() {
        let raw = "(z1s0) y  (z1z0) U  (s1z2) x2 (z3z0) U' (z0z3) R' (z2s1) y2 (z0z2) R2 (z1z0) U  (z1s1) y  (z0z1) R  (z3z0) U' (s1z3) x' (z0z1) R  (z2z0) U2 (z0z1) R  (z2s0) y2 (z0z3) R' (z3z0) U' (s1z3) x' (z0z2) R2 (z2s0) y2 (z0z2) R2 (z1s1) y  (z1z0) U  (z0z2) R2 (s1z2) x2 (z1z0) U  (z0z2) R2 (s1z0)    (z1s0) y  (z0z2) R2";
        let moves = Moves::from_solution_string(raw);
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert!(!steps.commands.is_empty());
    }
}
