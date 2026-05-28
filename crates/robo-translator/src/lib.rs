use anyhow::{bail, Result};
use robo_core::{Moves, Steps, Translator};

/// MOVE2STR patterns from the 2L solver output (20 leg-moves).
const MOVE2STR: [&str; 20] = [
    "(z1z0) U",  "(z2z0) U2", "(z3z0) U'",
    "(z0z1) R",  "(z0z2) R2", "(z0z3) R'",
    "(z1s0) y",  "(z2s0) y2", "(z3s0) y'",
    "(s0z1) x",  "(s0z2) x2", "(s0z3) x'",
    "(z0s1)",    "(s1z0)",
    "(z1s1) y",  "(z2s1) y2", "(z3s1) y'",
    "(s1z1) x",  "(s1z2) x2", "(s1z3) x'",
];

/// Leg state transitions: NEXT_STATE[current_leg][move_index] → new leg (-1 = invalid).
const NEXT_STATE: [[i8; 20]; 3] = [
    [ 1,  0,  1,  2,  0,  2,  1,  0,  1,  2,  0,  2,  2,  1, -1,  2, -1, -1,  1, -1],
    [ 0,  1,  0, -1, -1, -1,  0,  1,  0, -1,  1, -1, -1,  0,  2, -1,  2,  2,  0,  2],
    [-1, -1, -1,  0,  2,  0, -1,  2, -1,  0,  2,  0,  0, -1,  1,  0,  1,  1, -1,  1],
];

// ===== Hardware State Machine =====

/// 臂角度状态（相对于平放位置的偏移）
#[derive(Clone, Copy, Debug, PartialEq)]
struct ArmState {
    u_angle: i32,  // U 臂角度: 0=平放, 90/-90/180
    r_angle: i32,  // R 臂角度: 0=平放, 90/-90/180
    u_claw: bool,  // U 爪: true=闭合, false=打开
    r_claw: bool,  // R 爪: true=闭合, false=打开
}

impl ArmState {
    fn new() -> Self {
        Self {
            u_angle: 0,
            r_angle: 0,
            u_claw: true,
            r_claw: true,
        }
    }
}

/// 将角度归一化到 [-180, 180]
fn normalize_angle(a: i32) -> i32 {
    let mut v = a % 360;
    if v > 180 { v -= 360; }
    if v <= -180 { v += 360; }
    v
}

/// 生成硬件指令序列
struct HardwareTranslator {
    state: ArmState,
    commands: Vec<String>,
}

impl HardwareTranslator {
    fn new() -> Self {
        Self {
            state: ArmState::new(),
            commands: Vec::new(),
        }
    }

    fn emit(&mut self, cmd: &str) {
        self.commands.push(cmd.to_string());
    }

    // ===== 基础操作 =====

    fn claw_u_open(&mut self) {
        if self.state.u_claw {
            self.emit("CLAW_U(0);");
            self.state.u_claw = false;
        }
    }

    fn claw_u_close(&mut self) {
        if !self.state.u_claw {
            self.emit("CLAW_U(1);");
            self.state.u_claw = true;
        }
    }

    fn claw_r_open(&mut self) {
        if self.state.r_claw {
            self.emit("CLAW_R(0);");
            self.state.r_claw = false;
        }
    }

    fn claw_r_close(&mut self) {
        if !self.state.r_claw {
            self.emit("CLAW_R(1);");
            self.state.r_claw = true;
        }
    }

    fn rotate_u(&mut self, degrees: i32) {
        if degrees != 0 {
            self.emit(&format!("ROTATE_U({:+});", degrees));
            self.state.u_angle = normalize_angle(self.state.u_angle + degrees);
        }
    }

    fn rotate_r(&mut self, degrees: i32) {
        if degrees != 0 {
            self.emit(&format!("ROTATE_R({:+});", degrees));
            self.state.r_angle = normalize_angle(self.state.r_angle + degrees);
        }
    }

    // ===== 回正操作 =====

    /// 确保 U 臂回到平放（angle=0 或 180），需要 R 爪闭合固定魔方
    fn ensure_u_flat(&mut self) {
        if self.state.u_angle == 0 || self.state.u_angle.abs() == 180 {
            return;
        }
        // R 爪必须闭合来固定魔方
        self.claw_r_close();
        // U 爪松开（不带魔方）
        self.claw_u_open();
        // 反转回正
        let back = -self.state.u_angle;
        self.rotate_u(back);
        // U 爪夹紧
        self.claw_u_close();
    }

    /// 确保 R 臂回到平放（angle=0 或 180），需要 U 爪闭合固定魔方
    fn ensure_r_flat(&mut self) {
        if self.state.r_angle == 0 || self.state.r_angle.abs() == 180 {
            return;
        }
        // U 爪必须闭合来固定魔方
        self.claw_u_close();
        // R 爪松开（不带魔方）
        self.claw_r_open();
        // 反转回正
        let back = -self.state.r_angle;
        self.rotate_r(back);
        // R 爪夹紧
        self.claw_r_close();
    }

    // ===== 高层动作 =====

    /// 单层 U 旋转：R 臂必须平放，两爪闭合，U 臂转动
    fn single_u(&mut self, degrees: i32) {
        // 确保 R 臂平放（对面臂不挡）
        self.ensure_r_flat();
        // 两爪都要闭合
        self.claw_u_close();
        self.claw_r_close();
        // U 臂转动（带动 U 层）
        self.rotate_u(degrees);
        // 180° 对称位置不需要回正，±90° 后面由 ensure 按需处理
    }

    /// 单层 R 旋转：U 臂必须平放，两爪闭合，R 臂转动
    fn single_r(&mut self, degrees: i32) {
        // 确保 U 臂平放
        self.ensure_u_flat();
        // 两爪都要闭合
        self.claw_u_close();
        self.claw_r_close();
        // R 臂转动（带动 R 层）
        self.rotate_r(degrees);
    }

    /// 整体 y 旋转（U 轴带整个魔方转）：R 爪松开，U 转，R 夹，180°不回正
    fn whole_y(&mut self, degrees: i32) {
        // R 爪松开
        self.claw_r_open();
        // U 爪闭合（带着魔方转）
        self.claw_u_close();
        // U 臂转动（整体旋转）
        self.rotate_u(degrees);
        // R 爪夹紧（固定魔方新位置）
        self.claw_r_close();
        // 180° 不需要回正（对称位置），±90° 需要回正
        if degrees.abs() != 180 {
            self.ensure_u_flat();
        }
    }

    /// 整体 x 旋转（R 轴带整个魔方转）：U 爪松开，R 转，U 夹，180°不回正
    fn whole_x(&mut self, degrees: i32) {
        // U 爪松开
        self.claw_u_open();
        // R 爪闭合（带着魔方转）
        self.claw_r_close();
        // R 臂转动（整体旋转）
        self.rotate_r(degrees);
        // U 爪夹紧（固定魔方新位置）
        self.claw_u_close();
        // 180° 不需要回正
        if degrees.abs() != 180 {
            self.ensure_r_flat();
        }
    }

    /// Regrab R 爪（toggle）
    fn regrab_r(&mut self) {
        self.claw_r_open();
        self.claw_r_close();
    }

    /// Regrab U 爪（toggle）
    fn regrab_u(&mut self) {
        self.claw_u_open();
        self.claw_u_close();
    }

    // ===== 执行 2L Move =====

    fn execute_move(&mut self, m: usize) {
        match m {
            // Move 0-2: 单层 U
            0 => self.single_u(90),
            1 => self.single_u(180),
            2 => self.single_u(-90),
            // Move 3-5: 单层 R
            3 => self.single_r(90),
            4 => self.single_r(180),
            5 => self.single_r(-90),
            // Move 6-8: 整体 y（U 轴带整体，R 松开）
            6 => self.whole_y(90),
            7 => self.whole_y(180),
            8 => self.whole_y(-90),
            // Move 9-11: 整体 x（R 轴带整体，U 松开）
            9  => self.whole_x(90),
            10 => self.whole_x(180),
            11 => self.whole_x(-90),
            // Move 12: R 爪 regrab
            12 => self.regrab_r(),
            // Move 13: U 爪 regrab
            13 => self.regrab_u(),
            // Move 14-16: y 旋转 + s1（同 whole_y，从不同 leg 状态）
            14 => self.whole_y(90),
            15 => self.whole_y(180),
            16 => self.whole_y(-90),
            // Move 17-19: x 旋转 + s1（同 whole_x，从不同 leg 状态）
            17 => self.whole_x(90),
            18 => self.whole_x(180),
            19 => self.whole_x(-90),
            _ => {}
        }
    }
}

// ===== Parser =====

fn parse_solution(solution: &str) -> Vec<usize> {
    let mut moves = Vec::new();
    let mut s = solution.trim();
    while !s.is_empty() {
        s = s.trim_start();
        if s.is_empty() {
            break;
        }
        let mut found = false;
        for (idx, pattern) in MOVE2STR.iter().enumerate() {
            let pat = pattern.trim_end();
            if s.starts_with(pat) {
                moves.push(idx);
                s = &s[pat.len()..];
                found = true;
                break;
            }
        }
        if !found {
            s = &s[1..];
        }
    }
    moves
}

// ===== Public API =====

#[derive(Clone, Debug, Default)]
pub struct BasicTranslator;

impl BasicTranslator {
    pub fn new() -> Self {
        Self
    }
}

impl Translator for BasicTranslator {
    fn translate(&self, moves: &Moves) -> Result<Steps> {
        let solution_str = moves.to_solution_string();
        let move_indices = parse_solution(&solution_str);

        if move_indices.is_empty() {
            return Ok(Steps {
                commands: vec![],
                encoded: String::new(),
            });
        }

        // 验证 leg state 合法性
        let mut leg: usize = 0;
        for (step, &m) in move_indices.iter().enumerate() {
            if m >= 20 {
                bail!("invalid move index {m} at step {}", step + 1);
            }
            let next_leg = NEXT_STATE[leg][m];
            if next_leg == -1 {
                bail!(
                    "invalid move {m} ({}) in leg state {leg} at step {}",
                    MOVE2STR[m],
                    step + 1
                );
            }
            leg = next_leg as usize;
        }

        // 生成硬件指令
        let mut hw = HardwareTranslator::new();

        // 初始化：两爪闭合
        hw.emit("CLAW_U(1);");
        hw.emit("CLAW_R(1);");

        for &m in &move_indices {
            hw.execute_move(m);
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
    fn single_u_move() {
        let moves = Moves::from_solution_string("(z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        // 初始化 + 直接转（两臂已平）
        assert_eq!(steps.commands, vec![
            "CLAW_U(1);", "CLAW_R(1);",  // init
            "ROTATE_U(+90);",             // 单层 U
        ]);
    }

    #[test]
    fn whole_y_then_u() {
        // (z1s0) y → 整体 y 旋转 90°，需要回正
        // (z1z0) U → 单层 U
        let moves = Moves::from_solution_string("(z1s0) y  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_U(1);", "CLAW_R(1);",    // init
            "CLAW_R(0);",                    // y: R 松开
            "ROTATE_U(+90);",                // y: U 带魔方转
            "CLAW_R(1);",                    // y: R 夹紧
            "CLAW_U(0);",                    // y: U 松开回正
            "ROTATE_U(-90);",                // y: U 回正
            "CLAW_U(1);",                    // y: U 夹紧
            "ROTATE_U(+90);",                // 单层 U
        ]);
    }

    #[test]
    fn whole_y2_no_reset() {
        // (z2s0) y2 → 整体 y 旋转 180°，不需要回正
        let moves = Moves::from_solution_string("(z2s0) y2");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert_eq!(steps.commands, vec![
            "CLAW_U(1);", "CLAW_R(1);",    // init
            "CLAW_R(0);",                    // y2: R 松开
            "ROTATE_U(+180);",               // y2: U 带魔方转 180°
            "CLAW_R(1);",                    // y2: R 夹紧（不回正）
        ]);
    }

    #[test]
    fn regrab_then_x() {
        // (z0s1) → R regrab
        // (s0z1) x → 整体 x
        // (z1z0) U → 单层 U
        let moves = Moves::from_solution_string("(z0s1)    (s0z1) x  (z1z0) U");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert!(steps.commands.contains(&"CLAW_R(0);".to_string()));
        assert!(steps.commands.contains(&"ROTATE_U(+90);".to_string()));
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
