//! 直接对应 RobotApp/RobotStep.h / HandStep.h 中的结构体。
//! 本模块只做"数据搬运"，不引入任何新语义。

// ===== 常量：与 C 端等值（HandStep.h / RobotStep.h）=====

// 面
pub const F: usize = 0;
pub const R: usize = 1;
pub const U: usize = 2;
pub const B: usize = 3;
pub const L_FACE: usize = 4;
pub const D: usize = 5;

// 距离
pub const _1: usize = 0;
pub const _2: usize = 1;
pub const _3: usize = 2;

// 手臂状态
pub const L_0_R_0: usize = 0;
pub const L_0_R_1: usize = 1;
pub const L_1_R_0: usize = 2;

// 机械动作 num（与 moveStr[10] 索引一致）
pub const L1: i32 = 0;
pub const L2: i32 = 1;
pub const L3: i32 = 2;
pub const LC: i32 = 3;
pub const LO: i32 = 4;
pub const R1: i32 = 5;
pub const R2: i32 = 6;
pub const R3: i32 = 7;
pub const RC: i32 = 8;
pub const RO: i32 = 9;

// 夹爪开合
pub const CLOSE: i32 = 0;
pub const OPEN: i32 = 1;

// 运动时间常量（HandStep.h:14-30）
pub const HAND_CLOSE_TIME: i32 = 300;
pub const HAND_OPEN_TIME: i32 = 300;
pub const HAND_MOVE_90: i32 = 300;
pub const HAND_MOVE_180: i32 = 300;
#[allow(dead_code)]
pub const DELAY_BETWEEN_2_STEP: i32 = 200;

pub const TIME_AIR: i32 = 120;
pub const TIM_KZ90: i32 = 53;
pub const TIM_ND90: i32 = 54;
pub const TIM_ND180: i32 = 90;
pub const TIM_DD90: i32 = 122;
pub const TIM_DD180: i32 = 194;

/// 与 C 端 moveStr[10] 完全一致（HandStep.h:11）
pub const MOVE_STR: [&str; 10] = ["4", "3", "2", "0", "1", "9", "8", "7", "5", "6"];

// ===== 数据结构 =====

#[derive(Clone, Copy, Debug, Default)]
pub struct HandState {
    pub is_left_open: i32,
    pub is_right_open: i32,
    pub left_not_nice: i32,
    pub right_not_nice: i32,
}

impl HandState {
    pub fn set(
        &mut self,
        is_left_open: i32,
        is_right_open: i32,
        left_not_nice: i32,
        right_not_nice: i32,
    ) {
        self.is_left_open = is_left_open;
        self.is_right_open = is_right_open;
        self.left_not_nice = left_not_nice;
        self.right_not_nice = right_not_nice;
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Rot {
    pub a: [[i32; 3]; 3],
}

impl Rot {
    pub fn set(&mut self, row0: usize, num0: i32, row1: usize, num1: i32, row2: usize, num2: i32) {
        for i in 0..3 {
            for j in 0..3 {
                self.a[i][j] = 0;
            }
        }
        self.a[row0][0] = num0;
        self.a[row1][1] = num1;
        self.a[row2][2] = num2;
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Point3 {
    pub a: [[i32; 1]; 3],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TheoryStep {
    pub face: Point3,
    pub distance: i32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct MechanicalStep {
    pub time: i32,
    pub num: i32, // -1 表示 M_END
}

/// 与 RobotStep.h:103 的 MechanicalGroup 等价。
/// `Steps` 长度按 C 端的 20 位（含 -1 终止）。
#[derive(Clone, Copy, Debug)]
pub struct MechanicalGroup {
    pub time: i32,
    pub step_num: i32,
    pub rot: Rot,
    pub end_hand_state: HandState,
    pub steps: [MechanicalStep; 20],
}

impl Default for MechanicalGroup {
    fn default() -> Self {
        Self {
            time: 0,
            step_num: 0,
            rot: Rot::default(),
            end_hand_state: HandState::default(),
            steps: [MechanicalStep { time: 0, num: -1 }; 20],
        }
    }
}

impl MechanicalGroup {
    /// C 端 MechanicalGroup::Set，遇到 num == -1 时停止并写入 M_END。
    pub fn set(&mut self, step_num: i32, src_steps: &[MechanicalStep; 20], rot: Rot, hs: HandState) {
        self.step_num = step_num;
        self.rot = rot;
        self.end_hand_state = hs;
        let mut i = 0usize;
        while i < 20 {
            // C 端用 name == "M_END" 作为终止；这里用 num == -1（M_END.num = -1）。
            if src_steps[i].num == -1 {
                break;
            }
            self.steps[i] = src_steps[i];
            i += 1;
        }
        self.steps[i] = MechanicalStep { time: 0, num: -1 };
    }
}

/// 数据表条目（与 RobotStep.h:170 的 OpEntry 等价）。
#[derive(Clone, Copy, Debug)]
pub struct OpEntry {
    pub face: usize,
    pub dist: usize,
    pub state: usize,
    pub variant: usize,
    pub step_count: i32,
    pub steps: [i32; 20],
    pub rot: [i32; 6],
    pub hs: [i32; 4],
}
