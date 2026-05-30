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

/// 机械动作助记符（顺序与 num 常量 L1..RO 一一对应）。
///
/// 这是 handstep 输出层的"语义"表示——上层用 `M_LC` 等语义标识，
/// 真正发到下位机时再由 transport 层用 digit_map 编码成数字字符。
/// 与 `robo_translator::MNEMONICS` 字面值一致，但 handstep 不依赖
/// translator crate，自带一份独立常量。
pub const MNEMONIC_STR: [&str; 10] = [
    "M_L1", "M_L2", "M_L3", "M_LC", "M_LO",
    "M_R1", "M_R2", "M_R3", "M_RC", "M_RO",
];

// ===== 时间成本配置（DFS 目标函数）=====
//
// 这些值是 DFS 用来比较"哪个变体更优"的成本权重。原始数值来自 C 端
// `RobotApp/HandStep.h:25-30`（`Time_Air / Tim_KZ90 / Tim_ND90 / ...`），
// 单位约为毫秒（与 motorControl 的加减速曲线总时长有关，未严格校准）。
//
// 物理含义：
//   air_open ：气缸"开爪"动作耗时（LO/RO 各加一次）。
//              注意：C 端 LC/RC（关爪）不加 cost，沿用之；如需修正
//              另外加 `air_close` 字段。
//   nd90/180 ：双爪都闭合时拧动一爪（带魔方层）的纯机械时间。
//   kz90     ：本爪开（空转）+ 对侧闭（维持），转 90°。
//   dd90/180 ：本爪闭（夹魔方）+ 对侧开（被带），即一爪带动整方。
//
// 字段公开 + Engine::with_cost 可注入，便于在前端调参后比较 DFS 结果。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CostModel {
    pub air_open: i32,  // LO / RO
    pub kz90: i32,      // 空转 90°
    pub nd90: i32,      // 拧动 90°
    pub nd180: i32,     // 拧动 180°
    pub dd90: i32,      // 带动 90°
    pub dd180: i32,     // 带动 180°
}

impl CostModel {
    /// C 端 HandStep.h:25-30 的默认值。
    pub const DEFAULT: Self = Self {
        air_open: 120,
        kz90: 53,
        nd90: 54,
        nd180: 90,
        dd90: 122,
        dd180: 194,
    };
}

impl Default for CostModel {
    fn default() -> Self { Self::DEFAULT }
}

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

/// 单步动作的 num（0..=9，对应 L1/L2/L3/LC/LO/R1/R2/R3/RC/RO）；
/// `-1` 表示 sentinel（C 端 M_END）。
///
/// 原 C 端 `MechanicalStep` 还带一个 `time` 字段（`HandMove90` 等 300 常数），
/// 但在 RobotApp 全工程内只写不读、DFS 也不依赖（DFS 用 `MechanicalGroup::time`
/// 整组累加值）。Rust 端把它退化成 `i8` 直接存动作 num，节省内存 + 提升 cache
/// 友好度。详见 docs §5.x。
pub type StepNum = i8;

/// 与 RobotStep.h:103 的 MechanicalGroup 等价（剔除 `MechanicalStep::time`）。
/// `steps` 长度按 C 端的 20 位（含 `-1` 终止）。
#[derive(Clone, Copy, Debug)]
pub struct MechanicalGroup {
    pub time: i32,
    pub step_num: i32,
    pub rot: Rot,
    pub end_hand_state: HandState,
    pub steps: [StepNum; 20],
}

impl Default for MechanicalGroup {
    fn default() -> Self {
        Self {
            time: 0,
            step_num: 0,
            rot: Rot::default(),
            end_hand_state: HandState::default(),
            steps: [-1; 20],
        }
    }
}

impl MechanicalGroup {
    /// C 端 `MechanicalGroup::Set`，遇到 `num == -1` 时停止并写入 sentinel。
    pub fn set(&mut self, step_num: i32, src_steps: &[StepNum; 20], rot: Rot, hs: HandState) {
        self.step_num = step_num;
        self.rot = rot;
        self.end_hand_state = hs;
        let mut i = 0usize;
        while i < 20 {
            if src_steps[i] == -1 {
                break;
            }
            self.steps[i] = src_steps[i];
            i += 1;
        }
        if i < 20 {
            self.steps[i] = -1;
        }
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
