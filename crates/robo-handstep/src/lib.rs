//! search → 机械步骤翻译，移植自 RobotApp/RobotStep.{h,cpp}。
//!
//! 输入：人手记号字符串（每 3 字符一段，face 字母 + 距离 1/2/3 + 空格），
//!     例如 `"F1 R2 U3 "`。
//! 输出：10-mnemonic 数字串（与 C 端 moveStr 一致）。
//!
//! 本 crate 与 `robo-translator` 是**两条独立的技术路线**（search vs search2l），
//! 互不替代。这里完全按 C 端语义复刻：操作库 + DFS + book 剪枝表 + 时间最优。
//!
//! ## 与 C 端对应关系
//! - `Engine` 持有 C 端所有全局变量（避免 Rust 全局可变状态）
//! - `Engine::all_init()` ≡ `robotstep::allInit()`
//! - `Engine::search()`   ≡ `robotstep::search()`
//! - `Engine::get_steps()`≡ `robotstep::getSteps()`
//!
//! ## C 端 bug 修复（§5.1）
//! `RobotStep.cpp:493-520` `TimeLibInit` 一段 `||`/`&&` 没有加括号，按 C 优先级
//! `&&` 高于 `||`，导致 DD90/DD180 分支永不进入；DFS 以 group.time 为目标
//! 函数选错变体。本移植按物理含义重写（L1/L3 分组 → 看爪状态分流到 KZ/DD）。
//!
//! 修复后真实 Kociemba 解（18-21 face）的机械步数普遍下降 1-4 步。
//! 详见 docs/robo-handstep-port-notes.md §5.1。

pub mod types;
mod op_table;

use std::sync::OnceLock;
use types::*;

// ============================================================================
//  Engine
// ============================================================================

const FACES: usize = 6;
const DISTS: usize = 3;
const STATES: usize = 3;
const VARIANTS: usize = 16;
const MECH_LIB_SIZE: usize = FACES * DISTS * STATES * VARIANTS; // = 864

/// 单次 search 输出机械步骤的最大数量。
///
/// C 端原值是 120（`MotorControl/main.c` `States[150]` 也只 ≤148），
/// 但 Kociemba 求解器 ~20 face 输入展开后 mech 步数可能超 200。
/// 256 在常见场景下足够；超过会 panic（仍受 g_theory_steps[25] 上界约束）。
const MOV_BUF_SIZE: usize = 256;

// ============================================================================
//  全局操作库（5.3：静态化）
// ============================================================================
//
// 操作库由 OP_TABLE 一次性派生而来，与 Engine 实例无关；多实例共享同一个
// 静态副本，避免每次 `Engine::new()` 都重做 864 条数据的初始化。

static MECH_LIB: OnceLock<Box<[MechanicalGroup; MECH_LIB_SIZE]>> = OnceLock::new();

#[inline]
fn lib_index(face: usize, dist: usize, state: usize, variant: usize) -> usize {
    ((face * DISTS + dist) * STATES + state) * VARIANTS + variant
}

fn build_mech_lib() -> Box<[MechanicalGroup; MECH_LIB_SIZE]> {
    build_mech_lib_with(CostModel::DEFAULT)
}

/// `build_mech_lib` 带可注入 cost 的版本，用于"前端调参后重新生成操作库"。
///
/// 注：MECH_LIB 是静态 OnceLock，运行时切换 cost 需要绕过它（提供独立
/// `Engine::with_cost` 持有自己的 lib，不污染全局）。这里先把构造函数
/// 拆分出来，给上层选择灵活性。
pub fn build_mech_lib_with(cost: CostModel) -> Box<[MechanicalGroup; MECH_LIB_SIZE]> {
    // 1. 先按 OP_TABLE 填充
    let mut lib_vec: Vec<MechanicalGroup> = vec![MechanicalGroup::default(); MECH_LIB_SIZE];

    for idx in 0..op_table::OP_TABLE_SIZE {
        let e = op_table::OP_TABLE[idx];
        // OP_TABLE.steps 已经是 num 值（i32），直接转 i8 拷贝；
        // -1 sentinel 自然带过去（i32::-1 == i8::-1）。
        let mut tmp_steps: [StepNum; 20] = [-1; 20];
        for i in 0..20 {
            tmp_steps[i] = e.steps[i] as StepNum;
            if e.steps[i] == -1 {
                break;
            }
        }
        let mut rot = Rot::default();
        rot.set(
            e.rot[0] as usize, e.rot[1],
            e.rot[2] as usize, e.rot[3],
            e.rot[4] as usize, e.rot[5],
        );
        let mut hs = HandState::default();
        hs.set(e.hs[0], e.hs[1], e.hs[2], e.hs[3]);
        lib_vec[lib_index(e.face, e.dist, e.state, e.variant)]
            .set(e.step_count, &tmp_steps, rot, hs);
    }

    // 2. TimeLibInit：根据 steps 算每个 group.time
    //
    // 修复 §5.1 bug：C 端原写法 `(num==L1) || (num==L3) && (...)` 因 C 优先级
    // `&&` 高于 `||`，导致只要 `num==L1` 就总命中第一个 KZ 分支，DD90/DD180
    // 几乎永不进入。这里按物理含义重写：
    //   - ND（拧动）：双爪都 CLOSE，转一爪带魔方一层
    //   - KZ（空转）：转动手 OPEN，对侧 CLOSE 维持魔方位置
    //   - DD（带动）：转动手 CLOSE 夹魔方，对侧 OPEN 被带着转
    //
    // 修复后操作库 group.time 字段值变化 → DFS 选出的"最优"变体可能与
    // C 端不同（C 端本就是错的）。baseline_* 测试需要重新校准。
    for i in F..=D {
        for j in _1..=_3 {
            for k in L_0_R_0..=L_1_R_0 {
                for l in 0..VARIANTS {
                    let group = &mut lib_vec[lib_index(i, j, k, l)];
                    group.time = 0;
                    let mut left_hand = CLOSE;
                    let mut right_hand = CLOSE;
                    let mut m = 0usize;
                    while group.steps[m] != -1 {
                        let num = group.steps[m] as i32;
                        // 气缸开合
                        if num == LO {
                            left_hand = OPEN;
                            group.time += cost.air_open;
                        } else if num == LC {
                            left_hand = CLOSE;
                        } else if num == RO {
                            right_hand = OPEN;
                            group.time += cost.air_open;
                        } else if num == RC {
                            right_hand = CLOSE;
                        }
                        // 拧动 ND（双爪都 CLOSE）
                        else if right_hand == CLOSE && left_hand == CLOSE {
                            if num == L2 || num == R2 {
                                group.time += cost.nd180;
                            } else {
                                group.time += cost.nd90;
                            }
                        }
                        // 左手转动：根据自身爪状态分流
                        else if num == L1 || num == L3 {
                            if left_hand == OPEN && right_hand == CLOSE {
                                // 左爪开 = 空转
                                group.time += cost.kz90;
                            } else if left_hand == CLOSE && right_hand == OPEN {
                                // 左爪闭夹魔方 + 右爪开被带 = 带动 90
                                group.time += cost.dd90;
                            }
                            // 其它组合（双开）物理上不会出现
                        } else if num == L2 {
                            // L2 在双闭已被上面分支吃掉；此处只可能是 CLOSE/OPEN
                            if left_hand == CLOSE && right_hand == OPEN {
                                group.time += cost.dd180;
                            }
                        }
                        // 右手转动：对称
                        else if num == R1 || num == R3 {
                            if right_hand == OPEN && left_hand == CLOSE {
                                group.time += cost.kz90;
                            } else if right_hand == CLOSE && left_hand == OPEN {
                                group.time += cost.dd90;
                            }
                        } else if num == R2 {
                            if right_hand == CLOSE && left_hand == OPEN {
                                group.time += cost.dd180;
                            }
                        }
                        m += 1;
                    }
                }
            }
        }
    }

    lib_vec
        .into_boxed_slice()
        .try_into()
        .expect("MECH_LIB_SIZE mismatch")
}

#[inline]
fn mech_lib() -> &'static [MechanicalGroup; MECH_LIB_SIZE] {
    MECH_LIB.get_or_init(build_mech_lib)
}

/// 移植自 RobotStep.cpp 的全部全局状态（去掉与 Engine 实例无关的部分）。
///
/// **5.3 优化**：操作库 `mech_lib` 不再持有，改走全局 `MECH_LIB`（static OnceLock）。  
/// **5.5 优化**：C 端 R_x1..R_z3 在 DFS 中未被读取，已删除。
pub struct Engine {
    // 六个面中心点（PointInit）—— 给 search() 解析人手记号时用
    p_frubld: [Point3; 6],

    // ===== Dfs 全局变量 =====
    g_time: [i32; 2],
    g_step_num: [i32; 2],
    g_theory_str_step: [i32; 2],
    g_theory_steps: [TheoryStep; 25],
    g_theory_steps2: [[TheoryStep; 25]; 2],
    g_cube_rot: Rot,
    g_hand_state: HandState,
    g_mov_buff: [[StepNum; MOV_BUF_SIZE]; 2],

    // ===== Dfs 存储变量 =====
    s_time: [i32; 2],
    s_step_num: [i32; 2],
    s_mov_buff: [[StepNum; MOV_BUF_SIZE]; 2],
    s_hand_state: [HandState; 2],
    s_rot: [Rot; 2],

    // ===== 剪枝表 book[25][2][3][3][2][3][2][3][2] =====
    //
    // 优化：lazy reset。
    //   每个槽位带一个 u32 `epoch` 标识"上次写入属于哪一轮 search"。
    //   `book_init` 不再 fill 64.8KB，只把 `book_epoch += 1`，
    //   读 book 时若 `book[i].epoch != book_epoch` 视为未访问（等价于 +∞）。
    //   `u32` 在 `book_epoch` 溢出（每秒上百万次 search 也要数十年才跑完）
    //   前不会出问题；保险起见 wrap 时做一次真正的 fill 复位。
    book: Box<[BookCell; BOOK_SIZE]>,
    book_epoch: u32,
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
struct BookCell {
    epoch: u32,
    time: i32,
}

// book 维度：[step=25][state=2][hand=3][row0=3][num0=2][row1=3][num1=2][row2=3][num2=2]
const BOOK_SIZE: usize = 25 * 2 * 3 * 3 * 2 * 3 * 2 * 3 * 2;

/// 手动展开的 book 索引计算（去循环 + 去 debug_assert，编译器可一次算清）。
/// 维度乘积常量，等价于嵌套数组 `book[step][state][hand][row0][num0][row1][num1][row2][num2]`。
#[inline(always)]
fn book_index(
    step: usize, state: usize, hand: usize,
    row0: usize, num0: usize,
    row1: usize, num1: usize,
    row2: usize, num2: usize,
) -> usize {
    // 维度按 C 端嵌套顺序 [25,2,3,3,2,3,2,3,2]：
    // off = ((((((((step*2 + state)*3 + hand)*3 + row0)*2 + num0)*3 + row1)*2 + num1)*3 + row2)*2 + num2
    let mut off = step * 2 + state;
    off = off * 3 + hand;
    off = off * 3 + row0;
    off = off * 2 + num0;
    off = off * 3 + row1;
    off = off * 2 + num1;
    off = off * 3 + row2;
    off = off * 2 + num2;
    off
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

impl Engine {
    pub fn new() -> Self {
        let mut e = Self {
            p_frubld: [Point3::default(); 6],
            g_time: [0; 2],
            g_step_num: [0; 2],
            g_theory_str_step: [0; 2],
            g_theory_steps: [TheoryStep::default(); 25],
            g_theory_steps2: [[TheoryStep::default(); 25]; 2],
            g_cube_rot: Rot::default(),
            g_hand_state: HandState::default(),
            g_mov_buff: [[-1; MOV_BUF_SIZE]; 2],
            s_time: [0; 2],
            s_step_num: [0; 2],
            s_mov_buff: [[-1; MOV_BUF_SIZE]; 2],
            s_hand_state: [HandState::default(); 2],
            s_rot: [Rot::default(); 2],
            // `Box::new([cell; N])` 会在栈上先构造再拷到堆，N 太大有
            // overflow 风险；用 vec![] + try_into 生成 Box<[T; N]>。
            book: vec![BookCell { epoch: 0, time: 0 }; BOOK_SIZE]
                .into_boxed_slice()
                .try_into()
                .expect("BOOK_SIZE mismatch"),
            book_epoch: 0,
        };
        e.all_init();
        e
    }

    /// 取静态操作库的某个 group（5.3）
    #[inline]
    fn lib(&self, face: usize, dist: usize, state: usize, variant: usize) -> &'static MechanicalGroup {
        &mech_lib()[lib_index(face, dist, state, variant)]
    }

    // ========================================================================
    //  对应 C 端 allInit / 各 *Init
    // ========================================================================

    pub fn all_init(&mut self) {
        // RobotStepsInit / OperateLibInit / TimeLibInit 已合并到全局
        // build_mech_lib（5.3）；RotInit 已删（5.5）。
        self.point_init();
        // 触发一次操作库初始化（首次调用时执行 build_mech_lib）。
        let _ = mech_lib();
    }

    /// RobotStep.cpp:373 PointInit
    fn point_init(&mut self) {
        // F = [1,0,0]^T
        self.p_frubld[F].a = [[1], [0], [0]];
        // R = [0,1,0]^T
        self.p_frubld[R].a = [[0], [1], [0]];
        // U = [0,0,1]^T
        self.p_frubld[U].a = [[0], [0], [1]];
        // B = [-1,0,0]^T
        self.p_frubld[B].a = [[-1], [0], [0]];
        // L = [0,-1,0]^T
        self.p_frubld[L_FACE].a = [[0], [-1], [0]];
        // D = [0,0,-1]^T
        self.p_frubld[D].a = [[0], [0], [-1]];
    }

    // ========================================================================
    //  对应 C 端 search / dfs / getSteps
    // ========================================================================

    /// RobotStep.cpp:83 search
    ///
    /// 输入约定（与 C 端一致）：每 3 字符一段，face 字母 + 距离 1/2/3 + 空格。
    /// 不满足该格式（长度非 3 倍数 / 非法字符）时返回 0，不再 panic。
    pub fn search(&mut self, theory_str: &str) -> i32 {
        let bytes = theory_str.as_bytes();
        // 防御：必须 3 字节对齐（5.6 防 panic）
        if bytes.len() % 3 != 0 {
            return 0;
        }
        let theory_str_length = (bytes.len() / 3) as i32;
        // 防御：theory 不能超过 g_theory_steps 容量（25）
        if theory_str_length as usize > self.g_theory_steps.len() {
            return 0;
        }
        // 分段搜索（与 C 端一致：第二段长度 = 0，即只跑第一段）
        self.g_theory_str_step[0] = theory_str_length;
        self.g_theory_str_step[1] = 0;

        // 初始化 g_theory_steps / g_theory_steps2
        for i in 0..theory_str_length as usize {
            // face 字符 → P_FRUBLD 索引 → face.a[j][0]
            let face_char = bytes[i * 3] as char;
            let face_idx = char2_int(face_char);
            // 防御：非 FRUBLD 字符直接放弃整次解析（5.6）
            if face_idx < 0 {
                return 0;
            }
            // C 端只把 face.a[j][0] 拷过去；这里同样三行赋值
            for j in 0..3 {
                self.g_theory_steps[i].face.a[j][0] = self.p_frubld[face_idx as usize].a[j][0];
            }
            // distance = (digit - '0') - 1   →   '1'→0='_1', '2'→1='_2', '3'→2='_3'
            let dist = (bytes[i * 3 + 1] as i32) - 0x30 - 1;
            // 防御：距离必须落在 [_1, _3] = [0, 2]
            if !(0..=2).contains(&dist) {
                return 0;
            }
            self.g_theory_steps[i].distance = dist;
        }

        for i in 0..self.g_theory_str_step[0] as usize {
            self.g_theory_steps2[0][i] = self.g_theory_steps[i];
        }
        for i in 0..self.g_theory_str_step[1] as usize {
            self.g_theory_steps2[1][i] =
                self.g_theory_steps[i + self.g_theory_str_step[0] as usize];
        }

        // 初始化搜索用的变量
        self.book_init();
        self.g_hand_state.set(0, 0, 0, 0);
        self.g_cube_rot.set(0, 1, 1, 1, 2, 1);
        self.g_time = [0, 0];
        self.s_time = [1_000_000, 1_000_000];
        self.g_step_num = [0, 0];
        self.s_step_num = [1000, 1000];
        // 优化：g_mov_buff/s_mov_buff 不再 reset。
        //   - g_mov_buff：DFS 写入只在 [0, g_step_num) 区间，读取也按
        //     g_step_num 截断（5.2），陈旧数据不会被观察到。
        //   - s_mov_buff：终点拷贝 copy_from_slice + getSteps 按 s_step_num
        //     截断，同理。

        // 深度搜索
        self.dfs(0, 0); // 第一阶段
        self.g_cube_rot = self.s_rot[0];
        self.g_hand_state = self.s_hand_state[0];
        self.dfs(0, 1); // 第二阶段（C 端默认空）

        self.s_step_num[0] + self.s_step_num[1]
    }

    /// RobotStep.cpp:130 getSteps
    pub fn get_steps(&self) -> String {
        let mut robot_steps = String::new();
        for i in 0..self.s_step_num[0] as usize {
            let num = self.s_mov_buff[0][i] as i32;
            if num < 0 || num >= 10 {
                break;
            }
            robot_steps.push_str(MOVE_STR[num as usize]);
        }
        robot_steps
    }

    /// RobotStep.cpp:265 bookInit（lazy reset 版）
    fn book_init(&mut self) {
        // 优化：通过递增 epoch 让所有旧槽位"自动失效"，O(1) 完成 reset。
        // 仅当 epoch 极少数情况下绕回 0 时才做一次真正的全量 fill。
        let (next, overflowed) = self.book_epoch.overflowing_add(1);
        if overflowed {
            // 极罕见：u32 epoch 用尽。把所有 cell 重置到 epoch=0 重新开始。
            self.book.fill(BookCell { epoch: 0, time: 0 });
            self.book_epoch = 1;
        } else {
            self.book_epoch = next;
        }
    }

    /// RobotStep.cpp:141 dfs
    fn dfs(&mut self, step: i32, state: usize) {
        #[cfg(test)]
        DFS_NODE_COUNTER.with(|c| c.set(c.get() + 1));
        // 到达最深处
        if step == self.g_theory_str_step[state] {
            if self.g_time[state] < self.s_time[state] {
                self.s_time[state] = self.g_time[state];
                self.s_step_num[state] = self.g_step_num[state];
                // 5.2：g_mov_buff 不再带 -1 sentinel（回溯时不复原），按
                // step_num 截断拷贝；getSteps() 也已按 s_step_num 读，等价。
                let n = self.g_step_num[state] as usize;
                self.s_mov_buff[state][..n].copy_from_slice(&self.g_mov_buff[state][..n]);
                self.s_rot[state] = self.g_cube_rot;
                self.s_hand_state[state] = self.g_hand_state;
            }
            return;
        }

        // 获取 face（face 号在 0..6）
        // __i: theorySteps2[state][step].face.a[__i][0] != 0 的第一行
        let theory_face = self.g_theory_steps2[state][step as usize].face;
        let mut __i = 0usize;
        while __i < 3 {
            if theory_face.a[__i][0] != 0 {
                break;
            }
            __i += 1;
        }
        // __j: g_cube_rot.a[__j][__i] != 0 的第一行
        let mut __j = 0usize;
        while __j < 3 {
            if self.g_cube_rot.a[__j][__i] != 0 {
                break;
            }
            __j += 1;
        }
        let face: usize;
        if __j == 0 {
            face = if self.g_cube_rot.a[__j][__i] == theory_face.a[__i][0] { F } else { B };
        } else if __j == 1 {
            face = if self.g_cube_rot.a[__j][__i] == theory_face.a[__i][0] { R } else { L_FACE };
        } else {
            // __j == 2
            face = if self.g_cube_rot.a[__j][__i] == theory_face.a[__i][0] { U } else { D };
        }

        let j = self.g_theory_steps2[state][step as usize].distance as usize;
        // hand = LeftNotNice * 2 + RightNotNice
        let k = (self.g_hand_state.left_not_nice * 2 + self.g_hand_state.right_not_nice) as usize;

        for l in 0..VARIANTS {
            // 5.2 优化：g_mov_buff 写入只发生在 [step_num, step_num+group.step_num)，
            // 回溯时只需恢复 g_step_num；下一次同位置写入会覆盖。
            // 因此删除 C 端的 tempMoveBuff[120] 全量拷贝——可观测输出完全等价
            // （s_mov_buff 终点截断按 s_step_num；g_mov_buff 内的"陈旧"数据
            // 永不会被读到 step_num 之外）。
            let _temp_rot = self.g_cube_rot;
            let _temp_hand_state = self.g_hand_state;
            let temp_step_num = self.g_step_num[state];
            let temp_time = self.g_time[state];

            // 加入本次节点
            let group = self.lib(face, j, k, l);
            self.g_time[state] += group.time;
            self.g_cube_rot = rot_mtpl_rot(group.rot, self.g_cube_rot);
            self.g_hand_state = group.end_hand_state;
            let step_num_to_copy = group.step_num as usize;
            let base = temp_step_num as usize;
            // 防御：缓冲区溢出时直接放弃此分支（与 book 剪枝一样视为不可达）
            if base + step_num_to_copy > MOV_BUF_SIZE {
                self.g_cube_rot = _temp_rot;
                self.g_hand_state = _temp_hand_state;
                self.g_time[state] = temp_time;
                self.g_step_num[state] = temp_step_num;
                continue;
            }
            // 整段 copy_from_slice，编译器可识别为 SIMD memcpy
            // （steps 已是 [i8; 20]，类型直接匹配 g_mov_buff 元素类型）。
            self.g_mov_buff[state][base..base + step_num_to_copy]
                .copy_from_slice(&group.steps[..step_num_to_copy]);
            self.g_step_num[state] += group.step_num;

            // 查看此结果在此深度下有没有发生过
            let mut row = [0usize; 3];
            let mut num = [0i32; 3];
            for _i in 0..3usize {
                let mut r = 0usize;
                while r < 3 {
                    if self.g_cube_rot.a[r][_i] != 0 {
                        if self.g_cube_rot.a[r][_i] == -1 {
                            num[_i] = 0;
                        } else if self.g_cube_rot.a[r][_i] == 1 {
                            num[_i] = 1;
                        } else {
                            // C 端 else（理论上 -1/1 之外不应出现）
                            num[_i] = -1;
                        }
                        break;
                    }
                    r += 1;
                }
                row[_i] = r;
            }
            // 把 i32 num（0/1）映射为下标 (0/1)，C 端直接当索引用了，因此一定要在 [0,1]
            // 内；否则 book 越界。这里加一个保护性 cast。
            let n0 = num[0].max(0).min(1) as usize;
            let n1 = num[1].max(0).min(1) as usize;
            let n2 = num[2].max(0).min(1) as usize;

            let hand =
                (self.g_hand_state.left_not_nice * 2 + self.g_hand_state.right_not_nice) as usize;
            let book_idx = book_index(
                step as usize, state, hand,
                row[0], n0, row[1], n1, row[2], n2,
            );

            // lazy book：cell.epoch != book_epoch 视为未访问（即 +∞）
            let cell = self.book[book_idx];
            let prev_time = if cell.epoch == self.book_epoch { cell.time } else { i32::MAX };
            if self.g_time[state] < prev_time {
                self.book[book_idx] = BookCell {
                    epoch: self.book_epoch,
                    time: self.g_time[state],
                };
                // 深搜
                self.dfs(step + 1, state);
            }
            // 复原（5.2：不再恢复 g_mov_buff 整个 buffer，只恢复水位线）
            self.g_cube_rot = _temp_rot;
            self.g_hand_state = _temp_hand_state;
            self.g_time[state] = temp_time;
            self.g_step_num[state] = temp_step_num;
        }
    }
}

// ============================================================================
//  辅助函数
// ============================================================================

/// RobotStep.cpp:284 char2Int
fn char2_int(c: char) -> i32 {
    match c {
        'F' => F as i32,
        'R' => R as i32,
        'U' => U as i32,
        'B' => B as i32,
        'L' => L_FACE as i32,
        'D' => D as i32,
        _ => -1,
    }
}

/// RobotStep.cpp:415 RotMtplRot
pub fn rot_mtpl_rot(l: Rot, r: Rot) -> Rot {
    let mut temp = Rot::default();
    for k in 0..3 {
        // l.a[k][j] != 0 的第一个 j
        let mut j = 0usize;
        while j < 3 {
            if l.a[k][j] != 0 {
                break;
            }
            j += 1;
        }
        // r.a[j][i] != 0 的第一个 i（注意 C 端用 j ∈ [0,3]，
        // 当 j==3（整行为 0）时下面的 r.a[j][i] 会越界——保留语义意味着也保持
        // 同样的越界行为是不安全的。这里加 bound 检查后让其默认 0，等价于
        // 不写入 temp.a[k][i]，比"未定义读"语义更稳；同时避免 panic。
        if j >= 3 {
            continue;
        }
        let mut i = 0usize;
        while i < 3 {
            if r.a[j][i] != 0 {
                break;
            }
            i += 1;
        }
        if i >= 3 {
            continue;
        }
        if l.a[k][j] == r.a[j][i] {
            temp.a[k][i] = 1;
        } else {
            temp.a[k][i] = -1;
        }
    }
    temp
}

/// RobotStep.cpp:439 RotMtplPoint3
pub fn rot_mtpl_point3(l: Rot, r: Point3) -> Point3 {
    let mut temp = Point3::default();
    for i in 0..3 {
        for j in 0..1 {
            temp.a[i][j] = l.a[i][0] * r.a[0][j] + l.a[i][1] * r.a[1][j] + l.a[i][2] * r.a[2][j];
        }
    }
    temp
}

// ============================================================================
//  Tests
// ============================================================================

#[cfg(test)]
thread_local! {
    static DFS_NODE_COUNTER: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn engine_constructs() {
        let _e = Engine::new();
    }

    #[test]
    fn op_table_loaded() {
        let e = Engine::new();
        // 检查至少有一个非默认值的 group
        let g = e.lib(F, _1, L_0_R_0, 0);
        assert_eq!(g.step_num, 1);
        assert_eq!(g.steps[0] as i32, L1);
    }

    #[test]
    fn empty_input_returns_zero() {
        let mut e = Engine::new();
        let n = e.search("");
        assert_eq!(n, 0);
        assert_eq!(e.get_steps(), "");
    }

    #[test]
    fn malformed_input_does_not_panic() {
        let mut e = Engine::new();
        // 长度非 3 倍数
        assert_eq!(e.search("F"), 0);
        assert_eq!(e.search("F1"), 0);
        // 非法 face
        assert_eq!(e.search("X1 "), 0);
        // 非法距离
        assert_eq!(e.search("F4 "), 0);
        assert_eq!(e.search("F0 "), 0);
        // 超长输入不能 panic（>25 face）
        let mut long = String::new();
        for _ in 0..30 { long.push_str("F1 "); }
        assert_eq!(e.search(&long), 0);
    }

    #[test]
    fn single_face_search_emits_some_steps() {
        let mut e = Engine::new();
        // F1 + 末尾必须有空格（C 端按每 3 字符一段切分）
        let n = e.search("F1 ");
        assert!(n > 0);
        let s = e.get_steps();
        // 输出全部由 MOVE_STR 字符组成
        assert!(s.chars().all(|c| MOVE_STR.iter().any(|m| m.starts_with(c))));
    }

    #[test]
    fn multi_face_runs_without_panic() {
        let mut e = Engine::new();
        let n = e.search("F1 R2 U3 ");
        assert!(n > 0);
        let s = e.get_steps();
        assert!(!s.is_empty());
    }

    /// D1 与 C 端的对齐曾经是 `1906954`（7 步：LO RO LC R3 RO R1 L1）。
    /// §5.1 修复 TimeLibInit 优先级 bug 后 DFS 选了不同变体 —— 同样 7 步，
    /// 但走 `1406259`（LO L1 LC R2 RC L1 RO，使用 R2 双闭 ND180 而不是
    /// 两次 R3+R1 的 KZ）。bug 修复让"真正最短时间"变体被发现。
    #[test]
    fn d1_after_time_bug_fix() {
        let mut e = Engine::new();
        let n = e.search("D1 ");
        let s = e.get_steps();
        eprintln!("D1: steps={}, output={:?}", n, s);
        assert_eq!(s, "1406259", "D1（§5.1 修复后的真·时间最短）");
    }

    #[test]
    fn l1_matches_reference() {
        let mut e = Engine::new();
        let n = e.search("L1 ");
        let s = e.get_steps();
        eprintln!("L1: steps={}, output={:?}", n, s);
        assert_eq!(s, "6359", "L1 应当输出 6359（C 端参考）");
    }

    /// Baseline：锁定六个面 / 三种距离 共 18 个单步输入的输出。
    ///
    /// 数值是从**首次成功移植**的版本捕获的；后续任何重构（5.2/5.3/5.4 等
    /// 性能优化）都必须保持这些输出**逐字节一致**，否则就是引入了语义偏移。
    /// 其中 D1、L1 已与 C 端对齐，其余条目是 Rust 移植版的自洽 baseline，
    /// 等 C 端 bench_compare 跑通后可换成 C 端真值。
    fn collect_baseline() -> Vec<(&'static str, String)> {
        let cases = [
            "F1 ", "F2 ", "F3 ",
            "R1 ", "R2 ", "R3 ",
            "U1 ", "U2 ", "U3 ",
            "B1 ", "B2 ", "B3 ",
            "L1 ", "L2 ", "L3 ",
            "D1 ", "D2 ", "D3 ",
        ];
        let mut out = Vec::new();
        for c in cases {
            let mut e = Engine::new();
            e.search(c);
            out.push((c, e.get_steps()));
        }
        out
    }

    #[test]
    fn baseline_single_face() {
        // 18 个单 face 输入的"快照"：每次重构后都必须等于这张表。
        // 失败时打印实际输出，便于人工审查是否符合预期。
        let actual = collect_baseline();
        let expected: &[(&str, &str)] = &[
            ("F1 ", "4"),
            ("F2 ", "3"),
            ("F3 ", "2"),
            ("R1 ", "9"),
            ("R2 ", "8"),
            ("R3 ", "7"),
            ("U1 ", "1406459"),
            ("U2 ", "1406458"),
            ("U3 ", "1406457"),
            ("B1 ", "1804"),
            ("B2 ", "1803"),
            ("B3 ", "1802"),
            ("L1 ", "6359"),
            ("L2 ", "6358"),
            ("L3 ", "6357"),
            ("D1 ", "1406259"),
            ("D2 ", "1406258"),
            ("D3 ", "1406257"),
        ];
        // 先打印一份实际输出，方便首次校准
        for (inp, out) in &actual {
            eprintln!("{} => {}", inp, out);
        }
        for ((inp, out), (e_inp, e_out)) in actual.iter().zip(expected.iter()) {
            assert_eq!(inp, e_inp);
            assert_eq!(out, e_out, "baseline 偏移：{} 实际 {:?} 期望 {:?}", inp, out, e_out);
        }
    }

    /// 性能 stress：用 `--nocapture` 看典型场景耗时及 DFS 搜索树规模。
    /// 不做正确性断言；正确性靠 `baseline_*` 测试。
    ///
    /// 当前典型数据（release，M-series Mac）：
    /// ```text
    /// perf_engine_new : 50 ctors in 25 µs   (avg 500 ns/ctor)
    /// perf_len1       : 200 runs (~18  nodes) avg 240 ns/run
    /// perf_len2       : 200 runs (~41  nodes) avg 3.8 µs/run
    /// perf_len3       : 200 runs (~71  nodes) avg 8.9 µs/run
    /// perf_len5       : 200 runs (~185 nodes) avg 27 µs/run
    /// perf_len7       : 200 runs (~310 nodes) avg 53 µs/run
    /// perf_len10      : 200 runs (~1100 nodes) avg 175 µs/run
    /// ```
    ///
    /// DFS 单节点 ~150 ns，已接近 cache 友好小函数极限。
    #[test]
    fn perf_long_input() {
        let input = "F1 R2 U3 B1 L2 D3 F2 R1 U2 B3 ";  // 10 face

        let start2 = std::time::Instant::now();
        let n2 = 50;
        for _ in 0..n2 {
            let _e = Engine::new();
        }
        let elapsed2 = start2.elapsed();
        eprintln!(
            "perf_engine_new: {} ctors in {:?} (avg {:?}/ctor)",
            n2, elapsed2, elapsed2 / n2 as u32
        );

        for inp in ["F1 ", "F1 R1 ", "F1 R1 U1 ", "F1 R1 U1 B1 L1 ",
                    "F1 R1 U1 B1 L1 D1 F1 ", input] {
            let mut e3 = Engine::new();
            DFS_NODE_COUNTER.with(|c| c.set(0));
            e3.search(inp);
            let nodes = DFS_NODE_COUNTER.with(|c| c.get());
            let n4 = 200;
            let s = std::time::Instant::now();
            for _ in 0..n4 {
                e3.search(inp);
            }
            let el = s.elapsed();
            eprintln!(
                "perf_len{}: {} runs in {:?} (avg {:?}/run, ~{} dfs nodes/run)",
                inp.len() / 3, n4, el, el / n4 as u32, nodes
            );
        }
    }



    /// 把标准 Kociemba 记号（`F / F2 / F'`）转换成 RobotStep 内部格式（`F1 / F2 / F3 `）。
    /// 用于喂给 search()。返回 None 表示无法解析。
    fn kociemba_to_robotstep(kociemba: &str) -> Option<String> {
        let mut out = String::new();
        for token in kociemba.split_whitespace() {
            let mut chars = token.chars();
            let face = chars.next()?;
            if !"FRUBLD".contains(face) {
                return None;
            }
            let suffix: String = chars.collect();
            let dist_char = match suffix.as_str() {
                "" => '1',  // F   → +90 → _1
                "2" => '2', // F2  → 180 → _2
                "'" => '3', // F'  → -90 → _3
                _ => return None,
            };
            out.push(face);
            out.push(dist_char);
            out.push(' ');
        }
        Some(out)
    }

    /// 用户给的真实 Kociemba 输出序列：跑一遍看耗时 + 输出长度。
    /// 不做正确性断言（没有 reference 真值），只验证不 panic 且输出非空。
    #[test]
    fn user_provided_sequences() {
        // (Kociemba 串, 期望机械步数, 期望完整输出)
        // 期望值是 §5.1 修复后的版本（"真正时间最优"，与 §5.3 删
        // MechanicalStep::time 重构前后 byte-equivalent）。
        let cases: &[(&str, i32, &str)] = &[
            ("R2 F' D2 F' U2 R2 F  R2 U2 F' D  L  F2 D' B' R' B  R2 F  U'",
             72, "862528264951703869519404814064582649517041490931706945271740296935391902"),
            ("L2 U' B2 R2 D' B2 L2 D' B2 R2 D  R2 L' D2 U' B' F  R  D2 B' L'",
             87, "635819069252819069253147073170695371903190695396953184028635717026451840491903625149072"),
            ("U  B' D  L  U2 F' U  L2 U' L  B2 U2 B2 D2 R  F2 B2 D2 R2 D",
             75, "170694547174064959464951703769254821409692514083818036451470931803170695384"),
            ("F2 R  U  R  B  R  D2 F2 U' B  D  L2 F2 D  B2 U  R2 D  L2",
             74, "39170414709414709317069531706945291740649593170695391806935391703639591803"),
            ("U' L' U' L' B' R  U  D' L  B2 R2 U  L2 U  R2 D' L2 U2 L2 D' B'",
             85, "1406457174027694527194062959414802629519041480831706945484635148082635148083818026457"),
            ("L2 F  B2 U2 L2 D  R2 B2 F2 D' L2 B2 U' R' B2 R  D  L  B2 F'",
             80, "63584148031706953818404635814062586358264581706945314707625284149069254917031802"),
            ("F  D  R2 L2 B2 D' F  D2 F2 U2 D' F2 B2 R2 D2 R2 L2 F  R  D",
             71, "62549170318031906953769518404836358639573180319069538318031706945491904"),
            ("R  L  B2 R' D' R  D  B' U' R2 D2 L2 F' D2 F2 B' L2 R",
             67, "9693591806935371740296925471740645769353818036951740283180264586359"),
            ("L  D  B  L' F  R2 B' U  L2 F2 U' F2 R2 U' L2 U' R2 U2 R2 F' L2",
             84, "635917046459692514907463514808625291903190695376925314086451470737180695383170694528"),
            ("R2 D2 B2 F2 R2 B2 U' B2 F2 L2 U  B  L2 F  R  U' R2 D  B' U2",
             81, "819069531906953180381840362957318031706953170962548180462951940476935396945147073"),
            ("R  D2 R' D2 B2 U2 F2 R2 U2 L2 R  B2 U' R  B' L2 B' L' R2 U' B",
             75, "919037692531480836358140645836358635917036951940296945147073769252180371904"),
        ];
        let mut e = Engine::new();
        for (i, (kc, exp_n, exp_s)) in cases.iter().enumerate() {
            let face_count = kc.split_whitespace().count();
            let rs = kociemba_to_robotstep(kc).expect("Kociemba 解析失败");
            DFS_NODE_COUNTER.with(|c| c.set(0));
            let start = std::time::Instant::now();
            let n = e.search(&rs);
            let elapsed = start.elapsed();
            let nodes = DFS_NODE_COUNTER.with(|c| c.get());
            let s = e.get_steps();
            eprintln!(
                "[{:2}] {:>2}f mech={:>3} nodes={:>5} t={:>9?} | {}",
                i, face_count, n, nodes, elapsed, s
            );
            assert_eq!(n, *exp_n,
                "[{}] 机械步数偏移：实际 {} 期望 {}", i, n, exp_n);
            assert_eq!(&s, exp_s,
                "[{}] 输出偏移：\n  实际 {}\n  期望 {}", i, s, exp_s);
        }
    }

    /// 多 face 长串 baseline：覆盖 DFS 多深度 + book 剪枝 + cube_rot 累乘。
    /// 等同 `baseline_single_face` 的角色——锁住语义、抓性能优化引入的偏移。
    #[test]
    fn baseline_multi_face() {
        let cases: &[(&str, &str)] = &[
            ("F1 R1 U1 ",          "4147094"),
            ("F1 R2 U3 ",          "4147086952"),
            ("F1 R1 F1 R1 ",       "4140969541409"),
            ("U1 R1 F1 D1 ",       "17069541490962549"),
            ("F1 R1 U1 B1 L1 D1 ", "4147094645917046459"),
        ];
        // 第一次跑时打印实际输出便于校准
        for (inp, _e_out) in cases {
            let mut e = Engine::new();
            e.search(inp);
            eprintln!("{} => {}", inp, e.get_steps());
        }
        for (inp, e_out) in cases {
            let mut e = Engine::new();
            e.search(inp);
            let actual = e.get_steps();
            assert_eq!(&actual, e_out,
                "多 face baseline 偏移：{:?} 实际 {:?} 期望 {:?}", inp, actual, e_out);
        }
    }
}
