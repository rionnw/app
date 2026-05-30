# `robo-handstep` 移植说明与优化建议

本文档记录把 `RobotApp/RobotStep.{h,cpp}` + `RobotStepData.inc`（C++）
移植到 Rust crate `robo-handstep` 的过程要点、刻意保留的可疑语义，
以及后续可独立实施的优化建议。

> 移植目标：**保持语义等价**。  
> 优化建议在本文档中列出，但**未实施**。任何选择落地的建议都应单独评估
> 与 C 端的等价性影响（特别是 §1 那条 bug，修后 DFS 选出的"最优"变体
> 可能与 C 端不一致）。

---

## 1. 移植映射对照

| C 端 (`RobotApp/RobotStep.cpp`)                              | Rust (`crates/robo-handstep`)                                  |
| ------------------------------------------------------------ | -------------------------------------------------------------- |
| `g_*` / `s_*` / `MechanicalGroupLib` / `book` 等全局变量      | `Engine` 结构体字段                                             |
| `allInit()`                                                  | `Engine::all_init`                                             |
| `RobotStepsInit / RotInit / PointInit / OperateLibInit / TimeLibInit` | 同名 snake_case 方法                                            |
| `search(theoryStr)`                                          | `Engine::search(&mut self, &str) -> i32`                       |
| `getSteps()`                                                 | `Engine::get_steps(&self) -> String`                           |
| `dfs(step, state)`                                           | `Engine::dfs(&mut self, i32, usize)`                           |
| `bookInit()`                                                 | `Engine::book_init`                                            |
| `char2Int`                                                   | 模块级 `char2_int`                                              |
| `RotMtplRot` / `RotMtplPoint3`                               | 模块级同名 snake_case                                            |
| `OP_TABLE` / `OP_TABLE_SIZE`                                 | `op_table::OP_TABLE` / `OP_TABLE_SIZE`（自动生成）              |

---

## 2. 数据表生成

操作库 864 条 `OpEntry` 通过一次性 Python 脚本从 C 端 `.inc` 转出：

```bash
python3 crates/robo-handstep/scripts/convert_op_table.py \
  > crates/robo-handstep/src/op_table.rs
```

脚本逐条读取 `RobotApp/RobotStepData.inc`，把 `L1`/`F`/`_1`/`L_0_R_0`
等 C 端常量名映射为数值，**不做任何语义改动**。

---

## 3. 等价性验证

当前已通过的对齐用例：

| 输入  | 步数 | 输出      | 与 C 端 |
| ----- | ---- | --------- | ------- |
| `D1 ` | 7    | `1906954` | ✅ 一致 |

更全面的对比建议：编 `RobotApp/robostep/search_opt/bench_compare`
作为参考 binary，对一份固定 corpus 跑两侧，逐行 diff。

---

## 4. 刻意保留的可疑语义（未修）

### 4.1 `RotInit` 中疑似 typo
`RobotStep.cpp:367`：
```cpp
R_z2 = RotMtplRot(R_z1, R_z2);   // 看着像应该写成 R_z3 = ...
```
`R_z3` 因此**始终为零矩阵**。Rust 端 `rot_init` 严格照搬，并在注释里
标注。`R_z3` 在当前 DFS 流程里实际未被读（DFS 只读 `OP_TABLE` 里的
`rot`），所以可观测层面无差异，但字段值与 C 端一致。

### 4.2 `TimeLibInit` 的 `||`/`&&` 优先级问题（**严重，见 §5.1**）
`RobotStep.cpp:493-520` 大段表达式没有加括号：
```cpp
else if ((num == L1) || (num == L3) && ((RH == CLOSE) && (LH == OPEN)))
```
按 C 优先级 `&&` > `||`，这等价于：
```cpp
else if ((num == L1) || ((num == L3) && ((RH == CLOSE) && (LH == OPEN))))
```
也就是 **只要 `num == L1` 就总命中第一个 KZ 分支**，"带动 DD90/DD180"
分支几乎永远走不到。Rust 端 `time_lib_init` 完全保留该结合性。

### 4.3 `RotMtplRot` 当某行全 0 时的越界读
C 端在 `l.a[k][j]` 一行全 0 时让 `j == 3`，随后 `r.a[j][i]` 越界（UB）。
Rust 直接照搬会 panic。当前实现：
```rust
if j >= 3 { continue; }   // 不再写入 temp.a[k][i]
```
等价于让该项贡献为 0。在 DFS 实际使用的合法旋转矩阵下不会触发——
两端可观测行为一致。**如果坚持完全等价（含 UB）**，移除保护即可。

### 4.4 `book` 维度与 `num` 取值
`dfs` 计算 `num[_i]` 时 C 端有 `else { num[_i] = -1; }` 分支；
但 `book[…][num0=2]` 维度只到 2。理论上 -1 不会出现（旋转矩阵元素
∈ {-1, 0, +1}），但如果出现就是 UB。Rust 端额外 clamp 到 `[0, 1]`
防止 panic，可观测层面无差异。

---

## 5. 优化建议

按优先级从高到低。**5.2 / 5.3 / 5.4 / 5.5 / 5.6 已实施**，性能数据见
§5.10；其余仍是建议。

### 5.1 修 `TimeLibInit` 的优先级 bug（**Bug 级**）

**问题**：见 §4.2。当前 `MechanicalGroup::time` 大量低估了"空转/带动"
代价，`MechanicalGroupLib[*][*][*][*].time` 字段里的相对大小关系
被破坏。DFS 以这个 `time` 为唯一目标函数，因此**当前 DFS 选出的
"最优"变体严格意义上不是真正全局时间最优**。

**修法**：6 处 `else if` 加括号，例如
```rust
else if (num == L1 || num == L3) && right_hand == CLOSE && left_hand == OPEN { ... }
```

**风险**：修完所有 864 条 group 的 `time` 字段会变，DFS 输出可能与
C 端**不再一致**。这是预期的 —— C 端本就是错的；但如果生产链路
依赖了"与 C 端 byte-for-byte 一致"，需要先把 C 端一起修才能动。

### 5.2 DFS 回溯免拷贝（**已实施**）

C 端原版每个递归节点都开 `int tempMoveBuff[120]` 整块复制。

**修法（已落地）**：用 `g_step_num` 作为水位线，回溯只恢复 `g_step_num`，
不再快照/恢复 `g_mov_buff`。同时把 dfs 终点 `s_mov_buff` 拷贝改成
按 `step_num` 截断的 `copy_from_slice`，免掉 -1 sentinel 扫描。

实测收益：见 §5.10。Baseline 全部对齐，输出逐字节等价。

### 5.3 `mech_lib` 静态化（**已实施**）

操作库与 Engine 实例无关：原始 `Vec<MechanicalGroup>` 改为
`static MECH_LIB: OnceLock<Box<[MechanicalGroup; 864]>>`，
`build_mech_lib()` 把 `RobotStepsInit + OperateLibInit + TimeLibInit`
合并成一次构造，多个 `Engine::new()` 共享。

### 5.4 `book` 改 `Box<[i32; SIZE]>`（**已实施**）

`book` 总尺寸 16200 i32 ≈ 64.8 KB。改成 `Box<[i32; BOOK_SIZE]>`
（堆上 + 编译期定长），`book_init` 改用 `fill(1_000_000)`（编译器
易识别为 memset）。

### 5.5 删除 `RotInit` 未被使用的字段（**已实施**）

`R_x1..R_z3`（9 个 Rot 矩阵）在 DFS 中从未被读取——已连同
`rot_init` 函数和模块顶部那段对 `R_z2 = ... R_z2` typo 的注释一并
删除。如未来需要做轮换 / 显式 rotate 路径再补回。

### 5.6 输入解析容错（**已实施**）

`Engine::search` 现在防御以下情况，全部返回 0 而非 panic：

- 长度非 3 倍数（`"F1"` 没尾空格）
- 非法 face 字符（`"X1 "`）
- 非法距离（`"F0 " / "F4 "`）
- 长度 > 25 face（超过 `g_theory_steps` 容量）

测试：`tests::malformed_input_does_not_panic`。

### 5.10 实施后的性能对比

#### 第一轮（5.2/5.3/5.4/5.5/5.6 完成后）

10 face 输入 / release / 200 次：

| 指标            | 优化前  | 优化后 | 提升  |
| --------------- | ------- | ------ | ----- |
| `Engine::new()` | 22 µs   | 0.5 µs | ~45×  |
| `search()`      | 931 µs  | 284 µs | ~3.3× |

`Engine::new` 主要受益于 5.3；`search` 主要受益于 5.2。

#### 第二轮（lazy book + 入口写消除 + book_index 展开）

| 输入长度 | search 提升 | DFS 节点 | µs/node |
| -------- | ----------- | -------- | ------- |
| 1 face   | 1.5 → 0.24 µs（**6×**） | 18    | 13 ns   |
| 2 face   | 5.1 → 3.8 µs            | 41    | 93 ns   |
| 3 face   | 9.9 → 8.6 µs            | 71    | 121 ns  |
| 5 face   | 29 → 28 µs              | 185   | 145 ns  |
| 7 face   | 54 → 53 µs              | 310   | 170 ns  |
| 10 face  | 177 → 177 µs            | 1107  | 160 ns  |

**结论**：`book_init` 改 lazy（epoch 计数）只对短查询显著；
长查询 (≥5 face) DFS 主体已经主导，每节点 ~150 ns 已经接近
cache 友好小函数的极限。

baseline 23 个用例（18 单 face + 5 多 face）输出全部 byte-equivalent。

#### 不再继续的原因

- 1 个 Kociemba 解通常 15-25 face；按线性外推 ~300-500 µs/解
- firmware 串口 115200 bps × 一帧 ~10 字节 ≈ 1 ms，**翻译时间已
  远低于传输时间**
- 上位机调用频率每秒最多十次，不会成瓶颈

继续算法层（IDA*、variant 按 time 排序、去递归）的工作量大、
等价性风险高，性价比急剧下降——已记入 §5.11 待选项。

### 5.11 待选优化（暂不实施）

- **A. OP_TABLE 内 16 variant 按 group.time 排序**：让 DFS 早期
  hit 短 time 的分支，强化后续 book 剪枝。预期 10 face 收益
  10-20%。风险：变更数据表布局、需要重新校准 baseline 输出
  （新输出才是"真正最优"，但与 C 端逐字节不再等价）。
- **B. IDA\* 替代当前 DFS+book**：admissible heuristic 下界用
  `min(group.time)`，每次 deepen 阈值。工作量大。
- **C. `RotMtplRot` 改 24 个置换的查表**：cube_rot 是
  octahedral 群元素，可以编码成 0-23 的 ID + `compose[24][24]`
  查表替代 9 次循环。工作量中。
- **D. DFS 改迭代 + 显式栈**：函数调用开销不大，主要是消除
  borrow checker 友好的状态拷贝模板代码。收益微。
- **E. `Box<[BookCell; N]>` 改 `Box<[u32; N]>` 紧凑表示**：
  把 epoch+time 编码到一个 u32（高 16 位 epoch + 低 16 位 time）。
  cache 占用从 64×8 = 512 KB 降到 64 KB（注：这里是按 cache
  line 算的——实际数据 16200×8 = 130KB → 16200×4 = 65KB）。
  需要 time 不超过 i16 上限 32767。10 face 累计 time 通常
  <2000，安全。预期 cache miss 减少 20-30%。

### 5.7 顶层无状态 API

调用方目前要 `e.search(...) ; e.get_steps()` 两步。建议：

- `Engine::translate(&mut self, &str) -> (i32, String)`：一步到位
- `pub fn translate(theory: &str) -> String`：顶层无状态版（内部
  `OnceLock<Mutex<Engine>>` 持有静态实例，避免每次重建 64 KB book）

### 5.8 命名整理

- `_1 / _2 / _3` 与 Rust "未使用变量"约定视觉冲突 → `D1 / D2 / D3`
  或 `DIST_1 / DIST_2 / DIST_3`
- `L_FACE` 与机械动作 `LC / LO / L1 / L2 / L3` 容易混淆 → 统一前缀，
  例如 `FACE_L`
- 模块级常量 `L1..RO: i32` 和动作枚举值容易和 Rust 习惯（枚举/类型
  安全）拉开距离 → 可以包成 `enum MechanicalNum` + `as i32`

### 5.9 与 C 端 `bench_compare` 做大规模等价性比对

把 `RobotApp/robostep/search_opt/bench_compare.cpp` 编出 binary，
对一份 hand-step corpus（例如 1000 条）跑出 reference output；
Rust 侧加 `tools/handstep-cli` 跑同一个 corpus，逐行 diff。

如果发现差异：90% 可能踩在 §4.2/§4.3/§4.4 任一处，按需收紧或
还原语义。

---

## 6. 与 `robo-translator` 的关系

`robo-handstep`（本 crate）与 `robo-translator` 是**两条独立的技术
路线**，互不替代：

|                      | `robo-handstep` (路线 ①) | `robo-translator` (路线 ②) |
| -------------------- | ----------------------- | -------------------------- |
| 上游 solver 输出格式 | 人手记号 `"F1 R2 U3 "`   | 2L 带括号 `"(z1s0) y …"`    |
| 翻译策略             | 操作库 + DFS 时间最优   | 状态机 + 贪心括号同步     |
| 最优性保证           | 全局时间最优（基于 §4.2 的偏差，相对最优） | 只保证可执行，不优化总时长 |
| 输入文法严格度       | 必须每段 3 字符         | 自由的 2L 字符串           |
| 数据规模             | 864 条 OP_TABLE         | 0（纯算法）                |

调用层应根据上游 solver 的实际输出选择哪一条。pipeline 是否要
做格式自动分流（嗅探首字符是 `(` 还是 `[A-Z]`）属架构层决策。
