#!/usr/bin/env python3
"""一次性脚本：把 RobotApp/RobotStepData.inc 的 OP_TABLE 转成 Rust 常量。

用法（从仓库根目录）：
    python3 crates/robo-handstep/scripts/convert_op_table.py \
        > crates/robo-handstep/src/op_table.rs

不做语义改动；只把 C 字面量数字原样塞进 Rust 数组。
"""
from __future__ import annotations
import re
import sys
from pathlib import Path

# C 端常量名 → 数值（与 RobotStep.h 一致）
NAME_TO_NUM = {
    "F": 0, "R": 1, "U": 2, "B": 3, "L": 4, "D": 5,
    "_1": 0, "_2": 1, "_3": 2,
    "L_0_R_0": 0, "L_0_R_1": 1, "L_1_R_0": 2,
    "L1": 0, "L2": 1, "L3": 2, "LC": 3, "LO": 4,
    "R1": 5, "R2": 6, "R3": 7, "RC": 8, "RO": 9,
}

ROW_RE = re.compile(
    r"\{\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,"
    r"\s*\{([^}]*)\}\s*,\s*\{([^}]*)\}\s*,\s*\{([^}]*)\}\s*\}",
    re.S,
)


def parse_int_list(s: str) -> list[int]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    out = []
    for p in parts:
        out.append(int(p) if p.lstrip("-").isdigit() else NAME_TO_NUM[p])
    return out


def main() -> int:
    root = Path(__file__).resolve().parents[3]
    inc = root / "RobotApp" / "RobotStepData.inc"
    text = inc.read_text(encoding="utf-8", errors="replace")

    rows = []
    for m in ROW_RE.finditer(text):
        face, dist, state, variant, step_count = m.group(1), m.group(2), m.group(3), int(m.group(4)), int(m.group(5))
        steps = parse_int_list(m.group(6))
        rot = parse_int_list(m.group(7))
        hs = parse_int_list(m.group(8))
        # 把 steps 补齐到 20 长度，-1 填充（与 C 端布局一致）
        if len(steps) < 20:
            steps = steps + [-1] * (20 - len(steps))
        else:
            steps = steps[:20]
        assert len(rot) == 6, f"rot 长度异常: {rot}"
        assert len(hs) == 4, f"hs 长度异常: {hs}"
        rows.append((NAME_TO_NUM[face], NAME_TO_NUM[dist], NAME_TO_NUM[state], variant, step_count, steps, rot, hs))

    out = []
    out.append("// 自动生成：勿手改。源：RobotApp/RobotStepData.inc")
    out.append("// 生成器：crates/robo-handstep/scripts/convert_op_table.py")
    out.append("")
    out.append("use crate::types::OpEntry;")
    out.append("")
    out.append(f"pub const OP_TABLE_SIZE: usize = {len(rows)};")
    out.append("")
    out.append(f"pub static OP_TABLE: [OpEntry; OP_TABLE_SIZE] = [")
    for face, dist, state, variant, step_count, steps, rot, hs in rows:
        steps_s = ", ".join(str(x) for x in steps)
        rot_s = ", ".join(str(x) for x in rot)
        hs_s = ", ".join(str(x) for x in hs)
        out.append(
            f"    OpEntry {{ face: {face}, dist: {dist}, state: {state}, variant: {variant}, "
            f"step_count: {step_count}, steps: [{steps_s}], rot: [{rot_s}], hs: [{hs_s}] }},"
        )
    out.append("];")
    out.append("")
    sys.stdout.write("\n".join(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
