use anyhow::Result;
use serde::{Deserialize, Serialize};

pub const FACELET_COUNT: usize = 54;
pub const SOLVED_FACE: &str = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CubeFace(String);

impl CubeFace {
    pub fn new(facelets: impl Into<String>) -> Result<Self> {
        let facelets = facelets.into();
        validate_facelets(&facelets)?;
        Ok(Self(facelets))
    }

    pub fn solved() -> Self {
        Self(SOLVED_FACE.to_string())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn into_string(self) -> String {
        self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Moves(pub Vec<String>);

impl Moves {
    pub fn empty() -> Self {
        Self(Vec::new())
    }

    pub fn from_solution_string(solution: &str) -> Self {
        Self(solution.split_whitespace().map(ToOwned::to_owned).collect())
    }

    pub fn to_solution_string(&self) -> String {
        self.0.join(" ")
    }

    pub fn as_slice(&self) -> &[String] {
        &self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Steps {
    pub commands: Vec<String>,
    pub encoded: String,
}

impl Steps {
    pub fn empty() -> Self {
        Self {
            commands: Vec::new(),
            encoded: String::new(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Roi {
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Frame {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub rgb: Vec<u8>,
}

impl Frame {
    pub fn new_rgb(width: u32, height: u32, rgb: Vec<u8>) -> Result<Self> {
        let expected = width as usize * height as usize * 3;
        if rgb.len() != expected {
            anyhow::bail!("RGB frame has {} bytes, expected {}", rgb.len(), expected);
        }

        Ok(Self {
            width,
            height,
            stride: width * 3,
            rgb,
        })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SolveReport {
    pub facelets: String,
    pub moves: Vec<String>,
    pub steps: Steps,
}

pub trait CameraSource {
    fn capture(&mut self) -> Result<Frame>;
}

pub trait Recognizer: Send + Sync {
    fn recognize(&self, frame: &Frame, rois: &[Roi]) -> Result<CubeFace>;
}

pub trait Solver: Send + Sync {
    fn solve(&self, face: &CubeFace) -> Result<Moves>;
}

pub trait Translator: Send + Sync {
    fn translate(&self, moves: &Moves) -> Result<Steps>;
}

/// 下位机数字映射的固定槽位数（与 mnemonic 数量一致）。
///
/// 索引顺序与 handstep 的 `MNEMONIC_STR` / robo-translator 的 `MNEMONICS` 一致：
/// `[M_L1, M_L2, M_L3, M_LC, M_LO, M_R1, M_R2, M_R3, M_RC, M_RO]`
pub const MNEMONIC_COUNT: usize = 10;

/// 下位机数字映射类型别名（10 个字符串槽位）。
///
/// `digit_map[i]` 表示第 i 个 mnemonic（按 `MNEMONIC_COUNT` 注释中的索引顺序）
/// 在下位机协议里的字符表示，通常是单个数字字符（如 `"4"`），但允许多字符。
pub type DigitMap = [String; MNEMONIC_COUNT];

pub trait Transport: Send {
    /// 发送机械动作序列到下位机。
    ///
    /// `mnemonics` 是助记符列表（如 `["M_L1", "M_LO", ...]`，来自 handstep
    /// 的 `Engine::get_mnemonics()`）。`digit_map` 把每个 mnemonic 翻译成
    /// 下位机协议里的字符表示——这是设备特有协议参数，由调用方持有。
    ///
    /// 实现负责按 `digit_map` 编码后写入硬件。`mnemonics` 中如果包含
    /// 不在 `MNEMONIC_STR` 集合的项，实现可以选择跳过或返回 Err。
    fn send_steps(&mut self, mnemonics: &[String], digit_map: &DigitMap) -> Result<()>;
}

pub fn validate_facelets(facelets: &str) -> Result<()> {
    if facelets.len() != FACELET_COUNT {
        anyhow::bail!(
            "cube face must contain {FACELET_COUNT} chars, got {}",
            facelets.len()
        );
    }

    let mut counts = [0usize; 6];
    for ch in facelets.bytes() {
        let idx = match ch {
            b'U' => 0,
            b'R' => 1,
            b'F' => 2,
            b'D' => 3,
            b'L' => 4,
            b'B' => 5,
            _ => anyhow::bail!("invalid cube face char '{}'", ch as char),
        };
        counts[idx] += 1;
    }

    if counts.iter().any(|&count| count != 9) {
        anyhow::bail!("cube face color counts must be 9 each, got {counts:?}");
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validates_solved_face() {
        assert!(CubeFace::new(SOLVED_FACE).is_ok());
    }

    #[test]
    fn parses_moves() {
        let moves = Moves::from_solution_string("R U R' U2");
        assert_eq!(moves.0, ["R", "U", "R'", "U2"]);
    }
}
