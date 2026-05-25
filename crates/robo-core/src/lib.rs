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

pub trait Transport: Send {
    fn send_steps(&mut self, steps: &Steps) -> Result<()>;
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
