use anyhow::Result;
use robo_core::{Moves, Steps, Translator};

#[derive(Clone, Debug, Default)]
pub struct BasicTranslator;

impl BasicTranslator {
    pub fn new() -> Self {
        Self
    }
}

impl Translator for BasicTranslator {
    fn translate(&self, moves: &Moves) -> Result<Steps> {
        let commands = moves
            .as_slice()
            .iter()
            .map(|mv| encode_move(mv))
            .collect::<Result<Vec<_>>>()?;
        let encoded = commands.join("");
        Ok(Steps { commands, encoded })
    }
}

fn encode_move(mv: &str) -> Result<String> {
    let face_code = match mv.chars().next() {
        Some('U') => "U",
        Some('R') => "R",
        Some('F') => "F",
        Some('D') => "D",
        Some('L') => "L",
        Some('B') => "B",
        _ => anyhow::bail!("unsupported move '{mv}'"),
    };
    let amount_code = if mv.ends_with('2') {
        "2"
    } else if mv.ends_with('\'') {
        "3"
    } else {
        "1"
    };
    Ok(format!("{face_code}{amount_code};"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_move_tokens() {
        let steps = BasicTranslator::new()
            .translate(&Moves(vec!["R".into(), "U2".into(), "F'".into()]))
            .unwrap();
        assert_eq!(steps.encoded, "R1;U2;F3;");
    }
}
