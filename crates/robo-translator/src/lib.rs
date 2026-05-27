use anyhow::{bail, Result};
use robo_core::{Moves, Steps, Translator};

/// MOVE2STR patterns from the 2L solver output (20 leg-moves).
/// Each entry is the pattern prefix (without trailing space padding).
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
    [ 1,  0,  1,  2,  0,  2,  1,  0,  1,  2,  0,  2,  2,  1, -1,  2, -1, -1,  1, -1], // pp
    [ 0,  1,  0, -1, -1, -1,  0,  1,  0, -1,  1, -1, -1,  0,  2, -1,  2,  2,  0,  2], // vp
    [-1, -1, -1,  0,  2,  0, -1,  2, -1,  0,  2,  0,  0, -1,  1,  0,  1,  1, -1,  1], // pv
];

/// Hardware motor command sequences for each of the 20 moves.
fn move_sequence(m: usize) -> &'static [&'static str] {
    match m {
        // Move 0-2: L rotates (single layer U)
        0 => &["ROTATE_L(+90);"],
        1 => &["ROTATE_L(+180);"],
        2 => &["ROTATE_L(-90);"],
        // Move 3-5: R rotates (single layer R)
        3 => &["ROTATE_R(+90);"],
        4 => &["ROTATE_R(+180);"],
        5 => &["ROTATE_R(-90);"],
        // Move 6-8: R claw opens, L rotates whole cube, R claw closes
        6 => &["CLAW_R(0);", "ROTATE_L(+90);", "CLAW_R(1);"],
        7 => &["CLAW_R(0);", "ROTATE_L(+180);", "CLAW_R(1);"],
        8 => &["CLAW_R(0);", "ROTATE_L(-90);", "CLAW_R(1);"],
        // Move 9-11: L claw opens, R rotates whole cube, L claw closes
        9  => &["CLAW_L(0);", "ROTATE_R(+90);", "CLAW_L(1);"],
        10 => &["CLAW_L(0);", "ROTATE_R(+180);", "CLAW_L(1);"],
        11 => &["CLAW_L(0);", "ROTATE_R(-90);", "CLAW_L(1);"],
        // Move 12: R side toggle (regrab)
        12 => &["CLAW_R(0);", "CLAW_R(1);"],
        // Move 13: L side toggle (regrab)
        13 => &["CLAW_L(0);", "CLAW_L(1);"],
        // Move 14-16: R claw toggles + L rotates
        14 => &["CLAW_R(0);", "ROTATE_L(+90);", "CLAW_R(1);"],
        15 => &["CLAW_R(0);", "ROTATE_L(+180);", "CLAW_R(1);"],
        16 => &["CLAW_R(0);", "ROTATE_L(-90);", "CLAW_R(1);"],
        // Move 17-19: L claw toggles + R rotates
        17 => &["CLAW_L(0);", "ROTATE_R(+90);", "CLAW_L(1);"],
        18 => &["CLAW_L(0);", "ROTATE_R(+180);", "CLAW_L(1);"],
        19 => &["CLAW_L(0);", "ROTATE_R(-90);", "CLAW_L(1);"],
        _ => &[],
    }
}

/// Parse the 2L solver output string into move indices.
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
            // Skip one character on unrecognized input
            s = &s[1..];
        }
    }
    moves
}

/// Convert move indices to hardware command list.
fn moves_to_hardware(move_indices: &[usize]) -> Result<Vec<String>> {
    let mut commands = Vec::new();
    let mut leg: usize = 0;

    // Init: both claws closed
    commands.push("CLAW_L(1);".to_string());
    commands.push("CLAW_R(1);".to_string());

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

        for cmd in move_sequence(m) {
            commands.push(cmd.to_string());
        }

        leg = next_leg as usize;
    }

    Ok(commands)
}

#[derive(Clone, Debug, Default)]
pub struct BasicTranslator;

impl BasicTranslator {
    pub fn new() -> Self {
        Self
    }
}

impl Translator for BasicTranslator {
    fn translate(&self, moves: &Moves) -> Result<Steps> {
        // The Moves from Search2L is the raw solution string split by whitespace.
        // We need to reconstruct the full string and re-parse it with pattern matching.
        let solution_str = moves.to_solution_string();
        let move_indices = parse_solution(&solution_str);

        if move_indices.is_empty() {
            return Ok(Steps {
                commands: vec![],
                encoded: String::new(),
            });
        }

        let commands = moves_to_hardware(&move_indices)?;
        let encoded = commands.join(" ");

        Ok(Steps { commands, encoded })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_known_solution() {
        let raw = "(z1s0) y  (z1z0) U  (s1z2) x2 (z3z0) U'";
        let indices = parse_solution(raw);
        assert_eq!(indices, vec![6, 0, 18, 2]);
    }

    #[test]
    fn translate_simple_moves() {
        // Move 0 (U): leg 0→1, then Move 2 (U'): leg 1→0
        let raw = "(z1z0) U  (z3z0) U'";
        let moves = Moves::from_solution_string(raw);
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert!(steps.commands.contains(&"ROTATE_L(+90);".to_string()));
        assert!(steps.commands.contains(&"ROTATE_L(-90);".to_string()));
    }

    #[test]
    fn translate_full_example() {
        let raw = "(z1s0) y  (z1z0) U  (s1z2) x2 (z3z0) U' (z0z3) R' (z2s1) y2 (z0z2) R2 (z1z0) U  (z1s1) y  (z0z1) R  (z3z0) U' (s1z3) x' (z0z1) R  (z2z0) U2 (z0z1) R  (z2s0) y2 (z0z3) R' (z3z0) U' (s1z3) x' (z0z2) R2 (z2s0) y2 (z0z2) R2 (z1s1) y  (z1z0) U  (z0z2) R2 (s1z2) x2 (z1z0) U  (z0z2) R2 (s1z0)    (z1s0) y  (z0z2) R2";
        let moves = Moves::from_solution_string(raw);
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert!(!steps.commands.is_empty());
        // Count motor ops (non-init)
        let ops = steps.commands.len();
        assert!(ops > 30); // Should have many commands for 31 moves
    }

    #[test]
    fn empty_solution() {
        let moves = Moves::from_solution_string("");
        let steps = BasicTranslator::new().translate(&moves).unwrap();
        assert!(steps.commands.is_empty());
    }
}
