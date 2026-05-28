#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::new_without_default)]
#[allow(clippy::needless_range_loop)]
mod util;
#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::new_without_default)]
#[allow(clippy::needless_range_loop)]
mod cubie_cube;
#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::new_without_default)]
#[allow(clippy::needless_range_loop)]
mod coord_cube;
#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::new_without_default)]
#[allow(clippy::needless_range_loop)]
mod search;
#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::new_without_default)]
#[allow(clippy::needless_range_loop)]
mod cubie_cube_2l;
#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::new_without_default)]
#[allow(clippy::needless_range_loop)]
mod coord_cube_2l;
#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::new_without_default)]
#[allow(clippy::needless_range_loop)]
mod search_2l;

use anyhow::Result;
use robo_core::{CubeFace, Moves, Solver};
use search_2l::Search2L;
use std::sync::Mutex;

/// Rubik's Cube solver using min2phase Search2L (two-layer / robot leg-move variant).
pub struct Min2PhaseSolver {
    inner: Mutex<Search2L>,
}

impl Default for Min2PhaseSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl Min2PhaseSolver {
    pub fn new() -> Self {
        Search2L::init();
        Self {
            inner: Mutex::new(Search2L::new()),
        }
    }
}

impl Solver for Min2PhaseSolver {
    fn solve(&self, face: &CubeFace) -> Result<Moves> {
        let facelets = face.as_str();

        let mut search = self.inner.lock().unwrap();
        let solution = search.solution(facelets, 70, 10_000_000, 1500, 0);

        if solution.starts_with("Error") {
            anyhow::bail!("min2phase Search2L failed: {solution}");
        }

        Ok(Moves::from_solution_string(&solution))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solved_cube_returns_no_moves() {
        let face = CubeFace::solved();
        let moves = Min2PhaseSolver::new().solve(&face).unwrap();
        assert!(moves.0.is_empty() || moves.0.iter().all(|m| m.trim().is_empty()));
    }
}
