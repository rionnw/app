use anyhow::{Context, Result};
use robo_core::{CubeFace, Moves, Solver};
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int};
use std::ptr;

extern "C" {
    fn robo_min2phase_solve(
        facelets: *const c_char,
        solution: *mut *mut c_char,
        error: *mut *mut c_char,
    ) -> c_int;
    fn robo_min2phase_free(value: *mut c_char);
}

#[derive(Clone, Debug, Default)]
pub struct Min2PhaseSolver;

impl Min2PhaseSolver {
    pub fn new() -> Self {
        Self
    }
}

impl Solver for Min2PhaseSolver {
    fn solve(&self, face: &CubeFace) -> Result<Moves> {
        let input = CString::new(face.as_str()).context("cube face contains NUL byte")?;
        let mut solution_ptr: *mut c_char = ptr::null_mut();
        let mut error_ptr: *mut c_char = ptr::null_mut();

        let code =
            unsafe { robo_min2phase_solve(input.as_ptr(), &mut solution_ptr, &mut error_ptr) };

        if code != 0 {
            let message = unsafe { take_c_string(error_ptr) }
                .unwrap_or_else(|| format!("min2phase failed with code {code}"));
            anyhow::bail!("{message}");
        }

        let solution = unsafe { take_c_string(solution_ptr) }.unwrap_or_default();
        Ok(Moves::from_solution_string(&solution))
    }
}

unsafe fn take_c_string(ptr: *mut c_char) -> Option<String> {
    if ptr.is_null() {
        return None;
    }
    let value = CStr::from_ptr(ptr).to_string_lossy().into_owned();
    robo_min2phase_free(ptr);
    Some(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solved_cube_returns_no_moves() {
        let face = CubeFace::solved();
        let moves = Min2PhaseSolver::new().solve(&face).unwrap();
        assert!(moves.0.is_empty());
    }
}
