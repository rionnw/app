use anyhow::Result;
use robo_core::{CameraSource, DigitMap, Recognizer, Roi, SolveReport, Solver, Translator, Transport};

pub mod multi;

/// 旧版同步流水线（按 `Solver` + `Translator` trait 拼装）。新代码应优先用
/// `multi::translate_optimal`（多候选 + handstep 择优）。本结构保留向后兼容。
pub struct SolvePipeline<C, R, S, T> {
    camera: C,
    recognizer: R,
    solver: S,
    translator: T,
}

impl<C, R, S, T> SolvePipeline<C, R, S, T>
where
    C: CameraSource,
    R: Recognizer,
    S: Solver,
    T: Translator,
{
    pub fn new(camera: C, recognizer: R, solver: S, translator: T) -> Self {
        Self {
            camera,
            recognizer,
            solver,
            translator,
        }
    }

    pub fn solve_once(&mut self, rois: &[Roi]) -> Result<SolveReport> {
        let frame = self.camera.capture()?;
        let face = self.recognizer.recognize(&frame, rois)?;
        let moves = self.solver.solve(&face)?;
        let steps = self.translator.translate(&moves)?;

        Ok(SolveReport {
            facelets: face.into_string(),
            moves: moves.0,
            steps,
        })
    }
}

/// 把 `SolveReport.steps.commands` 当作 mnemonic 列表发到 transport。
/// 调用方需要传入 `digit_map`（编码用）。
pub fn send_report<T: Transport>(
    transport: &mut T,
    report: &SolveReport,
    digit_map: &DigitMap,
) -> Result<()> {
    transport.send_steps(&report.steps.commands, digit_map)
}
