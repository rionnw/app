use anyhow::Result;
use robo_core::{CameraSource, Recognizer, Roi, SolveReport, Solver, Translator, Transport};

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

pub fn send_report<T: Transport>(transport: &mut T, report: &SolveReport) -> Result<()> {
    transport.send_steps(&report.steps)
}
