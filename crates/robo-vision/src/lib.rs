use anyhow::{Context, Result};
use image::{imageops, RgbImage};
use imageproc::stats::histogram;
use robo_core::{CubeFace, Frame, Recognizer, Roi};

const FACE_NAMES: [char; 6] = ['U', 'R', 'F', 'D', 'L', 'B'];

#[derive(Clone, Debug, Default)]
pub struct ColorClusterRecognizer;

impl Recognizer for ColorClusterRecognizer {
    fn recognize(&self, frame: &Frame, rois: &[Roi]) -> Result<CubeFace> {
        anyhow::ensure!(rois.len() == 54, "recognition requires exactly 54 ROIs");
        let colors = rois
            .iter()
            .map(|roi| mean_rgb(frame, *roi))
            .collect::<Result<Vec<_>>>()?;
        let classes = classify_by_centers(&colors)?;
        let facelets = classes
            .iter()
            .map(|&class| FACE_NAMES[class as usize])
            .collect::<String>();
        CubeFace::new(facelets)
    }
}

pub fn mean_rgb(frame: &Frame, roi: Roi) -> Result<[f32; 3]> {
    anyhow::ensure!(
        roi.width > 0 && roi.height > 0,
        "ROI must have a positive size"
    );
    anyhow::ensure!(
        roi.x + roi.width <= frame.width && roi.y + roi.height <= frame.height,
        "ROI {:?} is outside frame {}x{}",
        roi,
        frame.width,
        frame.height
    );

    let image = RgbImage::from_raw(frame.width, frame.height, frame.rgb.clone())
        .context("failed to create RGB image from frame")?;
    let crop = imageops::crop_imm(&image, roi.x, roi.y, roi.width, roi.height).to_image();
    let hist = histogram(&crop);
    let count = (roi.width * roi.height) as f32;

    Ok([
        channel_mean(&hist.channels[0], count),
        channel_mean(&hist.channels[1], count),
        channel_mean(&hist.channels[2], count),
    ])
}

fn channel_mean(histogram: &[u32; 256], count: f32) -> f32 {
    histogram
        .iter()
        .enumerate()
        .map(|(value, &weight)| value as f32 * weight as f32)
        .sum::<f32>()
        / count
}

pub fn classify_by_centers(colors: &[[f32; 3]]) -> Result<[u8; 54]> {
    anyhow::ensure!(colors.len() == 54, "expected 54 sampled colors");

    let center_indices = [4usize, 13, 22, 31, 40, 49];
    let mut centers = center_indices.map(|idx| colors[idx]);
    let mut classes = [0u8; 54];

    for (class, idx) in center_indices.into_iter().enumerate() {
        classes[idx] = class as u8;
    }

    for _ in 0..100 {
        let previous = classes;
        for (idx, color) in colors.iter().enumerate() {
            if center_indices.contains(&idx) {
                continue;
            }
            classes[idx] = nearest_center(*color, &centers)
                .with_context(|| format!("failed to classify sticker {idx}"))?
                as u8;
        }

        if previous == classes {
            break;
        }

        let mut sums = [[0f32; 3]; 6];
        let mut counts = [0u32; 6];
        for (color, &class) in colors.iter().zip(classes.iter()) {
            let class = class as usize;
            sums[class][0] += color[0];
            sums[class][1] += color[1];
            sums[class][2] += color[2];
            counts[class] += 1;
        }

        for class in 0..6 {
            if counts[class] == 0 {
                continue;
            }
            centers[class] = [
                sums[class][0] / counts[class] as f32,
                sums[class][1] / counts[class] as f32,
                sums[class][2] / counts[class] as f32,
            ];
        }
    }

    let mut counts = [0u8; 6];
    for &class in &classes {
        counts[class as usize] += 1;
    }
    anyhow::ensure!(
        counts == [9; 6],
        "recognized color counts are not 9 each: {counts:?}"
    );

    Ok(classes)
}

fn nearest_center(color: [f32; 3], centers: &[[f32; 3]; 6]) -> Result<usize> {
    centers
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            distance2(color, **a)
                .partial_cmp(&distance2(color, **b))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(idx, _)| idx)
        .context("no color centers available")
}

fn distance2(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dr = a[0] - b[0];
    let dg = a[1] - b[1];
    let db = a[2] - b[2];
    dr * dr + dg * dg + db * db
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn computes_roi_mean() {
        let frame = Frame::new_rgb(2, 1, vec![10, 20, 30, 30, 40, 50]).unwrap();
        let mean = mean_rgb(
            &frame,
            Roi {
                x: 0,
                y: 0,
                width: 2,
                height: 1,
            },
        )
        .unwrap();
        assert_eq!(mean, [20.0, 30.0, 40.0]);
    }
}
