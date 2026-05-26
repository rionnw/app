use anyhow::{Context, Result};
use image::{imageops, RgbImage};
use imageproc::stats::histogram;
use robo_core::{CubeFace, Frame, Recognizer, Roi};

const FACE_NAMES: [char; 6] = ['U', 'R', 'F', 'D', 'L', 'B'];
const CENTER_ORIGINAL_ROI_INDICES: [usize; 6] = [31, 40, 22, 13, 4, 49];
const SOLVER_FACELET_ORIGINAL_ROI_INDICES: [usize; 54] = [
    29, 32, 35, 28, 31, 34, 27, 30, 33, 36, 37, 38, 39, 40, 41, 42, 43, 44, 20, 23, 26, 19, 22, 25,
    18, 21, 24, 11, 14, 17, 10, 13, 16, 9, 12, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 45, 46, 47, 48, 49,
    50, 51, 52, 53,
];

#[derive(Clone, Debug, PartialEq)]
pub struct RecognitionDetails {
    pub sample_colors: Vec<[f32; 3]>,
    pub center_colors: [[f32; 3]; 6],
    pub classes: [u8; 54],
    pub labels: [String; 54],
    pub facelets: String,
}

#[derive(Clone, Debug, Default)]
pub struct ColorClusterRecognizer;

impl Recognizer for ColorClusterRecognizer {
    fn recognize(&self, frame: &Frame, rois: &[Roi]) -> Result<CubeFace> {
        let details = recognize_original_roi_details(frame, rois)?;
        let facelets = details.facelets;
        CubeFace::new(facelets)
    }
}

pub fn recognize_original_roi_details(frame: &Frame, rois: &[Roi]) -> Result<RecognitionDetails> {
    let details = recognize_original_roi_diagnostic_details(frame, rois)?;
    validate_color_class_counts(&details.classes)?;
    Ok(details)
}

pub fn recognize_original_roi_diagnostic_details(
    frame: &Frame,
    rois: &[Roi],
) -> Result<RecognitionDetails> {
    anyhow::ensure!(rois.len() == 54, "recognition requires exactly 54 ROIs");
    let sample_colors = rois
        .iter()
        .map(|roi| mean_rgb(frame, *roi))
        .collect::<Result<Vec<_>>>()?;
    let center_colors = center_original_roi_indices().map(|idx| sample_colors[idx]);
    let classes = classify_by_centers_unvalidated(&sample_colors)?;
    let labels = std::array::from_fn(|idx| class_label(classes[idx]).to_string());
    let facelets = facelets_from_original_roi_classes(&classes)?;

    Ok(RecognitionDetails {
        sample_colors,
        center_colors,
        classes,
        labels,
        facelets,
    })
}

pub fn center_original_roi_indices() -> [usize; 6] {
    CENTER_ORIGINAL_ROI_INDICES
}

pub fn original_roi_label(index: usize) -> Result<String> {
    anyhow::ensure!(index < 54, "ROI index must be in 0..54, got {index}");
    for original_roi_idx in SOLVER_FACELET_ORIGINAL_ROI_INDICES {
        if original_roi_idx == index {
            let facelet_idx = SOLVER_FACELET_ORIGINAL_ROI_INDICES
                .iter()
                .position(|&candidate| candidate == index)
                .expect("index came from solver mapping");
            let face = FACE_NAMES[facelet_idx / 9];
            let face_index = facelet_idx % 9 + 1;
            return Ok(format!("{face}{face_index}"));
        }
    }

    anyhow::bail!("missing solver mapping for ROI index {index}")
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
    let classes = classify_by_centers_unvalidated(colors)?;
    validate_color_class_counts(&classes)?;
    Ok(classes)
}

pub fn classify_by_centers_unvalidated(colors: &[[f32; 3]]) -> Result<[u8; 54]> {
    anyhow::ensure!(colors.len() == 54, "expected 54 sampled colors");

    let mut centers = CENTER_ORIGINAL_ROI_INDICES.map(|idx| colors[idx]);
    let mut classes = [0u8; 54];

    for (class, idx) in CENTER_ORIGINAL_ROI_INDICES.into_iter().enumerate() {
        classes[idx] = class as u8;
    }

    for _ in 0..100 {
        let previous = classes;
        for (idx, color) in colors.iter().enumerate() {
            if CENTER_ORIGINAL_ROI_INDICES.contains(&idx) {
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

    Ok(classes)
}

pub fn color_class_counts(classes: &[u8; 54]) -> [u8; 6] {
    let mut counts = [0u8; 6];
    for &class in classes {
        counts[class as usize] += 1;
    }
    counts
}

pub fn validate_color_class_counts(classes: &[u8; 54]) -> Result<()> {
    let counts = color_class_counts(classes);
    anyhow::ensure!(
        counts == [9; 6],
        "recognized color counts are not 9 each: {counts:?}"
    );

    Ok(())
}

pub fn facelets_from_original_roi_classes(classes: &[u8; 54]) -> Result<String> {
    let mut facelets = String::with_capacity(SOLVER_FACELET_ORIGINAL_ROI_INDICES.len());
    for original_roi_idx in SOLVER_FACELET_ORIGINAL_ROI_INDICES {
        let class = classes[original_roi_idx] as usize;
        let face = FACE_NAMES
            .get(class)
            .with_context(|| format!("invalid color class {class} at ROI {original_roi_idx}"))?;
        facelets.push(*face);
    }
    Ok(facelets)
}

pub fn class_label(class: u8) -> char {
    FACE_NAMES.get(class as usize).copied().unwrap_or('?')
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
    use robo_core::SOLVED_FACE;

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

    #[test]
    fn recognizes_solved_cube_from_original_roi_order() {
        let colors = original_roi_order_solved_colors();
        let frame = Frame::new_rgb(
            colors.len() as u32,
            1,
            colors
                .iter()
                .flat_map(|color| color.iter().copied())
                .collect::<Vec<_>>(),
        )
        .unwrap();
        let rois = (0..54)
            .map(|idx| Roi {
                x: idx,
                y: 0,
                width: 1,
                height: 1,
            })
            .collect::<Vec<_>>();

        let face = ColorClusterRecognizer.recognize(&frame, &rois).unwrap();

        assert_eq!(face.as_str(), SOLVED_FACE);
    }

    #[test]
    fn reorders_original_roi_classes_to_solver_facelets() {
        let classes = original_roi_order_solved_classes();

        let facelets = facelets_from_original_roi_classes(&classes).unwrap();

        assert_eq!(facelets, SOLVED_FACE);
    }

    #[test]
    fn exposes_original_roi_labels_and_center_indices() {
        assert_eq!(original_roi_label(31).unwrap(), "U5");
        assert_eq!(original_roi_label(40).unwrap(), "R5");
        assert_eq!(original_roi_label(22).unwrap(), "F5");
        assert_eq!(original_roi_label(13).unwrap(), "D5");
        assert_eq!(original_roi_label(4).unwrap(), "L5");
        assert_eq!(original_roi_label(49).unwrap(), "B5");
        assert_eq!(center_original_roi_indices(), [31, 40, 22, 13, 4, 49]);
    }

    #[test]
    fn samples_recognition_details_in_original_roi_order() {
        let colors = original_roi_order_solved_colors();
        let frame = Frame::new_rgb(
            colors.len() as u32,
            1,
            colors
                .iter()
                .flat_map(|color| color.iter().copied())
                .collect::<Vec<_>>(),
        )
        .unwrap();
        let rois = (0..54)
            .map(|idx| Roi {
                x: idx,
                y: 0,
                width: 1,
                height: 1,
            })
            .collect::<Vec<_>>();

        let details = recognize_original_roi_details(&frame, &rois).unwrap();

        assert_eq!(details.facelets, SOLVED_FACE);
        assert_eq!(details.classes[31], 0);
        assert_eq!(details.labels[31], "U");
        assert_eq!(details.center_colors[0], [240.0, 240.0, 240.0]);
        assert_eq!(details.sample_colors[31], [240.0, 240.0, 240.0]);
    }

    #[test]
    fn samples_diagnostic_details_when_color_counts_are_invalid() {
        let mut colors = original_roi_order_solved_colors();
        colors[0] = colors[31];
        let frame = Frame::new_rgb(
            colors.len() as u32,
            1,
            colors
                .iter()
                .flat_map(|color| color.iter().copied())
                .collect::<Vec<_>>(),
        )
        .unwrap();
        let rois = (0..54)
            .map(|idx| Roi {
                x: idx,
                y: 0,
                width: 1,
                height: 1,
            })
            .collect::<Vec<_>>();

        let details = recognize_original_roi_diagnostic_details(&frame, &rois).unwrap();

        assert_eq!(details.classes[0], 0);
        assert_eq!(color_class_counts(&details.classes), [10, 9, 9, 9, 8, 9]);
        assert!(validate_color_class_counts(&details.classes).is_err());
        assert!(recognize_original_roi_details(&frame, &rois).is_err());
    }

    fn original_roi_order_solved_colors() -> [[u8; 3]; 54] {
        let face_colors = [
            (b'U', [240, 240, 240]),
            (b'R', [220, 40, 40]),
            (b'F', [40, 180, 80]),
            (b'D', [240, 220, 40]),
            (b'L', [240, 140, 40]),
            (b'B', [40, 80, 220]),
        ];
        let labels = [
            b'L', b'L', b'L', b'L', b'L', b'L', b'L', b'L', b'L', b'D', b'D', b'D', b'D', b'D',
            b'D', b'D', b'D', b'D', b'F', b'F', b'F', b'F', b'F', b'F', b'F', b'F', b'F', b'U',
            b'U', b'U', b'U', b'U', b'U', b'U', b'U', b'U', b'R', b'R', b'R', b'R', b'R', b'R',
            b'R', b'R', b'R', b'B', b'B', b'B', b'B', b'B', b'B', b'B', b'B', b'B',
        ];
        labels.map(|label| {
            face_colors
                .iter()
                .find_map(|(face, color)| (*face == label).then_some(*color))
                .unwrap()
        })
    }

    fn original_roi_order_solved_classes() -> [u8; 54] {
        [
            4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 5, 5, 5, 5,
        ]
    }
}
