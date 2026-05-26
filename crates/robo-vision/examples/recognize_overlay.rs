use std::env;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use image::{Rgb, RgbImage};
use imageproc::drawing::draw_filled_rect_mut;
use imageproc::rect::Rect;
use robo_core::{CubeFace, Roi, Solver};
use robo_solver::Min2PhaseSolver;
use robo_vision::{
    class_label, color_class_counts, original_roi_label, recognize_original_roi_diagnostic_details,
    validate_color_class_counts,
};
use serde_json::Value;

const MARKER_SIZE: u32 = 15;

fn main() -> Result<()> {
    let args = Args::parse(env::args().skip(1).collect())?;
    let image = image::open(&args.image_path)
        .with_context(|| format!("failed to open image {}", args.image_path.display()))?
        .to_rgb8();
    let rois = load_rois(&args.roi_path, image.width(), image.height())
        .with_context(|| format!("failed to load ROI file {}", args.roi_path.display()))?;
    let frame = robo_core::Frame::new_rgb(image.width(), image.height(), image.clone().into_raw())
        .context("failed to build RGB frame")?;

    let details = recognize_original_roi_diagnostic_details(&frame, &rois)?;
    let mut annotated = image;
    draw_overlay(
        &mut annotated,
        &rois,
        &details.classes,
        &details.center_colors,
    )?;
    annotated
        .save(&args.output_path)
        .with_context(|| format!("failed to write {}", args.output_path.display()))?;

    println!("facelets: {}", details.facelets);
    println!("color counts: {:?}", color_class_counts(&details.classes));
    match validate_color_class_counts(&details.classes) {
        Ok(()) => match CubeFace::new(details.facelets.clone()) {
            Ok(face) => {
                let solver = Min2PhaseSolver::new();
                match solver.solve(&face) {
                    Ok(moves) => println!("solve: {}", moves.to_solution_string()),
                    Err(err) => println!("solve error: {err:#}"),
                }
            }
            Err(err) => println!("validation error: {err:#}"),
        },
        Err(err) => println!("validation error: {err:#}"),
    }
    println!("overlay: {}", args.output_path.display());
    println!("roi classifications:");
    for (idx, class) in details.classes.iter().enumerate() {
        println!(
            "  {:02} {} -> {}",
            idx,
            original_roi_label(idx)?,
            class_label(*class)
        );
    }

    Ok(())
}

struct Args {
    image_path: PathBuf,
    roi_path: PathBuf,
    output_path: PathBuf,
}

impl Args {
    fn parse(args: Vec<String>) -> Result<Self> {
        if args.iter().any(|arg| arg == "-h" || arg == "--help") {
            print_usage();
            std::process::exit(0);
        }

        anyhow::ensure!(
            args.len() == 2 || args.len() == 3,
            "expected IMAGE ROI_JSON [OUTPUT]\n\n{}",
            usage()
        );

        let image_path = PathBuf::from(&args[0]);
        let roi_path = PathBuf::from(&args[1]);
        let output_path = args
            .get(2)
            .map(PathBuf::from)
            .unwrap_or_else(|| default_output_path(&image_path));

        Ok(Self {
            image_path,
            roi_path,
            output_path,
        })
    }
}

fn usage() -> &'static str {
    "Usage: cargo run -p robo-vision --example recognize_overlay -- IMAGE ROI_JSON [OUTPUT]"
}

fn print_usage() {
    println!("{}", usage());
    println!();
    println!("Reads an image and 54 original-order ROIs, then writes IMAGE_STEM.recognized.png by default.");
    println!(
        "ROI JSON may be {{\"rois\":[{{\"x\",\"y\",\"width\",\"height\"}}]}} or a 54-item array."
    );
}

fn default_output_path(image_path: &Path) -> PathBuf {
    let parent = image_path.parent().unwrap_or_else(|| Path::new(""));
    let stem = image_path
        .file_stem()
        .and_then(|value| value.to_str())
        .unwrap_or("image");
    parent.join(format!("{stem}.recognized.png"))
}

fn load_rois(path: &Path, image_width: u32, image_height: u32) -> Result<Vec<Roi>> {
    let data = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    let value: Value = serde_json::from_str(&data).context("ROI JSON is not valid JSON")?;
    let items = if let Some(items) = value.as_array() {
        items
    } else {
        value
            .get("rois")
            .and_then(Value::as_array)
            .context("ROI JSON must be a 54-item array or an object with a rois array")?
    };

    anyhow::ensure!(items.len() == 54, "ROI JSON must contain exactly 54 ROIs");
    let mut rois: Vec<Option<Roi>> = vec![None; 54];
    for (array_index, item) in items.iter().enumerate() {
        let original_index = original_index_from_item(item).unwrap_or(array_index);
        anyhow::ensure!(
            original_index < 54,
            "ROI item {array_index} maps to invalid original index {original_index}"
        );
        anyhow::ensure!(
            rois[original_index].is_none(),
            "duplicate ROI for original index {original_index}"
        );
        rois[original_index] = Some(parse_roi(item, image_width, image_height)?);
    }

    rois.into_iter()
        .enumerate()
        .map(|(idx, roi)| roi.with_context(|| format!("missing ROI for original index {idx}")))
        .collect()
}

fn original_index_from_item(item: &Value) -> Option<usize> {
    let object = item.as_object()?;
    if let Some(index) = object.get("index").and_then(Value::as_u64) {
        if index < 54 {
            return Some(index as usize);
        }
    }

    let label = object
        .get("label")
        .or_else(|| object.get("id"))
        .and_then(Value::as_str)?;
    (0..54).find(|&idx| original_roi_label(idx).ok().as_deref() == Some(label))
}

fn parse_roi(value: &Value, image_width: u32, image_height: u32) -> Result<Roi> {
    let rect = value
        .get("rect")
        .filter(|candidate| candidate.is_object())
        .unwrap_or(value);
    let x = number_field(rect, "x")?;
    let y = number_field(rect, "y")?;
    let width = number_field(rect, "width").or_else(|_| number_field(rect, "w"))?;
    let height = number_field(rect, "height").or_else(|_| number_field(rect, "h"))?;
    let [x, y, width, height] = scale_rect([x, y, width, height], image_width, image_height);

    let roi = Roi {
        x: round_non_negative(x, "x")?,
        y: round_non_negative(y, "y")?,
        width: round_positive(width, "width")?,
        height: round_positive(height, "height")?,
    };
    anyhow::ensure!(
        roi.x + roi.width <= image_width && roi.y + roi.height <= image_height,
        "ROI {:?} is outside image {}x{}",
        roi,
        image_width,
        image_height
    );
    Ok(roi)
}

fn number_field(value: &Value, name: &str) -> Result<f64> {
    value
        .get(name)
        .and_then(Value::as_f64)
        .with_context(|| format!("ROI is missing numeric field {name}"))
}

fn scale_rect(rect: [f64; 4], image_width: u32, image_height: u32) -> [f64; 4] {
    if rect.iter().all(|value| *value >= 0.0 && *value <= 1.0) {
        [
            rect[0] * image_width as f64,
            rect[1] * image_height as f64,
            rect[2] * image_width as f64,
            rect[3] * image_height as f64,
        ]
    } else {
        rect
    }
}

fn round_non_negative(value: f64, name: &str) -> Result<u32> {
    anyhow::ensure!(
        value.is_finite() && value >= 0.0,
        "ROI field {name} must be non-negative"
    );
    Ok(value.round() as u32)
}

fn round_positive(value: f64, name: &str) -> Result<u32> {
    anyhow::ensure!(
        value.is_finite() && value > 0.0,
        "ROI field {name} must be positive"
    );
    Ok(value.round().max(1.0) as u32)
}

fn draw_overlay(
    image: &mut RgbImage,
    rois: &[Roi],
    classes: &[u8; 54],
    center_colors: &[[f32; 3]; 6],
) -> Result<()> {
    for (idx, roi) in rois.iter().enumerate() {
        let class = classes[idx] as usize;
        let color = rgb_from_sample(center_colors[class]);
        let rect = marker_rect(roi, image.width(), image.height());
        draw_filled_rect_mut(image, rect, color);
    }

    Ok(())
}

fn rgb_from_sample(sample: [f32; 3]) -> Rgb<u8> {
    Rgb([
        sample[0].round().clamp(0.0, 255.0) as u8,
        sample[1].round().clamp(0.0, 255.0) as u8,
        sample[2].round().clamp(0.0, 255.0) as u8,
    ])
}

fn marker_rect(roi: &Roi, image_width: u32, image_height: u32) -> Rect {
    let center_x = roi.x + roi.width / 2;
    let center_y = roi.y + roi.height / 2;
    let left = center_x
        .saturating_sub(MARKER_SIZE / 2)
        .min(image_width.saturating_sub(MARKER_SIZE));
    let top = center_y
        .saturating_sub(MARKER_SIZE / 2)
        .min(image_height.saturating_sub(MARKER_SIZE));

    Rect::at(left as i32, top as i32).of_size(MARKER_SIZE, MARKER_SIZE)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn marker_rect_uses_fixed_size_around_center() {
        let roi = Roi {
            x: 40,
            y: 50,
            width: 10,
            height: 10,
        };

        let rect = marker_rect(&roi, 200, 200);

        assert_eq!(rect.left(), 38);
        assert_eq!(rect.top(), 48);
        assert_eq!(rect.width(), 15);
        assert_eq!(rect.height(), 15);
    }

    #[test]
    fn marker_rect_clamps_to_image_bounds() {
        let roi = Roi {
            x: 0,
            y: 0,
            width: 10,
            height: 10,
        };

        let rect = marker_rect(&roi, 20, 18);

        assert_eq!(rect.left(), 0);
        assert_eq!(rect.top(), 0);
        assert_eq!(rect.width(), 15);
        assert_eq!(rect.height(), 15);
    }
}
