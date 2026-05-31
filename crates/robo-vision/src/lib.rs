use anyhow::{Context, Result};
use image::{imageops, RgbImage};
use imageproc::stats::histogram;
use robo_core::{CubeFace, Frame, Recognizer, Roi};

const FACE_NAMES: [char; 6] = ['U', 'R', 'F', 'D', 'L', 'B'];
// Authoritative face → 9 ROI indices mapping mirrors the legacy
// RubiksCubeSolver `map[]`/`table[]` pair. Order is U, R, F, D, L, B; each
// face lists the click indices (0..53 in user-marking order) that feed solver
// facelet slots X1..X9 in row-major.
const CENTER_ORIGINAL_ROI_INDICES: [usize; 6] = [49, 40, 31, 22, 4, 13];
const SOLVER_FACELET_ORIGINAL_ROI_INDICES: [usize; 54] = [
    53, 52, 51, 50, 49, 48, 47, 46, 45, 38, 41, 44, 37, 40, 43, 36, 39, 42, 29, 32, 35, 28, 31, 34,
    27, 30, 33, 20, 23, 26, 19, 22, 25, 18, 21, 24, 6, 3, 0, 7, 4, 1, 8, 5, 2, 15, 12, 9, 16, 13,
    10, 17, 14, 11,
];

#[derive(Clone, Debug, PartialEq)]
pub struct RecognitionDetails {
    pub sample_colors: Vec<[f32; 3]>,
    pub center_colors: [[f32; 3]; 6],
    pub classes: [u8; 54],
    pub labels: [String; 54],
    pub facelets: String,
}

/// 颜色聚类算法变体。
///
/// - `Rgb`：默认实现，直接在 RGB 空间做 KMeans（54 sticker，6 类）。
/// - `Cpp`：复刻 RubiksCubeSolver 旧 C++ `clusterWithKnn`：BGR → RR 旋转矩阵
///   投影 → z/=2 → 整数 KMeans。常量与算法不变量与原版一致。
/// - `Lab`：在 CIE L\*a\*b\* 空间做 KMeans，色差感知比 RGB 更鲁棒。
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum ColorClassifierKind {
    #[default]
    Rgb,
    Cpp,
    Lab,
}

#[derive(Clone, Debug, Default)]
pub struct ColorClusterRecognizer {
    pub kind: ColorClassifierKind,
}

impl Recognizer for ColorClusterRecognizer {
    fn recognize(&self, frame: &Frame, rois: &[Roi]) -> Result<CubeFace> {
        let details = recognize_original_roi_details(frame, rois, self.kind)?;
        let facelets = details.facelets;
        CubeFace::new(facelets)
    }
}

pub fn recognize_original_roi_details(
    frame: &Frame,
    rois: &[Roi],
    kind: ColorClassifierKind,
) -> Result<RecognitionDetails> {
    let details = recognize_original_roi_diagnostic_details(frame, rois, kind)?;
    validate_color_class_counts(&details.classes)?;
    Ok(details)
}

pub fn recognize_original_roi_diagnostic_details(
    frame: &Frame,
    rois: &[Roi],
    kind: ColorClassifierKind,
) -> Result<RecognitionDetails> {
    anyhow::ensure!(rois.len() == 54, "recognition requires exactly 54 ROIs");
    let sample_colors = rois
        .iter()
        .map(|roi| mean_rgb(frame, *roi))
        .collect::<Result<Vec<_>>>()?;
    let center_colors = center_original_roi_indices().map(|idx| sample_colors[idx]);
    let classes = classify_by_centers_unvalidated_with_kind(&sample_colors, kind)?;
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

/// 默认 RGB 空间 KMeans（保留旧 API，等价于 `classify_by_centers_rgb`）。
pub fn classify_by_centers_unvalidated(colors: &[[f32; 3]]) -> Result<[u8; 54]> {
    classify_by_centers_unvalidated_with_kind(colors, ColorClassifierKind::Rgb)
}

/// 三种空间统一入口：根据 `kind` 选择 RGB / CPP / LAB 分类器。
pub fn classify_by_centers_unvalidated_with_kind(
    colors: &[[f32; 3]],
    kind: ColorClassifierKind,
) -> Result<[u8; 54]> {
    match kind {
        ColorClassifierKind::Rgb => classify_by_centers_rgb(colors),
        ColorClassifierKind::Cpp => classify_by_centers_cpp(colors),
        ColorClassifierKind::Lab => classify_by_centers_lab(colors),
    }
}

/// RGB 空间 KMeans。等价于历史 `classify_by_centers_unvalidated` 行为，未做色彩转换。
pub fn classify_by_centers_rgb(colors: &[[f32; 3]]) -> Result<[u8; 54]> {
    anyhow::ensure!(colors.len() == 54, "expected 54 sampled colors");
    let samples: [[f32; 3]; 54] = std::array::from_fn(|idx| colors[idx]);
    Ok(run_kmeans_with_centers_f32(&samples))
}

/// 复刻 RubiksCubeSolver 旧 C++ `ColorCluster::clusterWithKnn`：
///
/// 1. 输入 RGB → 转 BGR（旧版 OpenCV `cv::Scalar(B, G, R)`）。
/// 2. 用常量化的 RR 矩阵投影到旋转坐标（B1=35°、B2=132°，π=3.141592654）。
/// 3. z 分量整数除 2（关键：原版有"y'/2 截断"操作，模拟扁圆色相空间）。
/// 4. 整数 KMeans，初始中心 = 6 个 ROI 中心位置投影后的整型值；只对 i=0..47
///    重分类，i=48..53 永远固定为对应 class；终止条件：本轮 idx == 上一轮 idx；
///    质心更新用 `(Center*(n-1)+sample)/n` 的增量整数平均。
///
/// 中心初始化按仓内 `CENTER_ORIGINAL_ROI_INDICES`（U R F D L B 顺序），因此
/// 输出 class 0..5 直接对齐新版 `FACE_NAMES`，无需做旧版 F R U B L D ↔ 新版的转换。
pub fn classify_by_centers_cpp(colors: &[[f32; 3]]) -> Result<[u8; 54]> {
    anyhow::ensure!(colors.len() == 54, "expected 54 sampled colors");

    /// 旧 C++ ColorCluster 用的字面量 π，刻意保持不变以保证 RR 矩阵一致。
    const PI_LEGACY: f64 = 3.141592654;
    /// 角度（度），Ry 旋转。
    const B1_DEG: f64 = 35.0;
    /// 角度（度），Rx 旋转。
    const B2_DEG: f64 = 132.0;

    // RR = Ry · Rx，按旧 C++ 推导的 9 个数值（运行期一次性计算 → 编译器常量折叠）。
    let b1 = B1_DEG * PI_LEGACY / 180.0;
    let b2 = B2_DEG * PI_LEGACY / 180.0;
    let (sb1, cb1) = (b1.sin(), b1.cos());
    let (sb2, cb2) = (b2.sin(), b2.cos());
    // 标准 Rx (绕 X 轴, 角度=B1) = [[1,0,0],[0,cB1,-sB1],[0,sB1,cB1]]
    // 标准 Ry (绕 Y 轴, 角度=B2) = [[cB2,0,sB2],[0,1,0],[-sB2,0,cB2]]
    // RR = Ry · Rx
    let rr: [[f64; 3]; 3] = [
        [cb2, sb2 * sb1, sb2 * cb1],
        [0.0, cb1, -sb1],
        [-sb2, cb2 * sb1, cb2 * cb1],
    ];

    // 把 RGB f32 → BGR i32 → RR 投影 → z/=2 截断（与旧 C++ int 运算保持位级一致风格）。
    let project = |rgb: [f32; 3]| -> [i32; 3] {
        let b = rgb[2].round() as i32;
        let g = rgb[1].round() as i32;
        let r = rgb[0].round() as i32;
        let bgr = [b as f64, g as f64, r as f64];
        let mut out = [0i32; 3];
        for row in 0..3 {
            let v = rr[row][0] * bgr[0] + rr[row][1] * bgr[1] + rr[row][2] * bgr[2];
            out[row] = v as i32; // 截断（与 C++ static_cast<int> 行为一致）
        }
        out[2] /= 2; // 旧版 z/2，模拟蓝色通道压缩
        out
    };

    let projected: [[i32; 3]; 54] = std::array::from_fn(|idx| project(colors[idx]));
    Ok(run_kmeans_with_centers_i32(&projected))
}

/// CIE L\*a\*b\* 空间 KMeans。sRGB（gamma 2.2 简化）→ linear RGB → XYZ(D65) → Lab。
/// 与 RGB 分类器同样的 KMeans 主循环，仅样本空间不同。
pub fn classify_by_centers_lab(colors: &[[f32; 3]]) -> Result<[u8; 54]> {
    anyhow::ensure!(colors.len() == 54, "expected 54 sampled colors");
    let lab: [[f32; 3]; 54] = std::array::from_fn(|idx| rgb_to_lab(colors[idx]));
    Ok(run_kmeans_with_centers_f32(&lab))
}

/// f32 KMeans 主循环；样本顺序与 `colors` 一致，初始中心 = 6 个 ROI 中心位置的样本。
/// 终止条件、固定中心索引等不变量与历史 RGB 实现完全一致。
fn run_kmeans_with_centers_f32(samples: &[[f32; 3]; 54]) -> [u8; 54] {
    let mut centers: [[f32; 3]; 6] = CENTER_ORIGINAL_ROI_INDICES.map(|idx| samples[idx]);
    let mut classes = [0u8; 54];
    for (class, idx) in CENTER_ORIGINAL_ROI_INDICES.into_iter().enumerate() {
        classes[idx] = class as u8;
    }

    for _ in 0..100 {
        let previous = classes;
        for (idx, color) in samples.iter().enumerate() {
            if CENTER_ORIGINAL_ROI_INDICES.contains(&idx) {
                continue;
            }
            classes[idx] = nearest_center_f32(*color, &centers) as u8;
        }
        if previous == classes {
            break;
        }
        let mut sums = [[0f32; 3]; 6];
        let mut counts = [0u32; 6];
        for (color, &class) in samples.iter().zip(classes.iter()) {
            let c = class as usize;
            sums[c][0] += color[0];
            sums[c][1] += color[1];
            sums[c][2] += color[2];
            counts[c] += 1;
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
    classes
}

/// 旧 C++ 风格的整数 KMeans 主循环：i32 样本/中心、整数除法、增量整数平均。
fn run_kmeans_with_centers_i32(samples: &[[i32; 3]; 54]) -> [u8; 54] {
    // Center: i32[3][6]，按仓内顺序 U R F D L B 与 CENTER_ORIGINAL_ROI_INDICES 对齐。
    let mut centers: [[i32; 3]; 6] = CENTER_ORIGINAL_ROI_INDICES.map(|idx| samples[idx]);
    let mut classes = [0u8; 54];
    for (class, idx) in CENTER_ORIGINAL_ROI_INDICES.into_iter().enumerate() {
        classes[idx] = class as u8;
    }
    // count[class]：当前 class 已纳入"增量整数平均"的样本数（含初始中心 1 个）。
    let mut counts = [1u32; 6];

    for _ in 0..100 {
        let previous = classes;
        for (idx, sample) in samples.iter().enumerate() {
            if CENTER_ORIGINAL_ROI_INDICES.contains(&idx) {
                continue;
            }
            let class = nearest_center_i32(*sample, &centers) as usize;
            classes[idx] = class as u8;

            // 旧版 C++：count++ 后 Center = (Center * (count-1) + sample) / count；
            // 整数除法截断，故每轮收敛取决于 idx 序列与中心更新时机。
            counts[class] = counts[class].saturating_add(1);
            let n = counts[class] as i32;
            for k in 0..3 {
                centers[class][k] = (centers[class][k] * (n - 1) + sample[k]) / n;
            }
        }
        if previous == classes {
            break;
        }
    }
    classes
}

fn nearest_center_f32(color: [f32; 3], centers: &[[f32; 3]; 6]) -> usize {
    let mut best = 0usize;
    let mut best_dist = f32::INFINITY;
    for (idx, c) in centers.iter().enumerate() {
        let d = distance2(color, *c);
        if d < best_dist {
            best_dist = d;
            best = idx;
        }
    }
    best
}

fn nearest_center_i32(sample: [i32; 3], centers: &[[i32; 3]; 6]) -> usize {
    let mut best = 0usize;
    // 旧 C++ 用 double 初值 1e8；这里用 i64 以避免 i32 平方溢出。
    let mut best_dist: i64 = i64::MAX;
    for (idx, c) in centers.iter().enumerate() {
        let dx = (sample[0] - c[0]) as i64;
        let dy = (sample[1] - c[1]) as i64;
        let dz = (sample[2] - c[2]) as i64;
        let d = dx * dx + dy * dy + dz * dz;
        if d < best_dist {
            best_dist = d;
            best = idx;
        }
    }
    best
}

/// sRGB(0..255) → CIE L\*a\*b\*。使用 D65 白点、sRGB γ=2.4 反伽马。
fn rgb_to_lab(rgb: [f32; 3]) -> [f32; 3] {
    fn srgb_to_linear(v: f32) -> f32 {
        let n = (v / 255.0).clamp(0.0, 1.0);
        if n <= 0.04045 {
            n / 12.92
        } else {
            ((n + 0.055) / 1.055).powf(2.4)
        }
    }
    let r = srgb_to_linear(rgb[0]);
    let g = srgb_to_linear(rgb[1]);
    let b = srgb_to_linear(rgb[2]);

    // sRGB → XYZ (D65)，矩阵来自 IEC 61966-2-1 / Lindbloom。
    let x = 0.4124564 * r + 0.3575761 * g + 0.1804375 * b;
    let y = 0.2126729 * r + 0.7151522 * g + 0.0721750 * b;
    let z = 0.0193339 * r + 0.1191920 * g + 0.9503041 * b;

    // 归一化到 D65 参考白。
    let xn = x / 0.95047;
    let yn = y;
    let zn = z / 1.08883;

    fn f(t: f32) -> f32 {
        const DELTA: f32 = 6.0 / 29.0;
        if t > DELTA * DELTA * DELTA {
            t.cbrt()
        } else {
            t / (3.0 * DELTA * DELTA) + 4.0 / 29.0
        }
    }
    let fx = f(xn);
    let fy = f(yn);
    let fz = f(zn);

    let l = 116.0 * fy - 16.0;
    let a = 500.0 * (fx - fy);
    let b = 200.0 * (fy - fz);
    [l, a, b]
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

        let face = ColorClusterRecognizer::default().recognize(&frame, &rois).unwrap();

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
        assert_eq!(original_roi_label(49).unwrap(), "U5");
        assert_eq!(original_roi_label(40).unwrap(), "R5");
        assert_eq!(original_roi_label(31).unwrap(), "F5");
        assert_eq!(original_roi_label(22).unwrap(), "D5");
        assert_eq!(original_roi_label(4).unwrap(), "L5");
        assert_eq!(original_roi_label(13).unwrap(), "B5");
        assert_eq!(center_original_roi_indices(), [49, 40, 31, 22, 4, 13]);
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

        let details =
            recognize_original_roi_details(&frame, &rois, ColorClassifierKind::Rgb).unwrap();

        assert_eq!(details.facelets, SOLVED_FACE);
        assert_eq!(details.classes[49], 0);
        assert_eq!(details.labels[49], "U");
        assert_eq!(details.center_colors[0], [240.0, 240.0, 240.0]);
        assert_eq!(details.sample_colors[49], [240.0, 240.0, 240.0]);
    }

    #[test]
    fn samples_diagnostic_details_when_color_counts_are_invalid() {
        let mut colors = original_roi_order_solved_colors();
        colors[0] = colors[49];
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

        let details =
            recognize_original_roi_diagnostic_details(&frame, &rois, ColorClassifierKind::Rgb)
                .unwrap();

        assert_eq!(details.classes[0], 0);
        assert_eq!(color_class_counts(&details.classes), [10, 9, 9, 9, 8, 9]);
        assert!(validate_color_class_counts(&details.classes).is_err());
        assert!(recognize_original_roi_details(&frame, &rois, ColorClassifierKind::Rgb).is_err());
    }

    #[test]
    fn cpp_classifier_recovers_solved_cube() {
        let colors_u8 = original_roi_order_solved_colors();
        let colors: Vec<[f32; 3]> = colors_u8
            .iter()
            .map(|c| [c[0] as f32, c[1] as f32, c[2] as f32])
            .collect();
        let classes = classify_by_centers_cpp(&colors).unwrap();
        assert_eq!(color_class_counts(&classes), [9; 6]);
        assert_eq!(facelets_from_original_roi_classes(&classes).unwrap(), SOLVED_FACE);
    }

    #[test]
    fn lab_classifier_recovers_solved_cube() {
        let colors_u8 = original_roi_order_solved_colors();
        let colors: Vec<[f32; 3]> = colors_u8
            .iter()
            .map(|c| [c[0] as f32, c[1] as f32, c[2] as f32])
            .collect();
        let classes = classify_by_centers_lab(&colors).unwrap();
        assert_eq!(color_class_counts(&classes), [9; 6]);
        assert_eq!(facelets_from_original_roi_classes(&classes).unwrap(), SOLVED_FACE);
    }

    #[test]
    fn dispatch_kind_routes_to_specific_classifier() {
        let colors_u8 = original_roi_order_solved_colors();
        let colors: Vec<[f32; 3]> = colors_u8
            .iter()
            .map(|c| [c[0] as f32, c[1] as f32, c[2] as f32])
            .collect();
        for kind in [
            ColorClassifierKind::Rgb,
            ColorClassifierKind::Cpp,
            ColorClassifierKind::Lab,
        ] {
            let classes = classify_by_centers_unvalidated_with_kind(&colors, kind).unwrap();
            assert_eq!(color_class_counts(&classes), [9; 6], "kind={:?}", kind);
        }
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
            b'L', b'L', b'L', b'L', b'L', b'L', b'L', b'L', b'L', b'B', b'B', b'B', b'B', b'B',
            b'B', b'B', b'B', b'B', b'D', b'D', b'D', b'D', b'D', b'D', b'D', b'D', b'D', b'F',
            b'F', b'F', b'F', b'F', b'F', b'F', b'F', b'F', b'R', b'R', b'R', b'R', b'R', b'R',
            b'R', b'R', b'R', b'U', b'U', b'U', b'U', b'U', b'U', b'U', b'U', b'U',
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
            4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2,
            2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ]
    }
}
