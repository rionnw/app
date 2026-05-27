//! recognize-cli: 读取图片和 ROI 文件，识别魔方色块，将结果绘制在图片上另存。
//!
//! Usage:
//!   recognize-cli <image> <roi.json> [output.jpg]

use anyhow::{Context, Result};
use image::{Rgb, RgbImage};
use robo_core::{Frame, Recognizer, Roi};
use robo_vision::ColorClusterRecognizer;
use serde::Deserialize;
use std::path::Path;

#[derive(Deserialize)]
struct RoiFile {
    rois: Vec<RoiEntry>,
}

#[derive(Deserialize)]
struct RoiEntry {
    x: u32,
    y: u32,
    width: u32,
    height: u32,
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: recognize-cli <image> <roi.json> [output.jpg]");
        eprintln!();
        eprintln!("读取图片和 ROI 坐标文件，识别魔方 54 个色块，");
        eprintln!("在原图上标注识别结果并另存为新图片。");
        std::process::exit(1);
    }

    let image_path = &args[1];
    let roi_path = &args[2];
    let output_path = if args.len() > 3 {
        args[3].clone()
    } else {
        let stem = Path::new(image_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("output");
        format!("{stem}_recognized.jpg")
    };

    // 加载图片
    let img = image::open(image_path).context("无法打开图片")?;
    let rgb_img = img.to_rgb8();
    let (width, height) = rgb_img.dimensions();
    let frame = Frame::new_rgb(width, height, rgb_img.clone().into_raw())
        .context("构建 Frame 失败")?;

    // 加载 ROI
    let roi_content = std::fs::read_to_string(roi_path).context("无法读取 ROI 文件")?;
    let roi_file: RoiFile = serde_json::from_str(&roi_content).context("ROI JSON 解析失败")?;
    let rois: Vec<Roi> = roi_file
        .rois
        .iter()
        .map(|r| Roi {
            x: r.x,
            y: r.y,
            width: r.width,
            height: r.height,
        })
        .collect();

    if rois.len() != 54 {
        anyhow::bail!("ROI 数量必须为 54，当前 {}", rois.len());
    }

    // 识别
    let recognizer = ColorClusterRecognizer;
    let face = recognizer.recognize(&frame, &rois)?;
    let facelets = face.as_str();
    println!("识别结果: {facelets}");

    // 绘制结果在图片上
    let mut output_img = rgb_img;
    let face_colors: Vec<Rgb<u8>> = "URFDLB"
        .chars()
        .map(|c| match c {
            'U' => Rgb([255, 255, 255]), // 白
            'R' => Rgb([255, 0, 0]),     // 红
            'F' => Rgb([0, 200, 0]),     // 绿
            'D' => Rgb([255, 255, 0]),   // 黄
            'L' => Rgb([255, 128, 0]),   // 橙
            'B' => Rgb([0, 0, 255]),     // 蓝
            _ => Rgb([128, 128, 128]),
        })
        .collect();

    for (i, roi) in rois.iter().enumerate() {
        let ch = facelets.as_bytes()[i];
        let color_idx = match ch {
            b'U' => 0,
            b'R' => 1,
            b'F' => 2,
            b'D' => 3,
            b'L' => 4,
            b'B' => 5,
            _ => 0,
        };
        let color = face_colors[color_idx];
        draw_rect(&mut output_img, roi.x, roi.y, roi.width, roi.height, color);
    }

    if let Some(parent) = Path::new(&output_path).parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).ok();
        }
    }
    output_img.save(&output_path).context("保存图片失败")?;
    println!("结果图片已保存: {output_path}");
    Ok(())
}

fn draw_rect(img: &mut RgbImage, x: u32, y: u32, w: u32, h: u32, color: Rgb<u8>) {
    let (iw, ih) = img.dimensions();
    // 画 3 像素粗边框
    for t in 0..3u32 {
        for dx in 0..w {
            let px = x + dx;
            if px < iw {
                if y + t < ih {
                    img.put_pixel(px, y + t, color);
                }
                if y + h >= 1 + t && y + h - 1 - t < ih {
                    img.put_pixel(px, y + h - 1 - t, color);
                }
            }
        }
        for dy in 0..h {
            let py = y + dy;
            if py < ih {
                if x + t < iw {
                    img.put_pixel(x + t, py, color);
                }
                if x + w >= 1 + t && x + w - 1 - t < iw {
                    img.put_pixel(x + w - 1 - t, py, color);
                }
            }
        }
    }
}
