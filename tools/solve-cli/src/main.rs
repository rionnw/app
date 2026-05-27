//! solve-cli: 求解魔方，支持 facelets 字符串或图片+ROI 输入。
//!
//! Usage:
//!   solve-cli --facelets <54-char string>
//!   solve-cli --image <image> --roi <roi.json>

use anyhow::{Context, Result};
use robo_core::{CubeFace, Frame, Recognizer, Roi, Solver, Translator};
use robo_solver::Min2PhaseSolver;
use robo_translator::BasicTranslator;
use robo_vision::ColorClusterRecognizer;
use serde::Deserialize;
use std::time::Instant;

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

    if args.len() < 2 {
        print_usage();
        std::process::exit(1);
    }

    let face = match args[1].as_str() {
        "--facelets" | "-f" => {
            let facelets = args.get(2).context("缺少 facelets 参数")?;
            CubeFace::new(facelets.clone()).map_err(|e| anyhow::anyhow!("{e}"))?
        }
        "--image" | "-i" => {
            let image_path = args.get(2).context("缺少图片路径")?;
            let roi_idx = args.iter().position(|a| a == "--roi" || a == "-r")
                .context("使用 --image 时需要 --roi 参数")?;
            let roi_path = args.get(roi_idx + 1).context("缺少 ROI 文件路径")?;
            recognize_from_file(image_path, roi_path)?
        }
        _ => {
            // 尝试作为 facelets 直接解析
            if args[1].len() == 54 {
                CubeFace::new(args[1].clone()).map_err(|e| anyhow::anyhow!("{e}"))?
            } else {
                print_usage();
                std::process::exit(1);
            }
        }
    };

    println!("Facelets: {}", face.as_str());

    // 求解
    eprintln!("正在初始化 solver...");
    let t0 = Instant::now();
    let solver = Min2PhaseSolver::new();
    eprintln!("Solver 初始化完成 ({:.2}s)", t0.elapsed().as_secs_f64());

    let t1 = Instant::now();
    let moves = solver.solve(&face)?;
    let solve_ms = t1.elapsed().as_millis();
    println!("Solution: {}", moves.to_solution_string());
    println!("Solve time: {solve_ms} ms");

    // 翻译为硬件指令
    let translator = BasicTranslator::new();
    let steps = translator.translate(&moves)?;
    println!("\nHardware commands ({} ops):", steps.commands.len());
    for cmd in &steps.commands {
        println!("  {cmd}");
    }
    println!("\nEncoded: {}", steps.encoded);

    Ok(())
}

fn recognize_from_file(image_path: &str, roi_path: &str) -> Result<CubeFace> {
    let img = image::open(image_path).context("无法打开图片")?;
    let rgb = img.to_rgb8();
    let (w, h) = rgb.dimensions();
    let frame = Frame::new_rgb(w, h, rgb.into_raw())?;

    let roi_content = std::fs::read_to_string(roi_path).context("无法读取 ROI 文件")?;
    let roi_file: RoiFile = serde_json::from_str(&roi_content)?;
    let rois: Vec<Roi> = roi_file.rois.iter().map(|r| Roi {
        x: r.x, y: r.y, width: r.width, height: r.height,
    }).collect();

    let recognizer = ColorClusterRecognizer;
    recognizer.recognize(&frame, &rois).context("识别失败")
}

fn print_usage() {
    eprintln!("CubeSolver CLI - 魔方求解工具");
    eprintln!();
    eprintln!("Usage:");
    eprintln!("  solve-cli --facelets <54-char facelets string>");
    eprintln!("  solve-cli --image <image.jpg> --roi <roi.json>");
    eprintln!("  solve-cli <54-char facelets string>");
    eprintln!();
    eprintln!("Examples:");
    eprintln!("  solve-cli UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB");
    eprintln!("  solve-cli -i example/im.jpg -r example/robot-roi.json");
}
