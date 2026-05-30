//! 相机/编码 micro-benchmark
//!
//! 拆开测量每个阶段的真实耗时，用于定位 grid 流的瓶颈。
//! 用法：
//!   cargo run --release -p camera-bench -- --slots 4 --frames 60
//!   cargo run --release -p camera-bench -- --slots 1 --frames 120 --width 640 --height 480
//!
//! 输出每路 / 每阶段的 P50/P95/P99 耗时（ms）。

use anyhow::{Context, Result};
use image::{codecs::jpeg::JpegEncoder, imageops, ExtendedColorType, RgbImage};
use nokhwa::{
    pixel_format::RgbFormat,
    utils::{
        ApiBackend, CameraFormat, CameraIndex, FrameFormat, RequestedFormat, RequestedFormatType,
        Resolution,
    },
    Camera,
};
use std::{
    env,
    io::Cursor,
    time::{Duration, Instant},
};

#[derive(Default, Clone)]
struct Stats {
    samples: Vec<f64>,
}

impl Stats {
    fn push(&mut self, ms: f64) {
        self.samples.push(ms);
    }
    fn percentile(&self, p: f64) -> f64 {
        if self.samples.is_empty() {
            return 0.0;
        }
        let mut s = self.samples.clone();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let idx = ((s.len() as f64 - 1.0) * p).round() as usize;
        s[idx]
    }
    fn mean(&self) -> f64 {
        if self.samples.is_empty() {
            return 0.0;
        }
        self.samples.iter().sum::<f64>() / self.samples.len() as f64
    }
    fn report(&self, name: &str) {
        if self.samples.is_empty() {
            println!("  {name:30} (no samples)");
            return;
        }
        println!(
            "  {name:30} n={:4} mean={:5.1}ms  p50={:5.1}  p95={:5.1}  p99={:5.1}  max={:5.1}",
            self.samples.len(),
            self.mean(),
            self.percentile(0.50),
            self.percentile(0.95),
            self.percentile(0.99),
            self.samples.iter().cloned().fold(0.0f64, f64::max),
        );
    }
}

struct Args {
    slots: u32,
    frames: u32,
    width: u32,
    height: u32,
    fps: u32,
    format: FrameFormat,
    list: bool,
    probe: bool,
}

impl Args {
    fn parse() -> Self {
        let mut slots = 4u32;
        let mut frames = 60u32;
        let mut width = 640u32;
        let mut height = 480u32;
        let mut fps = 30u32;
        let mut format = FrameFormat::MJPEG;
        let mut list = false;
        let mut probe = false;

        let argv: Vec<String> = env::args().collect();
        let mut i = 1;
        while i < argv.len() {
            match argv[i].as_str() {
                "--slots" => {
                    slots = argv[i + 1].parse().expect("--slots N");
                    i += 2;
                }
                "--frames" => {
                    frames = argv[i + 1].parse().expect("--frames N");
                    i += 2;
                }
                "--width" => {
                    width = argv[i + 1].parse().expect("--width N");
                    i += 2;
                }
                "--height" => {
                    height = argv[i + 1].parse().expect("--height N");
                    i += 2;
                }
                "--fps" => {
                    fps = argv[i + 1].parse().expect("--fps N");
                    i += 2;
                }
                "--format" => {
                    format = match argv[i + 1].to_uppercase().as_str() {
                        "MJPEG" | "MJPG" => FrameFormat::MJPEG,
                        "YUYV" => FrameFormat::YUYV,
                        other => panic!("unsupported format {other}"),
                    };
                    i += 2;
                }
                "--list" => {
                    list = true;
                    i += 1;
                }
                "--probe" => {
                    probe = true;
                    i += 1;
                }
                other => {
                    eprintln!("unknown arg: {other}");
                    i += 1;
                }
            }
        }
        Self {
            slots,
            frames,
            width,
            height,
            fps,
            format,
            list,
            probe,
        }
    }
}

/// 列出每个相机支持的所有 (resolution, fps, frame_format) 组合，
/// 方便用户挑一个真实可用的参数再跑 bench。
fn list_all_cameras() -> Result<()> {
    let devices = nokhwa::query(ApiBackend::Auto).unwrap_or_default();
    if devices.is_empty() {
        println!("(no cameras found)");
        return Ok(());
    }
    for d in &devices {
        println!("\n# camera #{} {}", d.index(), d.human_name());
        // 用 None 打开，nokhwa 会用设备默认格式，避免占用太久
        let requested = RequestedFormat::new::<RgbFormat>(RequestedFormatType::None);
        let mut cam = match Camera::new(d.index().clone(), requested) {
            Ok(c) => c,
            Err(err) => {
                println!("  (failed to open: {err})");
                continue;
            }
        };
        let formats = cam.compatible_camera_formats().unwrap_or_default();
        if formats.is_empty() {
            println!("  (no compatible formats reported)");
            continue;
        }
        for f in formats {
            println!(
                "  {}x{} @ {}fps {:?}",
                f.resolution().width(),
                f.resolution().height(),
                f.frame_rate(),
                f.format()
            );
        }
    }
    Ok(())
}

/// 实测在不同 RequestedFormat 策略下，相机一秒能拿到多少帧。
/// 用于绕过 nokhwa Closest 对 MSMF 后端协商不准的问题。
fn probe_fps(args: &Args) -> Result<()> {
    let res = Resolution::new(args.width, args.height);
    println!("[probe] camera #0 sweep at {}x{}", args.width, args.height);

    let mut strategies: Vec<(String, RequestedFormatType)> = vec![
        (
            // 关键策略：在指定分辨率下让 driver 给最高 fps，不手动指定 fps。
            // 这正对应 RobotApp 的 C++ 写法（set FOURCC=MJPG + WIDTH/HEIGHT，
            // 不 set FPS，由 driver 自动给最高）。
            format!("HighestResolution({}x{})", args.width, args.height),
            RequestedFormatType::HighestResolution(res),
        ),
        (
            "AbsoluteHighestFrameRate".to_string(),
            RequestedFormatType::AbsoluteHighestFrameRate,
        ),
        (
            "AbsoluteHighestResolution".to_string(),
            RequestedFormatType::AbsoluteHighestResolution,
        ),
        (
            "HighestFrameRate(30)".to_string(),
            RequestedFormatType::HighestFrameRate(30),
        ),
    ];

    // 对所有可能的 FrameFormat 都尝试一次 Closest@30，看 nokhwa MSMF 后端
    // 哪些格式实际能拿到帧。Logi C270 在 MSMF 下只暴露 NV12 / YUYV，没有 MJPEG，
    // 这里要把 NV12 / YUYV 也测出来。
    for fmt in [
        FrameFormat::MJPEG,
        FrameFormat::YUYV,
        FrameFormat::NV12,
        FrameFormat::GRAY,
        FrameFormat::RAWRGB,
    ] {
        strategies.push((
            format!("Closest({:?}@30)", fmt),
            RequestedFormatType::Closest(CameraFormat::new(res, fmt, 30)),
        ));
    }

    for (name, strategy) in strategies {
        let requested = RequestedFormat::new::<RgbFormat>(strategy);
        let mut cam = match Camera::new(CameraIndex::Index(0), requested) {
            Ok(c) => c,
            Err(err) => {
                println!("  {:<32}  open FAILED: {err}", name);
                continue;
            }
        };
        if let Err(err) = cam.open_stream() {
            println!("  {:<32}  stream FAILED: {err}", name);
            continue;
        }
        let neg_fps = cam.frame_rate();
        let neg_res = cam.resolution();
        let neg_fmt = cam.frame_format();
        // warm up 3 frames
        for _ in 0..3 {
            let _ = cam.frame();
        }
        // 测 1 秒实际取帧数
        let start = Instant::now();
        let mut count = 0u32;
        while start.elapsed() < Duration::from_secs(1) {
            if cam.frame().is_ok() {
                count += 1;
            }
        }
        println!(
            "  {:<32}  negotiated={}x{}@{}fps {:?}  measured={:.1} fps",
            name,
            neg_res.width(),
            neg_res.height(),
            neg_fps,
            neg_fmt,
            count as f64
        );
        drop(cam);
        std::thread::sleep(Duration::from_millis(200));
    }
    Ok(())
}

fn open_camera(idx: u32, args: &Args) -> Result<Camera> {
    // Closest 比 Exact 更稳：相机若不支持精确的 (w,h,fmt,fps) 组合，
    // Exact 会 fallback 到设备返回的第一个格式（常见是 1fps），导致 frame()
    // 像被卡住一样；Closest 会找最接近的可用格式（比如 30fps 不行就 25fps）。
    let format = CameraFormat::new(Resolution::new(args.width, args.height), args.format, args.fps);
    let requested = RequestedFormat::new::<RgbFormat>(RequestedFormatType::Closest(format));
    let mut camera = Camera::new(CameraIndex::Index(idx), requested)
        .with_context(|| format!("failed to open camera #{idx}"))?;
    let res = camera.resolution();
    let fps = camera.frame_rate();
    let fmt = camera.frame_format();
    if fps != args.fps || res.width() != args.width || res.height() != args.height || fmt != args.format {
        eprintln!(
            "  [warn] camera #{idx} negotiated to {}x{} @ {}fps fmt={:?} (requested {}x{} @ {}fps {:?})",
            res.width(), res.height(), fps, fmt, args.width, args.height, args.fps, args.format
        );
    }
    camera
        .open_stream()
        .with_context(|| format!("failed to start stream on camera #{idx}"))?;
    Ok(camera)
}

fn encode_jpeg_quality(rgb: &[u8], w: u32, h: u32, quality: u8) -> Result<Vec<u8>> {
    let mut bytes = Cursor::new(Vec::with_capacity(rgb.len() / 8));
    JpegEncoder::new_with_quality(&mut bytes, quality)
        .encode(rgb, w, h, ExtendedColorType::Rgb8)
        .context("encode failed")?;
    Ok(bytes.into_inner())
}

fn encode_jpeg_via_rgbimage(rgb: &[u8], w: u32, h: u32, quality: u8) -> Result<Vec<u8>> {
    // 旧路径：先 RgbImage::from_raw（要求拥有 Vec），再 encode_image。
    let image = RgbImage::from_raw(w, h, rgb.to_vec()).context("from_raw")?;
    let mut bytes = Cursor::new(Vec::new());
    JpegEncoder::new_with_quality(&mut bytes, quality)
        .encode_image(&image)
        .context("encode_image")?;
    Ok(bytes.into_inner())
}

fn main() -> Result<()> {
    let args = Args::parse();

    if args.list {
        return list_all_cameras();
    }
    if args.probe {
        return probe_fps(&args);
    }

    println!(
        "[bench] slots={} frames={} {}x{}@{}fps fmt={:?}",
        args.slots, args.frames, args.width, args.height, args.fps, args.format
    );

    println!("[bench] enumerating cameras...");
    let devices = nokhwa::query(ApiBackend::Auto).unwrap_or_default();
    for d in &devices {
        println!("  - #{} {}", d.index(), d.human_name());
    }

    println!("[bench] opening {} camera(s)...", args.slots);
    let mut cams: Vec<Camera> = Vec::new();
    for i in 0..args.slots {
        let cam = open_camera(i, &args)?;
        let res = cam.resolution();
        println!(
            "  slot {i} opened: {}x{} @ {}fps fmt={:?}",
            res.width(),
            res.height(),
            cam.frame_rate(),
            cam.frame_format()
        );
        cams.push(cam);
    }

    // ── 阶段 1：每路单独 capture (frame() 阻塞 + decode_image)
    println!("\n[stage 1] per-slot serial capture (nokhwa frame() + decode RGB)");
    let mut frame_block = vec![Stats::default(); cams.len()];
    let mut decode = vec![Stats::default(); cams.len()];
    let mut last_rgb: Vec<Vec<u8>> = vec![Vec::new(); cams.len()];

    let mut warmup = 5u32;
    let mut taken = 0u32;
    while taken < args.frames {
        for (slot, cam) in cams.iter_mut().enumerate() {
            let t0 = Instant::now();
            let buf = cam.frame().context("frame()")?;
            let t1 = Instant::now();
            let img = buf.decode_image::<RgbFormat>().context("decode_image")?;
            let t2 = Instant::now();

            if warmup > 0 {
                continue;
            }
            frame_block[slot].push((t1 - t0).as_secs_f64() * 1e3);
            decode[slot].push((t2 - t1).as_secs_f64() * 1e3);
            last_rgb[slot] = img.into_raw();
        }
        if warmup > 0 {
            warmup -= 1;
        } else {
            taken += 1;
        }
    }

    for (slot, s) in frame_block.iter().enumerate() {
        s.report(&format!("slot{slot} frame()"));
    }
    for (slot, s) in decode.iter().enumerate() {
        s.report(&format!("slot{slot} decode_image"));
    }

    // 单路实际可达 fps（帧间隔 = frame() + decode）
    for (slot, _) in cams.iter().enumerate() {
        let total = frame_block[slot].mean() + decode[slot].mean();
        if total > 0.0 {
            println!(
                "  slot{slot} effective {:.1} fps (frame+decode = {:.1} ms)",
                1000.0 / total,
                total
            );
        }
    }

    // ── 阶段 2：JPEG 编码（直接 buffer vs RgbImage 中转）
    println!("\n[stage 2] JPEG encode of single tile (640x480 RGB)");
    let tile_w = args.width;
    let tile_h = args.height;
    let tile_rgb = if last_rgb[0].is_empty() {
        vec![128u8; (tile_w * tile_h * 3) as usize]
    } else {
        last_rgb[0].clone()
    };

    let mut q60_direct = Stats::default();
    let mut q72_direct = Stats::default();
    let mut q60_via_image = Stats::default();
    let mut q72_via_image = Stats::default();
    for _ in 0..50 {
        let t = Instant::now();
        let _ = encode_jpeg_quality(&tile_rgb, tile_w, tile_h, 60)?;
        q60_direct.push(t.elapsed().as_secs_f64() * 1e3);

        let t = Instant::now();
        let _ = encode_jpeg_quality(&tile_rgb, tile_w, tile_h, 72)?;
        q72_direct.push(t.elapsed().as_secs_f64() * 1e3);

        let t = Instant::now();
        let _ = encode_jpeg_via_rgbimage(&tile_rgb, tile_w, tile_h, 60)?;
        q60_via_image.push(t.elapsed().as_secs_f64() * 1e3);

        let t = Instant::now();
        let _ = encode_jpeg_via_rgbimage(&tile_rgb, tile_w, tile_h, 72)?;
        q72_via_image.push(t.elapsed().as_secs_f64() * 1e3);
    }
    q60_direct.report("tile JPEG q=60 (direct)");
    q72_direct.report("tile JPEG q=72 (direct)");
    q60_via_image.report("tile JPEG q=60 (via RgbImage)");
    q72_via_image.report("tile JPEG q=72 (via RgbImage)");

    // ── 阶段 3：grid（1280x960）JPEG 编码
    println!("\n[stage 3] JPEG encode of grid (1280x960 RGB)");
    let grid_w = tile_w * 2;
    let grid_h = tile_h * 2;
    let grid_rgb = vec![100u8; (grid_w * grid_h * 3) as usize];
    let mut grid_q60 = Stats::default();
    let mut grid_q72 = Stats::default();
    for _ in 0..50 {
        let t = Instant::now();
        let _ = encode_jpeg_quality(&grid_rgb, grid_w, grid_h, 60)?;
        grid_q60.push(t.elapsed().as_secs_f64() * 1e3);

        let t = Instant::now();
        let _ = encode_jpeg_quality(&grid_rgb, grid_w, grid_h, 72)?;
        grid_q72.push(t.elapsed().as_secs_f64() * 1e3);
    }
    grid_q60.report("grid JPEG q=60");
    grid_q72.report("grid JPEG q=72");

    // ── 阶段 4：grid blit（4 块 RGB 直接拷到 grid buffer）
    println!("\n[stage 4] grid blit (4 tiles of {tile_w}x{tile_h} -> {grid_w}x{grid_h})");
    let mut blit = Stats::default();
    let mut output = vec![0u8; (grid_w * grid_h * 3) as usize];
    for _ in 0..50 {
        let t = Instant::now();
        for (idx, src) in last_rgb.iter().take(4).enumerate() {
            let src_use = if src.is_empty() { &tile_rgb } else { src };
            let dst_x = (idx as u32 % 2) * tile_w;
            let dst_y = (idx as u32 / 2) * tile_h;
            // 假设 src 已经是 tile_w x tile_h
            for y in 0..tile_h {
                let src_off = (y * tile_w * 3) as usize;
                let dst_off = (((dst_y + y) * grid_w + dst_x) * 3) as usize;
                let n = (tile_w * 3) as usize;
                if src_off + n <= src_use.len() && dst_off + n <= output.len() {
                    output[dst_off..dst_off + n].copy_from_slice(&src_use[src_off..src_off + n]);
                }
            }
        }
        blit.push(t.elapsed().as_secs_f64() * 1e3);
    }
    blit.report("grid blit (4 tiles)");

    // ── 阶段 5：image::imageops::resize（如果源尺寸 != tile）
    println!("\n[stage 5] imageops::resize({tile_w}x{tile_h} -> {tile_w}x{tile_h}) (no-op cost)");
    let mut resize_same = Stats::default();
    if let Some(img) = RgbImage::from_raw(tile_w, tile_h, tile_rgb.clone()) {
        for _ in 0..30 {
            let t = Instant::now();
            let _r = imageops::resize(&img, tile_w, tile_h, imageops::FilterType::Triangle);
            resize_same.push(t.elapsed().as_secs_f64() * 1e3);
        }
        resize_same.report("resize same-size Triangle");

        // 不同尺寸（缩小一倍）
        let mut resize_down = Stats::default();
        for _ in 0..30 {
            let t = Instant::now();
            let _r = imageops::resize(&img, tile_w / 2, tile_h / 2, imageops::FilterType::Triangle);
            resize_down.push(t.elapsed().as_secs_f64() * 1e3);
        }
        resize_down.report("resize 1/4 area Triangle");

        let mut resize_nn = Stats::default();
        for _ in 0..30 {
            let t = Instant::now();
            let _r = imageops::resize(&img, tile_w / 2, tile_h / 2, imageops::FilterType::Nearest);
            resize_nn.push(t.elapsed().as_secs_f64() * 1e3);
        }
        resize_nn.report("resize 1/4 area Nearest");
    }

    // 关闭相机
    drop(cams);
    let _ = robo_camera::TILE_WIDTH; // 引用一下，避免 unused crate
    let _ = robo_core::Frame::new_rgb(1, 1, vec![0u8; 3]);
    std::thread::sleep(Duration::from_millis(50));
    Ok(())
}
