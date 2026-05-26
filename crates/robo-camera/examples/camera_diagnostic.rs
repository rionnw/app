use anyhow::{Context, Result};
use image::codecs::jpeg::JpegEncoder;
use nokhwa::{
    pixel_format::RgbFormat,
    utils::{
        CameraFormat, CameraIndex, FrameFormat, RequestedFormat, RequestedFormatType, Resolution,
    },
    Camera,
};
use robo_camera::{frame_format_from_str, list_cameras};
use std::{
    env,
    io::Cursor,
    time::{Duration, Instant},
};

const DEFAULT_FRAMES: usize = 60;
const WARMUP_FRAMES: usize = 5;
const JPEG_QUALITY: u8 = 72;

fn main() -> Result<()> {
    let frame_count = arg_usize("--frames").unwrap_or(DEFAULT_FRAMES);
    let requested_index = arg_u32("--index");
    let requested_format = arg_camera_format()?;

    println!("Backend real camera diagnostic");
    println!("frames={frame_count} warmup={WARMUP_FRAMES} jpeg_quality={JPEG_QUALITY}");
    println!(
        "ROBO_UI_MOCK_CAMERA={}",
        env::var("ROBO_UI_MOCK_CAMERA").unwrap_or_else(|_| "<unset>".to_string())
    );

    let cameras = list_cameras().context("failed to list cameras")?;
    if cameras.is_empty() {
        anyhow::bail!("no cameras discovered");
    }

    println!("\nCameras:");
    for camera in &cameras {
        println!(
            "- index={} name={} description={}",
            camera.index, camera.name, camera.description
        );
    }

    let index = requested_index
        .or_else(|| {
            cameras
                .iter()
                .find_map(|camera| camera.index.parse::<u32>().ok())
        })
        .context("no numeric camera index found; pass --index <n>")?;

    let formats = load_formats(index)
        .with_context(|| format!("failed to list formats for camera {index}"))?;
    println!("\nFormats for camera {index}:");
    for format in &formats {
        println!(
            "- {}x{} {}fps {}",
            format.width(),
            format.height(),
            format.frame_rate(),
            format.format()
        );
    }

    let selected = requested_format
        .or_else(|| select_format(&formats))
        .context("no usable camera format found")?;
    println!(
        "\nSelected: {}x{} {}fps {}",
        selected.width(),
        selected.height(),
        selected.frame_rate(),
        selected.format()
    );

    let requested = RequestedFormat::new::<RgbFormat>(RequestedFormatType::Exact(selected));
    let mut camera = Camera::new(CameraIndex::Index(index), requested)
        .with_context(|| format!("failed to open camera {index}"))?;

    let open_started = Instant::now();
    camera
        .open_stream()
        .context("failed to open camera stream")?;
    println!("open_stream_ms={:.2}", ms(open_started.elapsed()));

    for _ in 0..WARMUP_FRAMES {
        let _ = camera.frame().context("failed during warmup capture")?;
    }

    let mut raw_capture = Vec::with_capacity(frame_count);
    let mut decode = Vec::with_capacity(frame_count);
    let mut encode = Vec::with_capacity(frame_count);
    let mut total = Vec::with_capacity(frame_count);
    let mut raw_bytes = Vec::with_capacity(frame_count);
    let mut jpeg_bytes = Vec::with_capacity(frame_count);
    let mut observed_format = String::new();
    let run_started = Instant::now();

    for _ in 0..frame_count {
        let frame_started = Instant::now();

        let capture_started = Instant::now();
        let buffer = camera.frame().context("failed to capture frame")?;
        raw_capture.push(capture_started.elapsed());
        observed_format = buffer.source_frame_format().to_string();
        raw_bytes.push(buffer.buffer().len() as f64);

        let decode_started = Instant::now();
        let image = buffer
            .decode_image::<RgbFormat>()
            .context("failed to decode frame as RGB")?;
        decode.push(decode_started.elapsed());

        let encode_started = Instant::now();
        let mut bytes = Cursor::new(Vec::new());
        JpegEncoder::new_with_quality(&mut bytes, JPEG_QUALITY)
            .encode_image(&image)
            .context("failed to encode decoded frame as JPEG")?;
        jpeg_bytes.push(bytes.get_ref().len() as f64);
        encode.push(encode_started.elapsed());

        total.push(frame_started.elapsed());
    }

    let elapsed = run_started.elapsed();
    let observed_fps = frame_count as f64 / elapsed.as_secs_f64();

    println!("\nObserved source format: {observed_format}");
    print_duration_stats("raw_capture_ms", &raw_capture);
    print_duration_stats("rgb_decode_ms", &decode);
    print_duration_stats("jpeg_encode_ms", &encode);
    print_duration_stats("total_frame_ms", &total);
    print_f64_stats("raw_bytes", &raw_bytes);
    print_f64_stats("jpeg_bytes", &jpeg_bytes);
    println!("observed_fps={observed_fps:.2}");

    Ok(())
}

fn load_formats(index: u32) -> Result<Vec<CameraFormat>> {
    let mut camera = open_for_format_query(index)
        .with_context(|| format!("failed to open camera {index} for format query"))?;
    let mut formats = camera
        .compatible_camera_formats()
        .with_context(|| format!("failed to query compatible formats for camera {index}"))?;
    if formats.is_empty() {
        let active_format = camera
            .refresh_camera_format()
            .unwrap_or_else(|_| camera.camera_format());
        println!("compatible_camera_formats returned no rows; using active format");
        formats.push(active_format);
    }
    formats.sort_by(|a, b| {
        (a.width(), a.height(), a.frame_rate(), a.format()).cmp(&(
            b.width(),
            b.height(),
            b.frame_rate(),
            b.format(),
        ))
    });
    formats.dedup_by(|a, b| {
        a.width() == b.width()
            && a.height() == b.height()
            && a.frame_rate() == b.frame_rate()
            && a.format() == b.format()
    });
    Ok(formats)
}

fn select_format(formats: &[CameraFormat]) -> Option<CameraFormat> {
    for format in [
        practical_format(FrameFormat::MJPEG),
        practical_format(FrameFormat::NV12),
        practical_format(FrameFormat::YUYV),
        practical_format(FrameFormat::RAWRGB),
        practical_format(FrameFormat::RAWBGR),
    ] {
        if formats.contains(&format) {
            return Some(format);
        }
    }

    formats
        .iter()
        .copied()
        .find(|format| format.width() == 640 && format.height() == 480)
        .or_else(|| {
            formats
                .iter()
                .copied()
                .find(|format| format.format() == FrameFormat::MJPEG)
        })
        .or_else(|| formats.first().copied())
}

fn open_for_format_query(index: u32) -> Result<Camera> {
    let requests = [
        RequestedFormatType::Closest(practical_format(FrameFormat::MJPEG)),
        RequestedFormatType::Closest(practical_format(FrameFormat::NV12)),
        RequestedFormatType::Closest(practical_format(FrameFormat::YUYV)),
        RequestedFormatType::HighestFrameRate(30),
        RequestedFormatType::AbsoluteHighestFrameRate,
        RequestedFormatType::AbsoluteHighestResolution,
        RequestedFormatType::None,
    ];

    let mut errors = Vec::new();
    for request in requests {
        match Camera::new(
            CameraIndex::Index(index),
            RequestedFormat::new::<RgbFormat>(request),
        ) {
            Ok(camera) => return Ok(camera),
            Err(err) => errors.push(format!("{request}: {err}")),
        }
    }

    anyhow::bail!("{}", errors.join("; "))
}

fn practical_format(frame_format: FrameFormat) -> CameraFormat {
    CameraFormat::new(Resolution::new(640, 480), frame_format, 30)
}

fn arg_camera_format() -> Result<Option<CameraFormat>> {
    let Some(frame_format) = arg_value("--format") else {
        return Ok(None);
    };
    let width = arg_u32("--width").unwrap_or(640);
    let height = arg_u32("--height").unwrap_or(480);
    let fps = arg_u32("--fps").unwrap_or(30);
    Ok(Some(CameraFormat::new(
        Resolution::new(width, height),
        frame_format_from_str(&frame_format)?,
        fps,
    )))
}

fn arg_u32(flag: &str) -> Option<u32> {
    arg_value(flag).and_then(|value| value.parse().ok())
}

fn arg_usize(flag: &str) -> Option<usize> {
    arg_value(flag).and_then(|value| value.parse().ok())
}

fn arg_value(flag: &str) -> Option<String> {
    let mut args = env::args();
    while let Some(arg) = args.next() {
        if arg == flag {
            return args.next();
        }
    }
    None
}

fn print_duration_stats(label: &str, values: &[Duration]) {
    let values = values.iter().map(|value| ms(*value)).collect::<Vec<_>>();
    print_f64_stats(label, &values);
}

fn print_f64_stats(label: &str, values: &[f64]) {
    if values.is_empty() {
        return;
    }

    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.total_cmp(b));
    let avg = sorted.iter().sum::<f64>() / sorted.len() as f64;
    let p95_index = ((sorted.len() as f64 * 0.95).ceil() as usize)
        .saturating_sub(1)
        .min(sorted.len() - 1);

    println!(
        "{label}: min={:.2} avg={:.2} p95={:.2} max={:.2}",
        sorted[0],
        avg,
        sorted[p95_index],
        sorted[sorted.len() - 1]
    );
}

fn ms(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1000.0
}
