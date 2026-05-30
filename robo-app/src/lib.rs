use std::{
    collections::{HashMap, VecDeque},
    io::{Cursor, Read},
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, Condvar, Mutex,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use base64::{engine::general_purpose::STANDARD, Engine};
use image::{codecs::jpeg::JpegEncoder, imageops, ExtendedColorType, RgbImage};
use robo_camera::{
    frame_format_from_str, CameraConfig, CameraControlKind, CameraSlotStatus, CameraSlotWorker,
    CameraSlotWorkerEvent, CameraStatusEventKind, FramePacket, MultiCameraCapture,
    MultiCameraSource,
};
use robo_core::{CubeFace, DigitMap, Frame, Recognizer, Roi};
use robo_pipeline::multi::translate_optimal;
use robo_solver::search::{Search, SearchOptions};
use robo_transport::{default_digit_map, SerialTransport, MNEMONICS, MOVE_COUNT};
use robo_vision::ColorClusterRecognizer;
use serde::{Deserialize, Serialize};
use tauri::{Emitter, Manager};
use tiny_http::{Header, Response, Server};

struct AppState {
    camera: Mutex<CameraWorker>,
    mock_camera: Mutex<MockCameraWorker>,
    camera_stream: Mutex<Option<CameraStreamRuntime>>,
    serial: Mutex<Option<SerialRuntime>>,
    latest_frame: Arc<Mutex<Option<LatestFrame>>>,
    latest_frame_seq: Mutex<u64>,
    stream_hub: Arc<FrameHub>,
    discovery_cache: Mutex<DiscoveryCache>,
    frame_server_port: u16,
    mode: RuntimeMode,
    /// 求解表就绪标志（`Search::init()` 完成后 set；前端等待此标志再 invoke 求解命令）
    solver: Arc<std::sync::OnceLock<()>>,
    /// 动作 → 下位机数字映射（commands 用 mnemonic，encoded 用此映射）
    digit_map: Mutex<DigitMap>,
    /// 应用配置（持久化到 `app-config.json`）
    config: Mutex<AppConfig>,
    /// 前端推送的 ROI 状态，由 grid_timer 在每张 grid JPEG 上直接绘制矩形
    /// 与编号——把"54 个 SVG 矩形 + 文字"的渲染开销从 webview 主线程搬到
    /// 后端，避免 30Hz 画面刷新与 SVG 重排叠加导致点击/标注卡顿。
    overlay: Arc<Mutex<RoiOverlayState>>,
}

#[derive(Default, Clone)]
struct RoiOverlayState {
    /// 54 个 ROI（按 0..54 排序，None 表示未标注）。坐标已归一化到 [0,1]。
    /// 前端在相机模式下负责通过 `set_overlay_rois` 同步给后端。
    rois: Vec<Option<NormRoi>>,
    /// 当前选中的 ROI 索引（用于高亮颜色）。None 表示无选中。
    current: Option<usize>,
    /// 是否启用 overlay 绘制。文件模式 / 关闭相机时前端会传 false 关掉，
    /// 避免 grid_timer 多做无用功。
    enabled: bool,
}

#[derive(Default, Clone)]
struct NormRoi {
    x: f32,
    y: f32,
    w: f32,
    h: f32,
    /// ROI 标签（"U1".."B9"），由 grid_timer 画在矩形上方。
    label: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RuntimeMode {
    mock_camera: bool,
    mock_serial: bool,
}

impl RuntimeMode {
    fn from_env() -> Self {
        Self::from_env_values(
            std::env::var("CUBESOLVER_MOCK_CAMERA").ok().as_deref(),
            std::env::var("CUBESOLVER_MOCK_SERIAL").ok().as_deref(),
        )
    }

    fn from_env_values(mock_camera: Option<&str>, mock_serial: Option<&str>) -> Self {
        Self {
            mock_camera: env_flag_enabled(mock_camera),
            mock_serial: env_flag_enabled(mock_serial),
        }
    }
}

fn env_flag_enabled(value: Option<&str>) -> bool {
    value
        .map(|value| {
            matches!(
                value.trim().to_ascii_lowercase().as_str(),
                "1" | "true" | "yes" | "on"
            )
        })
        .unwrap_or(false)
}

#[derive(Clone)]
struct LatestFrame {
    bytes: Vec<u8>,
}

#[derive(Clone)]
struct StreamFrame {
    seq: u64,
    width: u32,
    height: u32,
    jpeg: Arc<Vec<u8>>,
    rgb: Arc<Frame>,
    capture_ms: u128,
    encode_ms: u128,
    created_at: Instant,
}

#[derive(Clone)]
struct SlotStreamState {
    status: CameraSlotStatus,
    frame: Option<StreamFrame>,
    last_diagnostic_log_at: Option<Instant>,
}

#[derive(Default)]
struct FrameHub {
    inner: Mutex<FrameHubInner>,
    changed: Condvar,
}

#[derive(Default)]
struct FrameHubInner {
    session_id: u64,
    slots: Vec<SlotStreamState>,
    grid: Option<StreamFrame>,
    tile_width: u32,
    tile_height: u32,
    columns: u32,
    grid_seq: u64,
    active: bool,
    last_grid_encode_at: Option<Instant>,
    last_grid_diagnostic_log_at: Option<Instant>,
    last_snapshot_diagnostic_log_at: Option<Instant>,
    /// 节流"camera-stream-event(frame)"事件给前端 UI 的频率。
    /// 4 路 worker × ~25fps = 100 events/s，前端 setState 跟不上还会
    /// 把整个 React 渲染线程打满 → 画面卡顿 / 看似的"500fps"假象。
    /// 实际帧依然走 MJPEG `<img>` 实时显示，UI 状态栏 5fps 刷新就够了。
    last_frame_event_emit_at: Option<Instant>,
}

#[derive(Default)]
struct DiscoveryCache {
    cameras: Option<CachedValue<Vec<CameraDeviceDto>>>,
    formats: HashMap<u32, CachedValue<Vec<CameraFormatDto>>>,
}

struct CachedValue<T> {
    value: T,
    expires_at: Instant,
}

struct CameraStreamRuntime {
    workers: Vec<CameraStreamWorker>,
    /// 处理来自 worker 的状态/事件流（不再处理 frame，frame 直接写入 hub）。
    aggregator: Option<JoinHandle<()>>,
    /// 定时拉取 4 路最新 RGB → 拼接 → JPEG 编码 → 写回 hub.grid。
    /// 让 grid 编码频率与 worker 产帧解耦，永远拼"最新一张"，不会堆积延迟。
    grid_timer: Option<JoinHandle<()>>,
    grid_timer_stop: Arc<AtomicBool>,
}

/// grid 拼接 + JPEG 编码的固定周期：33ms（≈30Hz），与 RobotApp 的
/// `camTimer->start(33)` 一致。grid encode 实测 ~10ms，足够留出余量。
const GRID_ENCODE_INTERVAL: Duration = Duration::from_millis(33);
const DIAGNOSTIC_LOG_INTERVAL: Duration = Duration::from_secs(1);
const DIAGNOSTIC_WARN_THRESHOLD: Duration = Duration::from_millis(200);
const BACKEND_DIAGNOSTIC_PREFIX: &str = "[backend-diagnostic]";

/// 拼接画布的单格尺寸固定为 640x480，2x2 布局合成 1280x960 总画布——与
/// RobotApp 历史约定一致，ROI 坐标基于这一固定画布编辑/保存，切换源相机
/// 分辨率时画布尺寸不变，ROI 永远对得上。
/// 单格内部直接 resize 到 (640, 480)，**允许变形**，不做 letterbox。
const GRID_TILE_WIDTH: u32 = 640;
const GRID_TILE_HEIGHT: u32 = 480;

impl Default for AppState {
    fn default() -> Self {
        let latest_frame = Arc::new(Mutex::new(None));
        let stream_hub = Arc::new(FrameHub::default());
        let frame_server_port =
            start_frame_server(Arc::clone(&latest_frame), Arc::clone(&stream_hub)).unwrap_or(0);
        Self {
            camera: Mutex::default(),
            mock_camera: Mutex::default(),
            camera_stream: Mutex::default(),
            serial: Mutex::default(),
            latest_frame,
            latest_frame_seq: Mutex::default(),
            stream_hub,
            discovery_cache: Mutex::default(),
            frame_server_port,
            mode: RuntimeMode::from_env(),
            solver: Arc::new(std::sync::OnceLock::new()),
            digit_map: Mutex::new(default_digit_map()),
            config: Mutex::new(AppConfig::default()),
            overlay: Arc::new(Mutex::new(RoiOverlayState::default())),
        }
    }
}

#[derive(Default)]
struct CameraWorker {
    tx: Option<mpsc::Sender<CameraRequest>>,
}

enum CameraRequest {
    Capture(mpsc::Sender<Result<MultiCameraCapture, String>>),
    Controls {
        slot: usize,
        reply: mpsc::Sender<Result<Vec<robo_camera::CameraControlInfo>, String>>,
    },
    SetControl {
        slot: usize,
        id: String,
        value: f64,
        reply: mpsc::Sender<Result<(), String>>,
    },
    Close(mpsc::Sender<Result<(), String>>),
}

impl CameraWorker {
    fn open(&mut self, configs: Vec<CameraConfig>) -> Result<()> {
        self.close()?;
        let (tx, rx) = mpsc::channel::<CameraRequest>();
        thread::spawn(move || {
            let mut source = match MultiCameraSource::open(configs, 2) {
                Ok(source) => source,
                Err(err) => {
                    while let Ok(request) = rx.recv() {
                        match request {
                            CameraRequest::Capture(reply) => {
                                let _ = reply.send(Err(err.to_string()));
                            }
                            CameraRequest::Controls { reply, .. } => {
                                let _ = reply.send(Err(err.to_string()));
                            }
                            CameraRequest::SetControl { reply, .. } => {
                                let _ = reply.send(Err(err.to_string()));
                            }
                            CameraRequest::Close(reply) => {
                                let _ = reply.send(Ok(()));
                                break;
                            }
                        }
                    }
                    return;
                }
            };

            while let Ok(request) = rx.recv() {
                match request {
                    CameraRequest::Capture(reply) => {
                        let result = source.capture_with_status().map_err(|err| err.to_string());
                        let _ = reply.send(result);
                    }
                    CameraRequest::Controls { slot, reply } => {
                        let result = source.camera_controls(slot).map_err(|err| err.to_string());
                        let _ = reply.send(result);
                    }
                    CameraRequest::SetControl {
                        slot,
                        id,
                        value,
                        reply,
                    } => {
                        let result = source
                            .set_camera_control(slot, &id, value)
                            .map_err(|err| err.to_string());
                        let _ = reply.send(result);
                    }
                    CameraRequest::Close(reply) => {
                        let _ = reply.send(Ok(()));
                        break;
                    }
                }
            }
        });
        self.tx = Some(tx);
        Ok(())
    }

    fn capture(&self) -> Result<MultiCameraCapture> {
        let tx = self.tx.as_ref().context("camera is not open")?.clone();
        let (reply_tx, reply_rx) = mpsc::channel();
        tx.send(CameraRequest::Capture(reply_tx))
            .context("failed to request camera frame")?;
        reply_rx
            .recv()
            .context("camera worker stopped")?
            .map_err(anyhow::Error::msg)
    }

    fn controls(&self, slot: usize) -> Result<Vec<robo_camera::CameraControlInfo>> {
        let tx = self.tx.as_ref().context("camera is not open")?.clone();
        let (reply, rx) = mpsc::channel();
        tx.send(CameraRequest::Controls { slot, reply })
            .context("failed to request camera controls")?;
        rx.recv()
            .context("camera worker stopped")?
            .map_err(anyhow::Error::msg)
    }

    fn set_control(&self, slot: usize, id: String, value: f64) -> Result<()> {
        let tx = self.tx.as_ref().context("camera is not open")?.clone();
        let (reply, rx) = mpsc::channel();
        tx.send(CameraRequest::SetControl {
            slot,
            id,
            value,
            reply,
        })
        .context("failed to request camera control update")?;
        rx.recv()
            .context("camera worker stopped")?
            .map_err(anyhow::Error::msg)
    }

    fn close(&mut self) -> Result<()> {
        if let Some(tx) = self.tx.take() {
            let (reply_tx, reply_rx) = mpsc::channel();
            tx.send(CameraRequest::Close(reply_tx))
                .context("failed to request camera close")?;
            reply_rx
                .recv()
                .context("camera worker stopped while closing")?
                .map_err(anyhow::Error::msg)?;
        }
        Ok(())
    }
}

#[derive(Default)]
struct MockCameraWorker {
    configs: Vec<CameraConfig>,
    control_values: HashMap<(usize, String), f64>,
    seq: u64,
}

impl MockCameraWorker {
    fn open(&mut self, configs: Vec<CameraConfig>) -> Result<()> {
        anyhow::ensure!(!configs.is_empty(), "at least one mock camera is required");
        self.configs = configs;
        self.control_values.clear();
        self.seq = 0;
        Ok(())
    }

    fn close(&mut self) {
        self.configs.clear();
        self.control_values.clear();
        self.seq = 0;
    }

    fn capture(&mut self) -> Result<MultiCameraCapture> {
        anyhow::ensure!(!self.configs.is_empty(), "mock camera is not open");
        self.seq = self.seq.wrapping_add(1);
        let frames = self
            .configs
            .iter()
            .enumerate()
            .map(|(slot, config)| {
                mock_camera_frame(slot, config, self.seq)
                    .map(Arc::new)
                    .map(Some)
            })
            .collect::<Result<Vec<_>>>()?;
        // mock 路径同样使用全局固定的 grid tile 尺寸，让识别端拿到的帧
        // 永远是 1280x960，与 ROI 坐标系对齐。
        let frame = compose_grid_frame(frames, GRID_TILE_WIDTH, GRID_TILE_HEIGHT, 2)?
            .context("failed to compose mock camera capture")?;
        Ok(MultiCameraCapture {
            frame,
            statuses: self
                .configs
                .iter()
                .enumerate()
                .map(|(slot, config)| {
                    mock_camera_status(slot, config, true, "mock camera connected")
                })
                .collect(),
            events: Vec::new(),
        })
    }

    fn controls(&self, slot: usize) -> Result<Vec<robo_camera::CameraControlInfo>> {
        anyhow::ensure!(
            slot < self.configs.len(),
            "mock camera slot {slot} is not open"
        );
        Ok(mock_camera_controls(slot)
            .into_iter()
            .map(|mut control| {
                if let Some(value) = self.control_values.get(&(slot, control.id.clone())) {
                    control.value = *value;
                }
                control
            })
            .collect())
    }

    fn set_control(&mut self, slot: usize, id: String, value: f64) -> Result<()> {
        let controls = self.controls(slot)?;
        anyhow::ensure!(
            controls.iter().any(|control| control.id == id),
            "unknown mock camera control {id}"
        );
        self.control_values.insert((slot, id), value);
        Ok(())
    }
}

struct MockCameraStreamWorker {
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl MockCameraStreamWorker {
    fn spawn(
        slot: usize,
        config: CameraConfig,
        events: mpsc::Sender<CameraSlotWorkerEvent>,
    ) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = Arc::clone(&stop);
        let join = thread::spawn(move || {
            let _ = events.send(CameraSlotWorkerEvent::Status(mock_camera_status(
                slot,
                &config,
                true,
                "mock camera connected",
            )));
            let _ = events.send(CameraSlotWorkerEvent::Event(
                robo_camera::CameraStatusEvent {
                    slot,
                    index: config.index,
                    kind: CameraStatusEventKind::Connected,
                    message: "mock camera connected".to_string(),
                },
            ));

            let frame_interval = Duration::from_millis((1000 / config.fps.max(1) as u64).max(1));
            let mut seq = 0u64;
            while !worker_stop.load(Ordering::SeqCst) {
                seq = seq.wrapping_add(1);
                match mock_camera_frame(slot, &config, seq) {
                    Ok(frame) => {
                        if events
                            .send(CameraSlotWorkerEvent::Frame(FramePacket {
                                slot,
                                index: config.index,
                                seq,
                                frame,
                                capture_ms: frame_interval.as_millis(),
                            }))
                            .is_err()
                        {
                            break;
                        }
                    }
                    Err(err) => {
                        let _ = events.send(CameraSlotWorkerEvent::Status(mock_camera_status(
                            slot,
                            &config,
                            false,
                            &err.to_string(),
                        )));
                        break;
                    }
                }
                thread::sleep(frame_interval);
            }

            let _ = events.send(CameraSlotWorkerEvent::Event(
                robo_camera::CameraStatusEvent {
                    slot,
                    index: config.index,
                    kind: CameraStatusEventKind::Disconnected,
                    message: "mock camera stopped".to_string(),
                },
            ));
        });
        Self {
            stop,
            join: Some(join),
        }
    }

    fn close(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

fn mock_camera_devices() -> Vec<CameraDeviceDto> {
    (0..4)
        .map(|index| CameraDeviceDto {
            index: index.to_string(),
            name: format!("Mock Camera {}", index + 1),
            description: format!("Synthetic mock camera slot {}", index + 1),
        })
        .collect()
}

fn mock_camera_formats(_index: u32) -> Vec<CameraFormatDto> {
    [(640, 480), (1280, 720), (1920, 1080)]
        .into_iter()
        .map(|(width, height)| CameraFormatDto {
            width,
            height,
            fps: 30,
            frame_format: "MJPEG".to_string(),
        })
        .collect()
}

fn mock_camera_status(
    slot: usize,
    config: &CameraConfig,
    connected: bool,
    message: &str,
) -> CameraSlotStatus {
    CameraSlotStatus {
        slot,
        index: config.index,
        connected,
        message: message.to_string(),
    }
}

fn mock_camera_controls(slot: usize) -> Vec<robo_camera::CameraControlInfo> {
    vec![
        robo_camera::CameraControlInfo {
            id: "brightness".to_string(),
            name: format!("Mock Brightness {}", slot + 1),
            kind: CameraControlKind::Integer,
            value: 50.0,
            default: 50.0,
            min: Some(0.0),
            max: Some(100.0),
            step: Some(1.0),
            active: true,
            flags: vec!["mock".to_string()],
        },
        robo_camera::CameraControlInfo {
            id: "exposure".to_string(),
            name: format!("Mock Exposure {}", slot + 1),
            kind: CameraControlKind::Float,
            value: 0.5,
            default: 0.5,
            min: Some(0.0),
            max: Some(1.0),
            step: Some(0.05),
            active: true,
            flags: vec!["mock".to_string()],
        },
        robo_camera::CameraControlInfo {
            id: "auto_exposure".to_string(),
            name: format!("Mock Auto Exposure {}", slot + 1),
            kind: CameraControlKind::Boolean,
            value: 1.0,
            default: 1.0,
            min: Some(0.0),
            max: Some(1.0),
            step: Some(1.0),
            active: true,
            flags: vec!["mock".to_string()],
        },
    ]
}

fn mock_camera_frame(slot: usize, config: &CameraConfig, seq: u64) -> Result<Frame> {
    let width = config.width.max(1);
    let height = config.height.max(1);
    let palette = [
        [220u8, 64u8, 64u8],
        [64u8, 180u8, 96u8],
        [64u8, 112u8, 220u8],
        [220u8, 180u8, 64u8],
    ];
    let base = palette[slot % palette.len()];
    let mut rgb = vec![0u8; width as usize * height as usize * 3];
    let moving_bar = (seq as u32 * 7) % width;

    for y in 0..height {
        for x in 0..width {
            let idx = ((y * width + x) * 3) as usize;
            let gradient = ((x * 80 / width) + (y * 50 / height)) as u8;
            rgb[idx] = base[0].saturating_add(gradient / 4);
            rgb[idx + 1] = base[1].saturating_add(gradient / 3);
            rgb[idx + 2] = base[2].saturating_add(gradient / 2);
            if x.abs_diff(moving_bar) < 4 {
                rgb[idx] = 255;
                rgb[idx + 1] = 255u8.saturating_sub((slot as u8).saturating_mul(35));
                rgb[idx + 2] = 255;
            }
        }
    }

    let band_height = (height / 8).clamp(24, 96);
    fill_rect(
        &mut rgb,
        width,
        height,
        0,
        0,
        width,
        band_height,
        [16, 16, 16],
    );
    draw_number(
        &mut rgb,
        width,
        height,
        12,
        6,
        (slot + 1) as u32,
        [255, 255, 255],
    );
    draw_number(
        &mut rgb,
        width,
        height,
        (width / 4).max(72),
        6,
        (seq % 10_000) as u32,
        [255, 240, 160],
    );

    Frame::new_rgb(width, height, rgb).context("failed to build mock camera frame")
}

fn draw_number(
    rgb: &mut [u8],
    width: u32,
    height: u32,
    mut x: u32,
    y: u32,
    value: u32,
    color: [u8; 3],
) {
    let digits = value.to_string();
    for digit in digits.bytes().filter_map(|digit| digit.checked_sub(b'0')) {
        draw_digit(rgb, width, height, x, y, digit, color);
        x += 18;
    }
}

fn draw_digit(rgb: &mut [u8], width: u32, height: u32, x: u32, y: u32, digit: u8, color: [u8; 3]) {
    let segments = match digit {
        0 => [true, true, true, true, true, true, false],
        1 => [false, true, true, false, false, false, false],
        2 => [true, true, false, true, true, false, true],
        3 => [true, true, true, true, false, false, true],
        4 => [false, true, true, false, false, true, true],
        5 => [true, false, true, true, false, true, true],
        6 => [true, false, true, true, true, true, true],
        7 => [true, true, true, false, false, false, false],
        8 => [true, true, true, true, true, true, true],
        9 => [true, true, true, true, false, true, true],
        _ => return,
    };
    let rects = [
        (x + 2, y, 10, 3),
        (x + 12, y + 2, 3, 10),
        (x + 12, y + 14, 3, 10),
        (x + 2, y + 24, 10, 3),
        (x, y + 14, 3, 10),
        (x, y + 2, 3, 10),
        (x + 2, y + 12, 10, 3),
    ];
    for (enabled, (rect_x, rect_y, rect_width, rect_height)) in segments.into_iter().zip(rects) {
        if enabled {
            fill_rect(
                rgb,
                width,
                height,
                rect_x,
                rect_y,
                rect_width,
                rect_height,
                color,
            );
        }
    }
}

fn fill_rect(
    rgb: &mut [u8],
    width: u32,
    height: u32,
    x: u32,
    y: u32,
    rect_width: u32,
    rect_height: u32,
    color: [u8; 3],
) {
    let max_x = (x + rect_width).min(width);
    let max_y = (y + rect_height).min(height);
    for py in y.min(height)..max_y {
        for px in x.min(width)..max_x {
            let idx = ((py * width + px) * 3) as usize;
            rgb[idx..idx + 3].copy_from_slice(&color);
        }
    }
}

impl CameraStreamRuntime {
    fn open(
        configs: Vec<CameraConfig>,
        hub: Arc<FrameHub>,
        app: tauri::AppHandle,
        overlay: Arc<Mutex<RoiOverlayState>>,
    ) -> Result<Self> {
        let session_id = hub.configure(&configs)?;
        let (tx, rx) = mpsc::channel();
        let workers = configs
            .iter()
            .cloned()
            .enumerate()
            .map(|(slot, config)| {
                CameraStreamWorker::Real(CameraSlotWorker::spawn(slot, config, tx.clone()))
            })
            .collect::<Vec<_>>();
        drop(tx);

        let aggregator =
            spawn_camera_stream_aggregator(Arc::clone(&hub), session_id, app.clone(), rx);
        let grid_timer_stop = Arc::new(AtomicBool::new(false));
        let grid_timer = spawn_grid_timer(
            Arc::clone(&hub),
            session_id,
            app,
            Arc::clone(&grid_timer_stop),
            overlay,
        );

        Ok(Self {
            workers,
            aggregator: Some(aggregator),
            grid_timer: Some(grid_timer),
            grid_timer_stop,
        })
    }

    fn open_mock(
        configs: Vec<CameraConfig>,
        hub: Arc<FrameHub>,
        app: tauri::AppHandle,
        overlay: Arc<Mutex<RoiOverlayState>>,
    ) -> Result<Self> {
        let session_id = hub.configure(&configs)?;
        let (tx, rx) = mpsc::channel();
        let workers = configs
            .iter()
            .cloned()
            .enumerate()
            .map(|(slot, config)| {
                CameraStreamWorker::Mock(MockCameraStreamWorker::spawn(slot, config, tx.clone()))
            })
            .collect::<Vec<_>>();
        drop(tx);

        let aggregator =
            spawn_camera_stream_aggregator(Arc::clone(&hub), session_id, app.clone(), rx);
        let grid_timer_stop = Arc::new(AtomicBool::new(false));
        let grid_timer = spawn_grid_timer(
            Arc::clone(&hub),
            session_id,
            app,
            Arc::clone(&grid_timer_stop),
            overlay,
        );

        Ok(Self {
            workers,
            aggregator: Some(aggregator),
            grid_timer: Some(grid_timer),
            grid_timer_stop,
        })
    }

    fn close(&mut self) {
        // 先停 grid timer：notify_all 唤醒任何在 hub.changed 上等待的线程，
        // timer 自身的 sleep 也会通过 stop flag 提前退出。
        self.grid_timer_stop.store(true, Ordering::SeqCst);
        if let Some(handle) = self.grid_timer.take() {
            // 不 join——driver 可能正在 capture 阻塞。timer 是纯 CPU 线程，
            // 50ms 内会自然退出，drop handle detach 即可。
            drop(handle);
        }
        for worker in &mut self.workers {
            worker.close();
        }
        self.workers.clear();
        // Dropping the JoinHandle detaches the aggregator. It exits once all
        // worker senders close, but close_camera_stream must not wait on a
        // device thread that may be stuck in a driver call.
        let _ = self.aggregator.take();
    }
}

impl Drop for CameraStreamRuntime {
    fn drop(&mut self) {
        self.close();
    }
}

enum CameraStreamWorker {
    Real(CameraSlotWorker),
    Mock(MockCameraStreamWorker),
}

impl CameraStreamWorker {
    fn close(&mut self) {
        match self {
            Self::Real(worker) => worker.close(),
            Self::Mock(worker) => worker.close(),
        }
    }
}

impl CameraStreamRuntime {
    /// 流式预览模式下读取相机控制参数：直接走 CameraSlotWorker 的 control channel
    /// （capture loop 间隙处理），无需关闭流就能拿到当前生效的硬件参数值。
    fn controls(&self, slot: usize) -> Result<Vec<robo_camera::CameraControlInfo>> {
        let worker = self
            .workers
            .get(slot)
            .with_context(|| format!("camera stream slot {slot} does not exist"))?;
        match worker {
            CameraStreamWorker::Real(worker) => worker.controls(),
            CameraStreamWorker::Mock(_) => {
                anyhow::bail!("mock stream slot does not support real camera controls")
            }
        }
    }

    fn set_control(&self, slot: usize, id: String, value: f64) -> Result<()> {
        let worker = self
            .workers
            .get(slot)
            .with_context(|| format!("camera stream slot {slot} does not exist"))?;
        match worker {
            CameraStreamWorker::Real(worker) => worker.set_control(id, value),
            CameraStreamWorker::Mock(_) => {
                anyhow::bail!("mock stream slot does not support real camera controls")
            }
        }
    }
}

fn spawn_camera_stream_aggregator(
    hub: Arc<FrameHub>,
    session_id: u64,
    app: tauri::AppHandle,
    rx: mpsc::Receiver<CameraSlotWorkerEvent>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        while let Ok(event) = rx.recv() {
            match event {
                CameraSlotWorkerEvent::Frame(packet) => {
                    if let Err(err) = publish_stream_packet(&hub, session_id, &app, packet) {
                        let _ = app.emit(
                            "camera-stream-event",
                            CameraStreamEventDto {
                                kind: "error".to_string(),
                                slot: None,
                                index: None,
                                seq: None,
                                width: None,
                                height: None,
                                fps: None,
                                capture_ms: None,
                                encode_ms: None,
                                message: Some(err.to_string()),
                                statuses: hub.statuses().unwrap_or_default(),
                            },
                        );
                    }
                }
                CameraSlotWorkerEvent::Status(status) => {
                    let statuses = hub.update_status(session_id, status).unwrap_or_default();
                    let _ = app.emit(
                        "camera-stream-event",
                        CameraStreamEventDto {
                            kind: "status".to_string(),
                            slot: None,
                            index: None,
                            seq: None,
                            width: None,
                            height: None,
                            fps: None,
                            capture_ms: None,
                            encode_ms: None,
                            message: None,
                            statuses,
                        },
                    );
                }
                CameraSlotWorkerEvent::Event(event) => {
                    if !hub.is_active_session(session_id) {
                        continue;
                    }
                    let _ = app.emit(
                        "camera-stream-event",
                        CameraStreamEventDto {
                            kind: match event.kind {
                                CameraStatusEventKind::Connected => "connected".to_string(),
                                CameraStatusEventKind::Disconnected => "disconnected".to_string(),
                            },
                            slot: Some(event.slot),
                            index: Some(event.index),
                            seq: None,
                            width: None,
                            height: None,
                            fps: None,
                            capture_ms: None,
                            encode_ms: None,
                            message: Some(event.message),
                            statuses: hub.statuses().unwrap_or_default(),
                        },
                    );
                }
            }
        }
    })
}

/// 定时拼接 grid + 编码 JPEG 写回 hub.grid 的后台线程。
///
/// 与 worker 完全解耦：worker 只负责覆盖式更新各槽位的最新一帧；这里每隔
/// `GRID_ENCODE_INTERVAL`（33ms / ≈30Hz，与 RobotApp camTimer 一致）拉一次
/// 4 路最新 RGB 进行拼接 + 编码：
/// - grid 输出帧率稳定 30Hz，不受 worker 产帧速率波动影响；
/// - 不会因 4 路 worker 并发产帧而把 aggregator 单线程打满；
/// - driver 即使吐出旧帧 / 速率失控（500fps）也只是不停覆盖槽位最新一帧，
///   timer 永远拼"当下最新"，UI 延迟上限 = 一个 timer 周期 ≈ 33ms。
fn spawn_grid_timer(
    hub: Arc<FrameHub>,
    session_id: u64,
    app: tauri::AppHandle,
    stop: Arc<AtomicBool>,
    overlay: Arc<Mutex<RoiOverlayState>>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        while !stop.load(Ordering::SeqCst) {
            let cycle_started = Instant::now();
            // 在锁内只读 4 路 RGB 的 Arc 引用 + tile 配置，立刻释放锁；
            // 避免 compose + JPEG 编码这种 ~10ms 的重活儿持锁阻塞 worker。
            let snapshot = match hub.snapshot_slots(session_id) {
                Ok(Some(snapshot)) => snapshot,
                Ok(None) => break, // session 失效（被 close 或 reconfigure），退出
                Err(_) => break,
            };
            // 只有在至少有一路收到帧时才做 grid 编码——避免设备未就绪时
            // 反复编码全黑画面。
            let any_frame = snapshot.frames.iter().any(|f| f.is_some());
            if any_frame {
                let compose_started = Instant::now();
                match compose_grid_frame(
                    snapshot.frames,
                    snapshot.tile_width,
                    snapshot.tile_height,
                    snapshot.columns,
                ) {
                    Ok(Some(mut grid_frame)) => {
                        // 在 RGB buffer 上直接画 ROI 矩形（不画文字，简单快），
                        // 避免前端 54 个 SVG 矩形 + 文字与 30Hz 画面刷新争主线程。
                        if let Ok(overlay_state) = overlay.lock() {
                            if overlay_state.enabled {
                                draw_overlay_rois(&mut grid_frame, &overlay_state);
                            }
                        }
                        let compose_ms = compose_started.elapsed().as_millis();
                        let encode_started = Instant::now();
                        match encode_frame_jpeg(&grid_frame) {
                            Ok(grid_jpeg) => {
                                let grid_encode_ms = encode_started.elapsed().as_millis();
                                let _ = hub.publish_grid_frame(
                                    session_id,
                                    grid_frame,
                                    grid_jpeg,
                                    compose_ms,
                                    grid_encode_ms,
                                );
                            }
                            Err(err) => {
                                let _ = app.emit(
                                    "camera-stream-event",
                                    CameraStreamEventDto {
                                        kind: "error".to_string(),
                                        slot: None,
                                        index: None,
                                        seq: None,
                                        width: None,
                                        height: None,
                                        fps: None,
                                        capture_ms: None,
                                        encode_ms: None,
                                        message: Some(format!("grid encode failed: {err}")),
                                        statuses: hub.statuses().unwrap_or_default(),
                                    },
                                );
                            }
                        }
                    }
                    Ok(None) => {}
                    Err(_) => {}
                }
            }

            // sleep 补足到 GRID_ENCODE_INTERVAL，让本拍周期固定为 33ms。
            // 用 stop flag 分段轮询，避免 close 时还要等满一拍。
            let elapsed = cycle_started.elapsed();
            if elapsed < GRID_ENCODE_INTERVAL {
                let mut remain = GRID_ENCODE_INTERVAL - elapsed;
                while remain > Duration::ZERO {
                    if stop.load(Ordering::SeqCst) {
                        return;
                    }
                    let chunk = remain.min(Duration::from_millis(20));
                    thread::sleep(chunk);
                    remain = remain.saturating_sub(chunk);
                }
            }
        }
    })
}

/// 由 timer 线程拼好 grid 后写回 hub.grid 的辅助方法。
struct GridSnapshot {
    frames: Vec<Option<Arc<Frame>>>,
    tile_width: u32,
    tile_height: u32,
    columns: u32,
}

impl FrameHub {
    fn configure(&self, configs: &[CameraConfig]) -> Result<u64> {
        anyhow::ensure!(!configs.is_empty(), "at least one camera is required");
        // 固定 tile 尺寸，使 grid 总分辨率与各路相机的实际分辨率解耦。
        // 单格内部由 compose_grid_frame letterbox 适配源尺寸。
        let tile_width = GRID_TILE_WIDTH;
        let tile_height = GRID_TILE_HEIGHT;
        let slots = configs
            .iter()
            .cloned()
            .enumerate()
            .map(|(slot, config)| SlotStreamState {
                status: CameraSlotStatus {
                    slot,
                    index: config.index,
                    connected: false,
                    message: "waiting for camera".to_string(),
                },
                frame: None,
                last_diagnostic_log_at: None,
            })
            .collect();

        let mut inner = self
            .inner
            .lock()
            .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?;
        let session_id = inner.session_id.wrapping_add(1);
        *inner = FrameHubInner {
            session_id,
            slots,
            grid: None,
            tile_width,
            tile_height,
            columns: 2,
            grid_seq: 0,
            active: true,
            last_grid_encode_at: None,
            last_grid_diagnostic_log_at: None,
            last_snapshot_diagnostic_log_at: None,
            last_frame_event_emit_at: None,
        };
        self.changed.notify_all();
        Ok(session_id)
    }

    fn clear(&self) -> Result<()> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?;
        let session_id = inner.session_id.wrapping_add(1);
        *inner = FrameHubInner {
            session_id,
            ..FrameHubInner::default()
        };
        self.changed.notify_all();
        Ok(())
    }

    fn stream_info(&self, port: u16) -> Result<CameraStreamInfoDto> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?;
        let columns = inner.columns.max(1);
        let rows = if inner.slots.is_empty() {
            0
        } else {
            (inner.slots.len() as u32 + columns - 1) / columns
        };
        Ok(CameraStreamInfoDto {
            grid_url: format!("http://127.0.0.1:{port}/grid.mjpeg"),
            slot_urls: (0..inner.slots.len())
                .map(|slot| format!("http://127.0.0.1:{port}/slot/{slot}.mjpeg"))
                .collect(),
            width: inner.tile_width * columns,
            height: inner.tile_height * rows.max(1),
            statuses: inner
                .slots
                .iter()
                .map(|slot| status_dto(&slot.status))
                .collect(),
        })
    }

    fn update_status(
        &self,
        session_id: u64,
        status: CameraSlotStatus,
    ) -> Result<Vec<CameraStatusDto>> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?;
        if inner.session_id != session_id {
            return Ok(inner
                .slots
                .iter()
                .map(|slot| status_dto(&slot.status))
                .collect());
        }
        if let Some(slot) = inner.slots.get_mut(status.slot) {
            slot.status = status;
        }
        let statuses = inner
            .slots
            .iter()
            .map(|slot| status_dto(&slot.status))
            .collect();
        self.changed.notify_all();
        Ok(statuses)
    }

    /// 写入指定槽位的最新帧——**只更新内存里的"最新一帧"，不做任何 grid 拼接/编码**。
    ///
    /// grid 的合成由独立的 `spawn_grid_timer` 线程定时（50ms）拉取所有槽位的最新
    /// RGB 自行完成，与 worker 产帧速率彻底解耦：
    /// - worker 产帧再快也只是覆盖式刷新自己槽位的"最新一帧"，不会堆积；
    /// - grid timer 永远拼"当下最新的 4 张"，所以即便某路相机吐出旧帧，下一拍
    ///   一旦真的有新帧到，UI 就能立刻看到——彻底消除"延迟越积越高"的现象。
    fn publish_slot_frame(
        &self,
        session_id: u64,
        packet: FramePacket,
        jpeg: Vec<u8>,
        encode_ms: u128,
    ) -> Result<Option<StreamFrame>> {
        let FramePacket {
            slot,
            index,
            seq,
            frame,
            capture_ms,
        } = packet;
        let created_at = Instant::now();
        let stream_frame = StreamFrame {
            seq,
            width: frame.width,
            height: frame.height,
            jpeg: Arc::new(jpeg),
            rgb: Arc::new(frame),
            capture_ms,
            encode_ms,
            created_at,
        };

        let mut inner = self
            .inner
            .lock()
            .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?;
        if inner.session_id != session_id {
            return Ok(None);
        }
        let slot_state = inner
            .slots
            .get_mut(slot)
            .with_context(|| format!("stream slot {} does not exist", slot))?;
        let slot_interval = slot_state
            .frame
            .as_ref()
            .map(|frame| created_at.duration_since(frame.created_at));
        let slot_warning = slot_interval
            .map(|interval| interval > DIAGNOSTIC_WARN_THRESHOLD)
            .unwrap_or(false);
        if should_log_diagnostic(
            &mut slot_state.last_diagnostic_log_at,
            created_at,
            slot_warning,
        ) {
            log::info!(
                "{BACKEND_DIAGNOSTIC_PREFIX} category=slot_frame level={} slot={} packet_seq={} capture_ms={} encode_ms={} interval_ms={}",
                diagnostic_level(slot_warning),
                slot,
                seq,
                stream_frame.capture_ms,
                stream_frame.encode_ms,
                diagnostic_duration_ms(slot_interval)
            );
        }
        slot_state.status = CameraSlotStatus {
            slot,
            index,
            connected: true,
            message: "connected".to_string(),
        };
        slot_state.frame = Some(stream_frame.clone());
        // 仅唤醒 /slot/N.mjpeg 这种慢消费者的 wait_slot_frame；
        // grid 由 timer 自行调度，无需在这里 notify。
        self.changed.notify_all();
        Ok(Some(stream_frame))
    }

    /// 在锁内只 clone Arc<Frame> 引用 + 读 tile 配置，立刻释放锁；
    /// 用于 grid_timer 在锁外做耗时的 compose + 编码。
    fn snapshot_slots(&self, session_id: u64) -> Result<Option<GridSnapshot>> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?;
        if !inner.active || inner.session_id != session_id {
            return Ok(None);
        }
        let frames = inner
            .slots
            .iter()
            .map(|slot| slot.frame.as_ref().map(|frame| Arc::clone(&frame.rgb)))
            .collect::<Vec<_>>();
        Ok(Some(GridSnapshot {
            frames,
            tile_width: inner.tile_width,
            tile_height: inner.tile_height,
            columns: inner.columns,
        }))
    }

    /// timer 拼接好 grid + 编码完 JPEG 后写回 hub。
    /// 同时记录 grid 诊断日志，节流到 1Hz。
    fn publish_grid_frame(
        &self,
        session_id: u64,
        grid_frame: Frame,
        grid_jpeg: Vec<u8>,
        compose_ms: u128,
        grid_encode_ms: u128,
    ) -> Result<()> {
        let grid_created_at = Instant::now();
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?;
        if !inner.active || inner.session_id != session_id {
            return Ok(());
        }
        let grid_interval = inner
            .grid
            .as_ref()
            .map(|frame| grid_created_at.duration_since(frame.created_at));
        inner.grid_seq += 1;
        let grid_seq = inner.grid_seq;
        let grid_warning = grid_interval
            .map(|interval| interval > DIAGNOSTIC_WARN_THRESHOLD)
            .unwrap_or(false)
            || compose_ms > DIAGNOSTIC_WARN_THRESHOLD.as_millis()
            || grid_encode_ms > DIAGNOSTIC_WARN_THRESHOLD.as_millis();
        if should_log_diagnostic(
            &mut inner.last_grid_diagnostic_log_at,
            grid_created_at,
            grid_warning,
        ) {
            log::info!(
                "{BACKEND_DIAGNOSTIC_PREFIX} category=grid_frame level={} grid_seq={} compose_ms={} encode_ms={} interval_ms={}",
                diagnostic_level(grid_warning),
                grid_seq,
                compose_ms,
                grid_encode_ms,
                diagnostic_duration_ms(grid_interval)
            );
        }
        inner.grid = Some(StreamFrame {
            seq: grid_seq,
            width: grid_frame.width,
            height: grid_frame.height,
            jpeg: Arc::new(grid_jpeg),
            rgb: Arc::new(grid_frame),
            capture_ms: 0,
            encode_ms: grid_encode_ms,
            created_at: grid_created_at,
        });
        inner.last_grid_encode_at = Some(grid_created_at);
        self.changed.notify_all();
        Ok(())
    }

    fn wait_slot_frame(&self, slot: usize, last_seq: u64) -> Option<StreamFrame> {
        let mut inner = self.inner.lock().ok()?;
        loop {
            if !inner.active {
                return None;
            }
            if let Some(frame) = inner
                .slots
                .get(slot)
                .and_then(|slot| slot.frame.as_ref())
                .filter(|frame| frame.seq != last_seq)
            {
                return Some(frame.clone());
            }
            inner = self.changed.wait(inner).ok()?;
        }
    }

    fn wait_grid_frame(&self, last_seq: u64) -> Option<StreamFrame> {
        let mut inner = self.inner.lock().ok()?;
        loop {
            if !inner.active {
                return None;
            }
            if let Some(frame) = inner.grid.as_ref().filter(|frame| frame.seq != last_seq) {
                return Some(frame.clone());
            }
            inner = self.changed.wait(inner).ok()?;
        }
    }

    fn latest_grid_rgb(&self) -> Result<Frame> {
        self.inner
            .lock()
            .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?
            .grid
            .as_ref()
            .map(|frame| (*frame.rgb).clone())
            .context("no stream frame available")
    }

    fn statuses(&self) -> Result<Vec<CameraStatusDto>> {
        Ok(self
            .inner
            .lock()
            .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?
            .slots
            .iter()
            .map(|slot| status_dto(&slot.status))
            .collect())
    }

    fn is_active_session(&self, session_id: u64) -> bool {
        self.inner
            .lock()
            .map(|inner| inner.active && inner.session_id == session_id)
            .unwrap_or(false)
    }
}

#[derive(Debug, Deserialize)]
struct CameraConfigDto {
    index: u32,
    width: u32,
    height: u32,
    fps: u32,
    #[serde(rename = "frameFormat")]
    frame_format: Option<String>,
}

impl From<CameraConfigDto> for CameraConfig {
    fn from(value: CameraConfigDto) -> Self {
        let default = CameraConfig::default();
        Self {
            index: value.index,
            width: value.width,
            height: value.height,
            fps: value.fps,
            frame_format: value
                .frame_format
                .as_deref()
                .and_then(|format| frame_format_from_str(format).ok())
                .unwrap_or(default.frame_format),
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct CameraDeviceDto {
    index: String,
    name: String,
    description: String,
}

#[derive(Clone, Debug, Serialize)]
struct CameraFormatDto {
    width: u32,
    height: u32,
    fps: u32,
    #[serde(rename = "frameFormat")]
    frame_format: String,
}

#[derive(Debug, Serialize)]
struct FrameResponse {
    width: u32,
    height: u32,
    seq: u64,
    #[serde(rename = "frameUrl")]
    frame_url: String,
    statuses: Vec<CameraStatusDto>,
    events: Vec<CameraEventDto>,
}

#[derive(Debug, Serialize)]
struct CameraStreamInfoDto {
    #[serde(rename = "gridUrl")]
    grid_url: String,
    #[serde(rename = "slotUrls")]
    slot_urls: Vec<String>,
    width: u32,
    height: u32,
    statuses: Vec<CameraStatusDto>,
}

#[derive(Clone, Debug, Serialize)]
struct CameraStreamEventDto {
    kind: String,
    slot: Option<usize>,
    index: Option<u32>,
    seq: Option<u64>,
    width: Option<u32>,
    height: Option<u32>,
    fps: Option<f64>,
    #[serde(rename = "captureMs")]
    capture_ms: Option<u128>,
    #[serde(rename = "encodeMs")]
    encode_ms: Option<u128>,
    message: Option<String>,
    statuses: Vec<CameraStatusDto>,
}

#[derive(Clone, Debug, Serialize)]
struct CameraStatusDto {
    slot: usize,
    index: u32,
    connected: bool,
    message: String,
}

#[derive(Debug, Serialize)]
struct CameraEventDto {
    slot: usize,
    index: u32,
    kind: String,
    message: String,
}

#[derive(Debug, Serialize)]
struct CameraControlDto {
    id: String,
    name: String,
    kind: String,
    value: f64,
    default: f64,
    min: Option<f64>,
    max: Option<f64>,
    step: Option<f64>,
    active: bool,
    flags: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct RoiDto {
    x: u32,
    y: u32,
    width: u32,
    height: u32,
}

impl From<RoiDto> for Roi {
    fn from(value: RoiDto) -> Self {
        Self {
            x: value.x,
            y: value.y,
            width: value.width,
            height: value.height,
        }
    }
}

fn status_dto(status: &CameraSlotStatus) -> CameraStatusDto {
    CameraStatusDto {
        slot: status.slot,
        index: status.index,
        connected: status.connected,
        message: status.message.clone(),
    }
}

/// 在 grid RGB buffer 上直接画 54 个 ROI 矩形 + 编号文字。
/// 当前选中的 ROI 用蓝色描边，其余用绿色——和前端 SVG 配色一致。
/// 文字（如 "U1".."B9"）画在矩形上方，白色填充 + 黑色 1px 轮廓便于在
/// 任意背景上可读。
///
/// 为了画面清晰且性能好，矩形使用 2px 描边、文字用内置 5x7 bitmap 字体
/// 放大 2 倍（10x14），纯像素操作不抗锯齿；54 个 ROI 总写入 < 1ms。
fn draw_overlay_rois(frame: &mut Frame, overlay: &RoiOverlayState) {
    let width = frame.width as i32;
    let height = frame.height as i32;
    if width == 0 || height == 0 {
        return;
    }

    const STROKE_PX: i32 = 2;
    const ACTIVE_RGB: [u8; 3] = [37, 99, 235]; // 蓝色（与前端 .roi-rect.is-active 一致）
    const NORMAL_RGB: [u8; 3] = [19, 148, 71]; // 绿色（与前端 .roi-rect 一致）
    const TEXT_RGB: [u8; 3] = [255, 255, 255];
    const TEXT_OUTLINE_RGB: [u8; 3] = [15, 23, 42]; // 接近黑（前端 .roi-label 描边色）
    const TEXT_SCALE: i32 = 2; // 5x7 → 10x14 像素

    for (idx, opt_roi) in overlay.rois.iter().enumerate() {
        let Some(roi) = opt_roi else { continue };
        let x0 = (roi.x * width as f32).round() as i32;
        let y0 = (roi.y * height as f32).round() as i32;
        let x1 = ((roi.x + roi.w) * width as f32).round() as i32;
        let y1 = ((roi.y + roi.h) * height as f32).round() as i32;
        if x1 <= x0 || y1 <= y0 {
            continue;
        }
        let color = if Some(idx) == overlay.current {
            ACTIVE_RGB
        } else {
            NORMAL_RGB
        };
        draw_rect_stroke(
            &mut frame.rgb,
            width,
            height,
            x0,
            y0,
            x1 - 1,
            y1 - 1,
            STROKE_PX,
            color,
        );

        if !roi.label.is_empty() {
            // 文字在矩形上方居中：每字符宽 5*scale + 1 间距
            let char_w = BITMAP_FONT_WIDTH as i32 * TEXT_SCALE;
            let char_h = BITMAP_FONT_HEIGHT as i32 * TEXT_SCALE;
            let spacing = TEXT_SCALE; // 字符间隔 1*scale 像素
            let text_w = roi.label.chars().count() as i32 * char_w
                + (roi.label.chars().count() as i32 - 1).max(0) * spacing;
            let mut text_x = x0 + (x1 - x0) / 2 - text_w / 2;
            // 优先放在矩形上方；空间不够则放下方。
            let mut text_y = y0 - char_h - 2;
            if text_y < 0 {
                text_y = y1 + 2;
            }
            // 边界裁剪由 draw_bitmap_char 内部处理；这里直接画。
            draw_text(
                &mut frame.rgb,
                width,
                height,
                text_x.max(0),
                text_y.max(0),
                &roi.label,
                TEXT_SCALE,
                TEXT_RGB,
                Some(TEXT_OUTLINE_RGB),
            );
            // 静默 unused warning
            let _ = &mut text_x;
            let _ = &mut text_y;
        }
    }
}

/// 在 RGB buffer 上画一个矩形描边（实心边框，宽度 stroke_px）。
fn draw_rect_stroke(
    rgb: &mut [u8],
    width: i32,
    height: i32,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
    stroke_px: i32,
    color: [u8; 3],
) {
    let stroke = stroke_px.max(1);
    let cx0 = x0.max(0);
    let cy0 = y0.max(0);
    let cx1 = x1.min(width - 1);
    let cy1 = y1.min(height - 1);
    if cx0 > cx1 || cy0 > cy1 {
        return;
    }
    // 上下两条横边
    for t in 0..stroke {
        let y_top = (y0 + t).clamp(0, height - 1);
        let y_bot = (y1 - t).clamp(0, height - 1);
        for y in [y_top, y_bot] {
            let row_start = (y * width * 3) as usize;
            for x in cx0..=cx1 {
                let off = row_start + (x as usize) * 3;
                rgb[off] = color[0];
                rgb[off + 1] = color[1];
                rgb[off + 2] = color[2];
            }
        }
    }
    // 左右两条竖边
    for t in 0..stroke {
        let x_left = (x0 + t).clamp(0, width - 1);
        let x_right = (x1 - t).clamp(0, width - 1);
        for x in [x_left, x_right] {
            for y in cy0..=cy1 {
                let off = ((y * width + x) * 3) as usize;
                rgb[off] = color[0];
                rgb[off + 1] = color[1];
                rgb[off + 2] = color[2];
            }
        }
    }
}

// ─── 5x7 bitmap 字体（仅 ROI 标签需要的字符） ────────────────────
//
// 每个字符 5 列 × 7 行，按 row-major 编码为 7 个 u8（取低 5 位）。
// 1 = 前景像素，0 = 背景。这是 ROI 标签 "U1".."B9" 用到的字符集（字母 6 个 + 数字 9 个）。
// 自实现避免引入 imageproc / ab_glyph 等大依赖；54 个标签每帧绘制 < 1ms。

const BITMAP_FONT_WIDTH: usize = 5;
const BITMAP_FONT_HEIGHT: usize = 7;

fn bitmap_glyph(c: char) -> Option<&'static [u8; BITMAP_FONT_HEIGHT]> {
    Some(match c {
        // 字母（仅 ROI face 标签用到的）
        'B' => &[0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110],
        'D' => &[0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110],
        'F' => &[0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000],
        'L' => &[0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111],
        'R' => &[0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001],
        'U' => &[0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        // 数字
        '0' => &[0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110],
        '1' => &[0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        '2' => &[0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111],
        '3' => &[0b11111, 0b00010, 0b00100, 0b00010, 0b00001, 0b10001, 0b01110],
        '4' => &[0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
        '5' => &[0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
        '6' => &[0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
        '7' => &[0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
        '8' => &[0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
        '9' => &[0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100],
        _ => return None,
    })
}

/// 在 (x, y) 处绘制一个字符（左上角为基准点），放大 `scale` 倍。
/// outline 提供时，先在四周一圈画轮廓色再叠主色，文字在任意背景都可读。
#[allow(clippy::too_many_arguments)]
fn draw_bitmap_char(
    rgb: &mut [u8],
    width: i32,
    height: i32,
    x: i32,
    y: i32,
    c: char,
    scale: i32,
    color: [u8; 3],
    outline: Option<[u8; 3]>,
) {
    let Some(glyph) = bitmap_glyph(c) else {
        return;
    };
    let scale = scale.max(1);

    let put = |rgb: &mut [u8], px: i32, py: i32, color: [u8; 3]| {
        if px < 0 || py < 0 || px >= width || py >= height {
            return;
        }
        let off = ((py * width + px) * 3) as usize;
        rgb[off] = color[0];
        rgb[off + 1] = color[1];
        rgb[off + 2] = color[2];
    };

    // 先画轮廓（在每个前景像素的 8 邻居方向画一遍 outline），再画主色。
    if let Some(out_color) = outline {
        for (row, bits) in glyph.iter().enumerate() {
            for col in 0..BITMAP_FONT_WIDTH {
                let bit = (*bits >> (BITMAP_FONT_WIDTH - 1 - col)) & 1;
                if bit == 0 {
                    continue;
                }
                let bx = x + col as i32 * scale;
                let by = y + row as i32 * scale;
                for sy in 0..scale {
                    for sx in 0..scale {
                        for dy in -1..=1 {
                            for dx in -1..=1 {
                                put(rgb, bx + sx + dx, by + sy + dy, out_color);
                            }
                        }
                    }
                }
            }
        }
    }
    for (row, bits) in glyph.iter().enumerate() {
        for col in 0..BITMAP_FONT_WIDTH {
            let bit = (*bits >> (BITMAP_FONT_WIDTH - 1 - col)) & 1;
            if bit == 0 {
                continue;
            }
            let bx = x + col as i32 * scale;
            let by = y + row as i32 * scale;
            for sy in 0..scale {
                for sx in 0..scale {
                    put(rgb, bx + sx, by + sy, color);
                }
            }
        }
    }
}

/// 在 (x, y) 处绘制一行文本（仅支持 bitmap_glyph 中已定义的字符）。
#[allow(clippy::too_many_arguments)]
fn draw_text(
    rgb: &mut [u8],
    width: i32,
    height: i32,
    x: i32,
    y: i32,
    text: &str,
    scale: i32,
    color: [u8; 3],
    outline: Option<[u8; 3]>,
) {
    let mut cursor = x;
    let char_w = BITMAP_FONT_WIDTH as i32 * scale;
    let spacing = scale; // 字符间隙 1*scale 像素
    for ch in text.chars() {
        draw_bitmap_char(rgb, width, height, cursor, y, ch, scale, color, outline);
        cursor += char_w + spacing;
    }
}

fn encode_frame_jpeg(frame: &Frame) -> Result<Vec<u8>> {
    // 直接用底层 encode 接口，避免 RgbImage::from_raw 需要的整张 rgb 数据 clone
    // （1280x960x3 ≈ 3.5MB/帧，省一次大块内存拷贝）。质量 60 对预览足够，
    // 比 quality=72 快约 25%、字节少约 30%，画质肉眼几乎看不出差别。
    let mut bytes = Cursor::new(Vec::with_capacity(frame.rgb.len() / 8));
    JpegEncoder::new_with_quality(&mut bytes, 60)
        .encode(
            &frame.rgb,
            frame.width,
            frame.height,
            ExtendedColorType::Rgb8,
        )
        .context("failed to encode stream frame as JPEG")?;
    Ok(bytes.into_inner())
}

fn should_log_diagnostic(last_log_at: &mut Option<Instant>, now: Instant, warning: bool) -> bool {
    let should_log = warning
        || last_log_at
            .map(|last| now.duration_since(last) >= DIAGNOSTIC_LOG_INTERVAL)
            .unwrap_or(true);
    if should_log {
        *last_log_at = Some(now);
    }
    should_log
}

fn should_log_throttled_diagnostic(last_log_at: &mut Option<Instant>, now: Instant) -> bool {
    let should_log = last_log_at
        .map(|last| now.duration_since(last) >= DIAGNOSTIC_LOG_INTERVAL)
        .unwrap_or(true);
    if should_log {
        *last_log_at = Some(now);
    }
    should_log
}

fn diagnostic_level(warning: bool) -> &'static str {
    if warning {
        "warn"
    } else {
        "info"
    }
}

fn diagnostic_duration_ms(duration: Option<Duration>) -> String {
    duration
        .map(|duration| duration.as_millis().to_string())
        .unwrap_or_else(|| "na".to_string())
}

fn publish_stream_packet(
    hub: &FrameHub,
    session_id: u64,
    app: &tauri::AppHandle,
    packet: FramePacket,
) -> Result<()> {
    // 不再每帧 encode 单 tile JPEG —— 前端走 /grid.mjpeg 由 grid_timer 统一编码。
    // 4 路 × ~22fps × 3ms encode = ~265ms/s（>1/4 核）的 CPU 浪费，省掉。
    // 单 tile JPEG 留空，/slot/N.mjpeg endpoint 暂不可用（无需求时不再发负担）。
    let slot = packet.slot;
    let index = packet.index;
    let seq = packet.seq;
    let Some(frame) = hub.publish_slot_frame(session_id, packet, Vec::new(), 0)? else {
        return Ok(());
    };
    // 把"frame"事件 emit 到前端的频率限制到 1Hz：
    // 4 路 worker 并发产帧时，每帧都 emit 会让前端 React 状态栏触发整个 App
    // 重渲染（包括 ROI SVG 等无关内容）；MJPEG <img> 显示和 setFrameStats
    // 没关系，UI 状态栏 1s 更新一次完全够用。
    const FRAME_EVENT_EMIT_INTERVAL: Duration = Duration::from_millis(1000);
    let should_emit = {
        match hub.inner.lock() {
            Ok(mut inner) => {
                let now = Instant::now();
                let due = match inner.last_frame_event_emit_at {
                    Some(prev) => now.duration_since(prev) >= FRAME_EVENT_EMIT_INTERVAL,
                    None => true,
                };
                if due {
                    inner.last_frame_event_emit_at = Some(now);
                }
                due
            }
            Err(_) => false,
        }
    };
    if !should_emit {
        return Ok(());
    }

    let fps = if frame.capture_ms == 0 {
        None
    } else {
        Some(1000.0 / frame.capture_ms as f64)
    };
    let _ = app.emit(
        "camera-stream-event",
        CameraStreamEventDto {
            kind: "frame".to_string(),
            slot: Some(slot),
            index: Some(index),
            seq: Some(seq),
            width: Some(frame.width),
            height: Some(frame.height),
            fps,
            capture_ms: Some(frame.capture_ms),
            encode_ms: Some(frame.encode_ms),
            message: None,
            statuses: hub.statuses().unwrap_or_default(),
        },
    );
    Ok(())
}

fn compose_grid_frame(
    frames: Vec<Option<Arc<Frame>>>,
    tile_width: u32,
    tile_height: u32,
    columns: u32,
) -> Result<Option<Frame>> {
    if frames.is_empty() || tile_width == 0 || tile_height == 0 || columns == 0 {
        return Ok(None);
    }
    let rows = (frames.len() as u32 + columns - 1) / columns;
    let output_width = tile_width * columns;
    let output_height = tile_height * rows;
    let mut output = vec![0u8; output_width as usize * output_height as usize * 3];

    for (idx, frame) in frames.into_iter().enumerate() {
        let tile_x = (idx as u32 % columns) * tile_width;
        let tile_y = (idx as u32 / columns) * tile_height;
        match frame {
            Some(frame) => blit_fit_rgb(
                &frame,
                &mut output,
                output_width,
                tile_width,
                tile_height,
                tile_x,
                tile_y,
            ),
            None => fill_tile(
                &mut output,
                output_width,
                tile_width,
                tile_height,
                tile_x,
                tile_y,
                12,
            ),
        }
    }

    Frame::new_rgb(output_width, output_height, output)
        .map(Some)
        .context("failed to build grid stream frame")
}

fn blit_fit_rgb(
    src: &Frame,
    dst: &mut [u8],
    dst_width: u32,
    tile_width: u32,
    tile_height: u32,
    dst_x: u32,
    dst_y: u32,
) {
    // 与 RobotApp 一致：直接拉伸到 tile 尺寸，**允许变形**，不做 letterbox。
    // 这样无论各路相机原生分辨率/比例如何，拼接画布始终是固定布局，
    // ROI 坐标永远对得上。
    if src.width == tile_width && src.height == tile_height {
        blit_rgb(src, dst, dst_width, dst_x, dst_y);
        return;
    }

    let Some(image) = RgbImage::from_raw(src.width, src.height, src.rgb.clone()) else {
        fill_tile(dst, dst_width, tile_width, tile_height, dst_x, dst_y, 12);
        return;
    };
    let resized = imageops::resize(
        &image,
        tile_width,
        tile_height,
        imageops::FilterType::Triangle,
    );
    let Ok(frame) = Frame::new_rgb(tile_width, tile_height, resized.into_raw()) else {
        fill_tile(dst, dst_width, tile_width, tile_height, dst_x, dst_y, 12);
        return;
    };
    blit_rgb(&frame, dst, dst_width, dst_x, dst_y);
}

fn blit_rgb(src: &Frame, dst: &mut [u8], dst_width: u32, dst_x: u32, dst_y: u32) {
    let row_bytes = src.width as usize * 3;
    for y in 0..src.height {
        let src_start = y as usize * row_bytes;
        let dst_start = ((dst_y + y) * dst_width * 3 + dst_x * 3) as usize;
        dst[dst_start..dst_start + row_bytes]
            .copy_from_slice(&src.rgb[src_start..src_start + row_bytes]);
    }
}

fn fill_tile(
    dst: &mut [u8],
    dst_width: u32,
    width: u32,
    height: u32,
    dst_x: u32,
    dst_y: u32,
    value: u8,
) {
    for y in 0..height {
        let start = ((dst_y + y) * dst_width * 3 + dst_x * 3) as usize;
        let end = start + width as usize * 3;
        dst[start..end].fill(value);
    }
}

#[derive(Debug, Serialize)]
struct SerialPortDto {
    name: String,
    port_type: String,
}

#[derive(Debug, Serialize)]
struct SerialReadResponse {
    text: String,
    motion_finished: bool,
    param_write_ok: bool,
    param_write_error: bool,
}

enum SerialRuntime {
    Real(SerialTransport),
    Mock(MockSerialTransport),
}

impl SerialRuntime {
    /// 直接发送已编码的下位机字符串（前端已用 user digit_map 编码好）。
    fn send_encoded(&mut self, encoded: &str) -> Result<()> {
        match self {
            Self::Real(transport) => transport.send_encoded(encoded),
            Self::Mock(transport) => transport.send_encoded(encoded),
        }
    }

    fn read_available(&mut self) -> Result<Vec<u8>> {
        match self {
            Self::Real(transport) => transport.read_available(),
            Self::Mock(transport) => transport.read_available(),
        }
    }
}

#[derive(Default)]
struct MockSerialTransport {
    last_payload: Option<String>,
    responses: VecDeque<Vec<u8>>,
}

impl MockSerialTransport {
    fn read_available(&mut self) -> Result<Vec<u8>> {
        Ok(self.responses.pop_front().unwrap_or_default())
    }

    fn send_encoded(&mut self, encoded: &str) -> Result<()> {
        // Mock transport：把发出去的内容存下来供测试断言用，不实际发硬件
        self.last_payload = Some(encoded.to_string());
        self.responses.push_back(b"OK\n".to_vec());
        self.responses.push_back(b"ND\n".to_vec());
        Ok(())
    }

    #[cfg(test)]
    fn last_payload(&self) -> Option<&str> {
        self.last_payload.as_deref()
    }
}

#[derive(Debug, Serialize)]
struct SolveFaceletsResponse {
    facelets: String,
    /// Kociemba face 序列（如 "R2 F' D2 ..."）
    moves: Vec<String>,
    /// 机械步骤助记符（M_L1 / M_R3 / ...），按 user digit_map 重映射后的顺序
    steps: Vec<String>,
    /// 机械步骤的最终编码字符串（按 user digit_map）
    encoded_steps: String,
    /// solver（Search.solutions） 阶段耗时（毫秒）
    search_elapsed_ms: u64,
    /// handstep（候选并行翻译为机械步骤） 阶段耗时（毫秒）
    handstep_elapsed_ms: u64,
    /// 最终选中候选的机械步数（pipeline 已按此最小化）
    mech_steps: i32,
    /// solver 产出的候选数量（≥1）
    candidate_count: usize,
}

const DISCOVERY_CACHE_TTL: Duration = Duration::from_secs(5);

fn format_diagnostic_log(message: &str) -> String {
    format!(
        "[diagnostic] {}",
        message.replace('\r', "\\r").replace('\n', "\\n")
    )
}

#[tauri::command]
fn diagnostic_log(message: String) -> Result<(), String> {
    log::info!("{}", format_diagnostic_log(&message));
    Ok(())
}

#[derive(Debug, Serialize)]
struct DefaultSavePaths {
    /// 软件安装目录（用作 dialog.save() 的 defaultPath 起点）
    install_dir: String,
    /// ROI 默认文件名（用作 defaultPath 末段）
    roi_filename: String,
    /// 步骤映射默认文件名
    move_mapping_filename: String,
}

/// 返回前端弹"保存"对话框时用的默认目录与默认文件名。
///
/// install_dir 来自 `app_install_dir()`（macOS .app bundle 同级目录 / 其它平台
/// 是可执行文件所在目录），roi_filename / move_mapping_filename 与启动时自动加载
/// 用的文件名一致——用户保存到这个目录下的同名文件，下次启动会自动读回。
#[tauri::command]
fn get_default_save_paths() -> Result<DefaultSavePaths, String> {
    Ok(DefaultSavePaths {
        install_dir: app_install_dir()?.display().to_string(),
        roi_filename: DEFAULT_ROI_FILENAME.to_string(),
        move_mapping_filename: MOVE_MAPPING_FILENAME.to_string(),
    })
}

/// 把任意文本内容写到指定的绝对路径（路径由前端 `dialog.save()` 选好）。
///
/// 用于保存 ROI 等用户文件。如果用户把它保存到默认安装目录下的标准文件名
/// （如 `app_install_dir/robot-roi.json`），启动时会被 `load_default_roi`
/// 自动读回；保存到其它路径仅作导出/备份。
#[tauri::command]
fn save_text_file_to_path(
    path: String,
    contents: String,
) -> Result<String, String> {
    let target = std::path::PathBuf::from(&path);
    if target.as_os_str().is_empty() {
        return Err("路径为空。".to_string());
    }
    if let Some(parent) = target.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).map_err(|err| err.to_string())?;
        }
    }
    std::fs::write(&target, contents).map_err(|err| err.to_string())?;
    log::info!("文件已保存: {}", target.display());
    Ok(target.display().to_string())
}

#[tauri::command]
fn list_cameras(state: tauri::State<'_, AppState>) -> Result<Vec<CameraDeviceDto>, String> {
    if state.mode.mock_camera {
        return Ok(mock_camera_devices());
    }

    let now = Instant::now();
    if let Ok(cache) = state.discovery_cache.lock() {
        if let Some(cached) = cache
            .cameras
            .as_ref()
            .filter(|cached| cached.expires_at > now)
        {
            return Ok(cached.value.clone());
        }
    }

    let devices: Vec<CameraDeviceDto> = robo_camera::list_cameras()
        .map(|devices| {
            devices
                .into_iter()
                .map(|device| CameraDeviceDto {
                    index: device.index,
                    name: device.name,
                    description: device.description,
                })
                .collect()
        })
        .map_err(|err| err.to_string())?;

    if let Ok(mut cache) = state.discovery_cache.lock() {
        cache.cameras = Some(CachedValue {
            value: devices.clone(),
            expires_at: now + DISCOVERY_CACHE_TTL,
        });
    }
    Ok(devices)
}

#[tauri::command]
fn list_camera_formats(
    state: tauri::State<'_, AppState>,
    index: u32,
) -> Result<Vec<CameraFormatDto>, String> {
    if state.mode.mock_camera {
        return Ok(mock_camera_formats(index));
    }

    let now = Instant::now();
    if let Ok(cache) = state.discovery_cache.lock() {
        if let Some(cached) = cache
            .formats
            .get(&index)
            .filter(|cached| cached.expires_at > now)
        {
            return Ok(cached.value.clone());
        }
    }

    let formats: Vec<CameraFormatDto> = robo_camera::list_camera_formats(index)
        .map(|formats| {
            formats
                .into_iter()
                .map(|format| CameraFormatDto {
                    width: format.width,
                    height: format.height,
                    fps: format.fps,
                    frame_format: format.frame_format,
                })
                .collect()
        })
        .map_err(|err| err.to_string())?;

    if let Ok(mut cache) = state.discovery_cache.lock() {
        cache.formats.insert(
            index,
            CachedValue {
                value: formats.clone(),
                expires_at: now + DISCOVERY_CACHE_TTL,
            },
        );
    }
    Ok(formats)
}

#[tauri::command]
fn open_cameras(
    state: tauri::State<'_, AppState>,
    configs: Vec<CameraConfigDto>,
) -> Result<(), String> {
    let configs = configs
        .into_iter()
        .map(CameraConfig::from)
        .collect::<Vec<_>>();
    if state.mode.mock_camera {
        return state
            .mock_camera
            .lock()
            .map_err(|_| "mock camera state is poisoned".to_string())?
            .open(configs)
            .map_err(|err| err.to_string());
    }

    state
        .camera
        .lock()
        .map_err(|_| "camera state is poisoned".to_string())?
        .open(configs)
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn close_cameras(state: tauri::State<'_, AppState>) -> Result<(), String> {
    if state.mode.mock_camera {
        state
            .mock_camera
            .lock()
            .map_err(|_| "mock camera state is poisoned".to_string())?
            .close();
        return Ok(());
    }

    state
        .camera
        .lock()
        .map_err(|_| "camera state is poisoned".to_string())?
        .close()
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn open_camera_stream(
    app: tauri::AppHandle,
    state: tauri::State<'_, AppState>,
    configs: Vec<CameraConfigDto>,
) -> Result<CameraStreamInfoDto, String> {
    let configs = configs
        .into_iter()
        .map(CameraConfig::from)
        .collect::<Vec<_>>();
    let mut runtime = state
        .camera_stream
        .lock()
        .map_err(|_| "camera stream state is poisoned".to_string())?;
    if let Some(existing) = runtime.as_mut() {
        existing.close();
    }
    *runtime = None;
    let overlay = Arc::clone(&state.overlay);
    let next_runtime = match if state.mode.mock_camera {
        CameraStreamRuntime::open_mock(configs, Arc::clone(&state.stream_hub), app, overlay)
    } else {
        CameraStreamRuntime::open(configs, Arc::clone(&state.stream_hub), app, overlay)
    } {
        Ok(runtime) => runtime,
        Err(err) => {
            let _ = state.stream_hub.clear();
            return Err(err.to_string());
        }
    };
    *runtime = Some(next_runtime);
    state
        .stream_hub
        .stream_info(state.frame_server_port)
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn close_camera_stream(state: tauri::State<'_, AppState>) -> Result<(), String> {
    if let Some(mut runtime) = state
        .camera_stream
        .lock()
        .map_err(|_| "camera stream state is poisoned".to_string())?
        .take()
    {
        runtime.close();
    }
    state.stream_hub.clear().map_err(|err| err.to_string())
}

#[tauri::command]
fn camera_stream_info(state: tauri::State<'_, AppState>) -> Result<CameraStreamInfoDto, String> {
    state
        .stream_hub
        .stream_info(state.frame_server_port)
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn snapshot_frame(state: tauri::State<'_, AppState>) -> Result<tauri::ipc::Response, String> {
    let (frame, grid_seq, age, should_log) = {
        let mut inner = state
            .stream_hub
            .inner
            .lock()
            .map_err(|_| "stream hub is poisoned".to_string())?;
        let now = Instant::now();
        let Some(grid) = inner.grid.as_ref() else {
            return Err("no stream frame available".to_string());
        };
        let frame = (*grid.jpeg).clone();
        let grid_seq = grid.seq;
        let age = now.duration_since(grid.created_at);
        let should_log =
            should_log_throttled_diagnostic(&mut inner.last_snapshot_diagnostic_log_at, now);
        (frame, grid_seq, age, should_log)
    };
    if should_log {
        let warning = age > DIAGNOSTIC_WARN_THRESHOLD;
        log::info!(
            "{BACKEND_DIAGNOSTIC_PREFIX} category=snapshot_frame level={} grid_seq={} age_ms={}",
            diagnostic_level(warning),
            grid_seq,
            age.as_millis()
        );
    }
    Ok(tauri::ipc::Response::new(frame))
}

#[tauri::command]
fn capture_frame(state: tauri::State<'_, AppState>) -> Result<FrameResponse, String> {
    let capture = capture_from_state(&state).map_err(|err| err.to_string())?;
    encode_capture(capture, &state).map_err(|err| err.to_string())
}

/// 拿一帧 JPEG 并 base64 编码成 data URL，给前端"保存图片"等场景使用。
///
/// 优先级：
/// 1. **stream_hub 的最新 grid JPEG**：相机以 MJPEG 流模式跑时（开 camera_stream），
///    grid 帧由 aggregator 实时刷新；这条路径完全不需要去抢相机。
/// 2. fallback 到 `AppState.latest_frame`：旧的 `capture_frame` 命令同步抓帧后会
///    写到这里。当用户手动 `capture_frame` 而不是流式预览时仍可用。
/// 3. 两者都没有 → `no camera frame available`。
///
/// 历史上只看 `latest_frame` 字段，导致流式预览模式下永远报 "no camera frame
/// available"——因为流路径不写这个字段。
#[tauri::command]
fn latest_frame_data_url(state: tauri::State<'_, AppState>) -> Result<String, String> {
    // 1) 先看 stream_hub.grid.jpeg（流式预览的实时最新帧，零开销 clone Arc<Vec<u8>>）
    if let Ok(inner) = state.stream_hub.inner.lock() {
        if let Some(grid) = inner.grid.as_ref() {
            return Ok(format!(
                "data:image/jpeg;base64,{}",
                STANDARD.encode(grid.jpeg.as_slice())
            ));
        }
    }

    // 2) fallback：旧 capture_frame 写入的 latest_frame
    let frame = state
        .latest_frame
        .lock()
        .map_err(|_| "latest frame state is poisoned".to_string())?;
    let frame = frame
        .as_ref()
        .ok_or_else(|| "no camera frame available".to_string())?;
    Ok(format!(
        "data:image/jpeg;base64,{}",
        STANDARD.encode(&frame.bytes)
    ))
}

#[tauri::command]
fn list_camera_controls(
    state: tauri::State<'_, AppState>,
    slot: usize,
) -> Result<Vec<CameraControlDto>, String> {
    // 路由：
    // - mock 模式：走 mock_camera（不依赖任何硬件状态）；
    // - 真机 + 流式预览运行中：走 CameraStreamRuntime（CameraSlotWorker 间隙处理），
    //   不需要关闭相机就能读到当前硬件值；
    // - 真机 + 仅 capture_frame 模式：走旧 CameraWorker。
    let controls = if state.mode.mock_camera {
        state
            .mock_camera
            .lock()
            .map_err(|_| "mock camera state is poisoned".to_string())?
            .controls(slot)
    } else if let Some(runtime) = state
        .camera_stream
        .lock()
        .map_err(|_| "camera stream state is poisoned".to_string())?
        .as_ref()
    {
        runtime.controls(slot)
    } else {
        state
            .camera
            .lock()
            .map_err(|_| "camera state is poisoned".to_string())?
            .controls(slot)
    }
    .map_err(|err| err.to_string())?;
    Ok(controls
        .into_iter()
        .map(|control| CameraControlDto {
            id: control.id,
            name: control.name,
            kind: match control.kind {
                CameraControlKind::Integer => "integer".to_string(),
                CameraControlKind::Float => "float".to_string(),
                CameraControlKind::Boolean => "boolean".to_string(),
            },
            value: control.value,
            default: control.default,
            min: control.min,
            max: control.max,
            step: control.step,
            active: control.active,
            flags: control.flags,
        })
        .collect())
}

#[tauri::command]
fn set_camera_control(
    state: tauri::State<'_, AppState>,
    slot: usize,
    id: String,
    value: f64,
) -> Result<(), String> {
    // 路由同 list_camera_controls：流式预览运行时直接通过 worker 的 control channel
    // 写参数，与 capture loop 串行；不再要求用户先关相机。
    if state.mode.mock_camera {
        return state
            .mock_camera
            .lock()
            .map_err(|_| "mock camera state is poisoned".to_string())?
            .set_control(slot, id, value)
            .map_err(|err| err.to_string());
    }
    if let Some(runtime) = state
        .camera_stream
        .lock()
        .map_err(|_| "camera stream state is poisoned".to_string())?
        .as_ref()
    {
        return runtime
            .set_control(slot, id, value)
            .map_err(|err| err.to_string());
    }
    state
        .camera
        .lock()
        .map_err(|_| "camera state is poisoned".to_string())?
        .set_control(slot, id, value)
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn solve_current_frame(
    state: tauri::State<'_, AppState>,
    rois: Vec<RoiDto>,
) -> Result<SolveFaceletsResponse, String> {
    state.solver.get().ok_or("solver 尚未初始化完成")?;
    let capture = capture_from_state(&state).map_err(|err| err.to_string())?;
    let rois = rois.into_iter().map(Roi::from).collect::<Vec<_>>();
    let recognizer = ColorClusterRecognizer;
    let face = recognizer
        .recognize(&capture.frame, &rois)
        .map_err(|err| err.to_string())?;
    solve_face(face, current_digit_map(&state), current_solver_timeout_ms(&state))
}

#[tauri::command]
fn solve_latest_frame(
    state: tauri::State<'_, AppState>,
    rois: Vec<RoiDto>,
) -> Result<SolveFaceletsResponse, String> {
    state.solver.get().ok_or("solver 尚未初始化完成")?;
    let frame = state
        .stream_hub
        .latest_grid_rgb()
        .map_err(|err| err.to_string())?;
    let rois = rois.into_iter().map(Roi::from).collect::<Vec<_>>();
    let recognizer = ColorClusterRecognizer;
    let face = recognizer
        .recognize(&frame, &rois)
        .map_err(|err| err.to_string())?;
    solve_face(face, current_digit_map(&state), current_solver_timeout_ms(&state))
}

#[tauri::command]
fn solve_image_file(
    state: tauri::State<'_, AppState>,
    image_data_url: String,
    rois: Vec<RoiDto>,
) -> Result<SolveFaceletsResponse, String> {
    state.solver.get().ok_or("solver 尚未初始化完成")?;
    let frame = decode_image_data_url(&image_data_url).map_err(|err| err.to_string())?;
    let rois = rois.into_iter().map(Roi::from).collect::<Vec<_>>();
    let recognizer = ColorClusterRecognizer;
    let face = recognizer
        .recognize(&frame, &rois)
        .map_err(|err| err.to_string())?;
    solve_face(face, current_digit_map(&state), current_solver_timeout_ms(&state))
}

#[tauri::command]
fn solve_facelets(state: tauri::State<'_, AppState>, facelets: String) -> Result<SolveFaceletsResponse, String> {
    state.solver.get().ok_or("solver 尚未初始化完成")?;
    let face = CubeFace::new(facelets).map_err(|err| err.to_string())?;
    solve_face(face, current_digit_map(&state), current_solver_timeout_ms(&state))
}

#[tauri::command]
fn list_serial_ports(state: tauri::State<'_, AppState>) -> Result<Vec<SerialPortDto>, String> {
    if state.mode.mock_serial {
        return Ok(vec![SerialPortDto {
            name: "ROBO_MOCK_SERIAL".to_string(),
            port_type: "Mock".to_string(),
        }]);
    }

    serialport::available_ports()
        .map(|ports| {
            ports
                .into_iter()
                .map(|port| SerialPortDto {
                    name: port.port_name,
                    port_type: format!("{:?}", port.port_type),
                })
                .collect()
        })
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn open_serial(
    state: tauri::State<'_, AppState>,
    port_name: String,
    baud_rate: u32,
) -> Result<(), String> {
    let transport = if state.mode.mock_serial {
        let _ = (port_name, baud_rate);
        SerialRuntime::Mock(MockSerialTransport::default())
    } else {
        SerialRuntime::Real(
            SerialTransport::open(&port_name, baud_rate).map_err(|err| err.to_string())?,
        )
    };
    *state
        .serial
        .lock()
        .map_err(|_| "serial state is poisoned".to_string())? = Some(transport);
    Ok(())
}

#[tauri::command]
fn close_serial(state: tauri::State<'_, AppState>) -> Result<(), String> {
    *state
        .serial
        .lock()
        .map_err(|_| "serial state is poisoned".to_string())? = None;
    Ok(())
}

/// 把已编码的下位机数字串（前端用 user digit_map 编码好）写到串口。
///
/// 历史上本命令叫 `commands` 收 mnemonic 列表，内部再用 digit_map 重新编码；
/// 现统一让前端把 `encoded_steps` 直接传过来——避免一处映射两处转换的对账风险，
/// 修复 `invalid args 'commands' for command 'send_steps'` 的不匹配问题。
#[tauri::command]
fn send_steps(
    state: tauri::State<'_, AppState>,
    encoded_steps: String,
) -> Result<(), String> {
    let mut serial = state
        .serial
        .lock()
        .map_err(|_| "serial state is poisoned".to_string())?;
    let transport = serial
        .as_mut()
        .ok_or_else(|| "serial port is not open".to_string())?;
    transport
        .send_encoded(&encoded_steps)
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn read_serial(state: tauri::State<'_, AppState>) -> Result<SerialReadResponse, String> {
    let mut serial = state
        .serial
        .lock()
        .map_err(|_| "serial state is poisoned".to_string())?;
    let transport = serial
        .as_mut()
        .ok_or_else(|| "serial port is not open".to_string())?;
    let bytes = transport.read_available().map_err(|err| err.to_string())?;
    let text = String::from_utf8_lossy(&bytes).to_string();
    Ok(SerialReadResponse {
        motion_finished: text.contains("ND"),
        param_write_ok: text.contains("OK"),
        param_write_error: text.contains("ER"),
        text,
    })
}

fn solve_face(
    face: CubeFace,
    digit_map: DigitMap,
    timeout_ms: u64,
) -> Result<SolveFaceletsResponse, String> {
    // 求解参数：solver_timeout_ms / max=∞ / slack=0
    // 实测 50 cube benchmark：mech 平均 68.9，solver 100ms 严格用满，
    // 比旧 300ms/max=8 配置 mech 持平且延迟降到 1/3；
    // timeout 由 app-config.json 提供（默认 100ms），可由前端 set_app_config 调整。
    let opts = SearchOptions {
        timeout: Duration::from_millis(timeout_ms.max(1)),
        max_solutions: usize::MAX,
        length_slack: 0,
        ..Default::default()
    };
    let res = translate_optimal(face.as_str(), opts).map_err(|err| err.to_string())?;

    // pipeline 输出 mnemonic 列表（语义层），用 user digit_map 编码出"展示串"
    // 给前端 UI 显示用；真正发到下位机时由 send_steps 重做编码（保证一致）。
    let steps: Vec<String> = res
        .best
        .mech_mnemonics
        .iter()
        .map(|s| s.to_string())
        .collect();
    let encoded_steps = robo_transport::encode_mnemonics(&steps, &digit_map);
    let moves: Vec<String> = res
        .best
        .kociemba
        .split_whitespace()
        .map(|s| s.to_string())
        .collect();

    log::info!(
        "solve: facelets={} → {}f kociemba, mech={} steps, solver={}ms / handstep={}ms ({} 候选, timed_out={})",
        face.as_str(),
        moves.len(),
        res.best.mech_steps,
        res.solver_elapsed.as_millis(),
        res.handstep_elapsed.as_millis(),
        res.candidates.len(),
        res.solver_timed_out,
    );

    Ok(SolveFaceletsResponse {
        facelets: face.into_string(),
        moves,
        steps,
        encoded_steps,
        search_elapsed_ms: res.solver_elapsed.as_millis() as u64,
        handstep_elapsed_ms: res.handstep_elapsed.as_millis() as u64,
        mech_steps: res.best.mech_steps,
        candidate_count: res.candidates.len(),
    })
}

fn current_digit_map(state: &AppState) -> DigitMap {
    state
        .digit_map
        .lock()
        .map(|guard| guard.clone())
        .unwrap_or_else(|_| default_digit_map())
}

fn current_solver_timeout_ms(state: &AppState) -> u64 {
    state
        .config
        .lock()
        .map(|guard| guard.solver_timeout_ms)
        .unwrap_or(DEFAULT_SOLVER_TIMEOUT_MS)
}

// ===== 应用配置：持久化 + Tauri 命令 =====

const APP_CONFIG_FILENAME: &str = "app-config.json";
/// solver 超时默认值；与历史硬编码保持一致（100ms）。
/// 写入 `app-config.json` 后由用户覆盖；不存在或解析失败时回退到此默认值。
const DEFAULT_SOLVER_TIMEOUT_MS: u64 = 100;

#[derive(Clone, Debug, Serialize, Deserialize)]
struct AppConfig {
    /// solver 单次求解的超时时间（毫秒）
    #[serde(default = "default_solver_timeout_ms_value")]
    solver_timeout_ms: u64,
}

fn default_solver_timeout_ms_value() -> u64 {
    DEFAULT_SOLVER_TIMEOUT_MS
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            solver_timeout_ms: DEFAULT_SOLVER_TIMEOUT_MS,
        }
    }
}

fn app_config_path() -> Result<std::path::PathBuf, String> {
    let dir = app_install_dir()?;
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    Ok(dir.join(APP_CONFIG_FILENAME))
}

fn load_app_config_from_disk() -> Option<AppConfig> {
    let path = app_config_path().ok()?;
    if !path.is_file() {
        return None;
    }
    let content = std::fs::read_to_string(&path).ok()?;
    match serde_json::from_str::<AppConfig>(&content) {
        Ok(cfg) => {
            log::info!("已加载应用配置: {}", path.display());
            Some(cfg)
        }
        Err(e) => {
            log::warn!("应用配置解析失败 ({}), 使用默认值", e);
            None
        }
    }
}

fn save_app_config_to_disk(cfg: &AppConfig) -> Result<std::path::PathBuf, String> {
    let path = app_config_path()?;
    let payload = serde_json::to_string_pretty(cfg).map_err(|e| e.to_string())?;
    std::fs::write(&path, payload).map_err(|e| e.to_string())?;
    log::info!("应用配置已保存: {}", path.display());
    Ok(path)
}

#[tauri::command]
fn get_app_config(state: tauri::State<'_, AppState>) -> Result<AppConfig, String> {
    Ok(state
        .config
        .lock()
        .map(|guard| guard.clone())
        .unwrap_or_default())
}

#[derive(Debug, Clone, serde::Deserialize)]
struct OverlayRoiDto {
    /// 0..54 的索引；缺省 / 越界则忽略此项。
    index: usize,
    x: f32,
    y: f32,
    w: f32,
    h: f32,
    /// ROI 标签（"U1".."B9"），由 grid_timer 画在矩形上方。
    label: String,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct OverlayRoisDto {
    /// 仅包含已标注的 ROI；未在列表中的索引视为未标注。
    rois: Vec<OverlayRoiDto>,
    /// 当前选中的 ROI 索引（用于高亮颜色）。
    current: Option<usize>,
    /// 是否启用 overlay；文件模式 / 关闭相机时前端传 false。
    enabled: bool,
}

/// 由前端在 ROI 状态变更时（标注 / 翻页 / 删除 / 重置 / 启用关闭）调用，
/// 写入 AppState.overlay；下一拍 grid_timer 会用最新数据画矩形。
#[tauri::command]
fn set_overlay_rois(
    state: tauri::State<'_, AppState>,
    payload: OverlayRoisDto,
) -> Result<(), String> {
    let mut rois: Vec<Option<NormRoi>> = vec![None; 54];
    for item in payload.rois {
        if item.index < 54 {
            rois[item.index] = Some(NormRoi {
                x: item.x.clamp(0.0, 1.0),
                y: item.y.clamp(0.0, 1.0),
                w: item.w.clamp(0.0, 1.0),
                h: item.h.clamp(0.0, 1.0),
                label: item.label,
            });
        }
    }
    let mut guard = state
        .overlay
        .lock()
        .map_err(|_| "overlay state is poisoned".to_string())?;
    *guard = RoiOverlayState {
        rois,
        current: payload.current.filter(|i| *i < 54),
        enabled: payload.enabled,
    };
    Ok(())
}

#[tauri::command]
fn set_app_config(
    state: tauri::State<'_, AppState>,
    config: AppConfig,
) -> Result<AppConfig, String> {
    let mut sanitized = config;
    if sanitized.solver_timeout_ms == 0 {
        sanitized.solver_timeout_ms = DEFAULT_SOLVER_TIMEOUT_MS;
    }
    {
        let mut guard = state
            .config
            .lock()
            .map_err(|_| "app config state is poisoned".to_string())?;
        *guard = sanitized.clone();
    }
    save_app_config_to_disk(&sanitized)?;
    Ok(sanitized)
}

// ===== 动作 → 数字映射：持久化 + Tauri 命令 =====

const MOVE_MAPPING_FILENAME: &str = "move_mapping.json";

#[derive(Debug, Serialize, Deserialize)]
struct MoveMappingDto {
    /// 助记符（只读，前端展示用）
    mnemonics: Vec<String>,
    /// 当前数字映射（10 个字符串，与 mnemonics 一一对应）
    digits: Vec<String>,
}

impl MoveMappingDto {
    fn from_map(map: &DigitMap) -> Self {
        Self {
            mnemonics: MNEMONICS.iter().map(|s| s.to_string()).collect(),
            digits: map.iter().cloned().collect(),
        }
    }
}

/// 返回"软件同目录"用于持久化用户配置。
///
/// 规则：
/// - 普通可执行文件：可执行文件所在目录。
/// - macOS `.app` 包：检测到路径形如 `…/Foo.app/Contents/MacOS/Foo`，
///   返回 `.app` 的同级目录（即用户在 Finder 看到 `CubeSolver.app` 的那个文件夹），
///   避免写入 `.app` 内部（签名/公证后只读，且对用户不可见）。
fn app_install_dir() -> Result<std::path::PathBuf, String> {
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    let exe_dir = exe
        .parent()
        .ok_or_else(|| "无法解析可执行文件父目录".to_string())?
        .to_path_buf();

    // macOS: …/Foo.app/Contents/MacOS/Foo  → 回到 Foo.app 的同级目录
    #[cfg(target_os = "macos")]
    {
        let mut anc = exe_dir.as_path();
        // exe_dir = …/Foo.app/Contents/MacOS
        if anc.file_name().and_then(|s| s.to_str()) == Some("MacOS") {
            if let Some(contents) = anc.parent() {
                if contents.file_name().and_then(|s| s.to_str()) == Some("Contents") {
                    if let Some(app_bundle) = contents.parent() {
                        if app_bundle
                            .extension()
                            .and_then(|s| s.to_str())
                            .map(|e| e.eq_ignore_ascii_case("app"))
                            .unwrap_or(false)
                        {
                            if let Some(beside) = app_bundle.parent() {
                                anc = beside;
                                return Ok(anc.to_path_buf());
                            }
                        }
                    }
                }
            }
        }
    }

    Ok(exe_dir)
}

fn move_mapping_path(_app: &tauri::AppHandle) -> Result<std::path::PathBuf, String> {
    let dir = app_install_dir()?;
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    Ok(dir.join(MOVE_MAPPING_FILENAME))
}

/// 校验：必须是 0-9 的一个排列。
fn validate_digit_map(digits: &[String]) -> Result<DigitMap, String> {
    if digits.len() != MOVE_COUNT {
        return Err(format!("映射长度必须为 {}，当前 {}", MOVE_COUNT, digits.len()));
    }
    let mut sorted: Vec<String> = digits.iter().map(|s| s.trim().to_string()).collect();
    let original = sorted.clone();
    sorted.sort();
    let expected: Vec<String> = (0..10).map(|i| i.to_string()).collect();
    if sorted != expected {
        return Err("映射必须为 0-9 各出现一次的排列".to_string());
    }
    let mut out: DigitMap = Default::default();
    for (i, s) in original.into_iter().enumerate() {
        out[i] = s;
    }
    Ok(out)
}

fn load_move_mapping_from_disk(app: &tauri::AppHandle) -> Option<DigitMap> {
    let path = move_mapping_path(app).ok()?;
    if !path.is_file() {
        return None;
    }
    let content = std::fs::read_to_string(&path).ok()?;
    let dto: MoveMappingDto = serde_json::from_str(&content).ok()?;
    match validate_digit_map(&dto.digits) {
        Ok(m) => {
            log::info!("已加载动作映射: {}", path.display());
            Some(m)
        }
        Err(e) => {
            log::warn!("动作映射文件校验失败 ({})，使用默认映射", e);
            None
        }
    }
}

#[tauri::command]
fn get_move_mapping(state: tauri::State<'_, AppState>) -> Result<MoveMappingDto, String> {
    Ok(MoveMappingDto::from_map(&current_digit_map(&state)))
}

/// 只更新内存中的 digit_map（校验后），不写文件。
///
/// 文件持久化由前端单独调 `save_move_mapping_to_path`（弹 dialog 选路径）。
/// 当用户保存到默认路径 `app_install_dir/move_mapping.json` 时，下次启动会自动
/// 读回（见 `load_move_mapping_from_disk`）；保存到其它路径仅作导出/备份。
#[tauri::command]
fn set_move_mapping(
    state: tauri::State<'_, AppState>,
    digits: Vec<String>,
) -> Result<MoveMappingDto, String> {
    let new_map = validate_digit_map(&digits)?;
    {
        let mut guard = state
            .digit_map
            .lock()
            .map_err(|_| "digit_map state is poisoned".to_string())?;
        *guard = new_map.clone();
    }
    Ok(MoveMappingDto::from_map(&new_map))
}

/// 把当前内存中的步骤映射序列化写到指定绝对路径。
///
/// 路径由前端 `dialog.save()` 选好；空路径或目录不存在视为错误。
/// 写入成功后返回最终落盘路径（与传入相同，便于日志展示）。
#[tauri::command]
fn save_move_mapping_to_path(
    state: tauri::State<'_, AppState>,
    path: String,
) -> Result<String, String> {
    let map = current_digit_map(&state);
    let dto = MoveMappingDto::from_map(&map);
    let payload = serde_json::to_string_pretty(&dto).map_err(|e| e.to_string())?;
    let target = std::path::PathBuf::from(&path);
    if let Some(parent) = target.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
    }
    std::fs::write(&target, payload).map_err(|e| e.to_string())?;
    log::info!("步骤映射已保存: {}", target.display());
    Ok(target.display().to_string())
}

#[tauri::command]
fn reset_move_mapping(
    app: tauri::AppHandle,
    state: tauri::State<'_, AppState>,
) -> Result<MoveMappingDto, String> {
    let default = default_digit_map();
    {
        let mut guard = state
            .digit_map
            .lock()
            .map_err(|_| "digit_map state is poisoned".to_string())?;
        *guard = default.clone();
    }
    if let Ok(path) = move_mapping_path(&app) {
        let _ = std::fs::remove_file(&path);
    }
    log::info!("动作映射已重置为默认");
    Ok(MoveMappingDto::from_map(&default))
}

fn decode_image_data_url(data_url: &str) -> Result<Frame> {
    let encoded = data_url
        .split_once(',')
        .map(|(_, data)| data)
        .unwrap_or(data_url);
    let bytes = STANDARD
        .decode(encoded)
        .context("failed to decode image data URL")?;
    let image = image::load_from_memory(&bytes)
        .context("failed to decode image file")?
        .to_rgb8();

    // 与相机拼接画布同尺寸。任意输入图片都强制拉伸到 1280x960（允许变形），
    // 让识别端的 ROI 坐标系与"实时相机帧"一致——保存的 robot-roi.json
    // 既适用于相机直出，也适用于回放任意分辨率的快照。
    let target_width = GRID_TILE_WIDTH * 2;
    let target_height = GRID_TILE_HEIGHT * 2;
    let normalized = if image.width() == target_width && image.height() == target_height {
        image
    } else {
        imageops::resize(
            &image,
            target_width,
            target_height,
            imageops::FilterType::Triangle,
        )
    };
    Frame::new_rgb(target_width, target_height, normalized.into_raw())
        .context("failed to build frame from image file")
}

fn capture_from_state(state: &AppState) -> Result<MultiCameraCapture> {
    if state.mode.mock_camera {
        state
            .mock_camera
            .lock()
            .map_err(|_| anyhow::anyhow!("mock camera state is poisoned"))?
            .capture()
    } else {
        state
            .camera
            .lock()
            .map_err(|_| anyhow::anyhow!("camera state is poisoned"))?
            .capture()
    }
}

fn encode_capture(capture: MultiCameraCapture, state: &AppState) -> Result<FrameResponse> {
    let frame = capture.frame;
    let width = frame.width;
    let height = frame.height;
    let image = RgbImage::from_raw(frame.width, frame.height, frame.rgb)
        .context("failed to create image from frame")?;
    let mut bytes = Cursor::new(Vec::new());
    JpegEncoder::new_with_quality(&mut bytes, 72)
        .encode_image(&image)
        .context("failed to encode frame as JPEG")?;
    let bytes = bytes.into_inner();
    let seq = {
        let mut seq = state
            .latest_frame_seq
            .lock()
            .map_err(|_| anyhow::anyhow!("latest frame seq is poisoned"))?;
        *seq += 1;
        *seq
    };
    *state
        .latest_frame
        .lock()
        .map_err(|_| anyhow::anyhow!("latest frame state is poisoned"))? =
        Some(LatestFrame { bytes });

    Ok(FrameResponse {
        width,
        height,
        seq,
        frame_url: if state.frame_server_port == 0 {
            String::new()
        } else {
            format!(
                "http://127.0.0.1:{}/frame.jpg?seq={seq}",
                state.frame_server_port
            )
        },
        statuses: capture
            .statuses
            .into_iter()
            .map(|status| CameraStatusDto {
                slot: status.slot,
                index: status.index,
                connected: status.connected,
                message: status.message,
            })
            .collect(),
        events: capture
            .events
            .into_iter()
            .map(|event| CameraEventDto {
                slot: event.slot,
                index: event.index,
                kind: match event.kind {
                    CameraStatusEventKind::Connected => "connected".to_string(),
                    CameraStatusEventKind::Disconnected => "disconnected".to_string(),
                },
                message: event.message,
            })
            .collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_image_data_url_to_rgb_frame() {
        let image = RgbImage::from_raw(2, 1, vec![255, 0, 0, 0, 255, 0])
            .expect("test image should be valid");
        let mut bytes = Cursor::new(Vec::new());
        JpegEncoder::new_with_quality(&mut bytes, 90)
            .encode_image(&image)
            .expect("test image should encode");
        let data_url = format!(
            "data:image/jpeg;base64,{}",
            STANDARD.encode(bytes.into_inner())
        );
        let frame = decode_image_data_url(&data_url).expect("data URL should decode");

        // decode_image_data_url 强制 resize 到 grid 画布尺寸（1280x960），
        // 让 ROI 坐标系无论图源是相机还是任意分辨率快照都保持一致。
        let expected_width = GRID_TILE_WIDTH * 2;
        let expected_height = GRID_TILE_HEIGHT * 2;
        assert_eq!(frame.width, expected_width);
        assert_eq!(frame.height, expected_height);
        assert_eq!(
            frame.rgb.len(),
            (expected_width * expected_height * 3) as usize
        );
    }

    #[test]
    fn runtime_mode_parses_mock_env_flags() {
        assert_eq!(
            RuntimeMode::from_env_values(Some("1"), Some("true")),
            RuntimeMode {
                mock_camera: true,
                mock_serial: true,
            }
        );
        assert_eq!(
            RuntimeMode::from_env_values(Some("off"), None),
            RuntimeMode {
                mock_camera: false,
                mock_serial: false,
            }
        );
    }

    #[test]
    fn diagnostic_log_format_has_stable_single_line_prefix() {
        assert_eq!(
            format_diagnostic_log("图像加载完成\nnext"),
            "[diagnostic] 图像加载完成\\nnext"
        );
    }

    #[test]
    fn mock_camera_devices_and_formats_are_stable() {
        let devices = mock_camera_devices();
        assert_eq!(devices.len(), 4);
        assert_eq!(devices[0].index, "0");
        assert!(devices[0].name.contains("Mock Camera 1"));

        let formats = mock_camera_formats(2);
        assert_eq!(formats.len(), 3);
        assert_eq!(
            (formats[0].width, formats[0].height, formats[0].fps),
            (640, 480, 30)
        );
        assert!(formats.iter().all(|format| format.frame_format == "MJPEG"));
    }

    #[test]
    fn mock_camera_frame_generation_is_rgb_and_changes() {
        let first = mock_camera_frame(1, &CameraConfig::default(), 1).expect("first frame");
        let second = mock_camera_frame(1, &CameraConfig::default(), 2).expect("second frame");

        assert_eq!(first.width, 640);
        assert_eq!(first.height, 480);
        assert_eq!(first.rgb.len(), 640 * 480 * 3);
        assert_ne!(first.rgb, second.rgb);
    }

    #[test]
    fn mock_serial_queues_ok_then_motion_done() {
        let mut serial = MockSerialTransport::default();
        // 直接发送已编码的下位机字符串（前端用 user digit_map 编码好后传过来）
        serial
            .send_encoded("41")
            .expect("mock send should succeed");

        assert_eq!(serial.last_payload(), Some("41"));
        assert_eq!(
            serial.read_available().expect("first read"),
            b"OK\n".to_vec()
        );
        assert_eq!(
            serial.read_available().expect("second read"),
            b"ND\n".to_vec()
        );
        assert!(serial.read_available().expect("drained").is_empty());
    }
}

enum StreamRoute {
    Grid,
    Slot(usize),
}

struct MjpegStreamReader {
    hub: Arc<FrameHub>,
    route: StreamRoute,
    last_seq: u64,
    pending: Cursor<Vec<u8>>,
}

impl MjpegStreamReader {
    fn new(hub: Arc<FrameHub>, route: StreamRoute) -> Self {
        Self {
            hub,
            route,
            last_seq: 0,
            pending: Cursor::new(Vec::new()),
        }
    }

    fn load_next_frame(&mut self) -> std::io::Result<()> {
        let frame = match self.route {
            StreamRoute::Grid => self.hub.wait_grid_frame(self.last_seq),
            StreamRoute::Slot(slot) => self.hub.wait_slot_frame(slot, self.last_seq),
        };
        let Some(frame) = frame else {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "camera stream closed",
            ));
        };
        self.last_seq = frame.seq;
        let mut part = Vec::new();
        part.extend_from_slice(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ");
        part.extend_from_slice(frame.jpeg.len().to_string().as_bytes());
        part.extend_from_slice(b"\r\n\r\n");
        part.extend_from_slice(&frame.jpeg);
        part.extend_from_slice(b"\r\n");
        self.pending = Cursor::new(part);
        Ok(())
    }
}

impl Read for MjpegStreamReader {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if self.pending.position() as usize >= self.pending.get_ref().len() {
            self.load_next_frame()?;
        }
        self.pending.read(buf)
    }
}

fn start_frame_server(
    latest_frame: Arc<Mutex<Option<LatestFrame>>>,
    stream_hub: Arc<FrameHub>,
) -> Result<u16> {
    let server = Server::http("127.0.0.1:0").map_err(|err| anyhow::anyhow!("{err}"))?;
    let port = match server.server_addr() {
        tiny_http::ListenAddr::IP(addr) => addr.port(),
        #[allow(unreachable_patterns)]
        _ => anyhow::bail!("frame server did not bind to an IP address"),
    };

    thread::spawn(move || {
        for request in server.incoming_requests() {
            let latest_frame = Arc::clone(&latest_frame);
            let stream_hub = Arc::clone(&stream_hub);
            thread::spawn(move || {
                let url = request.url().to_string();
                if url.starts_with("/grid.mjpeg") {
                    respond_mjpeg(
                        request,
                        MjpegStreamReader::new(stream_hub, StreamRoute::Grid),
                    );
                    return;
                }
                if let Some(slot) = parse_slot_stream_url(&url) {
                    respond_mjpeg(
                        request,
                        MjpegStreamReader::new(stream_hub, StreamRoute::Slot(slot)),
                    );
                    return;
                }
                respond_latest_frame(request, latest_frame);
            });
        }
    });

    Ok(port)
}

fn parse_slot_stream_url(url: &str) -> Option<usize> {
    let path = url.split('?').next().unwrap_or(url);
    let suffix = path.strip_prefix("/slot/")?.strip_suffix(".mjpeg")?;
    suffix.parse().ok()
}

fn respond_mjpeg(request: tiny_http::Request, reader: MjpegStreamReader) {
    let content_type = Header::from_bytes(
        &b"Content-Type"[..],
        &b"multipart/x-mixed-replace; boundary=frame"[..],
    )
    .ok();
    let cache_control = Header::from_bytes(&b"Cache-Control"[..], &b"no-store, max-age=0"[..]).ok();
    let mut response = Response::new(200.into(), Vec::new(), reader, None, None);
    if let Some(header) = content_type {
        response.add_header(header);
    }
    if let Some(header) = cache_control {
        response.add_header(header);
    }
    let _ = request.respond(response);
}

fn respond_latest_frame(
    request: tiny_http::Request,
    latest_frame: Arc<Mutex<Option<LatestFrame>>>,
) {
    let content_type = Header::from_bytes(&b"Content-Type"[..], &b"image/jpeg"[..]).ok();
    let cache_control = Header::from_bytes(&b"Cache-Control"[..], &b"no-store, max-age=0"[..]).ok();
    let frame = latest_frame
        .lock()
        .ok()
        .and_then(|frame| frame.as_ref().map(|frame| frame.bytes.clone()));
    let mut response = match frame {
        Some(bytes) => Response::from_data(bytes),
        None => Response::from_string("no camera frame available").with_status_code(404),
    };
    if let Some(header) = content_type {
        response.add_header(header);
    }
    if let Some(header) = cache_control {
        response.add_header(header);
    }
    let _ = request.respond(response);
}

/// 初始化日志：控制台输出 info+，文件输出 debug+（按日期分文件）。
fn setup_logging() {
    use std::fs;

    let logs_dir = Path::new("logs");
    let _ = fs::create_dir_all(logs_dir);

    let today = chrono::Local::now().format("%Y-%m-%d").to_string();
    let log_file = fern::log_file(logs_dir.join(format!("{today}.log")))
        .expect("failed to open log file");

    fern::Dispatch::new()
        .format(|out, message, record| {
            out.finish(format_args!(
                "[{} {} {}] {}",
                chrono::Local::now().format("%H:%M:%S%.3f"),
                record.level(),
                record.target(),
                message
            ))
        })
        // 文件输出：debug 及以上
        .chain(
            fern::Dispatch::new()
                .level(log::LevelFilter::Debug)
                .chain(log_file),
        )
        // 控制台输出：info 及以上（可通过 RUST_LOG 覆盖）
        .chain(
            fern::Dispatch::new()
                .level(
                    std::env::var("RUST_LOG")
                        .ok()
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(log::LevelFilter::Info),
                )
                .chain(std::io::stderr()),
        )
        .apply()
        .expect("failed to initialize logging");
}

/// 保存解算时的图片帧到 imgs/ 目录，同时保存解算结果为同名 .json。
#[tauri::command]
fn save_solve_image(image_data_url: String, solve_result: Option<String>) -> Result<String, String> {
    let imgs_dir = Path::new("imgs");
    std::fs::create_dir_all(imgs_dir).map_err(|e| e.to_string())?;

    let timestamp = chrono::Local::now().format("%Y%m%d_%H%M%S%.3f");
    let stem = format!("solve_{timestamp}");
    let img_path = imgs_dir.join(format!("{stem}.jpg"));

    let encoded = image_data_url
        .split_once(',')
        .map(|(_, data)| data)
        .unwrap_or(&image_data_url);
    let bytes = STANDARD.decode(encoded).map_err(|e| e.to_string())?;
    std::fs::write(&img_path, &bytes).map_err(|e| e.to_string())?;

    if let Some(json) = solve_result {
        let json_path = imgs_dir.join(format!("{stem}.json"));
        if let Err(e) = std::fs::write(&json_path, &json) {
            log::warn!("解算结果 JSON 保存失败: {}", e);
        }
    }

    log::info!("解算图片已保存: {}", img_path.display());
    Ok(img_path.display().to_string())
}

/// 前端主动查询 solver 是否就绪（防止错过 solver-ready 事件）。
#[tauri::command]
fn check_solver_ready(state: tauri::State<'_, AppState>) -> Result<bool, String> {
    Ok(state.solver.get().is_some())
}

const DEFAULT_ROI_FILENAME: &str = "robot-roi.json";

/// 候选 ROI 路径（按优先级）：
/// 1. `app_install_dir/robot-roi.json`：保存 ROI 时的目标位置（用户改动持久化）
/// 2. cwd 下 `robot-roi.json`：开发模式从 robo-app/ 启动、或打包时随包附带的默认值
fn roi_candidate_paths() -> Vec<std::path::PathBuf> {
    let mut paths = Vec::new();
    if let Ok(dir) = app_install_dir() {
        paths.push(dir.join(DEFAULT_ROI_FILENAME));
    }
    paths.push(std::path::PathBuf::from(DEFAULT_ROI_FILENAME));
    paths
}

/// 启动时尝试读取默认 ROI 文件 (robot-roi.json)，不存在则返回 null。
///
/// 用户保存到 `app_install_dir/robot-roi.json`（前端 dialog.save() 默认就指向
/// 这里）时下次启动会被自动读回；保存到其它路径仅作导出/备份。
/// 没找到默认路径文件时再 fallback 到 cwd（兼容打包附带的默认 ROI）。
#[tauri::command]
fn load_default_roi() -> Option<String> {
    for path in roi_candidate_paths() {
        if path.is_file() {
            match std::fs::read_to_string(&path) {
                Ok(content) => {
                    log::info!("已加载默认 ROI: {}", path.display());
                    return Some(content);
                }
                Err(e) => {
                    log::warn!("读取 ROI 失败 ({}): {}", path.display(), e);
                }
            }
        } else {
            log::debug!("ROI 文件不存在: {}", path.display());
        }
    }
    None
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    setup_logging();

    log::info!("CubeSolver starting");

    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .plugin(tauri_plugin_dialog::init())
        .manage(AppState::default())
        .setup(|app| {
            let solver_lock = {
                let state: tauri::State<'_, AppState> = app.state();
                Arc::clone(&state.solver)
            };
            let handle = app.handle().clone();

            // 启动时尝试从 app_install_dir 读取保存的动作映射
            if let Some(loaded) = load_move_mapping_from_disk(&handle) {
                let state: tauri::State<'_, AppState> = app.state();
                if let Ok(mut guard) = state.digit_map.lock() {
                    *guard = loaded;
                };
            }

            // 启动时尝试从 app_install_dir 读取应用配置（solver_timeout_ms 等）
            if let Some(loaded) = load_app_config_from_disk() {
                let state: tauri::State<'_, AppState> = app.state();
                if let Ok(mut guard) = state.config.lock() {
                    *guard = loaded;
                };
            }

            // 求解表初始化：在 setup 期间同步完成（spawn + join），
            // 避免懒加载导致用户第一次解算时多花数十 ms 表生成时间。
            // join 阻塞 setup 主线程 ~50-100ms（M-series Mac 实测），
            // 期间 logger / digit_map 加载等其它 setup 已经完成，
            // 用户感受为"启动多 100ms"，但运行时延迟稳定。
            //
            // 用 spawn 而非直接调 Search::init() 是为了：
            // 1. 不阻塞 main thread 的 panic handler / event loop 注册；
            // 2. 失败时（极罕见）log 信息仍然完整。
            let init_handle = thread::spawn(move || {
                let t0 = Instant::now();
                Search::init();
                let _ = solver_lock.set(());
                let _ = handle.emit("solver-ready", ());
                log::info!(
                    "solver 初始化完成 ({:?}, min2phase Search + handstep 多候选择优)",
                    t0.elapsed()
                );
            });
            // 同步等待表初始化完成
            if let Err(e) = init_handle.join() {
                log::error!("solver init thread panicked: {:?}", e);
            }
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            diagnostic_log,
            save_text_file_to_path,
            get_default_save_paths,
            list_cameras,
            list_camera_formats,
            open_cameras,
            close_cameras,
            open_camera_stream,
            close_camera_stream,
            camera_stream_info,
            snapshot_frame,
            capture_frame,
            latest_frame_data_url,
            list_camera_controls,
            set_camera_control,
            solve_current_frame,
            solve_latest_frame,
            solve_image_file,
            solve_facelets,
            save_solve_image,
            check_solver_ready,
            load_default_roi,
            list_serial_ports,
            open_serial,
            close_serial,
            read_serial,
            send_steps,
            get_move_mapping,
            set_move_mapping,
            save_move_mapping_to_path,
            reset_move_mapping,
            get_app_config,
            set_app_config,
            set_overlay_rois
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
