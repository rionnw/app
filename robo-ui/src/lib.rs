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
use image::{codecs::jpeg::JpegEncoder, imageops, RgbImage};
use robo_camera::{
    frame_format_from_str, CameraConfig, CameraControlKind, CameraSlotStatus, CameraSlotWorker,
    CameraSlotWorkerEvent, CameraStatusEventKind, FramePacket, MultiCameraCapture,
    MultiCameraSource,
};
use robo_core::{CubeFace, Frame, Recognizer, Roi, Solver, Steps, Translator, Transport};
use robo_solver::Min2PhaseSolver;
use robo_translator::BasicTranslator;
use robo_transport::SerialTransport;
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
    solver: Arc<std::sync::OnceLock<Min2PhaseSolver>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RuntimeMode {
    mock_camera: bool,
    mock_serial: bool,
}

impl RuntimeMode {
    fn from_env() -> Self {
        Self::from_env_values(
            std::env::var("ROBO_UI_MOCK_CAMERA").ok().as_deref(),
            std::env::var("ROBO_UI_MOCK_SERIAL").ok().as_deref(),
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
    aggregator: Option<JoinHandle<()>>,
}

const GRID_ENCODE_INTERVAL: Duration = Duration::from_millis(33);
const DIAGNOSTIC_LOG_INTERVAL: Duration = Duration::from_secs(1);
const DIAGNOSTIC_WARN_THRESHOLD: Duration = Duration::from_millis(200);
const BACKEND_DIAGNOSTIC_PREFIX: &str = "[backend-diagnostic]";

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
        let tile_width = self
            .configs
            .iter()
            .map(|config| config.width)
            .max()
            .unwrap_or(640);
        let tile_height = self
            .configs
            .iter()
            .map(|config| config.height)
            .max()
            .unwrap_or(480);
        let frame = compose_grid_frame(frames, tile_width, tile_height, 2)?
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
    fn open(configs: Vec<CameraConfig>, hub: Arc<FrameHub>, app: tauri::AppHandle) -> Result<Self> {
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

        let aggregator = spawn_camera_stream_aggregator(Arc::clone(&hub), session_id, app, rx);

        Ok(Self {
            workers,
            aggregator: Some(aggregator),
        })
    }

    fn open_mock(
        configs: Vec<CameraConfig>,
        hub: Arc<FrameHub>,
        app: tauri::AppHandle,
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

        let aggregator = spawn_camera_stream_aggregator(Arc::clone(&hub), session_id, app, rx);

        Ok(Self {
            workers,
            aggregator: Some(aggregator),
        })
    }

    fn close(&mut self) {
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

impl FrameHub {
    fn configure(&self, configs: &[CameraConfig]) -> Result<u64> {
        anyhow::ensure!(!configs.is_empty(), "at least one camera is required");
        let tile_width = configs
            .iter()
            .map(|config| config.width)
            .max()
            .unwrap_or(640);
        let tile_height = configs
            .iter()
            .map(|config| config.height)
            .max()
            .unwrap_or(480);
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

        let (frames, tile_width, tile_height, columns, should_encode_grid) = {
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
            let frames = inner
                .slots
                .iter()
                .map(|slot| slot.frame.as_ref().map(|frame| Arc::clone(&frame.rgb)))
                .collect::<Vec<_>>();
            let now = Instant::now();
            let should_encode_grid = match inner.last_grid_encode_at {
                Some(last) => now.duration_since(last) >= GRID_ENCODE_INTERVAL,
                None => true,
            } || inner.grid.is_none();
            if should_encode_grid {
                inner.last_grid_encode_at = Some(now);
            }
            self.changed.notify_all();
            (
                frames,
                inner.tile_width,
                inner.tile_height,
                inner.columns,
                should_encode_grid,
            )
        };

        if should_encode_grid {
            let compose_started = Instant::now();
            if let Some(grid_frame) = compose_grid_frame(frames, tile_width, tile_height, columns)?
            {
                let compose_ms = compose_started.elapsed().as_millis();
                let started = Instant::now();
                let grid_jpeg = encode_frame_jpeg(&grid_frame)?;
                let grid_encode_ms = started.elapsed().as_millis();
                let grid_created_at = Instant::now();
                let mut inner = self
                    .inner
                    .lock()
                    .map_err(|_| anyhow::anyhow!("stream hub is poisoned"))?;
                if inner.session_id != session_id || !inner.active {
                    return Ok(None);
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
                    capture_ms: stream_frame.capture_ms,
                    encode_ms: grid_encode_ms,
                    created_at: grid_created_at,
                });
                self.changed.notify_all();
            }
        }

        Ok(Some(stream_frame))
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

fn encode_frame_jpeg(frame: &Frame) -> Result<Vec<u8>> {
    let image = RgbImage::from_raw(frame.width, frame.height, frame.rgb.clone())
        .context("failed to create image from stream frame")?;
    let mut bytes = Cursor::new(Vec::new());
    JpegEncoder::new_with_quality(&mut bytes, 72)
        .encode_image(&image)
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
    let started = Instant::now();
    let jpeg = encode_frame_jpeg(&packet.frame)?;
    let encode_ms = started.elapsed().as_millis();
    let slot = packet.slot;
    let index = packet.index;
    let seq = packet.seq;
    let Some(frame) = hub.publish_slot_frame(session_id, packet, jpeg, encode_ms)? else {
        return Ok(());
    };
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
    fill_tile(dst, dst_width, tile_width, tile_height, dst_x, dst_y, 12);
    if src.width == tile_width && src.height == tile_height {
        blit_rgb(src, dst, dst_width, dst_x, dst_y);
        return;
    }

    let scale = (tile_width as f32 / src.width as f32).min(tile_height as f32 / src.height as f32);
    let fit_width = ((src.width as f32 * scale).round() as u32).clamp(1, tile_width);
    let fit_height = ((src.height as f32 * scale).round() as u32).clamp(1, tile_height);
    let Some(image) = RgbImage::from_raw(src.width, src.height, src.rgb.clone()) else {
        return;
    };
    let resized = imageops::resize(
        &image,
        fit_width,
        fit_height,
        imageops::FilterType::Triangle,
    );
    let Ok(frame) = Frame::new_rgb(fit_width, fit_height, resized.into_raw()) else {
        return;
    };
    let offset_x = dst_x + (tile_width - fit_width) / 2;
    let offset_y = dst_y + (tile_height - fit_height) / 2;
    blit_rgb(&frame, dst, dst_width, offset_x, offset_y);
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
    fn send_steps(&mut self, steps: &Steps) -> Result<()> {
        match self {
            Self::Real(transport) => transport.send_steps(steps),
            Self::Mock(transport) => transport.send_steps(steps),
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

    #[cfg(test)]
    fn last_payload(&self) -> Option<&str> {
        self.last_payload.as_deref()
    }
}

impl Transport for MockSerialTransport {
    fn send_steps(&mut self, steps: &Steps) -> Result<()> {
        self.last_payload = Some(steps.encoded.clone());
        self.responses.push_back(b"OK\n".to_vec());
        self.responses.push_back(b"ND\n".to_vec());
        Ok(())
    }
}

#[derive(Debug, Serialize)]
struct SolveFaceletsResponse {
    facelets: String,
    moves: Vec<String>,
    steps: Vec<String>,
    encoded_steps: String,
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

#[tauri::command]
fn save_text_file(
    filename: String,
    contents: String,
) -> Result<String, String> {
    let safe_filename = Path::new(&filename)
        .file_name()
        .and_then(|name| name.to_str())
        .filter(|name| !name.is_empty())
        .ok_or_else(|| "文件名无效。".to_string())?;
    let path = Path::new(safe_filename);

    std::fs::write(path, contents).map_err(|err| err.to_string())?;
    log::info!("文件已保存: {}", path.display());
    Ok(path.display().to_string())
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
    let next_runtime = match if state.mode.mock_camera {
        CameraStreamRuntime::open_mock(configs, Arc::clone(&state.stream_hub), app)
    } else {
        CameraStreamRuntime::open(configs, Arc::clone(&state.stream_hub), app)
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

#[tauri::command]
fn latest_frame_data_url(state: tauri::State<'_, AppState>) -> Result<String, String> {
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
    let controls = if state.mode.mock_camera {
        state
            .mock_camera
            .lock()
            .map_err(|_| "mock camera state is poisoned".to_string())?
            .controls(slot)
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
    if state.mode.mock_camera {
        state
            .mock_camera
            .lock()
            .map_err(|_| "mock camera state is poisoned".to_string())?
            .set_control(slot, id, value)
            .map_err(|err| err.to_string())
    } else {
        state
            .camera
            .lock()
            .map_err(|_| "camera state is poisoned".to_string())?
            .set_control(slot, id, value)
            .map_err(|err| err.to_string())
    }
}

#[tauri::command]
fn solve_current_frame(
    state: tauri::State<'_, AppState>,
    rois: Vec<RoiDto>,
) -> Result<SolveFaceletsResponse, String> {
    let solver = state.solver.get().ok_or("solver 尚未初始化完成")?;
    let capture = capture_from_state(&state).map_err(|err| err.to_string())?;
    let rois = rois.into_iter().map(Roi::from).collect::<Vec<_>>();
    let recognizer = ColorClusterRecognizer;
    let face = recognizer
        .recognize(&capture.frame, &rois)
        .map_err(|err| err.to_string())?;
    solve_face(face, solver)
}

#[tauri::command]
fn solve_latest_frame(
    state: tauri::State<'_, AppState>,
    rois: Vec<RoiDto>,
) -> Result<SolveFaceletsResponse, String> {
    let solver = state.solver.get().ok_or("solver 尚未初始化完成")?;
    let frame = state
        .stream_hub
        .latest_grid_rgb()
        .map_err(|err| err.to_string())?;
    let rois = rois.into_iter().map(Roi::from).collect::<Vec<_>>();
    let recognizer = ColorClusterRecognizer;
    let face = recognizer
        .recognize(&frame, &rois)
        .map_err(|err| err.to_string())?;
    solve_face(face, solver)
}

#[tauri::command]
fn solve_image_file(
    state: tauri::State<'_, AppState>,
    image_data_url: String,
    rois: Vec<RoiDto>,
) -> Result<SolveFaceletsResponse, String> {
    let solver = state.solver.get().ok_or("solver 尚未初始化完成")?;
    let frame = decode_image_data_url(&image_data_url).map_err(|err| err.to_string())?;
    let rois = rois.into_iter().map(Roi::from).collect::<Vec<_>>();
    let recognizer = ColorClusterRecognizer;
    let face = recognizer
        .recognize(&frame, &rois)
        .map_err(|err| err.to_string())?;
    solve_face(face, solver)
}

#[tauri::command]
fn solve_facelets(state: tauri::State<'_, AppState>, facelets: String) -> Result<SolveFaceletsResponse, String> {
    let solver = state.solver.get().ok_or("solver 尚未初始化完成")?;
    let face = CubeFace::new(facelets).map_err(|err| err.to_string())?;
    solve_face(face, solver)
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

#[tauri::command]
fn send_steps(state: tauri::State<'_, AppState>, encoded_steps: String) -> Result<(), String> {
    let mut serial = state
        .serial
        .lock()
        .map_err(|_| "serial state is poisoned".to_string())?;
    let transport = serial
        .as_mut()
        .ok_or_else(|| "serial port is not open".to_string())?;
    let steps = Steps {
        commands: Vec::new(),
        encoded: encoded_steps,
    };
    transport.send_steps(&steps).map_err(|err| err.to_string())
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

fn solve_face(face: CubeFace, solver: &Min2PhaseSolver) -> Result<SolveFaceletsResponse, String> {
    let translator = BasicTranslator::new();
    let moves = solver.solve(&face).map_err(|err| err.to_string())?;
    let steps = translator
        .translate(&moves)
        .map_err(|err| err.to_string())?;

    Ok(SolveFaceletsResponse {
        facelets: face.into_string(),
        moves: moves.0,
        steps: steps.commands,
        encoded_steps: steps.encoded,
    })
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
    let (width, height) = image.dimensions();
    Frame::new_rgb(width, height, image.into_raw()).context("failed to build frame from image file")
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

        assert_eq!(frame.width, 2);
        assert_eq!(frame.height, 1);
        assert_eq!(frame.rgb.len(), 6);
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
        serial
            .send_steps(&Steps {
                commands: Vec::new(),
                encoded: "R U R'".to_string(),
            })
            .expect("mock send should succeed");

        assert_eq!(serial.last_payload(), Some("R U R'"));
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

/// 保存解算时的图片帧到 imgs/ 目录。
#[tauri::command]
fn save_solve_image(image_data_url: String) -> Result<String, String> {
    let imgs_dir = Path::new("imgs");
    std::fs::create_dir_all(imgs_dir).map_err(|e| e.to_string())?;

    let timestamp = chrono::Local::now().format("%Y%m%d_%H%M%S%.3f");
    let filename = format!("solve_{timestamp}.jpg");
    let path = imgs_dir.join(&filename);

    let encoded = image_data_url
        .split_once(',')
        .map(|(_, data)| data)
        .unwrap_or(&image_data_url);
    let bytes = STANDARD.decode(encoded).map_err(|e| e.to_string())?;
    std::fs::write(&path, &bytes).map_err(|e| e.to_string())?;

    log::info!("解算图片已保存: {}", path.display());
    Ok(path.display().to_string())
}

/// 启动时尝试读取默认 ROI 文件 (robot-roi.json)，不存在则返回 null。
#[tauri::command]
fn load_default_roi() -> Option<String> {
    let roi_path = Path::new("robot-roi.json");
    if roi_path.is_file() {
        match std::fs::read_to_string(roi_path) {
            Ok(content) => {
                log::info!("已加载默认 ROI: {}", roi_path.display());
                Some(content)
            }
            Err(e) => {
                log::warn!("读取默认 ROI 失败: {}", e);
                None
            }
        }
    } else {
        log::debug!("默认 ROI 文件不存在: {}", roi_path.display());
        None
    }
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    setup_logging();

    log::info!("CubeSolver starting");

    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .manage(AppState::default())
        .setup(|app| {
            let solver_lock = {
                let state: tauri::State<'_, AppState> = app.state();
                Arc::clone(&state.solver)
            };
            let handle = app.handle().clone();
            thread::spawn(move || {
                let solver = Min2PhaseSolver::new();
                let _ = solver_lock.set(solver);
                let _ = handle.emit("solver-ready", ());
                log::info!("solver 初始化完成");
            });
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            diagnostic_log,
            save_text_file,
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
            load_default_roi,
            list_serial_ports,
            open_serial,
            close_serial,
            read_serial,
            send_steps
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
