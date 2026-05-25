use std::{
    io::Cursor,
    sync::{mpsc, Arc, Mutex},
    thread,
};

use anyhow::{Context, Result};
use base64::{engine::general_purpose::STANDARD, Engine};
use image::{codecs::jpeg::JpegEncoder, RgbImage};
use robo_camera::{
    frame_format_from_str, CameraConfig, CameraControlKind, CameraStatusEventKind,
    MultiCameraCapture, MultiCameraSource,
};
use robo_core::{CubeFace, Recognizer, Roi, Solver, Steps, Translator, Transport};
use robo_solver::Min2PhaseSolver;
use robo_translator::BasicTranslator;
use robo_transport::SerialTransport;
use robo_vision::ColorClusterRecognizer;
use serde::{Deserialize, Serialize};
use tiny_http::{Header, Response, Server};

struct AppState {
    camera: Mutex<CameraWorker>,
    serial: Mutex<Option<SerialTransport>>,
    latest_frame: Arc<Mutex<Option<LatestFrame>>>,
    latest_frame_seq: Mutex<u64>,
    frame_server_port: u16,
}

#[derive(Clone)]
struct LatestFrame {
    bytes: Vec<u8>,
}

impl Default for AppState {
    fn default() -> Self {
        let latest_frame = Arc::new(Mutex::new(None));
        let frame_server_port = start_frame_server(Arc::clone(&latest_frame)).unwrap_or(0);
        Self {
            camera: Mutex::default(),
            serial: Mutex::default(),
            latest_frame,
            latest_frame_seq: Mutex::default(),
            frame_server_port,
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

#[derive(Debug, Serialize)]
struct CameraDeviceDto {
    index: String,
    name: String,
    description: String,
}

#[derive(Debug, Serialize)]
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

#[derive(Debug, Serialize)]
struct SolveFaceletsResponse {
    facelets: String,
    moves: Vec<String>,
    steps: Vec<String>,
    encoded_steps: String,
}

#[tauri::command]
fn list_cameras() -> Result<Vec<CameraDeviceDto>, String> {
    robo_camera::list_cameras()
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
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn list_camera_formats(index: u32) -> Result<Vec<CameraFormatDto>, String> {
    robo_camera::list_camera_formats(index)
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
        .map_err(|err| err.to_string())
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
    state
        .camera
        .lock()
        .map_err(|_| "camera state is poisoned".to_string())?
        .open(configs)
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn close_cameras(state: tauri::State<'_, AppState>) -> Result<(), String> {
    state
        .camera
        .lock()
        .map_err(|_| "camera state is poisoned".to_string())?
        .close()
        .map_err(|err| err.to_string())
}

#[tauri::command]
fn capture_frame(state: tauri::State<'_, AppState>) -> Result<FrameResponse, String> {
    let capture = state
        .camera
        .lock()
        .map_err(|_| "camera state is poisoned".to_string())?
        .capture()
        .map_err(|err| err.to_string())?;
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
    let controls = state
        .camera
        .lock()
        .map_err(|_| "camera state is poisoned".to_string())?
        .controls(slot)
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
    let capture = state
        .camera
        .lock()
        .map_err(|_| "camera state is poisoned".to_string())?
        .capture()
        .map_err(|err| err.to_string())?;
    let rois = rois.into_iter().map(Roi::from).collect::<Vec<_>>();
    let recognizer = ColorClusterRecognizer;
    let face = recognizer
        .recognize(&capture.frame, &rois)
        .map_err(|err| err.to_string())?;
    solve_face(face)
}

#[tauri::command]
fn solve_facelets(facelets: String) -> Result<SolveFaceletsResponse, String> {
    let face = CubeFace::new(facelets).map_err(|err| err.to_string())?;
    solve_face(face)
}

#[tauri::command]
fn list_serial_ports() -> Result<Vec<SerialPortDto>, String> {
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
    let transport = SerialTransport::open(&port_name, baud_rate).map_err(|err| err.to_string())?;
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

fn solve_face(face: CubeFace) -> Result<SolveFaceletsResponse, String> {
    let solver = Min2PhaseSolver::new();
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

fn start_frame_server(latest_frame: Arc<Mutex<Option<LatestFrame>>>) -> Result<u16> {
    let server = Server::http("127.0.0.1:0").map_err(|err| anyhow::anyhow!("{err}"))?;
    let port = match server.server_addr() {
        tiny_http::ListenAddr::IP(addr) => addr.port(),
        #[allow(unreachable_patterns)]
        _ => anyhow::bail!("frame server did not bind to an IP address"),
    };

    thread::spawn(move || {
        let content_type = Header::from_bytes(&b"Content-Type"[..], &b"image/jpeg"[..]).ok();
        let cache_control =
            Header::from_bytes(&b"Cache-Control"[..], &b"no-store, max-age=0"[..]).ok();

        for request in server.incoming_requests() {
            let frame = latest_frame
                .lock()
                .ok()
                .and_then(|frame| frame.as_ref().map(|frame| frame.bytes.clone()));

            let mut response = match frame {
                Some(bytes) => Response::from_data(bytes),
                None => Response::from_string("no camera frame available").with_status_code(404),
            };
            if let Some(header) = content_type.clone() {
                response.add_header(header);
            }
            if let Some(header) = cache_control.clone() {
                response.add_header(header);
            }
            let _ = request.respond(response);
        }
    });

    Ok(port)
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .manage(AppState::default())
        .invoke_handler(tauri::generate_handler![
            list_cameras,
            list_camera_formats,
            open_cameras,
            close_cameras,
            capture_frame,
            latest_frame_data_url,
            list_camera_controls,
            set_camera_control,
            solve_current_frame,
            solve_facelets,
            list_serial_ports,
            open_serial,
            close_serial,
            read_serial,
            send_steps
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
