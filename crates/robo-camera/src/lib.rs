use anyhow::{Context, Result};
use image::{imageops, RgbImage};
use nokhwa::{
    pixel_format::RgbFormat,
    utils::{
        all_known_camera_controls, ApiBackend, CameraFormat, CameraIndex, ControlValueDescription,
        ControlValueSetter, FrameFormat, KnownCameraControl, RequestedFormat, RequestedFormatType,
        Resolution,
    },
    Camera,
};
use robo_core::{CameraSource, Frame};
use std::{
    str::FromStr,
    time::{Duration, Instant},
};

const RECONNECT_INTERVAL: Duration = Duration::from_secs(2);

#[derive(Clone, Debug)]
pub struct CameraConfig {
    pub index: u32,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub frame_format: FrameFormat,
}

#[derive(Clone, Debug)]
pub struct CameraDevice {
    pub index: String,
    pub name: String,
    pub description: String,
}

#[derive(Clone, Debug)]
pub struct CameraFormatInfo {
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub frame_format: String,
}

#[derive(Clone, Debug)]
pub struct CameraControlInfo {
    pub id: String,
    pub name: String,
    pub kind: CameraControlKind,
    pub value: f64,
    pub default: f64,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub step: Option<f64>,
    pub active: bool,
    pub flags: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CameraControlKind {
    Integer,
    Float,
    Boolean,
}

pub fn list_cameras() -> Result<Vec<CameraDevice>> {
    let cameras = nokhwa::query(ApiBackend::Auto).context("failed to query cameras")?;
    Ok(cameras
        .into_iter()
        .map(|camera| CameraDevice {
            index: camera.index().to_string(),
            name: camera.human_name(),
            description: camera.description().to_string(),
        })
        .collect())
}

pub fn list_camera_formats(index: u32) -> Result<Vec<CameraFormatInfo>> {
    let requested = RequestedFormat::new::<RgbFormat>(RequestedFormatType::None);
    let mut camera = Camera::new(CameraIndex::Index(index), requested)
        .with_context(|| format!("failed to open camera {index} for format query"))?;
    let mut formats = camera
        .compatible_camera_formats()
        .with_context(|| format!("failed to query camera {index} formats"))?
        .into_iter()
        .map(|format| CameraFormatInfo {
            width: format.width(),
            height: format.height(),
            fps: format.frame_rate(),
            frame_format: format.format().to_string(),
        })
        .collect::<Vec<_>>();
    formats.sort_by(|a, b| {
        (a.width, a.height, a.fps, &a.frame_format).cmp(&(
            b.width,
            b.height,
            b.fps,
            &b.frame_format,
        ))
    });
    formats.dedup_by(|a, b| {
        a.width == b.width
            && a.height == b.height
            && a.fps == b.fps
            && a.frame_format == b.frame_format
    });
    Ok(formats)
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            index: 0,
            width: 640,
            height: 480,
            fps: 30,
            frame_format: FrameFormat::MJPEG,
        }
    }
}

pub struct NokhwaCamera {
    camera: Camera,
}

impl NokhwaCamera {
    pub fn open(config: CameraConfig) -> Result<Self> {
        let format = CameraFormat::new(
            Resolution::new(config.width, config.height),
            config.frame_format,
            config.fps,
        );
        let requested = RequestedFormat::new::<RgbFormat>(RequestedFormatType::Exact(format));
        let mut camera = Camera::new(CameraIndex::Index(config.index), requested)
            .with_context(|| format!("failed to open camera {}", config.index))?;
        camera
            .open_stream()
            .context("failed to open camera stream")?;
        Ok(Self { camera })
    }

    pub fn controls(&self) -> Result<Vec<CameraControlInfo>> {
        let controls = self
            .camera
            .camera_controls()
            .context("failed to query camera controls")?;
        Ok(controls
            .into_iter()
            .filter_map(|control| {
                control_info_from_description(
                    control.control(),
                    control.name().to_string(),
                    control.description(),
                    control.active(),
                    control.flag().iter().map(ToString::to_string).collect(),
                )
            })
            .collect())
    }

    pub fn set_control(&mut self, id: &str, value: f64) -> Result<()> {
        let control = known_control_from_id(id)?;
        let current = self
            .camera
            .camera_control(control)
            .with_context(|| format!("failed to query camera control {id}"))?;
        let setter = setter_from_description(current.description(), value)?;
        self.camera
            .set_camera_control(control, setter)
            .with_context(|| format!("failed to set camera control {id}"))
    }
}

pub fn frame_format_from_str(value: &str) -> Result<FrameFormat> {
    FrameFormat::from_str(value).with_context(|| format!("unsupported camera frame format {value}"))
}

impl CameraSource for NokhwaCamera {
    fn capture(&mut self) -> Result<Frame> {
        let buffer = self.camera.frame().context("failed to capture frame")?;
        let resolution = buffer.resolution();
        let image = buffer
            .decode_image::<RgbFormat>()
            .context("failed to decode camera frame as RGB")?;

        Frame::new_rgb(resolution.width(), resolution.height(), image.into_raw())
    }
}

pub struct MultiCameraSource {
    slots: Vec<CameraSlot>,
    tile_width: u32,
    tile_height: u32,
    columns: u32,
}

struct CameraSlot {
    config: CameraConfig,
    camera: Option<NokhwaCamera>,
    connected: bool,
    last_error: Option<String>,
    last_open_attempt: Option<Instant>,
}

#[derive(Clone, Debug)]
pub struct CameraSlotStatus {
    pub slot: usize,
    pub index: u32,
    pub connected: bool,
    pub message: String,
}

#[derive(Clone, Debug)]
pub struct CameraStatusEvent {
    pub slot: usize,
    pub index: u32,
    pub kind: CameraStatusEventKind,
    pub message: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CameraStatusEventKind {
    Connected,
    Disconnected,
}

#[derive(Clone, Debug)]
pub struct MultiCameraCapture {
    pub frame: Frame,
    pub statuses: Vec<CameraSlotStatus>,
    pub events: Vec<CameraStatusEvent>,
}

impl MultiCameraSource {
    pub fn open(configs: Vec<CameraConfig>, columns: u32) -> Result<Self> {
        anyhow::ensure!(!configs.is_empty(), "at least one camera is required");
        anyhow::ensure!(columns > 0, "columns must be greater than zero");
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
            .into_iter()
            .map(|config| CameraSlot {
                config,
                camera: None,
                connected: false,
                last_error: None,
                last_open_attempt: None,
            })
            .collect::<Vec<_>>();

        Ok(Self {
            slots,
            tile_width,
            tile_height,
            columns,
        })
    }

    pub fn capture_with_status(&mut self) -> Result<MultiCameraCapture> {
        let rows = (self.slots.len() as u32 + self.columns - 1) / self.columns;
        let output_width = self.tile_width * self.columns;
        let output_height = self.tile_height * rows;
        let mut output = vec![0u8; output_width as usize * output_height as usize * 3];
        let mut statuses = Vec::with_capacity(self.slots.len());
        let mut events = Vec::new();

        for (idx, slot) in self.slots.iter_mut().enumerate() {
            let tile_x = (idx as u32 % self.columns) * self.tile_width;
            let tile_y = (idx as u32 / self.columns) * self.tile_height;
            let frame = capture_slot(slot, idx, &mut events);

            match frame {
                Ok(frame) => {
                    blit_fit_rgb(
                        &frame,
                        &mut output,
                        output_width,
                        self.tile_width,
                        self.tile_height,
                        tile_x,
                        tile_y,
                    );
                }
                Err(_) => {
                    fill_placeholder(
                        &mut output,
                        output_width,
                        self.tile_width,
                        self.tile_height,
                        tile_x,
                        tile_y,
                        idx,
                    );
                }
            }

            statuses.push(CameraSlotStatus {
                slot: idx,
                index: slot.config.index,
                connected: slot.connected,
                message: slot
                    .last_error
                    .clone()
                    .unwrap_or_else(|| "connected".to_string()),
            });
        }

        Ok(MultiCameraCapture {
            frame: Frame::new_rgb(output_width, output_height, output)?,
            statuses,
            events,
        })
    }

    pub fn camera_controls(&mut self, slot_index: usize) -> Result<Vec<CameraControlInfo>> {
        let slot = self
            .slots
            .get_mut(slot_index)
            .with_context(|| format!("camera slot {slot_index} does not exist"))?;
        ensure_slot_camera(slot, slot_index, &mut Vec::new())?;
        slot.camera
            .as_ref()
            .context("camera is not available")?
            .controls()
    }

    pub fn set_camera_control(&mut self, slot_index: usize, id: &str, value: f64) -> Result<()> {
        let slot = self
            .slots
            .get_mut(slot_index)
            .with_context(|| format!("camera slot {slot_index} does not exist"))?;
        ensure_slot_camera(slot, slot_index, &mut Vec::new())?;
        slot.camera
            .as_mut()
            .context("camera is not available")?
            .set_control(id, value)
    }
}

impl CameraSource for MultiCameraSource {
    fn capture(&mut self) -> Result<Frame> {
        Ok(self.capture_with_status()?.frame)
    }
}

fn capture_slot(
    slot: &mut CameraSlot,
    slot_index: usize,
    events: &mut Vec<CameraStatusEvent>,
) -> Result<Frame> {
    ensure_slot_camera(slot, slot_index, events)?;

    let Some(camera) = slot.camera.as_mut() else {
        anyhow::bail!("camera is not available");
    };

    match camera.capture() {
        Ok(frame) => {
            if !slot.connected {
                events.push(CameraStatusEvent {
                    slot: slot_index,
                    index: slot.config.index,
                    kind: CameraStatusEventKind::Connected,
                    message: "camera recovered".to_string(),
                });
            }
            slot.connected = true;
            slot.last_error = None;
            Ok(frame)
        }
        Err(err) => {
            slot.camera = None;
            mark_disconnected(slot, slot_index, err.to_string(), events);
            Err(err)
        }
    }
}

fn ensure_slot_camera(
    slot: &mut CameraSlot,
    slot_index: usize,
    events: &mut Vec<CameraStatusEvent>,
) -> Result<()> {
    if slot.camera.is_some() {
        return Ok(());
    }

    let now = Instant::now();
    if slot
        .last_open_attempt
        .is_some_and(|attempt| now.duration_since(attempt) < RECONNECT_INTERVAL)
    {
        anyhow::bail!(
            "{}",
            slot.last_error
                .as_deref()
                .unwrap_or("camera reconnect is throttled")
        );
    }
    slot.last_open_attempt = Some(now);

    {
        match NokhwaCamera::open(slot.config.clone()) {
            Ok(camera) => {
                slot.camera = Some(camera);
                if !slot.connected {
                    events.push(CameraStatusEvent {
                        slot: slot_index,
                        index: slot.config.index,
                        kind: CameraStatusEventKind::Connected,
                        message: "camera connected".to_string(),
                    });
                }
                slot.connected = true;
                slot.last_error = None;
            }
            Err(err) => {
                mark_disconnected(slot, slot_index, err.to_string(), events);
                anyhow::bail!("{err}");
            }
        }
    }
    Ok(())
}

fn control_info_from_description(
    control_id: KnownCameraControl,
    name: String,
    description: &ControlValueDescription,
    active: bool,
    flags: Vec<String>,
) -> Option<CameraControlInfo> {
    let id = control_id.to_string();
    match description {
        ControlValueDescription::Integer {
            value,
            default,
            step,
        } => Some(CameraControlInfo {
            id,
            name,
            kind: CameraControlKind::Integer,
            value: *value as f64,
            default: *default as f64,
            min: None,
            max: None,
            step: Some(*step as f64),
            active,
            flags,
        }),
        ControlValueDescription::IntegerRange {
            min,
            max,
            value,
            step,
            default,
        } => Some(CameraControlInfo {
            id,
            name,
            kind: CameraControlKind::Integer,
            value: *value as f64,
            default: *default as f64,
            min: Some(*min as f64),
            max: Some(*max as f64),
            step: Some(*step as f64),
            active,
            flags,
        }),
        ControlValueDescription::Float {
            value,
            default,
            step,
        } => Some(CameraControlInfo {
            id,
            name,
            kind: CameraControlKind::Float,
            value: *value,
            default: *default,
            min: None,
            max: None,
            step: Some(*step),
            active,
            flags,
        }),
        ControlValueDescription::FloatRange {
            min,
            max,
            value,
            step,
            default,
        } => Some(CameraControlInfo {
            id,
            name,
            kind: CameraControlKind::Float,
            value: *value,
            default: *default,
            min: Some(*min),
            max: Some(*max),
            step: Some(*step),
            active,
            flags,
        }),
        ControlValueDescription::Boolean { value, default } => Some(CameraControlInfo {
            id,
            name,
            kind: CameraControlKind::Boolean,
            value: if *value { 1.0 } else { 0.0 },
            default: if *default { 1.0 } else { 0.0 },
            min: Some(0.0),
            max: Some(1.0),
            step: Some(1.0),
            active,
            flags,
        }),
        _ => None,
    }
}

fn setter_from_description(
    description: &ControlValueDescription,
    value: f64,
) -> Result<ControlValueSetter> {
    Ok(match description {
        ControlValueDescription::Integer { .. } | ControlValueDescription::IntegerRange { .. } => {
            ControlValueSetter::Integer(value.round() as i64)
        }
        ControlValueDescription::Float { .. } | ControlValueDescription::FloatRange { .. } => {
            ControlValueSetter::Float(value)
        }
        ControlValueDescription::Boolean { .. } => ControlValueSetter::Boolean(value >= 0.5),
        _ => anyhow::bail!("camera control type is not supported by the UI"),
    })
}

fn known_control_from_id(id: &str) -> Result<KnownCameraControl> {
    all_known_camera_controls()
        .into_iter()
        .find(|control| control.to_string() == id)
        .with_context(|| format!("unknown camera control {id}"))
}

fn mark_disconnected(
    slot: &mut CameraSlot,
    slot_index: usize,
    message: String,
    events: &mut Vec<CameraStatusEvent>,
) {
    if slot.connected {
        events.push(CameraStatusEvent {
            slot: slot_index,
            index: slot.config.index,
            kind: CameraStatusEventKind::Disconnected,
            message: message.clone(),
        });
    }
    slot.connected = false;
    slot.last_error = Some(message);
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
    let frame = match Frame::new_rgb(fit_width, fit_height, resized.into_raw()) {
        Ok(frame) => frame,
        Err(_) => return,
    };
    let offset_x = dst_x + (tile_width - fit_width) / 2;
    let offset_y = dst_y + (tile_height - fit_height) / 2;
    blit_rgb(&frame, dst, dst_width, offset_x, offset_y);
}

fn fill_placeholder(
    dst: &mut [u8],
    dst_width: u32,
    width: u32,
    height: u32,
    dst_x: u32,
    dst_y: u32,
    slot: usize,
) {
    fill_tile(
        dst,
        dst_width,
        width,
        height,
        dst_x,
        dst_y,
        (slot as u8 % 4) * 10,
    );
}

fn fill_tile(
    dst: &mut [u8],
    dst_width: u32,
    width: u32,
    height: u32,
    dst_x: u32,
    dst_y: u32,
    tint: u8,
) {
    for y in 0..height {
        for x in 0..width {
            let idx = ((dst_y + y) * dst_width * 3 + (dst_x + x) * 3) as usize;
            let checker = ((x / 24 + y / 24) % 2) as u8;
            let base = if checker == 0 { 28 } else { 42 };
            dst[idx] = base;
            dst[idx + 1] = base;
            dst[idx + 2] = base + tint;
        }
    }
}
