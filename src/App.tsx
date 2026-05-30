import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import { save as dialogSave } from "@tauri-apps/plugin-dialog";
import { useEffect, useMemo, useRef, useState } from "react";
import "./App.css";
import { resolveCanvasPreviewFps, shouldRequestCanvasFrame } from "./canvasPreview";
import {
  createCameraFrameGapDiagnostic,
  createImageBoxDiagnostic,
  createImageLoadDiagnostic,
  hasMaterialImageBoxChange,
  isCameraFrameGapAbnormal,
  sendDiagnosticLog,
  shouldLogThrottledDiagnostic,
} from "./imageDiagnostics";
import {
  applyViewPreset,
  createInitialPanelVisibility,
  createSavedPanelVisibility,
  getCameraProfileKey,
  panelIds,
  panelLabels,
  togglePanelVisibility,
  viewPresetLabels,
  type PanelVisibility,
  type ViewPreset,
} from "./panelView";
import { getLoadedImageSize, updateImageSize } from "./imageLayout";
import { createFixedPixelRoi } from "./roiAnnotation";
import {
  createDefaultRoiRegions,
  createRobotAppRoiExport,
  getRoiIndexView,
  normalizeLoadedRoiRegions,
  type RoiRegion,
} from "./roiIndexView";
import { createSolveFrameRequest, type SolveImageSource } from "./solveAction";
import { MoveMappingEditor } from "./MoveMappingEditor";

type CameraDevice = { index: string; name: string; description: string };
type CameraConfig = { index: number; width: number; height: number; fps: number; frameFormat: string };
type CameraStatus = { slot: number; index: number; connected: boolean; message: string };
type CameraControl = {
  id: string;
  name: string;
  kind: "integer" | "float" | "boolean";
  value: number;
  default: number;
  min: number | null;
  max: number | null;
  step: number | null;
  active: boolean;
  flags: string[];
};
type CameraStreamInfo = {
  gridUrl: string;
  slotUrls: string[];
  width: number;
  height: number;
  statuses: CameraStatus[];
};
type CameraStreamEvent = {
  kind: "frame" | "status" | "connected" | "disconnected" | "error";
  slot: number | null;
  index: number | null;
  seq: number | null;
  width: number | null;
  height: number | null;
  fps: number | null;
  captureMs: number | null;
  encodeMs: number | null;
  message: string | null;
  statuses: CameraStatus[];
};
type SerialPort = { name: string; port_type: string };
type SerialReadResponse = {
  text: string;
  motion_finished: boolean;
  param_write_ok: boolean;
  param_write_error: boolean;
};
type SolveResponse = {
  facelets: string;
  moves: string[];
  steps: string[];
  encoded_steps: string;
  /// solver（Search.solutions）阶段耗时（毫秒）
  search_elapsed_ms: number;
  /// handstep（候选并行翻译为机械步骤）阶段耗时（毫秒）
  handstep_elapsed_ms: number;
  /// 最终选中候选的机械步数
  mech_steps: number;
  /// solver 产出的候选数量
  candidate_count: number;
};

type SolveStats = {
  searchElapsedMs: number;
  handstepElapsedMs: number;
  mechSteps: number;
  candidateCount: number;
  faceMoves: number;
};
type ImageBox = { left: number; top: number; width: number; height: number };
type LogItem = { time: string; text: string; kind: "info" | "warn" | "error" };
type CameraPreset = { label: string; width: number; height: number; fps: number; frameFormat: string };

const solvedFacelets = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
const firstRoiRegionId = getRoiIndexView(0).label;
const panelVisibilityStorageKey = "cubesolver.panel-visibility";
const imageLayoutDiagnosticThrottleMs = 3_000;
const cameraGapDiagnosticThrottleMs = 5_000;
const canvasFrameDiagnosticThrottleMs = 3_000;
const roiLabelOffset = 0.006;
const roiLabelInset = 0.012;

const clampUnit = (value: number) => Math.min(1 - roiLabelInset, Math.max(roiLabelInset, value));

const getRoiLabelPosition = (rect: RoiRegion["rect"]) => {
  if (!rect) return { x: roiLabelInset, y: roiLabelInset };
  return {
    x: clampUnit(rect.x + rect.w / 2),
    y: clampUnit(rect.y + rect.h + roiLabelOffset),
  };
};

const defaultCameraConfigs = (): CameraConfig[] =>
  Array.from({ length: 4 }, (_, index) => ({ index, width: 640, height: 480, fps: 30, frameFormat: "MJPEG" }));

const cameraPresets: CameraPreset[] = [
  { label: "320 x 240 @ 30 MJPEG", width: 320, height: 240, fps: 30, frameFormat: "MJPEG" },
  { label: "640 x 480 @ 30 MJPEG", width: 640, height: 480, fps: 30, frameFormat: "MJPEG" },
  { label: "800 x 600 @ 30 MJPEG", width: 800, height: 600, fps: 30, frameFormat: "MJPEG" },
  { label: "1280 x 720 @ 30 MJPEG", width: 1280, height: 720, fps: 30, frameFormat: "MJPEG" },
  { label: "1280 x 720 @ 60 MJPEG", width: 1280, height: 720, fps: 60, frameFormat: "MJPEG" },
  { label: "1920 x 1080 @ 30 MJPEG", width: 1920, height: 1080, fps: 30, frameFormat: "MJPEG" },
];

const presetLabel = (format: { width: number; height: number; fps: number; frameFormat: string }) =>
  `${format.width} x ${format.height} @ ${format.fps} ${format.frameFormat}`;

const presetValue = (format: { width: number; height: number; fps: number; frameFormat: string }) =>
  `${format.width}x${format.height}@${format.fps}:${format.frameFormat}`;

const isSupportedPreviewFormat = (format: { width: number; height: number; fps: number }) => {
  const pixels = format.width * format.height;
  const aspectRatio = format.width / format.height;
  const commonAspectRatio =
    Math.abs(aspectRatio - 4 / 3) < 0.04 || Math.abs(aspectRatio - 16 / 9) < 0.04;

  return (
    format.fps >= 30 &&
    format.width >= 320 &&
    format.height >= 240 &&
    pixels <= 1920 * 1080 &&
    commonAspectRatio
  );
};

const sameCameraStatuses = (left: CameraStatus[], right: CameraStatus[]) =>
  left.length === right.length &&
  left.every((item, index) => {
    const other = right[index];
    return (
      other &&
      item.slot === other.slot &&
      item.index === other.index &&
      item.connected === other.connected &&
      item.message === other.message
    );
  });

const toFrameArrayBuffer = (frame: ArrayBuffer | Uint8Array | number[]) => {
  if (frame instanceof ArrayBuffer) return frame;
  const bytes = frame instanceof Uint8Array ? frame : new Uint8Array(frame);
  const copy = new Uint8Array(bytes.byteLength);
  copy.set(bytes);
  return copy.buffer;
};

const loadImageElement = async (url: string) => {
  const image = new Image();
  image.decoding = "async";
  image.src = url;
  await image.decode();
  return image;
};

const nowTime = () =>
  new Intl.DateTimeFormat("zh-CN", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }).format(new Date());

function App() {
  const imageInputRef = useRef<HTMLInputElement>(null);
  const roiInputRef = useRef<HTMLInputElement>(null);
  const stageRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageLoadTimingRef = useRef<{ src: string; startedAtMs: number } | null>(null);
  const lastImageSrcRef = useRef<string | null>(null);
  const lastLoggedImageBoxRef = useRef<{ box: ImageBox | null; loggedAtMs: number | null }>({
    box: null,
    loggedAtMs: null,
  });
  const lastCameraFrameAtBySlotRef = useRef<Record<number, number>>({});
  const lastCameraFrameGapLogAtBySlotRef = useRef<Record<number, number>>({});
  const lastCanvasFrameLogAtRef = useRef<number | null>(null);

  const [devices, setDevices] = useState<CameraDevice[]>([]);
  const [cameraConfigs, setCameraConfigs] = useState<CameraConfig[]>(defaultCameraConfigs);
  const [cameraStatuses, setCameraStatuses] = useState<CameraStatus[]>([]);
  const [swapSlot, setSwapSlot] = useState<number | null>(null);
  const [controlSlot, setControlSlot] = useState(0);
  const [loadedControlSlot, setLoadedControlSlot] = useState<number | null>(null);
  const [cameraControls, setCameraControls] = useState<CameraControl[]>([]);
  const [cameraFormats, setCameraFormats] = useState<CameraPreset[]>([]);
  const [cameraOpen, setCameraOpen] = useState(false);
  const [imageSource, setImageSource] = useState<SolveImageSource>(null);
  const [imageSrc, setImageSrc] = useState<string | null>(null);
  const [imageName, setImageName] = useState("实时相机画面");
  const [frameStats, setFrameStats] = useState("stream idle");
  const [naturalSize, setNaturalSize] = useState({ width: 0, height: 0 });
  /// 相机已打开时记下 grid MJPEG URL 与原始尺寸，便于"读取图片→切回相机"在不重开相机的情况下恢复视图
  const [cameraStreamUrl, setCameraStreamUrl] = useState<string | null>(null);
  const [cameraStreamSize, setCameraStreamSize] = useState<{ width: number; height: number } | null>(null);
  const [imageBox, setImageBox] = useState<ImageBox>({ left: 0, top: 0, width: 0, height: 0 });

  const [regions, setRegions] = useState<RoiRegion[]>(createDefaultRoiRegions);
  const [currentRegionId, setCurrentRegionId] = useState(firstRoiRegionId);
  const [annotationMode, setAnnotationMode] = useState(false);
  const [showRoi, setShowRoi] = useState(true);
  const [focusCurrentRoi, setFocusCurrentRoi] = useState(false);
  const [viewMenuOpen, setViewMenuOpen] = useState(false);
  const [mappingEditorOpen, setMappingEditorOpen] = useState(false);
  const [panelVisibility, setPanelVisibility] = useState<PanelVisibility>(() => {
    try {
      return createSavedPanelVisibility(JSON.parse(localStorage.getItem(panelVisibilityStorageKey) || "null"));
    } catch {
      return createInitialPanelVisibility();
    }
  });

  const [ports, setPorts] = useState<SerialPort[]>([]);
  const [selectedPort, setSelectedPort] = useState("");
  const [baudRate, setBaudRate] = useState(115200);
  const [serialOpen, setSerialOpen] = useState(false);

  const [status, setStatus] = useState("空闲");
  const [solverReady, setSolverReady] = useState(false);
  const [autoSaveImage, setAutoSaveImage] = useState(false);
  const [facelets, setFacelets] = useState(solvedFacelets);
  const [moves, setMoves] = useState<string[]>([]);
  const [steps, setSteps] = useState<string[]>([]);
  const [encodedSteps, setEncodedSteps] = useState("");
  const [solveStats, setSolveStats] = useState<SolveStats | null>(null);
  const [timerRunning, setTimerRunning] = useState(false);
  const [elapsedMs, setElapsedMs] = useState(0);
  const [logs, setLogs] = useState<LogItem[]>([{ time: nowTime(), text: "CubeSolver 已启动。", kind: "info" }]);

  const markedCount = regions.filter((region) => region.rect).length;
  const elapsedText = useMemo(() => {
    const centiseconds = Math.floor(elapsedMs / 10);
    const minutes = Math.floor(centiseconds / 6000);
    const seconds = Math.floor((centiseconds % 6000) / 100);
    const cs = centiseconds % 100;
    return `${minutes.toString().padStart(2, "0")}:${seconds.toString().padStart(2, "0")}.${cs
      .toString()
      .padStart(2, "0")}`;
  }, [elapsedMs]);
  const imageAspectLabel = useMemo(() => {
    if (!naturalSize.width || !naturalSize.height) return "-";
    return `${naturalSize.width} x ${naturalSize.height}`;
  }, [naturalSize]);
  const maxConfiguredFps = useMemo(() => Math.max(1, ...cameraConfigs.map((config) => config.fps || 1)), [cameraConfigs]);
  const canvasPreviewFps = useMemo(() => resolveCanvasPreviewFps(maxConfiguredFps), [maxConfiguredFps]);
  const activeCameraConfig = cameraConfigs[controlSlot] ?? cameraConfigs[0];
  const slotParamsVisible = loadedControlSlot === controlSlot;
  const activeCameraDevice = devices.find((device) => device.index === String(activeCameraConfig?.index ?? ""));
  const cameraProfileKey = useMemo(() => {
    if (!activeCameraDevice || !cameraFormats.length) return "";
    return getCameraProfileKey({
      name: activeCameraDevice.name,
      description: activeCameraDevice.description,
      formats: cameraFormats,
    });
  }, [activeCameraDevice, cameraFormats]);
  const currentRegionIndex = regions.findIndex((region) => region.id === currentRegionId);
  const currentRegion = regions[currentRegionIndex] ?? regions[0];
  const visibleRegions = focusCurrentRoi ? regions.filter((region) => region.id === currentRegionId) : regions;
  const showCanvasPreview = cameraOpen && imageSource === "camera";

  const addLog = (text: string, kind: LogItem["kind"] = "info") => {
    setLogs((items) => [{ time: nowTime(), text, kind }, ...items].slice(0, 120));
  };

  const addDiagnosticLog = (text: string, kind: LogItem["kind"] = "info") => {
    addLog(text, kind);
    void sendDiagnosticLog(text, invoke);
  };

  const isControlWritable = (control: CameraControl) =>
    control.active &&
    !control.flags.some((flag) => {
      const normalized = flag.toLowerCase();
      return normalized.includes("readonly") || normalized.includes("disabled");
    });

  const cameraControlStorageKey = (slot = controlSlot) => `cubesolver.camera-controls.slot-${slot}`;

  const saveCameraControls = () => {
    const values = cameraControls.map((control) => ({ id: control.id, value: control.value }));
    localStorage.setItem(cameraControlStorageKey(), JSON.stringify(values));
    addLog(`已保存槽 ${controlSlot + 1} 的相机参数。`);
    closeCameraControlsPanel();
  };

  const restoreDefaultCameraControls = async () => {
    const writableControls = cameraControls.filter(isControlWritable);
    for (const control of writableControls) {
      await setCameraControlValue(control, control.default);
    }
    addLog(`槽 ${controlSlot + 1} 已恢复默认参数。`);
    refreshCameraControls(controlSlot);
  };

  const refreshCameras = async () => {
    try {
      const nextDevices = await invoke<CameraDevice[]>("list_cameras");
      setDevices(nextDevices);
      addLog(`扫描到 ${nextDevices.length} 个相机。`);
    } catch (error) {
      addLog(String(error), "error");
    }
  };

  const refreshPorts = async () => {
    try {
      const nextPorts = await invoke<SerialPort[]>("list_serial_ports");
      setPorts(nextPorts);
      setSelectedPort((current) => current || nextPorts[0]?.name || "");
      addLog(`扫描到 ${nextPorts.length} 个串口。`);
    } catch (error) {
      addLog(String(error), "error");
    }
  };

  const refreshCameraControls = async (slot = controlSlot) => {
    setLoadedControlSlot(slot);
    const config = cameraConfigs[slot];
    try {
      if (config) {
        const formats = await invoke<CameraPreset[]>("list_camera_formats", { index: config.index });
        const supportedFormats = formats.filter(isSupportedPreviewFormat);
        setCameraFormats(supportedFormats.map((format) => ({ ...format, label: presetLabel(format) })));
        if (formats.length && !supportedFormats.length) {
          addLog("原生格式已读取，但没有符合 30 FPS 与分辨率过滤条件的格式。", "warn");
        }
      }
      if (!cameraOpen) {
        setCameraControls([]);
        addLog(`槽 ${slot + 1} 已读取相机格式；打开相机后可读取硬件控制参数。`);
        return;
      }
      // 流式预览运行中也能读硬件控制参数：后端走 CameraSlotWorker 的 control channel，
      // 在 capture 间隙读取，不需要关闭相机。
      const controls = await invoke<CameraControl[]>("list_camera_controls", { slot });
      setCameraControls(controls);
      addLog(`已读取槽 ${slot + 1} 的相机控制参数（${controls.length} 项）。`);
    } catch (error) {
      setCameraControls([]);
      setCameraFormats([]);
      addLog(String(error), "warn");
    }
  };

  useEffect(() => {
    refreshCameras();
    refreshPorts();
    const unlisten = listen("solver-ready", () => {
      setSolverReady(true);
    });
    // 主动查询 solver 状态（防止事件在前端渲染前已发出）
    const checkReady = () => {
      invoke<boolean>("check_solver_ready").then((ready) => {
        if (ready) setSolverReady(true);
      }).catch(() => {});
    };
    checkReady();
    const pollReady = setInterval(checkReady, 500);
    // 尝试加载默认 ROI 文件
    invoke<string | null>("load_default_roi").then((content) => {
      if (content) {
        try {
          const data = normalizeLoadedRoiRegions(JSON.parse(content), naturalSize);
          setRegions(data);
          setCurrentRegionId(data.find((region) => !region.rect)?.id ?? data[0].id);
          addLog("已自动加载默认 ROI: robot-roi.json");
        } catch (e) {
          addLog(`默认 ROI 解析失败: ${String(e)}`, "warn");
        }
      }
    }).catch(() => {});
    return () => {
      unlisten.then((f) => f());
      clearInterval(pollReady);
    };
  }, []);

  useEffect(() => {
    localStorage.setItem(panelVisibilityStorageKey, JSON.stringify(panelVisibility));
  }, [panelVisibility]);

  useEffect(() => {
    if (!timerRunning) return;
    const startAt = performance.now() - elapsedMs;
    const timer = window.setInterval(() => {
      setElapsedMs(performance.now() - startAt);
    }, 33);
    return () => window.clearInterval(timer);
  }, [timerRunning]);

  useEffect(() => {
    if (!serialOpen) return;
    let stopped = false;
    let busy = false;

    const tick = async () => {
      if (busy || stopped) return;
      busy = true;
      try {
        const result = await invoke<SerialReadResponse>("read_serial");
        if (!result.text) return;

        if (result.motion_finished) {
          setTimerRunning(false);
          addLog("收到结束信号 ND，计时已停止。");
        } else if (result.param_write_ok) {
          addLog("参数写入成功。");
        } else if (result.param_write_error) {
          addLog("参数写入失败，请检查设备连接。", "error");
        } else if (result.text.trim()) {
          addLog(`串口：${result.text.trim()}`);
        }
      } catch (error) {
        addLog(String(error), "warn");
      } finally {
        busy = false;
      }
    };

    const timer = window.setInterval(tick, 80);
    return () => {
      stopped = true;
      window.clearInterval(timer);
    };
  }, [serialOpen]);

  useEffect(() => {
    if (!imageSrc) {
      imageLoadTimingRef.current = null;
      lastImageSrcRef.current = null;
      lastLoggedImageBoxRef.current = { box: null, loggedAtMs: null };
      return;
    }

    if (lastImageSrcRef.current === imageSrc) return;

    if (imageSource === "camera") {
      imageLoadTimingRef.current = null;
      lastImageSrcRef.current = imageSrc;
      lastLoggedImageBoxRef.current = { box: null, loggedAtMs: null };
      addLog(`图像源切换：camera ${imageName}，使用画布拉取最新帧。`);
      return;
    }

    imageLoadTimingRef.current = { src: imageSrc, startedAtMs: performance.now() };
    lastImageSrcRef.current = imageSrc;
    lastLoggedImageBoxRef.current = { box: null, loggedAtMs: null };
    addLog(`图像源切换：${imageSource ?? "unknown"} ${imageName}，等待加载。`);
  }, [imageSrc, imageSource, imageName]);

  useEffect(() => {
    const updateImageBox = () => {
      const stage = stageRef.current;
      if (!stage || !naturalSize.width || !naturalSize.height) return;
      const stageRect = stage.getBoundingClientRect();
      const imageRatio = naturalSize.width / naturalSize.height;
      const stageRatio = stageRect.width / stageRect.height;
      const width = stageRatio > imageRatio ? stageRect.height * imageRatio : stageRect.width;
      const height = stageRatio > imageRatio ? stageRect.height : stageRect.width / imageRatio;
      const nextImageBox = {
        left: (stageRect.width - width) / 2,
        top: (stageRect.height - height) / 2,
        width,
        height,
      };
      setImageBox(nextImageBox);

      const nowMs = performance.now();
      const lastLogged = lastLoggedImageBoxRef.current;
      if (
        hasMaterialImageBoxChange(lastLogged.box, nextImageBox) &&
        shouldLogThrottledDiagnostic({
          nowMs,
          lastLoggedAtMs: lastLogged.loggedAtMs,
          intervalMs: imageLayoutDiagnosticThrottleMs,
        })
      ) {
        lastLoggedImageBoxRef.current = { box: nextImageBox, loggedAtMs: nowMs };
        addDiagnosticLog(createImageBoxDiagnostic(nextImageBox));
      }
    };

    updateImageBox();
    const observer = new ResizeObserver(updateImageBox);
    if (stageRef.current) observer.observe(stageRef.current);
    window.addEventListener("resize", updateImageBox);
    return () => {
      observer.disconnect();
      window.removeEventListener("resize", updateImageBox);
    };
  }, [naturalSize]);

  useEffect(() => {
    let disposed = false;
    let unlisten: (() => void) | null = null;
    listen<CameraStreamEvent>("camera-stream-event", (event) => {
      const payload = event.payload;
      if (payload.statuses.length) {
        setCameraStatuses((statuses) =>
          sameCameraStatuses(statuses, payload.statuses) ? statuses : payload.statuses,
        );
      }
      if (payload.kind === "frame" && payload.slot !== null) {
        const nowMs = performance.now();
        const previousFrameAtMs = lastCameraFrameAtBySlotRef.current[payload.slot] ?? null;
        lastCameraFrameAtBySlotRef.current[payload.slot] = nowMs;
        if (previousFrameAtMs !== null) {
          const gapMs = nowMs - previousFrameAtMs;
          const lastGapLoggedAtMs = lastCameraFrameGapLogAtBySlotRef.current[payload.slot] ?? null;
          if (
            isCameraFrameGapAbnormal({ gapMs, fps: payload.fps }) &&
            shouldLogThrottledDiagnostic({
              nowMs,
              lastLoggedAtMs: lastGapLoggedAtMs,
              intervalMs: cameraGapDiagnosticThrottleMs,
            })
          ) {
            lastCameraFrameGapLogAtBySlotRef.current[payload.slot] = nowMs;
            addDiagnosticLog(createCameraFrameGapDiagnostic({ slot: payload.slot, gapMs, fps: payload.fps }), "warn");
          }
        }
        const fps = payload.fps === null ? "-" : payload.fps.toFixed(1);
        const captureMs = payload.captureMs === null ? "-" : payload.captureMs.toString();
        const encodeMs = payload.encodeMs === null ? "-" : payload.encodeMs.toString();
        setFrameStats(`槽 ${payload.slot + 1}: ${fps} FPS / 抓帧 ${captureMs} ms / 编码 ${encodeMs} ms`);
      }
      if (payload.kind === "connected" && payload.slot !== null) {
        addLog(`相机槽 ${payload.slot + 1} / index ${payload.index} 已连接或恢复。`);
      }
      if (payload.kind === "disconnected" && payload.slot !== null) {
        addLog(`相机槽 ${payload.slot + 1} / index ${payload.index} 断联：${payload.message ?? ""}`, "warn");
      }
      if (payload.kind === "error" && payload.message) {
        addLog(payload.message, "error");
      }
    }).then((nextUnlisten) => {
      if (disposed) {
        nextUnlisten();
      } else {
        unlisten = nextUnlisten;
      }
    });
    return () => {
      disposed = true;
      unlisten?.();
    };
  }, []);

  useEffect(() => {
    if (!showCanvasPreview || !naturalSize.width || !naturalSize.height) return;

    const canvas = canvasRef.current;
    const context = canvas?.getContext("2d");
    if (!canvas || !context) return;

    canvas.width = naturalSize.width;
    canvas.height = naturalSize.height;

    let stopped = false;
    let frameRequest = 0;
    let inFlight = false;
    let lastRequestAtMs: number | null = null;

    const drawLatestFrame = async (requestAtMs: number) => {
      inFlight = true;
      lastRequestAtMs = requestAtMs;
      let objectUrl: string | null = null;
      let bitmap: ImageBitmap | null = null;

      try {
        const frame = await invoke<ArrayBuffer | Uint8Array | number[]>("snapshot_frame");
        if (stopped) return;

        const blob = new Blob([toFrameArrayBuffer(frame)], { type: "image/jpeg" });
        let drawable: CanvasImageSource;
        if ("createImageBitmap" in window) {
          bitmap = await createImageBitmap(blob);
          drawable = bitmap;
        } else {
          objectUrl = URL.createObjectURL(blob);
          drawable = await loadImageElement(objectUrl);
        }
        if (stopped) return;

        if (canvas.width !== naturalSize.width) canvas.width = naturalSize.width;
        if (canvas.height !== naturalSize.height) canvas.height = naturalSize.height;
        context.drawImage(drawable, 0, 0, naturalSize.width, naturalSize.height);
      } catch (error) {
        // 相机刚启动 / 切换槽位的瞬时窗口里，stream_hub 还没接到第一帧，
        // 后端会返 "no stream frame available"——这是预期，不刷屏到用户日志。
        const message = String(error);
        const isWarmup = message.includes("no stream frame available");
        const nowMs = performance.now();
        if (
          !stopped &&
          !isWarmup &&
          shouldLogThrottledDiagnostic({
            nowMs,
            lastLoggedAtMs: lastCanvasFrameLogAtRef.current,
            intervalMs: canvasFrameDiagnosticThrottleMs,
          })
        ) {
          lastCanvasFrameLogAtRef.current = nowMs;
          addDiagnosticLog(`画布预览帧失败：${message}`, "warn");
        }
      } finally {
        bitmap?.close();
        if (objectUrl) URL.revokeObjectURL(objectUrl);
        inFlight = false;
      }
    };

    const tick = (nowMs: number) => {
      if (stopped) return;
      if (shouldRequestCanvasFrame({ inFlight, lastRequestAtMs, nowMs, targetFps: canvasPreviewFps })) {
        void drawLatestFrame(nowMs);
      }
      frameRequest = requestAnimationFrame(tick);
    };

    frameRequest = requestAnimationFrame(tick);
    return () => {
      stopped = true;
      cancelAnimationFrame(frameRequest);
    };
  }, [showCanvasPreview, naturalSize.width, naturalSize.height, canvasPreviewFps]);

  const applyCameraConfig = async (index: number, patch: Partial<CameraConfig>) => {
    const next = cameraConfigs.map((item, itemIndex) => (itemIndex === index ? { ...item, ...patch } : item));
    setCameraConfigs(next);
    await reopenIfNeeded(next);
  };

  const updateCameraPreset = async (index: number, value: string) => {
    const preset = cameraPresets.find((item) => presetValue(item) === value);
    if (!preset) return;
    await applyCameraConfig(index, {
      width: preset.width,
      height: preset.height,
      fps: preset.fps,
      frameFormat: preset.frameFormat,
    });
  };

  const reopenIfNeeded = async (configs: CameraConfig[]) => {
    if (!cameraOpen) return;
    try {
      const stream = await invoke<CameraStreamInfo>("open_camera_stream", { configs });
      lastCameraFrameAtBySlotRef.current = {};
      lastCameraFrameGapLogAtBySlotRef.current = {};
      lastCanvasFrameLogAtRef.current = null;
      setImageSrc(stream.gridUrl);
      setImageSource("camera");
      setCameraStreamUrl(stream.gridUrl);
      setCameraStreamSize({ width: stream.width, height: stream.height });
      setNaturalSize({ width: stream.width, height: stream.height });
      setCameraStatuses(stream.statuses);
      setFrameStats("stream restarting");
      addLog(`相机流已按新配置重启，画布预览目标 ${canvasPreviewFps} FPS。`);
      refreshCameraControls(controlSlot);
    } catch (error) {
      addLog(String(error), "error");
    }
  };

  const selectCameraSlot = async (slot: number) => {
    if (swapSlot === null) {
      setSwapSlot(slot);
      addLog(`已选择槽 ${slot + 1}，再点另一个槽进行互换。`);
      return;
    }
    if (swapSlot === slot) {
      setSwapSlot(null);
      return;
    }
    const next = [...cameraConfigs];
    [next[swapSlot], next[slot]] = [next[slot], next[swapSlot]];
    setCameraConfigs(next);
    setSwapSlot(null);
    // 配置交换后流会被重启 → 当前展开的参数面板基于的是旧 slot 索引，
    // 折叠面板让用户看到"重新读取"提示，避免误以为还在调旧设备。
    if (loadedControlSlot !== null) {
      setLoadedControlSlot(null);
      setCameraControls([]);
      setCameraFormats([]);
    }
    await reopenIfNeeded(next);
  };

  const openCamera = async () => {
    try {
      const stream = await invoke<CameraStreamInfo>("open_camera_stream", { configs: cameraConfigs });
      lastCameraFrameAtBySlotRef.current = {};
      lastCameraFrameGapLogAtBySlotRef.current = {};
      lastCanvasFrameLogAtRef.current = null;
      setImageSrc(stream.gridUrl);
      setImageSource("camera");
      setCameraStreamUrl(stream.gridUrl);
      setCameraStreamSize({ width: stream.width, height: stream.height });
      setNaturalSize({ width: stream.width, height: stream.height });
      setCameraStatuses(stream.statuses);
      setImageName("实时相机流");
      setFrameStats("stream starting");
      setCameraOpen(true);
      setStatus("相机预览中");
      addLog(`相机流已启动。图像通过画布拉取最新合成帧，目标 ${canvasPreviewFps} FPS。`);
    } catch (error) {
      addLog(String(error), "error");
    }
  };

  const closeCamera = async () => {
    try {
      await invoke("close_camera_stream");
      setCameraOpen(false);
      setCameraStatuses([]);
      setCameraControls([]);
      setLoadedControlSlot(null);
      // 仅当当前展示的是相机画面时才清空展示；如果用户在看文件图片，保留它
      if (imageSource === "camera") {
        setImageSrc(null);
        setImageSource(null);
      }
      setCameraStreamUrl(null);
      setCameraStreamSize(null);
      lastCameraFrameAtBySlotRef.current = {};
      lastCameraFrameGapLogAtBySlotRef.current = {};
      lastCanvasFrameLogAtRef.current = null;
      setNaturalSize({ width: 0, height: 0 });
      setFrameStats("stream idle");
      setStatus("空闲");
      addLog("相机流已关闭。");
    } catch (error) {
      addLog(String(error), "error");
    }
  };

  const setCameraControlValue = async (control: CameraControl, value: number) => {
    // 乐观更新：先把 UI 滑块值切到 value，后端写参数失败时再 refresh 拉回真实值。
    setCameraControls((items) => items.map((item) => (item.id === control.id ? { ...item, value } : item)));
    try {
      await invoke("set_camera_control", { slot: controlSlot, id: control.id, value });
    } catch (error) {
      addLog(String(error), "error");
      refreshCameraControls(controlSlot);
    }
  };

  const selectControlSlot = (slot: number) => {
    setControlSlot(slot);
    setLoadedControlSlot(null);
    setCameraControls([]);
    setCameraFormats([]);
  };

  const closeCameraControlsPanel = () => {
    setLoadedControlSlot(null);
    setCameraControls([]);
    setCameraFormats([]);
    setSwapSlot(null);
  };

  const applyPanelPreset = (preset: ViewPreset) => {
    setPanelVisibility(applyViewPreset(preset));
    setViewMenuOpen(false);
  };

  const togglePanel = (panel: keyof PanelVisibility) => {
    setPanelVisibility((visibility) => togglePanelVisibility(visibility, panel));
  };

  const normalizedPointFromEvent = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!imageBox.width || !imageBox.height) return null;
    const stageRect = event.currentTarget.getBoundingClientRect();
    const x = (event.clientX - stageRect.left - imageBox.left) / imageBox.width;
    const y = (event.clientY - stageRect.top - imageBox.top) / imageBox.height;
    if (x < 0 || x > 1 || y < 0 || y > 1) return null;
    return { x, y };
  };

  const placeFixedRoi = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!annotationMode || !imageSrc || !naturalSize.width || !naturalSize.height) return;
    const point = normalizedPointFromEvent(event);
    if (!point) return;
    const rect = createFixedPixelRoi(point, naturalSize);
    setRegions((items) =>
      items.map((region) => (region.id === currentRegionId ? { ...region, rect } : region)),
    );
    addLog(`已标注 ${currentRegionId}：10 x 10 px。`);
    selectNextMissingRegion(currentRegionId);
  };

  const selectNextMissingRegion = (afterId = currentRegionId) => {
    const startIndex = regions.findIndex((region) => region.id === afterId);
    const ordered = [...regions.slice(startIndex + 1), ...regions.slice(0, startIndex + 1)];
    const next = ordered.find((region) => !region.rect);
    if (next) setCurrentRegionId(next.id);
  };

  const selectRegionByOffset = (offset: number) => {
    const safeRegionIndex = currentRegionIndex >= 0 ? currentRegionIndex : 0;
    const nextIndex = (safeRegionIndex + offset + regions.length) % regions.length;
    setCurrentRegionId(regions[nextIndex].id);
  };

  const clearCurrentRegion = () => {
    setRegions((items) =>
      items.map((region) => (region.id === currentRegionId ? { ...region, rect: null } : region)),
    );
    addLog(`已清除 ${currentRegionId}。`);
  };

  const getFrameRois = () =>
    [...regions].sort((left, right) => left.index - right.index).map((region) => ({
      x: Math.round((region.rect?.x || 0) * naturalSize.width),
      y: Math.round((region.rect?.y || 0) * naturalSize.height),
      width: Math.max(1, Math.round((region.rect?.w || 0) * naturalSize.width)),
      height: Math.max(1, Math.round((region.rect?.h || 0) * naturalSize.height)),
    }));

  const solveCurrentImage = async () => {
    if (markedCount < 54 || !naturalSize.width || !naturalSize.height) {
      addLog(`ROI 未完成：${markedCount}/54。`, "warn");
      return null;
    }

    setStatus("识别解算中");
    const request = createSolveFrameRequest({
      cameraOpen,
      imageSource,
      imageSrc,
      rois: getFrameRois(),
    });
    const result = await invoke<SolveResponse>(request.command, request.args);
    applySolveResult(result);
    addLog(request.successLog);

    // 异步保存解算图片和结果（不阻塞主流程）
    if (autoSaveImage) {
      const solveResultJson = JSON.stringify(result, null, 2);
      void (async () => {
        try {
          // 与 saveImage 同：相机视图下 imageSrc 是 MJPEG URL，不能直接当 data URL
          let dataUrl: string | null;
          if (imageSource === "camera") {
            dataUrl = await invoke<string>("latest_frame_data_url");
          } else if (imageSource === "file" && imageSrc) {
            dataUrl = imageSrc;
          } else if (cameraOpen) {
            dataUrl = await invoke<string>("latest_frame_data_url");
          } else {
            dataUrl = null;
          }
          if (dataUrl) {
            await invoke("save_solve_image", { imageDataUrl: dataUrl, solveResult: solveResultJson });
          }
        } catch (e) {
          addLog(`图片保存失败: ${String(e)}`, "warn");
        }
      })();
    }

    return result;
  };

  const solveFromFrame = async () => {
    try {
      await solveCurrentImage();
    } catch (error) {
      setStatus("解算失败");
      addLog(String(error), "error");
    }
  };

  const saveImage = async () => {
    try {
      // 相机视图下 imageSrc 是 MJPEG 流 URL（http://...grid.mjpeg），不是 data URL，
      // 不能直接喂给 base64 解码——必须从后端拉一帧 JPEG 转出的 data URL。
      // 文件视图下 imageSrc 已经是 data:image/...;base64,xxxx 形式，直接用即可。
      let dataUrl: string | null;
      if (imageSource === "camera") {
        dataUrl = await invoke<string>("latest_frame_data_url");
      } else if (imageSource === "file" && imageSrc) {
        dataUrl = imageSrc;
      } else if (cameraOpen) {
        // 兜底：相机开着但 imageSource 还没切到 'camera'
        dataUrl = await invoke<string>("latest_frame_data_url");
      } else {
        dataUrl = null;
      }
      if (!dataUrl) {
        addLog("没有可保存的图片。", "warn");
        return;
      }
      const path = await invoke<string>("save_solve_image", { imageDataUrl: dataUrl });
      addLog(`图片已保存: ${path}`);
    } catch (error) {
      addLog(`图片保存失败: ${String(error)}`, "error");
    }
  };

  const solveFromFacelets = async () => {
    try {
      setStatus("解算中");
      const result = await invoke<SolveResponse>("solve_facelets", { facelets });
      applySolveResult(result);
      addLog("已按 facelets 字符串解算。");
    } catch (error) {
      setStatus("解算失败");
      addLog(String(error), "error");
    }
  };

  const applySolveResult = (result: SolveResponse) => {
    setFacelets(result.facelets);
    setMoves(result.moves);
    setSteps(result.steps);
    setEncodedSteps(result.encoded_steps);
    const stats: SolveStats = {
      searchElapsedMs: result.search_elapsed_ms,
      handstepElapsedMs: result.handstep_elapsed_ms,
      mechSteps: result.mech_steps,
      candidateCount: result.candidate_count,
      faceMoves: result.moves.length,
    };
    const totalElapsedMs = stats.searchElapsedMs + stats.handstepElapsedMs;
    setSolveStats(stats);
    addLog(
      `解算完成：${stats.faceMoves} 步 Moves → ${result.steps.length} 步机械（mech=${stats.mechSteps}），求解总耗时 ${totalElapsedMs}ms（solver ${stats.searchElapsedMs}ms + handstep ${stats.handstepElapsedMs}ms，${stats.candidateCount} 候选）。`,
    );
    setStatus(`已生成步骤（${result.steps.length} 步 / ${totalElapsedMs}ms）`);
  };

  const openSerial = async () => {
    if (!selectedPort) {
      addLog("请先选择串口。", "warn");
      return;
    }
    try {
      await invoke("open_serial", { portName: selectedPort, baudRate });
      setSerialOpen(true);
      addLog(`串口 ${selectedPort} 已打开。`);
    } catch (error) {
      addLog(String(error), "error");
    }
  };

  const closeSerial = async () => {
    try {
      await invoke("close_serial");
      setSerialOpen(false);
      setTimerRunning(false);
      addLog("串口已关闭。");
    } catch (error) {
      addLog(String(error), "error");
    }
  };

  const sendToRobot = async () => {
    if (!encodedSteps) {
      addLog("没有可发送的步骤。", "warn");
      return;
    }
    try {
      await invoke("send_steps", { encodedSteps });
      setElapsedMs(0);
      setTimerRunning(true);
      addLog("步骤已发送到串口。");
    } catch (error) {
      addLog(String(error), "error");
    }
  };

  const runDirectly = async () => {
    try {
      const result = await solveCurrentImage();
      if (!result?.encoded_steps) return;
      await invoke("send_steps", { encodedSteps: result.encoded_steps });
      setElapsedMs(0);
      setTimerRunning(true);
      addLog("识别、解算、转换完成，步骤已发送到串口。");
    } catch (error) {
      setStatus("运行失败");
      addLog(String(error), "error");
    }
  };

  /// 读取本地图片：仅切换视图源到文件，**不**关闭相机硬件流。
  /// 相机继续在后端跑，下方 `restoreCameraView` 可以瞬时切回。
  const loadImageFile = async (file: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      setImageSource("file");
      setImageSrc(String(reader.result));
      setImageName(file.name);
      setFrameStats("file loaded");
      setStatus(cameraOpen ? "已读取图片（相机后台运行中）" : "已读取图片");
      addLog(
        cameraOpen
          ? `已读取图片：${file.name}（相机仍在后台运行，可点「切回相机」恢复实时画面）`
          : `已读取图片：${file.name}`,
      );
    };
    reader.onerror = () => {
      const msg = reader.error?.message ?? "unknown";
      addLog(`读取图片失败：${file.name}（${msg}）`, "error");
      setStatus("读取失败");
    };
    reader.readAsDataURL(file);
  };

  /// 在相机仍开启的情况下把视图切回相机流（不重新打开硬件，瞬时完成）。
  const restoreCameraView = () => {
    if (!cameraOpen || !cameraStreamUrl) {
      addLog("相机未开启，无法切回相机视图。", "warn");
      return;
    }
    setImageSrc(cameraStreamUrl);
    setImageSource("camera");
    if (cameraStreamSize) {
      setNaturalSize(cameraStreamSize);
    }
    setImageName("实时相机流");
    setFrameStats("stream resumed");
    setStatus("相机预览中");
    addLog("已切回相机实时画面。");
  };

  const handleStreamImageError = () => {
    if (cameraOpen) {
      addLog("相机 MJPEG 流加载失败，请关闭后重新打开相机。", "error");
    }
  };

  const handleRoiInput = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const data = normalizeLoadedRoiRegions(JSON.parse(String(reader.result)), naturalSize);
        setRegions(data);
        setCurrentRegionId(data.find((region) => !region.rect)?.id ?? data[0].id);
        addLog(`已读取 ROI：${file.name}`);
      } catch (error) {
        addLog(error instanceof Error ? error.message : String(error), "error");
      }
    };
    reader.readAsText(file);
    event.target.value = "";
  };

  const downloadText = (filename: string, text: string, type = "application/json") => {
    const url = URL.createObjectURL(new Blob([text], { type }));
    const link = document.createElement("a");
    link.href = url;
    link.download = filename;
    link.style.display = "none";
    document.body.appendChild(link);
    link.click();
    link.remove();
    window.setTimeout(() => URL.revokeObjectURL(url), 1_000);
  };

  const getSavedRoiText = () => {
    const hasNaturalSize = naturalSize.width > 0 && naturalSize.height > 0;
    const payload = hasNaturalSize
      ? createRobotAppRoiExport(regions, naturalSize)
      : [...regions].sort((left, right) => left.index - right.index);

    return {
      text: JSON.stringify(payload, null, 2),
      format: hasNaturalSize ? "RobotApp 像素格式" : "内部归一化格式",
    };
  };

  /// 弹原生保存对话框，默认目录 = 软件安装目录，默认文件名预填好。
  /// 用户保存到默认目录下的同名文件，下次启动会自动加载（见后端
  /// `load_default_roi` / `load_move_mapping_from_disk`）；保存到其他位置则
  /// 仅作导出/备份。返回用户选择的绝对路径，取消时返回 null。
  const promptSavePath = async (
    filenameHint: string,
    title: string,
  ): Promise<string | null> => {
    let defaultPath = filenameHint;
    try {
      const dirs = await invoke<{ install_dir: string }>("get_default_save_paths");
      // 用 / 拼接对 macOS / Linux 直接生效；Windows 上 dialog.save 也接受正斜杠
      defaultPath = `${dirs.install_dir}/${filenameHint}`;
    } catch {
      // 拿不到默认目录就只填文件名，让对话框用系统默认起点
    }
    const picked = await dialogSave({
      title,
      defaultPath,
      filters: [{ name: "JSON", extensions: ["json"] }],
    });
    return picked ?? null;
  };

  const saveRoi = async () => {
    const filename = "robot-roi.json";
    const { text, format } = getSavedRoiText();
    let target: string | null;
    try {
      target = await promptSavePath(filename, "保存 ROI");
    } catch (error) {
      addLog(`打开保存对话框失败：${String(error)}`, "error");
      return;
    }
    if (!target) {
      addLog("保存 ROI 已取消。");
      return;
    }
    try {
      const path = await invoke<string>("save_text_file_to_path", {
        path: target,
        contents: text,
      });
      addLog(`ROI 已保存：${path}（${format}）。`);
    } catch (error) {
      try {
        downloadText(filename, text);
        addLog(`Tauri 保存失败，已尝试浏览器下载：${String(error)}`, "warn");
        addLog(`已触发 ROI 下载：${filename}（${format}）。`);
      } catch (fallbackError) {
        addLog(`ROI 保存失败：${String(fallbackError)}`, "error");
      }
    }
  };

  return (
    <main className="robot-shell">
      <input
        ref={imageInputRef}
        type="file"
        accept="image/*"
        hidden
        onChange={(event) => {
          const file = event.target.files?.[0];
          if (file) void loadImageFile(file);
          event.target.value = "";
        }}
      />
      <input ref={roiInputRef} type="file" accept="application/json,.json" hidden onChange={handleRoiInput} />

      <header className="top-bar">
        <div>
          <h1>CubeSolver</h1>
          <span>{status}</span>
        </div>
        <div className="top-actions">
          <div className="view-menu">
            <button type="button" onClick={() => setViewMenuOpen((open) => !open)} aria-expanded={viewMenuOpen}>
              视图
            </button>
            {viewMenuOpen && (
              <>
              <div className="view-backdrop" onClick={() => setViewMenuOpen(false)} />
              <div className="view-popover">
                <div className="view-presets">
                  {(Object.keys(viewPresetLabels) as ViewPreset[]).map((preset) => (
                    <button type="button" key={preset} onClick={() => applyPanelPreset(preset)}>
                      {viewPresetLabels[preset]}
                    </button>
                  ))}
                </div>
                <div className="view-toggles">
                  {panelIds.map((panel) => (
                    <label key={panel}>
                      <input type="checkbox" checked={panelVisibility[panel]} onChange={() => togglePanel(panel)} />
                      <span>{panelLabels[panel]}</span>
                    </label>
                  ))}
                </div>
              </div>
              </>
            )}
          </div>
          {cameraOpen && imageSource === "file" ? (
            <button
              type="button"
              onClick={restoreCameraView}
              title="相机仍在后台运行；点击瞬时切回相机画面"
            >
              切回相机
            </button>
          ) : (
            <button type="button" onClick={() => imageInputRef.current?.click()}>
              读取图片
            </button>
          )}
          <button type="button" onClick={saveImage} disabled={!imageSrc && !cameraOpen}>
            保存图片
          </button>
          <button type="button" onClick={() => roiInputRef.current?.click()}>
            读取 ROI
          </button>
          <button type="button" onClick={() => void saveRoi()}>
            保存 ROI
          </button>
          <button type="button" onClick={() => setAnnotationMode((mode) => !mode)}>
            {annotationMode ? "退出标注" : "标注 ROI"}
          </button>
          <button
            type="button"
            onClick={() => {
              setRegions(createDefaultRoiRegions());
              setCurrentRegionId(firstRoiRegionId);
            }}
          >
            清空 ROI
          </button>
          <button
            type="button"
            title="编辑步骤映射（M_L1…M_RO → 0-9）"
            onClick={() => setMappingEditorOpen(true)}
          >
            步骤映射
          </button>
        </div>
      </header>

      <MoveMappingEditor
        open={mappingEditorOpen}
        onClose={() => setMappingEditorOpen(false)}
      />

      <section className="workspace">
        <section className="image-panel">
          <div className="panel-title">
            <div className="panel-heading">
              <h2>图像与 ROI</h2>
            </div>
            <div className="image-meta">
              <span>{imageName}</span>
              <span>{imageAspectLabel}</span>
              <span>{frameStats}</span>
              <span>ROI {markedCount}/54</span>
            </div>
          </div>

          <div
            ref={stageRef}
            className={`image-stage ${annotationMode ? "is-annotating" : ""}`}
            onPointerDown={placeFixedRoi}
          >
            {imageSrc ? (
              <>
                {showCanvasPreview ? (
                  <canvas
                    ref={canvasRef}
                    className="stitched-image"
                    style={{
                      left: imageBox.left,
                      top: imageBox.top,
                      width: imageBox.width,
                      height: imageBox.height,
                    }}
                  />
                ) : (
                  <img
                    alt="cube camera frame"
                    className="stitched-image"
                    src={imageSrc}
                    style={{
                      left: imageBox.left,
                      top: imageBox.top,
                      width: imageBox.width,
                      height: imageBox.height,
                    }}
                    onLoad={(event) => {
                      const nextSize = getLoadedImageSize(event.currentTarget);
                      setNaturalSize((size) => updateImageSize(size, nextSize));
                      const timing = imageLoadTimingRef.current;
                      const loadedAtMs = performance.now();
                      const durationMs = timing && timing.src === imageSrc ? loadedAtMs - timing.startedAtMs : 0;
                      addDiagnosticLog(
                        createImageLoadDiagnostic({
                          source: imageSource,
                          name: imageName,
                          durationMs,
                          width: nextSize.width,
                          height: nextSize.height,
                        }),
                      );
                    }}
                    onError={handleStreamImageError}
                  />
                )}
                {showRoi && (
                  <svg
                    className="roi-layer"
                    style={{
                      left: imageBox.left,
                      top: imageBox.top,
                      width: imageBox.width,
                      height: imageBox.height,
                    }}
                    viewBox="0 0 1 1"
                    preserveAspectRatio="none"
                  >
                    {visibleRegions.map((region) => {
                      if (!region.rect) return null;
                      const labelPosition = getRoiLabelPosition(region.rect);
                      return (
                        <g key={region.id} onClick={() => setCurrentRegionId(region.id)}>
                          <rect
                            className={region.id === currentRegionId ? "roi-rect is-active" : "roi-rect"}
                            x={region.rect.x}
                            y={region.rect.y}
                            width={region.rect.w}
                            height={region.rect.h}
                            vectorEffect="non-scaling-stroke"
                          />
                          <text
                            className="roi-label"
                            x={labelPosition.x}
                            y={labelPosition.y}
                            textAnchor="middle"
                            dominantBaseline="hanging"
                          >
                            {region.id}
                          </text>
                        </g>
                      );
                    })}
                  </svg>
                )}
              </>
            ) : (
              <div className="empty-image">
                <strong>等待相机画面</strong>
                <span>打开相机或读取一张图片后，可以在这里标注 54 个 ROI。</span>
              </div>
            )}
          </div>

          <div className="bottom-actions">
            <button type="button" onClick={cameraOpen ? closeCamera : openCamera}>
              {cameraOpen ? "关闭相机" : "打开相机"}
            </button>
            <button type="button" onClick={solveFromFrame} disabled={!solverReady}>
              {solverReady ? "识别并解算" : "解算器初始化中…"}
            </button>
            <button type="button" onClick={runDirectly} disabled={!solverReady}>
              {solverReady ? "直接运行" : "解算器初始化中…"}
            </button>
          </div>
        </section>

        <aside className="side-panel">
          {panelVisibility.timer && (
          <section className="panel-card timer-card">
            <div className="card-title">
              <h2>计时器</h2>
              <button
                type="button"
                onClick={() => {
                  setElapsedMs(0);
                  setTimerRunning(false);
                }}
              >
                复位
              </button>
            </div>
            <div className="timer-display">{elapsedText}</div>
            <div className="button-row">
              <button type="button" onClick={() => setTimerRunning(true)}>
                开始
              </button>
              <button type="button" onClick={() => setTimerRunning(false)}>
                停止
              </button>
            </div>
            <p className="hint-line">{timerRunning ? "运行中，等待下位机 ND 结束信号。" : "发送步骤后自动开始计时。"}</p>
          </section>
          )}

          {panelVisibility.solve && (
          <section className="panel-card solve-card">
            <div className="card-title">
              <h2>解算结果</h2>
              <button type="button" onClick={solveFromFacelets} disabled={!solverReady}>
                字符串解算
              </button>
            </div>
            <textarea value={facelets} onChange={(event) => setFacelets(event.target.value)} spellCheck={false} />
            <div className="solve-overview" aria-label="解算统计总览">
              {solveStats ? (
                <>
                  <div className="solve-stat">
                    <span className="solve-stat-label">求解总耗时</span>
                    <span
                      className="solve-stat-value"
                      title={`solver=${solveStats.searchElapsedMs}ms · handstep=${solveStats.handstepElapsedMs}ms`}
                    >
                      {solveStats.searchElapsedMs + solveStats.handstepElapsedMs} ms
                      <span className="solve-stat-sub">
                        solver={solveStats.searchElapsedMs}ms · hs={solveStats.handstepElapsedMs}ms
                      </span>
                    </span>
                  </div>
                  <div className="solve-stat">
                    <span className="solve-stat-label">机械步数</span>
                    <span className="solve-stat-value">
                      {steps.length}
                      <span className="solve-stat-sub">mech={solveStats.mechSteps}</span>
                    </span>
                  </div>
                  <div className="solve-stat">
                    <span className="solve-stat-label">Moves 步数</span>
                    <span className="solve-stat-value">{solveStats.faceMoves}</span>
                  </div>
                  <div className="solve-stat">
                    <span className="solve-stat-label">候选数</span>
                    <span className="solve-stat-value">{solveStats.candidateCount}</span>
                  </div>
                </>
              ) : (
                <div className="solve-overview-empty">尚未解算</div>
              )}
            </div>
            <div className="solve-summary">
              <div className="result-box">
                <label>Moves</label>
                <pre className="moves-output">{moves.join(" ") || "未生成"}</pre>
              </div>
              <div className="result-box">
                <label>Encoded</label>
                <pre className="encoded-output">{encodedSteps || "未生成"}</pre>
              </div>
            </div>
            <div className="result-box steps-box">
              <label>Steps</label>
              {steps.length ? (
                <ol className="steps-list">
                  {steps.map((step, index) => (
                    <li key={`${step}-${index}`}>{step}</li>
                  ))}
                </ol>
              ) : (
                <div className="empty-steps">未生成</div>
              )}
            </div>
          </section>
          )}

          {panelVisibility.camera && (
          <section className="panel-card">
            <div className="card-title">
              <h2>相机</h2>
              <button type="button" onClick={refreshCameras}>
                扫描
              </button>
            </div>
            <label className="auto-save-label">
              解算后自动保存图片和结果
              <input type="checkbox" checked={autoSaveImage} onChange={(e) => setAutoSaveImage(e.target.checked)} />
            </label>
            <div className="device-list">
              {devices.length ? (
                devices.map((device) => (
                  <span key={device.index}>
                    {device.index}: {device.name}
                  </span>
                ))
              ) : (
                <span>未发现相机</span>
              )}
            </div>
            <div className="slot-list">
              {cameraConfigs.map((config, index) => {
                const slotStatus = cameraStatuses.find((item) => item.slot === index);
                return (
                  <button
                    className={`slot-button ${swapSlot === index ? "is-selected" : ""}`}
                    type="button"
                    key={index}
                    title="点击两个槽位交换相机位置；交换会自动重启相机流"
                    onClick={() => selectCameraSlot(index)}
                  >
                    <span>槽 {index + 1}</span>
                    <strong>Index {config.index}</strong>
                    <em className={slotStatus?.connected ? "ok" : "bad"}>
                      {slotStatus ? (slotStatus.connected ? "在线" : "断联") : "未预览"}
                    </em>
                  </button>
                );
              })}
            </div>
            <div className="sub-card-title">
              <h3>槽位参数</h3>
              <div className="card-actions">
                <button type="button" onClick={() => refreshCameraControls(controlSlot)}>
                  读取
                </button>
                <button type="button" onClick={saveCameraControls} disabled={!cameraControls.length}>
                  保存
                </button>
                <button type="button" onClick={restoreDefaultCameraControls} disabled={!cameraControls.length}>
                  默认
                </button>
                <button type="button" onClick={closeCameraControlsPanel} disabled={!slotParamsVisible}>
                  关闭
                </button>
              </div>
            </div>
            <label className="field">
              <span>参数槽</span>
              <select value={controlSlot} onChange={(event) => selectControlSlot(Number(event.target.value))}>
                {cameraConfigs.map((_, index) => (
                  <option value={index} key={index}>
                    槽 {index + 1}
                  </option>
                ))}
              </select>
            </label>
            <p className="hint-line">
              {slotParamsVisible
                ? `参数调节界面已打开，调整后实时生效；当前最高配置 ${maxConfiguredFps} FPS。交换槽位会自动重启相机流并重置面板。`
                : "点击读取后展开该槽位的 Index、分辨率、FPS 和可调参数；格式列表只显示 30 FPS 及以上的常用分辨率。"}
            </p>
            <div className="camera-profile">
              <span>类型标识</span>
              <code title={cameraProfileKey || "读取槽位格式后生成"}>
                {cameraProfileKey || "读取格式后生成；不使用易变的 index"}
              </code>
            </div>
            {slotParamsVisible && activeCameraConfig && (
              <div className="slot-param-panel">
                <div className="camera-grid">
                  <label>
                    <span>Index</span>
                    <input
                      type="number"
                      min={0}
                      value={activeCameraConfig.index}
                      onChange={(event) => applyCameraConfig(controlSlot, { index: Number(event.target.value) })}
                    />
                  </label>
                  <label className="preset-field">
                    <span>格式</span>
                    <select
                      value={presetValue(activeCameraConfig)}
                      disabled={slotParamsVisible && !cameraFormats.length}
                      onChange={(event) => updateCameraPreset(controlSlot, event.target.value)}
                    >
                      {(slotParamsVisible ? cameraFormats : cameraPresets).length ? (
                        (slotParamsVisible ? cameraFormats : cameraPresets).map((preset) => (
                          <option value={presetValue(preset)} key={presetValue(preset)}>
                            {preset.label}
                          </option>
                        ))
                      ) : (
                        <option value={presetValue(activeCameraConfig)}>无符合条件的原生格式</option>
                      )}
                    </select>
                  </label>
                </div>
                <div className="control-list">
                  {cameraControls.length ? (
                    cameraControls.map((control) => {
                      const writable = isControlWritable(control);
                      return (
                        <label
                          className={`control-row ${writable ? "" : "is-disabled"}`}
                          key={control.id}
                          title={writable ? "" : `该参数不可调：${control.flags.join(", ") || "inactive"}`}
                        >
                          <span>{control.name || control.id}</span>
                          {control.kind === "boolean" ? (
                            <input
                              type="checkbox"
                              disabled={!writable}
                              checked={control.value >= 0.5}
                              onChange={(event) => setCameraControlValue(control, event.target.checked ? 1 : 0)}
                            />
                          ) : (
                            <input
                              type="range"
                              disabled={!writable}
                              min={control.min ?? control.value - 100}
                              max={control.max ?? control.value + 100}
                              step={control.step && control.step > 0 ? control.step : control.kind === "integer" ? 1 : 0.1}
                              value={control.value}
                              onChange={(event) => setCameraControlValue(control, Number(event.target.value))}
                            />
                          )}
                          <em>{control.kind === "boolean" ? (control.value >= 0.5 ? "开" : "关") : control.value.toFixed(2)}</em>
                        </label>
                      );
                    })
                  ) : (
                    <p className="hint-line">打开相机后可读取该槽位支持的硬件参数。</p>
                  )}
                </div>
              </div>
            )}
          </section>
          )}

          {panelVisibility.roi && (
          <section className="panel-card">
            <div className="card-title">
              <h2>ROI</h2>
              <div className="card-actions">
                <button type="button" onClick={() => setFocusCurrentRoi((focused) => !focused)}>
                  {focusCurrentRoi ? "显示全部" : "只看当前"}
                </button>
                <button type="button" onClick={() => setShowRoi((visible) => !visible)}>
                  {showRoi ? "隐藏" : "显示"}
                </button>
              </div>
            </div>
            <label className="field">
              <span>当前格</span>
              <select value={currentRegionId} onChange={(event) => setCurrentRegionId(event.target.value)}>
                {regions.map((region) => (
                  <option value={region.id} key={region.id}>
                    {region.id} {region.rect ? "已标注" : "未标注"}
                  </option>
                ))}
              </select>
            </label>
            <div className="roi-status">
              <strong>{currentRegion?.id}</strong>
              <span>{currentRegion?.rect ? "已标注，点击图像可重标覆盖" : "未标注，进入标注后点击图像放置 10 x 10 ROI"}</span>
            </div>
            <div className="button-row roi-actions">
              <button type="button" onClick={() => selectRegionByOffset(-1)}>
                上一格
              </button>
              <button type="button" onClick={() => selectRegionByOffset(1)}>
                下一格
              </button>
              <button
                type="button"
                onClick={() => {
                  setAnnotationMode(true);
                  setShowRoi(true);
                }}
              >
                标当前
              </button>
              <button type="button" onClick={clearCurrentRegion} disabled={!currentRegion?.rect}>
                清当前
              </button>
            </div>
          </section>
          )}

          {panelVisibility.serial && (
          <section className="panel-card">
            <div className="card-title">
              <h2>串口</h2>
              <button type="button" onClick={refreshPorts}>
                扫描
              </button>
            </div>
            <label className="field">
              <span>端口</span>
              <select value={selectedPort} onChange={(event) => setSelectedPort(event.target.value)}>
                {ports.map((port) => (
                  <option value={port.name} key={port.name}>
                    {port.name}
                  </option>
                ))}
              </select>
            </label>
            <label className="field">
              <span>波特率</span>
              <input type="number" min={1200} value={baudRate} onChange={(event) => setBaudRate(Number(event.target.value))} />
            </label>
            <div className="button-row">
              <button type="button" onClick={serialOpen ? closeSerial : openSerial}>
                {serialOpen ? "关闭" : "连接"}
              </button>
              <button type="button" onClick={sendToRobot}>
                发送
              </button>
            </div>
          </section>
          )}

          {panelVisibility.logs && (
          <section className="panel-card log-card">
            <h2>日志</h2>
            <div className="log-output">
              {logs.map((item, index) => (
                <p key={`${item.time}-${index}`} className={`log-line ${item.kind}`}>
                  <span>{item.time}</span>
                  {item.text}
                </p>
              ))}
            </div>
          </section>
          )}
        </aside>
      </section>
    </main>
  );
}

export default App;
