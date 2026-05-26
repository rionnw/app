type ImageDiagnosticSource = "camera" | "file" | null;

type ImageLoadDiagnosticInput = {
  source: ImageDiagnosticSource;
  name: string;
  durationMs: number;
  width: number;
  height: number;
};

type ImageBoxDiagnosticInput = {
  left: number;
  top: number;
  width: number;
  height: number;
};

type ThrottledDiagnosticInput = {
  nowMs: number;
  lastLoggedAtMs: number | null;
  intervalMs: number;
};

type CameraFrameGapInput = {
  gapMs: number;
  fps: number | null;
};

type CameraFrameGapDiagnosticInput = CameraFrameGapInput & {
  slot: number;
};

type DiagnosticLogInvoke = (command: string, args: { message: string }) => Promise<unknown>;

const imageBoxMaterialChangePx = 2;
const minimumCameraFrameGapWarningMs = 1_000;
const cameraFrameGapWarningFrameCount = 6;

const roundedMs = (value: number) => Math.round(value);

export const createImageLoadDiagnostic = ({
  source,
  name,
  durationMs,
  width,
  height,
}: ImageLoadDiagnosticInput) =>
  `图像加载完成：${source ?? "unknown"} ${name}，${width} x ${height}，用时 ${roundedMs(durationMs)} ms。`;

export const createImageBoxDiagnostic = ({ left, top, width, height }: ImageBoxDiagnosticInput) =>
  `图像布局更新：显示框 ${Math.round(width)} x ${Math.round(height)}，偏移 ${Math.round(left)},${Math.round(top)}。`;

export const hasMaterialImageBoxChange = (
  current: ImageBoxDiagnosticInput | null,
  next: ImageBoxDiagnosticInput,
  thresholdPx = imageBoxMaterialChangePx,
) =>
  current === null ||
  Math.abs(current.left - next.left) >= thresholdPx ||
  Math.abs(current.top - next.top) >= thresholdPx ||
  Math.abs(current.width - next.width) >= thresholdPx ||
  Math.abs(current.height - next.height) >= thresholdPx;

export const shouldLogThrottledDiagnostic = ({ nowMs, lastLoggedAtMs, intervalMs }: ThrottledDiagnosticInput) =>
  lastLoggedAtMs === null || nowMs - lastLoggedAtMs >= intervalMs;

export const isCameraFrameGapAbnormal = ({ gapMs, fps }: CameraFrameGapInput) => {
  const expectedFrameMs = 1_000 / Math.max(1, fps ?? 1);
  return gapMs >= Math.max(minimumCameraFrameGapWarningMs, expectedFrameMs * cameraFrameGapWarningFrameCount);
};

export const createCameraFrameGapDiagnostic = ({ slot, gapMs, fps }: CameraFrameGapDiagnosticInput) => {
  const fpsLabel = fps === null ? "-" : fps.toFixed(1);
  return `相机帧间隔偏大：槽 ${slot + 1} 间隔 ${roundedMs(gapMs)} ms，事件 FPS ${fpsLabel}。`;
};

export const sendDiagnosticLog = async (message: string, invokeCommand: DiagnosticLogInvoke) => {
  try {
    await invokeCommand("diagnostic_log", { message });
  } catch {
    // Diagnostics must never interrupt the UI log path.
  }
};
