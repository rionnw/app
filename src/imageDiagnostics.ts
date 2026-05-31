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

type DiagnosticLogInvoke = (command: string, args: { message: string }) => Promise<unknown>;

const imageBoxMaterialChangePx = 2;

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

// 旧的 isCameraFrameGapAbnormal / createCameraFrameGapDiagnostic 已移除：
// 它们基于"前端事件 timestamp"算 gap，但后端把 camera-stream-event(frame)
// 节流到 1Hz 给前端，前端测出来的 gap 永远是 ~1000ms，必然误报。
// 现在改由后端 worker 真实 capture-to-capture 间隔触发 camera-frame-gap-warning
// 事件，前端在 App.tsx 里直接 listen 后写入日志。

export const sendDiagnosticLog = async (message: string, invokeCommand: DiagnosticLogInvoke) => {
  try {
    await invokeCommand("diagnostic_log", { message });
  } catch {
    // Diagnostics must never interrupt the UI log path.
  }
};
