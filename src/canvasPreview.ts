export type CanvasFrameTiming = {
  fetchMs: number;
  decodeMs: number;
  drawMs: number;
  width: number;
  height: number;
};

export type CanvasFrameRequestState = {
  inFlight: boolean;
  lastRequestAtMs: number | null;
  nowMs: number;
  targetFps: number;
};

const defaultCanvasPreviewFps = 30;

const roundedMs = (value: number) => Math.round(value);

export const resolveCanvasPreviewFps = (configuredFps: number, maxFps = defaultCanvasPreviewFps) => {
  if (!Number.isFinite(configuredFps) || configuredFps <= 0) return maxFps;
  return Math.max(1, Math.min(maxFps, Math.floor(configuredFps)));
};

export const shouldRequestCanvasFrame = ({
  inFlight,
  lastRequestAtMs,
  nowMs,
  targetFps,
}: CanvasFrameRequestState) => {
  if (inFlight) return false;
  if (lastRequestAtMs === null) return true;

  const frameIntervalMs = 1_000 / Math.max(1, targetFps);
  return nowMs - lastRequestAtMs >= frameIntervalMs;
};

export const createCanvasFrameDiagnostic = ({
  fetchMs,
  decodeMs,
  drawMs,
  width,
  height,
}: CanvasFrameTiming) =>
  `画布预览帧：获取 ${roundedMs(fetchMs)} ms，解码 ${roundedMs(decodeMs)} ms，绘制 ${roundedMs(drawMs)} ms，${width} x ${height}。`;
