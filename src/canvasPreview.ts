export type CanvasFrameRequestState = {
  inFlight: boolean;
  lastRequestAtMs: number | null;
  nowMs: number;
  targetFps: number;
};

const defaultCanvasPreviewFps = 30;

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
