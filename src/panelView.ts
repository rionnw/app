export type PanelId = "timer" | "camera" | "roi" | "serial" | "solve" | "logs";
export type ViewPreset = "all" | "calibration" | "solve" | "run";

export type PanelVisibility = Record<PanelId, boolean>;

export type CameraProfileInput = {
  name: string;
  description: string;
  formats: Array<{
    width: number;
    height: number;
    fps: number;
    frameFormat: string;
  }>;
};

export const panelIds: PanelId[] = ["timer", "camera", "roi", "serial", "solve", "logs"];

export const panelLabels: Record<PanelId, string> = {
  timer: "计时器",
  camera: "相机",
  roi: "ROI",
  serial: "串口",
  solve: "解算结果",
  logs: "日志",
};

export const viewPresetLabels: Record<ViewPreset, string> = {
  all: "全部",
  calibration: "标定",
  solve: "解算",
  run: "运行",
};

export const createInitialPanelVisibility = (): PanelVisibility =>
  panelIds.reduce(
    (visibility, id) => ({
      ...visibility,
      [id]: true,
    }),
    {} as PanelVisibility,
  );

export const createSavedPanelVisibility = (saved: unknown): PanelVisibility => {
  const initial = createInitialPanelVisibility();
  if (!saved || typeof saved !== "object") return initial;
  const savedRecord = saved as Record<string, unknown>;

  return panelIds.reduce(
    (visibility, id) => ({
      ...visibility,
      [id]: typeof savedRecord[id] === "boolean" ? savedRecord[id] : true,
    }),
    {} as PanelVisibility,
  );
};

export const applyViewPreset = (preset: ViewPreset): PanelVisibility => {
  if (preset === "calibration") {
    return {
      timer: false,
      camera: true,
      roi: true,
      serial: false,
      solve: false,
      logs: true,
    };
  }

  if (preset === "solve") {
    return {
      timer: true,
      camera: false,
      roi: false,
      serial: true,
      solve: true,
      logs: true,
    };
  }

  if (preset === "run") {
    return {
      timer: true,
      camera: false,
      roi: false,
      serial: true,
      solve: true,
      logs: false,
    };
  }

  return createInitialPanelVisibility();
};

export const togglePanelVisibility = (visibility: PanelVisibility, panel: PanelId): PanelVisibility => ({
  ...visibility,
  [panel]: !visibility[panel],
});

export const getCameraProfileKey = ({ name, description, formats }: CameraProfileInput) => {
  const formatKey = [...formats]
    .sort(
      (left, right) =>
        left.width - right.width ||
        left.height - right.height ||
        left.fps - right.fps ||
        left.frameFormat.localeCompare(right.frameFormat),
    )
    .map((format) => `${format.width}x${format.height}@${format.fps}:${format.frameFormat}`)
    .join(",");

  return [name.trim(), description.trim(), formatKey].join("|");
};
