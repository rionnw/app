import type { NaturalSize, RoiRect } from "./roiAnnotation";

export type RoiFace = "U" | "R" | "F" | "D" | "L" | "B";

export type RoiIndexView = {
  index: number;
  face: RoiFace;
  faceIndex: number;
  label: string;
};

export type RoiRegion = RoiIndexView & {
  id: string;
  rect: RoiRect | null;
};

export type RobotAppRoiExport = {
  rois: Array<{ x: number; y: number; width: number; height: number }>;
};

const faceOrder: RoiFace[] = ["U", "R", "F", "D", "L", "B"];

const originalFaceMapping = {
  B: [45, 46, 47, 48, 49, 50, 51, 52, 53],
  D: [11, 14, 17, 10, 13, 16, 9, 12, 15],
  F: [20, 23, 26, 19, 22, 25, 18, 21, 24],
  L: [0, 1, 2, 3, 4, 5, 6, 7, 8],
  R: [36, 37, 38, 39, 40, 41, 42, 43, 44],
  U: [29, 32, 35, 28, 31, 34, 27, 30, 33],
} satisfies Record<RoiFace, number[]>;

const createRoiIndexViews = () => {
  const views: Array<RoiIndexView | undefined> = Array.from({ length: 54 });

  for (const face of faceOrder) {
    originalFaceMapping[face].forEach((index, faceIndex) => {
      views[index] = {
        index,
        face,
        faceIndex,
        label: `${face}${faceIndex + 1}`,
      };
    });
  }

  return views.map((view, index) => {
    if (!view) throw new Error(`Missing ROI face mapping for original index ${index}.`);
    return view;
  });
};

const roiIndexViews = createRoiIndexViews();

const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === "object" && value !== null && !Array.isArray(value);

const toFiniteNumber = (value: unknown) => (typeof value === "number" && Number.isFinite(value) ? value : null);

const toRoiFace = (value: unknown): RoiFace | null =>
  typeof value === "string" && faceOrder.includes(value as RoiFace) ? (value as RoiFace) : null;

const isOriginalIndex = (value: number | null): value is number =>
  value !== null && Number.isInteger(value) && value >= 0 && value < 54;

const isFaceIndex = (value: number | null): value is number =>
  value !== null && Number.isInteger(value) && value >= 0 && value < 9;

const viewFromFaceIndex = (face: RoiFace, faceIndex: number) => getRoiIndexView(originalFaceMapping[face][faceIndex]);

const parseLabelView = (label: unknown) => {
  if (typeof label !== "string") return null;
  const match = /^([URFDLB])([1-9])$/.exec(label);
  if (!match) return null;
  return viewFromFaceIndex(match[1] as RoiFace, Number(match[2]) - 1);
};

const getRegionView = (value: unknown, arrayIndex: number) => {
  if (!isRecord(value)) return getRoiIndexView(arrayIndex);

  const face = toRoiFace(value.face);
  const regionIndex = toFiniteNumber(value.index);
  const explicitFaceIndex = toFiniteNumber(value.faceIndex);
  const id = typeof value.id === "string" ? value.id : null;

  if (face && isFaceIndex(explicitFaceIndex)) return viewFromFaceIndex(face, explicitFaceIndex);

  if (face && regionIndex !== null && regionIndex >= 1 && regionIndex <= 9 && id === `${face}${regionIndex}`) {
    return viewFromFaceIndex(face, regionIndex - 1);
  }

  const labelView = parseLabelView(id ?? value.label);
  if (labelView) return labelView;

  if (isOriginalIndex(regionIndex)) return getRoiIndexView(regionIndex);

  return getRoiIndexView(arrayIndex);
};

const normalizeRect = (value: unknown, naturalSize?: NaturalSize): RoiRect | null => {
  if (!isRecord(value)) return null;

  const rectSource = isRecord(value.rect) ? value.rect : value;
  const x = toFiniteNumber(rectSource.x);
  const y = toFiniteNumber(rectSource.y);
  const w = toFiniteNumber(rectSource.w) ?? toFiniteNumber(rectSource.width);
  const h = toFiniteNumber(rectSource.h) ?? toFiniteNumber(rectSource.height);

  if (x === null || y === null || w === null || h === null) return null;
  if (x <= 1 && y <= 1 && w <= 1 && h <= 1) return { x, y, w, h };
  if (!naturalSize?.width || !naturalSize.height) return null;

  return {
    x: x / naturalSize.width,
    y: y / naturalSize.height,
    w: w / naturalSize.width,
    h: h / naturalSize.height,
  };
};

export const getRoiIndexView = (index: number): RoiIndexView => {
  if (!Number.isInteger(index) || index < 0 || index >= roiIndexViews.length) {
    throw new RangeError(`ROI index must be an integer from 0 to 53; received ${index}.`);
  }

  return roiIndexViews[index];
};

export const createDefaultRoiRegions = (): RoiRegion[] =>
  roiIndexViews.map((view) => ({
    ...view,
    id: view.label,
    rect: null,
  }));

export const normalizeLoadedRoiRegions = (input: unknown, naturalSize?: NaturalSize): RoiRegion[] => {
  const items = Array.isArray(input) ? input : isRecord(input) && Array.isArray(input.rois) ? input.rois : null;
  if (!items || items.length !== 54) throw new Error("ROI 数量必须是 54。");

  const regions = createDefaultRoiRegions();
  items.forEach((item, arrayIndex) => {
    const view = getRegionView(item, arrayIndex);
    regions[view.index] = {
      ...view,
      id: view.label,
      rect: normalizeRect(item, naturalSize),
    };
  });

  return regions;
};

export const createRobotAppRoiExport = (regions: RoiRegion[], naturalSize: NaturalSize): RobotAppRoiExport => ({
  rois: [...regions].sort((left, right) => left.index - right.index).map((region) => ({
    x: Math.round((region.rect?.x || 0) * naturalSize.width),
    y: Math.round((region.rect?.y || 0) * naturalSize.height),
    width: Math.max(1, Math.round((region.rect?.w || 0) * naturalSize.width)),
    height: Math.max(1, Math.round((region.rect?.h || 0) * naturalSize.height)),
  })),
});
