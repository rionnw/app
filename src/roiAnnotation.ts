export type NormalizedPoint = { x: number; y: number };
export type NaturalSize = { width: number; height: number };
export type RoiRect = { x: number; y: number; w: number; h: number };

export const fixedRoiSizePx = 10;

const clamp = (value: number, min: number, max: number) => Math.min(max, Math.max(min, value));

export const createFixedPixelRoi = (point: NormalizedPoint, naturalSize: NaturalSize): RoiRect => {
  const width = fixedRoiSizePx / naturalSize.width;
  const height = fixedRoiSizePx / naturalSize.height;

  return {
    x: clamp(point.x - width / 2, 0, 1 - width),
    y: clamp(point.y - height / 2, 0, 1 - height),
    w: width,
    h: height,
  };
};
