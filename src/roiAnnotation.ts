export type NormalizedPoint = { x: number; y: number };
export type NaturalSize = { width: number; height: number };
export type RoiRect = { x: number; y: number; w: number; h: number };

export const fixedRoiSizePx = 10;

/// 标注归一化基准：与后端 `decode_image_data_url` 强制 resize 到的相机 grid
/// 尺寸 1280×960 保持一致；任何 ROI 创建/导出/加载的归一化都以此为锚点，
/// 这样文件模式 `<img>` 的真实分辨率仅决定视觉布局，不影响 ROI 几何。
export const ROI_REFERENCE_SIZE: NaturalSize = { width: 1280, height: 960 };

const clamp = (value: number, min: number, max: number) => Math.min(max, Math.max(min, value));

export const createFixedPixelRoi = (
  point: NormalizedPoint,
  referenceSize: NaturalSize = ROI_REFERENCE_SIZE,
): RoiRect => {
  const width = fixedRoiSizePx / referenceSize.width;
  const height = fixedRoiSizePx / referenceSize.height;

  return {
    x: clamp(point.x - width / 2, 0, 1 - width),
    y: clamp(point.y - height / 2, 0, 1 - height),
    w: width,
    h: height,
  };
};
