export type ImageSize = {
  width: number;
  height: number;
};

export const getLoadedImageSize = (image: Pick<HTMLImageElement, "naturalWidth" | "naturalHeight">): ImageSize => ({
  width: image.naturalWidth,
  height: image.naturalHeight,
});

export const updateImageSize = (current: ImageSize, next: ImageSize): ImageSize =>
  current.width === next.width && current.height === next.height ? current : next;
