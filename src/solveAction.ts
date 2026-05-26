export type SolveImageSource = "camera" | "file" | null;

export type SolveRoi = {
  x: number;
  y: number;
  width: number;
  height: number;
};

type SolveFrameRequestInput = {
  cameraOpen: boolean;
  imageSource: SolveImageSource;
  imageSrc: string | null;
  rois: SolveRoi[];
};

export const createSolveFrameRequest = ({ cameraOpen, imageSource, imageSrc, rois }: SolveFrameRequestInput) => {
  if (imageSource === "file" && imageSrc) {
    return {
      command: "solve_image_file",
      args: { imageDataUrl: imageSrc, rois },
      successLog: "当前文件图片已识别并解算。",
    };
  }

  return {
    command: cameraOpen ? "solve_latest_frame" : "solve_current_frame",
    args: { rois },
    successLog: "当前相机帧已识别并解算。",
  };
};
