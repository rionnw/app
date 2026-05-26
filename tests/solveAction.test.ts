import { describe, expect, test } from "bun:test";

import { createSolveFrameRequest } from "../src/solveAction";

const rois = [{ x: 1, y: 2, width: 3, height: 4 }];

describe("solve action helpers", () => {
  test("uses the loaded image file when the image source is file", () => {
    expect(createSolveFrameRequest({ cameraOpen: false, imageSource: "file", imageSrc: "data:image/png;base64,abc", rois })).toEqual({
      command: "solve_image_file",
      args: { imageDataUrl: "data:image/png;base64,abc", rois },
      successLog: "当前文件图片已识别并解算。",
    });
  });

  test("uses latest stream frame when camera preview is open", () => {
    expect(createSolveFrameRequest({ cameraOpen: true, imageSource: "camera", imageSrc: "http://127.0.0.1/stream", rois })).toEqual({
      command: "solve_latest_frame",
      args: { rois },
      successLog: "当前相机帧已识别并解算。",
    });
  });
});
