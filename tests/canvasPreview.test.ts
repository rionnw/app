import { describe, expect, test } from "bun:test";

import {
  createCanvasFrameDiagnostic,
  resolveCanvasPreviewFps,
  shouldRequestCanvasFrame,
} from "../src/canvasPreview";

describe("canvas preview helpers", () => {
  test("caps camera canvas preview to a controllable target fps", () => {
    expect(resolveCanvasPreviewFps(60)).toBe(30);
    expect(resolveCanvasPreviewFps(20)).toBe(20);
    expect(resolveCanvasPreviewFps(0)).toBe(30);
  });

  test("skips ticks while a canvas frame is already in flight", () => {
    expect(shouldRequestCanvasFrame({ inFlight: true, lastRequestAtMs: null, nowMs: 100, targetFps: 30 })).toBe(false);
  });

  test("paces frame requests without queueing extra frames", () => {
    expect(shouldRequestCanvasFrame({ inFlight: false, lastRequestAtMs: null, nowMs: 100, targetFps: 30 })).toBe(true);
    expect(shouldRequestCanvasFrame({ inFlight: false, lastRequestAtMs: 100, nowMs: 120, targetFps: 30 })).toBe(false);
    expect(shouldRequestCanvasFrame({ inFlight: false, lastRequestAtMs: 100, nowMs: 134, targetFps: 30 })).toBe(true);
  });

  test("formats throttled canvas frame diagnostics", () => {
    expect(
      createCanvasFrameDiagnostic({
        fetchMs: 5.4,
        decodeMs: 7.5,
        drawMs: 1.2,
        width: 1280,
        height: 960,
      }),
    ).toBe("画布预览帧：获取 5 ms，解码 8 ms，绘制 1 ms，1280 x 960。");
  });
});
