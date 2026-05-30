import { describe, expect, test } from "bun:test";

import { resolveCanvasPreviewFps, shouldRequestCanvasFrame } from "../src/canvasPreview";

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
});
