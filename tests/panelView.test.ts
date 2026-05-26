import { describe, expect, test } from "bun:test";
import {
  applyViewPreset,
  createInitialPanelVisibility,
  createSavedPanelVisibility,
  getCameraProfileKey,
  panelIds,
  togglePanelVisibility,
} from "../src/panelView";

describe("panel view helpers", () => {
  test("starts with every right panel visible", () => {
    expect(createInitialPanelVisibility()).toEqual({
      timer: true,
      camera: true,
      roi: true,
      serial: true,
      solve: true,
      logs: true,
    });
  });

  test("applies focused presets without losing panel keys", () => {
    const solveView = applyViewPreset("solve");

    expect(Object.keys(solveView)).toEqual(panelIds);
    expect(solveView.solve).toBe(true);
    expect(solveView.serial).toBe(true);
    expect(solveView.timer).toBe(true);
    expect(solveView.camera).toBe(false);
    expect(solveView.roi).toBe(false);
  });

  test("toggles one panel while keeping the rest unchanged", () => {
    const next = togglePanelVisibility(createInitialPanelVisibility(), "camera");

    expect(next.camera).toBe(false);
    expect(next.solve).toBe(true);
    expect(next.logs).toBe(true);
  });

  test("normalizes saved visibility and fills missing panel keys", () => {
    expect(createSavedPanelVisibility({ solve: false, camera: false, madeUp: true })).toEqual({
      timer: true,
      camera: false,
      roi: true,
      serial: true,
      solve: false,
      logs: true,
    });
  });

  test("builds a stable camera type profile key without using the volatile index", () => {
    expect(
      getCameraProfileKey({
        name: "USB Camera",
        description: "UVC Camera",
        formats: [
          { width: 640, height: 480, fps: 30, frameFormat: "MJPEG" },
          { width: 1280, height: 720, fps: 30, frameFormat: "MJPEG" },
        ],
      }),
    ).toBe("USB Camera|UVC Camera|640x480@30:MJPEG,1280x720@30:MJPEG");
  });
});
