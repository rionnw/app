import { describe, expect, test } from "bun:test";

import {
  createDefaultRoiRegions,
  createRobotAppRoiExport,
  getRoiIndexView,
  normalizeLoadedRoiRegions,
} from "../src/roiIndexView";

describe("ROI index view mapping", () => {
  test("labels original ROI indices from the authoritative face mapping", () => {
    expect(Array.from({ length: 54 }, (_, index) => getRoiIndexView(index).label)).toEqual([
      "L1",
      "L2",
      "L3",
      "L4",
      "L5",
      "L6",
      "L7",
      "L8",
      "L9",
      "D7",
      "D4",
      "D1",
      "D8",
      "D5",
      "D2",
      "D9",
      "D6",
      "D3",
      "F7",
      "F4",
      "F1",
      "F8",
      "F5",
      "F2",
      "F9",
      "F6",
      "F3",
      "U7",
      "U4",
      "U1",
      "U8",
      "U5",
      "U2",
      "U9",
      "U6",
      "U3",
      "R1",
      "R2",
      "R3",
      "R4",
      "R5",
      "R6",
      "R7",
      "R8",
      "R9",
      "B1",
      "B2",
      "B3",
      "B4",
      "B5",
      "B6",
      "B7",
      "B8",
      "B9",
    ]);
    expect(getRoiIndexView(0)).toEqual({ index: 0, face: "L", faceIndex: 0, label: "L1" });
    expect(getRoiIndexView(33)).toEqual({ index: 33, face: "U", faceIndex: 8, label: "U9" });
    expect(getRoiIndexView(53)).toEqual({ index: 53, face: "B", faceIndex: 8, label: "B9" });
  });

  test("creates default regions in original ROI array order", () => {
    const regions = createDefaultRoiRegions();

    expect(regions).toHaveLength(54);
    expect(regions.map((region) => region.index)).toEqual(Array.from({ length: 54 }, (_, index) => index));
    expect(regions[0]).toMatchObject({ id: "L1", face: "L", faceIndex: 0, index: 0, rect: null });
    expect(regions[30]).toMatchObject({ id: "U8", face: "U", faceIndex: 7, index: 30, rect: null });
    expect(regions[33]).toMatchObject({ id: "U9", face: "U", faceIndex: 8, index: 33, rect: null });
  });

  test("normalizes legacy face-ordered saved regions into original ROI array order", () => {
    const legacyRegions = ["U", "R", "F", "D", "L", "B"].flatMap((face) =>
      Array.from({ length: 9 }, (_, index) => ({
        id: `${face}${index + 1}`,
        face,
        index: index + 1,
        rect: { x: (index + 1) / 100, y: 0.2, w: 0.03, h: 0.04 },
      })),
    );

    const regions = normalizeLoadedRoiRegions(legacyRegions);

    expect(regions.map((region) => region.index)).toEqual(Array.from({ length: 54 }, (_, index) => index));
    expect(regions[29]).toMatchObject({ id: "U1", face: "U", faceIndex: 0, rect: legacyRegions[0].rect });
    expect(regions[0]).toMatchObject({ id: "L1", face: "L", faceIndex: 0, rect: legacyRegions[36].rect });
    expect(regions[45]).toMatchObject({ id: "B1", face: "B", faceIndex: 0, rect: legacyRegions[45].rect });
  });

  test("normalizes RobotApp roi.json pixel rectangles in original array order", () => {
    const rois = Array.from({ length: 54 }, (_, index) => ({
      x: index * 10,
      y: index * 5,
      width: 10,
      height: 10,
    }));

    const regions = normalizeLoadedRoiRegions({ rois }, { width: 1000, height: 500 });

    expect(regions[0]).toMatchObject({ id: "L1", rect: { x: 0, y: 0, w: 0.01, h: 0.02 } });
    expect(regions[33]).toMatchObject({ id: "U9", rect: { x: 0.33, y: 0.33, w: 0.01, h: 0.02 } });
  });

  test("exports RobotApp pixel rectangles in original ROI array order", () => {
    const regions = createDefaultRoiRegions().map((region) => ({
      ...region,
      rect: {
        x: region.index / 100,
        y: region.index / 200,
        w: 0.01,
        h: 0.02,
      },
    }));

    const exportData = createRobotAppRoiExport(regions, { width: 1000, height: 500 });

    expect(exportData.rois).toHaveLength(54);
    expect(exportData.rois[0]).toEqual({ x: 0, y: 0, width: 10, height: 10 });
    expect(exportData.rois[33]).toEqual({ x: 330, y: 83, width: 10, height: 10 });
    expect(exportData.rois[53]).toEqual({ x: 530, y: 133, width: 10, height: 10 });
  });
});
