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
      "L3",
      "L6",
      "L9",
      "L2",
      "L5",
      "L8",
      "L1",
      "L4",
      "L7",
      "B3",
      "B6",
      "B9",
      "B2",
      "B5",
      "B8",
      "B1",
      "B4",
      "B7",
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
      "R7",
      "R4",
      "R1",
      "R8",
      "R5",
      "R2",
      "R9",
      "R6",
      "R3",
      "U9",
      "U8",
      "U7",
      "U6",
      "U5",
      "U4",
      "U3",
      "U2",
      "U1",
    ]);
    expect(getRoiIndexView(0)).toEqual({ index: 0, face: "L", faceIndex: 2, label: "L3" });
    expect(getRoiIndexView(33)).toEqual({ index: 33, face: "F", faceIndex: 8, label: "F9" });
    expect(getRoiIndexView(53)).toEqual({ index: 53, face: "U", faceIndex: 0, label: "U1" });
  });

  test("creates default regions in original ROI array order", () => {
    const regions = createDefaultRoiRegions();

    expect(regions).toHaveLength(54);
    expect(regions.map((region) => region.index)).toEqual(Array.from({ length: 54 }, (_, index) => index));
    expect(regions[0]).toMatchObject({ id: "L3", face: "L", faceIndex: 2, index: 0, rect: null });
    expect(regions[30]).toMatchObject({ id: "F8", face: "F", faceIndex: 7, index: 30, rect: null });
    expect(regions[33]).toMatchObject({ id: "F9", face: "F", faceIndex: 8, index: 33, rect: null });
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
    expect(regions[53]).toMatchObject({ id: "U1", face: "U", faceIndex: 0, rect: legacyRegions[0].rect });
    expect(regions[6]).toMatchObject({ id: "L1", face: "L", faceIndex: 0, rect: legacyRegions[36].rect });
    expect(regions[15]).toMatchObject({ id: "B1", face: "B", faceIndex: 0, rect: legacyRegions[45].rect });
  });

  test("normalizes RobotApp roi.json pixel rectangles in original array order", () => {
    const rois = Array.from({ length: 54 }, (_, index) => ({
      x: index * 10,
      y: index * 5,
      width: 10,
      height: 10,
    }));

    const regions = normalizeLoadedRoiRegions({ rois }, { width: 1000, height: 500 });

    expect(regions[0]).toMatchObject({ id: "L3", rect: { x: 0, y: 0, w: 0.01, h: 0.02 } });
    expect(regions[33]).toMatchObject({ id: "F9", rect: { x: 0.33, y: 0.33, w: 0.01, h: 0.02 } });
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
