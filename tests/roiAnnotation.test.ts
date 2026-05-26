import { describe, expect, test } from "bun:test";

import { createFixedPixelRoi } from "../src/roiAnnotation";

const appSource = await Bun.file(new URL("../src/App.tsx", import.meta.url)).text();

describe("ROI annotation behavior", () => {
  test("uses fixed click placement instead of drag drawing", () => {
    expect(appSource).toContain("placeFixedRoi");
    expect(appSource).toContain("createFixedPixelRoi");
    expect(appSource).not.toContain("onPointerMove={");
    expect(appSource).not.toContain("onPointerUp={");
    expect(appSource).not.toContain("onPointerCancel={");
    expect(appSource).not.toContain("draftRect");
  });

  test("centers a 10px ROI on the clicked image point", () => {
    expect(createFixedPixelRoi({ x: 0.5, y: 0.5 }, { width: 100, height: 50 })).toEqual({
      x: 0.45,
      y: 0.4,
      w: 0.1,
      h: 0.2,
    });
  });

  test("clamps fixed ROI placement to image bounds", () => {
    expect(createFixedPixelRoi({ x: 0.01, y: 0.99 }, { width: 100, height: 50 })).toEqual({
      x: 0,
      y: 0.8,
      w: 0.1,
      h: 0.2,
    });
  });
});
