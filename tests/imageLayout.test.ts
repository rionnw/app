import { describe, expect, test } from "bun:test";

import { getLoadedImageSize, updateImageSize } from "../src/imageLayout";

describe("image layout helpers", () => {
  test("reads loaded image dimensions synchronously", () => {
    expect(getLoadedImageSize({ naturalWidth: 1280, naturalHeight: 720 })).toEqual({
      width: 1280,
      height: 720,
    });
  });

  test("keeps the previous image size object when dimensions are unchanged", () => {
    const previous = { width: 640, height: 480 };

    expect(updateImageSize(previous, { width: 640, height: 480 })).toBe(previous);
  });
});
