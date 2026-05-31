import { describe, expect, test } from "bun:test";

import {
  createImageBoxDiagnostic,
  createImageLoadDiagnostic,
  hasMaterialImageBoxChange,
  sendDiagnosticLog,
  shouldLogThrottledDiagnostic,
} from "../src/imageDiagnostics";

describe("image diagnostics helpers", () => {
  test("formats image load timing with source and dimensions", () => {
    expect(
      createImageLoadDiagnostic({
        source: "camera",
        name: "实时相机流",
        durationMs: 128.4,
        width: 1280,
        height: 720,
      }),
    ).toBe("图像加载完成：camera 实时相机流，1280 x 720，用时 128 ms。");
  });

  test("detects material image box size changes while ignoring tiny resize noise", () => {
    const current = { left: 10, top: 12, width: 640, height: 480 };

    expect(hasMaterialImageBoxChange(current, { left: 10.4, top: 12.5, width: 640.6, height: 480.4 })).toBe(false);
    expect(hasMaterialImageBoxChange(current, { left: 10, top: 12, width: 650, height: 480 })).toBe(true);
  });

  test("formats image layout diagnostics with rounded display box dimensions", () => {
    expect(createImageBoxDiagnostic({ left: 10.2, top: 12.7, width: 639.6, height: 479.5 })).toBe(
      "图像布局更新：显示框 640 x 480，偏移 10,13。",
    );
  });

  test("throttles repeated diagnostics until the interval elapses", () => {
    expect(shouldLogThrottledDiagnostic({ nowMs: 1_000, lastLoggedAtMs: null, intervalMs: 5_000 })).toBe(true);
    expect(shouldLogThrottledDiagnostic({ nowMs: 3_000, lastLoggedAtMs: 1_000, intervalMs: 5_000 })).toBe(false);
    expect(shouldLogThrottledDiagnostic({ nowMs: 6_100, lastLoggedAtMs: 1_000, intervalMs: 5_000 })).toBe(true);
  });

  // 旧测试 "reports abnormal camera frame gaps relative to configured fps" 已删除。
  // 原因：相机帧间隔诊断已从前端移至后端 worker 侧（capture-to-capture 真实间隔），
  // 不再在前端基于 1Hz 节流后的 frame 事件 timestamp 自算 gap。后端通过
  // camera-frame-gap-warning 事件推送，前端 listen 即可，无需本地阈值算法。

  test("forwards diagnostics to the backend command", async () => {
    const calls: Array<{ command: string; args: unknown }> = [];

    await sendDiagnosticLog("图像加载完成：camera", async (command, args) => {
      calls.push({ command, args });
    });

    expect(calls).toEqual([{ command: "diagnostic_log", args: { message: "图像加载完成：camera" } }]);
  });

  test("ignores backend diagnostic forwarding failures", async () => {
    await expect(
      sendDiagnosticLog("图像布局更新", async () => {
        throw new Error("backend unavailable");
      }),
    ).resolves.toBeUndefined();
  });
});
