# camera-bench

定位相机预览延迟瓶颈的 micro-benchmark 工具。

## 编译

```bash
# 在仓库根目录
bun run bench:build
# 或
cargo build --release -p camera-bench
```

产物路径：`target/release/camera-bench(.exe)`，是单文件可执行，可以直接拷到任意机器跑（同 OS / 架构）。

## 命令

### 列出 driver 报告的所有候选格式

```bash
bun run bench:list
# 或
cargo run --release -p camera-bench -- --list
```

输出每个相机所有 `(分辨率 × fps × frame_format)` 组合。**这是排查"配置 30fps 但实际只有 22fps"问题的第一步**——driver 没暴露给后端的格式，nokhwa 永远协商不到。

### 探测最佳协商策略

```bash
bun run bench:probe
# 或
cargo run --release -p camera-bench -- --probe
```

依次按多种 `RequestedFormatType` 策略打开相机，**实测**每种策略下 1 秒钟能取到多少帧。重点看：

- `HighestResolution(640x480)`：等价于 RobotApp C++ 的 `set FOURCC + WIDTH/HEIGHT 不 set FPS` 写法
- `AbsoluteHighestFrameRate`：driver 报告里 fps 最高的格式
- `Closest(MJPEG@30)` / `Closest(YUYV@30)` / `Closest(NV12@30)`：分别强制特定格式

对比 `negotiated` 与 `measured` 两列，可以发现：
- `measured >> negotiated` —— driver 报错 fps 但实际能跑（nokhwa MSMF 已知问题）
- `measured ≈ negotiated` —— driver 报告就是真实帧率
- `measured` 在某个策略下显著更高 —— 我们应用应改用该策略

### 完整分阶段 bench

```bash
cargo run --release -p camera-bench -- --slots 1 --frames 60
cargo run --release -p camera-bench -- --slots 4 --frames 60   # 4 路并发
```

分别测量：

1. `frame()` 阻塞耗时（硬件实际帧率上限）
2. `decode_image` MJPEG → RGB 解码耗时
3. JPEG 编码（单 tile / grid 1280×960，q=60 / q=72，直接 buffer / 经 RgbImage 中转）
4. grid blit 内存拷贝
5. `imageops::resize` 不同 filter 的耗时

## 选项

| flag | 默认 | 说明 |
|---|---|---|
| `--slots N` | 4 | 同时打开几个相机（按 `CameraIndex::Index(0..N)`） |
| `--frames N` | 60 | 每个 stage 采样多少帧 |
| `--width / --height` | 640 / 480 | 请求分辨率 |
| `--fps` | 30 | 请求帧率（仅对 `Closest` 策略生效） |
| `--format` | MJPEG | `MJPEG / YUYV` 等 |
| `--list` | - | 仅列设备格式，不抓帧 |
| `--probe` | - | 仅探测协商策略，不做完整 bench |

## 典型输出解读

### Logi C270 在 Windows MSMF 下（问题状态）

```
  - #0 Logi C270 HD WebCam
  slot 0 opened: 640x480 @ 1fps fmt=MJPEG          ← driver 报 1fps
  slot0 frame()    mean=40.2ms  p50=46.3 ...       ← 实际 22fps
```

driver 报告 1fps 但实际跑 22fps，且 `--list` 里只看到 NV12 / YUYV，没有 MJPEG 30fps 的条目——这说明 nokhwa 的 MSMF 后端没拿到完整格式列表。

### 解决方向

1. 优先：`--probe` 找出能跑到 30fps 的策略，调整 `crates/robo-camera/src/lib.rs` 里的 `RequestedFormatType`
2. 备选：换用 OpenCV `cv::CAP_DSHOW` 后端（DirectShow 通常能拿到 MJPEG 30fps）
