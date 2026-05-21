import { useEffect, useMemo, useRef, useState } from "react";
import "./App.css";

type Face = "U" | "R" | "F" | "D" | "L" | "B";

type RoiRect = {
  x: number;
  y: number;
  w: number;
  h: number;
};

type StickerRegion = {
  id: string;
  face: Face;
  index: number;
  rect: RoiRect | null;
  locked: boolean;
};

type ImageBox = {
  left: number;
  top: number;
  width: number;
  height: number;
};

type LogItem = {
  time: string;
  text: string;
  kind: "info" | "warn" | "error";
};

const faces: Face[] = ["U", "R", "F", "D", "L", "B"];

const defaultRegions = (): StickerRegion[] =>
  faces.flatMap((face) =>
    Array.from({ length: 9 }, (_, index) => ({
      id: `${face}${index + 1}`,
      face,
      index: index + 1,
      rect: null,
      locked: false,
    })),
  );

const nowTime = () =>
  new Intl.DateTimeFormat("zh-CN", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }).format(new Date());

function App() {
  const imageInputRef = useRef<HTMLInputElement>(null);
  const roiInputRef = useRef<HTMLInputElement>(null);
  const stageRef = useRef<HTMLDivElement>(null);
  const dragStartRef = useRef<{ x: number; y: number } | null>(null);

  const [toolsOpen, setToolsOpen] = useState(false);
  const [cameraOpen, setCameraOpen] = useState(false);
  const [serialConnected, setSerialConnected] = useState(true);
  const [selectedPort, setSelectedPort] = useState("COM4");
  const [imageSrc, setImageSrc] = useState<string | null>(null);
  const [imageName, setImageName] = useState("后台拼接图像");
  const [naturalSize, setNaturalSize] = useState({ width: 0, height: 0 });
  const [imageBox, setImageBox] = useState<ImageBox>({ left: 0, top: 0, width: 0, height: 0 });
  const [regions, setRegions] = useState<StickerRegion[]>(defaultRegions);
  const [currentRegionId, setCurrentRegionId] = useState("U1");
  const [annotationMode, setAnnotationMode] = useState(false);
  const [showRoi, setShowRoi] = useState(true);
  const [draftRect, setDraftRect] = useState<RoiRect | null>(null);
  const [elapsed, setElapsed] = useState("00.00");
  const [runStatus, setRunStatus] = useState("空闲");
  const [solution, setSolution] = useState("未解算");
  const [logs, setLogs] = useState<LogItem[]>([
    { time: nowTime(), text: "RobotApp initialized.", kind: "info" },
  ]);

  const markedCount = regions.filter((region) => region.rect).length;

  const imageAspectLabel = useMemo(() => {
    if (!naturalSize.width || !naturalSize.height) return "-";
    return `${naturalSize.width} x ${naturalSize.height}`;
  }, [naturalSize]);

  useEffect(() => {
    const updateImageBox = () => {
      const stage = stageRef.current;
      if (!stage || !naturalSize.width || !naturalSize.height) return;

      const stageRect = stage.getBoundingClientRect();
      const imageRatio = naturalSize.width / naturalSize.height;
      const stageRatio = stageRect.width / stageRect.height;
      const width = stageRatio > imageRatio ? stageRect.height * imageRatio : stageRect.width;
      const height = stageRatio > imageRatio ? stageRect.height : stageRect.width / imageRatio;

      setImageBox({
        left: (stageRect.width - width) / 2,
        top: (stageRect.height - height) / 2,
        width,
        height,
      });
    };

    updateImageBox();

    const observer = new ResizeObserver(updateImageBox);
    if (stageRef.current) observer.observe(stageRef.current);
    window.addEventListener("resize", updateImageBox);

    return () => {
      observer.disconnect();
      window.removeEventListener("resize", updateImageBox);
    };
  }, [naturalSize]);

  const addLog = (text: string, kind: LogItem["kind"] = "info") => {
    setLogs((items) => [{ time: nowTime(), text, kind }, ...items].slice(0, 80));
  };

  const selectNextMissingRegion = (afterId = currentRegionId) => {
    const startIndex = regions.findIndex((region) => region.id === afterId);
    const ordered = [...regions.slice(startIndex + 1), ...regions.slice(0, startIndex + 1)];
    const next = ordered.find((region) => !region.rect);
    if (next) setCurrentRegionId(next.id);
  };

  const normalizedPointFromEvent = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!imageBox.width || !imageBox.height) return null;
    const stageRect = event.currentTarget.getBoundingClientRect();
    const x = (event.clientX - stageRect.left - imageBox.left) / imageBox.width;
    const y = (event.clientY - stageRect.top - imageBox.top) / imageBox.height;
    if (x < 0 || x > 1 || y < 0 || y > 1) return null;
    return { x, y };
  };

  const beginAnnotation = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!annotationMode || !imageSrc) return;
    const point = normalizedPointFromEvent(event);
    if (!point) return;
    dragStartRef.current = point;
    setDraftRect({ x: point.x, y: point.y, w: 0, h: 0 });
    event.currentTarget.setPointerCapture(event.pointerId);
  };

  const updateAnnotation = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!dragStartRef.current) return;
    const point = normalizedPointFromEvent(event);
    if (!point) return;

    const x = Math.min(dragStartRef.current.x, point.x);
    const y = Math.min(dragStartRef.current.y, point.y);
    const w = Math.abs(point.x - dragStartRef.current.x);
    const h = Math.abs(point.y - dragStartRef.current.y);
    setDraftRect({ x, y, w, h });
  };

  const finishAnnotation = () => {
    if (!dragStartRef.current || !draftRect) return;
    dragStartRef.current = null;

    if (draftRect.w < 0.005 || draftRect.h < 0.005) {
      setDraftRect(null);
      return;
    }

    setRegions((items) =>
      items.map((region) => (region.id === currentRegionId ? { ...region, rect: draftRect } : region)),
    );
    addLog(`ROI ${currentRegionId} marked.`);
    setDraftRect(null);
    selectNextMissingRegion(currentRegionId);
  };

  const loadImageFile = (file: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      setImageSrc(String(reader.result));
      setImageName(file.name);
      addLog(`Loaded stitched image: ${file.name}`);
    };
    reader.readAsDataURL(file);
  };

  const handleImageInput = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (file) loadImageFile(file);
    setToolsOpen(false);
    event.target.value = "";
  };

  const handleRoiInput = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = () => {
      try {
        const data = JSON.parse(String(reader.result)) as StickerRegion[];
        const knownIds = new Set(defaultRegions().map((region) => region.id));
        const nextRegions = data.filter((region) => knownIds.has(region.id));
        if (nextRegions.length !== 54) throw new Error("ROI count must be 54");
        setRegions(nextRegions);
        addLog(`Loaded ROI config: ${file.name}`);
      } catch (error) {
        addLog(error instanceof Error ? error.message : "Failed to load ROI config.", "error");
      }
    };
    reader.readAsText(file);
    setToolsOpen(false);
    event.target.value = "";
  };

  const toggleCamera = () => {
    setCameraOpen((open) => {
      const next = !open;
      addLog(next ? "Cameras opening..." : "Cameras closed.");
      if (next) setTimeout(() => addLog("Opened cameras."), 250);
      return next;
    });
  };

  const startSolve = () => {
    if (!imageSrc) {
      addLog("No stitched image available for solving.", "warn");
      return;
    }
    if (markedCount < 54) {
      addLog(`ROI incomplete: ${markedCount}/54.`, "warn");
      return;
    }

    setSolution("R U R' U' F2 D L2");
    setRunStatus("已解算");
    addLog("Cube state recognized. Solution generated.");
  };

  const startRun = () => {
    if (!serialConnected) {
      addLog("Serial port is not connected.", "error");
      return;
    }
    if (solution === "未解算") {
      addLog("Solve before running robot.", "warn");
      return;
    }

    setRunStatus("运行中");
    setElapsed("00.00");
    addLog("Robot run started.");
    window.setTimeout(() => {
      setElapsed("03.42");
      setRunStatus("完成");
      addLog("Robot run finished.");
    }, 900);
  };

  const downloadText = (filename: string, text: string, type = "application/json") => {
    const url = URL.createObjectURL(new Blob([text], { type }));
    const link = document.createElement("a");
    link.href = url;
    link.download = filename;
    link.click();
    URL.revokeObjectURL(url);
  };

  const saveRoi = () => {
    downloadText("robot-roi.json", JSON.stringify(regions, null, 2));
    addLog("ROI config exported.");
    setToolsOpen(false);
  };

  const saveOriginalImage = () => {
    if (!imageSrc) {
      addLog("No image to save.", "warn");
      return;
    }
    const link = document.createElement("a");
    link.href = imageSrc;
    link.download = imageName || "stitched-image.png";
    link.click();
    addLog("Original image saved.");
    setToolsOpen(false);
  };

  const saveAnnotatedImage = async () => {
    if (!imageSrc || !naturalSize.width || !naturalSize.height) {
      addLog("No image to save.", "warn");
      return;
    }

    const image = new Image();
    image.src = imageSrc;
    await image.decode();

    const canvas = document.createElement("canvas");
    canvas.width = naturalSize.width;
    canvas.height = naturalSize.height;
    const context = canvas.getContext("2d");
    if (!context) return;

    context.drawImage(image, 0, 0);
    context.lineWidth = Math.max(3, naturalSize.width * 0.003);
    context.font = `${Math.max(18, naturalSize.width * 0.018)}px sans-serif`;
    context.textBaseline = "top";

    regions.forEach((region) => {
      if (!region.rect) return;
      const { x, y, w, h } = region.rect;
      const px = x * naturalSize.width;
      const py = y * naturalSize.height;
      const pw = w * naturalSize.width;
      const ph = h * naturalSize.height;
      context.strokeStyle = region.id === currentRegionId ? "#f5c542" : "#31c96b";
      context.fillStyle = "rgba(49, 201, 107, 0.16)";
      context.fillRect(px, py, pw, ph);
      context.strokeRect(px, py, pw, ph);
      context.fillStyle = "#ffffff";
      context.fillText(region.id, px + 6, py + 5);
    });

    const link = document.createElement("a");
    link.href = canvas.toDataURL("image/png");
    link.download = "stitched-image-roi.png";
    link.click();
    addLog("Annotated image saved.");
    setToolsOpen(false);
  };

  const clearImage = () => {
    setImageSrc(null);
    setImageName("后台拼接图像");
    setNaturalSize({ width: 0, height: 0 });
    addLog("Image cleared.");
    setToolsOpen(false);
  };

  const clearRoi = () => {
    setRegions(defaultRegions());
    setCurrentRegionId("U1");
    addLog("ROI cleared.");
    setToolsOpen(false);
  };

  const exportLogs = () => {
    downloadText(
      "robot-log.txt",
      logs.map((item) => `[${item.time}] ${item.kind.toUpperCase()} ${item.text}`).join("\n"),
      "text/plain",
    );
  };

  return (
    <main className="robot-shell">
      <input ref={imageInputRef} type="file" accept="image/*" hidden onChange={handleImageInput} />
      <input ref={roiInputRef} type="file" accept="application/json,.json" hidden onChange={handleRoiInput} />

      <header className="menu-bar">
        <div className="tool-menu">
          <button className="menu-trigger" type="button" onClick={() => setToolsOpen((open) => !open)}>
            工具
            <span aria-hidden="true">⌄</span>
          </button>
          {toolsOpen && (
            <div className="tool-popover">
              <section>
                <h2>图像</h2>
                <button type="button" onClick={() => imageInputRef.current?.click()}>
                  读取图片
                </button>
                <button type="button" onClick={saveOriginalImage}>
                  保存图片
                </button>
                <button type="button" onClick={saveAnnotatedImage}>
                  保存带标注图片
                </button>
                <button type="button" onClick={clearImage}>
                  清空图片
                </button>
              </section>
              <section>
                <h2>ROI 标注</h2>
                <button
                  type="button"
                  onClick={() => {
                    setAnnotationMode((mode) => !mode);
                    setToolsOpen(false);
                  }}
                >
                  {annotationMode ? "退出标记 ROI" : "标记 ROI"}
                </button>
                <button type="button" onClick={saveRoi}>
                  保存 ROI
                </button>
                <button type="button" onClick={() => roiInputRef.current?.click()}>
                  读取 ROI
                </button>
                <button
                  type="button"
                  onClick={() => {
                    setShowRoi((visible) => !visible);
                    setToolsOpen(false);
                  }}
                >
                  {showRoi ? "隐藏 ROI" : "显示 ROI"}
                </button>
                <button type="button" onClick={clearRoi}>
                  清空 ROI
                </button>
              </section>
              <section>
                <h2>调试</h2>
                <button
                  type="button"
                  onClick={() => {
                    addLog("Serial ports refreshed.");
                    setToolsOpen(false);
                  }}
                >
                  刷新串口
                </button>
                <button
                  type="button"
                  onClick={() => {
                    addLog("Parameters read from device.");
                    setToolsOpen(false);
                  }}
                >
                  读取参数
                </button>
                <button
                  type="button"
                  onClick={() => {
                    addLog("Parameters written to device.");
                    setToolsOpen(false);
                  }}
                >
                  写入参数
                </button>
                <button
                  type="button"
                  onClick={() => {
                    setLogs([]);
                    setToolsOpen(false);
                  }}
                >
                  清空日志
                </button>
                <button type="button" onClick={exportLogs}>
                  导出日志
                </button>
              </section>
            </div>
          )}
        </div>
      </header>

      <section className="workspace">
        <section className="image-panel">
          <div className="panel-title">
            <h1>魔方图像</h1>
            <div className="image-meta">
              <span>{imageName}</span>
              <span>{imageAspectLabel}</span>
              <span>ROI {markedCount}/54</span>
            </div>
          </div>

          <div
            ref={stageRef}
            className={`image-stage ${annotationMode ? "is-annotating" : ""}`}
            onPointerDown={beginAnnotation}
            onPointerMove={updateAnnotation}
            onPointerUp={finishAnnotation}
            onPointerCancel={finishAnnotation}
          >
            {imageSrc ? (
              <>
                <img
                  alt="Stitched cube"
                  className="stitched-image"
                  src={imageSrc}
                  style={{
                    left: imageBox.left,
                    top: imageBox.top,
                    width: imageBox.width,
                    height: imageBox.height,
                  }}
                  onLoad={(event) =>
                    setNaturalSize({
                      width: event.currentTarget.naturalWidth,
                      height: event.currentTarget.naturalHeight,
                    })
                  }
                />
                {showRoi && (
                  <svg
                    className="roi-layer"
                    style={{
                      left: imageBox.left,
                      top: imageBox.top,
                      width: imageBox.width,
                      height: imageBox.height,
                    }}
                    viewBox="0 0 1 1"
                    preserveAspectRatio="none"
                  >
                    {regions.map((region) =>
                      region.rect ? (
                        <g key={region.id} onClick={() => setCurrentRegionId(region.id)}>
                          <rect
                            className={region.id === currentRegionId ? "roi-rect is-active" : "roi-rect"}
                            x={region.rect.x}
                            y={region.rect.y}
                            width={region.rect.w}
                            height={region.rect.h}
                            vectorEffect="non-scaling-stroke"
                          />
                          <text className="roi-label" x={region.rect.x + 0.006} y={region.rect.y + 0.018}>
                            {region.id}
                          </text>
                        </g>
                      ) : null,
                    )}
                    {draftRect && (
                      <rect
                        className="roi-rect is-draft"
                        x={draftRect.x}
                        y={draftRect.y}
                        width={draftRect.w}
                        height={draftRect.h}
                        vectorEffect="non-scaling-stroke"
                      />
                    )}
                  </svg>
                )}
              </>
            ) : (
              <div className="empty-image">
                <strong>等待拼接图像</strong>
                <span>通过工具菜单读取图片，或打开相机后由后台推送图像。</span>
              </div>
            )}
          </div>

          <div className="bottom-actions">
            <button type="button" onClick={toggleCamera}>
              {cameraOpen ? "关闭相机" : "打开相机"}
            </button>
            <button type="button" onClick={startSolve}>
              开始解算
            </button>
            <button type="button" onClick={startRun}>
              开始运行
            </button>
          </div>
        </section>

        <aside className="side-panel">
          <section className="timer-card">
            <h2>运动计时</h2>
            <div className="timer-display">{elapsed}</div>
            <div className="status-line">状态：{runStatus}</div>
          </section>

          <section className="panel-card">
            <h2>串口</h2>
            <label className="field">
              <span>串口信息</span>
              <select value={selectedPort} onChange={(event) => setSelectedPort(event.target.value)}>
                <option>COM4</option>
                <option>COM5</option>
                <option>/dev/tty.usbserial</option>
              </select>
            </label>
            <div className="button-row">
              <button type="button" onClick={() => setSerialConnected((connected) => !connected)}>
                {serialConnected ? "关闭" : "连接"}
              </button>
              <button type="button" onClick={() => addLog("Serial ports refreshed.")}>
                刷新
              </button>
            </div>
            <p className="strong-line">串口状态：{serialConnected ? "已连接" : "未连接"}</p>
          </section>

          <section className="panel-card log-card">
            <h2>信息输出</h2>
            <div className="log-output">
              {logs.map((item, index) => (
                <p key={`${item.time}-${index}`} className={`log-line ${item.kind}`}>
                  <span>{item.time}</span>
                  {item.text}
                </p>
              ))}
            </div>
          </section>
        </aside>
      </section>
    </main>
  );
}

export default App;
