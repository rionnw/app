import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { gsap } from "gsap";

const canvas = document.querySelector("#scene");
const renderer = new THREE.WebGLRenderer({
  canvas,
  antialias: true,
  alpha: false,
  preserveDrawingBuffer: true,
});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x111312, 1);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x111312, 14, 30);

const camera = new THREE.PerspectiveCamera(36, 1, 0.1, 100);
camera.position.set(0, 0.25, 10.8);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0, 0);
controls.minDistance = 5;
controls.maxDistance = 18;

const state = {
  clawA: "CLOSED",  // U claw
  clawB: "CLOSED",  // R claw
  // 臂角度（归一化到 [-180, 180]），用于决定是否需要回正
  armAAngle: 0,
  armBAngle: 0,
  isAnimating: false,
  // Playback
  actions: [],
  currentStep: 0,
  isPlaying: false,
};

const ui = {
  status: document.querySelector("#motion-status"),
  warning: document.querySelector("#warning"),
  clawAState: document.querySelector("#claw-a-state"),
  clawBState: document.querySelector("#claw-b-state"),
  mode: document.querySelector("#kinematic-mode"),
  buttons: [...document.querySelectorAll("button")],
  toggleA: document.querySelector("#toggle-claw-a"),
  toggleB: document.querySelector("#toggle-claw-b"),
  clawACw: document.querySelector("#claw-a-cw"),
  clawACcw: document.querySelector("#claw-a-ccw"),
  clawBCw: document.querySelector("#claw-b-cw"),
  clawBCcw: document.querySelector("#claw-b-ccw"),
  faceletsInput: document.querySelector("#facelets-input"),
  applyFacelets: document.querySelector("#apply-facelets"),
  movesInput: document.querySelector("#moves-input"),
  runSolution: document.querySelector("#run-solution"),
  stepSolution: document.querySelector("#step-solution"),
  stopSolution: document.querySelector("#stop-solution"),
  resetCube: document.querySelector("#reset-cube"),
  progressFill: document.querySelector("#progress-fill"),
  progressText: document.querySelector("#progress-text"),
};

// ===== Color Palette =====
// solver facelet colors -> sticker color
const FACE_COLORS = {
  U: 0xf5f5f0, // white
  D: 0xf1d23b, // yellow
  R: 0xd63d30, // red
  L: 0xf47a20, // orange
  F: 0x2fac66, // green
  B: 0x2d75d6, // blue
};

const _labelTextureCache = {};

const stageWrapper = new THREE.Group();
stageWrapper.name = "StageWrapper";
stageWrapper.position.set(1.05, -0.08, 0);
stageWrapper.rotation.z = THREE.MathUtils.degToRad(45);
scene.add(stageWrapper);

const cubeGroup = new THREE.Group();
cubeGroup.name = "Cube_Group";
stageWrapper.add(cubeGroup);

const clawAGroup = new THREE.Group();
clawAGroup.name = "ClawA_Group";
stageWrapper.add(clawAGroup);

const clawBGroup = new THREE.Group();
clawBGroup.name = "ClawB_Group";
stageWrapper.add(clawBGroup);

const lights = makeLights();
lights.forEach((light) => scene.add(light));

clawAGroup.add(makeClawA());
clawBGroup.add(makeClawB());
makeAnchor("AnchorA", new THREE.Vector3(0, -1.5, 0), clawAGroup, 0x58d68d);
makeAnchor("AnchorB", new THREE.Vector3(-1.5, 0, 0), clawBGroup, 0x6ab7ff);

const cubies = [];
createCubies();
updateClawLinearPositions();
updateUi();

// ===== Manual control bindings =====
ui.toggleA.addEventListener("click", () => toggleClaw("A"));
ui.toggleB.addEventListener("click", () => toggleClaw("B"));
ui.clawACw.addEventListener("click", () => rotateClaw("A", 1));
ui.clawACcw.addEventListener("click", () => rotateClaw("A", -1));
ui.clawBCw.addEventListener("click", () => rotateClaw("B", 1));
ui.clawBCcw.addEventListener("click", () => rotateClaw("B", -1));

// ===== Playback bindings =====
ui.applyFacelets.addEventListener("click", applyFaceletsToCube);
ui.runSolution.addEventListener("click", runSolution);
ui.stepSolution.addEventListener("click", async () => {
  if (!ensureActionsLoaded()) return;
  await stepSolution();
});
ui.stopSolution.addEventListener("click", () => {
  state.isPlaying = false;
  ui.runSolution.textContent = "▶ Run";
});
ui.resetCube.addEventListener("click", resetCube);
ui.movesInput.addEventListener("input", () => {
  // 输入变化时重置进度，下一次 Run/Step 会重新解析
  state._lastMovesText = null;
  state.currentStep = 0;
  state.actions = [];
  updateProgress();
});

window.addEventListener("resize", resize);
resize();
renderer.setAnimationLoop(() => {
  controls.update();
  renderer.render(scene, camera);
});

function makeLights() {
  const ambient = new THREE.AmbientLight(0xffffff, 1.4);
  const key = new THREE.DirectionalLight(0xffffff, 2.2);
  key.position.set(4, 7, 5);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  const rim = new THREE.DirectionalLight(0x8fd4ff, 1.2);
  rim.position.set(-5, 3, -4);
  return [ambient, key, rim];
}

function makeClawA() {
  const group = new THREE.Group();
  group.name = "ClawA_Mesh";
  group.add(makeClawBar(new THREE.Vector3(0, -1.55, -1.15), new THREE.Vector3(3.45, 0.18, 0.16), 0x58d68d));
  group.add(makeClawBar(new THREE.Vector3(0, -1.55, 1.15), new THREE.Vector3(3.45, 0.18, 0.16), 0x58d68d));
  group.add(makeClawBar(new THREE.Vector3(-1.62, -1.55, 0), new THREE.Vector3(0.16, 0.18, 2.38), 0x45b878));
  group.add(makeClawBar(new THREE.Vector3(1.62, -1.55, 0), new THREE.Vector3(0.16, 0.18, 2.38), 0x45b878));
  return group;
}

function makeClawB() {
  const group = new THREE.Group();
  group.name = "ClawB_Mesh";
  group.add(makeClawBar(new THREE.Vector3(-1.55, 0, -1.15), new THREE.Vector3(0.18, 3.45, 0.16), 0x6ab7ff));
  group.add(makeClawBar(new THREE.Vector3(-1.55, 0, 1.15), new THREE.Vector3(0.18, 3.45, 0.16), 0x6ab7ff));
  group.add(makeClawBar(new THREE.Vector3(-1.55, -1.62, 0), new THREE.Vector3(0.18, 0.16, 2.38), 0x4b91d9));
  group.add(makeClawBar(new THREE.Vector3(-1.55, 1.62, 0), new THREE.Vector3(0.18, 0.16, 2.38), 0x4b91d9));
  return group;
}

function makeClawBar(position, size, color) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(size.x, size.y, size.z),
    new THREE.MeshStandardMaterial({ color, roughness: 0.38, metalness: 0.28 }),
  );
  mesh.position.copy(position);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function makeAnchor(name, position, parent, color) {
  const anchor = new THREE.Mesh(
    new THREE.SphereGeometry(0.07, 16, 10),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.72 }),
  );
  anchor.name = name;
  anchor.position.copy(position);
  parent.add(anchor);
}

function createCubies() {
  for (let x = -1; x <= 1; x += 1) {
    for (let y = -1; y <= 1; y += 1) {
      for (let z = -1; z <= 1; z += 1) {
        const cubie = makeCubie(x, y, z);
        cubeGroup.add(cubie);
        cubies.push(cubie);
      }
    }
  }
}

function makeCubie(x, y, z) {
  const group = new THREE.Group();
  group.name = `SubCube_${x}_${y}_${z}`;
  group.position.set(x, y, z);

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.96, 0.96, 0.96),
    new THREE.MeshStandardMaterial({ color: 0x101212, roughness: 0.72, metalness: 0.05 }),
  );
  body.castShadow = true;
  body.receiveShadow = true;
  group.add(body);

  // face 字母按硬件视角标注：U 爪在 -Y 面，R 爪在 -X 面
  const stickers = [
    { visible: x === 1,  pos: [0.486, 0, 0],   rot: [0, Math.PI / 2, 0],   face: "L" },
    { visible: x === -1, pos: [-0.486, 0, 0],  rot: [0, -Math.PI / 2, 0],  face: "R" },
    { visible: y === 1,  pos: [0, 0.486, 0],   rot: [-Math.PI / 2, 0, 0],  face: "D" },
    { visible: y === -1, pos: [0, -0.486, 0],  rot: [Math.PI / 2, 0, 0],   face: "U" },
    { visible: z === 1,  pos: [0, 0, 0.486],   rot: [0, 0, 0],             face: "F" },
    { visible: z === -1, pos: [0, 0, -0.486],  rot: [0, Math.PI, 0],       face: "B" },
  ];

  for (const s of stickers) {
    if (!s.visible) continue;
    const isCenter = isCenterStickerOf(s.face, x, y, z);
    const baseColor = FACE_COLORS[s.face];
    const material = isCenter
      ? new THREE.MeshStandardMaterial({
          map: makeFaceLabelTexture(s.face, baseColor),
          roughness: 0.5,
          metalness: 0.02,
          side: THREE.DoubleSide,
        })
      : new THREE.MeshStandardMaterial({
          color: baseColor,
          roughness: 0.5,
          metalness: 0.02,
          side: THREE.DoubleSide,
        });
    const sticker = new THREE.Mesh(new THREE.PlaneGeometry(0.72, 0.72), material);
    sticker.position.set(...s.pos);
    sticker.rotation.set(...s.rot);
    sticker.renderOrder = 2;
    sticker.userData.face = s.face;
    sticker.userData.solvedFace = s.face;
    sticker.userData.isCenter = isCenter;
    group.add(sticker);
  }

  return group;
}

function isCenterStickerOf(face, x, y, z) {
  switch (face) {
    case "U": return y === -1 && x === 0 && z === 0;
    case "D": return y === 1 && x === 0 && z === 0;
    case "R": return x === -1 && y === 0 && z === 0;
    case "L": return x === 1 && y === 0 && z === 0;
    case "F": return z === 1 && x === 0 && y === 0;
    case "B": return z === -1 && x === 0 && y === 0;
  }
  return false;
}

function makeFaceLabelTexture(face, baseColorHex) {
  const key = `${face}_${baseColorHex.toString(16)}`;
  if (_labelTextureCache[key]) return _labelTextureCache[key];
  const size = 256;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  // 背景填面色
  ctx.fillStyle = "#" + baseColorHex.toString(16).padStart(6, "0");
  ctx.fillRect(0, 0, size, size);
  // 黑色字母
  ctx.fillStyle = "rgba(0,0,0,0.85)";
  ctx.font = "bold 180px system-ui, -apple-system, Arial, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(face, size / 2, size / 2 + 8);
  const texture = new THREE.CanvasTexture(canvas);
  texture.anisotropy = 4;
  _labelTextureCache[key] = texture;
  return texture;
}

// ===== Facelets Application =====

/**
 * 把 54 字符 facelets 字符串应用到 3D 魔方的 sticker 颜色上
 * 顺序：U(0-8) R(9-17) F(18-26) D(27-35) L(36-44) B(45-53)
 * 每面 9 个块按行从左到右、从上到下
 */
function applyFaceletsToCube() {
  const text = ui.faceletsInput.value.trim();
  if (text.length !== 54) {
    showWarning(`Facelets must be 54 chars (got ${text.length}).`, "error");
    return;
  }
  if (!/^[URFDLB]+$/.test(text)) {
    showWarning("Facelets must contain only U/R/F/D/L/B.", "error");
    return;
  }

  // 先 reset 立方体到归零位置
  resetCube();

  // 收集所有 sticker，按面分组
  const byFace = { U: [], R: [], F: [], D: [], L: [], B: [] };
  for (const cubie of cubies) {
    for (const child of cubie.children) {
      if (child.userData && child.userData.face) {
        byFace[child.userData.face].push(child);
      }
    }
  }

  // 对每个面：按 facelet 顺序排序 sticker 然后涂色
  applyFaceColors(byFace.U, "U", text.slice(0, 9));
  applyFaceColors(byFace.R, "R", text.slice(9, 18));
  applyFaceColors(byFace.F, "F", text.slice(18, 27));
  applyFaceColors(byFace.D, "D", text.slice(27, 36));
  applyFaceColors(byFace.L, "L", text.slice(36, 45));
  applyFaceColors(byFace.B, "B", text.slice(45, 54));

  clearWarning();
  showWarning(`Applied facelets to cube.`, "info");
}

function applyFaceColors(stickers, faceName, faceStr) {
  if (stickers.length !== 9) {
    console.warn(`face ${faceName} has ${stickers.length} stickers, expected 9`);
    return;
  }
  // 按 facelet 标准顺序（行从上到下，列从左到右）排序 sticker
  // 取每个 sticker 所在 cubie 的本地坐标位置作为排序依据
  const sorted = sortFaceStickers(stickers, faceName);
  for (let i = 0; i < 9; i++) {
    const ch = faceStr[i];
    const color = FACE_COLORS[ch];
    if (color === undefined) continue;
    const sticker = sorted[i];
    if (sticker.userData.isCenter) {
      // 中心块用纹理（带字母），按新颜色生成一张
      sticker.material.map = makeFaceLabelTexture(ch, color);
      sticker.material.color.setHex(0xffffff);
      sticker.material.needsUpdate = true;
    } else {
      sticker.material.map = null;
      sticker.material.color.setHex(color);
      sticker.material.needsUpdate = true;
    }
  }
}

/**
 * 按 facelet 顺序排序面上的 9 个 sticker。
 * facelet 标准顺序参考 min2phase：每面 9 个块按 row-major，行从上到下。
 *
 * 立方体坐标系：x 右(R)，y 上(U)，z 前(F)。
 */
function sortFaceStickers(stickers, face) {
  const items = stickers.map((s) => {
    const p = s.parent.position; // cubie 位置（-1, 0, 1）
    return { sticker: s, x: p.x, y: p.y, z: p.z };
  });

  // 硬件视角：U=-Y, D=+Y, R=-X, L=+X, F=+Z, B=-Z
  // facelet 排序参考 Face.h 展开图（标准 URFDLB）
  let getRow, getCol;
  switch (face) {
    case "U": // 俯视，从 -Y 看 +Y。U1 后左, U9 前右
      getRow = (it) => it.z + 1;     // z=-1(B)→0, z=+1(F)→2
      getCol = (it) => 1 - it.x;     // x=+1(L)→0, x=-1(R)→2
      break;
    case "D": // 仰视，从 +Y 看 -Y
      getRow = (it) => 1 - it.z;
      getCol = (it) => 1 - it.x;
      break;
    case "F": // 法向 +Z
      getRow = (it) => it.y + 1;     // y=-1(U)→0, y=+1(D)→2
      getCol = (it) => 1 - it.x;
      break;
    case "B": // 法向 -Z
      getRow = (it) => it.y + 1;
      getCol = (it) => it.x + 1;
      break;
    case "R": // 法向 -X
      getRow = (it) => it.y + 1;
      getCol = (it) => 1 - it.z;
      break;
    case "L": // 法向 +X
      getRow = (it) => it.y + 1;
      getCol = (it) => it.z + 1;
      break;
  }

  items.sort((a, b) => {
    const ra = getRow(a), rb = getRow(b);
    if (ra !== rb) return ra - rb;
    return getCol(a) - getCol(b);
  });

  return items.map((it) => it.sticker);
}

// ===== Load moves from textarea =====

function ensureActionsLoaded() {
  const text = ui.movesInput.value.trim();
  if (!text) {
    showWarning("Please enter moves first.", "error");
    return false;
  }
  // Re-parse only when text changed
  if (state._lastMovesText !== text) {
    // 自动识别：硬件指令包含 CLAW_ 或 ROTATE_
    const isHardware = /\b(CLAW_[UR]|ROTATE_[UR])\b/.test(text);
    state.actions = isHardware
      ? parseHardwareActions(text)
      : parseTwoLActions(text);
    state.currentStep = 0;
    state._lastMovesText = text;
    updateProgress();
  }
  if (state.actions.length === 0) {
    showWarning("No valid moves parsed.", "error");
    return false;
  }
  return true;
}

// ===== Parse hardware commands =====
// 格式: CLAW_U(0|1); CLAW_R(0|1); ROTATE_U(±deg); ROTATE_R(±deg);
// 各指令以 ; 或 空白 分隔
function parseHardwareActions(text) {
  const actions = [];
  const re = /(CLAW_U|CLAW_R|ROTATE_U|ROTATE_R)\s*\(\s*([+\-]?\d+)\s*\)/g;
  let m;
  while ((m = re.exec(text)) !== null) {
    const cmd = m[1];
    const val = parseInt(m[2], 10);
    switch (cmd) {
      case "CLAW_U":
        actions.push({ kind: "HW_CLAW", which: "A", close: val !== 0 });
        break;
      case "CLAW_R":
        actions.push({ kind: "HW_CLAW", which: "B", close: val !== 0 });
        break;
      case "ROTATE_U":
        actions.push({ kind: "HW_ROTATE", which: "A", deg: val });
        break;
      case "ROTATE_R":
        actions.push({ kind: "HW_ROTATE", which: "B", deg: val });
        break;
    }
  }
  return actions;
}

// ===== Parse 2L solution into atomic actions =====

// 方向约定与 robo-translator 一致：U=-90, U'=+90, R=-90, R'=+90
const ACTION_TOKENS = [
  ["U2", { kind: "U", deg: 180 }],
  ["U'", { kind: "U", deg: 90 }],
  ["U",  { kind: "U", deg: -90 }],
  ["R2", { kind: "R", deg: 180 }],
  ["R'", { kind: "R", deg: 90 }],
  ["R",  { kind: "R", deg: -90 }],
  ["x2", { kind: "X", deg: 180 }],
  ["x'", { kind: "X", deg: 90 }],
  ["x",  { kind: "X", deg: -90 }],
  ["y2", { kind: "Y", deg: 180 }],
  ["y'", { kind: "Y", deg: 90 }],
  ["y",  { kind: "Y", deg: -90 }],
];

function parseTwoLActions(solution) {
  const actions = [];
  let s = solution.trim();
  while (s.length > 0) {
    s = s.replace(/^\s+/, "");
    if (!s) break;
    if (s.startsWith("(")) {
      // skip leg state notation
      const end = s.indexOf(")");
      if (end >= 0) {
        s = s.slice(end + 1).replace(/^\s+/, "");
      } else {
        s = s.slice(1);
        continue;
      }
      // 检查是否是 regrab（括号后无动作）
      if (!s || s.startsWith("(")) {
        actions.push({ kind: "REGRAB" });
        continue;
      }
    }
    let matched = false;
    for (const [token, act] of ACTION_TOKENS) {
      if (s.startsWith(token)) {
        actions.push(act);
        s = s.slice(token.length);
        matched = true;
        break;
      }
    }
    if (!matched) s = s.slice(1);
  }
  return actions;
}

// ===== Solution Playback =====

function updateProgress() {
  const total = state.actions.length;
  const cur = state.currentStep;
  ui.progressText.textContent = `${cur}/${total}`;
  ui.progressFill.style.width = total > 0 ? `${(cur / total) * 100}%` : "0%";
}

async function stepSolution() {
  if (state.isAnimating || state.currentStep >= state.actions.length) return;
  const action = state.actions[state.currentStep];
  state.currentStep += 1;
  updateProgress();
  await executeAction(action);
}

async function runSolution() {
  if (state.isPlaying) {
    state.isPlaying = false;
    ui.runSolution.textContent = "▶ Run";
    return;
  }
  if (!ensureActionsLoaded()) return;
  // 如果上一次跑完了，重新从头开始
  if (state.currentStep >= state.actions.length) {
    state.currentStep = 0;
    updateProgress();
  }
  state.isPlaying = true;
  ui.runSolution.textContent = "⏸ Pause";

  while (state.isPlaying && state.currentStep < state.actions.length) {
    await stepSolution();
    await sleep(80);
  }

  state.isPlaying = false;
  ui.runSolution.textContent = "▶ Run";
}

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

// ===== Execute one 2L action on the 3D model =====
// 规则与 robo-translator 保持一致：
//  - single U/R 前需对面臂"平放"（is_flat: 0 或 ±180）
//  - 整体 y(绕U轴): R 松→U 转→R 紧；若 U 竖立(非平放)则回正
//  - 整体 x(绕R轴): U 松→R 转→U 紧；若 R 竖立则回正
//  - 180° 不回正

async function ensureArmFlat(which) {
  const angle = which === "A" ? state.armAAngle : state.armBAngle;
  if (isArmFlat(angle)) return;
  // 回正：先夹另一只手固定，松开本爪，反方向转回 0
  const other = which === "A" ? "B" : "A";
  await ensureClawState(other, "CLOSED");
  await ensureClawState(which, "OPEN");
  await rotateClawAsync(which, -angle / 90);
  await ensureClawState(which, "CLOSED");
}

async function executeAction(action) {
  switch (action.kind) {
    case "U":
      // 单层 U：R 臂必须平放，两爪闭合，U 转
      await ensureArmFlat("B");
      await ensureClawState("A", "CLOSED");
      await ensureClawState("B", "CLOSED");
      await rotateClawAsync("A", action.deg / 90);
      break;
    case "R":
      // 单层 R：U 臂必须平放
      await ensureArmFlat("A");
      await ensureClawState("A", "CLOSED");
      await ensureClawState("B", "CLOSED");
      await rotateClawAsync("B", action.deg / 90);
      break;
    case "Y":
      // 绕 U 轴整体旋转：R 松→U 带→R 紧→若 U 竖则回正（180° 不回正）
      await ensureClawState("B", "OPEN");
      await ensureClawState("A", "CLOSED");
      await rotateClawAsync("A", action.deg / 90);
      await ensureClawState("B", "CLOSED");
      if (!isArmFlat(state.armAAngle)) {
        await ensureArmFlat("A");
      }
      break;
    case "X":
      // 绕 R 轴整体旋转：U 松→R 带→U 紧→若 R 竖则回正
      await ensureClawState("A", "OPEN");
      await ensureClawState("B", "CLOSED");
      await rotateClawAsync("B", action.deg / 90);
      await ensureClawState("A", "CLOSED");
      if (!isArmFlat(state.armBAngle)) {
        await ensureArmFlat("B");
      }
      break;
    case "REGRAB":
      // translator 中 regrab 不产生硬件指令，这里也跳过
      break;
    case "HW_CLAW":
      // 硬件指令：直接开/关爪，不做任何额外判断
      await ensureClawState(action.which, action.close ? "CLOSED" : "OPEN");
      break;
    case "HW_ROTATE":
      // 硬件指令：直接转臂指定角度（视觉上 90° 的倍数动画）
      await rotateClawAsync(action.which, action.deg / 90);
      break;
  }
}

function ensureClawState(which, target) {
  return new Promise((resolve) => {
    const cur = which === "A" ? state.clawA : state.clawB;
    if (cur === target) {
      resolve();
      return;
    }
    toggleClaw(which, resolve);
  });
}

// ===== Manual control (already existing) =====

function toggleClaw(which, onComplete) {
  if (state.isAnimating) {
    if (onComplete) onComplete();
    return;
  }
  setAnimating(true, "Moving");
  clearWarning();

  const key = which === "A" ? "clawA" : "clawB";
  state[key] = state[key] === "CLOSED" ? "OPEN" : "CLOSED";
  const target = which === "A"
    ? { y: state.clawA === "CLOSED" ? 0 : -0.65 }
    : { x: state.clawB === "CLOSED" ? 0 : -0.65 };
  const group = which === "A" ? clawAGroup : clawBGroup;

  gsap.to(group.position, {
    ...target,
    duration: 0.28,
    ease: "power2.inOut",
    onComplete: () => {
      setAnimating(false);
      updateUi();
      if (onComplete) onComplete();
    },
  });
  updateUi();
}

function rotateClaw(which, direction, onComplete) {
  if (state.isAnimating) {
    if (onComplete) onComplete();
    return;
  }
  clearWarning();

  const selfOpen = which === "A" ? state.clawA === "OPEN" : state.clawB === "OPEN";

  const isA = which === "A";
  const drivingGroup = isA ? clawAGroup : clawBGroup;
  // 两臂旋转方向均按硬件视角校正
  const axis = isA
    ? new THREE.Vector3(0, -1, 0)
    : new THREE.Vector3(-1, 0, 0);

  // 本爪 OPEN 时：仅旋转空臂，不带任何 cubie（用于回正）
  const targetCubies = selfOpen ? [] : selectCubiesForRotation(which);

  setAnimating(true, targetCubies.length === 27 ? "Whole cube" : (targetCubies.length === 0 ? "Arm reset" : "Layer turn"));
  targetCubies.forEach((cubie) => drivingGroup.attach(cubie));

  // direction can be a multiplier (e.g., 2 for 180°)
  const targetQuaternion = new THREE.Quaternion().setFromAxisAngle(axis, direction * Math.PI / 2);
  targetQuaternion.multiply(drivingGroup.quaternion);

  const duration = Math.abs(direction) >= 2 ? 0.6 : 0.42;

  gsap.to(drivingGroup.quaternion, {
    x: targetQuaternion.x,
    y: targetQuaternion.y,
    z: targetQuaternion.z,
    w: targetQuaternion.w,
    duration,
    ease: "power2.inOut",
    onComplete: () => {
      targetCubies.forEach((cubie) => cubeGroup.attach(cubie));
      snapCubies(targetCubies);
      // 更新臂角度（归一化）
      const delta = direction * 90;
      if (isA) state.armAAngle = normalizeAngle(state.armAAngle + delta);
      else state.armBAngle = normalizeAngle(state.armBAngle + delta);
      setAnimating(false);
      updateUi();
      if (onComplete) onComplete();
    },
  });
}

function normalizeAngle(a) {
  let v = a % 360;
  if (v > 180) v -= 360;
  if (v <= -180) v += 360;
  return v;
}

function isArmFlat(angle) {
  return angle === 0 || Math.abs(angle) === 180;
}

function rotateClawAsync(which, direction) {
  return new Promise((resolve) => {
    rotateClaw(which, direction, resolve);
  });
}

function selectCubiesForRotation(which) {
  if (which === "A") {
    if (state.clawB === "OPEN") return [...cubies];
    return cubies.filter((cubie) => Math.round(getStagePosition(cubie).y) === -1);
  }

  if (state.clawA === "OPEN") return [...cubies];
  return cubies.filter((cubie) => Math.round(getStagePosition(cubie).x) === -1);
}

function getStagePosition(object) {
  const world = new THREE.Vector3();
  object.getWorldPosition(world);
  return stageWrapper.worldToLocal(world);
}

function snapCubies(targetCubies) {
  targetCubies.forEach((cubie) => {
    cubie.position.x = Math.round(cubie.position.x);
    cubie.position.y = Math.round(cubie.position.y);
    cubie.position.z = Math.round(cubie.position.z);
    cubie.quaternion.normalize();
  });
}

function updateClawLinearPositions() {
  clawAGroup.position.y = state.clawA === "CLOSED" ? 0 : -0.65;
  clawBGroup.position.x = state.clawB === "CLOSED" ? 0 : -0.65;
}

function updateUi() {
  ui.clawAState.textContent = state.clawA;
  ui.clawBState.textContent = state.clawB;
  ui.status.textContent = state.isAnimating ? "Busy" : "Idle";
  ui.mode.textContent = modeText();
  ui.clawACw.disabled = state.isAnimating || state.clawA === "OPEN";
  ui.clawACcw.disabled = state.isAnimating || state.clawA === "OPEN";
  ui.clawBCw.disabled = state.isAnimating || state.clawB === "OPEN";
  ui.clawBCcw.disabled = state.isAnimating || state.clawB === "OPEN";
}

function modeText() {
  if (state.clawA === "OPEN" && state.clawB === "OPEN") return "Loose";
  if (state.clawA === "OPEN" || state.clawB === "OPEN") return "Whole cube";
  return "Layer turn";
}

function setAnimating(value, label = "Busy") {
  state.isAnimating = value;
  ui.status.textContent = value ? label : "Idle";
}

function showWarning(message, kind = "info") {
  ui.warning.textContent = message;
  ui.warning.dataset.kind = kind;
}

function clearWarning() {
  ui.warning.textContent = "";
  ui.warning.removeAttribute("data-kind");
}

function resize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
}

function resetCube() {
  // 重置 cubies 位置和旋转，清掉 sticker 的颜色覆盖
  for (const cubie of cubies) {
    // restore to original parent if needed
    if (cubie.parent !== cubeGroup) {
      cubeGroup.attach(cubie);
    }
  }
  // 重新创建：移除所有 cubies 重新生成
  for (const cubie of cubies) {
    cubeGroup.remove(cubie);
  }
  cubies.length = 0;
  createCubies();
  // 重置两臂角度与朝向
  state.armAAngle = 0;
  state.armBAngle = 0;
  clawAGroup.quaternion.identity();
  clawBGroup.quaternion.identity();
  state.currentStep = 0;
  updateProgress();
  clearWarning();
}
