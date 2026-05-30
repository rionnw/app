"""
魔方颜色识别脚本 (Python 调试版)

使用 robo-app/robot-roi.json 中的 54 个 ROI，对图片进行 LAB 颜色空间
聚类识别，将识别结果用色块标注在原图 ROI 位置上另存。

使用方法：
    python scripts/recognize.py [图片路径...]

    不带参数时处理 roboapp/imgs/ 下所有 PNG 图片。
    结果保存到 scripts/output/ 目录。

依赖：
    pip install opencv-python numpy scipy scikit-learn
"""

import cv2
import numpy as np
import json
import os
import sys
import glob
from scipy.optimize import linear_sum_assignment

# ============ 参数 ============
N_CLUSTERS = 6
BLOCKS_PER_COLOR = 9
ROI_DRAW_SIZE = 15  # 标注时绘制的 ROI 边框大小

# ROI 到 solver facelet 的映射（从 Rust robo-vision 中的 SOLVER_FACELET_ORIGINAL_ROI_INDICES）
# solver_facelet[i] 的颜色来自 original_roi[SOLVER_FACELET_ORIGINAL_ROI_INDICES[i]]
SOLVER_FACELET_ORIGINAL_ROI_INDICES = [
    29, 32, 35, 28, 31, 34, 27, 30, 33,  # U face
    36, 37, 38, 39, 40, 41, 42, 43, 44,  # R face
    20, 23, 26, 19, 22, 25, 18, 21, 24,  # F face
    11, 14, 17, 10, 13, 16,  9, 12, 15,  # D face
     0,  1,  2,  3,  4,  5,  6,  7,  8,  # L face
    45, 46, 47, 48, 49, 50, 51, 52, 53,  # B face
]

# 6 个中心色块在 original ROI 中的索引（从 Rust CENTER_ORIGINAL_ROI_INDICES）
# 顺序: U, R, F, D, L, B 面的中心
CENTER_ORIGINAL_ROI_IDS = [31, 40, 22, 13, 4, 49]

# 自适应过滤参数
PERCENTILE_LOW = 5
PERCENTILE_HIGH = 95
MIN_VALID_PIXELS = 4
# L 通道权重：降低以减少亮度对聚类的干扰（白/绿/黄在高光下 L 都很高）
L_WEIGHT = 0.3

# 面标签
FACE_NAMES = "URFDLB"
FACE_COLORS_BGR = [
    (255, 255, 255),  # U - 白
    (0, 0, 255),      # R - 红
    (0, 200, 0),      # F - 绿
    (0, 255, 255),    # D - 黄
    (0, 128, 255),    # L - 橙
    (255, 0, 0),      # B - 蓝
]


def load_rois(roi_file):
    """加载 robo-app 格式的 ROI 文件: {"rois": [{x, y, width, height}, ...]}"""
    with open(roi_file, "r") as f:
        data = json.load(f)
    rois = data["rois"]
    assert len(rois) == 54, f"ROI 数量必须为 54，当前 {len(rois)}"
    return rois


def extract_roi_color(lab_image, roi):
    """从 LAB 图像的 ROI 区域提取鲁棒的 (L*w, a*, b*) 特征"""
    x, y, w, h = roi["x"], roi["y"], roi["width"], roi["height"]
    y1 = max(0, y)
    y2 = min(lab_image.shape[0], y + h)
    x1 = max(0, x)
    x2 = min(lab_image.shape[1], x + w)

    if y2 <= y1 or x2 <= x1:
        return np.array([128.0 * L_WEIGHT, 128.0, 128.0])

    patch = lab_image[y1:y2, x1:x2]
    l_ch = patch[:, :, 0].astype(np.float32).flatten()
    a_ch = patch[:, :, 1].astype(np.float32).flatten()
    b_ch = patch[:, :, 2].astype(np.float32).flatten()

    if len(l_ch) == 0:
        return np.array([128.0 * L_WEIGHT, 128.0, 128.0])

    # 百分位过滤高光/暗区
    l_lo = np.percentile(l_ch, PERCENTILE_LOW)
    l_hi = np.percentile(l_ch, PERCENTILE_HIGH)
    mask = (l_ch >= l_lo) & (l_ch <= l_hi)

    if np.sum(mask) < MIN_VALID_PIXELS:
        mask = np.ones(len(l_ch), dtype=bool)

    l_val = float(np.median(l_ch[mask])) * L_WEIGHT
    a_val = float(np.median(a_ch[mask]))
    b_val = float(np.median(b_ch[mask]))
    return np.array([l_val, a_val, b_val])


def cluster_colors(features):
    """
    KMeans 聚类 + 匈牙利算法约束均衡分配。
    
    初始化中心使用 6 个中心色块的实际颜色，KMeans 的 cluster 0 对应
    CENTER_ORIGINAL_ROI_IDS[0] (U面中心), cluster 1 对应 R面中心, ...
    因此 class_to_face 就是直接映射: class i → face i。
    """
    n = len(features)
    assert n == N_CLUSTERS * BLOCKS_PER_COLOR

    # 用 6 个中心色块初始化：cluster i 的初始中心 = 面 i 的中心色块颜色
    init_centers = features[CENTER_ORIGINAL_ROI_IDS].copy()

    # KMeans
    from sklearn.cluster import KMeans
    kmeans = KMeans(n_clusters=N_CLUSTERS, init=init_centers, n_init=1,
                    max_iter=300, random_state=42)
    kmeans.fit(features)

    # 匈牙利算法约束均衡
    centers = kmeans.cluster_centers_
    dist = np.zeros((n, N_CLUSTERS))
    for j in range(N_CLUSTERS):
        dist[:, j] = np.linalg.norm(features - centers[j], axis=1)

    # 扩展代价矩阵: 每类 9 个槽
    cost = np.zeros((n, n))
    for j in range(N_CLUSTERS):
        for s in range(BLOCKS_PER_COLOR):
            cost[:, j * BLOCKS_PER_COLOR + s] = dist[:, j]

    # 强制中心色块分配到对应面：将其到正确类的 cost 设为 0，到其他类设为极大值
    for face_idx, roi_idx in enumerate(CENTER_ORIGINAL_ROI_IDS):
        for j in range(N_CLUSTERS):
            for s in range(BLOCKS_PER_COLOR):
                col = j * BLOCKS_PER_COLOR + s
                if j == face_idx:
                    cost[roi_idx, col] = 0.0
                else:
                    cost[roi_idx, col] = 1e9

    row_ind, col_ind = linear_sum_assignment(cost)
    labels = np.zeros(n, dtype=int)
    for r, c in zip(row_ind, col_ind):
        labels[r] = c // BLOCKS_PER_COLOR

    return labels


def original_roi_labels_to_facelets(labels):
    """
    将 original ROI 顺序的聚类标签转换为 solver facelet 字符串。
    
    由于 KMeans 初始化顺序是 [U, R, F, D, L, B]，
    class i 直接对应 face i，即 FACE_NAMES[i]。
    """
    facelets = []
    for facelet_idx in range(54):
        original_roi_idx = SOLVER_FACELET_ORIGINAL_ROI_INDICES[facelet_idx]
        cls = labels[original_roi_idx]
        facelets.append(FACE_NAMES[cls])

    return "".join(facelets)


def annotate_image(image, rois, labels, features_bgr):
    """在原图 ROI 位置标注识别颜色（用同类中心色块的实际颜色填充）"""
    result = image.copy()

    # 计算每类的平均 BGR 颜色（从实际像素）
    class_colors = {}
    for cls in range(N_CLUSTERS):
        member_bgrs = [features_bgr[i] for i in range(len(labels)) if labels[i] == cls]
        if member_bgrs:
            avg = np.mean(member_bgrs, axis=0).astype(int)
            class_colors[cls] = tuple(int(c) for c in avg)
        else:
            class_colors[cls] = (128, 128, 128)

    for i, roi in enumerate(rois):
        x, y = roi["x"], roi["y"]
        cx = x + roi["width"] // 2
        cy = y + roi["height"] // 2
        half = ROI_DRAW_SIZE // 2
        x1 = cx - half
        y1 = cy - half
        x2 = cx + half
        y2 = cy + half

        color = class_colors[labels[i]]

        # 填充色块
        cv2.rectangle(result, (x1, y1), (x2, y2), color, -1)
        # 边框
        cv2.rectangle(result, (x1, y1), (x2, y2), (0, 0, 0), 1)

    return result


def process_image(image_path, rois, output_dir):
    """处理单张图片"""
    image = cv2.imread(image_path)
    if image is None:
        print(f"  跳过：无法读取 {image_path}")
        return None

    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

    # 提取 54 个 ROI 的 LAB 颜色特征
    features = np.array([extract_roi_color(lab, roi) for roi in rois])

    # 提取 BGR 均值（用于标注）
    features_bgr = []
    for roi in rois:
        x, y, w, h = roi["x"], roi["y"], roi["width"], roi["height"]
        patch = image[max(0,y):min(image.shape[0],y+h), max(0,x):min(image.shape[1],x+w)]
        features_bgr.append(patch.mean(axis=(0,1)))

    # 聚类
    labels = cluster_colors(features)

    # 生成 facelets（使用正确的映射）
    facelets = original_roi_labels_to_facelets(labels)

    # 验证每类是否均衡
    counts = [int(np.sum(labels == i)) for i in range(N_CLUSTERS)]
    balanced = all(c == BLOCKS_PER_COLOR for c in counts)
    status = "✓" if balanced else "✗"

    basename = os.path.splitext(os.path.basename(image_path))[0]
    print(f"  {status} {basename}: {facelets}  counts={counts}")

    # 标注并保存
    annotated = annotate_image(image, rois, labels, features_bgr)
    output_path = os.path.join(output_dir, f"{basename}_recognized.jpg")
    cv2.imwrite(output_path, annotated)

    # 保存结果 JSON
    result_data = {
        "image": os.path.basename(image_path),
        "facelets": facelets,
        "labels": labels.tolist(),
        "counts": counts,
        "balanced": balanced,
    }
    json_path = os.path.join(output_dir, f"{basename}_recognized.json")
    with open(json_path, "w") as f:
        json.dump(result_data, f, indent=2)

    return result_data


def main():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    roi_file = os.path.join(base_dir, "robo-app", "robot-roi.json")
    output_dir = os.path.join(base_dir, "scripts", "output")

    if not os.path.exists(roi_file):
        print(f"错误：未找到 ROI 文件 {roi_file}")
        sys.exit(1)

    rois = load_rois(roi_file)
    print(f"已加载 {len(rois)} 个 ROI from {roi_file}")

    os.makedirs(output_dir, exist_ok=True)

    # 确定输入图片
    if len(sys.argv) > 1:
        image_paths = sys.argv[1:]
    else:
        img_dir = os.path.join(base_dir, "roboapp", "imgs")
        image_paths = sorted(glob.glob(os.path.join(img_dir, "*.png")))
        if not image_paths:
            print(f"错误：{img_dir}/ 下没有找到 PNG 图片")
            sys.exit(1)

    print(f"待处理 {len(image_paths)} 张图片，输出到 {output_dir}/\n")

    results = []
    for path in image_paths:
        r = process_image(path, rois, output_dir)
        if r:
            results.append(r)

    # 汇总
    n_balanced = sum(1 for r in results if r["balanced"])
    print(f"\n完成：{len(results)} 张图，{n_balanced}/{len(results)} 均衡")
    print(f"结果保存在: {output_dir}/")


if __name__ == "__main__":
    main()
