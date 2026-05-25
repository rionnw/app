#include "ColorCluster.h"

#include <numeric>

#include <spdlog/spdlog.h>
#include <unordered_set>

ColorCluster::ColorCluster()
{
	cube.fromJsonFile();
}

ClusterRes ColorCluster::cluster(const cv::Mat& img, const std::vector<cv::Rect>& rois)
{
	auto res = ClusterRes();
	cube.fillCubeColor(img, rois);

	res.isCubeSolved = false;
	res.cubeFace.clear();
	res.msg.clear();

	auto classes = clusterWithKnn(cube.RGBs);

	if (cube.canSolve(classes)) {
		std::stringstream ss;

		for (auto idx : cube.U) {
			std::cout << classes[idx] << " ";
		}
		std::cout << std::endl;

		for (auto idx : cube.allFaces) {
			ss << cube.faceStr[classes[idx]];
		}
		std::cout << std::endl;
		res.cubeFace = ss.str();
		res.isCubeSolved = true;
		res.msg = "cube can be solved!";
		spdlog::warn("cube can be solved! cubeFace: {}", res.cubeFace);
		return res;
	}
	else {
		res.msg = "cube can not be solved!";
		spdlog::warn("cube can not be solved!");
	}

	return res;
}

std::array<int, 54> ColorCluster::clusterWithKnn(const std::array<RGB, 54>& RGBs) {
	// U, R, F, D, L, B
	std::unordered_set<int> centerIdxsSet(cube.centerIdxs.begin(), cube.centerIdxs.end());

	// 初始化所有色块类别为-1
	std::array<int, 54> classes{};
	classes.fill(-1);

	std::array<RGB, 54> projs{};

	for (int i = 0; i < RGBs.size(); ++i) {
		// 应用旋转矩阵 R = Ry * Rx
		const auto& RGB = RGBs[i];
		auto& proj = projs[i];
		proj.R = RR(0, 0) * RGB.B + RR(0, 1) * RGB.G + RR(0, 2) * RGB.R;
		proj.G = RR(1, 0) * RGB.B + RR(1, 1) * RGB.G + RR(1, 2) * RGB.R;
		proj.B = (RR(2, 0) * RGB.B + RR(2, 1) * RGB.G + RR(2, 2) * RGB.R) / 2;
	}

	// 计算每个色块的平均颜色值
	std::array<RGB, 6> centers{};
	for (int i = 0; i < 6; ++i) {
		const int idx = cube.centerIdxs[i];
		classes[idx] = i; // 将中心块的类别设置为其对应的颜色
		centers[i] = projs[idx];
	}

	// 最大迭代次数，防止无限循环
	const int maxIterations = 500;
	int iteration = 0;

	std::array<int, 54> preClasses{};

	int iter = 0;
	while (++iteration < maxIterations) {
		iter++;
		// 保存当前分类结果
		std::copy(classes.begin(), classes.end(), preClasses.begin());

		for (int i = 0; i < classes.size(); ++i) {
			if (centerIdxsSet.find(i) != centerIdxsSet.end()) {
				continue; // 跳过中心块
			}

			double minDistance = DBL_MAX;
			int bestColorClass = -1;

			// 计算与每个聚类中心的欧氏距离平方
			for (int c = 0; c < centers.size(); ++c) {
				const double distance = projs[i].distance(centers[c]);
				if (distance < minDistance) {
					minDistance = distance;
					bestColorClass = c;
				}
			}
			classes[i] = bestColorClass;
		}

		// 检查分类是否稳定
		bool classificationChanged = false;
		for (int i = 0; i < 54; ++i) {
			if (preClasses[i] != classes[i]) {
				classificationChanged = true;
				break; // 一旦发现变化立即退出 for 循环，说明分类不稳定
			}
		}

		// 如果没有变化，分类稳定了，退出 while 循环
		if (!classificationChanged) {
			break;
		}

		// 重新计算聚类中心
		std::array<int, 6> classCounts{};
		std::array<RGB, 6> newCenters{};

		// 累加每个类别的所有色块颜色值

		for (int i = 0; i < 54; ++i) {
			const int colorClass = classes[i];
			if (colorClass != -1) {
				classCounts[colorClass]++;

				// 累加颜色值
				newCenters[colorClass] = newCenters[colorClass] + projs[i];
			}
		}

		// 计算新的聚类中心（平均值）
		for (int i = 0; i < newCenters.size(); ++i) {
			if (classCounts[i] > 0) {
				newCenters[i] = newCenters[i] / classCounts[i];
			}
		}
		centers = newCenters; // 更新聚类中心
	}

	return classes;
}

std::array<int, 54> ColorCluster::clusterWithKnn2(const std::array<RGB, 54>& RGBs) {

	std::array<HSV, 54> HSVs{};
	std::array<int, 54> classes{};

	// 转换为HSV色彩空间
	for (int i = 0; i < 54; i++) {
		HSVs[i] = RGBs[i].toHSV();
	}

	// 使用最大堆维护最小的9个元素及其下标
	auto compare = [&](int aIdx, int bIdx) {
		return HSVs[aIdx].S < HSVs[bIdx].S; // 最大堆比较函数
		};

	std::priority_queue<int, std::vector<int>, decltype(compare)> maxHeap(compare);

	// 遍历数组，维护堆的大小为9
	for (int i = 0; i < HSVs.size(); ++i) {
		if (maxHeap.size() < 9) {
			maxHeap.push(i);
		}
		else if (HSVs[i].S < HSVs[maxHeap.top()].S) {
			maxHeap.pop();
			maxHeap.push(i);
		}
	}

	// 将堆中的下标转移到结果vector
	std::vector<int> whiteIdxs;
	whiteIdxs.reserve(9);

	while (!maxHeap.empty()) {
		whiteIdxs.push_back(maxHeap.top());
		maxHeap.pop();
	}

	// 检测有没有白色中心块
	std::unordered_set<int> centerIdxsSet(cube.centerIdxs.begin(), cube.centerIdxs.end());
	int whiteCenterIdx = -1;
	for (auto idx : whiteIdxs) {
		if (centerIdxsSet.find(idx) != centerIdxsSet.end()) {
			whiteCenterIdx = idx;
			break;
		}
	}

	if (whiteCenterIdx == -1) {
		spdlog::info("no white center!");
		return {};
	}

	// 1 选择魔方的中心块颜色 H 作为初始的聚类中心
	std::array<double, 5> centers{ 0 };
	int pos = 0;
	for (auto idx : centerIdxsSet) {
		if (idx != whiteCenterIdx) {
			centers[pos++] = HSVs[idx].H;
		}
	}

	std::array<int, 54> preClasses{};

	std::unordered_set<int> whiteIdxsSet(whiteIdxs.begin(), whiteIdxs.end());

	// 最大迭代次数，防止无限循环
	const int maxIterations = 300;
	int iteration = 0;
	while (++iteration < maxIterations) {
		std::copy(classes.begin(), classes.end(), preClasses.begin());

		for (int i = 0; i < HSVs.size(); ++i) {
			// 跳过白色中心块
			if (whiteIdxsSet.find(i) != whiteIdxsSet.end()) {
				continue;
			}

			double h = HSVs[i].H;
			double minDistance = DBL_MAX;
			int bestColorClass = -1;

			for (int j = 0; j < centers.size(); ++j) {
				// 要计算这两个角度之间的绝对值差，首先需要处理两个角度间的差值使其落在0到360度之间，
				// 因为简单地做差可能得出的结果会超过这个范围，例如当h0 = 350而h = 10时，它们的差应该是20度而不是340度。
				auto diff = std::fmod(std::fabs(h - centers[j]), 360.0); // 先求差值并取绝对值，然后取模360度
				auto distance = diff > 180.0 ? 360.0 - diff : diff; // 如果差值大于180度，则补足360度

				if (distance < minDistance)
				{
					minDistance = distance;
					bestColorClass = j;
				}
			}
			classes[i] = bestColorClass;
		}

		// 检查分类是否稳定
		bool classificationChanged = false;
		for (int i = 0; i < 54; ++i) {
			if (preClasses[i] != classes[i]) {
				classificationChanged = true;
				break; // 一旦发现变化立即退出 for 循环，说明分类不稳定
			}
		}

		// 如果没有变化，分类稳定了，退出 while 循环
		if (!classificationChanged) {
			break;
		}

		// 重新计算聚类中心
		std::array<int, 5> classCounts{};
		std::array<double, 5> newCenters{};

		for (int i = 0; i < 54; ++i) {
			const int colorClass = classes[i];
			if (colorClass != -1) {
				classCounts[colorClass]++;
				newCenters[colorClass] += HSVs[i].H;
			}
		}

		// 计算新的聚类中心（平均值）
		for (int i = 0; i < newCenters.size(); ++i) {
			if (classCounts[i] > 0) {
				newCenters[i] /= classCounts[i];
			}
		}
		centers = newCenters; // 更新聚类中心
	}

	return classes;
}
