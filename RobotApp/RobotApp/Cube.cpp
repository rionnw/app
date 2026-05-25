#include "Cube.h"

const int CUBE_SIZE = 50; // 3x3x3 cube

std::vector<cv::Rect> pos{
	// Left face
	cv::Rect(0 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(0 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(0 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(1 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(1 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(1 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(2 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(2 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(2 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),


	// Front face
	cv::Rect(3 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(3 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(3 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(4 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(4 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(4 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(5 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(5 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(5 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	// Up face
	cv::Rect(3 * CUBE_SIZE, 2 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(3 * CUBE_SIZE, 1 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(3 * CUBE_SIZE, 0 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(4 * CUBE_SIZE, 2 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(4 * CUBE_SIZE, 1 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(4 * CUBE_SIZE, 0 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(5 * CUBE_SIZE, 2 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(5 * CUBE_SIZE, 1 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(5 * CUBE_SIZE, 0 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	// Back face
	cv::Rect(11 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(11 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(11 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(10 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(10 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(10 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(9 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(9 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(9 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	// Right face
	cv::Rect(8 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(8 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(8 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(7 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(7 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(7 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(6 * CUBE_SIZE, 3 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(6 * CUBE_SIZE, 4 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(6 * CUBE_SIZE, 5 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	// Down face
	cv::Rect(5 * CUBE_SIZE, 6 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(4 * CUBE_SIZE, 6 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(3 * CUBE_SIZE, 6 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(5 * CUBE_SIZE, 7 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(4 * CUBE_SIZE, 7 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(3 * CUBE_SIZE, 7 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),

	cv::Rect(5 * CUBE_SIZE, 8 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(4 * CUBE_SIZE, 8 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
	cv::Rect(3 * CUBE_SIZE, 8 * CUBE_SIZE, CUBE_SIZE, CUBE_SIZE)
};


cv::Mat drawCube(bool isSave) {
	// 初始化随机数种子（通常在程序开始时调用一次）
	srand(static_cast<unsigned int>(time(nullptr)));
	cv::Mat img(9 * CUBE_SIZE, 12 * CUBE_SIZE, CV_8UC3, cv::Scalar(200, 200, 200)); // 创建一个白色背景的图像
	for (int i = 0; i < 54; ++i) {
		cv::rectangle(img, pos[i], cv::Scalar(rand() % 256, rand() % 256, rand() % 256), -1);
	}
	cv::copyMakeBorder(img, img, 20, 20, 20, 20, cv::BORDER_CONSTANT, cv::Scalar(200, 200, 200));
	if (isSave) {
		cv::imwrite("cube.png", img); // 保存图像
	}
	return img;
}

HSV RGB::toHSV() const {
	// 归一化处理，将0-255范围的值转换为0-1之间
	double rNorm = R / 255.0;
	double gNorm = G / 255.0;
	double bNorm = B / 255.0;

	// 找出RGB中的最大值和最小值
	double maxVal = std::max({ rNorm, gNorm, bNorm });
	double minVal = std::min({ rNorm, gNorm, bNorm });

	// 计算V值（亮度）
	double v = maxVal;

	// 计算S值（饱和度）
	double s = (maxVal == 0) ? 0 : (maxVal - minVal) / maxVal;

	// 计算H值（色调）
	double h = 0;
	if (maxVal != minVal) {
		double diff = maxVal - minVal;

		if (maxVal == rNorm) {
			h = std::fmod(60 * ((gNorm - bNorm) / diff) + 360, 360);
		}
		else if (maxVal == gNorm) {
			h = std::fmod(60 * ((bNorm - rNorm) / diff) + 120, 360);
		}
		else {
			h = std::fmod(60 * ((rNorm - gNorm) / diff) + 240, 360);
		}
	}

	// 返回HSV值，H范围是 0-360，S和V范围是 0-1
	return HSV{ h, s, v };
}

RGB HSV::toRGB() const {
	return RGB{};
}

bool Cube::toJsonFile() {
	try {
		json cubeJson;

		// 将六个面的数据存入JSON对象
		cubeJson["U"] = U;
		cubeJson["R"] = R;
		cubeJson["F"] = F;
		cubeJson["D"] = D;
		cubeJson["L"] = L;
		cubeJson["B"] = B;

		// 写入文件
		std::ofstream file("cube.json");
		if (!file.is_open()) {
			return false;
		}
		file << cubeJson.dump(2); // 缩进2个空格的格式化输出
		return true;
	}
	catch (const std::exception& e) {
		spdlog::error("Serialization failed: {}", e.what());
		return false;
	}
}

// 从JSON文件反序列化为数据
bool Cube::fromJsonFile(const std::string& filePath) {
	try {
		json cubeJson;

		// 读取文件
		std::ifstream file(filePath);
		if (!file.is_open()) {
			return false;
		}
		file >> cubeJson;

		// 验证每个面的数据类型
		for (const auto& face : faceStr) {
			if (!cubeJson.contains(face)) {
				spdlog::error("JSON缺少面数据: {}", face);
				throw std::runtime_error(std::string("JSON缺少面数据: ") + face);
			}

			if (!cubeJson[face].is_array()) {
				throw std::runtime_error(std::string("面数据") + face + "不是数组类型");
			}
		}

		// 反序列化数据
		U = cubeJson["U"].get<std::vector<int>>();
		R = cubeJson["R"].get<std::vector<int>>();
		F = cubeJson["F"].get<std::vector<int>>();
		D = cubeJson["D"].get<std::vector<int>>();
		L = cubeJson["L"].get<std::vector<int>>();
		B = cubeJson["B"].get<std::vector<int>>();

		allFaces.clear();
		allFaces.insert(allFaces.end(), U.begin(), U.end());
		allFaces.insert(allFaces.end(), R.begin(), R.end());
		allFaces.insert(allFaces.end(), F.begin(), F.end());
		allFaces.insert(allFaces.end(), D.begin(), D.end());
		allFaces.insert(allFaces.end(), L.begin(), L.end());
		allFaces.insert(allFaces.end(), B.begin(), B.end());

		centerIdxs = { U[4], R[4], F[4], D[4], L[4], B[4] };

		edgeIdxs = { {
			{ U[1], B[1] }, { U[3], L[1] }, { U[5], R[1] }, { U[7], F[1] },
			{ L[3], B[5] }, { B[3], R[5] }, { R[3], F[5] }, { F[3], L[5] },
			{ D[1], F[7] }, { D[3], L[7] }, { D[5], R[7] }, { D[7], B[7] }
		} };

		cornerIdxs = { {
			{ U[0], L[0], B[2] }, { U[2], B[0], R[2] }, { U[6], F[0], L[2] }, { U[8], R[0], F[2] },
			{ D[0], F[6], L[8] }, { D[2], R[6], F[8] }, { D[6], L[6], B[8] }, { D[8], B[6], R[8] }
		} };

		return true;
	}
	catch (const std::exception& e) {
		spdlog::error("Deserialization failed: {}", e.what());
		return false;
	}
}

bool Cube::canSolve(const std::array<int, 54>& classes) const {
	// 1 判断魔方是否为标准6色且每色9个块
	// 统计每种颜色的出现次数
	std::array<int, 6> colorCounts{ 0 }; // 初始化为全0
	// 遍历魔方的54个块，统计每种颜色的数量
	for (int i = 0; i < 54; ++i) {
		const int color = classes[i];
		// 防御性检查：确保颜色索引在有效范围内
		if (color < 0 || color > 5) {
			return false;
		}
		++colorCounts[color];
	}

	// 检查每种颜色是否恰好出现9次
	for (int count : colorCounts) {
		if (count != 9) {
			spdlog::info("cube is not 6x9!");
			return false;
		}
	}

	// 2 判断中心块是否错误、重复
	for (int i = 0; i < centerIdxs.size(); ++i) {
		auto idx = centerIdxs[i];
		if (classes[idx] != i) {
			spdlog::info("center idx: {} color: {} is not correct!", idx, classes[idx]);
			return false; // 中心块颜色不正确
		}
	}

	// 3 判断每个边块的两个颜色是否相同
	for (const auto& edge : edgeIdxs) {
		if (classes[edge[0]] % 3 == classes[edge[1]] % 3) {
			spdlog::info("edge idx: {} color: {} is not correct!", edge[0], classes[edge[0]]);
			return false; // 边块颜色不正确
		}
	}

	// 4 判断每个角块的三个颜色是否相同
	for (const auto& corner : cornerIdxs) {
		if (classes[corner[0]] == classes[corner[1]] ||
			classes[corner[0]] == classes[corner[2]] ||
			classes[corner[1]] == classes[corner[2]]) {
			spdlog::info("corner idx: {} color: {} is not correct!", corner[0], classes[corner[0]]);
			return false; // 角块颜色不正确
		}
	}

	// 5 判断边块色向是否正确
	int ColorSum = 0;
	int map[3] = { 1, 0, 2 };
	for (const auto& edge : edgeIdxs) {
		if (map[classes[edge[0]] % 3] < map[classes[edge[1]] % 3]) {
			ColorSum += 1;
		}
	}
	if (ColorSum & 1) {
		// spdlog::info("edge idx: {} color: {} is not correct!", edge[0], classes[edge[0]]);
		return false;
	}

	// 6 判断角块色向是否正确
	ColorSum = 0;
	for (const auto& corner : cornerIdxs) {
		if (classes[corner[0]] % 3 == 2)
			ColorSum += 0;
		else if (classes[corner[1]] % 3 == 2)
			ColorSum += 1;
		else if (classes[corner[2]] % 3 == 2)
			ColorSum += 2;
	}
	if (ColorSum % 3 != 0) {
		return false;
	}

	// 7 判断色片位置 TODO
	return true;
}

void Cube::fillCubeColor(const cv::Mat& img, const std::vector<cv::Rect>& rois) {
	if (rois.size() != 54) {
		spdlog::error("rois size is not 54!");
		return;
	}
	for (int i = 0; i < rois.size(); ++i) {
		// 获取每个区域的平均颜色值
		auto BGR = mean(img(rois[i]));
		RGBs[i] = { BGR[2], BGR[1], BGR[0] };
	}
}

void Cube::fillCubeColor(const std::vector<cv::Scalar>& colors) {
	if (colors.size() != 54) {
		spdlog::error("colors size is not 54!");
		return;
	}
	for (int i = 0; i < colors.size(); ++i) {
		// 获取每个区域的平均颜色值
		auto& BGR = colors[i];
		RGBs[i] = { BGR[2], BGR[1], BGR[0] };
	}
}