#pragma once

#include <array>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>

#include <spdlog/spdlog.h>

#include <opencv2/opencv.hpp>

using json = nlohmann::json;

//                           2-----------2------------1
//                           | U0(20)  U1(23)  U2(26) |
//                           |                        |
//                           3 U3(19)  U4(22)  U5(25) 1
//                           |                        |
//                           | U6(18)  U7(21)  U8(24) |
//  2-----------3------------3-----------0------------0-----------1------------1------------2------------2
//  | L0(2)   L1(5)   L2(8)  | F0(11)  F1(14)  F2(17) | R0(42)  R1(39)  R2(36) |  B0(33)  B1(30)  B2(27) |
//  |                        |                        |                        |                         |
// 11 L3(1)   L4(4)   L5(7)  9 F3(10)  F4(13)  F5(16) 8 R3(43)  R4(40)  R5(37) 10 B3(34)  B4(31)  B5(28) 11
//  |                        |                        |                        |                         |
//  | L6(0)   L7(3)   L8(6)  | F6(9)   F7(12)  F8(15) | R6(44)  R7(41)  R8(38) |  B6(35)  B7(32)  B8(29) |
//  3-----------7------------5-----------4------------4-----------5------------7------------6------------3
//                           | D0(53)  D1(52)  D2(51) |
//                           |                        |
//                           7 D3(50)  D4(49)  D5(48) 5
//                           |                        |
//                           | D6(47)  D7(46)  D8(45) |
//                           6-----------6------------7


//                           2-----------2------------1
//                           | U0(53)  U1(52)  U2(51) |
//                           |                        |
//                           3 U3(50)  U4(49)  U5(48) 1
//                           |                        |
//                           | U6(47)  U7(46)  U8(45) |
//  2-----------3------------3-----------0------------0-----------1------------1------------2------------2
//  | L0(6)   L1(3)   L2(0)  | F0(29)  F1(32)  F2(35) | R0(38)  R1(41)  R2(44) |  B0(15)  B1(12)  B2(9)  |
//  |                        |                        |                        |                         |
// 11 L3(7)   L4(4)   L5(1)  9 F3(28)  F4(31)  F5(34) 8 R3(37)  R4(40)  R5(43) 10 B3(16)  B4(13)  B5(10) 11
//  |                        |                        |                        |                         |
//  | L6(8)   L7(5)   L8(2)  | F6(27)  F7(30)  F8(33) | R6(36)  R7(39)  R8(42) |  B6(17)  B7(14)  B8(11) |
//  3-----------7------------5-----------4------------4-----------5------------7------------6------------3
//                           | D0(20)  D1(23)  D2(26) |
//                           |                        |
//                           7 D3(19)  D4(22)  D5(25) 5
//                           |                        |
//                           | D6(18)  D7(21)  D8(24) |
//                           6-----------6------------7


cv::Mat drawCube(bool isSave = true);

using CubeFace = std::vector<int>;
using CubeEdge = std::array<int, 2>;
using CubeCorner = std::array<int, 3>;
using CubeClasses = std::array<int, 54>;

struct HSV;

struct RGB {
	double R, G, B;

	// RGB to HSV conversion
	HSV toHSV() const;

	// 重载 + 运算符
	RGB operator+(const RGB& other) const {
		return { R + other.R, G + other.G, B + other.B };
	}

	// 重载 - 运算符
	RGB operator-(const RGB& other) const {
		return { R - other.R, G - other.G, B - other.B };
	}

	// 重载 * 运算符
	RGB operator*(double scalar) const {
		return { R * scalar, G * scalar, B * scalar };
	}

	// 重载 / 运算符
	RGB operator/(double scalar) const {
		if (scalar == 0) throw std::runtime_error("Division by zero in RGB operator/");
		return { R / scalar, G / scalar, B / scalar };
	}

	// 欧几里得距离计算
	double distance(const RGB& other) const {
		return std::sqrt(std::pow(R - other.R, 2) + std::pow(G - other.G, 2) + std::pow(B - other.B, 2));
	}
};

struct HSV {
	double H, S, V;
	RGB toRGB() const;

	double distance(const HSV& other) const {
		return std::sqrt(std::pow(H - other.H, 2) + std::pow(S - other.S, 2) + std::pow(V - other.V, 2));
	}
};


struct Cube
{
	// U, R, F, D, L, B
	const std::vector<std::string> faceStr{ "U", "R", "F", "D", "L", "B" };
	std::vector<int> U, R, F, D, L, B; // 6 faces of the cube
	std::vector<int> allFaces;
	std::array<int, 6> centerIdxs; // 6 centers of the cube
	std::array<CubeEdge, 12> edgeIdxs; // 12 edges of the cube
	std::array<CubeCorner, 8> cornerIdxs; // 8 corners of the cube
	std::array<RGB, 54> RGBs; // 颜色数据

	bool toJsonFile(); // 将数据序列化为JSON文件

	// 从JSON文件反序列化为数据
	bool fromJsonFile(const std::string& filePath = "cube.json");

	bool canSolve(const std::array<int, 54>& classes) const; // 判断魔方是否可解

	void fillCubeColor(const cv::Mat& img, const std::vector<cv::Rect>& rois);

	void fillCubeColor(const std::vector<cv::Scalar>& colors);
};

