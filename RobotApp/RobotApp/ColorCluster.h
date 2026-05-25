#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "Cube.h"

struct ClusterRes
{
	bool isCubeSolved;
	std::string cubeFace;
	std::string msg;
	ClusterRes() :isCubeSolved(false), cubeFace(""), msg("") {};
};

class ColorCluster
{
public:
	ColorCluster();
	~ColorCluster() = default;

	ClusterRes cluster(const cv::Mat& img, const std::vector<cv::Rect>& rois);

private:
	const double B1 = 35;
	const double B2 = 132;

	const double pi = 3.141592654;
	const double cB1 = cos(B1 * pi / 180.0);
	const double cB2 = cos(B2 * pi / 180.0);
	const double sB1 = sin(B1 * pi / 180.0);
	const double sB2 = sin(B2 * pi / 180.0);

	// 初始化旋转矩阵
	const cv::Matx33d Rx = cv::Matx33d(1.0, 0.0, 0.0, 0.0, cB1, -sB1, 0.0, sB1, cB1);
	const cv::Matx33d Ry = cv::Matx33d(cB2, 0.0, sB2, 0.0, 1.0, 0.0, -sB2, 0.0, cB2);
	const cv::Matx33d RR = Ry * Rx;

	Cube cube;

	std::array<int, 54> clusterWithKnn(const std::array<RGB, 54>& RGBs);

	std::array<int, 54> clusterWithKnn2(const std::array<RGB, 54>& RGBs);
};

